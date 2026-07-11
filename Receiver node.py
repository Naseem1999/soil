#!/usr/bin/env python3
"""
================================================================
  AI SOIL MONITORING — RECEIVER NODE (Raspberry Pi 4)
  Zero pip installs — uses only:
    • spidev      (pre-installed on Raspberry Pi OS)
    • RPi.GPIO    (pre-installed on Raspberry Pi OS)
    • http.server (Python 3 built-in)
    • json        (Python 3 built-in)
    • threading   (Python 3 built-in)

  Run:  python3 receiver_node.py
  Open: http://<raspberry-pi-ip>:5000
        or http://raspberrypi.local:5000

  Enable SPI once (then reboot):
    sudo raspi-config → Interface Options → SPI → Enable
================================================================
  LoRa RA-02 → RPi4 wiring
  VCC   → Pin 1  (3.3V)   ← NEVER 5V!
  GND   → Pin 6
  MOSI  → Pin 19 (GPIO10 SPI0)
  MISO  → Pin 21 (GPIO9  SPI0)
  SCK   → Pin 23 (GPIO11 SPI0)
  NSS   → Pin 24 (GPIO8  CE0)
  RST   → Pin 15 (GPIO22)
  DIO0  → Pin 13 (GPIO27)
================================================================
"""

import spidev
import RPi.GPIO as GPIO
import time
import threading
import json
import socket
from datetime import datetime
from http.server import BaseHTTPRequestHandler, HTTPServer

# ── Config ───────────────────────────────────────────────────
LORA_RST_PIN  = 22
LORA_DIO0_PIN = 27
LOG_FILE      = "soil_log.csv"
WEB_PORT      = 5000

# ── Shared state ─────────────────────────────────────────────
node_store = {
    i: {
        'data': None,
        'pkt':  0,
        'rssi': None,
        'snr':  None,
        'ts':   None,
        'rec':  '',
    }
    for i in range(1, 5)
}
store_lock = threading.Lock()


# ─── LoRa RA-02 Driver ───────────────────────────────────────
class LoRaReceiver:
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(LORA_RST_PIN,  GPIO.OUT)
        GPIO.setup(LORA_DIO0_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)
        self.spi.max_speed_hz = 5_000_000
        self.spi.mode = 0b00

        self._reset()
        self._init()

    def _reset(self):
        GPIO.output(LORA_RST_PIN, GPIO.LOW);  time.sleep(0.02)
        GPIO.output(LORA_RST_PIN, GPIO.HIGH); time.sleep(0.05)

    def _wr(self, reg, val):
        self.spi.xfer2([reg | 0x80, val])

    def _rd(self, reg):
        return self.spi.xfer2([reg & 0x7F, 0x00])[1]

    def _rd_burst(self, reg, n):
        return self.spi.xfer2([reg & 0x7F] + [0x00] * n)[1:]

    def _init(self):
        ver = self._rd(0x42)
        print("[LoRa] Chip version: 0x{:02X}".format(ver))
        if ver != 0x12:
            raise RuntimeError("LoRa not found (got 0x{:02X}) — check wiring!".format(ver))

        self._wr(0x01, 0x00); time.sleep(0.01)   # sleep mode
        self._wr(0x01, 0x80); time.sleep(0.01)   # LoRa mode

        frf = int((433e6 / 32e6) * (1 << 19))
        self._wr(0x06, (frf >> 16) & 0xFF)
        self._wr(0x07, (frf >>  8) & 0xFF)
        self._wr(0x08,  frf        & 0xFF)

        self._wr(0x09, 0x8F)   # PA_BOOST max power
        self._wr(0x0C, 0x23)   # LNA max gain
        self._wr(0x1D, 0x72)   # BW=125kHz, CR=4/5, explicit header
        self._wr(0x1E, 0xA4)   # SF=10, CRC on
        self._wr(0x26, 0x04)   # AGC auto
        self._wr(0x20, 0x00)
        self._wr(0x21, 0x08)   # preamble length = 8
        self._wr(0x39, 0xF3)   # sync word — must match all transmitters
        self._wr(0x0E, 0x00)   # FIFO TX base
        self._wr(0x0F, 0x00)   # FIFO RX base
        self._wr(0x40, 0x00)   # DIO0 = RxDone
        self._wr(0x01, 0x81)   # standby
        print("[LoRa] Initialized OK")

    def start_rx(self):
        self._wr(0x01, 0x85)   # continuous RX mode

    def packet_ready(self):
        return GPIO.input(LORA_DIO0_PIN) == GPIO.HIGH

    def read_packet(self):
        irq = self._rd(0x12)
        self._wr(0x12, 0xFF)                 # clear all IRQ flags
        if not (irq & 0x40):
            return None, None, None          # RxDone not set
        if irq & 0x20:
            print("[LoRa] CRC error — packet dropped")
            return None, None, None
        nb   = self._rd(0x13)                # number of received bytes
        cur  = self._rd(0x10)                # FIFO RX current address
        self._wr(0x0D, cur)
        raw  = self._rd_burst(0x00, nb)
        rssi = self._rd(0x1A) - 157
        snrb = self._rd(0x19)
        snr  = (snrb if snrb < 128 else snrb - 256) / 4.0
        try:
            payload = bytes(raw).decode('utf-8').strip()
        except Exception:
            payload = None
        return payload, rssi, snr

    def close(self):
        self.spi.close()
        GPIO.cleanup()


# ─── Packet Parser ───────────────────────────────────────────
def parse_packet(payload):
    """
    Parses: NODE:1,PKT:5,M:32.1,T:25.3,EC:450,PH:6.8,N:42,P:18,K:130
    Returns (node_id, pkt_num, data_dict) or (None, None, None).
    Fields of -1 (sensor fail on transmitter side) become None.
    """
    try:
        fields = {}
        for part in payload.split(','):
            if ':' not in part:
                continue
            k, v = part.split(':', 1)
            fields[k.strip().upper()] = v.strip()

        node_id = int(fields['NODE'])
        pkt_num = int(fields['PKT'])

        def f(key):
            v = float(fields.get(key, '-1'))
            return None if v < 0 else v

        data = {
            'moisture':     f('M'),
            'temperature':  f('T'),
            'conductivity': f('EC'),
            'ph':           f('PH'),
            'nitrogen':     f('N'),
            'phosphorus':   f('P'),
            'potassium':    f('K'),
        }
        return node_id, pkt_num, data

    except Exception as e:
        print("[Parse] Error: {} | raw: {}".format(e, payload))
        return None, None, None


# ─── Recommendation Engine ───────────────────────────────────
def recommend(data):
    if not data:
        return "No data received"
    tips = []

    ph = data.get('ph')
    if ph is not None:
        if ph < 5.5:   tips.append("Add lime — pH too low ({})".format(ph))
        elif ph > 7.5: tips.append("Acidify soil — pH too high ({})".format(ph))

    n = data.get('nitrogen')
    if n is not None:
        if n < 20:    tips.append("Apply nitrogen fertilizer (N={})".format(n))
        elif n > 120: tips.append("Reduce nitrogen input (N={})".format(n))

    p = data.get('phosphorus')
    if p is not None:
        if p < 10:   tips.append("Apply phosphorus fertilizer (P={})".format(p))
        elif p > 80: tips.append("Reduce phosphorus input (P={})".format(p))

    k = data.get('potassium')
    if k is not None:
        if k < 80:    tips.append("Apply potassium fertilizer (K={})".format(k))
        elif k > 300: tips.append("Reduce potassium input (K={})".format(k))

    m = data.get('moisture')
    if m is not None:
        if m < 20:   tips.append("Irrigate now — soil too dry ({}%)".format(m))
        elif m > 85: tips.append("Improve drainage — waterlogged ({}%)".format(m))

    ec = data.get('conductivity')
    if ec is not None:
        if ec > 4000: tips.append("Flush soil — EC too high ({})".format(ec))

    return " | ".join(tips) if tips else "Soil OK — no action needed"


# ─── CSV Logger ──────────────────────────────────────────────
def init_log():
    try:
        with open(LOG_FILE, 'x') as f:
            f.write("timestamp,node,pkt,moisture,temperature,conductivity,"
                    "ph,nitrogen,phosphorus,potassium,rssi,snr,recommendation\n")
    except FileExistsError:
        pass

def log_row(node_id, pkt, data, rssi, snr, rec):
    ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    def v(key, dec=1):
        val = data.get(key) if data else None
        return "{:.{}f}".format(val, dec) if val is not None else ""
    row = "{},{},{},{},{},{},{},{},{},{},{},{},{}\n".format(
        ts, node_id, pkt,
        v('moisture'), v('temperature'), v('conductivity', 0),
        v('ph'), v('nitrogen', 0), v('phosphorus', 0), v('potassium', 0),
        rssi, round(snr, 1), rec,
    )
    with open(LOG_FILE, 'a') as f:
        f.write(row)


# ─── LoRa background thread ──────────────────────────────────
def lora_loop():
    try:
        lora = LoRaReceiver()
    except Exception as e:
        print("[LoRa] FATAL:", e)
        return

    lora.start_rx()
    print("[LoRa] Listening for packets from nodes 1–4...")

    while True:
        if lora.packet_ready():
            payload, rssi, snr = lora.read_packet()
            lora.start_rx()   # re-arm immediately

            if not payload:
                continue

            print("[RX] {} | RSSI:{}dBm SNR:{:.1f}dB".format(payload, rssi, snr))
            node_id, pkt, data = parse_packet(payload)

            if node_id and 1 <= node_id <= 4:
                rec = recommend(data)
                ts  = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

                with store_lock:
                    node_store[node_id].update({
                        'data': data,
                        'pkt':  pkt,
                        'rssi': rssi,
                        'snr':  round(snr, 1),
                        'ts':   ts,
                        'rec':  rec,
                    })

                log_row(node_id, pkt, data, rssi, snr, rec)
                print("  → Node {} Pkt {} | {}".format(node_id, pkt, rec))

        time.sleep(0.05)


# ─── Dashboard HTML (served inline — no template files needed) ──
DASHBOARD_HTML = b"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Soil Monitor</title>
<style>
*,*::before,*::after{box-sizing:border-box;margin:0;padding:0}
body{font-family:system-ui,sans-serif;background:#0f1117;color:#e2e8f0;min-height:100vh;padding:24px 16px}
header{text-align:center;margin-bottom:28px}
header h1{font-size:1.5rem;font-weight:600;color:#f8fafc}
header p{font-size:.83rem;color:#64748b;margin-top:4px}
#ts{font-size:.75rem;color:#475569;margin-top:6px}
.grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(270px,1fr));gap:16px;max-width:1180px;margin:0 auto}
.card{background:#1e2330;border:1px solid #2d3448;border-radius:14px;padding:18px;transition:border-color .3s}
.card.live{border-color:#22c55e}.card.stale{border-color:#f59e0b}.card.off{border-color:#ef4444;opacity:.7}
.ch{display:flex;justify-content:space-between;align-items:flex-start;margin-bottom:14px}
.ch h2{font-size:.95rem;font-weight:600;color:#f1f5f9}
.meta{text-align:right;font-size:.7rem;color:#64748b;line-height:1.6}
.badge{display:inline-block;font-size:.65rem;font-weight:700;padding:2px 7px;border-radius:99px}
.blv{background:#14532d;color:#86efac}.bst{background:#78350f;color:#fde68a}.bof{background:#450a0a;color:#fca5a5}
.stats{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-bottom:12px}
.stat{background:#111827;border-radius:8px;padding:9px 11px}
.sl{font-size:.65rem;text-transform:uppercase;letter-spacing:.05em;color:#6b7280;margin-bottom:2px}
.sv{font-size:1.18rem;font-weight:600;color:#f9fafb;line-height:1.1}
.su{font-size:.68rem;color:#9ca3af;margin-left:2px}
.sv.ok{color:#34d399}.sv.warn{color:#fbbf24}.sv.err{color:#f87171}
.bw{background:#111827;border-radius:3px;height:4px;margin-top:5px;overflow:hidden}
.b{height:100%;border-radius:3px;transition:width .5s}
.bm{background:#38bdf8}.bp{background:#a78bfa}
.rec{background:#0f2818;border:1px solid #14532d;border-radius:8px;padding:9px 11px;font-size:.78rem;color:#86efac;line-height:1.4}
.rec.warn{background:#1c1204;border-color:#78350f;color:#fde68a}
.rec.off{background:#1a0a0a;border-color:#450a0a;color:#fca5a5}
.sig{font-size:.7rem;color:#475569;margin-top:8px;text-align:right}
.wait{color:#6b7280;font-size:.82rem;text-align:center;padding:20px 0}
</style>
</head>
<body>
<header>
  <h1>&#127807; AI Soil Nutrition Monitor</h1>
  <p>4 ESP32 transmitter nodes &#8594; Raspberry Pi 4 receiver &#8594; LoRa 433 MHz</p>
  <div id="ts">Connecting...</div>
</header>
<div class="grid" id="grid"></div>
<script>
const P=[
  {k:'moisture',    l:'Moisture',    u:'%',     lo:20, hi:85,  bar:'m', bmax:100},
  {k:'temperature', l:'Temperature', u:'\\u00b0C', lo:5,  hi:40,  bar:null},
  {k:'conductivity',l:'EC',          u:'\\u00b5S/cm',lo:0,hi:4000,bar:null},
  {k:'ph',          l:'pH',          u:'',      lo:5.5,hi:7.5, bar:'p', bmax:14},
  {k:'nitrogen',    l:'Nitrogen',    u:'mg/kg', lo:20, hi:120, bar:null},
  {k:'phosphorus',  l:'Phosphorus',  u:'mg/kg', lo:10, hi:80,  bar:null},
  {k:'potassium',   l:'Potassium',   u:'mg/kg', lo:80, hi:300, bar:null},
];

function cls(v,lo,hi){return v===null?'err':(v<lo||v>hi)?'warn':'ok'}
function fmt(v,d){return v===null||v===undefined?'&#8212;':Number(v).toFixed(d??1)}

function card(id,s){
  const d=s.data, off=!s.ts;
  const stale=s.ts&&(Date.now()-new Date(s.ts).getTime())>120000;
  const cc=off?'card off':stale?'card stale':'card live';
  const badge=off?'<span class="badge bof">Offline</span>':stale?'<span class="badge bst">Stale</span>':'<span class="badge blv">Live</span>';
  if(off||!d) return `<div class="${cc}"><div class="ch"><h2>Node ${id}</h2><div class="meta">${badge}</div></div><div class="wait">Waiting for first packet&hellip;</div><div class="rec off">No data yet</div></div>`;
  const stats=P.map(p=>{
    const v=d[p.k], c=cls(v,p.lo,p.hi);
    const dc=['ph','moisture','temperature'].includes(p.k)?1:0;
    const bar=p.bar&&v!==null?`<div class="bw"><div class="b b${p.bar}" style="width:${Math.min(100,v/p.bmax*100).toFixed(1)}%"></div></div>`:'';
    return `<div class="stat"><div class="sl">${p.l}</div><div class="sv ${c}">${fmt(v,dc)}<span class="su">${p.u}</span></div>${bar}</div>`;
  }).join('');
  const ok=s.rec==='Soil OK \u2014 no action needed';
  return `<div class="${cc}">
    <div class="ch"><h2>Node ${id}</h2><div class="meta">Pkt&nbsp;#${s.pkt}<br>${s.ts?s.ts.split(' ')[1]:'&mdash;'}<br>${badge}</div></div>
    <div class="stats">${stats}</div>
    <div class="rec${ok?'':' warn'}">&#128161; ${s.rec||'&mdash;'}</div>
    <div class="sig">RSSI:&nbsp;${s.rssi}&nbsp;dBm &nbsp;|&nbsp; SNR:&nbsp;${s.snr}&nbsp;dB</div>
  </div>`;
}

async function refresh(){
  try{
    const r=await fetch('/api/data');
    const j=await r.json();
    document.getElementById('grid').innerHTML=Object.entries(j.nodes).map(([id,s])=>card(id,s)).join('');
    document.getElementById('ts').textContent='Last update: '+new Date().toLocaleTimeString();
  }catch(e){document.getElementById('ts').textContent='Connection error — retrying...';}
}
refresh();
setInterval(refresh,5000);
</script>
</body>
</html>"""


# ─── HTTP Server (built-in, no Flask) ────────────────────────
class Handler(BaseHTTPRequestHandler):

    def log_message(self, format, *args):
        pass   # silence default request logging (keeps terminal clean)

    def _send(self, code, ctype, body):
        self.send_response(code)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path == '/' or self.path == '/index.html':
            self._send(200, 'text/html; charset=utf-8', DASHBOARD_HTML)

        elif self.path == '/api/data':
            with store_lock:
                snapshot = {str(i): dict(node_store[i]) for i in range(1, 5)}
            body = json.dumps({
                'nodes': snapshot,
                'time':  datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            }).encode()
            self._send(200, 'application/json', body)

        elif self.path == '/api/log':
            try:
                with open(LOG_FILE) as f:
                    lines = f.readlines()
                headers = lines[0].strip().split(',')
                rows = [dict(zip(headers, l.strip().split(','))) for l in lines[-101:][1:]]
                body = json.dumps(rows).encode()
            except Exception:
                body = b'[]'
            self._send(200, 'application/json', body)

        else:
            self._send(404, 'text/plain', b'Not found')


# ─── Entry point ─────────────────────────────────────────────
if __name__ == '__main__':
    # Get local IP for friendly startup message
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('8.8.8.8', 80))
        local_ip = s.getsockname()[0]
        s.close()
    except Exception:
        local_ip = 'raspberrypi.local'

    print("=" * 50)
    print("  AI Soil Monitor — Receiver (RPi4)")
    print("  Dashboard → http://{}:{}".format(local_ip, WEB_PORT))
    print("  Also try  → http://raspberrypi.local:{}".format(WEB_PORT))
    print("=" * 50)

    init_log()

    # LoRa in background thread
    t = threading.Thread(target=lora_loop, daemon=True)
    t.start()

    # Built-in HTTP server — no Flask needed
    server = HTTPServer(('0.0.0.0', WEB_PORT), Handler)
    print("[Web] Server running on port {}".format(WEB_PORT))
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[System] Stopped")
        server.shutdown()
        GPIO.cleanup()