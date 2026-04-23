#!/usr/bin/env python3
"""
ADS1299 ECG Monitor - Arduino Pin Version
Single Channel (CH1) + BIAS | Real-Time Web Waveform at :8080
PYNQ-Z2 | ADS1299 via Arduino SPI Header
"""

import os, mmap, time, csv, datetime, threading, json
from http.server import HTTPServer, BaseHTTPRequestHandler
from collections import deque

os.environ['XILINX_XRT'] = '/usr'
from pynq import Overlay

# ─── Config ────────────────────────────────────────────────────────────────────
OVERLAY  = '/home/xilinx/ads1299_ecg.bit'
SPI_BASE = 0x41E00000
GPIO_BASE= 0x41200000
WEB_PORT = 8080
VREF     = 4.5
GAIN     = 24
SCALE    = (VREF / GAIN) / 8388608 * 1e6   # raw → µV

CMD_SDATAC = 0x11; CMD_RDATAC = 0x10
CMD_START  = 0x08; CMD_STOP   = 0x0A
CMD_WREG   = 0x40; CMD_RREG   = 0x20

# Shared circular buffer — last 5 seconds @ 250 SPS = 1250 points
data_buffer = deque(maxlen=1250)
buffer_lock = threading.Lock()
sample_count = 0

# ─── Low-level Memory ──────────────────────────────────────────────────────────
class MEM:
    def __init__(self, base):
        self.fd  = os.open('/dev/mem', os.O_RDWR | os.O_SYNC)
        self.mem = mmap.mmap(self.fd, 4096, mmap.MAP_SHARED,
                             mmap.PROT_READ | mmap.PROT_WRITE, offset=base)
    def wr(self, off, val):
        self.mem.seek(off); self.mem.write(val.to_bytes(4, 'little'))
    def rd(self, off):
        self.mem.seek(off); return int.from_bytes(self.mem.read(4), 'little')
    def close(self):
        self.mem.close(); os.close(self.fd)

# ─── ADS1299 Driver ────────────────────────────────────────────────────────────
class ADS1299:
    def __init__(self):
        print('Loading overlay...')
        self.overlay = Overlay(OVERLAY)
        print('Overlay loaded.')
        self.spi  = MEM(SPI_BASE)
        self.gpio = MEM(GPIO_BASE)
        self._init_spi()

    def _init_spi(self):
        self.spi.wr(0x40, 0x0000000A)   # SRR reset
        time.sleep(0.01)
        self.spi.wr(0x60, 0x00000086)   # CR: SPE|Master|ManualSS
        self.spi.wr(0x70, 0xFFFFFFFF)   # SSR: deassert all
        time.sleep(0.01)
        print(f'SPI CR=0x{self.spi.rd(0x60):08X} SR=0x{self.spi.rd(0x64):08X}')

    def _transfer(self, data):
        self.spi.wr(0x70, 0xFFFFFFFE)   # Assert CS
        time.sleep(2e-6)
        rx = []
        for b in data:
            self.spi.wr(0x68, b & 0xFF)
            for _ in range(100000):
                if self.spi.rd(0x64) & 0x04: break
                time.sleep(1e-6)
            rx.append(self.spi.rd(0x6C) & 0xFF)
        time.sleep(2e-6)
        self.spi.wr(0x70, 0xFFFFFFFF)   # Deassert CS
        return rx

    def wreg(self, addr, val):
        self._transfer([CMD_WREG | (addr & 0x1F), 0x00, val])
        time.sleep(1e-5)

    def rreg(self, addr):
        r = self._transfer([CMD_RREG | (addr & 0x1F), 0x00, 0x00])
        return r[2]

    def cmd(self, c):
        self._transfer([c]); time.sleep(1e-4)

    def reset(self):
        print('Resetting ADS1299...')
        self.gpio.wr(0x00, 0x00000000)   # Assert RESET low
        time.sleep(0.001)
        self.gpio.wr(0x00, 0x00000001)   # Release RESET high
        time.sleep(0.01)
        print('Reset done.')

    def configure(self):
        self.cmd(CMD_SDATAC)
        dev_id = self.rreg(0x00)
        print(f'Device ID: 0x{dev_id:02X} (expect 0x3E)')
        if dev_id != 0x3E:
            print('WARNING: Unexpected Device ID — check wiring!')
        else:
            print('SPI communication OK!')

        self.wreg(0x01, 0xD6)   # CONFIG1: 250 SPS
        self.wreg(0x02, 0xC0)   # CONFIG2: internal reference, test signal off
        self.wreg(0x03, 0xEC)   # CONFIG3: BIAS buffer enabled, BIAS_REFINT enabled
        self.wreg(0x05, 0x60)   # CH1: ON, Gain=24, normal electrode input
        for r in range(0x06, 0x0D):
            self.wreg(r, 0x81)  # CH2 to CH8: OFF (power down)
        self.wreg(0x0D, 0x01)   # BIAS_SENSP: driven from CH1 positive only
        self.wreg(0x0E, 0x01)   # BIAS_SENSN: driven from CH1 negative only
        print('ADS1299 configured (CH1 + BIAS only).')

    def drdy_ready(self):
        # GPIO CH2 reads DRDY pin (active low = data ready)
        return (self.gpio.rd(0x08) & 0x01) == 0

    def read_sample(self):
        """Returns ch1_uv as a single float, or None if not ready."""
        if not self.drdy_ready():
            return None
        raw = self._transfer([0x00] * 27)   # 3 status bytes + 8 channels * 3 bytes
        def to_signed24(b):
            v = (b[0] << 16) | (b[1] << 8) | b[2]
            return v - 0x1000000 if v >= 0x800000 else v
        ch1 = to_signed24(raw[3:6]) * SCALE   # CH1 only
        return ch1

    def start(self):
        self.cmd(CMD_START); self.cmd(CMD_RDATAC)

    def stop(self):
        self.cmd(CMD_SDATAC); self.cmd(CMD_STOP)

    def close(self):
        self.spi.close(); self.gpio.close()

# ─── Web Server (Real-Time Waveform) ───────────────────────────────────────────
HTML_PAGE = """<!DOCTYPE html>
<html>
<head>
<title>ADS1299 ECG Real-Time Monitor</title>
<style>
  body   { background:#111; color:#0f0; font-family:monospace; margin:0; padding:10px; }
  h2     { color:#0f0; text-align:center; margin:5px 0; }
  .info  { text-align:center; color:#888; font-size:12px; margin-bottom:8px; }
  canvas { display:block; margin:8px auto; background:#000;
           border:1px solid #0f0; border-radius:4px; }
  #stats { text-align:center; font-size:13px; margin-top:6px; color:#00ff88; }
</style>
</head>
<body>
<h2>&#x1FAC0; ADS1299 ECG Real-Time Monitor</h2>
<div class="info">PYNQ-Z2 | Arduino SPI | CH1 + BIAS | 250 SPS | Gain=24 | Vref=4.5V</div>
<canvas id="c1" width="900" height="300"></canvas>
<div id="stats">Connecting...</div>
<script>
const W=900, H=300, PAD=50;

function draw(id, data, color, label){
  const cv = document.getElementById(id);
  const ctx = cv.getContext('2d');
  ctx.fillStyle = '#000';
  ctx.fillRect(0, 0, W, H);

  // Grid lines
  ctx.strokeStyle = '#1a1a1a'; ctx.lineWidth = 1;
  for(let y = 0; y < H; y += H/4){
    ctx.beginPath(); ctx.moveTo(PAD, y); ctx.lineTo(W, y); ctx.stroke();
  }
  for(let x = PAD; x < W; x += 80){
    ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, H); ctx.stroke();
  }

  // Zero line
  ctx.strokeStyle = '#2a2a2a'; ctx.lineWidth = 1;
  ctx.beginPath(); ctx.moveTo(PAD, H/2); ctx.lineTo(W, H/2); ctx.stroke();

  if(!data || data.length < 2) return;

  // Auto scale
  let mn = Math.min(...data), mx = Math.max(...data);
  let rng = mx - mn || 1;
  mn -= rng * 0.15; mx += rng * 0.15; rng = mx - mn;

  // Y axis labels
  ctx.fillStyle = '#666'; ctx.font = '11px monospace';
  ctx.fillText(mx.toFixed(1) + ' uV', 2, 14);
  ctx.fillText(((mx+mn)/2).toFixed(1) + ' uV', 2, H/2 + 4);
  ctx.fillText(mn.toFixed(1) + ' uV', 2, H - 4);

  // Channel label
  ctx.fillStyle = color; ctx.font = 'bold 12px monospace';
  ctx.fillText(label, PAD + 6, 18);

  // Waveform
  ctx.strokeStyle = color; ctx.lineWidth = 1.8;
  ctx.beginPath();
  data.forEach((v, i) => {
    const x = PAD + (i / (data.length - 1)) * (W - PAD);
    const y = H - ((v - mn) / rng) * H;
    i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
  });
  ctx.stroke();
}

async function poll(){
  try{
    const r = await fetch('/data');
    const d = await r.json();
    draw('c1', d.ch1, '#00ff88', 'CH1 (ECG)');
    const last = d.ch1.length > 0 ? d.ch1[d.ch1.length - 1].toFixed(2) : '—';
    document.getElementById('stats').innerHTML =
      'CH1: ' + last + ' &micro;V &nbsp;|&nbsp; Total Samples: ' + d.total + ' &nbsp;|&nbsp; 250 SPS';
  } catch(e){
    document.getElementById('stats').textContent = 'Reconnecting...';
  }
  setTimeout(poll, 80);   // ~12 fps
}
poll();
</script>
</body>
</html>"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a): pass   # suppress access logs

    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.end_headers()
            self.wfile.write(HTML_PAGE.encode())

        elif self.path == '/data':
            with buffer_lock:
                pts = list(data_buffer)   # list of floats (CH1 only)
            payload = json.dumps({
                'ch1'  : pts,
                'total': sample_count
            }).encode()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(payload)

        else:
            self.send_response(404)
            self.end_headers()


def run_web():
    srv = HTTPServer(('0.0.0.0', WEB_PORT), Handler)
    srv.serve_forever()

# ─── Main ──────────────────────────────────────────────────────────────────────
def main():
    global sample_count

    ads = ADS1299()
    ads.reset()
    ads.configure()
    ads.start()

    # Start web server in background thread
    t = threading.Thread(target=run_web, daemon=True)
    t.start()
    print(f'\n📡 Real-time waveform → open http://192.168.2.99:{WEB_PORT} in your browser\n')
    print('Press Ctrl+C to stop and save CSV...\n')

    fname = f'/home/xilinx/ecg_{datetime.datetime.now():%Y%m%d_%H%M%S}.csv'
    f = open(fname, 'w', newline='')
    writer = csv.writer(f)
    writer.writerow(['sample', 't_ms', 'ch1_uv'])

    t_start = time.time()
    try:
        while True:
            result = ads.read_sample()
            if result is None:
                time.sleep(0.0005)
                continue
            ch1  = result
            t_ms = (time.time() - t_start) * 1000
            writer.writerow([sample_count, f'{t_ms:.2f}', f'{ch1:.4f}'])
            with buffer_lock:
                data_buffer.append(ch1)
            sample_count += 1
            if sample_count % 250 == 0:
                print(f'  t={t_ms/1000:.1f}s  CH1={ch1:8.2f} uV')

    except KeyboardInterrupt:
        print('\nStopping...')
    finally:
        ads.stop()
        ads.close()
        f.close()
        print(f'Saved {sample_count} samples → {fname}')


if __name__ == '__main__':
    main()
