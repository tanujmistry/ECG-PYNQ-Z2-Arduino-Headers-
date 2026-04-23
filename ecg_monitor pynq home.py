#!/usr/bin/env python3
import os, mmap, time, csv, datetime
os.environ['XILINX_XRT'] = '/usr'
from pynq import Overlay

OVERLAY   = '/home/xilinx/ads1299_ecg.bit'
SPI_BASE  = 0x41E00000
GPIO_BASE = 0x41200000

CMD_SDATAC = 0x11; CMD_RDATAC = 0x10
CMD_START  = 0x08; CMD_STOP   = 0x0A
CMD_WREG   = 0x40; CMD_RREG   = 0x20

class MEM:
    def __init__(self, base):
        self.fd  = os.open('/dev/mem', os.O_RDWR | os.O_SYNC)
        self.mem = mmap.mmap(self.fd, 4096, mmap.MAP_SHARED,
                             mmap.PROT_READ | mmap.PROT_WRITE,
                             offset=base)
    def wr(self, off, val):
        self.mem.seek(off)
        self.mem.write(val.to_bytes(4, 'little'))
    def rd(self, off):
        self.mem.seek(off)
        return int.from_bytes(self.mem.read(4), 'little')

class ADS1299:
    def __init__(self):
        print('Loading overlay...')
        self.overlay = Overlay(OVERLAY)
        print('Overlay loaded.')
        self.spi  = MEM(SPI_BASE)
        self.gpio = MEM(GPIO_BASE)
        self._init_spi()

    def _init_spi(self):
        self.spi.wr(0x40, 0x0000000A)
        time.sleep(0.01)
        self.spi.wr(0x60, 0x00000086)
        self.spi.wr(0x70, 0xFFFFFFFF)
        time.sleep(0.01)
        print(f'SPI CR=0x{self.spi.rd(0x60):08X} SR=0x{self.spi.rd(0x64):08X}')

    def _transfer(self, data):
        self.spi.wr(0x70, 0xFFFFFFFE)
        time.sleep(0.000002)
        rx = []
        for b in data:
            self.spi.wr(0x68, b & 0xFF)
            for _ in range(100000):
                if self.spi.rd(0x64) & 0x04:
                    break
                time.sleep(0.000001)
            rx.append(self.spi.rd(0x6C) & 0xFF)
        time.sleep(0.000002)
        self.spi.wr(0x70, 0xFFFFFFFF)
        return rx

    def wreg(self, addr, val):
        self._transfer([CMD_WREG | (addr & 0x1F), 0x00, val])
        time.sleep(0.00001)

    def rreg(self, addr):
        return self._transfer([CMD_RREG | (addr & 0x1F), 0x00, 0x00])[2]

    def cmd(self, c):
        self._transfer([c])
        time.sleep(0.001)

    def reset(self):
        print('Resetting ADS1299...')
        self.gpio.wr(0x00, 0)
        time.sleep(0.001)
        self.gpio.wr(0x00, 1)
        time.sleep(0.1)
        print('Reset done.')

    def init_ecg(self):
        self.reset()
        self.cmd(CMD_SDATAC)
        dev = self.rreg(0x00)
        print(f'Device ID: 0x{dev:02X}  (expect 0x3E)')
        if dev != 0x3E:
            print('WARNING: Check SPI wiring!')
        else:
            print('SPI communication OK!')
        self.wreg(0x01, 0xD6)
        self.wreg(0x02, 0xC0)
        self.wreg(0x03, 0xEC)
        time.sleep(0.15)
        self.wreg(0x05, 0x60)
        self.wreg(0x06, 0x60)
        for r in range(0x07, 0x0D):
            self.wreg(r, 0x81)
        self.wreg(0x0D, 0x03)
        self.wreg(0x0E, 0x03)
        print('ADS1299 configured.')

    def start(self):
        self.cmd(CMD_START)
        self.cmd(CMD_RDATAC)

    def stop(self):
        self.cmd(CMD_SDATAC)
        self.cmd(CMD_STOP)

    def read_frame(self):
        timeout = 0
        while (self.gpio.rd(0x08) & 0x01):
            timeout += 1
            if timeout > 200000:
                return None, 0, 0
        raw = self._transfer([0x00]*27)
        def s24(b):
            v = (b[0]<<16)|(b[1]<<8)|b[2]
            return v - 0x1000000 if v & 0x800000 else v
        return raw[:3], s24(raw[3:6]), s24(raw[6:9])

    def to_uv(self, raw, gain=24, vref=4.5):
        return raw * (vref/gain) / (2**23) * 1e6

def main():
    print('='*55)
    print(' ADS1299 ECG Monitor  |  PYNQ-Z2  |  PMOD A')
    print(' Press Ctrl+C to stop and save CSV')
    print('='*55)
    ads = ADS1299()
    ads.init_ecg()
    ads.start()
    ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    fname = f'/home/xilinx/ecg_{ts}.csv'
    f = open(fname, 'w', newline='')
    w = csv.writer(f)
    w.writerow(['n', 't_ms', 'ch1_uv', 'ch2_uv'])
    n = 0
    try:
        while True:
            st, ch1, ch2 = ads.read_frame()
            if st is None:
                print('DRDY timeout - check wiring')
                continue
            uv1 = ads.to_uv(ch1)
            uv2 = ads.to_uv(ch2)
            w.writerow([n, n*4, f'{uv1:.1f}', f'{uv2:.1f}'])
            if n % 25 == 0:
                bar = '#' * min(50, int(abs(uv1)/200))
                print(f'n={n:6d}  CH1={uv1:+8.0f}uV  CH2={uv2:+8.0f}uV  |{bar}')
            n += 1
    except KeyboardInterrupt:
        ads.stop()
        f.close()
        print(f'\nSaved {n} samples to {fname}')

if __name__ == '__main__':
    main()
