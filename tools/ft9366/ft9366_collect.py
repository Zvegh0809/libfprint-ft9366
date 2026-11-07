#!/usr/bin/env python3
"""
FT9366 (2808:a658) data collection for matcher development.

Saves N raw images per finger (16-bit PGM, 64x80, unprocessed sensor data)
plus empty-sensor background frames captured at the start of the session.

Usage:
    sudo python3 ft9366_collect.py --finger right_index
    sudo python3 ft9366_collect.py --finger right_middle --count 20

Output: ~/ft9366/data/<finger>/<timestamp>/ (bg_*.pgm, img_*.pgm, meta.json)

WARNING: these are real fingerprint images. Development only; never commit
them and delete them when done.
Requires: python-pyusb
"""
import argparse, datetime, json, os, pwd, sys, time

import usb.core
import usb.util

VID, PID = 0x2808, 0xA658
EP_OUT, EP_IN = 0x01, 0x82
W, H = 64, 80
RAW_LEN = W * H * 2
TIMEOUT = 2000
POLL_FINGER = 0.032
POLL_LIFT = 0.1
FINGER_OFF_LEVEL = 0x01C0
BG_FRAMES = 5
BG_TOL = 60

HERE = os.path.dirname(os.path.abspath(__file__))
CMD_READ_ID = bytes.fromhex("04fb9a8b0001")
CMD_READ_IMG = bytes.fromhex("06f99a051400")
CMD_READ_9881 = bytes.fromhex("04fb98810001")


class Sensor:
    def __init__(self, seq):
        self.seq = seq
        self.dev = usb.core.find(idVendor=VID, idProduct=PID)
        if self.dev is None:
            sys.exit("No 2808:a658 device (still passed through to a VM/container?)")
        if self.dev.is_kernel_driver_active(0):
            self.dev.detach_kernel_driver(0)
        self.dev.set_configuration()
        usb.util.claim_interface(self.dev, 0)

    def close(self):
        try:
            self.run("sleep")
        finally:
            usb.util.release_interface(self.dev, 0)

    def xfer(self, out, inlen=0):
        self.dev.write(EP_OUT, out, TIMEOUT)
        if inlen:
            return bytes(self.dev.read(EP_IN, max(inlen, 512), TIMEOUT))
        return b""

    def run(self, name):
        reply = b""
        for out_hex, inlen, _expect, gap in self.seq[name]:
            if gap:
                time.sleep(gap)
            r = self.xfer(bytes.fromhex(out_hex), inlen)
            if inlen:
                reply = r
        return reply

    def read_image(self):
        self.run("capture")
        self.dev.write(EP_OUT, CMD_READ_IMG, TIMEOUT)
        raw = bytes(self.dev.read(EP_IN, RAW_LEN, TIMEOUT))
        self.run("post")
        if len(raw) != RAW_LEN:
            raise RuntimeError(f"short image: {len(raw)} bytes")
        return raw

    def wait_finger(self, timeout=60):
        self.run("arm")
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.xfer(CMD_READ_ID, 2) == b"\x93\x62":
                return True
            time.sleep(POLL_FINGER)
        return False

    def finger_level(self):
        self.run("probe_pre")
        v = int.from_bytes(self.xfer(CMD_READ_9881, 2), "big") | 0x0002
        self.xfer(bytes.fromhex("05fa98810001") + v.to_bytes(2, "big"))
        r = self.run("probe_post")          # last reply: 0x80b8, 4 words
        return min(int.from_bytes(r[i:i + 2], "big") for i in range(0, 8, 2))

    def wait_lift(self, timeout=30):
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.finger_level() > FINGER_OFF_LEVEL:
                return True
            time.sleep(POLL_LIFT)
        return False


def mean16(raw):
    return sum(int.from_bytes(raw[i:i + 2], "big") for i in range(0, len(raw), 2)) / (W * H)


def save_pgm16(path, raw):
    with open(path, "wb") as f:
        f.write(f"P5 {W} {H} 4095\n".encode())
        f.write(raw)                         # PGM16 is big-endian, like the sensor


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--finger", required=True, help="e.g. right_index")
    ap.add_argument("--count", type=int, default=20)
    ap.add_argument("--out", default=None)
    ap.add_argument("--seq", default=os.path.join(HERE, "ft9366_seq2.json"))
    a = ap.parse_args()

    user = os.environ.get("SUDO_USER") or pwd.getpwuid(os.getuid()).pw_name
    pw = pwd.getpwnam(user)
    base = a.out or os.path.join(pw.pw_dir, "ft9366", "data")
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    outdir = os.path.join(base, a.finger, stamp)
    os.makedirs(outdir, exist_ok=True)

    seq = json.load(open(a.seq))
    s = Sensor(seq)
    meta = dict(finger=a.finger, started=stamp, width=W, height=H, bits=12,
                bg=[], images=[])
    try:
        s.run("wake")
        if s.xfer(CMD_READ_ID, 2) != b"\x93\x62":
            sys.exit("Unexpected chip ID")

        print("Capturing background - do NOT touch the sensor...")
        time.sleep(1.0)
        means = []
        for k in range(BG_FRAMES):
            raw = s.read_image()
            m = mean16(raw)
            means.append(m)
            save_pgm16(os.path.join(outdir, f"bg_{k}.pgm"), raw)
            meta["bg"].append(dict(file=f"bg_{k}.pgm", mean=round(m, 1)))
        good = [m for m in means if m + BG_TOL >= max(means)]
        print(f"  background means: {[round(m) for m in means]}  (usable: {len(good)}/{BG_FRAMES})")

        n = 0
        while n < a.count:
            print(f"[{n + 1}/{a.count}] Place your finger ({a.finger}), a slightly different area each time...",
                  end="", flush=True)
            if not s.wait_finger():
                print(" timeout, exiting.")
                break
            raw = s.read_image()
            m = mean16(raw)
            name = f"img_{n:02d}.pgm"
            save_pgm16(os.path.join(outdir, name), raw)
            meta["images"].append(dict(file=name, mean=round(m, 1),
                                       t=round(time.time(), 3)))
            n += 1
            print(f" OK (mean {m:.0f}). Lift it.", end="", flush=True)
            if not s.wait_lift():
                print(" (lift not detected)")
            else:
                print()
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        s.close()
        meta["count"] = len(meta["images"])
        with open(os.path.join(outdir, "meta.json"), "w") as f:
            json.dump(meta, f, indent=1)
        # hand the files back to the invoking user instead of root
        for root, dirs, files in os.walk(os.path.join(base, a.finger)):
            for x in dirs + files:
                os.chown(os.path.join(root, x), pw.pw_uid, pw.pw_gid)
        os.chown(os.path.join(base, a.finger), pw.pw_uid, pw.pw_gid)
        os.chown(base, pw.pw_uid, pw.pw_gid)
        print(f"Saved: {outdir}  ({meta['count']} images)")


if __name__ == "__main__":
    main()
