#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1-or-later
"""
FT9366 (USB 2808:a658) proof of concept: read one fingerprint image on Linux.

Clean-room: the commands come solely from the Windows driver's USB traffic
(usbmon), stored in ft9366_seq.json. No vendor code is used.

Usage (no VM/container may hold the device via USB passthrough):
    sudo python3 ft9366_poc.py                # start with the recorded "arm" stage
    sudo python3 ft9366_poc.py --full-init    # run the full init (with calibration) first

Output: fp_raw16.pgm (16-bit raw, 62x80) and fp_view.pgm (8-bit, contrast stretched).
Requires: python-pyusb
"""
import argparse, json, os, sys, time

import usb.core
import usb.util

VID, PID = 0x2808, 0xA658
EP_OUT, EP_IN = 0x01, 0x82
W, H = 64, 80                 # raw image: 64 words x 80 rows, 16-bit big-endian
TIMEOUT = 2000

HERE = os.path.dirname(os.path.abspath(__file__))


def xfer(dev, out_hex, inlen, expect, verbose):
    dev.write(EP_OUT, bytes.fromhex(out_hex), TIMEOUT)
    if not inlen:
        return b""
    data = bytes(dev.read(EP_IN, max(inlen, 512), TIMEOUT))
    if verbose or (expect and data.hex() != expect):
        mark = "  " if not expect or data.hex() == expect else "!!"
        print(f"{mark} >> {out_hex:<46} << {data.hex()[:32]}"
              + (f"   (recorded: {expect})" if expect and data.hex() != expect else ""))
    return data


def run(dev, seq, name, verbose):
    print(f"-- {name} ({len(seq)} commands)")
    for out_hex, inlen, expect, gap in seq:
        if gap:
            time.sleep(gap)
        xfer(dev, out_hex, inlen, expect, verbose)


def wait_finger(dev, timeout_s=30):
    """Poll 0x9a8b every ~32 ms like the Windows driver: 0x0000 = waiting,
    0x9362 = finger."""
    print(f"-- place your finger on the sensor (max {timeout_s} s)...")
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        r = xfer(dev, "04fb9a8b0001", 2, "", False)
        if r == b"\x93\x62":
            print(f"   finger detected after {time.time() - t0:.2f} s")
            return True
        time.sleep(0.032)
    return False


def save_pgm(raw):
    rows = [raw[r * W * 2:(r + 1) * W * 2] for r in range(H)]
    crop = [row[2:(W - 1) * 2] for row in rows]            # columns 0 and 63 are empty
    w = W - 2
    with open("fp_raw16.pgm", "wb") as f:
        f.write(f"P5 {w} {H} 4095\n".encode())
        f.write(b"".join(crop))                            # PGM16 is big-endian
    vals = [int.from_bytes(c[i:i + 2], "big") for c in crop for i in range(0, len(c), 2)]
    s = sorted(vals)
    lo, hi = s[len(s) // 50], s[-len(s) // 50]
    hi = max(hi, lo + 1)
    px = bytes(max(0, min(255, (v - lo) * 255 // (hi - lo))) for v in vals)
    with open("fp_view.pgm", "wb") as f:
        f.write(f"P5 {w} {H} 255\n".encode())
        f.write(px)
    print(f"-- saved fp_raw16.pgm, fp_view.pgm  (min {s[0]}, max {s[-1]})")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--full-init", action="store_true")
    ap.add_argument("-v", "--verbose", action="store_true")
    ap.add_argument("--seq", default=os.path.join(HERE, "ft9366_seq.json"))
    a = ap.parse_args()

    seq = json.load(open(a.seq))
    dev = usb.core.find(idVendor=VID, idProduct=PID)
    if dev is None:
        sys.exit("No 2808:a658 device (still passed through to a VM/container?)")
    if dev.is_kernel_driver_active(0):
        dev.detach_kernel_driver(0)
    dev.set_configuration()
    usb.util.claim_interface(dev, 0)

    try:
        if a.full_init:
            run(dev, seq["init"], "init", a.verbose)
        run(dev, seq["arm"], "wake + arm finger detection", a.verbose)
        if not wait_finger(dev):
            sys.exit("No finger event.")
        run(dev, seq["capture"], "prepare capture", a.verbose)
        print("-- reading image (06 f9 9a05 1400)")
        dev.write(EP_OUT, bytes.fromhex("06f99a051400"), TIMEOUT)
        raw = bytes(dev.read(EP_IN, W * H * 2, TIMEOUT))
        print(f"   {len(raw)} bytes")
        if len(raw) == W * H * 2:
            save_pgm(raw)
        run(dev, seq["post"], "restore", a.verbose)
    finally:
        try:
            run(dev, seq["sleep"], "sleep", a.verbose)
        except usb.core.USBError as e:
            print("sleep sequence failed:", e)
        usb.util.release_interface(dev, 0)


if __name__ == "__main__":
    main()
