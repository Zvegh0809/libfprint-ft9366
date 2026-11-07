# FocalTech FT9366 driver for libfprint (unofficial)

An experimental, unofficial libfprint driver for the **FocalTech FT9366**
fingerprint sensor behind a **Realtek RTS5811** USB bridge
(**USB ID `2808:a658`**), found in some ASUS Vivobook laptops.

> **Status: experimental.** It works for daily use on the developer's
> machine, but it has been tested on a single device and a single person.
> It is not part of upstream libfprint.

## Tested on

| | |
|---|---|
| Laptop | ASUS Vivobook Pro 15 (M6500QE) |
| OS | CachyOS (Arch Linux), fprintd + PAM (`sudo`, screen unlock) |
| libfprint base | 1.94.100 |

If you have a device with the same USB ID, reports (working or not) are very
welcome - see [Testing and reporting](#testing-and-reporting).

## How it works

- **Image driver.** The sensor delivers a 64x80, 12-bit image. The driver
  subtracts a background captured at activation, filters and crops it to 60x77.
- **Custom matcher.** NBIS finds too few minutiae on such a small image, so
  the driver uses its own matcher (`ft9366_match.c`): SIFT keypoints, a
  RANSAC similarity transform, and normalised cross-correlation over the
  aligned overlap. It is plugged in through new optional extract/match hooks
  in `FpImageDeviceClass` (the core changes in `libfprint/fpi-image-device.c`).
- **Encrypted templates.** A template contains the enhanced fingerprint
  image, so it is stored only encrypted (AES-256-GCM, key from a TPM-sealed
  systemd credential). Without a key the device refuses to open. Details:
  [TEMPLATE-ENCRYPTION.md](TEMPLATE-ENCRYPTION.md).

### Accuracy

On the development set (4 fingers x 20 images, one person) the threshold of
0.80 gives about **21% FRR with 15 enrollment images and no false accepts**.
This is a small dataset; the threshold will be retuned once more people
have been measured.

## Installation

### Arch Linux and derivatives

```sh
git clone -b ft9366 https://github.com/Zvegh0809/libfprint-ft9366 ~/libfprint
cd ~/libfprint/tools/ft9366
makepkg -si                       # installs libfprint-ft9366 in place of libfprint
```

The package is called `libfprint-ft9366`; it provides and conflicts with
`libfprint`, so pacman asks to remove the official package once, and fprintd
keeps working. `pacman -Syu` will not swap it back. To get new driver
changes, pull this branch and rebuild; to go back, run
`sudo pacman -S libfprint`.

### Other distributions

Build with meson and install over the system libfprint. `--prefix=/usr` is
required, otherwise it lands in `/usr/local` and fprintd keeps loading the
system library:

```sh
git clone -b ft9366 https://github.com/Zvegh0809/libfprint-ft9366 ~/libfprint
cd ~/libfprint
meson setup build --prefix=/usr -Ddoc=false -Dgtk-examples=false
ninja -C build
sudo ninja -C build install
```

### Key setup and enrollment

```sh
sudo sh ~/libfprint/tools/ft9366/ft9366-setup-key.sh   # needs a TPM2; see the script header
fprintd-enroll
fprintd-verify
```

The driver asks for 16 touches per finger. Move the finger slightly between
touches so different areas get covered.

## Security model and limitations

- Templates are protected **at rest**: copying `/var/lib/fprint` or the disk
  to another machine does not reveal them.
- On a running system, root can read both the key and the decrypted templates.
- The USB link between the sensor and the host is **neither encrypted nor
  authenticated**, and there is **no liveness detection**. Someone with
  physical access could replay a captured image.
- Use it for convenience (`sudo`, screen unlock), **not as the only factor**
  for disk encryption or other high-value secrets.

Please report security problems privately, as described in
[SECURITY.md](../../.github/SECURITY.md), not in a public issue.

## Known issues

- If a finger rests on the sensor when it activates, calibration waits up to
  about 10 seconds for it to be lifted, then fails with an error (just try
  again). Keep the sensor clear when starting.
- The finger-lift threshold and image polarity still need tuning
  (see the `TODO` comments in `ft9366.c`).

## Development tools

| File | Purpose |
|---|---|
| `ft9366_poc.py` | Minimal proof of concept: read one image with pyusb |
| `ft9366_collect.py` | Capture raw images per finger for matcher development |
| `ft9366_evaluate.py` | Evaluate the matching method (Python/OpenCV reference) |
| `ftm_eval.c` | Evaluate the C matcher on the same data |
| `test_crypt.c`, `test_hooks.c` | Tests for template encryption and the driver hooks |
| `ft9366_seq*.json` | Command sequences recorded from the Windows driver |
| `ft9366-setup-key.sh` | Create the template key and the fprintd drop-in |

The data tools save **real fingerprint images** (by default under
`~/ft9366/data`). Never commit them, and delete them when you are done.

## Testing and reporting

Please report problems with this driver **here, in this repository's
issues**, not to upstream libfprint: the upstream maintainers do not ship or
support it. Include:

- laptop model and `lsusb | grep 2808`,
- whether enrollment and verification work, roughly how often verification
  fails,
- the fprintd log of a failing attempt (`journalctl -u fprintd`), with
  `G_MESSAGES_DEBUG=all` if possible. The logs contain no image data.

## Clean-room statement

The protocol was derived solely by observing the USB traffic of the
Windows driver (usbmon). No vendor code is used. Reverse engineering for
interoperability is permitted under Article 6 of Directive 2009/24/EC.

## License

LGPL-2.1-or-later, like libfprint.
