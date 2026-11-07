# FT9366 - template encryption at rest

## What it does

- Every template blob (image + SIFT keypoints + descriptors) is encrypted
  with AES-256-GCM before it is stored in the FpPrint, so fprintd only ever
  writes ciphertext to `/var/lib/fprint`. The GCM tag makes the driver reject
  modified or swapped blobs.
- The key reaches fprintd as a systemd credential (`LoadCredentialEncrypted=`),
  sealed with TPM2 + the host secret: it cannot be opened on another machine
  or from a copied disk.
- Without a key the device does not open (fail closed), so an unencrypted
  template can never be created.
- The `FT9366_DUMP` image dump only exists in builds with `-DFT9366_DEVEL`.

## Installation

```sh
cd tools/ft9366
makepkg -si                                   # builds and installs libfprint (see PKGBUILD)

sudo sh ft9366-setup-key.sh                   # key + fprintd drop-in
# optionally, with Secure Boot enabled: sudo PCRS=7 sh ft9366-setup-key.sh

sudo fprintd-delete "$USER"                   # remove old unencrypted templates
fprintd-enroll
```

Check: files under `/var/lib/fprint/<user>/ft9366/` must not contain `P5`/readable
image data, and `fprintd-verify` must work.

## Development (examples/enroll, img-capture)

```sh
install -m 600 /dev/null ~/.ft9366-dev.key
head -c 32 /dev/urandom > ~/.ft9366-dev.key
FT9366_KEY_FILE=~/.ft9366-dev.key ./build/examples/enroll
```

The development key is only accepted with mode 0600. `CREDENTIALS_DIRECTORY`
(fprintd) always takes precedence.

## Test

```sh
D=libfprint/drivers/ft9366
gcc -std=gnu11 -Wall -Wextra -I"$D" $(pkg-config --cflags glib-2.0 openssl) \
    tools/ft9366/test_crypt.c "$D"/ft9366_crypt.c "$D"/ft9366_match.c -o /tmp/test_crypt \
    $(pkg-config --libs glib-2.0 openssl) -lm
cd "$(mktemp -d)" && /tmp/test_crypt
```

## What it does NOT protect against

- On a running system, root (or an attacker who gets code into fprintd) can
  access both the key and the decrypted templates: this is protection at rest.
- The sensor's USB link is neither encrypted nor authenticated, and there is
  no liveness detection, so captured images can be replayed by someone with
  physical access.
- Images saved during development (`~/ft9366/data`, `*.pgm`, logs) are not
  encrypted; delete or archive them yourself.
