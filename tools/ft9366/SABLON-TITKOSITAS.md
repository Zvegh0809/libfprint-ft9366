# FT9366 – a tárolt sablon titkosítása

## Mit csinál

- Minden sablon-blob (kép + SIFT-kulcspontok + leírók) AES-256-GCM-mel
  titkosítva kerül az FpPrint-be, így a fprintd `/var/lib/fprint` alá már
  csak rejtjelezett adatot ír. A GCM-tag miatt a módosított vagy kicserélt
  blobot a driver elutasítja.
- A kulcs systemd-credentialként érkezik a fprintd-hez (`LoadCredentialEncrypted=`),
  TPM2 + host-titokkal lezárva: másik gépen, kimásolt lemezről nem nyitható.
- Kulcs nélkül az eszköz meg sem nyílik (fail closed), tehát titkosítatlan
  sablon nem keletkezhet.
- A `FT9366_DUMP` képmentés csak `-DFT9366_DEVEL` fordítással létezik.

## Telepítés

```sh
cd ~/libfprint
patch -p1 --dry-run < ~/ft9366/ft9366-template-encryption.patch && \
patch -p1 < ~/ft9366/ft9366-template-encryption.patch
ninja -C build            # a meson.build változott, magától újrakonfigurál
# telepítés a megszokott módon (a fprintd ezt a libfprintet töltse be!)

sudo sh ~/ft9366/ft9366-setup-key.sh        # kulcs + fprintd drop-in
# opcionálisan Secure Boot mellett: sudo PCRS=7 sh ~/ft9366/ft9366-setup-key.sh

sudo fprintd-delete "$USER"                  # régi, titkosítatlan sablonok
fprintd-enroll                               # újraregisztrálás
```

Ellenőrzés: a `/var/lib/fprint/<user>/ft9366/...` fájlokban nem szabad
`P5`-ös/olvasható képtartalomnak lennie; `fprintd-verify` működik.

## Fejlesztés (examples/enroll, img-capture)

```sh
install -m 600 /dev/null ~/.ft9366-dev.key
head -c 32 /dev/urandom > ~/.ft9366-dev.key
FT9366_KEY_FILE=~/.ft9366-dev.key ./build/examples/enroll
```

A fejlesztői kulcs csak 0600-as jogosultsággal fogadható el. A
`CREDENTIALS_DIRECTORY` (fprintd) mindig elsőbbséget kap.

## Teszt

```sh
D=~/libfprint/libfprint/drivers/ft9366
gcc -std=gnu11 -Wall -Wextra -I"$D" $(pkg-config --cflags glib-2.0 openssl) \
    ~/ft9366/test_crypt.c "$D"/ft9366_crypt.c "$D"/ft9366_match.c -o /tmp/test_crypt \
    $(pkg-config --libs glib-2.0 openssl) -lm
cd "$(mktemp -d)" && /tmp/test_crypt
```

## Mit NEM véd ki

- Futó rendszeren a root (vagy a fprintd-be kódot juttató támadó) a kulcshoz
  és a visszafejtett sablonhoz is hozzáfér: ez tárolás közbeni védelem.
- A fejlesztés közben lementett képek (`~/ft9366/data`, `*.pgm`, logok)
  nem titkosítottak – ezeket kézzel kell törölni/archiválni.
