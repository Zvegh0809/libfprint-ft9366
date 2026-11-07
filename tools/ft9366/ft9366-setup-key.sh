#!/bin/sh
# FT9366 sablonkulcs létrehozása és átadása a fprintd-nek (systemd credential)
#
# - 32 bájt véletlen kulcs, rögtön systemd-creds-szel titkosítva:
#   host+tpm2 => csak ezen a gépen, ezzel a TPM-mel ÉS a
#   /var/lib/systemd/credential.secret fájllal nyitható.
# - Nyílt kulcs soha nem kerül lemezre; futás közben a fprintd
#   $CREDENTIALS_DIRECTORY-jában van (nem swapolható memória).
# - Opcionális PCR-kötés: PCRS=7 ./ft9366-setup-key.sh
#   (csak bekapcsolt Secure Boot mellett van értelme; firmware- vagy
#   Secure Boot-változás után a kulcs nem nyílik => újra kell regisztrálni)
#
# Új kulcs = minden FT9366-os ujjat újra kell regisztrálni.
set -eu

NAME=ft9366-template.key
DIR=/etc/credstore.encrypted
CRED="$DIR/$NAME"
DROPIN_DIR=/etc/systemd/system/fprintd.service.d
DROPIN="$DROPIN_DIR/ft9366-key.conf"

if [ "$(id -u)" -ne 0 ]; then
    echo "Rootként futtasd: sudo $0" >&2
    exit 1
fi

if [ ! -e /dev/tpmrm0 ]; then
    echo "Nincs TPM2 (/dev/tpmrm0). BIOS-ban kapcsold be az AMD fTPM-et," >&2
    echo "vagy (gyengébb) futtasd WITH_KEY=host értékkel." >&2
    [ "${WITH_KEY:-}" = host ] || exit 1
fi

if [ -e "$CRED" ]; then
    echo "Már létezik: $CRED" >&2
    echo "Ha tényleg új kulcs kell: töröld, és utána regisztráld újra az ujjakat." >&2
    exit 1
fi

install -d -m 0700 "$DIR"

umask 077
head -c 32 /dev/urandom |
    systemd-creds encrypt \
        --with-key="${WITH_KEY:-host+tpm2}" \
        ${PCRS:+--tpm2-pcrs="$PCRS"} \
        --name="$NAME" - "$CRED"
chmod 0600 "$CRED"

# ellenőrzés: visszafejthető és pontosan 32 bájt
LEN=$(systemd-creds decrypt --name="$NAME" "$CRED" - | wc -c)
if [ "$LEN" -ne 32 ]; then
    echo "Hiba: a visszafejtett kulcs $LEN bájt" >&2
    exit 1
fi

install -d -m 0755 "$DROPIN_DIR"
cat > "$DROPIN" <<EOF
# FT9366 libfprint driver: sablontitkosító kulcs
[Service]
LoadCredentialEncrypted=$NAME:$CRED
EOF

systemctl daemon-reload
systemctl try-restart fprintd.service || true

echo "Kész: $CRED (${WITH_KEY:-host+tpm2}${PCRS:+, PCR $PCRS})"
echo "Drop-in: $DROPIN"
