#!/bin/sh
# SPDX-License-Identifier: LGPL-2.1-or-later
# Create the FT9366 template key and hand it to fprintd as a systemd credential.
#
# - 32 random bytes, encrypted immediately with systemd-creds. With host+tpm2
#   the key opens only on this machine, with this TPM AND
#   /var/lib/systemd/credential.secret.
# - The plaintext key never touches the disk; at runtime it lives only in
#   fprintd's $CREDENTIALS_DIRECTORY (non-swappable memory).
# - Optional PCR binding: PCRS=7 ./ft9366-setup-key.sh
#   (only meaningful with Secure Boot enabled; after a firmware or Secure Boot
#   change the key will not open and all fingers must be re-enrolled)
#
# A new key means every FT9366 finger has to be re-enrolled.
set -eu

NAME=ft9366-template.key
DIR=/etc/credstore.encrypted
CRED="$DIR/$NAME"
DROPIN_DIR=/etc/systemd/system/fprintd.service.d
DROPIN="$DROPIN_DIR/ft9366-key.conf"

if [ "$(id -u)" -ne 0 ]; then
    echo "Run as root: sudo $0" >&2
    exit 1
fi

if [ ! -e /dev/tpmrm0 ]; then
    echo "No TPM2 (/dev/tpmrm0). Enable the firmware TPM in the BIOS," >&2
    echo "or (weaker) run with WITH_KEY=host." >&2
    [ "${WITH_KEY:-}" = host ] || exit 1
fi

if [ -e "$CRED" ]; then
    echo "Already exists: $CRED" >&2
    echo "If you really need a new key: delete it, then re-enroll all fingers." >&2
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

# sanity check: decrypts to exactly 32 bytes
LEN=$(systemd-creds decrypt --name="$NAME" "$CRED" - | wc -c)
if [ "$LEN" -ne 32 ]; then
    echo "Error: decrypted key is $LEN bytes" >&2
    exit 1
fi

install -d -m 0755 "$DROPIN_DIR"
cat > "$DROPIN" <<EOF
# FT9366 libfprint driver: template encryption key
[Service]
LoadCredentialEncrypted=$NAME:$CRED
EOF

systemctl daemon-reload
systemctl try-restart fprintd.service || true

echo "Done: $CRED (${WITH_KEY:-host+tpm2}${PCRS:+, PCR $PCRS})"
echo "Drop-in: $DROPIN"
