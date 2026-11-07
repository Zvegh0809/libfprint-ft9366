/*
 * FT9366 template encryption at rest (AES-256-GCM, OpenSSL 3)
 *
 * The host-side matcher's template contains the enhanced 60x77 fingerprint
 * image (NCC scoring runs on it), so a template is effectively the
 * fingerprint itself. Templates therefore only leave the driver encrypted;
 * plaintext exists in memory only while matching.
 *
 * Blob format (v1):
 *   "FT9E" | 0x01 | nonce[12] | ciphertext | tag[16]
 * AAD: the 5-byte header + a fixed context string, so blobs encrypted for
 * another purpose or format version are rejected.
 *
 * Key (32 raw bytes):
 *   1. $CREDENTIALS_DIRECTORY/ft9366-template.key - production: a systemd
 *      credential passed to fprintd.service, sealed with TPM2
 *      (LoadCredentialEncrypted=), see tools/ft9366/TEMPLATE-ENCRYPTION.md;
 *   2. $FT9366_KEY_FILE - development only (e.g. examples/enroll).
 * Without a key the driver neither enrolls nor matches (fail closed).
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#pragma once

#include <glib.h>

G_BEGIN_DECLS

#define FT9366_CRYPT_KEY_LEN     32
#define FT9366_CRYPT_CRED_NAME   "ft9366-template.key"
#define FT9366_CRYPT_OVERHEAD    (4 + 1 + 12 + 16)

#define FT9366_CRYPT_ERROR (ft9366_crypt_error_quark ())
GQuark ft9366_crypt_error_quark (void);

typedef enum {
  FT9366_CRYPT_ERROR_NO_KEY,     /* no (valid) key                 */
  FT9366_CRYPT_ERROR_FORMAT,     /* not a sealed blob              */
  FT9366_CRYPT_ERROR_AUTH,       /* tag mismatch or wrong key      */
  FT9366_CRYPT_ERROR_INTERNAL,   /* OpenSSL error, OOM             */
} Ft9366CryptError;

/* Called on open: without a usable key the device does not open, so no
 * unencrypted template can ever be produced. */
gboolean ft9366_crypt_check_key (GError **error);

/* Returns a new blob of len + FT9366_CRYPT_OVERHEAD bytes (free with
 * g_free), or NULL and sets @error. */
guint8 *ft9366_crypt_seal (const guint8 *plain, gsize len,
                           gsize *out_len, GError **error);

/* Decrypts and authenticates. Free the result with
 * ft9366_crypt_free_plain(), which also wipes it. */
guint8 *ft9366_crypt_open (const guint8 *blob, gsize len,
                           gsize *out_len, GError **error);

void    ft9366_crypt_free_plain (guint8 *plain, gsize len);

/* Format check only (detects old unencrypted templates); does not
 * authenticate. */
gboolean ft9366_crypt_is_sealed (const guint8 *blob, gsize len);

G_END_DECLS
