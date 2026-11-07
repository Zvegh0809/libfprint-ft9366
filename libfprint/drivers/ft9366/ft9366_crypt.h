/*
 * FT9366 – sablonok titkosítása tároláshoz (AES-256-GCM, OpenSSL 3)
 *
 * A host-oldali illesztő sablonja a 60x77-es, feljavított ujjlenyomatképet
 * is tartalmazza (az NCC-pontozás ezen fut), vagyis a sablon gyakorlatilag
 * maga az ujjlenyomat. Ezért minden sablon-blob csak titkosítva hagyja el a
 * drivert; a nyílt szöveg csak az illesztés idejére, memóriában létezik.
 *
 * Blob-formátum (v1):
 *   "FT9E" | 0x01 | nonce[12] | rejtjelezett szöveg | tag[16]
 * Az AEAD kiegészítő adata (AAD): a 5 bájtos fejléc + FT9366_CRYPT_CONTEXT,
 * így más célra/más formátumverzióval titkosított blob nem fogadható el.
 *
 * Kulcs (32 bájt, nyers):
 *   1. $CREDENTIALS_DIRECTORY/ft9366-template.key  – éles használat:
 *      systemd-hitelesítő adat, a fprintd.service kapja meg TPM2-vel
 *      lezárva (LoadCredentialEncrypted=), lásd a README-t;
 *   2. $FT9366_KEY_FILE – csak fejlesztéshez (pl. examples/enroll).
 * Kulcs nélkül a driver nem regisztrál és nem illeszt (fail closed).
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
  FT9366_CRYPT_ERROR_NO_KEY,     /* nincs (érvényes) kulcs           */
  FT9366_CRYPT_ERROR_FORMAT,     /* nem ilyen formátumú blob         */
  FT9366_CRYPT_ERROR_AUTH,       /* hitelesítés sikertelen / más kulcs */
  FT9366_CRYPT_ERROR_INTERNAL,   /* OpenSSL-hiba, OOM                */
} Ft9366CryptError;

/* Igaz, ha van használható kulcs (megnyitáskor hívjuk: kulcs nélkül az
 * eszköz meg sem nyílik, így nem keletkezhet titkosítatlan sablon). */
gboolean ft9366_crypt_check_key (GError **error);

/* Titkosítás. Visszaad egy új, len + FT9366_CRYPT_OVERHEAD bájtos blobot
 * (g_free-vel felszabadítandó), vagy NULL-t és @error-t. */
guint8 *ft9366_crypt_seal (const guint8 *plain, gsize len,
                           gsize *out_len, GError **error);

/* Visszafejtés + hitelesítés. A visszaadott puffert a hívó
 * ft9366_crypt_free_plain()-nel szabadítja fel (nullázza is). */
guint8 *ft9366_crypt_open (const guint8 *blob, gsize len,
                           gsize *out_len, GError **error);

void    ft9366_crypt_free_plain (guint8 *plain, gsize len);

/* Igaz, ha a blob ebben a formátumban van (titkosítatlan régi sablonok
 * felismeréséhez). Nem ellenőrzi a hitelességet. */
gboolean ft9366_crypt_is_sealed (const guint8 *blob, gsize len);

G_END_DECLS
