/*
 * FT9366 – sablonok titkosítása tároláshoz – lásd ft9366_crypt.h
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#include "ft9366_crypt.h"

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include <openssl/crypto.h>
#include <openssl/evp.h>
#include <openssl/rand.h>

#define MAGIC          "FT9E"
#define MAGIC_LEN      4
#define VERSION        1
#define HDR_LEN        (MAGIC_LEN + 1)
#define NONCE_LEN      12
#define TAG_LEN        16
#define CONTEXT        "libfprint/ft9366/template/v1"

G_STATIC_ASSERT (FT9366_CRYPT_OVERHEAD == HDR_LEN + NONCE_LEN + TAG_LEN);

G_DEFINE_QUARK (ft9366-crypt-error-quark, ft9366_crypt_error)

/* ------------------------------------------------------------------ */
/* Kulcs                                                                */
/* ------------------------------------------------------------------ */

/* Pontosan FT9366_CRYPT_KEY_LEN bájtot olvas a fájlból; ha rövidebb vagy
 * hosszabb, hibát ad. Köztes másolat nem marad. */
static gboolean
read_key_file (const char *path, gboolean check_mode,
               guint8 key[FT9366_CRYPT_KEY_LEN], GError **error)
{
  guint8 buf[FT9366_CRYPT_KEY_LEN + 1];
  gsize got = 0;
  struct stat st;
  int fd;

  fd = open (path, O_RDONLY | O_CLOEXEC | O_NOFOLLOW);
  if (fd < 0)
    {
      g_set_error (error, FT9366_CRYPT_ERROR, FT9366_CRYPT_ERROR_NO_KEY,
                   "Cannot open template key %s: %s", path, g_strerror (errno));
      return FALSE;
    }

  if (fstat (fd, &st) != 0 || !S_ISREG (st.st_mode) ||
      (check_mode && (st.st_mode & (S_IRWXG | S_IRWXO))))
    {
      close (fd);
      g_set_error (error, FT9366_CRYPT_ERROR, FT9366_CRYPT_ERROR_NO_KEY,
                   "Template key %s is not a regular file readable only by its owner",
                   path);
      return FALSE;
    }

  while (got < sizeof (buf))
    {
      ssize_t r = read (fd, buf + got, sizeof (buf) - got);

      if (r < 0 && errno == EINTR)
        continue;
      if (r <= 0)
        break;
      got += r;
    }
  close (fd);

  if (got != FT9366_CRYPT_KEY_LEN)
    {
      OPENSSL_cleanse (buf, sizeof (buf));
      g_set_error (error, FT9366_CRYPT_ERROR, FT9366_CRYPT_ERROR_NO_KEY,
                   "Template key %s must be exactly %d raw bytes",
                   path, FT9366_CRYPT_KEY_LEN);
      return FALSE;
    }

  memcpy (key, buf, FT9366_CRYPT_KEY_LEN);
  OPENSSL_cleanse (buf, sizeof (buf));
  return TRUE;
}

static gboolean
load_key (guint8 key[FT9366_CRYPT_KEY_LEN], GError **error)
{
  const char *dir = g_getenv ("CREDENTIALS_DIRECTORY");
  const char *dev = g_getenv ("FT9366_KEY_FILE");

  /* Éles út: systemd credential. A könyvtárat a systemd hozza létre,
   * csak a szolgáltatás olvashatja, ezért itt nem nézzük a módot. */
  if (dir)
    {
      g_autofree char *path = g_build_filename (dir, FT9366_CRYPT_CRED_NAME, NULL);
      return read_key_file (path, FALSE, key, error);
    }

  /* Fejlesztői út (pl. examples/enroll root nélkül) */
  if (dev)
    return read_key_file (dev, TRUE, key, error);

  g_set_error (error, FT9366_CRYPT_ERROR, FT9366_CRYPT_ERROR_NO_KEY,
               "No template key: set up the '%s' systemd credential for fprintd "
               "(or FT9366_KEY_FILE for development)", FT9366_CRYPT_CRED_NAME);
  return FALSE;
}

gboolean
ft9366_crypt_check_key (GError **error)
{
  guint8 key[FT9366_CRYPT_KEY_LEN];
  gboolean ok = load_key (key, error);

  OPENSSL_cleanse (key, sizeof (key));
  return ok;
}

/* ------------------------------------------------------------------ */
/* AEAD                                                                 */
/* ------------------------------------------------------------------ */

static gboolean
aad_add (EVP_CIPHER_CTX *ctx, const guint8 *hdr)
{
  int outl;

  return EVP_EncryptUpdate (ctx, NULL, &outl, hdr, HDR_LEN) == 1 &&
         EVP_EncryptUpdate (ctx, NULL, &outl, (const guint8 *) CONTEXT,
                            sizeof (CONTEXT) - 1) == 1;
}

static gboolean
aad_add_dec (EVP_CIPHER_CTX *ctx, const guint8 *hdr)
{
  int outl;

  return EVP_DecryptUpdate (ctx, NULL, &outl, hdr, HDR_LEN) == 1 &&
         EVP_DecryptUpdate (ctx, NULL, &outl, (const guint8 *) CONTEXT,
                            sizeof (CONTEXT) - 1) == 1;
}

guint8 *
ft9366_crypt_seal (const guint8 *plain, gsize len, gsize *out_len, GError **error)
{
  guint8 key[FT9366_CRYPT_KEY_LEN];
  EVP_CIPHER_CTX *ctx = NULL;
  guint8 *out = NULL, *nonce, *ct, *tag;
  gboolean ok = FALSE;
  int outl, fin;

  g_return_val_if_fail (plain != NULL && out_len != NULL, NULL);

  if (len > G_MAXINT - FT9366_CRYPT_OVERHEAD)
    {
      g_set_error (error, FT9366_CRYPT_ERROR, FT9366_CRYPT_ERROR_INTERNAL,
                   "Template too large");
      return NULL;
    }

  if (!load_key (key, error))
    return NULL;

  out = g_malloc (len + FT9366_CRYPT_OVERHEAD);
  memcpy (out, MAGIC, MAGIC_LEN);
  out[MAGIC_LEN] = VERSION;
  nonce = out + HDR_LEN;
  ct = nonce + NONCE_LEN;
  tag = ct + len;

  /* 96 bites véletlen nonce: kulcsonként néhány száz titkosításnál az
   * ütközés esélye elhanyagolható (NIST SP 800-38D, 8.3: <= 2^32 hívás) */
  if (RAND_bytes (nonce, NONCE_LEN) != 1)
    goto out;

  ctx = EVP_CIPHER_CTX_new ();
  if (!ctx ||
      EVP_EncryptInit_ex (ctx, EVP_aes_256_gcm (), NULL, NULL, NULL) != 1 ||
      EVP_CIPHER_CTX_ctrl (ctx, EVP_CTRL_GCM_SET_IVLEN, NONCE_LEN, NULL) != 1 ||
      EVP_EncryptInit_ex (ctx, NULL, NULL, key, nonce) != 1 ||
      !aad_add (ctx, out) ||
      EVP_EncryptUpdate (ctx, ct, &outl, plain, (int) len) != 1 ||
      EVP_EncryptFinal_ex (ctx, ct + outl, &fin) != 1 ||
      (gsize) (outl + fin) != len ||
      EVP_CIPHER_CTX_ctrl (ctx, EVP_CTRL_GCM_GET_TAG, TAG_LEN, tag) != 1)
    goto out;

  ok = TRUE;

out:
  OPENSSL_cleanse (key, sizeof (key));
  EVP_CIPHER_CTX_free (ctx);
  if (!ok)
    {
      OPENSSL_cleanse (out, len + FT9366_CRYPT_OVERHEAD);
      g_free (out);
      g_set_error (error, FT9366_CRYPT_ERROR, FT9366_CRYPT_ERROR_INTERNAL,
                   "Template encryption failed");
      return NULL;
    }
  *out_len = len + FT9366_CRYPT_OVERHEAD;
  return out;
}

gboolean
ft9366_crypt_is_sealed (const guint8 *blob, gsize len)
{
  return blob && len >= FT9366_CRYPT_OVERHEAD &&
         memcmp (blob, MAGIC, MAGIC_LEN) == 0;
}

guint8 *
ft9366_crypt_open (const guint8 *blob, gsize len, gsize *out_len, GError **error)
{
  guint8 key[FT9366_CRYPT_KEY_LEN];
  EVP_CIPHER_CTX *ctx = NULL;
  const guint8 *nonce, *ct, *tag;
  guint8 *plain = NULL;
  gsize pl;
  int outl, fin, rc = 0;

  g_return_val_if_fail (out_len != NULL, NULL);

  if (!ft9366_crypt_is_sealed (blob, len) || len > G_MAXINT)
    {
      g_set_error (error, FT9366_CRYPT_ERROR, FT9366_CRYPT_ERROR_FORMAT,
                   "Template is not encrypted (enrolled with an older driver?)");
      return NULL;
    }
  if (blob[MAGIC_LEN] != VERSION)
    {
      g_set_error (error, FT9366_CRYPT_ERROR, FT9366_CRYPT_ERROR_FORMAT,
                   "Unsupported template encryption version %u", blob[MAGIC_LEN]);
      return NULL;
    }

  if (!load_key (key, error))
    return NULL;

  pl = len - FT9366_CRYPT_OVERHEAD;
  nonce = blob + HDR_LEN;
  ct = nonce + NONCE_LEN;
  tag = ct + pl;
  plain = g_malloc (pl ? pl : 1);

  ctx = EVP_CIPHER_CTX_new ();
  if (ctx &&
      EVP_DecryptInit_ex (ctx, EVP_aes_256_gcm (), NULL, NULL, NULL) == 1 &&
      EVP_CIPHER_CTX_ctrl (ctx, EVP_CTRL_GCM_SET_IVLEN, NONCE_LEN, NULL) == 1 &&
      EVP_DecryptInit_ex (ctx, NULL, NULL, key, nonce) == 1 &&
      aad_add_dec (ctx, blob) &&
      EVP_DecryptUpdate (ctx, plain, &outl, ct, (int) pl) == 1 &&
      EVP_CIPHER_CTX_ctrl (ctx, EVP_CTRL_GCM_SET_TAG, TAG_LEN, (void *) tag) == 1)
    rc = EVP_DecryptFinal_ex (ctx, plain + outl, &fin);   /* tag-ellenőrzés */

  OPENSSL_cleanse (key, sizeof (key));
  EVP_CIPHER_CTX_free (ctx);

  if (rc != 1)
    {
      ft9366_crypt_free_plain (plain, pl ? pl : 1);
      g_set_error (error, FT9366_CRYPT_ERROR, FT9366_CRYPT_ERROR_AUTH,
                   "Template authentication failed (wrong key or tampered data)");
      return NULL;
    }

  *out_len = pl;
  return plain;
}

void
ft9366_crypt_free_plain (guint8 *plain, gsize len)
{
  if (!plain)
    return;
  OPENSSL_cleanse (plain, len);
  g_free (plain);
}
