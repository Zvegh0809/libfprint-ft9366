/* SPDX-License-Identifier: LGPL-2.1-or-later */
/* Tests for ft9366_crypt; optional args: real unencrypted templates */
#include "ft9366_crypt.h"
#include "ft9366_match.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

static int fails = 0;
#define CHECK(c, msg) do { if (c) printf ("ok   %s\n", msg); else { printf ("FAIL %s\n", msg); fails++; } } while (0)

static void write_key (const char *p, unsigned char fill, size_t n, mode_t mode)
{
  FILE *f = fopen (p, "wb");
  for (size_t i = 0; i < n; i++) fputc ((int) (fill + i), f);
  fclose (f);
  chmod (p, mode);
}

int main (int argc, char **argv)
{
  GError *err = NULL;
  gsize slen, plen;
  guint8 *s, *p;
  const guint8 msg[] = "fingerprint-template";

  g_unsetenv ("CREDENTIALS_DIRECTORY");
  g_unsetenv ("FT9366_KEY_FILE");
  CHECK (!ft9366_crypt_check_key (&err) && err->code == FT9366_CRYPT_ERROR_NO_KEY, "no key: fail closed");
  g_clear_error (&err);

  write_key ("k_bad_mode", 1, 32, 0644);
  g_setenv ("FT9366_KEY_FILE", "k_bad_mode", TRUE);
  CHECK (!ft9366_crypt_check_key (&err), "group/world-readable key file rejected");
  g_clear_error (&err);

  write_key ("k_short", 1, 31, 0600);
  g_setenv ("FT9366_KEY_FILE", "k_short", TRUE);
  CHECK (!ft9366_crypt_check_key (&err), "31-byte key rejected");
  g_clear_error (&err);

  write_key ("k1", 1, 32, 0600);
  write_key ("k2", 2, 32, 0600);
  g_setenv ("FT9366_KEY_FILE", "k1", TRUE);
  CHECK (ft9366_crypt_check_key (NULL), "32-byte 0600 key accepted");

  s = ft9366_crypt_seal (msg, sizeof msg, &slen, &err);
  CHECK (s && slen == sizeof msg + FT9366_CRYPT_OVERHEAD, "seal");
  CHECK (memmem (s, slen, "fingerprint", 11) == NULL, "plaintext not visible in blob");
  p = ft9366_crypt_open (s, slen, &plen, &err);
  CHECK (p && plen == sizeof msg && memcmp (p, msg, plen) == 0, "open");
  ft9366_crypt_free_plain (p, plen);

  {
    gsize s2len; guint8 *s2 = ft9366_crypt_seal (msg, sizeof msg, &s2len, NULL);
    CHECK (memcmp (s + 5, s2 + 5, 12) != 0, "nonces differ between seals");
    g_free (s2);
  }

  for (gsize i = 0; i < slen; i++)
    {
      s[i] ^= 0x01;
      p = ft9366_crypt_open (s, slen, &plen, &err);
      s[i] ^= 0x01;
      if (p) { fails++; printf ("FAIL modified byte %zu accepted\n", i); ft9366_crypt_free_plain (p, plen); }
      g_clear_error (&err);
    }
  printf ("ok   single-bit flips rejected in all %zu bytes\n", slen);

  g_setenv ("FT9366_KEY_FILE", "k2", TRUE);
  p = ft9366_crypt_open (s, slen, &plen, &err);
  CHECK (!p && err && err->code == FT9366_CRYPT_ERROR_AUTH, "wrong key rejected");
  g_clear_error (&err);
  g_setenv ("FT9366_KEY_FILE", "k1", TRUE);

  p = ft9366_crypt_open (s, slen - 1, &plen, &err);
  CHECK (!p, "truncated blob rejected");
  g_clear_error (&err);
  g_free (s);

  /* systemd credential path */
  mkdir ("creds", 0700);
  write_key ("creds/" FT9366_CRYPT_CRED_NAME, 1, 32, 0400);
  g_setenv ("CREDENTIALS_DIRECTORY", "creds", TRUE);
  g_setenv ("FT9366_KEY_FILE", "k2", TRUE);   /* the credential must win */
  s = ft9366_crypt_seal (msg, sizeof msg, &slen, NULL);
  g_unsetenv ("CREDENTIALS_DIRECTORY");
  g_setenv ("FT9366_KEY_FILE", "k1", TRUE);
  p = ft9366_crypt_open (s, slen, &plen, NULL);
  CHECK (p != NULL, "CREDENTIALS_DIRECTORY takes precedence");
  ft9366_crypt_free_plain (p, plen); g_free (s);

  /* real templates: detect unencrypted blobs, check score after round trip */
  {
    int n = 0, same = 0;
    for (int a = 1; a < argc; a++)
      {
        gchar *raw; gsize rl;
        if (!g_file_get_contents (argv[a], &raw, &rl, NULL)) continue;
        p = ft9366_crypt_open ((guint8 *) raw, rl, &plen, &err);
        if (!p && err->code == FT9366_CRYPT_ERROR_FORMAT) n++;
        g_clear_error (&err);

        FtmFeatures *f0 = ftm_deserialize ((guint8 *) raw, rl);
        s = ft9366_crypt_seal ((guint8 *) raw, rl, &slen, NULL);
        p = ft9366_crypt_open (s, slen, &plen, NULL);
        FtmFeatures *f1 = ftm_deserialize (p, plen);
        FtmFeatures *probe = ftm_deserialize ((guint8 *) raw, rl);
        if (f0 && f1 && ftm_score (probe, f0) == ftm_score (probe, f1)) same++;
        ftm_free (f0); ftm_free (f1); ftm_free (probe);
        ft9366_crypt_free_plain (p, plen); g_free (s); g_free (raw);
      }
    printf ("%s unencrypted template detected (FORMAT error): %d/%d\n", n == argc - 1 ? "ok  " : "FAIL", n, argc - 1);
    printf ("%s identical match score after round trip: %d/%d\n", same == argc - 1 ? "ok  " : "FAIL", same, argc - 1);
    if (n != argc - 1 || same != argc - 1) fails++;
  }

  printf ("\n%s\n", fails ? "FAILURES" : "all tests passed");
  return fails != 0;
}
