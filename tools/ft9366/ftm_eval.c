/*
 * ftm_eval – offline evaluation of ft9366_match.c on recorded sessions
 *
 *   ftm_eval <data-dir> [score-matrix-out.bin] [image-dump-dir]
 *
 * <data-dir>/<finger>/<session>/{bg_*.pgm,img_*.pgm}: 16-bit raw frames as
 * written by ft9366_collect.py. Each image is preprocessed exactly like
 * build_image() in the ft9366 driver, then all ordered pairs are scored and
 * FRR/FAR are reported with the same rotating enrol/probe split as
 * ft9366_evaluate.py.
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#include "ft9366_match.h"

#include <dirent.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define RAW_W  64
#define RAW_H  80
#define IMG_X0 2
#define IMG_Y0 3
#define IMG_W  (RAW_W - 2 * IMG_X0)
#define IMG_H  (RAW_H - IMG_Y0)
#define HP_BOX 9
#define MAXIMG 512

#define CLAMP(v, lo, hi) ((v) < (lo) ? (lo) : ((v) > (hi) ? (hi) : (v)))
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define AT(buf, w, h, x, y) \
  (buf)[CLAMP ((int) (y), 0, (h) - 1) * (w) + CLAMP ((int) (x), 0, (w) - 1)]

static int
read_raw (const char *path, float *out)
{
  unsigned char buf[RAW_W * RAW_H * 2];
  FILE *f = fopen (path, "rb");
  char hdr[32];
  int i, w, h, mx;

  if (!f)
    return 0;
  if (fscanf (f, "P5 %d %d %d", &w, &h, &mx) != 3 || w != RAW_W || h != RAW_H)
    {
      fclose (f);
      return 0;
    }
  fgetc (f);
  (void) hdr;
  if (fread (buf, 1, sizeof (buf), f) != sizeof (buf))
    {
      fclose (f);
      return 0;
    }
  fclose (f);
  for (i = 0; i < RAW_W * RAW_H; i++)
    out[i] = (float) ((buf[2 * i] << 8) | buf[2 * i + 1]);
  return 1;
}

static int
cmp_float (const void *a, const void *b)
{
  float x = *(const float *) a, y = *(const float *) b;
  return (x > y) - (x < y);
}

/* copy of build_image() from drivers/ft9366/ft9366.c (ENLARGE == 1) */
static void
build_image (const float *raw, const float *bg, unsigned char *small)
{
  static float a[IMG_W * IMG_H], b[IMG_W * IMG_H];
  static double integ[(IMG_W + 1) * (IMG_H + 1)];
  const int n = IMG_W * IMG_H, r = HP_BOX / 2;
  float lo, hi;
  int x, y, i;

  for (y = 0; y < IMG_H; y++)
    for (x = 0; x < IMG_W; x++)
      {
        int ri = (y + IMG_Y0) * RAW_W + x + IMG_X0;
        a[y * IMG_W + x] = raw[ri] - bg[ri];
      }
  for (y = 0; y < IMG_H; y++)
    for (x = 0; x < IMG_W; x++)
      {
        float s =
          1 * AT (a, IMG_W, IMG_H, x - 1, y - 1) + 2 * AT (a, IMG_W, IMG_H, x, y - 1) + 1 * AT (a, IMG_W, IMG_H, x + 1, y - 1) +
          2 * AT (a, IMG_W, IMG_H, x - 1, y)     + 4 * AT (a, IMG_W, IMG_H, x, y)     + 2 * AT (a, IMG_W, IMG_H, x + 1, y) +
          1 * AT (a, IMG_W, IMG_H, x - 1, y + 1) + 2 * AT (a, IMG_W, IMG_H, x, y + 1) + 1 * AT (a, IMG_W, IMG_H, x + 1, y + 1);
        b[y * IMG_W + x] = s / 16.0f;
      }
  memset (integ, 0, sizeof (integ));
  for (y = 1; y <= IMG_H; y++)
    for (x = 1; x <= IMG_W; x++)
      integ[y * (IMG_W + 1) + x] = b[(y - 1) * IMG_W + (x - 1)]
                                   + integ[(y - 1) * (IMG_W + 1) + x]
                                   + integ[y * (IMG_W + 1) + x - 1]
                                   - integ[(y - 1) * (IMG_W + 1) + x - 1];
  for (y = 0; y < IMG_H; y++)
    for (x = 0; x < IMG_W; x++)
      {
        int x0 = MAX (x - r, 0), x1 = MIN (x + r + 1, IMG_W);
        int y0 = MAX (y - r, 0), y1 = MIN (y + r + 1, IMG_H);
        double s = integ[y1 * (IMG_W + 1) + x1] - integ[y0 * (IMG_W + 1) + x1]
                   - integ[y1 * (IMG_W + 1) + x0] + integ[y0 * (IMG_W + 1) + x0];
        a[y * IMG_W + x] = b[y * IMG_W + x] - (float) (s / ((x1 - x0) * (y1 - y0)));
      }
  memcpy (b, a, sizeof (a));
  qsort (b, n, sizeof (float), cmp_float);
  lo = b[n / 50];
  hi = b[n - 1 - n / 50];
  if (hi <= lo)
    hi = lo + 1;
  for (i = 0; i < n; i++)
    small[i] = (unsigned char) CLAMP ((int) ((a[i] - lo) * 255.0f / (hi - lo)), 0, 255);
}

static int
cmp_str (const void *a, const void *b)
{
  return strcmp (*(char *const *) a, *(char *const *) b);
}

static int
list_dir (const char *path, const char *prefix, char **out, int max)
{
  DIR *d = opendir (path);
  struct dirent *e;
  int n = 0;

  if (!d)
    return 0;
  while ((e = readdir (d)) && n < max)
    {
      if (e->d_name[0] == '.')
        continue;
      if (prefix && strncmp (e->d_name, prefix, strlen (prefix)) != 0)
        continue;
      out[n++] = strdup (e->d_name);
    }
  closedir (d);
  qsort (out, n, sizeof (char *), cmp_str);
  return n;
}

static double
now (void)
{
  struct timespec ts;
  clock_gettime (CLOCK_MONOTONIC, &ts);
  return ts.tv_sec + ts.tv_nsec * 1e-9;
}

int
main (int argc, char **argv)
{
  static unsigned char imgs[MAXIMG][FTM_W * FTM_H];
  static int finger[MAXIMG], idx[MAXIMG];
  static double C[MAXIMG][MAXIMG];
  FtmFeatures *F[MAXIMG];
  char *fingers[64], *sess[64], *files[256], path[1024];
  int nf, n = 0, f, s, i, j, per = 1 << 30, kpmin = 1 << 30, kpmax = 0;
  double t0, t_ext, t_match, imp_max = 0, kpsum = 0;
  const char *data, *dump = argc > 3 ? argv[3] : NULL;

  if (argc < 2)
    {
      fprintf (stderr, "usage: %s <data-dir> [matrix.bin] [dump-dir]\n", argv[0]);
      return 2;
    }
  data = argv[1];

  nf = list_dir (data, NULL, fingers, 64);
  for (f = 0; f < nf; f++)
    {
      int ns, cnt = 0;
      snprintf (path, sizeof path, "%s/%s", data, fingers[f]);
      ns = list_dir (path, NULL, sess, 64);
      for (s = 0; s < ns; s++)
        {
          float bg[RAW_W * RAW_H] = { 0 }, raw[RAW_W * RAW_H];
          char sdir[1024];
          int nb, ni, k, p;

          snprintf (sdir, sizeof sdir, "%s/%s/%s", data, fingers[f], sess[s]);
          nb = list_dir (sdir, "bg_", files, 256);
          for (k = 0; k < nb; k++)
            {
              snprintf (path, sizeof path, "%s/%s", sdir, files[k]);
              if (!read_raw (path, raw))
                return 1;
              for (p = 0; p < RAW_W * RAW_H; p++)
                bg[p] += raw[p] / nb;
              free (files[k]);
            }
          ni = list_dir (sdir, "img_", files, 256);
          for (k = 0; k < ni && n < MAXIMG; k++)
            {
              snprintf (path, sizeof path, "%s/%s", sdir, files[k]);
              if (!read_raw (path, raw))
                return 1;
              build_image (raw, bg, imgs[n]);
              if (dump)
                {
                  FILE *o;
                  snprintf (path, sizeof path, "%s/%03d_%s.pgm", dump, n, fingers[f]);
                  o = fopen (path, "wb");
                  fprintf (o, "P5 %d %d 255\n", FTM_W, FTM_H);
                  fwrite (imgs[n], 1, FTM_W * FTM_H, o);
                  fclose (o);
                }
              finger[n] = f;
              idx[n] = cnt++;
              n++;
              free (files[k]);
            }
          free (sess[s]);
        }
      if (cnt < per)
        per = cnt;
    }
  printf ("%d kép, %d ujj\n", n, nf);

  t0 = now ();
  for (i = 0; i < n; i++)
    {
      F[i] = ftm_extract (imgs[i]);
      if (!F[i])
        return 1;
      kpsum += ftm_n_keypoints (F[i]);
      kpmin = MIN (kpmin, ftm_n_keypoints (F[i]));
      kpmax = MAX (kpmax, ftm_n_keypoints (F[i]));
    }
  t_ext = now () - t0;

  t0 = now ();
  for (i = 0; i < n; i++)
    for (j = 0; j < n; j++)
      C[i][j] = i == j ? 0 : ftm_score (F[i], F[j]);
  t_match = now () - t0;

  for (i = 0; i < n; i++)
    for (j = 0; j < n; j++)
      if (finger[i] != finger[j] && C[i][j] > imp_max)
        imp_max = C[i][j];

  printf ("kulcspont/kép: min %d, átlag %.0f, max %d\n", kpmin, kpsum / n, kpmax);
  printf ("idő: kinyerés %.1f ms/kép, illesztés %.2f ms/pár\n",
          1000 * t_ext / n, 1000 * t_match / ((double) n * (n - 1)));
  printf ("különböző ujjú párok legmagasabb pontszáma: %.3f\n", imp_max);

  for (int nen = 5; nen <= 15; nen += 5)
    {
      static double G[100000], I[100000];
      int ng = 0, ni = 0;
      if (nen >= per)
        continue;
      for (s = 0; s < per; s++)
        for (f = 0; f < nf; f++)
          for (i = 0; i < n; i++)
            {
              int k, pr = 0;
              double best = 0;
              for (k = 0; k < per - nen; k++)
                if (idx[i] == (s + nen + k) % per)
                  pr = 1;
              if (!pr)
                continue;
              for (j = 0; j < n; j++)
                {
                  int en = 0;
                  if (finger[j] != f)
                    continue;
                  for (k = 0; k < nen; k++)
                    if (idx[j] == (s + k) % per)
                      en = 1;
                  if (en && C[i][j] > best)
                    best = C[i][j];
                }
              if (finger[i] == f)
                G[ng++] = best;
              else
                I[ni++] = best;
            }
      for (double thr = 0.75; thr < 0.86; thr += 0.05)
        {
          int fr = 0, fa = 0;
          for (i = 0; i < ng; i++)
            fr += G[i] < thr;
          for (i = 0; i < ni; i++)
            fa += I[i] >= thr;
          printf ("  regisztráció %2d kép, küszöb %.2f: FRR %5.1f%%  FAR %.2f%%\n",
                  nen, thr, 100.0 * fr / ng, 100.0 * fa / ni);
        }
    }

  /* serialisation round trip must not change any score */
  {
    double maxdiff = 0;
    size_t total = 0;
    int bad = 0;

    for (i = 0; i < n; i++)
      {
        size_t sz = ftm_serialized_size (F[i]);
        unsigned char *buf = malloc (sz);
        FtmFeatures *g;

        ftm_serialize (F[i], buf);
        total += sz;
        /* corrupt inputs must be rejected */
        bad += ftm_deserialize (buf, sz - 1) != NULL;
        buf[0] ^= 0xff;
        bad += ftm_deserialize (buf, sz) != NULL;
        buf[0] ^= 0xff;
        g = ftm_deserialize (buf, sz);
        free (buf);
        if (!g)
          {
            printf ("HIBA: deszerializálás sikertelen (%d)\n", i);
            return 1;
          }
        ftm_free (F[i]);
        F[i] = g;
      }
    for (i = 0; i < n; i++)
      for (j = 0; j < n; j++)
        if (i != j)
          maxdiff = fmax (maxdiff, fabs (ftm_score (F[i], F[j]) - C[i][j]));
    printf ("szerializálás: átlag %.1f KB/kép, oda-vissza max eltérés %.3g, "
            "hibás bemenet elfogadva: %d\n", total / 1024.0 / n, maxdiff, bad);
  }

  if (argc > 2)
    {
      FILE *o = fopen (argv[2], "wb");
      fwrite (&n, sizeof n, 1, o);
      for (i = 0; i < n; i++)
        fwrite (C[i], sizeof (double), n, o);
      fclose (o);
    }
  for (i = 0; i < n; i++)
    ftm_free (F[i]);
  for (f = 0; f < nf; f++)
    free (fingers[f]);
  return 0;
}
