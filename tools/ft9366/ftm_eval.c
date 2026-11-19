/*
 * ftm_eval - offline evaluation of ft9366_match.c on recorded sessions
 *
 *   ftm_eval <data-dir> [score-matrix-out.bin] [image-dump-dir]
 *
 * <data-dir>/<finger>/<session>/{bg_*.pgm,img_*.pgm}: 16-bit raw frames as
 * written by ft9366_collect.py. Images are preprocessed with the driver's
 * own code (ft9366_image.c), then all ordered pairs are scored.
 * FRR/FAR use a rotating split: for each start position, N consecutive
 * images of a finger are enrolled and the rest are probes.
 *
 * Build (from the repository root); add e.g. -DFTM_MIN_OVERLAP=0.3 or
 * -DFTM_MIN_INLIERS=5 to measure other matcher limits:
 *   D=libfprint/drivers/ft9366
 *   gcc -O2 -I$D tools/ft9366/ftm_eval.c $D/ft9366_image.c $D/ft9366_match.c -lm -o ftm_eval
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#include "ft9366_image.h"
#include "ft9366_match.h"

#include <dirent.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define RAW_W  FT9366_RAW_W
#define RAW_H  FT9366_RAW_H
#define MAXIMG 512
#define MAXBG  16

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

/* snprintf for paths: stop instead of silently truncating */
static void
path_fmt (char *buf, size_t size, const char *fmt, ...)
{
  va_list ap;
  int r;

  va_start (ap, fmt);
  r = vsnprintf (buf, size, fmt, ap);
  va_end (ap);
  if (r < 0 || (size_t) r >= size)
    {
      fprintf (stderr, "path too long: %s...\n", buf);
      exit (1);
    }
}

static int
read_raw (const char *path, uint8_t *buf, uint16_t *words)
{
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
  if (fread (buf, 1, RAW_W * RAW_H * 2, f) != RAW_W * RAW_H * 2)
    {
      fclose (f);
      return 0;
    }
  fclose (f);
  if (words)
    for (i = 0; i < RAW_W * RAW_H; i++)
      words[i] = (uint16_t) ((buf[2 * i] << 8) | buf[2 * i + 1]);
  return 1;
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
      path_fmt (path, sizeof path, "%s/%s", data, fingers[f]);
      ns = list_dir (path, NULL, sess, 64);
      for (s = 0; s < ns; s++)
        {
          static uint16_t bgf[MAXBG][RAW_W * RAW_H];
          static float work_a[FT9366_IMG_W * FT9366_IMG_H], work_b[FT9366_IMG_W * FT9366_IMG_H];
          static double work_integ[(FT9366_IMG_W + 1) * (FT9366_IMG_H + 1)];
          uint32_t bgm[MAXBG];
          float bg[RAW_W * RAW_H];
          uint8_t raw[RAW_W * RAW_H * 2];
          char sdir[1024];
          int nb, ni, k, p;

          path_fmt (sdir, sizeof sdir, "%s/%s/%s", data, fingers[f], sess[s]);
          nb = list_dir (sdir, "bg_", files, 256);
          if (nb > MAXBG || nb < 1)
            {
              fprintf (stderr, "%s: expected 1..%d background frames, found %d\n", sdir, MAXBG, nb);
              return 1;
            }
          for (k = 0; k < nb; k++)
            {
              uint64_t sum = 0;

              path_fmt (path, sizeof path, "%s/%s", sdir, files[k]);
              if (!read_raw (path, raw, bgf[k]))
                return 1;
              for (p = 0; p < RAW_W * RAW_H; p++)
                sum += bgf[k][p];
              bgm[k] = (uint32_t) (sum / (RAW_W * RAW_H));
              free (files[k]);
            }
          /* the driver's background, including its frame selection */
          ft9366_bg_average (&bgf[0][0], bgm, (unsigned) nb, bg);
          ni = list_dir (sdir, "img_", files, 256);
          for (k = 0; k < ni && n < MAXIMG; k++)
            {
              path_fmt (path, sizeof path, "%s/%s", sdir, files[k]);
              if (!read_raw (path, raw, NULL))
                return 1;
              ft9366_build_image (raw, bg, imgs[n], work_a, work_b, work_integ);
              if (dump)
                {
                  FILE *o;
                  path_fmt (path, sizeof path, "%s/%03d_%s.pgm", dump, n, fingers[f]);
                  o = fopen (path, "wb");
                  if (!o)
                    {
                      perror (path);
                      return 1;
                    }
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
  printf ("%d images, %d fingers\n", n, nf);
  if (n < 2 || nf < 1)
    {
      fprintf (stderr, "no usable images under %s\n", data);
      return 1;
    }

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

  printf ("keypoints/image: min %d, mean %.0f, max %d\n", kpmin, kpsum / n, kpmax);
  printf ("time: extraction %.1f ms/image, matching %.2f ms/pair\n",
          1000 * t_ext / n, 1000 * t_match / ((double) n * (n - 1)));
  printf ("highest impostor score: %.3f\n", imp_max);

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
          printf ("  enroll %2d images, threshold %.2f: FRR %5.1f%%  FAR %.2f%%\n",
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
            printf ("ERROR: deserialisation failed (%d)\n", i);
            return 1;
          }
        ftm_free (F[i]);
        F[i] = g;
      }
    for (i = 0; i < n; i++)
      for (j = 0; j < n; j++)
        if (i != j)
          maxdiff = fmax (maxdiff, fabs (ftm_score (F[i], F[j]) - C[i][j]));
    printf ("serialisation: mean %.1f KB/image, round-trip max deviation %.3g, "
            "corrupt inputs accepted: %d\n", total / 1024.0 / n, maxdiff, bad);
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
