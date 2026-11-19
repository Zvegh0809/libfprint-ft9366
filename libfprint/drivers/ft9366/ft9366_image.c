/*
 * FT9366 image preprocessing - see ft9366_image.h
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#include "ft9366_image.h"

#include <stdlib.h>
#include <string.h>

/* high-pass window, about one ridge period (~8.7 px) */
#define HP_BOX  9

#define IW      FT9366_IMG_W
#define IH      FT9366_IMG_H

#define CLAMP(v, lo, hi) ((v) < (lo) ? (lo) : ((v) > (hi) ? (hi) : (v)))
#define MIN(a, b)        ((a) < (b) ? (a) : (b))
#define MAX(a, b)        ((a) > (b) ? (a) : (b))
/* clamped pixel access */
#define AT(buf, w, h, x, y) \
  (buf)[CLAMP ((int) (y), 0, (h) - 1) * (w) + CLAMP ((int) (x), 0, (w) - 1)]

static int
cmp_float (const void *a, const void *b)
{
  float x = *(const float *) a, y = *(const float *) b;

  return (x > y) - (x < y);
}

unsigned
ft9366_bg_average (const uint16_t *frames, const uint32_t *means,
                   unsigned count, float *bg)
{
  const size_t npix = (size_t) FT9366_RAW_W * FT9366_RAW_H;
  uint32_t max_mean = 0;
  unsigned used = 0, k;
  size_t i;

  for (k = 0; k < count; k++)
    max_mean = MAX (max_mean, means[k]);
  for (k = 0; k < count; k++)
    used += means[k] + FT9366_BG_TOL >= max_mean;

  for (i = 0; i < npix; i++)
    {
      uint32_t s = 0;

      for (k = 0; k < count; k++)
        if (means[k] + FT9366_BG_TOL >= max_mean)
          s += frames[(size_t) k * npix + i];
      bg[i] = used ? (float) s / used : 0;
    }
  return used;
}

void
ft9366_build_image (const uint8_t *raw_be, const float *bg, uint8_t *out,
                    float *work_a, float *work_b, double *work_integ)
{
  float *a = work_a, *b = work_b;
  double *integ = work_integ;
  const size_t n = (size_t) IW * IH;
  const int r = HP_BOX / 2;
  float lo, hi;
  int x, y;

  /* 1. background subtraction + crop */
  for (y = 0; y < IH; y++)
    for (x = 0; x < IW; x++)
      {
        size_t ri = (size_t) (y + FT9366_IMG_Y0) * FT9366_RAW_W + x + FT9366_IMG_X0;
        float v = (raw_be[2 * ri] << 8) | raw_be[2 * ri + 1];

        if (bg)
          v -= bg[ri];
        a[y * IW + x] = v;
      }

  /* 2. 3x3 Gaussian blur */
  for (y = 0; y < IH; y++)
    for (x = 0; x < IW; x++)
      {
        float s =
          1 * AT (a, IW, IH, x - 1, y - 1) + 2 * AT (a, IW, IH, x, y - 1) + 1 * AT (a, IW, IH, x + 1, y - 1) +
          2 * AT (a, IW, IH, x - 1, y)     + 4 * AT (a, IW, IH, x, y)     + 2 * AT (a, IW, IH, x + 1, y) +
          1 * AT (a, IW, IH, x - 1, y + 1) + 2 * AT (a, IW, IH, x, y + 1) + 1 * AT (a, IW, IH, x + 1, y + 1);
        b[y * IW + x] = s / 16.0f;
      }

  /* 3. high-pass: subtract the HP_BOX box mean (integral image; edge
   *    windows only count in-image pixels) */
  for (y = 0; y <= IH; y++)
    integ[y * (IW + 1)] = 0;
  for (x = 0; x <= IW; x++)
    integ[x] = 0;
  for (y = 1; y <= IH; y++)
    for (x = 1; x <= IW; x++)
      integ[y * (IW + 1) + x] = b[(y - 1) * IW + (x - 1)]
                                + integ[(y - 1) * (IW + 1) + x]
                                + integ[y * (IW + 1) + x - 1]
                                - integ[(y - 1) * (IW + 1) + x - 1];
  for (y = 0; y < IH; y++)
    for (x = 0; x < IW; x++)
      {
        int x0 = MAX (x - r, 0), x1 = MIN (x + r + 1, IW);
        int y0 = MAX (y - r, 0), y1 = MIN (y + r + 1, IH);
        double s = integ[y1 * (IW + 1) + x1] - integ[y0 * (IW + 1) + x1]
                   - integ[y1 * (IW + 1) + x0] + integ[y0 * (IW + 1) + x0];

        a[y * IW + x] = b[y * IW + x] - (float) (s / ((x1 - x0) * (y1 - y0)));
      }

  /* 4. stretch the 2nd-98th percentile to 0..255 */
  memcpy (b, a, n * sizeof (float));
  qsort (b, n, sizeof (float), cmp_float);
  lo = b[n / 50];
  hi = b[n - 1 - n / 50];
  if (hi <= lo)
    hi = lo + 1;
  for (size_t i = 0; i < n; i++)
    out[i] = (uint8_t) CLAMP ((int) ((a[i] - lo) * 255.0f / (hi - lo)), 0, 255);

  explicit_bzero (a, n * sizeof (float));
  explicit_bzero (b, n * sizeof (float));
  explicit_bzero (integ, (size_t) (IW + 1) * (IH + 1) * sizeof (double));
}
