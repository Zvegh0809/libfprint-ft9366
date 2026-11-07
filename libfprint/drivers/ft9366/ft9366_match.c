/*
 * FT9366 host-side fingerprint matcher – see ft9366_match.h
 *
 * The SIFT part follows Lowe (IJCV 60(2):91-110, 2004) with the parameter
 * conventions of the widely used OpenCV implementation (3 layers per octave,
 * sigma 1.6, initial 2x upsampling, 4x4x8 descriptor), so that results can be
 * compared with the Python/OpenCV evaluation script (ft9366_evaluate.py).
 * Written from the paper; no code was copied from OpenCV or VLFeat.
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#include "ft9366_match.h"

#include <float.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

/* ------------------------------------------------------------------ */
/* Parameters (identical to ft9366_evaluate.py)                         */
/* ------------------------------------------------------------------ */

#define UPSCALE           3       /* cubic upscaling before SIFT        */
#define SIFT_LAYERS       3       /* scales per octave                  */
#define SIFT_SIGMA        1.6
#define SIFT_INIT_SIGMA   0.5
#define SIFT_CONTRAST     0.01
#define SIFT_EDGE         20.0
#define SIFT_BORDER       5
#define SIFT_MAX_INTERP   5
#define ORI_BINS          36
#define ORI_SIG_FCTR      1.5
#define ORI_RADIUS        (3 * ORI_SIG_FCTR)
#define ORI_PEAK_RATIO    0.8
#define DESC_W            4
#define DESC_BINS         8
#define DESC_LEN          (DESC_W * DESC_W * DESC_BINS)
#define DESC_SCL_FCTR     3.0
#define DESC_MAG_THR      0.2
#define DESC_INT_FCTR     512.0

#define RATIO             0.9     /* Lowe ratio test                    */
#define RANSAC_THR        2.0     /* px, in FTM_W x FTM_H coordinates   */
#define RANSAC_CONF       0.99
#define RANSAC_MAX_ITERS  2000
#define MIN_INLIERS       3
#define SCALE_MIN         0.9
#define SCALE_MAX         1.1
#define CONTENT_BOX       9
#define CONTENT_THR       0.25
#define MIN_OVERLAP       0.2

#define NPIX              (FTM_W * FTM_H)

typedef struct
{
  float x, y;                   /* in FTM_W x FTM_H image coordinates */
} Pt;

struct _FtmFeatures
{
  uint8_t  img[NPIX];           /* input image (kept for serialising) */
  float    e[NPIX];             /* z-normalised image                */
  uint8_t  content[NPIX];       /* ridge-bearing pixels              */
  int      n;                   /* keypoints                         */
  Pt      *pt;
  uint8_t *desc;                /* n * DESC_LEN                      */
};

/* ------------------------------------------------------------------ */
/* Small image helpers                                                  */
/* ------------------------------------------------------------------ */

typedef struct
{
  int    w, h;
  float *d;
} Img;

static int
img_alloc (Img *im, int w, int h)
{
  im->w = w;
  im->h = h;
  im->d = calloc ((size_t) w * h, sizeof (float));
  return im->d != NULL;
}

static void
img_free (Img *im)
{
  if (im->d)
    {
      memset (im->d, 0, (size_t) im->w * im->h * sizeof (float));
      free (im->d);
    }
  im->d = NULL;
}

#define PX(im, x, y) ((im)->d[(size_t) (y) * (im)->w + (x)])

/* reflect-101 border: ... 2 1 | 0 1 2 ... n-1 | n-2 n-3 ... */
static inline int
refl101 (int i, int n)
{
  if (n == 1)
    return 0;
  while (i < 0 || i >= n)
    {
      if (i < 0)
        i = -i;
      if (i >= n)
        i = 2 * n - 2 - i;
    }
  return i;
}

static inline int
clampi (int v, int lo, int hi)
{
  return v < lo ? lo : (v > hi ? hi : v);
}

/* separable Gaussian blur, reflect-101 border; dst may not alias src */
static int
gauss_blur (const Img *src, Img *dst, double sigma)
{
  int ks = ((int) lround (sigma * 8 + 1)) | 1;
  int r = ks / 2;
  float *k = malloc (ks * sizeof (float));
  float *tmp = malloc ((size_t) src->w * src->h * sizeof (float));
  double sum = 0;
  int x, y, i;

  if (!k || !tmp)
    {
      free (k);
      free (tmp);
      return 0;
    }

  for (i = 0; i < ks; i++)
    {
      double t = i - r;
      sum += (k[i] = (float) exp (-t * t / (2 * sigma * sigma)));
    }
  for (i = 0; i < ks; i++)
    k[i] = (float) (k[i] / sum);

  {
    /* horizontal pass on a reflect-padded copy of each row */
    float *pad = malloc ((size_t) (src->w + 2 * r) * sizeof (float));

    if (!pad)
      {
        free (k);
        free (tmp);
        return 0;
      }
    for (y = 0; y < src->h; y++)
      {
        const float *row = src->d + (size_t) y * src->w;
        float *out = tmp + (size_t) y * src->w;

        for (x = -r; x < src->w + r; x++)
          pad[x + r] = row[refl101 (x, src->w)];
        for (x = 0; x < src->w; x++)
          {
            const float *p = pad + x;
            float s = 0;
            for (i = 0; i < ks; i++)
              s += k[i] * p[i];
            out[x] = s;
          }
      }
    memset (pad, 0, (size_t) (src->w + 2 * r) * sizeof (float));
    free (pad);
  }

  /* vertical pass: accumulate whole rows (cache friendly) */
  for (y = 0; y < src->h; y++)
    {
      float *out = dst->d + (size_t) y * dst->w;

      memset (out, 0, (size_t) dst->w * sizeof (float));
      for (i = 0; i < ks; i++)
        {
          const float *row = tmp + (size_t) refl101 (y - r + i, src->h) * src->w;
          const float ki = k[i];
          for (x = 0; x < src->w; x++)
            out[x] += ki * row[x];
        }
    }

  memset (tmp, 0, (size_t) src->w * src->h * sizeof (float));
  free (tmp);
  free (k);
  return 1;
}

/* cubic convolution kernel, a = -0.75 */
static void
cubic_w (double t, double w[4])
{
  const double A = -0.75;

  w[0] = ((A * (t + 1) - 5 * A) * (t + 1) + 8 * A) * (t + 1) - 4 * A;
  w[1] = ((A + 2) * t - (A + 3)) * t * t + 1;
  w[2] = ((A + 2) * (1 - t) - (A + 3)) * (1 - t) * (1 - t) + 1;
  w[3] = 1 - w[0] - w[1] - w[2];
}

/* 8-bit image -> UPSCALE x bicubic, rounded and saturated to 0..255 */
static void
upscale_cubic (const uint8_t *src, int w, int h, Img *dst)
{
  int x, y, i, j;

  for (y = 0; y < dst->h; y++)
    {
      double fy = (y + 0.5) / UPSCALE - 0.5;
      int sy = (int) floor (fy);
      double wy[4];

      cubic_w (fy - sy, wy);
      for (x = 0; x < dst->w; x++)
        {
          double fx = (x + 0.5) / UPSCALE - 0.5;
          int sx = (int) floor (fx);
          double wx[4], s = 0;

          cubic_w (fx - sx, wx);
          for (j = 0; j < 4; j++)
            {
              const uint8_t *row = src + (size_t) clampi (sy - 1 + j, 0, h - 1) * w;
              double rs = 0;
              for (i = 0; i < 4; i++)
                rs += wx[i] * row[clampi (sx - 1 + i, 0, w - 1)];
              s += wy[j] * rs;
            }
          PX (dst, x, y) = (float) clampi ((int) lround (s), 0, 255);
        }
    }
}

/* 2x bilinear upsampling (pixel-centre aligned) */
static void
upsample2 (const Img *src, Img *dst)
{
  int x, y;

  for (y = 0; y < dst->h; y++)
    {
      double fy = (y + 0.5) * 0.5 - 0.5;
      int y0 = (int) floor (fy);
      double ty = fy - y0;
      int ya = clampi (y0, 0, src->h - 1), yb = clampi (y0 + 1, 0, src->h - 1);

      for (x = 0; x < dst->w; x++)
        {
          double fx = (x + 0.5) * 0.5 - 0.5;
          int x0 = (int) floor (fx);
          double tx = fx - x0;
          int xa = clampi (x0, 0, src->w - 1), xb = clampi (x0 + 1, 0, src->w - 1);

          PX (dst, x, y) = (float) ((1 - ty) * ((1 - tx) * PX (src, xa, ya) + tx * PX (src, xb, ya)) +
                                    ty * ((1 - tx) * PX (src, xa, yb) + tx * PX (src, xb, yb)));
        }
    }
}

/* nearest-neighbour halving */
static void
downsample2 (const Img *src, Img *dst)
{
  double sx = (double) src->w / dst->w, sy = (double) src->h / dst->h;
  int x, y;

  for (y = 0; y < dst->h; y++)
    for (x = 0; x < dst->w; x++)
      PX (dst, x, y) = PX (src, clampi ((int) floor (x * sx), 0, src->w - 1),
                                clampi ((int) floor (y * sy), 0, src->h - 1));
}

/* ------------------------------------------------------------------ */
/* SIFT                                                                 */
/* ------------------------------------------------------------------ */

#define N_GAUSS (SIFT_LAYERS + 3)
#define N_DOG   (SIFT_LAYERS + 2)

typedef struct
{
  int   octaves;
  Img  *g;                      /* octaves * N_GAUSS */
  Img  *dog;                    /* octaves * N_DOG   */
} Pyr;

typedef struct
{
  int   o, layer;               /* pyramid octave and integer layer  */
  float x, y;                   /* sub-pixel position in octave px   */
  float sigma;                  /* scale in octave px                */
  float angle;                  /* degrees, Lowe/OpenCV convention   */
  float resp;
} Kp;

typedef struct
{
  Kp  *v;
  int  n, cap;
} KpList;

static int
kp_push (KpList *l, const Kp *k)
{
  if (l->n == l->cap)
    {
      int cap = l->cap ? l->cap * 2 : 256;
      Kp *v = realloc (l->v, cap * sizeof (Kp));
      if (!v)
        return 0;
      l->v = v;
      l->cap = cap;
    }
  l->v[l->n++] = *k;
  return 1;
}

static void
pyr_free (Pyr *p)
{
  int i;

  if (p->g)
    for (i = 0; i < p->octaves * N_GAUSS; i++)
      img_free (&p->g[i]);
  if (p->dog)
    for (i = 0; i < p->octaves * N_DOG; i++)
      img_free (&p->dog[i]);
  free (p->g);
  free (p->dog);
  memset (p, 0, sizeof (*p));
}

static int
pyr_build (const Img *input, Pyr *p)
{
  double sig[N_GAUSS], k = pow (2.0, 1.0 / SIFT_LAYERS);
  Img base = { 0 };
  int o, i, ok = 1;

  memset (p, 0, sizeof (*p));

  /* base: 2x upsampled, blurred to SIFT_SIGMA (assuming 0.5 blur in input) */
  if (!img_alloc (&base, input->w * 2, input->h * 2))
    return 0;
  upsample2 (input, &base);

  p->octaves = (int) lround (log ((double) (base.w < base.h ? base.w : base.h)) / log (2.0) - 2) + 1;
  if (p->octaves < 1)
    p->octaves = 1;
  p->g = calloc (p->octaves * N_GAUSS, sizeof (Img));
  p->dog = calloc (p->octaves * N_DOG, sizeof (Img));
  if (!p->g || !p->dog)
    goto fail;

  sig[0] = SIFT_SIGMA;
  for (i = 1; i < N_GAUSS; i++)
    {
      double prev = pow (k, i - 1) * SIFT_SIGMA, total = prev * k;
      sig[i] = sqrt (total * total - prev * prev);
    }

  for (o = 0; o < p->octaves; o++)
    for (i = 0; i < N_GAUSS; i++)
      {
        Img *dst = &p->g[o * N_GAUSS + i];

        if (o == 0 && i == 0)
          {
            double d = SIFT_SIGMA * SIFT_SIGMA - 4 * SIFT_INIT_SIGMA * SIFT_INIT_SIGMA;
            if (!img_alloc (dst, base.w, base.h) ||
                !gauss_blur (&base, dst, sqrt (d > 0.01 ? d : 0.01)))
              goto fail;
          }
        else if (i == 0)
          {
            const Img *src = &p->g[(o - 1) * N_GAUSS + SIFT_LAYERS];
            if (!img_alloc (dst, src->w / 2, src->h / 2))
              goto fail;
            downsample2 (src, dst);
          }
        else
          {
            const Img *src = &p->g[o * N_GAUSS + i - 1];
            if (!img_alloc (dst, src->w, src->h) || !gauss_blur (src, dst, sig[i]))
              goto fail;
          }
      }

  for (o = 0; o < p->octaves; o++)
    for (i = 0; i < N_DOG; i++)
      {
        const Img *a = &p->g[o * N_GAUSS + i], *b = &p->g[o * N_GAUSS + i + 1];
        Img *d = &p->dog[o * N_DOG + i];
        size_t j;

        if (!img_alloc (d, a->w, a->h))
          goto fail;
        for (j = 0; j < (size_t) a->w * a->h; j++)
          d->d[j] = b->d[j] - a->d[j];
      }

  img_free (&base);
  return ok;

fail:
  img_free (&base);
  pyr_free (p);
  return 0;
}

/* solve 3x3 system H x = b (Gaussian elimination with partial pivoting) */
static int
solve3 (double H[3][3], const double b[3], double x[3])
{
  double A[3][4];
  int i, j, c;

  for (i = 0; i < 3; i++)
    {
      for (j = 0; j < 3; j++)
        A[i][j] = H[i][j];
      A[i][3] = b[i];
    }
  for (c = 0; c < 3; c++)
    {
      int piv = c;
      for (i = c + 1; i < 3; i++)
        if (fabs (A[i][c]) > fabs (A[piv][c]))
          piv = i;
      if (fabs (A[piv][c]) < DBL_EPSILON)
        return 0;
      if (piv != c)
        for (j = 0; j < 4; j++)
          {
            double t = A[c][j];
            A[c][j] = A[piv][j];
            A[piv][j] = t;
          }
      for (i = c + 1; i < 3; i++)
        {
          double f = A[i][c] / A[c][c];
          for (j = c; j < 4; j++)
            A[i][j] -= f * A[c][j];
        }
    }
  for (i = 2; i >= 0; i--)
    {
      double s = A[i][3];
      for (j = i + 1; j < 3; j++)
        s -= A[i][j] * x[j];
      x[i] = s / A[i][i];
    }
  return 1;
}

/* first and second derivatives of the DoG at (layer, r, c), image values
 * scaled to 0..1 */
static void
dog_derivs (const Pyr *p, int o, int layer, int r, int c,
            double dD[3], double H[3][3])
{
  const double s = 1.0 / 255, s1 = s * 0.5, s2 = s, s11 = s * 0.25;
  const Img *prv = &p->dog[o * N_DOG + layer - 1];
  const Img *cur = &p->dog[o * N_DOG + layer];
  const Img *nxt = &p->dog[o * N_DOG + layer + 1];
  double v2 = 2.0 * PX (cur, c, r);
  double dxx, dyy, dss, dxy, dxs, dys;

  dD[0] = (PX (cur, c + 1, r) - PX (cur, c - 1, r)) * s1;
  dD[1] = (PX (cur, c, r + 1) - PX (cur, c, r - 1)) * s1;
  dD[2] = (PX (nxt, c, r) - PX (prv, c, r)) * s1;

  dxx = (PX (cur, c + 1, r) + PX (cur, c - 1, r) - v2) * s2;
  dyy = (PX (cur, c, r + 1) + PX (cur, c, r - 1) - v2) * s2;
  dss = (PX (nxt, c, r) + PX (prv, c, r) - v2) * s2;
  dxy = (PX (cur, c + 1, r + 1) - PX (cur, c - 1, r + 1) -
         PX (cur, c + 1, r - 1) + PX (cur, c - 1, r - 1)) * s11;
  dxs = (PX (nxt, c + 1, r) - PX (nxt, c - 1, r) -
         PX (prv, c + 1, r) + PX (prv, c - 1, r)) * s11;
  dys = (PX (nxt, c, r + 1) - PX (nxt, c, r - 1) -
         PX (prv, c, r + 1) + PX (prv, c, r - 1)) * s11;

  H[0][0] = dxx; H[0][1] = dxy; H[0][2] = dxs;
  H[1][0] = dxy; H[1][1] = dyy; H[1][2] = dys;
  H[2][0] = dxs; H[2][1] = dys; H[2][2] = dss;
}

/* sub-pixel refinement + contrast and edge rejection (Lowe 2004, sec. 4) */
static int
refine_extremum (const Pyr *p, int o, int *layer, int *r, int *c, Kp *kp)
{
  const Img *im = &p->dog[o * N_DOG];
  double dD[3], H[3][3], X[3] = { 0 }, xc = 0, xr = 0, xi = 0;
  int it;

  for (it = 0; it < SIFT_MAX_INTERP; it++)
    {
      dog_derivs (p, o, *layer, *r, *c, dD, H);
      if (!solve3 (H, dD, X))
        return 0;
      xc = -X[0];
      xr = -X[1];
      xi = -X[2];

      if (fabs (xi) < 0.5 && fabs (xr) < 0.5 && fabs (xc) < 0.5)
        break;
      if (fabs (xi) > INT32_MAX / 3 || fabs (xr) > INT32_MAX / 3 || fabs (xc) > INT32_MAX / 3)
        return 0;

      *c += (int) lround (xc);
      *r += (int) lround (xr);
      *layer += (int) lround (xi);
      if (*layer < 1 || *layer > SIFT_LAYERS ||
          *c < SIFT_BORDER || *c >= im->w - SIFT_BORDER ||
          *r < SIFT_BORDER || *r >= im->h - SIFT_BORDER)
        return 0;
    }
  if (it >= SIFT_MAX_INTERP)
    return 0;

  {
    const Img *cur = &p->dog[o * N_DOG + *layer];
    double t, contr, tr, det;

    dog_derivs (p, o, *layer, *r, *c, dD, H);
    t = dD[0] * xc + dD[1] * xr + dD[2] * xi;
    contr = PX (cur, *c, *r) / 255.0 + t * 0.5;
    if (fabs (contr) * SIFT_LAYERS < SIFT_CONTRAST)
      return 0;

    tr = H[0][0] + H[1][1];
    det = H[0][0] * H[1][1] - H[0][1] * H[0][1];
    if (det <= 0 || tr * tr * SIFT_EDGE >= (SIFT_EDGE + 1) * (SIFT_EDGE + 1) * det)
      return 0;

    kp->o = o;
    kp->layer = *layer;
    kp->x = (float) (*c + xc);
    kp->y = (float) (*r + xr);
    kp->sigma = (float) (SIFT_SIGMA * pow (2.0, (*layer + xi) / SIFT_LAYERS));
    kp->resp = (float) fabs (contr);
  }
  return 1;
}

static double
deg_atan2 (double y, double x)
{
  double a = atan2 (y, x) * (180.0 / M_PI);
  return a < 0 ? a + 360.0 : a;
}

/* dominant orientations (Lowe 2004, sec. 5); emits one keypoint each */
static int
add_oriented (const Pyr *p, const Kp *kp, KpList *out)
{
  const Img *im = &p->g[kp->o * N_GAUSS + kp->layer];
  double sigma = ORI_SIG_FCTR * kp->sigma;
  int radius = (int) lround (ORI_RADIUS * kp->sigma);
  int px = (int) lround (kp->x), py = (int) lround (kp->y);
  double raw[ORI_BINS] = { 0 }, hist[ORI_BINS], omax = 0;
  int i, j;

  for (i = -radius; i <= radius; i++)
    {
      int y = py + i;
      if (y <= 0 || y >= im->h - 1)
        continue;
      for (j = -radius; j <= radius; j++)
        {
          int x = px + j, bin;
          double dx, dy, w;

          if (x <= 0 || x >= im->w - 1)
            continue;
          dx = PX (im, x + 1, y) - PX (im, x - 1, y);
          dy = PX (im, x, y - 1) - PX (im, x, y + 1);
          w = exp (-(i * i + j * j) / (2 * sigma * sigma));
          bin = (int) lround (deg_atan2 (dy, dx) * ORI_BINS / 360.0);
          if (bin >= ORI_BINS)
            bin -= ORI_BINS;
          if (bin < 0)
            bin += ORI_BINS;
          raw[bin] += w * sqrt (dx * dx + dy * dy);
        }
    }

  for (i = 0; i < ORI_BINS; i++)
    {
      int m2 = (i + ORI_BINS - 2) % ORI_BINS, m1 = (i + ORI_BINS - 1) % ORI_BINS;
      int p1 = (i + 1) % ORI_BINS, p2 = (i + 2) % ORI_BINS;
      hist[i] = (raw[m2] + raw[p2]) * (1.0 / 16) + (raw[m1] + raw[p1]) * (4.0 / 16) + raw[i] * (6.0 / 16);
      if (hist[i] > omax)
        omax = hist[i];
    }

  for (j = 0; j < ORI_BINS; j++)
    {
      int l = j > 0 ? j - 1 : ORI_BINS - 1, r = j < ORI_BINS - 1 ? j + 1 : 0;

      if (hist[j] > hist[l] && hist[j] > hist[r] && hist[j] >= omax * ORI_PEAK_RATIO)
        {
          double bin = j + 0.5 * (hist[l] - hist[r]) / (hist[l] - 2 * hist[j] + hist[r]);
          Kp k = *kp;

          bin = bin < 0 ? ORI_BINS + bin : (bin >= ORI_BINS ? bin - ORI_BINS : bin);
          k.angle = (float) (360.0 - (360.0 / ORI_BINS) * bin);
          if (fabs (k.angle - 360.0f) < FLT_EPSILON)
            k.angle = 0;
          if (!kp_push (out, &k))
            return 0;
        }
    }
  return 1;
}

static int
find_keypoints (const Pyr *p, KpList *out)
{
  int o, i, r, c;

  for (o = 0; o < p->octaves; o++)
    for (i = 1; i <= SIFT_LAYERS; i++)
      {
        const Img *prv = &p->dog[o * N_DOG + i - 1];
        const Img *cur = &p->dog[o * N_DOG + i];
        const Img *nxt = &p->dog[o * N_DOG + i + 1];

        for (r = SIFT_BORDER; r < cur->h - SIFT_BORDER; r++)
          for (c = SIFT_BORDER; c < cur->w - SIFT_BORDER; c++)
            {
              float v = PX (cur, c, r);
              int dy, dx, is_max = v > 0, is_min = v < 0;

              /* contrast pre-threshold floor(0.5*0.01/3*255) == 0 */
              if (v == 0)
                continue;
              for (dy = -1; dy <= 1 && (is_max || is_min); dy++)
                for (dx = -1; dx <= 1; dx++)
                  {
                    float a = PX (prv, c + dx, r + dy), b = PX (nxt, c + dx, r + dy);
                    float m = PX (cur, c + dx, r + dy);

                    if (is_max && (v < a || v < b || (v < m)))
                      is_max = 0;
                    if (is_min && (v > a || v > b || (v > m)))
                      is_min = 0;
                  }
              if (is_max || is_min)
                {
                  int layer = i, rr = r, cc = c;
                  Kp kp;

                  if (refine_extremum (p, o, &layer, &rr, &cc, &kp) &&
                      !add_oriented (p, &kp, out))
                    return 0;
                }
            }
      }
  return 1;
}

/* final position in FTM image coordinates (base is 2x of the UPSCALE image) */
static void
kp_image_pos (const Kp *k, float *x, float *y)
{
  float f = (float) (1 << k->o) * 0.5f / UPSCALE;

  *x = k->x * f;
  *y = k->y * f;
}

static int
kp_cmp (const void *pa, const void *pb)
{
  const Kp *a = pa, *b = pb;
  float ax, ay, bx, by;

  kp_image_pos (a, &ax, &ay);
  kp_image_pos (b, &bx, &by);
  if (ax != bx)
    return ax < bx ? -1 : 1;
  if (ay != by)
    return ay < by ? -1 : 1;
  if (a->sigma * (1 << a->o) != b->sigma * (1 << b->o))
    return a->sigma * (1 << a->o) > b->sigma * (1 << b->o) ? -1 : 1;
  if (a->angle != b->angle)
    return a->angle < b->angle ? -1 : 1;
  return 0;
}

/* 4x4x8 gradient histogram descriptor (Lowe 2004, sec. 6) */
static void
describe (const Pyr *p, const Kp *kp, uint8_t out[DESC_LEN])
{
  const Img *im = &p->g[kp->o * N_GAUSS + kp->layer];
  const int d = DESC_W, n = DESC_BINS;
  double hist[(DESC_W + 2) * (DESC_W + 2) * (DESC_BINS + 2)] = { 0 };
  float dst[DESC_LEN];
  double ori = 360.0 - kp->angle, scl = kp->sigma;
  double cos_t, sin_t, bins_per_deg = n / 360.0, exp_scale = -1.0 / (d * d * 0.5);
  double hist_w = DESC_SCL_FCTR * scl, nrm2 = 0, thr;
  int px = (int) lround (kp->x), py = (int) lround (kp->y);
  int radius, i, j, k;

  if (fabs (ori - 360.0) < FLT_EPSILON)
    ori = 0;
  cos_t = cos (ori * M_PI / 180.0) / hist_w;
  sin_t = sin (ori * M_PI / 180.0) / hist_w;
  radius = (int) lround (hist_w * M_SQRT2 * (d + 1) * 0.5);
  {
    int lim = (int) sqrt ((double) im->w * im->w + (double) im->h * im->h);
    if (radius > lim)
      radius = lim;
  }

  for (i = -radius; i <= radius; i++)
    for (j = -radius; j <= radius; j++)
      {
        double c_rot = j * cos_t - i * sin_t, r_rot = j * sin_t + i * cos_t;
        double rbin = r_rot + d / 2 - 0.5, cbin = c_rot + d / 2 - 0.5;
        int r = py + i, c = px + j;

        if (rbin > -1 && rbin < d && cbin > -1 && cbin < d &&
            r > 0 && r < im->h - 1 && c > 0 && c < im->w - 1)
          {
            double dx = PX (im, c + 1, r) - PX (im, c - 1, r);
            double dy = PX (im, c, r - 1) - PX (im, c, r + 1);
            double mag = sqrt (dx * dx + dy * dy) * exp ((c_rot * c_rot + r_rot * r_rot) * exp_scale);
            double obin = (deg_atan2 (dy, dx) - ori) * bins_per_deg;
            int r0 = (int) floor (rbin), c0 = (int) floor (cbin), o0 = (int) floor (obin);
            double fr = rbin - r0, fc = cbin - c0, fo = obin - o0;
            double v_r1, v_r0, v_rc11, v_rc10, v_rc01, v_rc00;
            int idx;

            if (o0 < 0)
              o0 += n;
            if (o0 >= n)
              o0 -= n;

            v_r1 = mag * fr;
            v_r0 = mag - v_r1;
            v_rc11 = v_r1 * fc;
            v_rc10 = v_r1 - v_rc11;
            v_rc01 = v_r0 * fc;
            v_rc00 = v_r0 - v_rc01;

            idx = ((r0 + 1) * (d + 2) + c0 + 1) * (n + 2) + o0;
            hist[idx]                           += v_rc00 * (1 - fo);
            hist[idx + 1]                       += v_rc00 * fo;
            hist[idx + (n + 2)]                 += v_rc01 * (1 - fo);
            hist[idx + (n + 3)]                 += v_rc01 * fo;
            hist[idx + (d + 2) * (n + 2)]       += v_rc10 * (1 - fo);
            hist[idx + (d + 2) * (n + 2) + 1]   += v_rc10 * fo;
            hist[idx + (d + 3) * (n + 2)]       += v_rc11 * (1 - fo);
            hist[idx + (d + 3) * (n + 2) + 1]   += v_rc11 * fo;
          }
      }

  for (i = 0; i < d; i++)
    for (j = 0; j < d; j++)
      {
        int idx = ((i + 1) * (d + 2) + (j + 1)) * (n + 2);
        hist[idx] += hist[idx + n];
        hist[idx + 1] += hist[idx + n + 1];
        for (k = 0; k < n; k++)
          dst[(i * d + j) * n + k] = (float) hist[idx + k];
      }

  for (k = 0; k < DESC_LEN; k++)
    nrm2 += (double) dst[k] * dst[k];
  thr = sqrt (nrm2) * DESC_MAG_THR;
  nrm2 = 0;
  for (k = 0; k < DESC_LEN; k++)
    {
      if (dst[k] > thr)
        dst[k] = (float) thr;
      nrm2 += (double) dst[k] * dst[k];
    }
  nrm2 = DESC_INT_FCTR / fmax (sqrt (nrm2), FLT_EPSILON);
  for (k = 0; k < DESC_LEN; k++)
    out[k] = (uint8_t) clampi ((int) lround (dst[k] * nrm2), 0, 255);
}

/* ------------------------------------------------------------------ */
/* Image normalisation and ridge mask                                   */
/* ------------------------------------------------------------------ */

/* 9x9 mean of src (reflect-101 border) > thr */
static void
content_mask (const float *src, uint8_t *mask)
{
  const int r = CONTENT_BOX / 2;
  float tmp[NPIX];
  int x, y, i;

  for (y = 0; y < FTM_H; y++)
    for (x = 0; x < FTM_W; x++)
      {
        float s = 0;
        for (i = -r; i <= r; i++)
          s += src[y * FTM_W + refl101 (x + i, FTM_W)];
        tmp[y * FTM_W + x] = s;
      }
  for (y = 0; y < FTM_H; y++)
    for (x = 0; x < FTM_W; x++)
      {
        float s = 0;
        for (i = -r; i <= r; i++)
          s += tmp[refl101 (y + i, FTM_H) * FTM_W + x];
        mask[y * FTM_W + x] = s / (CONTENT_BOX * CONTENT_BOX) > CONTENT_THR;
      }
  memset (tmp, 0, sizeof (tmp));
}

/* ------------------------------------------------------------------ */
/* Public: feature extraction                                           */
/* ------------------------------------------------------------------ */

/* image -> f->img, f->e, f->content; optionally the SIFT input in u8 */
static void
prepare_image (FtmFeatures *f, const uint8_t *img, uint8_t *u8)
{
  float sq[NPIX];
  double mean = 0, var = 0, sd;
  int i;

  memcpy (f->img, img, NPIX);
  for (i = 0; i < NPIX; i++)
    mean += img[i];
  mean /= NPIX;
  for (i = 0; i < NPIX; i++)
    var += (img[i] - mean) * (img[i] - mean);
  sd = sqrt (var / NPIX) + 1e-6;
  for (i = 0; i < NPIX; i++)
    {
      f->e[i] = (float) ((img[i] - mean) / sd);
      sq[i] = f->e[i] * f->e[i];
      /* SIFT input: +-3 sd mapped to 0..255 */
      if (u8)
        u8[i] = (uint8_t) clampi ((int) ((f->e[i] + 3.0f) / 6.0f * 255.0f), 0, 255);
    }
  content_mask (sq, f->content);
  memset (sq, 0, sizeof (sq));
}

FtmFeatures *
ftm_extract (const uint8_t *img)
{
  FtmFeatures *f = calloc (1, sizeof (FtmFeatures));
  uint8_t u8[NPIX];
  Img up = { 0 };
  Pyr pyr = { 0 };
  KpList kl = { 0 };
  int i, m;

  if (!f)
    return NULL;

  prepare_image (f, img, u8);

  if (!img_alloc (&up, FTM_W * UPSCALE, FTM_H * UPSCALE))
    goto fail;
  upscale_cubic (u8, FTM_W, FTM_H, &up);
  if (!pyr_build (&up, &pyr))
    goto fail;
  img_free (&up);

  if (!find_keypoints (&pyr, &kl))
    goto fail;

  /* sort and drop exact duplicates */
  if (kl.n > 1)
    {
      qsort (kl.v, kl.n, sizeof (Kp), kp_cmp);
      for (i = 0, m = 1; m < kl.n; m++)
        if (kp_cmp (&kl.v[i], &kl.v[m]) != 0)
          kl.v[++i] = kl.v[m];
      kl.n = i + 1;
    }

  f->n = kl.n;
  f->pt = calloc (kl.n ? kl.n : 1, sizeof (Pt));
  f->desc = calloc (kl.n ? (size_t) kl.n * DESC_LEN : 1, 1);
  if (!f->pt || !f->desc)
    goto fail;
  for (i = 0; i < kl.n; i++)
    {
      kp_image_pos (&kl.v[i], &f->pt[i].x, &f->pt[i].y);
      describe (&pyr, &kl.v[i], f->desc + (size_t) i * DESC_LEN);
    }

  pyr_free (&pyr);
  free (kl.v);
  memset (u8, 0, sizeof (u8));
  return f;

fail:
  img_free (&up);
  pyr_free (&pyr);
  free (kl.v);
  memset (u8, 0, sizeof (u8));
  ftm_free (f);
  return NULL;
}

double
ftm_coverage (const FtmFeatures *f)
{
  int i, c = 0;

  if (!f)
    return 0;
  for (i = 0; i < NPIX; i++)
    c += f->content[i];
  return (double) c / NPIX;
}

/* ------------------------------------------------------------------ */
/* Serialisation                                                        */
/* ------------------------------------------------------------------ */

#define SER_VERSION   1
#define SER_HDR       (1 + NPIX + 2)
#define SER_PER_KP    (8 + DESC_LEN)
#define SER_MAX_KP    4096

size_t
ftm_serialized_size (const FtmFeatures *f)
{
  return SER_HDR + (size_t) f->n * SER_PER_KP;
}

static void
put_f32 (uint8_t *p, float v)
{
  uint32_t u;

  memcpy (&u, &v, 4);
  p[0] = u & 0xff;
  p[1] = (u >> 8) & 0xff;
  p[2] = (u >> 16) & 0xff;
  p[3] = (u >> 24) & 0xff;
}

static float
get_f32 (const uint8_t *p)
{
  uint32_t u = (uint32_t) p[0] | ((uint32_t) p[1] << 8) |
               ((uint32_t) p[2] << 16) | ((uint32_t) p[3] << 24);
  float v;

  memcpy (&v, &u, 4);
  return v;
}

void
ftm_serialize (const FtmFeatures *f, uint8_t *buf)
{
  uint8_t *p = buf;
  int i;

  *p++ = SER_VERSION;
  memcpy (p, f->img, NPIX);
  p += NPIX;
  *p++ = f->n & 0xff;
  *p++ = (f->n >> 8) & 0xff;
  for (i = 0; i < f->n; i++, p += 8)
    {
      put_f32 (p, f->pt[i].x);
      put_f32 (p + 4, f->pt[i].y);
    }
  memcpy (p, f->desc, (size_t) f->n * DESC_LEN);
}

FtmFeatures *
ftm_deserialize (const uint8_t *buf, size_t len)
{
  FtmFeatures *f;
  const uint8_t *p = buf;
  int n, i;

  if (!buf || len < SER_HDR || buf[0] != SER_VERSION)
    return NULL;
  n = buf[1 + NPIX] | (buf[2 + NPIX] << 8);
  if (n > SER_MAX_KP || len != SER_HDR + (size_t) n * SER_PER_KP)
    return NULL;

  f = calloc (1, sizeof (FtmFeatures));
  if (!f)
    return NULL;
  f->n = n;
  f->pt = calloc (n ? n : 1, sizeof (Pt));
  f->desc = calloc (n ? (size_t) n * DESC_LEN : 1, 1);
  if (!f->pt || !f->desc)
    {
      ftm_free (f);
      return NULL;
    }

  prepare_image (f, p + 1, NULL);
  p += SER_HDR;
  for (i = 0; i < n; i++, p += 8)
    {
      f->pt[i].x = get_f32 (p);
      f->pt[i].y = get_f32 (p + 4);
      if (!isfinite (f->pt[i].x) || !isfinite (f->pt[i].y) ||
          f->pt[i].x < -1 || f->pt[i].x > FTM_W + 1 ||
          f->pt[i].y < -1 || f->pt[i].y > FTM_H + 1)
        {
          ftm_free (f);
          return NULL;
        }
    }
  memcpy (f->desc, p, (size_t) n * DESC_LEN);
  return f;
}

void
ftm_free (FtmFeatures *f)
{
  if (!f)
    return;
  if (f->pt)
    {
      memset (f->pt, 0, (size_t) (f->n ? f->n : 1) * sizeof (Pt));
      free (f->pt);
    }
  if (f->desc)
    {
      memset (f->desc, 0, f->n ? (size_t) f->n * DESC_LEN : 1);
      free (f->desc);
    }
  memset (f, 0, sizeof (*f));
  free (f);
}

int
ftm_n_keypoints (const FtmFeatures *f)
{
  return f ? f->n : 0;
}

/* ------------------------------------------------------------------ */
/* Matching: ratio test + RANSAC similarity + aligned NCC               */
/* ------------------------------------------------------------------ */

/* deterministic multiply-with-carry PRNG */
static uint32_t
rng_next (uint64_t *s)
{
  *s = (uint64_t) (uint32_t) *s * 4164903690u + (uint32_t) (*s >> 32);
  return (uint32_t) *s;
}

/* similarity M = [[a, -b, tx], [b, a, ty]] */
typedef struct
{
  double a, b, tx, ty;
} Sim;

static int
sim_from2 (const Pt *A, const Pt *B, Sim *m)
{
  double dx = A[0].x - A[1].x, dy = A[0].y - A[1].y;
  double DX = B[0].x - B[1].x, DY = B[0].y - B[1].y;
  double d = dx * dx + dy * dy;

  if (d < 1e-12)
    return 0;
  m->a = (dx * DX + dy * DY) / d;
  m->b = (dx * DY - dy * DX) / d;
  m->tx = (B[0].x + B[1].x - m->a * (A[0].x + A[1].x) + m->b * (A[0].y + A[1].y)) * 0.5;
  m->ty = (B[0].y + B[1].y - m->b * (A[0].x + A[1].x) - m->a * (A[0].y + A[1].y)) * 0.5;
  return 1;
}

static double
sim_err2 (const Sim *m, Pt a, Pt b)
{
  double ex = m->a * a.x - m->b * a.y + m->tx - b.x;
  double ey = m->b * a.x + m->a * a.y + m->ty - b.y;

  return ex * ex + ey * ey;
}

/* least-squares similarity over the inliers */
static int
sim_refit (const Pt *A, const Pt *B, const uint8_t *in, int n, Sim *m)
{
  double ax = 0, ay = 0, bx = 0, by = 0, sxx = 0, num_a = 0, num_b = 0;
  int i, k = 0;

  for (i = 0; i < n; i++)
    if (in[i])
      {
        ax += A[i].x; ay += A[i].y; bx += B[i].x; by += B[i].y;
        k++;
      }
  if (k < 2)
    return 0;
  ax /= k; ay /= k; bx /= k; by /= k;
  for (i = 0; i < n; i++)
    if (in[i])
      {
        double xa = A[i].x - ax, ya = A[i].y - ay, xb = B[i].x - bx, yb = B[i].y - by;
        sxx += xa * xa + ya * ya;
        num_a += xa * xb + ya * yb;
        num_b += xa * yb - ya * xb;
      }
  if (sxx < 1e-12)
    return 0;
  m->a = num_a / sxx;
  m->b = num_b / sxx;
  m->tx = bx - (m->a * ax - m->b * ay);
  m->ty = by - (m->b * ax + m->a * ay);
  return 1;
}

static int
ransac_update_iters (double conf, double outlier_ratio, int max_iters)
{
  double num = log (fmax (1 - conf, DBL_MIN));
  double denom = 1 - pow (1 - outlier_ratio, 2);

  if (denom < DBL_MIN)
    return 0;
  denom = log (denom);
  if (denom >= 0 || -num >= max_iters * (-denom))
    return max_iters;
  return (int) lround (num / denom);
}

/* RANSAC similarity from A[i] -> B[i]; returns number of inliers */
static int
ransac_sim (const Pt *A, const Pt *B, int n, Sim *best, uint8_t *mask)
{
  uint8_t *cur = malloc (n);
  uint64_t rng = (uint64_t) -1;
  int iters = RANSAC_MAX_ITERS, it, best_cnt = 0, i;

  if (!cur)
    return 0;
  memset (mask, 0, n);

  for (it = 0; it < iters; it++)
    {
      Pt sa[2], sb[2];
      Sim m;
      int i0, i1, cnt = 0;

      i0 = rng_next (&rng) % n;
      do
        i1 = rng_next (&rng) % n;
      while (i1 == i0);
      sa[0] = A[i0]; sa[1] = A[i1];
      sb[0] = B[i0]; sb[1] = B[i1];
      if (!sim_from2 (sa, sb, &m))
        continue;

      for (i = 0; i < n; i++)
        cnt += (cur[i] = sim_err2 (&m, A[i], B[i]) <= RANSAC_THR * RANSAC_THR);
      if (cnt > best_cnt && cnt > 1)
        {
          best_cnt = cnt;
          *best = m;
          memcpy (mask, cur, n);
          iters = ransac_update_iters (RANSAC_CONF, (double) (n - cnt) / n, RANSAC_MAX_ITERS);
        }
    }
  free (cur);

  if (best_cnt >= 2)
    sim_refit (A, B, mask, n, best);
  return best_cnt;
}

static int
align (const FtmFeatures *pa, const FtmFeatures *pb, Sim *m)
{
  Pt *A, *B;
  uint8_t *mask;
  int i, j, k, ng = 0, inl;
  double scale;

  if (pa->n < 3 || pb->n < 3)
    return 0;
  A = malloc (pa->n * sizeof (Pt));
  B = malloc (pa->n * sizeof (Pt));
  mask = malloc (pa->n);
  if (!A || !B || !mask)
    {
      free (A); free (B); free (mask);
      return 0;
    }

  /* nearest / second nearest neighbour, squared L2 on 8-bit descriptors */
  for (i = 0; i < pa->n; i++)
    {
      const uint8_t *q = pa->desc + (size_t) i * DESC_LEN;
      uint32_t d1 = UINT32_MAX, d2 = UINT32_MAX;
      int b1 = -1;

      for (j = 0; j < pb->n; j++)
        {
          const uint8_t *t = pb->desc + (size_t) j * DESC_LEN;
          uint32_t s = 0;

          for (k = 0; k < DESC_LEN; k++)
            {
              int dd = (int) q[k] - (int) t[k];
              s += (uint32_t) (dd * dd);
            }
          if (s < d1)
            {
              d2 = d1;
              d1 = s;
              b1 = j;
            }
          else if (s < d2)
            {
              d2 = s;
            }
        }
      /* sqrt(d1) < RATIO * sqrt(d2) */
      if (b1 >= 0 && d2 != UINT32_MAX && (double) d1 < RATIO * RATIO * (double) d2)
        {
          A[ng] = pa->pt[i];
          B[ng] = pb->pt[b1];
          ng++;
        }
    }

  inl = 0;
  if (ng >= MIN_INLIERS)
    inl = ransac_sim (A, B, ng, m, mask);

  free (A);
  free (B);
  free (mask);

  if (inl < MIN_INLIERS)
    return 0;
  scale = hypot (m->a, m->b);
  return scale > SCALE_MIN && scale < SCALE_MAX;
}

double
ftm_score (const FtmFeatures *probe, const FtmFeatures *tmpl)
{
  float wa[NPIX], wsq[NPIX];
  uint8_t valid[NPIX], wcontent[NPIX];
  double ia, ib, itx, ity, det;
  double sx = 0, sy = 0, sxx = 0, syy = 0, sxy = 0, den;
  Sim m;
  int x, y, i, cnt = 0;

  if (!probe || !tmpl || !align (probe, tmpl, &m))
    return 0.0;

  /* inverse of M for backward warping */
  det = m.a * m.a + m.b * m.b;
  if (det < 1e-12)
    return 0.0;
  ia = m.a / det;
  ib = m.b / det;
  itx = -(ia * m.tx + ib * m.ty);
  ity = -(-ib * m.tx + ia * m.ty);

  /* bilinear warp of the probe into the template frame; pixels whose
   * support leaves the probe image are invalid (sub-pixel step 1/32) */
  for (y = 0; y < FTM_H; y++)
    for (x = 0; x < FTM_W; x++)
      {
        double fx = ia * x + ib * y + itx;
        double fy = -ib * x + ia * y + ity;
        long qx = lround (fx * 32), qy = lround (fy * 32);
        long x0 = qx >> 5, y0 = qy >> 5;   /* arithmetic shift == floor */
        float tx = (qx & 31) / 32.0f, ty = (qy & 31) / 32.0f;
        int o = y * FTM_W + x;

        if (x0 < 0 || x0 >= FTM_W - 1 || y0 < 0 || y0 >= FTM_H - 1)
          {
            valid[o] = 0;
            wa[o] = 0;
          }
        else
          {
            const float *e = probe->e;
            size_t p0 = (size_t) y0 * FTM_W + x0;
            valid[o] = 1;
            wa[o] = (1 - ty) * ((1 - tx) * e[p0] + tx * e[p0 + 1]) +
                    ty * ((1 - tx) * e[p0 + FTM_W] + tx * e[p0 + FTM_W + 1]);
          }
        wsq[o] = wa[o] * wa[o];
      }
  content_mask (wsq, wcontent);

  for (i = 0; i < NPIX; i++)
    if (valid[i] && wcontent[i] && tmpl->content[i])
      {
        double a = wa[i], b = tmpl->e[i];
        sx += a; sy += b;
        sxx += a * a; syy += b * b; sxy += a * b;
        cnt++;
      }

  memset (wa, 0, sizeof (wa));
  memset (wsq, 0, sizeof (wsq));

  if (cnt < MIN_OVERLAP * NPIX)
    return 0.0;
  den = sqrt ((sxx - sx * sx / cnt) * (syy - sy * sy / cnt));
  if (den <= 0)
    return 0.0;
  return (sxy - sx * sy / cnt) / den;
}
