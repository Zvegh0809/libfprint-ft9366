/*
 * FocalTech FT9366 + Realtek RTS5811 bridge (USB 2808:a658) - libfprint image driver
 *
 * Clean-room: the protocol was derived solely from observing the Windows
 * driver's USB traffic (usbmon). No vendor code is used.
 *
 * Protocol (every command: opcode + its complement):
 *   04 fb AAAA NNNN        read N 16-bit registers     -> 2*N bytes
 *   05 fa AAAA NNNN DD..   write 16-bit registers
 *   06 f9 AAAA NNNN        read data block (image: 9a05, 0x1400 words = 10240 B)
 *   08 f7 RR 00 00         read 8-bit register          -> 1 byte
 *   09 f6 RR VV            write 8-bit register
 *   4c/4f/50/51/4b ...     Realtek bridge commands
 *
 * Image: 64 x 80 words, 16-bit big-endian, 12 bits used; columns 0 and 63 are empty.
 * Finger detect: once armed, 0x9a8b reads 0x0000, and 0x9362 again on touch.
 * Lift detect: the four 0x80b8 values are ~0x0220 without a finger,
 * ~0x0100-0x0140 with one.
 *
 * Matching: NBIS finds too few minutiae on a 60x77 image, so a custom matcher
 * (ft9366_match.c: SIFT + RANSAC + correlation over the aligned overlap) is
 * plugged in through the FpImageDeviceClass extract/match hooks.
 *
 * Status: EXPERIMENTAL. See the TODO comments.
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#define FP_COMPONENT "ft9366"

#include <stdlib.h>
#include <string.h>

#include "drivers_api.h"
#include "ft9366_seq.h"
#include "ft9366_match.h"
#include "ft9366_crypt.h"

#define EP_OUT            (0x01 | FPI_USB_ENDPOINT_OUT)
#define EP_IN             (0x02 | FPI_USB_ENDPOINT_IN)
#define USB_INTERFACE     0
#define CMD_TIMEOUT       2000
#define REPLY_BUF         512

#define RAW_W             64
#define RAW_H             80
#define RAW_LEN           (RAW_W * RAW_H * 2)
/* Crop: columns 0/63 are empty, the top 3 rows are noisy/saturated.
 * Leaves 60 columns (2..61) x 77 rows (3..79). */
#define IMG_X0            2
#define IMG_Y0            3
#define IMG_W             (RAW_W - 2 * IMG_X0)
#define IMG_H             (RAW_H - IMG_Y0)
/* high-pass window; ridge period is ~6 px */
#define HP_BOX            9
/* On activation BG_FRAMES empty frames are averaged into a background.
 * A finger lowers a frame's mean by ~200-450, so only frames within BG_TOL
 * of the brightest one are used. */
#define BG_FRAMES         5
#define BG_TOL            60
/* observed empty-sensor background mean: 2370-2570 */
#define BG_SANE_MIN       2300
/* A finger resting on the sensor during calibration is detected by the
 * background mean: a drop of more than BG_FINGER_DROP against the last
 * accepted calibration (or BG_SANE_MIN on the first one). While a finger
 * rests on it, the sensor returns all-zero frames after the first one, so
 * frames below BG_VALID_MIN are never valid. Calibration is retried up to
 * BG_RETRIES times; then a valid but lower level is accepted (a genuine
 * baseline shift, e.g. temperature), while no valid frame fails activation. */
#define BG_FINGER_DROP    150
#define BG_VALID_MIN      1000
#define BG_RETRIES        20
#define BG_RETRY_MS       300
#define BG_BYTES          ((gsize) BG_FRAMES * RAW_W * RAW_H * sizeof (guint16))

#define CHIP_ID           0x9362
#define POLL_FINGER_MS    32
#define POLL_LIFT_MS      100
/* All four 0x80b8 values above this => no finger. TODO: tune */
#define FINGER_OFF_LEVEL  0x01c0

/* On the development set (4 fingers x 20 images, one person) the best
 * impostor score was 0.764; at 0.80 with 15 enroll images FRR is ~21% with
 * no false accepts. Small dataset: retune with more subjects. */
#define MATCH_THRESHOLD   0.80
/* below either limit the capture is retried */
#define MIN_COVERAGE      0.30
#define MIN_KEYPOINTS     10

/* ridge spacing ~8.7 px (~460 dpi native), so the image is used unscaled */
G_STATIC_ASSERT (IMG_W == FTM_W && IMG_H == FTM_H);

static const guint8 cmd_read_id[]  = { 0x04, 0xfb, 0x9a, 0x8b, 0x00, 0x01 };
static const guint8 cmd_read_img[] = { 0x06, 0xf9, 0x9a, 0x05, 0x14, 0x00 };
static const guint8 cmd_read_9881[] = { 0x04, 0xfb, 0x98, 0x81, 0x00, 0x01 };

struct _FpiDeviceFt9366
{
  FpImageDevice parent;

  FpiSsm          *task_ssm;
  gboolean         deactivating;

  /* command sequence runner */
  const Ft9366Cmd *seq;
  gsize            seq_len;
  gsize            seq_pos;

  guint8           reply[16];
  gsize            reply_len;

  /* allocated separately: GObject instances are limited to 64 KiB */
  guint8          *raw;         /* RAW_LEN bytes */
  float           *bg;          /* RAW_W * RAW_H */
  float           *work_a;      /* IMG_W * IMG_H, build_image() scratch */
  float           *work_b;      /* IMG_W * IMG_H */
  double          *work_integ;  /* (IMG_W + 1) * (IMG_H + 1) */
  guint16         *bg_frames;   /* BG_FRAMES * RAW_W * RAW_H */
  guint32          bg_means[BG_FRAMES];
  guint            bg_count;
  guint            bg_retries;
  guint32          bg_ref_mean;   /* last accepted calibration, 0 = none */
  gboolean         have_bg;

  /* task SSM results, reported from the done callback */
  gboolean         finger_on;
  gboolean         finger_off;
  FpImage         *pending_img;
};

G_DECLARE_FINAL_TYPE (FpiDeviceFt9366, fpi_device_ft9366, FPI, DEVICE_FT9366, FpImageDevice);
G_DEFINE_TYPE (FpiDeviceFt9366, fpi_device_ft9366, FP_TYPE_IMAGE_DEVICE);

static const FpIdEntry id_table[] = {
  { .vid = 0x2808, .pid = 0xa658, },
  { .vid = 0,      .pid = 0,      .driver_data = 0 },
};

static void start_deactivate (FpiDeviceFt9366 *self);

/* ------------------------------------------------------------------ */
/* USB primitives                                                       */
/* ------------------------------------------------------------------ */

static void
send_bytes (FpiSsm *ssm, FpDevice *dev, const guint8 *data, gsize len)
{
  FpiUsbTransfer *transfer = fpi_usb_transfer_new (dev);

  fpi_usb_transfer_fill_bulk_full (transfer, EP_OUT,
                                   g_memdup2 (data, len), len, g_free);
  transfer->ssm = ssm;
  transfer->short_is_error = TRUE;
  fpi_usb_transfer_submit (transfer, CMD_TIMEOUT, NULL,
                           fpi_ssm_usb_transfer_cb, NULL);
}

static void
reply_cb (FpiUsbTransfer *transfer, FpDevice *dev,
          gpointer user_data, GError *error)
{
  FpiDeviceFt9366 *self = FPI_DEVICE_FT9366 (dev);

  if (error)
    {
      fpi_ssm_mark_failed (transfer->ssm, error);
      return;
    }

  self->reply_len = MIN ((gsize) transfer->actual_length, sizeof (self->reply));
  memcpy (self->reply, transfer->buffer, self->reply_len);
  fpi_ssm_next_state (transfer->ssm);
}

static void
recv_reply (FpiSsm *ssm, FpDevice *dev)
{
  FpiUsbTransfer *transfer = fpi_usb_transfer_new (dev);

  fpi_usb_transfer_fill_bulk (transfer, EP_IN, REPLY_BUF);
  transfer->ssm = ssm;
  fpi_usb_transfer_submit (transfer, CMD_TIMEOUT, NULL, reply_cb, NULL);
}

static void
image_cb (FpiUsbTransfer *transfer, FpDevice *dev,
          gpointer user_data, GError *error)
{
  FpiDeviceFt9366 *self = FPI_DEVICE_FT9366 (dev);

  if (error)
    {
      fpi_ssm_mark_failed (transfer->ssm, error);
      return;
    }

  memcpy (self->raw, transfer->buffer, RAW_LEN);
  fpi_ssm_next_state (transfer->ssm);
}

static void
recv_image (FpiSsm *ssm, FpDevice *dev)
{
  FpiUsbTransfer *transfer = fpi_usb_transfer_new (dev);

  fpi_usb_transfer_fill_bulk (transfer, EP_IN, RAW_LEN);
  transfer->ssm = ssm;
  transfer->short_is_error = TRUE;
  fpi_usb_transfer_submit (transfer, CMD_TIMEOUT, NULL, image_cb, NULL);
}

static guint16
reply_word (FpiDeviceFt9366 *self, gsize i)
{
  if (self->reply_len < 2 * i + 2)
    return 0;
  return (self->reply[2 * i] << 8) | self->reply[2 * i + 1];
}

/* ------------------------------------------------------------------ */
/* Sub-SSM that replays the ft9366_seq.h command tables                */
/* ------------------------------------------------------------------ */

enum seq_states {
  SEQ_DELAY,
  SEQ_SEND,
  SEQ_RECV,
  SEQ_ADVANCE,
  SEQ_NUM_STATES,
};

static void
seq_run_state (FpiSsm *ssm, FpDevice *dev)
{
  FpiDeviceFt9366 *self = FPI_DEVICE_FT9366 (dev);
  const Ft9366Cmd *c;

  if (self->seq_pos >= self->seq_len)
    {
      fpi_ssm_mark_completed (ssm);
      return;
    }
  c = &self->seq[self->seq_pos];

  switch (fpi_ssm_get_cur_state (ssm))
    {
    case SEQ_DELAY:
      if (c->delay_ms)
        fpi_ssm_next_state_delayed (ssm, c->delay_ms);
      else
        fpi_ssm_next_state (ssm);
      break;

    case SEQ_SEND:
      send_bytes (ssm, dev, c->data, c->len);
      break;

    case SEQ_RECV:
      if (c->inlen)
        recv_reply (ssm, dev);
      else
        fpi_ssm_next_state (ssm);
      break;

    case SEQ_ADVANCE:
      self->seq_pos++;
      fpi_ssm_jump_to_state (ssm, SEQ_DELAY);
      break;

    default:
      g_assert_not_reached ();
    }
}

#define run_seq(parent, dev, table) \
  run_seq_n ((parent), (dev), (table), G_N_ELEMENTS (table))

static void
run_seq_n (FpiSsm *parent, FpDevice *dev, const Ft9366Cmd *table, gsize n)
{
  FpiDeviceFt9366 *self = FPI_DEVICE_FT9366 (dev);
  FpiSsm *child = fpi_ssm_new (dev, seq_run_state, SEQ_NUM_STATES);

  self->seq = table;
  self->seq_len = n;
  self->seq_pos = 0;
  fpi_ssm_start_subsm (parent, child);
}

/* ------------------------------------------------------------------ */
/* Image processing                                                     */
/* ------------------------------------------------------------------ */

static int
cmp_float (const void *a, const void *b)
{
  float x = *(const float *) a, y = *(const float *) b;

  return (x > y) - (x < y);
}

static void
bg_reset (FpiDeviceFt9366 *self)
{
  explicit_bzero (self->bg_frames, BG_BYTES);
  explicit_bzero (self->bg_means, sizeof (self->bg_means));
  self->bg_count = 0;
}

static void
bg_accumulate (FpiDeviceFt9366 *self)
{
  guint16 *f = self->bg_frames + (gsize) self->bg_count * RAW_W * RAW_H;
  guint64 sum = 0;
  gsize i;

  for (i = 0; i < RAW_W * RAW_H; i++)
    {
      f[i] = (self->raw[2 * i] << 8) | self->raw[2 * i + 1];
      sum += f[i];
    }
  self->bg_means[self->bg_count] = sum / (RAW_W * RAW_H);
  fp_dbg ("background frame %u, mean %u",
          self->bg_count + 1, self->bg_means[self->bg_count]);
  self->bg_count++;
}

static guint32
bg_max_mean (FpiDeviceFt9366 *self)
{
  guint32 max_mean = 0;
  guint k;

  for (k = 0; k < self->bg_count; k++)
    max_mean = MAX (max_mean, self->bg_means[k]);
  return max_mean;
}

static gboolean
bg_finger_suspected (FpiDeviceFt9366 *self)
{
  guint32 m = bg_max_mean (self);

  if (m < BG_VALID_MIN)
    return TRUE;
  if (self->bg_ref_mean)
    return m + BG_FINGER_DROP < self->bg_ref_mean;
  return m < BG_SANE_MIN;
}

static void
bg_finalize (FpiDeviceFt9366 *self)
{
  guint32 max_mean = bg_max_mean (self);
  gboolean use[BG_FRAMES] = { FALSE };
  guint used = 0, k;
  gsize i;

  for (k = 0; k < self->bg_count; k++)
    if (self->bg_means[k] + BG_TOL >= max_mean)
      {
        use[k] = TRUE;
        used++;
      }
    else
      {
        fp_dbg ("background frame %u rejected (finger on sensor?)", k + 1);
      }

  for (i = 0; i < RAW_W * RAW_H; i++)
    {
      guint32 s = 0;

      for (k = 0; k < self->bg_count; k++)
        if (use[k])
          s += self->bg_frames[(gsize) k * RAW_W * RAW_H + i];
      self->bg[i] = (float) s / used;
    }

  if (max_mean < BG_SANE_MIN)
    fp_warn ("background mean %u is low - finger on the sensor during calibration?",
             max_mean);
  fp_dbg ("background: %u/%u frames used, mean %u", used, self->bg_count, max_mean);

  explicit_bzero (self->bg_frames, BG_BYTES);
  self->bg_ref_mean = max_mean;
  self->have_bg = TRUE;
}

/* clamped pixel access */
#define AT(buf, w, h, x, y) \
  (buf)[CLAMP ((gint) (y), 0, (h) - 1) * (w) + CLAMP ((gint) (x), 0, (w) - 1)]

static FpImage *
build_image (FpiDeviceFt9366 *self)
{
  float *a = self->work_a, *b = self->work_b;
  double *integ = self->work_integ;
  const gsize n = IMG_W * IMG_H;
  const gint r = HP_BOX / 2;
  FpImage *img;
  float lo, hi;
  gint x, y;

  /* 1. background subtraction + crop */
  for (y = 0; y < IMG_H; y++)
    for (x = 0; x < IMG_W; x++)
      {
        gsize ri = (y + IMG_Y0) * RAW_W + x + IMG_X0;
        float v = (self->raw[2 * ri] << 8) | self->raw[2 * ri + 1];

        if (self->have_bg)
          v -= self->bg[ri];
        a[y * IMG_W + x] = v;
      }

  /* 2. 3x3 Gaussian blur */
  for (y = 0; y < IMG_H; y++)
    for (x = 0; x < IMG_W; x++)
      {
        float s =
          1 * AT (a, IMG_W, IMG_H, x - 1, y - 1) + 2 * AT (a, IMG_W, IMG_H, x, y - 1) + 1 * AT (a, IMG_W, IMG_H, x + 1, y - 1) +
          2 * AT (a, IMG_W, IMG_H, x - 1, y)     + 4 * AT (a, IMG_W, IMG_H, x, y)     + 2 * AT (a, IMG_W, IMG_H, x + 1, y) +
          1 * AT (a, IMG_W, IMG_H, x - 1, y + 1) + 2 * AT (a, IMG_W, IMG_H, x, y + 1) + 1 * AT (a, IMG_W, IMG_H, x + 1, y + 1);
        b[y * IMG_W + x] = s / 16.0f;
      }

  /* 3. high-pass: subtract the HP_BOX box mean (integral image; edge
   *    windows only count in-image pixels) */
  for (y = 0; y <= IMG_H; y++)
    integ[y * (IMG_W + 1)] = 0;
  for (x = 0; x <= IMG_W; x++)
    integ[x] = 0;
  for (y = 1; y <= IMG_H; y++)
    for (x = 1; x <= IMG_W; x++)
      integ[y * (IMG_W + 1) + x] = b[(y - 1) * IMG_W + (x - 1)]
                                   + integ[(y - 1) * (IMG_W + 1) + x]
                                   + integ[y * (IMG_W + 1) + x - 1]
                                   - integ[(y - 1) * (IMG_W + 1) + x - 1];
  for (y = 0; y < IMG_H; y++)
    for (x = 0; x < IMG_W; x++)
      {
        gint x0 = MAX (x - r, 0), x1 = MIN (x + r + 1, IMG_W);
        gint y0 = MAX (y - r, 0), y1 = MIN (y + r + 1, IMG_H);
        double s = integ[y1 * (IMG_W + 1) + x1] - integ[y0 * (IMG_W + 1) + x1]
                   - integ[y1 * (IMG_W + 1) + x0] + integ[y0 * (IMG_W + 1) + x0];

        a[y * IMG_W + x] = b[y * IMG_W + x] - (float) (s / ((x1 - x0) * (y1 - y0)));
      }

  /* 4. stretch the 2nd-98th percentile to 0..255 */
  memcpy (b, a, n * sizeof (float));
  qsort (b, n, sizeof (float), cmp_float);
  lo = b[n / 50];
  hi = b[n - 1 - n / 50];
  if (hi <= lo)
    hi = lo + 1;
  img = fp_image_new (IMG_W, IMG_H);
  for (gsize i = 0; i < n; i++)
    img->data[i] = CLAMP ((gint) ((a[i] - lo) * 255.0f / (hi - lo)), 0, 255);

  /* TODO: verify polarity (FPI_IMAGE_COLORS_INVERTED?) */
  img->flags = 0;

  explicit_bzero (a, n * sizeof (float));
  explicit_bzero (b, n * sizeof (float));
  explicit_bzero (integ, (IMG_W + 1) * (IMG_H + 1) * sizeof (double));

  /* DEVELOPMENT ONLY: dump the image handed to libfprint. Compiled in only
   * with -Dc_args=-DFT9366_DEVEL, so release builds cannot write images to
   * disk even if FT9366_DUMP is set. */
#ifdef FT9366_DEVEL
  {
    const char *dump = g_getenv ("FT9366_DUMP");

    if (dump)
      {
        g_autofree char *hdr = g_strdup_printf ("P5 %d %d 255\n", IMG_W, IMG_H);
        g_autoptr(GByteArray) pgm = g_byte_array_new ();

        g_byte_array_append (pgm, (guint8 *) hdr, strlen (hdr));
        g_byte_array_append (pgm, img->data, IMG_W * IMG_H);
        g_file_set_contents (dump, (char *) pgm->data, pgm->len, NULL);
        fp_dbg ("DEBUG: image dumped to %s", dump);
      }
  }
#endif

  return img;
}

/* ------------------------------------------------------------------ */
/* Shared task completion                                               */
/* ------------------------------------------------------------------ */

static void
task_done_common (FpiDeviceFt9366 *self, GError *error)
{
  FpImageDevice *dev = FP_IMAGE_DEVICE (self);

  self->task_ssm = NULL;

  if (error && !self->deactivating)
    {
      fpi_image_device_session_error (dev, error);
      return;
    }
  g_clear_error (&error);

  if (self->deactivating)
    {
      g_clear_object (&self->pending_img);
      start_deactivate (self);
      return;
    }

  /* Results are reported only here: libfprint changes state in response and
   * may start a new SSM, which must not overlap with the finished one. */
  if (self->finger_on)
    {
      self->finger_on = FALSE;
      fpi_image_device_report_finger_status (dev, TRUE);
    }
  else if (self->pending_img)
    {
      FpImage *img = g_steal_pointer (&self->pending_img);
      fpi_image_device_image_captured (dev, img);
    }
  else if (self->finger_off)
    {
      self->finger_off = FALSE;
      fpi_image_device_report_finger_status (dev, FALSE);
    }
}

static void
task_done (FpiSsm *ssm, FpDevice *dev, GError *error)
{
  task_done_common (FPI_DEVICE_FT9366 (dev), error);
}

static void
start_task (FpiDeviceFt9366 *self, FpiSsmHandlerCallback handler, int nstates)
{
  g_assert (self->task_ssm == NULL);
  self->task_ssm = fpi_ssm_new (FP_DEVICE (self), handler, nstates);
  fpi_ssm_start (self->task_ssm, task_done);
}

/* ------------------------------------------------------------------ */
/* Open: wake up + chip ID check                                       */
/* ------------------------------------------------------------------ */

enum open_states {
  OPEN_WAKE,
  OPEN_ID_SEND,
  OPEN_ID_RECV,
  OPEN_ID_CHECK,
  OPEN_SLEEP,
  OPEN_NUM_STATES,
};

static void
open_run_state (FpiSsm *ssm, FpDevice *dev)
{
  FpiDeviceFt9366 *self = FPI_DEVICE_FT9366 (dev);

  switch (fpi_ssm_get_cur_state (ssm))
    {
    case OPEN_WAKE:
      run_seq (ssm, dev, ft9366_seq_wake);
      break;

    case OPEN_ID_SEND:
      send_bytes (ssm, dev, cmd_read_id, sizeof (cmd_read_id));
      break;

    case OPEN_ID_RECV:
      recv_reply (ssm, dev);
      break;

    case OPEN_ID_CHECK:
      if (reply_word (self, 0) != CHIP_ID)
        {
          fpi_ssm_mark_failed (ssm,
                               fpi_device_error_new_msg (FP_DEVICE_ERROR_NOT_SUPPORTED,
                                                         "Unexpected chip id 0x%04x",
                                                         reply_word (self, 0)));
          return;
        }
      fp_dbg ("chip id 0x%04x", CHIP_ID);
      fpi_ssm_next_state (ssm);
      break;

    case OPEN_SLEEP:
      run_seq (ssm, dev, ft9366_seq_sleep);
      break;

    default:
      g_assert_not_reached ();
    }
}

static void
open_done (FpiSsm *ssm, FpDevice *dev, GError *error)
{
  fpi_image_device_open_complete (FP_IMAGE_DEVICE (dev), error);
}

static void
dev_open (FpImageDevice *dev)
{
  GError *error = NULL;
  FpiSsm *ssm;

  /* Fail closed: refuse to work without the template key */
  if (!ft9366_crypt_check_key (&error))
    {
      fpi_image_device_open_complete (dev,
                                      fpi_device_error_new_msg (FP_DEVICE_ERROR_GENERAL,
                                                                "%s", error->message));
      g_clear_error (&error);
      return;
    }

  if (!g_usb_device_claim_interface (fpi_device_get_usb_device (FP_DEVICE (dev)),
                                     USB_INTERFACE, 0, &error))
    {
      fpi_image_device_open_complete (dev, error);
      return;
    }

  ssm = fpi_ssm_new (FP_DEVICE (dev), open_run_state, OPEN_NUM_STATES);
  fpi_ssm_start (ssm, open_done);
}

static void
dev_close (FpImageDevice *dev)
{
  FpiDeviceFt9366 *self = FPI_DEVICE_FT9366 (dev);
  GError *error = NULL;

  self->have_bg = FALSE;
  explicit_bzero (self->bg, RAW_W * RAW_H * sizeof (float));
  explicit_bzero (self->raw, RAW_LEN);

  g_usb_device_release_interface (fpi_device_get_usb_device (FP_DEVICE (dev)),
                                  USB_INTERFACE, 0, &error);
  fpi_image_device_close_complete (dev, error);
}

/* ------------------------------------------------------------------ */
/* Activate: capture the background (no finger!)                      */
/* ------------------------------------------------------------------ */

enum act_states {
  ACT_WAKE,
  ACT_PREP,
  ACT_IMG_SEND,
  ACT_IMG_RECV,
  ACT_STORE,
  ACT_POST,
  ACT_LOOP,
  ACT_NUM_STATES,
};

static void
act_run_state (FpiSsm *ssm, FpDevice *dev)
{
  FpiDeviceFt9366 *self = FPI_DEVICE_FT9366 (dev);

  switch (fpi_ssm_get_cur_state (ssm))
    {
    case ACT_WAKE:
      run_seq (ssm, dev, ft9366_seq_wake);
      break;

    case ACT_PREP:
      run_seq (ssm, dev, ft9366_seq_capture);
      break;

    case ACT_IMG_SEND:
      send_bytes (ssm, dev, cmd_read_img, sizeof (cmd_read_img));
      break;

    case ACT_IMG_RECV:
      recv_image (ssm, dev);
      break;

    case ACT_STORE:
      bg_accumulate (self);
      explicit_bzero (self->raw, RAW_LEN);
      fpi_ssm_next_state (ssm);
      break;

    case ACT_POST:
      run_seq (ssm, dev, ft9366_seq_post);
      break;

    case ACT_LOOP:
      if (self->bg_count < BG_FRAMES)
        {
          fpi_ssm_jump_to_state (ssm, ACT_PREP);
        }
      else if (bg_finger_suspected (self) && self->bg_retries < BG_RETRIES)
        {
          self->bg_retries++;
          fp_info ("background mean %u: finger on the sensor? retrying (%u/%u)",
                   bg_max_mean (self), self->bg_retries, BG_RETRIES);
          bg_reset (self);
          fpi_ssm_jump_to_state_delayed (ssm, ACT_PREP, BG_RETRY_MS);
        }
      else if (bg_max_mean (self) < BG_VALID_MIN)
        {
          bg_reset (self);
          fpi_ssm_mark_failed (ssm,
                               fpi_device_error_new_msg (FP_DEVICE_ERROR_GENERAL,
                                                         "Finger on the sensor during calibration; lift it and try again"));
        }
      else
        {
          bg_finalize (self);
          fpi_ssm_mark_completed (ssm);
        }
      break;

    default:
      g_assert_not_reached ();
    }
}

static void
act_done (FpiSsm *ssm, FpDevice *dev, GError *error)
{
  fpi_image_device_activate_complete (FP_IMAGE_DEVICE (dev), error);
}

static void
dev_activate (FpImageDevice *dev)
{
  FpiDeviceFt9366 *self = FPI_DEVICE_FT9366 (dev);
  FpiSsm *ssm;

  self->deactivating = FALSE;
  self->finger_on = self->finger_off = FALSE;
  self->bg_retries = 0;
  bg_reset (self);

  ssm = fpi_ssm_new (FP_DEVICE (dev), act_run_state, ACT_NUM_STATES);
  fpi_ssm_start (ssm, act_done);
}

/* ------------------------------------------------------------------ */
/* Wait for finger                                                      */
/* ------------------------------------------------------------------ */

enum fon_states {
  FON_ARM,
  FON_POLL_SEND,
  FON_POLL_RECV,
  FON_POLL_CHECK,
  FON_NUM_STATES,
};

static void
fon_run_state (FpiSsm *ssm, FpDevice *dev)
{
  FpiDeviceFt9366 *self = FPI_DEVICE_FT9366 (dev);

  switch (fpi_ssm_get_cur_state (ssm))
    {
    case FON_ARM:
      run_seq (ssm, dev, ft9366_seq_arm);
      break;

    case FON_POLL_SEND:
      if (self->deactivating)
        {
          fpi_ssm_mark_completed (ssm);
          return;
        }
      send_bytes (ssm, dev, cmd_read_id, sizeof (cmd_read_id));
      break;

    case FON_POLL_RECV:
      recv_reply (ssm, dev);
      break;

    case FON_POLL_CHECK:
      if (reply_word (self, 0) == CHIP_ID)
        {
          fp_dbg ("finger detected");
          self->finger_on = TRUE;
          fpi_ssm_mark_completed (ssm);
        }
      else
        {
          fpi_ssm_jump_to_state_delayed (ssm, FON_POLL_SEND, POLL_FINGER_MS);
        }
      break;

    default:
      g_assert_not_reached ();
    }
}

/* ------------------------------------------------------------------ */
/* Capture                                                              */
/* ------------------------------------------------------------------ */

enum cap_states {
  CAP_PREP,
  CAP_IMG_SEND,
  CAP_IMG_RECV,
  CAP_BUILD,
  CAP_POST,
  CAP_NUM_STATES,
};

static void
cap_run_state (FpiSsm *ssm, FpDevice *dev)
{
  FpiDeviceFt9366 *self = FPI_DEVICE_FT9366 (dev);

  switch (fpi_ssm_get_cur_state (ssm))
    {
    case CAP_PREP:
      run_seq (ssm, dev, ft9366_seq_capture);
      break;

    case CAP_IMG_SEND:
      send_bytes (ssm, dev, cmd_read_img, sizeof (cmd_read_img));
      break;

    case CAP_IMG_RECV:
      recv_image (ssm, dev);
      break;

    case CAP_BUILD:
      g_clear_object (&self->pending_img);
      self->pending_img = build_image (self);
      explicit_bzero (self->raw, RAW_LEN);
      fpi_ssm_next_state (ssm);
      break;

    case CAP_POST:
      run_seq (ssm, dev, ft9366_seq_post);
      break;

    default:
      g_assert_not_reached ();
    }
}

/* ------------------------------------------------------------------ */
/* Wait for finger lift                                                 */
/* ------------------------------------------------------------------ */

enum foff_states {
  FOFF_PRE,
  FOFF_9881_SEND,
  FOFF_9881_RECV,
  FOFF_9881_WRITE,
  FOFF_POST,
  FOFF_CHECK,
  FOFF_NUM_STATES,
};

static void
foff_run_state (FpiSsm *ssm, FpDevice *dev)
{
  FpiDeviceFt9366 *self = FPI_DEVICE_FT9366 (dev);

  switch (fpi_ssm_get_cur_state (ssm))
    {
    case FOFF_PRE:
      if (self->deactivating)
        {
          fpi_ssm_mark_completed (ssm);
          return;
        }
      run_seq (ssm, dev, ft9366_seq_probe_pre);
      break;

    case FOFF_9881_SEND:
      send_bytes (ssm, dev, cmd_read_9881, sizeof (cmd_read_9881));
      break;

    case FOFF_9881_RECV:
      recv_reply (ssm, dev);
      break;

    case FOFF_9881_WRITE:
      {
        /* the Windows driver sets bit 1 of 0x9881 here
         * (0f0c -> 0f0e, 0f8d -> 0f8f) */
        guint16 v = reply_word (self, 0) | 0x0002;
        guint8 cmd[8] = { 0x05, 0xfa, 0x98, 0x81, 0x00, 0x01, v >> 8, v & 0xff };

        send_bytes (ssm, dev, cmd, sizeof (cmd));
      }
      break;

    case FOFF_POST:
      run_seq (ssm, dev, ft9366_seq_probe_post);
      break;

    case FOFF_CHECK:
      {
        /* the last command with a reply in probe_post reads 0x80b8 */
        guint16 m = G_MAXUINT16;
        gsize i;

        for (i = 0; i < 4; i++)
          m = MIN (m, reply_word (self, i));
        fp_dbg ("lift probe: min level 0x%04x", m);

        if (self->reply_len == 8 && m > FINGER_OFF_LEVEL)
          {
            self->finger_off = TRUE;
            fpi_ssm_mark_completed (ssm);
          }
        else
          {
            fpi_ssm_jump_to_state_delayed (ssm, FOFF_PRE, POLL_LIFT_MS);
          }
      }
      break;

    default:
      g_assert_not_reached ();
    }
}

/* ------------------------------------------------------------------ */
/* State changes and deactivation                                      */
/* ------------------------------------------------------------------ */

static void
dev_change_state (FpImageDevice *dev, FpiImageDeviceState state)
{
  FpiDeviceFt9366 *self = FPI_DEVICE_FT9366 (dev);

  if (self->deactivating)
    return;

  switch (state)
    {
    case FPI_IMAGE_DEVICE_STATE_AWAIT_FINGER_ON:
      start_task (self, fon_run_state, FON_NUM_STATES);
      break;

    case FPI_IMAGE_DEVICE_STATE_CAPTURE:
      start_task (self, cap_run_state, CAP_NUM_STATES);
      break;

    case FPI_IMAGE_DEVICE_STATE_AWAIT_FINGER_OFF:
      start_task (self, foff_run_state, FOFF_NUM_STATES);
      break;

    case FPI_IMAGE_DEVICE_STATE_INACTIVE:
    case FPI_IMAGE_DEVICE_STATE_ACTIVATING:
    case FPI_IMAGE_DEVICE_STATE_DEACTIVATING:
    case FPI_IMAGE_DEVICE_STATE_IDLE:
    default:
      break;
    }
}

enum deact_states {
  DEACT_SLEEP,
  DEACT_NUM_STATES,
};

static void
deact_run_state (FpiSsm *ssm, FpDevice *dev)
{
  run_seq (ssm, dev, ft9366_seq_sleep);
}

static void
deact_done (FpiSsm *ssm, FpDevice *dev, GError *error)
{
  FpiDeviceFt9366 *self = FPI_DEVICE_FT9366 (dev);

  self->deactivating = FALSE;
  fpi_image_device_deactivate_complete (FP_IMAGE_DEVICE (dev), error);
}

static void
start_deactivate (FpiDeviceFt9366 *self)
{
  FpiSsm *ssm = fpi_ssm_new (FP_DEVICE (self), deact_run_state, DEACT_NUM_STATES);

  fpi_ssm_start (ssm, deact_done);
}

static void
dev_deactivate (FpImageDevice *dev)
{
  FpiDeviceFt9366 *self = FPI_DEVICE_FT9366 (dev);

  self->deactivating = TRUE;
  /* a running task stops at its next checkpoint and task_done_common
   * then starts the deactivation */
  if (self->task_ssm == NULL)
    start_deactivate (self);
}

/* ------------------------------------------------------------------ */
/* Matching (replaces NBIS)                                             */
/* ------------------------------------------------------------------ */

/* runs in a worker thread: must only read the image, not device state */
static GVariant *
ft9366_extract_template (FpImageDevice *dev, FpImage *image, GError **error)
{
  g_autoptr(GError) local_error = NULL;
  guint8 *buf = NULL, *sealed;
  FtmFeatures *f;
  GVariant *v;
  gsize len, sealed_len;

  if (image->width != FTM_W || image->height != FTM_H || !image->data)
    {
      g_propagate_error (error, fpi_device_error_new_msg (FP_DEVICE_ERROR_GENERAL,
                                                          "Unexpected image size %ux%u",
                                                          image->width, image->height));
      return NULL;
    }

  f = ftm_extract (image->data);
  if (!f)
    {
      g_propagate_error (error, fpi_device_error_new_msg (FP_DEVICE_ERROR_GENERAL,
                                                          "Feature extraction failed"));
      return NULL;
    }

  fp_dbg ("template: %d keypoints, coverage %.2f",
          ftm_n_keypoints (f), ftm_coverage (f));
  if (ftm_coverage (f) < MIN_COVERAGE || ftm_n_keypoints (f) < MIN_KEYPOINTS)
    {
      ftm_free (f);
      g_propagate_error (error, fpi_device_retry_new (FP_DEVICE_RETRY_CENTER_FINGER));
      return NULL;
    }

  len = ftm_serialized_size (f);
  buf = g_malloc (len);
  ftm_serialize (f, buf);
  ftm_free (f);

  /* The template (which contains the image) is stored only encrypted, so
   * fprintd never writes plaintext to /var/lib/fprint. */
  sealed = ft9366_crypt_seal (buf, len, &sealed_len, &local_error);
  ft9366_crypt_free_plain (g_steal_pointer (&buf), len);
  if (!sealed)
    {
      g_propagate_error (error, fpi_device_error_new_msg (FP_DEVICE_ERROR_GENERAL,
                                                          "%s", local_error->message));
      return NULL;
    }

  v = g_variant_new_fixed_array (G_VARIANT_TYPE_BYTE, sealed, sealed_len, 1);
  g_free (sealed);
  return v;
}

static FtmFeatures *
features_from_variant (GVariant *v, GError **error)
{
  g_autoptr(GError) local_error = NULL;
  const guint8 *data;
  guint8 *plain;
  gsize len, plain_len;
  FtmFeatures *f;

  if (!g_variant_is_of_type (v, G_VARIANT_TYPE_BYTESTRING))
    {
      g_set_error (error, FP_DEVICE_ERROR, FP_DEVICE_ERROR_DATA_INVALID,
                   "Invalid template type");
      return NULL;
    }
  data = g_variant_get_fixed_array (v, &len, 1);

  plain = ft9366_crypt_open (data, len, &plain_len, &local_error);
  if (!plain)
    {
      g_set_error (error, FP_DEVICE_ERROR, FP_DEVICE_ERROR_DATA_INVALID,
                   "%s; please re-enroll this finger", local_error->message);
      return NULL;
    }

  f = ftm_deserialize (plain, plain_len);
  ft9366_crypt_free_plain (plain, plain_len);
  if (!f)
    g_set_error (error, FP_DEVICE_ERROR, FP_DEVICE_ERROR_DATA_INVALID,
                 "Malformed template");
  return f;
}

static FpiMatchResult
ft9366_match_template (FpImageDevice *dev, GVariant *enrolled,
                       GVariant *probe, GError **error)
{
  FpiMatchResult res = FPI_MATCH_FAIL;
  FtmFeatures *pf;
  GVariantIter iter;
  GVariant *child;
  double best = 0;
  guint i = 0;

  pf = features_from_variant (probe, error);
  if (!pf)
    return FPI_MATCH_ERROR;

  g_variant_iter_init (&iter, enrolled);
  while (res == FPI_MATCH_FAIL && (child = g_variant_iter_next_value (&iter)))
    {
      g_autoptr(GVariant) inner = g_variant_get_variant (child);
      FtmFeatures *tf = features_from_variant (inner, error);
      double s;

      g_variant_unref (child);
      if (!tf)
        {
          g_prefix_error (error, "Enrolled template %u: ", i);
          res = FPI_MATCH_ERROR;
          break;
        }

      s = ftm_score (pf, tf);
      ftm_free (tf);
      best = MAX (best, s);
      if (s >= MATCH_THRESHOLD)
        res = FPI_MATCH_SUCCESS;
      i++;
    }

  fp_dbg ("match: best score %.3f over %u templates (threshold %.2f)",
          best, i, MATCH_THRESHOLD);
  ftm_free (pf);
  return res;
}

/* ------------------------------------------------------------------ */
/* GObject                                                              */
/* ------------------------------------------------------------------ */

static void
fpi_device_ft9366_init (FpiDeviceFt9366 *self)
{
  self->raw = g_malloc0 (RAW_LEN);
  self->bg = g_malloc0 (RAW_W * RAW_H * sizeof (float));
  self->bg_frames = g_malloc0 (BG_BYTES);
  self->work_a = g_new0 (float, IMG_W * IMG_H);
  self->work_b = g_new0 (float, IMG_W * IMG_H);
  self->work_integ = g_new0 (double, (IMG_W + 1) * (IMG_H + 1));
}

static void
fpi_device_ft9366_finalize (GObject *object)
{
  FpiDeviceFt9366 *self = FPI_DEVICE_FT9366 (object);

  /* do not leave fingerprint data in freed memory */
  explicit_bzero (self->raw, RAW_LEN);
  explicit_bzero (self->bg, RAW_W * RAW_H * sizeof (float));
  explicit_bzero (self->bg_frames, BG_BYTES);
  g_clear_pointer (&self->raw, g_free);
  g_clear_pointer (&self->bg, g_free);
  g_clear_pointer (&self->bg_frames, g_free);
  g_clear_pointer (&self->work_a, g_free);
  g_clear_pointer (&self->work_b, g_free);
  g_clear_pointer (&self->work_integ, g_free);
  g_clear_object (&self->pending_img);

  G_OBJECT_CLASS (fpi_device_ft9366_parent_class)->finalize (object);
}

static void
fpi_device_ft9366_class_init (FpiDeviceFt9366Class *klass)
{
  FpDeviceClass *dev_class = FP_DEVICE_CLASS (klass);
  FpImageDeviceClass *img_class = FP_IMAGE_DEVICE_CLASS (klass);

  G_OBJECT_CLASS (klass)->finalize = fpi_device_ft9366_finalize;

  dev_class->id = FP_COMPONENT;
  dev_class->full_name = "FocalTech FT9366 (Realtek RTS5811 bridge)";
  dev_class->type = FP_DEVICE_TYPE_USB;
  dev_class->id_table = id_table;
  dev_class->scan_type = FP_SCAN_TYPE_PRESS;
  dev_class->nr_enroll_stages = 16;   /* same as the Windows driver */

  img_class->img_open = dev_open;
  img_class->img_close = dev_close;
  img_class->activate = dev_activate;
  img_class->deactivate = dev_deactivate;
  img_class->change_state = dev_change_state;

  img_class->img_width = IMG_W;
  img_class->img_height = IMG_H;
  /* custom matcher; bz3_threshold is unused */
  img_class->extract_template = ft9366_extract_template;
  img_class->match_template = ft9366_match_template;
}
