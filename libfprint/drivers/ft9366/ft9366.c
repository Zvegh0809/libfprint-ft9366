/*
 * FocalTech FT9366 + Realtek RTS5811 bridge (USB 2808:a658) – libfprint image driver
 *
 * Clean-room: a protokoll kizárólag a Windows-driver USB-forgalmának
 * megfigyeléséből származik (usbmon). Gyári kódot nem használ.
 *
 * Protokoll (minden parancs: műveleti kód + annak negáltja):
 *   04 fb AAAA NNNN        N db 16 bites regiszter olvasása  -> 2*N bájt
 *   05 fa AAAA NNNN DD..   16 bites regiszter írása
 *   06 f9 AAAA NNNN        adatblokk olvasása (kép: 9a05, 0x1400 szó = 10240 B)
 *   08 f7 RR 00 00         8 bites regiszter olvasása     -> 1 bájt
 *   09 f6 RR VV            8 bites regiszter írása
 *   4c/4f/50/51/4b ...     Realtek-híd parancsok
 *
 * Kép: 64 x 80 szó, 16 bit big-endian, 12 bit hasznos; a 0. és 63. oszlop üres.
 * Ujjérzékelés: élesítés után a 0x9a8b 0x0000-t ad, ujj esetén újra 0x9362-t.
 * Felemelés: a 0x80b8-as 4 mért érték ujj nélkül ~0x0220, ujjal ~0x0100-0x0140.
 *
 * Illesztés: a 60x77-es képen az NBIS nem talál elég minutiát, ezért saját
 * illesztőt használunk (ft9366_match.c: SIFT + RANSAC + korreláció az
 * illesztett átfedésen), az FpImageDeviceClass extract/match hookjain át.
 *
 * Állapot: KÍSÉRLETI. Lásd a TODO megjegyzéseket.
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
/* Kivágás: a 0. és 63. oszlop üres, a felső 3 sor zajos/telített.
 * Marad 60 oszlop (2..61) x 77 sor (3..79). */
#define IMG_X0            2
#define IMG_Y0            3
#define IMG_W             (RAW_W - 2 * IMG_X0)
#define IMG_H             (RAW_H - IMG_Y0)
/* Nagyítás (saját bilineáris interpolációval). A mért vonalköz ~8.7 px,
 * vagyis a szenzor natívan ~460 dpi; a NBIS ~500 dpi-t vár, ezért 1. */
#define ENLARGE           1
#define OUT_W             (IMG_W * ENLARGE)
#define OUT_H             (IMG_H * ENLARGE)
/* felüláteresztő szűrő ablakmérete (a vonalköz ~6 pixel) */
#define HP_BOX            9
/* Aktiváláskor BG_FRAMES ujj nélküli képet veszünk fel. Ha közben ujj van a
 * szenzoron, az a kép átlaga ~200-450-nel kisebb; ezeket eldobjuk: csak a
 * legvilágosabbhoz képest BG_TOL-on belüli képeket átlagoljuk. */
#define BG_FRAMES         5
#define BG_TOL            60
/* ujj nélkül eddig 2370-2570 volt a háttér átlaga */
#define BG_SANE_MIN       2300
#define BG_BYTES          ((gsize) BG_FRAMES * RAW_W * RAW_H * sizeof (guint16))

#define CHIP_ID           0x9362
#define POLL_FINGER_MS    32
#define POLL_LIFT_MS      100
/* A 0x80b8 négy értékének mindegyike e fölött => nincs ujj. TODO: hangolni */
#define FINGER_OFF_LEVEL  0x01c0

/* Illesztés. A fejlesztői adatsoron (4 ujj x 20 kép, egy személy) a
 * különböző ujjú párok legmagasabb pontszáma 0.764 volt; 0.80-nál 15
 * regisztrációs képpel kb. 21% FRR, 0 hamis elfogadás. Kevés adat: a küszöböt
 * több ember mérése után újra kell hangolni. */
#define MATCH_THRESHOLD   0.80
/* ennél kevesebb vonalas terület / kulcspont => újrapróbálás */
#define MIN_COVERAGE      0.30
#define MIN_KEYPOINTS     10

G_STATIC_ASSERT (IMG_W == FTM_W && IMG_H == FTM_H && ENLARGE == 1);

static const guint8 cmd_read_id[]  = { 0x04, 0xfb, 0x9a, 0x8b, 0x00, 0x01 };
static const guint8 cmd_read_img[] = { 0x06, 0xf9, 0x9a, 0x05, 0x14, 0x00 };
static const guint8 cmd_read_9881[] = { 0x04, 0xfb, 0x98, 0x81, 0x00, 0x01 };

struct _FpiDeviceFt9366
{
  FpImageDevice parent;

  FpiSsm          *task_ssm;
  gboolean         deactivating;

  /* parancssor-futtató állapota */
  const Ft9366Cmd *seq;
  gsize            seq_len;
  gsize            seq_pos;

  /* utolsó rövid válasz */
  guint8           reply[16];
  gsize            reply_len;

  /* képek */
  /* a nagy puffereket külön foglaljuk (a GObject példány max. 64 KB lehet) */
  guint8          *raw;         /* RAW_LEN bájt */
  float           *bg;          /* RAW_W * RAW_H */
  guint16         *bg_frames;   /* BG_FRAMES * RAW_W * RAW_H */
  guint32          bg_means[BG_FRAMES];
  guint            bg_count;
  gboolean         have_bg;

  /* a task-SSM eredménye, a done-callbackben jelentjük */
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
/* USB alapműveletek                                                    */
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
/* Parancssor-futtató al-állapotgép (a ft9366_seq.h tábláihoz)          */
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
/* Kép feldolgozása                                                     */
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

/* egy háttérjelölt kép eltárolása */
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

/* a jó (ujj nélküli) képek átlaga lesz a háttér */
static void
bg_finalize (FpiDeviceFt9366 *self)
{
  guint32 max_mean = 0;
  gboolean use[BG_FRAMES] = { FALSE };
  guint used = 0, k;
  gsize i;

  for (k = 0; k < self->bg_count; k++)
    max_mean = MAX (max_mean, self->bg_means[k]);

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
    fp_warn ("background mean %u is low – finger on the sensor during calibration?",
             max_mean);
  fp_dbg ("background: %u/%u frames used, mean %u", used, self->bg_count, max_mean);

  explicit_bzero (self->bg_frames, BG_BYTES);
  self->have_bg = TRUE;
}

/* (x, y) a [0, w) x [0, h) tartományba szorítva */
#define AT(buf, w, h, x, y) \
  (buf)[CLAMP ((gint) (y), 0, (h) - 1) * (w) + CLAMP ((gint) (x), 0, (w) - 1)]

static FpImage *
build_image (FpiDeviceFt9366 *self)
{
  /* statikus pufferek, használat után nullázzuk */
  static float a[IMG_W * IMG_H];
  static float b[IMG_W * IMG_H];
  static double integ[(IMG_W + 1) * (IMG_H + 1)];
  static guint8 small[IMG_W * IMG_H];
  const gsize n = IMG_W * IMG_H;
  const gint r = HP_BOX / 2;
  FpImage *img;
  float lo, hi;
  gint x, y;

  /* 1. háttérlevonás + kivágás */
  for (y = 0; y < IMG_H; y++)
    for (x = 0; x < IMG_W; x++)
      {
        gsize ri = (y + IMG_Y0) * RAW_W + x + IMG_X0;
        float v = (self->raw[2 * ri] << 8) | self->raw[2 * ri + 1];

        if (self->have_bg)
          v -= self->bg[ri];
        a[y * IMG_W + x] = v;
      }

  /* 2. 3x3-as Gauss-simítás (1 2 1 / 2 4 2 / 1 2 1) / 16 */
  for (y = 0; y < IMG_H; y++)
    for (x = 0; x < IMG_W; x++)
      {
        float s =
          1 * AT (a, IMG_W, IMG_H, x - 1, y - 1) + 2 * AT (a, IMG_W, IMG_H, x, y - 1) + 1 * AT (a, IMG_W, IMG_H, x + 1, y - 1) +
          2 * AT (a, IMG_W, IMG_H, x - 1, y)     + 4 * AT (a, IMG_W, IMG_H, x, y)     + 2 * AT (a, IMG_W, IMG_H, x + 1, y) +
          1 * AT (a, IMG_W, IMG_H, x - 1, y + 1) + 2 * AT (a, IMG_W, IMG_H, x, y + 1) + 1 * AT (a, IMG_W, IMG_H, x + 1, y + 1);
        b[y * IMG_W + x] = s / 16.0f;
      }

  /* 3. felüláteresztő: a HP_BOX x HP_BOX ablak átlagának levonása
   *    (integrálképpel; a szélen csak a képen belüli pixelek számítanak) */
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

  /* 4. kontraszt: 2% - 98% percentilis 0..255-re */
  memcpy (b, a, sizeof (a));
  qsort (b, n, sizeof (float), cmp_float);
  lo = b[n / 50];
  hi = b[n - 1 - n / 50];
  if (hi <= lo)
    hi = lo + 1;
  for (gsize i = 0; i < n; i++)
    small[i] = CLAMP ((gint) ((a[i] - lo) * 255.0f / (hi - lo)), 0, 255);

  /* 5. bilineáris nagyítás ENLARGE-szeresre */
  img = fp_image_new (OUT_W, OUT_H);
  for (y = 0; y < OUT_H; y++)
    {
      float sy = CLAMP ((y + 0.5f) / ENLARGE - 0.5f, 0.0f, IMG_H - 1.0f);
      gint y0 = (gint) sy, y1 = MIN (y0 + 1, IMG_H - 1);
      float fy = sy - y0;

      for (x = 0; x < OUT_W; x++)
        {
          float sx = CLAMP ((x + 0.5f) / ENLARGE - 0.5f, 0.0f, IMG_W - 1.0f);
          gint x0 = (gint) sx, x1 = MIN (x0 + 1, IMG_W - 1);
          float fx = sx - x0;
          float v = small[y0 * IMG_W + x0] * (1 - fy) * (1 - fx)
                    + small[y0 * IMG_W + x1] * (1 - fy) * fx
                    + small[y1 * IMG_W + x0] * fy * (1 - fx)
                    + small[y1 * IMG_W + x1] * fy * fx;

          img->data[y * OUT_W + x] = (guint8) CLAMP ((gint) (v + 0.5f), 0, 255);
        }
    }

  /* TODO: polaritás ellenőrzése (FPI_IMAGE_COLORS_INVERTED?) */
  img->flags = 0;

  explicit_bzero (a, sizeof (a));
  explicit_bzero (b, sizeof (b));
  explicit_bzero (integ, sizeof (integ));
  explicit_bzero (small, sizeof (small));

  /* CSAK FEJLESZTÉSHEZ: a libfprintnek átadott kép kimentése.
   * Alapból ki sem fordul: -Dc_args=-DFT9366_DEVEL kell hozzá, így egy
   * kiadott buildben környezeti változóval sem lehet képet lemezre íratni. */
#ifdef FT9366_DEVEL
  {
    const char *dump = g_getenv ("FT9366_DUMP");

    if (dump)
      {
        g_autofree char *hdr = g_strdup_printf ("P5 %d %d 255\n", OUT_W, OUT_H);
        g_autoptr(GByteArray) pgm = g_byte_array_new ();

        g_byte_array_append (pgm, (guint8 *) hdr, strlen (hdr));
        g_byte_array_append (pgm, img->data, OUT_W * OUT_H);
        g_file_set_contents (dump, (char *) pgm->data, pgm->len, NULL);
        fp_dbg ("DEBUG: image dumped to %s", dump);
      }
  }
#endif

  return img;
}

/* ------------------------------------------------------------------ */
/* Közös done-kezelés                                                   */
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

  /* az eredményeket csak itt jelentjük: a libfprint ilyenkor állapotot vált
   * és újabb SSM-et indíthat, ami nem futhat az előzővel egyszerre */
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
/* Megnyitás: ébresztés + chip-azonosító                               */
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

  /* Fail closed: titkosítókulcs nélkül nem dolgozunk */
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
/* Aktiválás: háttérkép felvétele (ujj nélkül!)                        */
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
  bg_reset (self);

  /* TODO: ha a felhasználó épp rajta tartja az ujját, a háttér rossz lesz;
   * a 0x80b8-as próbával előtte ellenőrizni kellene */
  ssm = fpi_ssm_new (FP_DEVICE (dev), act_run_state, ACT_NUM_STATES);
  fpi_ssm_start (ssm, act_done);
}

/* ------------------------------------------------------------------ */
/* Ujjra várás                                                          */
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
/* Kép készítése                                                        */
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
/* Felemelésre várás                                                    */
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
        /* a Windows-driver ilyenkor a 0x9881 1. bitjét állítja be
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
        /* a probe_post-ban az utolsó válaszos parancs a 0x80b8 olvasása */
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
/* Állapotváltás és deaktiválás                                        */
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
  /* ha fut feladat, az a következő ellenőrzési pontnál leáll,
   * és a task_done_common indítja a deaktiválást */
  if (self->task_ssm == NULL)
    start_deactivate (self);
}

/* ------------------------------------------------------------------ */
/* Illesztés (az NBIS helyett)                                          */
/* ------------------------------------------------------------------ */

/* munkaszálon fut: csak a képet olvassa, eszközállapotot nem */
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

  /* A sablon (benne a kép) csak titkosítva kerül az FpPrint-be, így a
   * fprintd már rejtjelezett adatot ír a /var/lib/fprint alá. */
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

/* Visszafejt + deszerializál. Hibánál NULL és @error. */
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
      /* régi, titkosítatlan sablon vagy más kulcs: újra kell regisztrálni */
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
}

static void
fpi_device_ft9366_finalize (GObject *object)
{
  FpiDeviceFt9366 *self = FPI_DEVICE_FT9366 (object);

  /* ujjlenyomat-adat ne maradjon a felszabadított memóriában */
  explicit_bzero (self->raw, RAW_LEN);
  explicit_bzero (self->bg, RAW_W * RAW_H * sizeof (float));
  explicit_bzero (self->bg_frames, BG_BYTES);
  g_clear_pointer (&self->raw, g_free);
  g_clear_pointer (&self->bg, g_free);
  g_clear_pointer (&self->bg_frames, g_free);
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
  dev_class->nr_enroll_stages = 16;   /* a Windows-driver is 16 érintést kér */

  img_class->img_open = dev_open;
  img_class->img_close = dev_close;
  img_class->activate = dev_activate;
  img_class->deactivate = dev_deactivate;
  img_class->change_state = dev_change_state;

  img_class->img_width = OUT_W;
  img_class->img_height = OUT_H;
  /* saját illesztő; a bz3_threshold így nincs használatban */
  img_class->extract_template = ft9366_extract_template;
  img_class->match_template = ft9366_match_template;
}
