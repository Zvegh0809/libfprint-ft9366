/*
 * Test of the ft9366 extract/match hooks with real GVariants.
 * Includes ft9366.c directly; USB/SSM code is linked but never called.
 *
 *   test_hooks <dir with driver-processed 60x77 PGMs named NNN_<finger>.pgm>
 */
#include "ft9366.c"

#include <dirent.h>
#include <stdio.h>

/* --- minimal stand-ins for the libfprint helpers the hooks use --- */
GQuark fp_device_retry_quark (void) { return g_quark_from_static_string ("fp-device-retry"); }
GQuark fp_device_error_quark (void) { return g_quark_from_static_string ("fp-device-error"); }
GError *fpi_device_retry_new (FpDeviceRetry e) { return g_error_new (FP_DEVICE_RETRY, e, "retry %d", e); }
GError *
fpi_device_error_new_msg (FpDeviceError e, const char *msg, ...)
{
  va_list ap;
  GError *err;
  va_start (ap, msg);
  err = g_error_new_valist (FP_DEVICE_ERROR, e, msg, ap);
  va_end (ap);
  return err;
}

/* verbatim copy of the RAW branch added to fpi_print_add_print() */
static GVariant *
core_append (GVariant *data, GVariant *add)
{
  GVariantBuilder builder;

  g_assert (data == NULL || g_variant_is_of_type (data, G_VARIANT_TYPE ("av")));
  g_variant_builder_init (&builder, G_VARIANT_TYPE ("av"));
  if (data)
    {
      GVariantIter iter;
      GVariant *child;

      g_variant_iter_init (&iter, data);
      while ((child = g_variant_iter_next_value (&iter)))
        {
          g_variant_builder_add_value (&builder, child);
          g_variant_unref (child);
        }
    }
  g_variant_builder_add (&builder, "v", add);
  if (data)
    g_variant_unref (data);
  return g_variant_ref_sink (g_variant_builder_end (&builder));
}

/* same nesting as fp_print_serialize()/fp_print_deserialize() for RAW */
static GVariant *
storage_roundtrip (GVariant *data)
{
  GVariantBuilder b;
  GVariant *outer, *loaded, *print_data, *fp_data;
  GBytes *bytes;

  g_variant_builder_init (&b, G_VARIANT_TYPE ("(sv)"));
  g_variant_builder_add (&b, "s", "ft9366");
  g_variant_builder_add (&b, "v", g_variant_new_variant (data));
  outer = g_variant_ref_sink (g_variant_builder_end (&b));
  bytes = g_variant_get_data_as_bytes (outer);
  loaded = g_variant_ref_sink (g_variant_new_from_bytes (G_VARIANT_TYPE ("(sv)"), bytes, FALSE));
  g_variant_get (loaded, "(&sv)", NULL, &print_data);
  fp_data = g_variant_get_child_value (print_data, 0);
  g_variant_unref (print_data);
  g_variant_unref (loaded);
  g_variant_unref (outer);
  g_bytes_unref (bytes);
  return fp_data;
}

static GVariant *
extract (guint8 *pix)
{
  struct _FpImage img = { 0 };
  g_autoptr(GError) err = NULL;
  GVariant *v;

  img.width = FTM_W;
  img.height = FTM_H;
  img.data = pix;
  v = ft9366_extract_template (NULL, &img, &err);
  if (!v)
    g_error ("extract failed: %s", err->message);
  return g_variant_ref_sink (v);
}

int
main (int argc, char **argv)
{
  static guint8 pix[512][FTM_W * FTM_H];
  char *finger[512];
  int n = 0, i, f, nf = 0;
  char *names[16];
  DIR *d = opendir (argv[1]);
  struct dirent *e;
  GPtrArray *files = g_ptr_array_new_with_free_func (g_free);

  while ((e = readdir (d)))
    if (g_str_has_suffix (e->d_name, ".pgm"))
      g_ptr_array_add (files, g_strdup (e->d_name));
  closedir (d);
  g_ptr_array_sort_values (files, (GCompareFunc) strcmp);

  for (i = 0; i < (int) files->len; i++)
    {
      const char *name = g_ptr_array_index (files, i);
      g_autofree char *path = g_build_filename (argv[1], name, NULL);
      g_autofree char *contents = NULL;
      gsize len;
      char *us = strchr (name, '_');

      g_file_get_contents (path, &contents, &len, NULL);
      memcpy (pix[n], contents + len - FTM_W * FTM_H, FTM_W * FTM_H);
      finger[n] = g_strndup (us + 1, strlen (us + 1) - 4);
      n++;
    }

  /* finger list */
  for (i = 0; i < n; i++)
    {
      gboolean seen = FALSE;
      for (f = 0; f < nf; f++)
        seen |= !strcmp (names[f], finger[i]);
      if (!seen)
        names[nf++] = finger[i];
    }

  /* enrol: first 16 captures of each finger (as nr_enroll_stages = 16) */
  {
    GVariant *enrolled[16] = { 0 };
    int idx[16] = { 0 }, gen = 0, gen_ok = 0, imp = 0, imp_ok = 0;
    gsize bytes_total = 0;

    for (i = 0; i < n; i++)
      for (f = 0; f < nf; f++)
        if (!strcmp (names[f], finger[i]) && idx[f]++ < 16)
          {
            GVariant *t = extract (pix[i]);
            enrolled[f] = core_append (enrolled[f], t);
            g_variant_unref (t);
          }

    for (f = 0; f < nf; f++)
      {
        GVariant *loaded = storage_roundtrip (enrolled[f]);
        g_assert (g_variant_equal (loaded, enrolled[f]));
        bytes_total += g_variant_get_size (loaded);
        g_variant_unref (enrolled[f]);
        enrolled[f] = loaded;
        g_assert (g_variant_n_children (enrolled[f]) == 16);
      }
    printf ("enrolled %d fingers, avg stored size %.0f KB\n", nf, bytes_total / 1024.0 / nf);

    /* verify: remaining captures against every enrolled finger */
    memset (idx, 0, sizeof idx);
    for (i = 0; i < n; i++)
      {
        int own = -1;
        GVariant *probe;

        for (f = 0; f < nf; f++)
          if (!strcmp (names[f], finger[i]))
            own = f;
        if (idx[own]++ < 16)
          continue;
        probe = extract (pix[i]);
        for (f = 0; f < nf; f++)
          {
            g_autoptr(GError) err = NULL;
            FpiMatchResult r = ft9366_match_template (NULL, enrolled[f], probe, &err);

            g_assert (r != FPI_MATCH_ERROR);
            if (f == own)
              {
                gen++;
                gen_ok += r == FPI_MATCH_SUCCESS;
              }
            else
              {
                imp++;
                imp_ok += r == FPI_MATCH_SUCCESS;
              }
          }
        g_variant_unref (probe);
      }
    printf ("genuine accepted %d/%d, impostor accepted %d/%d\n", gen_ok, gen, imp_ok, imp);

    /* error paths */
    {
      static guint8 flat[FTM_W * FTM_H];
      struct _FpImage img = { 0 };
      g_autoptr(GError) err = NULL;
      g_autoptr(GVariant) junk = g_variant_ref_sink (g_variant_new_bytestring ("junk"));
      GVariantBuilder b;
      g_autoptr(GVariant) badav = NULL;
      FpiMatchResult r;

      memset (flat, 128, sizeof flat);
      img.width = FTM_W;
      img.height = FTM_H;
      img.data = flat;
      g_assert_null (ft9366_extract_template (NULL, &img, &err));
      g_assert_error (err, FP_DEVICE_RETRY, FP_DEVICE_RETRY_CENTER_FINGER);
      g_clear_error (&err);

      img.width = 10;
      g_assert_null (ft9366_extract_template (NULL, &img, &err));
      g_assert_error (err, FP_DEVICE_ERROR, FP_DEVICE_ERROR_GENERAL);
      g_clear_error (&err);

      r = ft9366_match_template (NULL, enrolled[0], junk, &err);
      g_assert (r == FPI_MATCH_ERROR);
      g_assert_error (err, FP_DEVICE_ERROR, FP_DEVICE_ERROR_DATA_INVALID);
      g_clear_error (&err);

      g_variant_builder_init (&b, G_VARIANT_TYPE ("av"));
      g_variant_builder_add (&b, "v", g_variant_new_bytestring ("junk"));
      badav = g_variant_ref_sink (g_variant_builder_end (&b));
      {
        g_autoptr(GVariant) probe = extract (pix[0]);
        r = ft9366_match_template (NULL, badav, probe, &err);
      }
      g_assert (r == FPI_MATCH_ERROR);
      g_assert_error (err, FP_DEVICE_ERROR, FP_DEVICE_ERROR_DATA_INVALID);
      printf ("error paths OK\n");
    }

    for (f = 0; f < nf; f++)
      g_variant_unref (enrolled[f]);
  }

  for (i = 0; i < n; i++)
    g_free (finger[i]);
  g_ptr_array_unref (files);
  return 0;
}
