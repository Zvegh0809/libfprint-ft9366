/*
 * FT9366 host-side fingerprint matcher
 *
 * Small-area press sensors (here 60 x 77 px after cropping) give too few
 * minutiae for NBIS/bozorth3. This matcher instead
 *
 *   1. z-normalises the enhanced image,
 *   2. finds SIFT keypoints/descriptors (Lowe, IJCV 60(2), 2004) on a
 *      3x upscaled copy,
 *   3. matches descriptors with Lowe's ratio test,
 *   4. estimates a similarity transform (shift + rotation + scale ~1) with
 *      RANSAC (Fischler & Bolles, CACM 24(6), 1981),
 *   5. scores the pair by normalised cross-correlation of the two images on
 *      their aligned, ridge-bearing overlap.
 *
 * The score is in [-1, 1]; genuine pairs are ~0.9, impostors stay below 0.8
 * on the development set.
 *
 * Plain C99 + libm, no allocations outside ftm_* functions, no global state.
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FTM_W  60
#define FTM_H  77

typedef struct _FtmFeatures FtmFeatures;

/* Build features from an 8-bit enhanced image of FTM_W x FTM_H pixels
 * (the image the ft9366 driver hands to libfprint). Returns NULL on OOM. */
FtmFeatures *ftm_extract (const uint8_t *img);

/* Free features; all buffers are zeroed first. NULL is allowed. */
void ftm_free (FtmFeatures *f);

/* Number of keypoints found (diagnostics). */
int ftm_n_keypoints (const FtmFeatures *f);

/* Similarity score of a probe against one enrolled template image.
 * Returns 0 if no consistent alignment is found. Not symmetric:
 * the probe is warped into the template's frame. */
double ftm_score (const FtmFeatures *probe, const FtmFeatures *tmpl);

/* Fraction of the image that carries ridges (0..1), for capture quality. */
double ftm_coverage (const FtmFeatures *f);

/* Serialisation: version byte, the 8-bit image, keypoints (float32 LE) and
 * descriptors. The image and descriptors are sensitive biometric data:
 * store the result encrypted.
 *
 * ftm_serialized_size() gives the exact buffer size for ftm_serialize().
 * ftm_deserialize() validates the length and rebuilds the derived data;
 * it returns NULL for malformed input or OOM. */
size_t       ftm_serialized_size (const FtmFeatures *f);
void         ftm_serialize (const FtmFeatures *f, uint8_t *buf);
FtmFeatures *ftm_deserialize (const uint8_t *buf, size_t len);

#ifdef __cplusplus
}
#endif
