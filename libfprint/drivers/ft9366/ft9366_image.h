/*
 * FT9366 image preprocessing, shared by the driver and
 * tools/ft9366/ftm_eval.c, so that offline evaluation sees exactly the
 * images the driver produces.
 *
 * Plain C99, no allocations, no global state.
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* raw frame: 64 x 80 words, 16-bit big-endian, 12 bits used */
#define FT9366_RAW_W   64
#define FT9366_RAW_H   80

/* Crop. Columns 0 and 63 carry no data. Columns 1 and 62 and rows 0-2
 * measure like the rest of the sensor, but widening the crop changes the
 * template format, so 60x77 (columns 2..61, rows 3..79) stays until the
 * full 62x80 has been evaluated with ftm_eval. */
#define FT9366_IMG_X0  2
#define FT9366_IMG_Y0  3
#define FT9366_IMG_W   (FT9366_RAW_W - 2 * FT9366_IMG_X0)
#define FT9366_IMG_H   (FT9366_RAW_H - FT9366_IMG_Y0)

/* A finger lowers a frame's mean by ~200-450, so only background frames
 * within FT9366_BG_TOL of the brightest one are averaged. */
#define FT9366_BG_TOL  60

/* Averages the @count frames (FT9366_RAW_W x FT9366_RAW_H raw values each,
 * with their means in @means) that are within FT9366_BG_TOL of the
 * brightest one into @bg. Returns the number of frames used (>= 1 when
 * @count >= 1). */
unsigned ft9366_bg_average (const uint16_t *frames, const uint32_t *means,
                            unsigned count, float *bg);

/* Turns one frame as sent by the sensor (@raw_be, big-endian 16-bit) into
 * the FT9366_IMG_W x FT9366_IMG_H 8-bit image the matcher uses: background
 * subtraction (@bg may be NULL), crop, 3x3 Gaussian, high-pass, 2nd-98th
 * percentile stretch. Scratch buffers: @work_a and @work_b hold
 * FT9366_IMG_W * FT9366_IMG_H floats, @work_integ
 * (FT9366_IMG_W + 1) * (FT9366_IMG_H + 1) doubles; all are wiped before
 * returning. */
void ft9366_build_image (const uint8_t *raw_be, const float *bg, uint8_t *out,
                         float *work_a, float *work_b, double *work_integ);

#ifdef __cplusplus
}
#endif
