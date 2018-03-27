/* linux/drivers/media/platform/s5p-jpeg/jpeg-core.c
 *
 * Copyright (c) 2011-2014 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Author: Andrzej Pietrasiewicz <andrzej.p@samsung.com>
 * Author: Jacek Anaszewski <j.anaszewski@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/gfp.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <media/v4l2-mem2mem.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-dma-contig.h>

#include "ast-jpeg-core.h"
#include "ast-jpeg-hw.h"
#include "regs-ast-jpeg.h"

#include <mach/irqs.h>
#include <mach/platform.h>
#include <plat/aspeed.h>
#include <plat/ast-scu.h>


static struct ast_jpeg_fmt sjpeg_formats[] = {
	{
		.name		= "JPEG JFIF",
		.fourcc		= V4L2_PIX_FMT_JPEG,
		.flags		= SJPEG_FMT_FLAG_ENC_CAPTURE |
				  SJPEG_FMT_FLAG_DEC_OUTPUT |
				  SJPEG_FMT_FLAG_S5P |
				  SJPEG_FMT_FLAG_EXYNOS3250 |
				  SJPEG_FMT_FLAG_EXYNOS4,
	},
	{
		.name		= "YUV 4:2:2 packed, YCbYCr",
		.fourcc		= V4L2_PIX_FMT_YUYV,
		.depth		= 16,
		.colplanes	= 1,
		.h_align	= 4,
		.v_align	= 3,
		.flags		= SJPEG_FMT_FLAG_ENC_OUTPUT |
				  SJPEG_FMT_FLAG_DEC_CAPTURE |
				  SJPEG_FMT_FLAG_S5P |
				  SJPEG_FMT_NON_RGB,
		.subsampling	= V4L2_JPEG_CHROMA_SUBSAMPLING_422,
	},
	{
		.name		= "YUV 4:2:2 packed, YCbYCr",
		.fourcc		= V4L2_PIX_FMT_YUYV,
		.depth		= 16,
		.colplanes	= 1,
		.h_align	= 1,
		.v_align	= 0,
		.flags		= SJPEG_FMT_FLAG_ENC_OUTPUT |
				  SJPEG_FMT_FLAG_DEC_CAPTURE |
				  SJPEG_FMT_FLAG_EXYNOS4 |
				  SJPEG_FMT_NON_RGB,
		.subsampling	= V4L2_JPEG_CHROMA_SUBSAMPLING_422,
	},
	{
		.name		= "YUV 4:2:2 packed, YCbYCr",
		.fourcc		= V4L2_PIX_FMT_YUYV,
		.depth		= 16,
		.colplanes	= 1,
		.h_align	= 2,
		.v_align	= 0,
		.flags		= SJPEG_FMT_FLAG_ENC_OUTPUT |
				  SJPEG_FMT_FLAG_DEC_CAPTURE |
				  SJPEG_FMT_FLAG_EXYNOS3250 |
				  SJPEG_FMT_NON_RGB,
		.subsampling	= V4L2_JPEG_CHROMA_SUBSAMPLING_422,
	},
	{
		.name		= "YUV 4:2:2 packed, YCrYCb",
		.fourcc		= V4L2_PIX_FMT_YVYU,
		.depth		= 16,
		.colplanes	= 1,
		.h_align	= 1,
		.v_align	= 0,
		.flags		= SJPEG_FMT_FLAG_ENC_OUTPUT |
				  SJPEG_FMT_FLAG_DEC_CAPTURE |
				  SJPEG_FMT_FLAG_EXYNOS4 |
				  SJPEG_FMT_NON_RGB,
		.subsampling	= V4L2_JPEG_CHROMA_SUBSAMPLING_422,
	},
	{
		.name		= "YUV 4:2:2 packed, YCrYCb",
		.fourcc		= V4L2_PIX_FMT_YVYU,
		.depth		= 16,
		.colplanes	= 1,
		.h_align	= 2,
		.v_align	= 0,
		.flags		= SJPEG_FMT_FLAG_ENC_OUTPUT |
				  SJPEG_FMT_FLAG_DEC_CAPTURE |
				  SJPEG_FMT_FLAG_EXYNOS3250 |
				  SJPEG_FMT_NON_RGB,
		.subsampling	= V4L2_JPEG_CHROMA_SUBSAMPLING_422,
	},
	{
		.name		= "YUV 4:2:2 packed, YCrYCb",
		.fourcc		= V4L2_PIX_FMT_UYVY,
		.depth		= 16,
		.colplanes	= 1,
		.h_align	= 2,
		.v_align	= 0,
		.flags		= SJPEG_FMT_FLAG_ENC_OUTPUT |
				  SJPEG_FMT_FLAG_DEC_CAPTURE |
				  SJPEG_FMT_FLAG_EXYNOS3250 |
				  SJPEG_FMT_NON_RGB,
		.subsampling	= V4L2_JPEG_CHROMA_SUBSAMPLING_422,
	},
	{
		.name		= "YUV 4:2:2 packed, YCrYCb",
		.fourcc		= V4L2_PIX_FMT_VYUY,
		.depth		= 16,
		.colplanes	= 1,
		.h_align	= 2,
		.v_align	= 0,
		.flags		= SJPEG_FMT_FLAG_ENC_OUTPUT |
				  SJPEG_FMT_FLAG_DEC_CAPTURE |
				  SJPEG_FMT_FLAG_EXYNOS3250 |
				  SJPEG_FMT_NON_RGB,
		.subsampling	= V4L2_JPEG_CHROMA_SUBSAMPLING_422,
	},
	{
		.name		= "RGB565",
		.fourcc		= V4L2_PIX_FMT_RGB565,
		.depth		= 16,
		.colplanes	= 1,
		.h_align	= 0,
		.v_align	= 0,
		.flags		= SJPEG_FMT_FLAG_ENC_OUTPUT |
				  SJPEG_FMT_FLAG_DEC_CAPTURE |
				  SJPEG_FMT_FLAG_EXYNOS4 |
				  SJPEG_FMT_RGB,
		.subsampling	= V4L2_JPEG_CHROMA_SUBSAMPLING_444,
	},
	{
		.name		= "YUV 4:2:0 planar, Y/CbCr",
		.fourcc		= V4L2_PIX_FMT_NV12,
		.depth		= 12,
		.colplanes	= 2,
		.h_align	= 1,
		.v_align	= 1,
		.flags		= SJPEG_FMT_FLAG_ENC_OUTPUT |
				  SJPEG_FMT_FLAG_DEC_CAPTURE |
				  SJPEG_FMT_FLAG_EXYNOS4 |
				  SJPEG_FMT_NON_RGB,
		.subsampling	= V4L2_JPEG_CHROMA_SUBSAMPLING_420,
	},
	{
		.name		= "YUV 4:2:0 planar, Y/CbCr",
		.fourcc		= V4L2_PIX_FMT_NV12,
		.depth		= 12,
		.colplanes	= 2,
		.h_align	= 3,
		.v_align	= 3,
		.flags		= SJPEG_FMT_FLAG_ENC_OUTPUT |
				  SJPEG_FMT_FLAG_DEC_CAPTURE |
				  SJPEG_FMT_FLAG_EXYNOS3250 |
				  SJPEG_FMT_NON_RGB,
		.subsampling	= V4L2_JPEG_CHROMA_SUBSAMPLING_420,
	},
	{
		.name		= "YUV 4:2:0 planar, Y/CbCr",
		.fourcc		= V4L2_PIX_FMT_NV12,
		.depth		= 12,
		.colplanes	= 2,
		.h_align	= 4,
		.v_align	= 4,
		.flags		= SJPEG_FMT_FLAG_ENC_OUTPUT |
				  SJPEG_FMT_FLAG_DEC_CAPTURE |
				  SJPEG_FMT_FLAG_S5P |
				  SJPEG_FMT_NON_RGB,
		.subsampling	= V4L2_JPEG_CHROMA_SUBSAMPLING_420,
	},
	{
		.name		= "YUV 4:2:0 planar, Y/CrCb",
		.fourcc		= V4L2_PIX_FMT_NV21,
		.depth		= 12,
		.colplanes	= 2,
		.h_align	= 3,
		.v_align	= 3,
		.flags		= SJPEG_FMT_FLAG_ENC_OUTPUT |
				  SJPEG_FMT_FLAG_DEC_CAPTURE |
				  SJPEG_FMT_FLAG_EXYNOS3250 |
				  SJPEG_FMT_NON_RGB,
		.subsampling	= V4L2_JPEG_CHROMA_SUBSAMPLING_420,
	},
	{
		.name		= "YUV 4:2:0 planar, Y/CrCb",
		.fourcc		= V4L2_PIX_FMT_NV21,
		.depth		= 12,
		.colplanes	= 2,
		.h_align	= 1,
		.v_align	= 1,
		.flags		= SJPEG_FMT_FLAG_ENC_OUTPUT |
				  SJPEG_FMT_FLAG_DEC_CAPTURE |
				  SJPEG_FMT_FLAG_EXYNOS3250 |
				  SJPEG_FMT_FLAG_EXYNOS4 |
				  SJPEG_FMT_NON_RGB,
		.subsampling	= V4L2_JPEG_CHROMA_SUBSAMPLING_420,
	},
	{
		.name		= "YUV 4:2:0 contiguous 3-planar, Y/Cb/Cr",
		.fourcc		= V4L2_PIX_FMT_YUV420,
		.depth		= 12,
		.colplanes	= 3,
		.h_align	= 1,
		.v_align	= 1,
		.flags		= SJPEG_FMT_FLAG_ENC_OUTPUT |
				  SJPEG_FMT_FLAG_DEC_CAPTURE |
				  SJPEG_FMT_FLAG_EXYNOS4 |
				  SJPEG_FMT_NON_RGB,
		.subsampling	= V4L2_JPEG_CHROMA_SUBSAMPLING_420,
	},
	{
		.name		= "YUV 4:2:0 contiguous 3-planar, Y/Cb/Cr",
		.fourcc		= V4L2_PIX_FMT_YUV420,
		.depth		= 12,
		.colplanes	= 3,
		.h_align	= 4,
		.v_align	= 4,
		.flags		= SJPEG_FMT_FLAG_ENC_OUTPUT |
				  SJPEG_FMT_FLAG_DEC_CAPTURE |
				  SJPEG_FMT_FLAG_EXYNOS3250 |
				  SJPEG_FMT_NON_RGB,
		.subsampling	= V4L2_JPEG_CHROMA_SUBSAMPLING_420,
	},
	{
		.name		= "Gray",
		.fourcc		= V4L2_PIX_FMT_GREY,
		.depth		= 8,
		.colplanes	= 1,
		.flags		= SJPEG_FMT_FLAG_ENC_OUTPUT |
				  SJPEG_FMT_FLAG_DEC_CAPTURE |
				  SJPEG_FMT_FLAG_EXYNOS4 |
				  SJPEG_FMT_NON_RGB,
		.subsampling	= V4L2_JPEG_CHROMA_SUBSAMPLING_GRAY,
	},
};
#define SJPEG_NUM_FORMATS ARRAY_SIZE(sjpeg_formats)

static const unsigned char qtbl_luminance[4][64] = {
	{/*level 0 - high compression quality */
		20, 16, 25, 39, 50, 46, 62, 68,
		16, 18, 23, 38, 38, 53, 65, 68,
		25, 23, 31, 38, 53, 65, 68, 68,
		39, 38, 38, 53, 65, 68, 68, 68,
		50, 38, 53, 65, 68, 68, 68, 68,
		46, 53, 65, 68, 68, 68, 68, 68,
		62, 65, 68, 68, 68, 68, 68, 68,
		68, 68, 68, 68, 68, 68, 68, 68
	},
	{/* level 1 */
		16, 11, 11, 16, 23, 27, 31, 30,
		11, 12, 12, 15, 20, 23, 23, 30,
		11, 12, 13, 16, 23, 26, 35, 47,
		16, 15, 16, 23, 26, 37, 47, 64,
		23, 20, 23, 26, 39, 51, 64, 64,
		27, 23, 26, 37, 51, 64, 64, 64,
		31, 23, 35, 47, 64, 64, 64, 64,
		30, 30, 47, 64, 64, 64, 64, 64
	},
	{/* level 2 */
		12,  8,  8, 12, 17, 21, 24, 23,
		 8,  9,  9, 11, 15, 19, 18, 23,
		 8,  9, 10, 12, 19, 20, 27, 36,
		12, 11, 12, 21, 20, 28, 36, 53,
		17, 15, 19, 20, 30, 39, 51, 59,
		21, 19, 20, 28, 39, 51, 59, 59,
		24, 18, 27, 36, 51, 59, 59, 59,
		23, 23, 36, 53, 59, 59, 59, 59
	},
	{/* level 3 - low compression quality */
		 8,  6,  6,  8, 12, 14, 16, 17,
		 6,  6,  6,  8, 10, 13, 12, 15,
		 6,  6,  7,  8, 13, 14, 18, 24,
		 8,  8,  8, 14, 13, 19, 24, 35,
		12, 10, 13, 13, 20, 26, 34, 39,
		14, 13, 14, 19, 26, 34, 39, 39,
		16, 12, 18, 24, 34, 39, 39, 39,
		17, 15, 24, 35, 39, 39, 39, 39
	}
};

static const unsigned char qtbl_chrominance[4][64] = {
	{/*level 0 - high compression quality */
		21, 25, 32, 38, 54, 68, 68, 68,
		25, 28, 24, 38, 54, 68, 68, 68,
		32, 24, 32, 43, 66, 68, 68, 68,
		38, 38, 43, 53, 68, 68, 68, 68,
		54, 54, 66, 68, 68, 68, 68, 68,
		68, 68, 68, 68, 68, 68, 68, 68,
		68, 68, 68, 68, 68, 68, 68, 68,
		68, 68, 68, 68, 68, 68, 68, 68
	},
	{/* level 1 */
		17, 15, 17, 21, 20, 26, 38, 48,
		15, 19, 18, 17, 20, 26, 35, 43,
		17, 18, 20, 22, 26, 30, 46, 53,
		21, 17, 22, 28, 30, 39, 53, 64,
		20, 20, 26, 30, 39, 48, 64, 64,
		26, 26, 30, 39, 48, 63, 64, 64,
		38, 35, 46, 53, 64, 64, 64, 64,
		48, 43, 53, 64, 64, 64, 64, 64
	},
	{/* level 2 */
		13, 11, 13, 16, 20, 20, 29, 37,
		11, 14, 14, 14, 16, 20, 26, 32,
		13, 14, 15, 17, 20, 23, 35, 40,
		16, 14, 17, 21, 23, 30, 40, 50,
		20, 16, 20, 23, 30, 37, 50, 59,
		20, 20, 23, 30, 37, 48, 59, 59,
		29, 26, 35, 40, 50, 59, 59, 59,
		37, 32, 40, 50, 59, 59, 59, 59
	},
	{/* level 3 - low compression quality */
		 9,  8,  9, 11, 14, 17, 19, 24,
		 8, 10,  9, 11, 14, 13, 17, 22,
		 9,  9, 13, 14, 13, 15, 23, 26,
		11, 11, 14, 14, 15, 20, 26, 33,
		14, 14, 13, 15, 20, 24, 33, 39,
		17, 13, 15, 20, 24, 32, 39, 39,
		19, 17, 23, 26, 33, 39, 39, 39,
		24, 22, 26, 33, 39, 39, 39, 39
	}
};

static const unsigned char hdctbl0[16] = {
	0, 1, 5, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0
};

static const unsigned char hdctblg0[12] = {
	0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0xa, 0xb
};
static const unsigned char hactbl0[16] = {
	0, 2, 1, 3, 3, 2, 4, 3, 5, 5, 4, 4, 0, 0, 1, 0x7d
};
static const unsigned char hactblg0[162] = {
	0x01, 0x02, 0x03, 0x00, 0x04, 0x11, 0x05, 0x12,
	0x21, 0x31, 0x41, 0x06, 0x13, 0x51, 0x61, 0x07,
	0x22, 0x71, 0x14, 0x32, 0x81, 0x91, 0xa1, 0x08,
	0x23, 0x42, 0xb1, 0xc1, 0x15, 0x52, 0xd1, 0xf0,
	0x24, 0x33, 0x62, 0x72, 0x82, 0x09, 0x0a, 0x16,
	0x17, 0x18, 0x19, 0x1a, 0x25, 0x26, 0x27, 0x28,
	0x29, 0x2a, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39,
	0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49,
	0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59,
	0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69,
	0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79,
	0x7a, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89,
	0x8a, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98,
	0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7,
	0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6,
	0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3, 0xc4, 0xc5,
	0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2, 0xd3, 0xd4,
	0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xe1, 0xe2,
	0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea,
	0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8,
	0xf9, 0xfa
};

/*
 * Fourcc downgrade schema lookup tables for 422 and 420
 * chroma subsampling - fourcc on each position maps on the
 * fourcc from the table fourcc_to_dwngrd_schema_id which allows
 * to get the most suitable fourcc counterpart for the given
 * downgraded subsampling property.
 */
static const u32 subs422_fourcc_dwngrd_schema[] = {
	V4L2_PIX_FMT_NV16,
	V4L2_PIX_FMT_NV61,
};

static const u32 subs420_fourcc_dwngrd_schema[] = {
	V4L2_PIX_FMT_NV12,
	V4L2_PIX_FMT_NV21,
	V4L2_PIX_FMT_NV12,
	V4L2_PIX_FMT_NV21,
	V4L2_PIX_FMT_NV12,
	V4L2_PIX_FMT_NV21,
	V4L2_PIX_FMT_GREY,
	V4L2_PIX_FMT_GREY,
	V4L2_PIX_FMT_GREY,
	V4L2_PIX_FMT_GREY,
};

/*
 * Lookup table for translation of a fourcc to the position
 * of its downgraded counterpart in the *fourcc_dwngrd_schema
 * tables.
 */
static const u32 fourcc_to_dwngrd_schema_id[] = {
	V4L2_PIX_FMT_YUYV,
	V4L2_PIX_FMT_NV12,
	V4L2_PIX_FMT_NV21,
	V4L2_PIX_FMT_YUV420,
};

static int ast_jpeg_get_dwngrd_sch_id_by_fourcc(u32 fourcc)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(fourcc_to_dwngrd_schema_id); ++i) {
		if (fourcc_to_dwngrd_schema_id[i] == fourcc)
			return i;
	}

	return -EINVAL;
}

static int ast_jpeg_adjust_fourcc_to_subsampling(
					enum v4l2_jpeg_chroma_subsampling subs,
					u32 in_fourcc,
					u32 *out_fourcc,
					struct ast_jpeg_ctx *ctx)
{
	int dwngrd_sch_id;

	if (ctx->subsampling != V4L2_JPEG_CHROMA_SUBSAMPLING_GRAY) {
		dwngrd_sch_id =
			ast_jpeg_get_dwngrd_sch_id_by_fourcc(in_fourcc);
		if (dwngrd_sch_id < 0)
			return -EINVAL;
	}

	switch (ctx->subsampling) {
	case V4L2_JPEG_CHROMA_SUBSAMPLING_GRAY:
		*out_fourcc = V4L2_PIX_FMT_GREY;
		break;
	case V4L2_JPEG_CHROMA_SUBSAMPLING_420:
		if (dwngrd_sch_id >
				ARRAY_SIZE(subs420_fourcc_dwngrd_schema) - 1)
			return -EINVAL;
		*out_fourcc = subs420_fourcc_dwngrd_schema[dwngrd_sch_id];
		break;
	case V4L2_JPEG_CHROMA_SUBSAMPLING_422:
		if (dwngrd_sch_id >
				ARRAY_SIZE(subs422_fourcc_dwngrd_schema) - 1)
			return -EINVAL;
		*out_fourcc = subs422_fourcc_dwngrd_schema[dwngrd_sch_id];
		break;
	default:
		*out_fourcc = V4L2_PIX_FMT_GREY;
		break;
	}

	return 0;
}

static inline struct ast_jpeg_ctx *ctrl_to_ctx(struct v4l2_ctrl *c)
{
	return container_of(c->handler, struct ast_jpeg_ctx, ctrl_handler);
}

static inline struct ast_jpeg_ctx *fh_to_ctx(struct v4l2_fh *fh)
{
	return container_of(fh, struct ast_jpeg_ctx, fh);
}

static int ast_jpeg_to_user_subsampling(struct ast_jpeg_ctx *ctx)
{
	WARN_ON(ctx->subsampling > 3);
	printk("ast_jpeg_to_user_subsampling \n");
#if 0
	switch (ctx->jpeg->variant->version) {
	case SJPEG_S5P:
		if (ctx->subsampling > 2)
			return V4L2_JPEG_CHROMA_SUBSAMPLING_GRAY;
		return ctx->subsampling;
	case SJPEG_EXYNOS3250:
	case SJPEG_EXYNOS5420:
		if (ctx->subsampling > 3)
			return V4L2_JPEG_CHROMA_SUBSAMPLING_411;
		return exynos3250_decoded_subsampling[ctx->subsampling];
	case SJPEG_EXYNOS4:
	case SJPEG_EXYNOS5433:
		if (ctx->subsampling > 2)
			return V4L2_JPEG_CHROMA_SUBSAMPLING_420;
		return exynos4x12_decoded_subsampling[ctx->subsampling];
	default:
		return V4L2_JPEG_CHROMA_SUBSAMPLING_GRAY;
	}
#endif	
}

static inline void ast_jpeg_set_qtbl(void __iomem *regs,
				     const unsigned char *qtbl,
				     unsigned long tab, int len)
{
	int i;

	for (i = 0; i < len; i++)
		writel((unsigned int)qtbl[i], regs + tab + (i * 0x04));
}

static inline void ast_jpeg_set_qtbl_lum(void __iomem *regs, int quality)
{
	/* this driver fills quantisation table 0 with data for luma */
	ast_jpeg_set_qtbl(regs, qtbl_luminance[quality],
			  S5P_JPG_QTBL_CONTENT(0),
			  ARRAY_SIZE(qtbl_luminance[quality]));
}

static inline void ast_jpeg_set_qtbl_chr(void __iomem *regs, int quality)
{
	/* this driver fills quantisation table 1 with data for chroma */
	ast_jpeg_set_qtbl(regs, qtbl_chrominance[quality],
			  S5P_JPG_QTBL_CONTENT(1),
			  ARRAY_SIZE(qtbl_chrominance[quality]));
}

static inline void ast_jpeg_set_htbl(void __iomem *regs,
				     const unsigned char *htbl,
				     unsigned long tab, int len)
{
	int i;

	for (i = 0; i < len; i++)
		writel((unsigned int)htbl[i], regs + tab + (i * 0x04));
}

static inline void ast_jpeg_set_hdctbl(void __iomem *regs)
{
	/* this driver fills table 0 for this component */
	ast_jpeg_set_htbl(regs, hdctbl0, S5P_JPG_HDCTBL(0),
						ARRAY_SIZE(hdctbl0));
}

static inline void ast_jpeg_set_hdctblg(void __iomem *regs)
{
	/* this driver fills table 0 for this component */
	ast_jpeg_set_htbl(regs, hdctblg0, S5P_JPG_HDCTBLG(0),
						ARRAY_SIZE(hdctblg0));
}

static inline void ast_jpeg_set_hactbl(void __iomem *regs)
{
	/* this driver fills table 0 for this component */
	ast_jpeg_set_htbl(regs, hactbl0, S5P_JPG_HACTBL(0),
						ARRAY_SIZE(hactbl0));
}

static inline void ast_jpeg_set_hactblg(void __iomem *regs)
{
	/* this driver fills table 0 for this component */
	ast_jpeg_set_htbl(regs, hactblg0, S5P_JPG_HACTBLG(0),
						ARRAY_SIZE(hactblg0));
}

static int get_byte(struct ast_jpeg_buffer *buf);
static int get_word_be(struct ast_jpeg_buffer *buf, unsigned int *word);
static void skip(struct ast_jpeg_buffer *buf, long len);

/*
 * ============================================================================
 * Device file operations
 * ============================================================================
 */

static int queue_init(void *priv, struct vb2_queue *src_vq,
		      struct vb2_queue *dst_vq);
static struct ast_jpeg_fmt *ast_jpeg_find_format(struct ast_jpeg_ctx *ctx,
				__u32 pixelformat, unsigned int fmt_type);
static int ast_jpeg_controls_create(struct ast_jpeg_ctx *ctx);

static int ast_jpeg_open(struct file *file)
{
	struct ast_jpeg *jpeg = video_drvdata(file);
	struct video_device *vfd = video_devdata(file);
	struct ast_jpeg_ctx *ctx;
	struct ast_jpeg_fmt *out_fmt, *cap_fmt;
	int ret = 0;
	printk("ast_jpeg_open \n");
	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	if (mutex_lock_interruptible(&jpeg->lock)) {
		ret = -ERESTARTSYS;
		goto free;
	}

	v4l2_fh_init(&ctx->fh, vfd);
	/* Use separate control handler per file handle */
	ctx->fh.ctrl_handler = &ctx->ctrl_handler;
	file->private_data = &ctx->fh;
	v4l2_fh_add(&ctx->fh);

	ctx->jpeg = jpeg;
	if (vfd == jpeg->vfd_encoder) {
		out_fmt = ast_jpeg_find_format(ctx, V4L2_PIX_FMT_RGB565,
							FMT_TYPE_OUTPUT);
		cap_fmt = ast_jpeg_find_format(ctx, V4L2_PIX_FMT_JPEG,
							FMT_TYPE_CAPTURE);
	}

	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(jpeg->m2m_dev, ctx, queue_init);
	if (IS_ERR(ctx->fh.m2m_ctx)) {
		ret = PTR_ERR(ctx->fh.m2m_ctx);
		goto error;
	}

	ctx->out_q.fmt = out_fmt;
	ctx->cap_q.fmt = cap_fmt;

	ret = ast_jpeg_controls_create(ctx);
	if (ret < 0)
		goto error;

	mutex_unlock(&jpeg->lock);
	return 0;

error:
	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);
	mutex_unlock(&jpeg->lock);
free:
	kfree(ctx);
	return ret;
}

static int ast_jpeg_release(struct file *file)
{
	struct ast_jpeg *jpeg = video_drvdata(file);
	struct ast_jpeg_ctx *ctx = fh_to_ctx(file->private_data);

	printk("ast_jpeg_release \n");

	mutex_lock(&jpeg->lock);
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
	v4l2_ctrl_handler_free(&ctx->ctrl_handler);
	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);
	mutex_unlock(&jpeg->lock);

	return 0;
}

static const struct v4l2_file_operations ast_jpeg_fops = {
	.owner		= THIS_MODULE,
	.open		= ast_jpeg_open,
	.release	= ast_jpeg_release,
	.poll		= v4l2_m2m_fop_poll,
	.unlocked_ioctl	= video_ioctl2,
	.mmap		= v4l2_m2m_fop_mmap,
};

/*
 * ============================================================================
 * video ioctl operations
 * ============================================================================
 */

static int get_byte(struct ast_jpeg_buffer *buf)
{
	if (buf->curr >= buf->size)
		return -1;

	return ((unsigned char *)buf->data)[buf->curr++];
}

static int get_word_be(struct ast_jpeg_buffer *buf, unsigned int *word)
{
	unsigned int temp;
	int byte;

	byte = get_byte(buf);
	if (byte == -1)
		return -1;
	temp = byte << 8;
	byte = get_byte(buf);
	if (byte == -1)
		return -1;
	*word = (unsigned int)byte | temp;
	return 0;
}

static void skip(struct ast_jpeg_buffer *buf, long len)
{
	if (len <= 0)
		return;

	while (len--)
		get_byte(buf);
}

static bool ast_jpeg_parse_hdr(struct ast_jpeg_q_data *result,
			       unsigned long buffer, unsigned long size,
			       struct ast_jpeg_ctx *ctx)
{
	int c, components = 0, notfound, n_dht = 0, n_dqt = 0;
	unsigned int height, width, word, subsampling = 0, sos = 0, sof = 0,
		     sof_len = 0;
	unsigned int dht[S5P_JPEG_MAX_MARKER], dht_len[S5P_JPEG_MAX_MARKER],
		     dqt[S5P_JPEG_MAX_MARKER], dqt_len[S5P_JPEG_MAX_MARKER];
	long length;
	struct ast_jpeg_buffer jpeg_buffer;

	jpeg_buffer.size = size;
	jpeg_buffer.data = buffer;
	jpeg_buffer.curr = 0;

	notfound = 1;
	while (notfound || !sos) {
		c = get_byte(&jpeg_buffer);
		if (c == -1)
			return false;
		if (c != 0xff)
			continue;
		do
			c = get_byte(&jpeg_buffer);
		while (c == 0xff);
		if (c == -1)
			return false;
		if (c == 0)
			continue;
		length = 0;
		switch (c) {
		/* SOF0: baseline JPEG */
		case SOF0:
			if (get_word_be(&jpeg_buffer, &word))
				break;
			length = (long)word - 2;
			if (!length)
				return false;
			sof = jpeg_buffer.curr; /* after 0xffc0 */
			sof_len = length;
			if (get_byte(&jpeg_buffer) == -1)
				break;
			if (get_word_be(&jpeg_buffer, &height))
				break;
			if (get_word_be(&jpeg_buffer, &width))
				break;
			components = get_byte(&jpeg_buffer);
			if (components == -1)
				break;

			if (components == 1) {
				subsampling = 0x33;
			} else {
				skip(&jpeg_buffer, 1);
				subsampling = get_byte(&jpeg_buffer);
				skip(&jpeg_buffer, 1);
			}
			if (components > 3)
				return false;
			skip(&jpeg_buffer, components * 2);
			notfound = 0;
			break;

		case DQT:
			if (get_word_be(&jpeg_buffer, &word))
				break;
			length = (long)word - 2;
			if (!length)
				return false;
			if (n_dqt >= S5P_JPEG_MAX_MARKER)
				return false;
			dqt[n_dqt] = jpeg_buffer.curr; /* after 0xffdb */
			dqt_len[n_dqt++] = length;
			skip(&jpeg_buffer, length);
			break;

		case DHT:
			if (get_word_be(&jpeg_buffer, &word))
				break;
			length = (long)word - 2;
			if (!length)
				return false;
			if (n_dht >= S5P_JPEG_MAX_MARKER)
				return false;
			dht[n_dht] = jpeg_buffer.curr; /* after 0xffc4 */
			dht_len[n_dht++] = length;
			skip(&jpeg_buffer, length);
			break;

		case SOS:
			sos = jpeg_buffer.curr - 2; /* 0xffda */
			break;

		/* skip payload-less markers */
		case RST ... RST + 7:
		case SOI:
		case EOI:
		case TEM:
			break;

		/* skip uninteresting payload markers */
		default:
			if (get_word_be(&jpeg_buffer, &word))
				break;
			length = (long)word - 2;
			skip(&jpeg_buffer, length);
			break;
		}
	}
	result->w = width;
	result->h = height;
	result->sos = sos;
	result->dht.n = n_dht;
	while (n_dht--) {
		result->dht.marker[n_dht] = dht[n_dht];
		result->dht.len[n_dht] = dht_len[n_dht];
	}
	result->dqt.n = n_dqt;
	while (n_dqt--) {
		result->dqt.marker[n_dqt] = dqt[n_dqt];
		result->dqt.len[n_dqt] = dqt_len[n_dqt];
	}
	result->sof = sof;
	result->sof_len = sof_len;
	result->size = result->components = components;

	switch (subsampling) {
	case 0x11:
		ctx->subsampling = V4L2_JPEG_CHROMA_SUBSAMPLING_444;
		break;
	case 0x21:
		ctx->subsampling = V4L2_JPEG_CHROMA_SUBSAMPLING_422;
		break;
	case 0x22:
		ctx->subsampling = V4L2_JPEG_CHROMA_SUBSAMPLING_420;
		break;
	case 0x33:
		ctx->subsampling = V4L2_JPEG_CHROMA_SUBSAMPLING_GRAY;
		break;
	default:
		return false;
	}

	return !notfound && sos;
}

static int ast_jpeg_querycap(struct file *file, void *priv,
			   struct v4l2_capability *cap)
{
	struct ast_jpeg_ctx *ctx = fh_to_ctx(priv);
	printk("ast_jpeg_querycap \n");

		strlcpy(cap->driver, "ast-jpeg",
			sizeof(cap->driver));
		strlcpy(cap->card, "ast-jpeg-encoder",
			sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s",
		 dev_name(ctx->jpeg->dev));
	cap->device_caps = V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_M2M;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;

	return 0;
}

static int enum_fmt(struct ast_jpeg_fmt *sjpeg_formats, int n,
		    struct v4l2_fmtdesc *f, u32 type)
{
	int i, num = 0;

	for (i = 0; i < n; ++i) {
		if (sjpeg_formats[i].flags & type) {
			/* index-th format of type type found ? */
			if (num == f->index)
				break;
			/* Correct type but haven't reached our index yet,
			 * just increment per-type index
			 */
			++num;
		}
	}

	/* Format not found */
	if (i >= n)
		return -EINVAL;

	strlcpy(f->description, sjpeg_formats[i].name, sizeof(f->description));
	f->pixelformat = sjpeg_formats[i].fourcc;

	return 0;
}

static int ast_jpeg_enum_fmt_vid_cap(struct file *file, void *priv,
				   struct v4l2_fmtdesc *f)
{
	struct ast_jpeg_ctx *ctx = fh_to_ctx(priv);
	printk("ast_jpeg_enum_fmt_vid_cap \n");
	return enum_fmt(sjpeg_formats, SJPEG_NUM_FORMATS, f,
				SJPEG_FMT_FLAG_ENC_CAPTURE);
}

static int ast_jpeg_enum_fmt_vid_out(struct file *file, void *priv,
				   struct v4l2_fmtdesc *f)
{
	struct ast_jpeg_ctx *ctx = fh_to_ctx(priv);

	return enum_fmt(sjpeg_formats, SJPEG_NUM_FORMATS, f,
			SJPEG_FMT_FLAG_ENC_OUTPUT);

}

static struct ast_jpeg_q_data *get_q_data(struct ast_jpeg_ctx *ctx,
					  enum v4l2_buf_type type)
{
	if (type == V4L2_BUF_TYPE_VIDEO_OUTPUT)
		return &ctx->out_q;
	if (type == V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return &ctx->cap_q;

	return NULL;
}

static int ast_jpeg_g_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct vb2_queue *vq;
	struct ast_jpeg_q_data *q_data = NULL;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	struct ast_jpeg_ctx *ct = fh_to_ctx(priv);

	vq = v4l2_m2m_get_vq(ct->fh.m2m_ctx, f->type);
	if (!vq)
		return -EINVAL;

	if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE &&
	    ct->mode == S5P_JPEG_DECODE && !ct->hdr_parsed)
		return -EINVAL;
	q_data = get_q_data(ct, f->type);
	BUG_ON(q_data == NULL);

	pix->width = q_data->w;
	pix->height = q_data->h;
	pix->field = V4L2_FIELD_NONE;
	pix->pixelformat = q_data->fmt->fourcc;
	pix->bytesperline = 0;
	if (q_data->fmt->fourcc != V4L2_PIX_FMT_JPEG) {
		u32 bpl = q_data->w;

		if (q_data->fmt->colplanes == 1)
			bpl = (bpl * q_data->fmt->depth) >> 3;
		pix->bytesperline = bpl;
	}
	pix->sizeimage = q_data->size;

	return 0;
}

static struct ast_jpeg_fmt *ast_jpeg_find_format(struct ast_jpeg_ctx *ctx,
				u32 pixelformat, unsigned int fmt_type)
{
	unsigned int k, fmt_flag;

		fmt_flag = (fmt_type == FMT_TYPE_OUTPUT) ?
				SJPEG_FMT_FLAG_ENC_OUTPUT :
				SJPEG_FMT_FLAG_ENC_CAPTURE;

	for (k = 0; k < ARRAY_SIZE(sjpeg_formats); k++) {
		struct ast_jpeg_fmt *fmt = &sjpeg_formats[k];

		if (fmt->fourcc == pixelformat &&
		    fmt->flags & fmt_flag &&
		    fmt->flags & ctx->jpeg->variant->fmt_ver_flag) {
			return fmt;
		}
	}

	return NULL;
}

static void jpeg_bound_align_image(struct ast_jpeg_ctx *ctx,
				   u32 *w, unsigned int wmin, unsigned int wmax,
				   unsigned int walign,
				   u32 *h, unsigned int hmin, unsigned int hmax,
				   unsigned int halign)
{
	int width, height, w_step, h_step;

	width = *w;
	height = *h;

	w_step = 1 << walign;
	h_step = 1 << halign;

	if (ctx->jpeg->variant->hw3250_compat) {
		/*
		 * Rightmost and bottommost pixels are cropped by the
		 * Exynos3250/compatible JPEG IP for RGB formats, for the
		 * specific width and height values respectively. This
		 * assignment will result in v4l_bound_align_image returning
		 * dimensions reduced by 1 for the aforementioned cases.
		 */
		if (w_step == 4 && ((width & 3) == 1)) {
			wmax = width;
			hmax = height;
		}
	}

	v4l_bound_align_image(w, wmin, wmax, walign, h, hmin, hmax, halign, 0);

	if (*w < width && (*w + w_step) < wmax)
		*w += w_step;
	if (*h < height && (*h + h_step) < hmax)
		*h += h_step;
}

static int vidioc_try_fmt(struct v4l2_format *f, struct ast_jpeg_fmt *fmt,
			  struct ast_jpeg_ctx *ctx, int q_type)
{
	struct v4l2_pix_format *pix = &f->fmt.pix;

	if (pix->field == V4L2_FIELD_ANY)
		pix->field = V4L2_FIELD_NONE;
	else if (pix->field != V4L2_FIELD_NONE)
		return -EINVAL;

	/* V4L2 specification suggests the driver corrects the format struct
	 * if any of the dimensions is unsupported
	 */
	if (q_type == FMT_TYPE_OUTPUT)
		jpeg_bound_align_image(ctx, &pix->width, S5P_JPEG_MIN_WIDTH,
				       S5P_JPEG_MAX_WIDTH, 0,
				       &pix->height, S5P_JPEG_MIN_HEIGHT,
				       S5P_JPEG_MAX_HEIGHT, 0);
	else
		jpeg_bound_align_image(ctx, &pix->width, S5P_JPEG_MIN_WIDTH,
				       S5P_JPEG_MAX_WIDTH, fmt->h_align,
				       &pix->height, S5P_JPEG_MIN_HEIGHT,
				       S5P_JPEG_MAX_HEIGHT, fmt->v_align);

	if (fmt->fourcc == V4L2_PIX_FMT_JPEG) {
		if (pix->sizeimage <= 0)
			pix->sizeimage = PAGE_SIZE;
		pix->bytesperline = 0;
	} else {
		u32 bpl = pix->bytesperline;

		if (fmt->colplanes > 1 && bpl < pix->width)
			bpl = pix->width; /* planar */

		if (fmt->colplanes == 1 && /* packed */
		    (bpl << 3) / fmt->depth < pix->width)
			bpl = (pix->width * fmt->depth) >> 3;

		pix->bytesperline = bpl;
		pix->sizeimage = (pix->width * pix->height * fmt->depth) >> 3;
	}

	return 0;
}

static int ast_jpeg_try_fmt_vid_cap(struct file *file, void *priv,
				  struct v4l2_format *f)
{
	struct ast_jpeg_ctx *ctx = fh_to_ctx(priv);
	struct v4l2_pix_format *pix = &f->fmt.pix;
	struct ast_jpeg_fmt *fmt;
	int ret;
printk("ast_jpeg_try_fmt_vid_cap \n");
	fmt = ast_jpeg_find_format(ctx, f->fmt.pix.pixelformat,
						FMT_TYPE_CAPTURE);
	if (!fmt) {
		v4l2_err(&ctx->jpeg->v4l2_dev,
			 "Fourcc format (0x%08x) invalid.\n",
			 f->fmt.pix.pixelformat);
		return -EINVAL;
	}

	/*
	 * Decompression of a JPEG file with 4:2:0 subsampling and odd
	 * width to the YUV 4:2:0 compliant formats produces a raw image
	 * with broken luma component. Adjust capture format to RGB565
	 * in such a case.
	 */
	if (ctx->subsampling == V4L2_JPEG_CHROMA_SUBSAMPLING_420 &&
	    (ctx->out_q.w & 1) &&
	    (pix->pixelformat == V4L2_PIX_FMT_NV12 ||
	     pix->pixelformat == V4L2_PIX_FMT_NV21 ||
	     pix->pixelformat == V4L2_PIX_FMT_YUV420)) {
		pix->pixelformat = V4L2_PIX_FMT_RGB565;
		fmt = ast_jpeg_find_format(ctx, pix->pixelformat,
							FMT_TYPE_CAPTURE);
	}

exit:
	return vidioc_try_fmt(f, fmt, ctx, FMT_TYPE_CAPTURE);
}

static int ast_jpeg_try_fmt_vid_out(struct file *file, void *priv,
				  struct v4l2_format *f)
{
	struct ast_jpeg_ctx *ctx = fh_to_ctx(priv);
	struct ast_jpeg_fmt *fmt;

	fmt = ast_jpeg_find_format(ctx, f->fmt.pix.pixelformat,
						FMT_TYPE_OUTPUT);
	if (!fmt) {
		v4l2_err(&ctx->jpeg->v4l2_dev,
			 "Fourcc format (0x%08x) invalid.\n",
			 f->fmt.pix.pixelformat);
		return -EINVAL;
	}

	return vidioc_try_fmt(f, fmt, ctx, FMT_TYPE_OUTPUT);
}

static int exynos4_jpeg_get_output_buffer_size(struct ast_jpeg_ctx *ctx,
						struct v4l2_format *f,
						int fmt_depth)
{
	struct v4l2_pix_format *pix = &f->fmt.pix;
	u32 pix_fmt = f->fmt.pix.pixelformat;
	int w = pix->width, h = pix->height, wh_align;
	int padding = 0;

	if (pix_fmt == V4L2_PIX_FMT_RGB32 ||
	    pix_fmt == V4L2_PIX_FMT_RGB565 ||
	    pix_fmt == V4L2_PIX_FMT_NV24 ||
	    pix_fmt == V4L2_PIX_FMT_NV42 ||
	    pix_fmt == V4L2_PIX_FMT_NV12 ||
	    pix_fmt == V4L2_PIX_FMT_NV21 ||
	    pix_fmt == V4L2_PIX_FMT_YUV420)
		wh_align = 4;
	else
		wh_align = 1;

	jpeg_bound_align_image(ctx, &w, S5P_JPEG_MIN_WIDTH,
			       S5P_JPEG_MAX_WIDTH, wh_align,
			       &h, S5P_JPEG_MIN_HEIGHT,
			       S5P_JPEG_MAX_HEIGHT, wh_align);

	if (ctx->jpeg->variant->version == SJPEG_EXYNOS4)
		padding = PAGE_SIZE;

	return (w * h * fmt_depth >> 3) + padding;
}

static int ast_jpeg_s_fmt(struct ast_jpeg_ctx *ct, struct v4l2_format *f)
{
	struct vb2_queue *vq;
	struct ast_jpeg_q_data *q_data = NULL;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	struct v4l2_ctrl *ctrl_subs;
	struct v4l2_rect scale_rect;
	unsigned int f_type;
	printk("ast_jpeg_s_fmt \n");
#if 0	
	vq = v4l2_m2m_get_vq(ct->fh.m2m_ctx, f->type);
	if (!vq)
		return -EINVAL;

	q_data = get_q_data(ct, f->type);
	BUG_ON(q_data == NULL);

	if (vb2_is_busy(vq)) {
		v4l2_err(&ct->jpeg->v4l2_dev, "%s queue busy\n", __func__);
		return -EBUSY;
	}

	f_type = V4L2_TYPE_IS_OUTPUT(f->type) ?
			FMT_TYPE_OUTPUT : FMT_TYPE_CAPTURE;

	q_data->fmt = ast_jpeg_find_format(ct, pix->pixelformat, f_type);
	q_data->w = pix->width;
	q_data->h = pix->height;
	if (q_data->fmt->fourcc != V4L2_PIX_FMT_JPEG) {
		/*
		 * During encoding Exynos4x12 SoCs access wider memory area
		 * than it results from Image_x and Image_y values written to
		 * the JPEG_IMAGE_SIZE register. In order to avoid sysmmu
		 * page fault calculate proper buffer size in such a case.
		 */
		if (ct->jpeg->variant->hw_ex4_compat &&
		    f_type == FMT_TYPE_OUTPUT && ct->mode == S5P_JPEG_ENCODE)
			q_data->size = exynos4_jpeg_get_output_buffer_size(ct,
							f,
							q_data->fmt->depth);
		else
			q_data->size = q_data->w * q_data->h *
						q_data->fmt->depth >> 3;
	} else {
		q_data->size = pix->sizeimage;
	}

	if (f_type == FMT_TYPE_OUTPUT) {
		ctrl_subs = v4l2_ctrl_find(&ct->ctrl_handler,
					V4L2_CID_JPEG_CHROMA_SUBSAMPLING);
		if (ctrl_subs)
			v4l2_ctrl_s_ctrl(ctrl_subs, q_data->fmt->subsampling);
		ct->crop_altered = false;
	}

	/*
	 * For decoding init crop_rect with capture buffer dimmensions which
	 * contain aligned dimensions of the input JPEG image and do it only
	 * if crop rectangle hasn't been altered by the user space e.g. with
	 * S_SELECTION ioctl. For encoding assign output buffer dimensions.
	 */
	if (!ct->crop_altered &&
	    ((ct->mode == S5P_JPEG_DECODE && f_type == FMT_TYPE_CAPTURE) ||
	     (ct->mode == S5P_JPEG_ENCODE && f_type == FMT_TYPE_OUTPUT))) {
		ct->crop_rect.width = pix->width;
		ct->crop_rect.height = pix->height;
	}
#endif
	return 0;
}

static int ast_jpeg_s_fmt_vid_cap(struct file *file, void *priv,
				struct v4l2_format *f)
{
	int ret;
	printk("ast_jpeg_s_fmt_vid_cap \n");
	ret = ast_jpeg_try_fmt_vid_cap(file, priv, f);
	if (ret)
		return ret;

	return ast_jpeg_s_fmt(fh_to_ctx(priv), f);
}

static int ast_jpeg_s_fmt_vid_out(struct file *file, void *priv,
				struct v4l2_format *f)
{
	int ret;
	printk("ast_jpeg_s_fmt_vid_out \n");

	ret = ast_jpeg_try_fmt_vid_out(file, priv, f);
	if (ret)
		return ret;

	return ast_jpeg_s_fmt(fh_to_ctx(priv), f);
}

/* Return 1 if rectangle a is enclosed in rectangle b, or 0 otherwise. */
static int enclosed_rectangle(struct v4l2_rect *a, struct v4l2_rect *b)
{
	if (a->left < b->left || a->top < b->top)
		return 0;
	if (a->left + a->width > b->left + b->width)
		return 0;
	if (a->top + a->height > b->top + b->height)
		return 0;

	return 1;
}

/*
 * V4L2 controls
 */

static int ast_jpeg_g_selection(struct file *file, void *priv,
			 struct v4l2_selection *s)
{
	struct ast_jpeg_ctx *ctx = fh_to_ctx(priv);

	if (s->type != V4L2_BUF_TYPE_VIDEO_OUTPUT &&
	    s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	/* For JPEG blob active == default == bounds */
	switch (s->target) {
	case V4L2_SEL_TGT_CROP:
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_COMPOSE_DEFAULT:
		s->r.width = ctx->out_q.w;
		s->r.height = ctx->out_q.h;
		s->r.left = 0;
		s->r.top = 0;
		break;
	case V4L2_SEL_TGT_COMPOSE:
	case V4L2_SEL_TGT_COMPOSE_BOUNDS:
	case V4L2_SEL_TGT_COMPOSE_PADDED:
		s->r.width = ctx->crop_rect.width;
		s->r.height =  ctx->crop_rect.height;
		s->r.left = ctx->crop_rect.left;
		s->r.top = ctx->crop_rect.top;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

/*
 * V4L2 controls
 */
static int ast_jpeg_s_selection(struct file *file, void *fh,
				  struct v4l2_selection *s)
{
	struct ast_jpeg_ctx *ctx = fh_to_ctx(file->private_data);
	struct v4l2_rect *rect = &s->r;
	int ret = -EINVAL;
	printk("ast_jpeg_s_selection\n");
#if 0
	if (s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	if (s->target == V4L2_SEL_TGT_COMPOSE) {
		if (ctx->mode != S5P_JPEG_DECODE)
			return -EINVAL;
		if (ctx->jpeg->variant->hw3250_compat)
			ret = exynos3250_jpeg_try_downscale(ctx, rect);
	} else if (s->target == V4L2_SEL_TGT_CROP) {
		if (ctx->mode != S5P_JPEG_ENCODE)
			return -EINVAL;
		if (ctx->jpeg->variant->hw3250_compat)
			ret = exynos3250_jpeg_try_crop(ctx, rect);
	}
#endif
	return ret;
}

static int ast_jpeg_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ast_jpeg_ctx *ctx = ctrl_to_ctx(ctrl);
	struct ast_jpeg *jpeg = ctx->jpeg;
	unsigned long flags;

	switch (ctrl->id) {
	case V4L2_CID_JPEG_CHROMA_SUBSAMPLING:
		spin_lock_irqsave(&jpeg->slock, flags);
		ctrl->val = ast_jpeg_to_user_subsampling(ctx);
		spin_unlock_irqrestore(&jpeg->slock, flags);
		break;
	}

	return 0;
}

static int ast_jpeg_adjust_subs_ctrl(struct ast_jpeg_ctx *ctx, int *ctrl_val)
{
	switch (ctx->jpeg->variant->version) {
	case SJPEG_S5P:
		return 0;
	case SJPEG_EXYNOS3250:
	case SJPEG_EXYNOS5420:
		/*
		 * The exynos3250/compatible device can produce JPEG image only
		 * of 4:4:4 subsampling when given RGB32 source image.
		 */
		if (ctx->out_q.fmt->fourcc == V4L2_PIX_FMT_RGB32)
			*ctrl_val = 0;
		break;
	case SJPEG_EXYNOS4:
		/*
		 * The exynos4x12 device requires input raw image fourcc
		 * to be V4L2_PIX_FMT_GREY if gray jpeg format
		 * is to be set.
		 */
		if (ctx->out_q.fmt->fourcc != V4L2_PIX_FMT_GREY &&
		    *ctrl_val == V4L2_JPEG_CHROMA_SUBSAMPLING_GRAY)
			return -EINVAL;
		break;
	}

	/*
	 * The exynos4x12 and exynos3250/compatible devices require resulting
	 * jpeg subsampling not to be lower than the input raw image
	 * subsampling.
	 */
	if (ctx->out_q.fmt->subsampling > *ctrl_val)
		*ctrl_val = ctx->out_q.fmt->subsampling;

	return 0;
}

static int ast_jpeg_try_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ast_jpeg_ctx *ctx = ctrl_to_ctx(ctrl);
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&ctx->jpeg->slock, flags);

	if (ctrl->id == V4L2_CID_JPEG_CHROMA_SUBSAMPLING)
		ret = ast_jpeg_adjust_subs_ctrl(ctx, &ctrl->val);

	spin_unlock_irqrestore(&ctx->jpeg->slock, flags);
	return ret;
}

static int ast_jpeg_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ast_jpeg_ctx *ctx = ctrl_to_ctx(ctrl);
	unsigned long flags;

	spin_lock_irqsave(&ctx->jpeg->slock, flags);

	switch (ctrl->id) {
	case V4L2_CID_JPEG_COMPRESSION_QUALITY:
		ctx->compr_quality = ctrl->val;
		break;
	case V4L2_CID_JPEG_RESTART_INTERVAL:
		ctx->restart_interval = ctrl->val;
		break;
	case V4L2_CID_JPEG_CHROMA_SUBSAMPLING:
		ctx->subsampling = ctrl->val;
		break;
	}

	spin_unlock_irqrestore(&ctx->jpeg->slock, flags);
	return 0;
}

static const struct v4l2_ctrl_ops ast_jpeg_ctrl_ops = {
	.g_volatile_ctrl	= ast_jpeg_g_volatile_ctrl,
	.try_ctrl		= ast_jpeg_try_ctrl,
	.s_ctrl			= ast_jpeg_s_ctrl,
};

static int ast_jpeg_controls_create(struct ast_jpeg_ctx *ctx)
{
	unsigned int mask = ~0x27; /* 444, 422, 420, GRAY */
	struct v4l2_ctrl *ctrl;
	int ret;

	v4l2_ctrl_handler_init(&ctx->ctrl_handler, 3);

		v4l2_ctrl_new_std(&ctx->ctrl_handler, &ast_jpeg_ctrl_ops,
				  V4L2_CID_JPEG_COMPRESSION_QUALITY,
				  0, 3, 1, S5P_JPEG_COMPR_QUAL_WORST);

		v4l2_ctrl_new_std(&ctx->ctrl_handler, &ast_jpeg_ctrl_ops,
				  V4L2_CID_JPEG_RESTART_INTERVAL,
				  0, 3, 0xffff, 0);
		if (ctx->jpeg->variant->version == SJPEG_S5P)
			mask = ~0x06; /* 422, 420 */

	ctrl = v4l2_ctrl_new_std_menu(&ctx->ctrl_handler, &ast_jpeg_ctrl_ops,
				      V4L2_CID_JPEG_CHROMA_SUBSAMPLING,
				      V4L2_JPEG_CHROMA_SUBSAMPLING_GRAY, mask,
				      V4L2_JPEG_CHROMA_SUBSAMPLING_422);

	if (ctx->ctrl_handler.error) {
		ret = ctx->ctrl_handler.error;
		goto error_free;
	}

	ret = v4l2_ctrl_handler_setup(&ctx->ctrl_handler);
	if (ret < 0)
		goto error_free;

	return ret;

error_free:
	v4l2_ctrl_handler_free(&ctx->ctrl_handler);
	return ret;
}

static const struct v4l2_ioctl_ops ast_jpeg_ioctl_ops = {
	.vidioc_querycap		= ast_jpeg_querycap,

	.vidioc_enum_fmt_vid_cap	= ast_jpeg_enum_fmt_vid_cap,
	.vidioc_enum_fmt_vid_out	= ast_jpeg_enum_fmt_vid_out,

	.vidioc_g_fmt_vid_cap		= ast_jpeg_g_fmt,
	.vidioc_g_fmt_vid_out		= ast_jpeg_g_fmt,

	.vidioc_try_fmt_vid_cap		= ast_jpeg_try_fmt_vid_cap,
	.vidioc_try_fmt_vid_out		= ast_jpeg_try_fmt_vid_out,

	.vidioc_s_fmt_vid_cap		= ast_jpeg_s_fmt_vid_cap,
	.vidioc_s_fmt_vid_out		= ast_jpeg_s_fmt_vid_out,

	.vidioc_reqbufs			= v4l2_m2m_ioctl_reqbufs,
	.vidioc_querybuf		= v4l2_m2m_ioctl_querybuf,
	.vidioc_qbuf			= v4l2_m2m_ioctl_qbuf,
	.vidioc_dqbuf			= v4l2_m2m_ioctl_dqbuf,

	.vidioc_streamon		= v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff		= v4l2_m2m_ioctl_streamoff,

	.vidioc_g_selection		= ast_jpeg_g_selection,
	.vidioc_s_selection		= ast_jpeg_s_selection,
};

/*
 * ============================================================================
 * mem2mem callbacks
 * ============================================================================
 */

static void ast_jpeg_device_run(void *priv)
{
	struct ast_jpeg_ctx *ctx = priv;
	struct ast_jpeg *jpeg = ctx->jpeg;
	struct vb2_buffer *src_buf, *dst_buf;
	unsigned long src_addr, dst_addr, flags;

	spin_lock_irqsave(&ctx->jpeg->slock, flags);

	src_buf = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	dst_buf = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);
	src_addr = vb2_dma_contig_plane_dma_addr(src_buf, 0);
	dst_addr = vb2_dma_contig_plane_dma_addr(dst_buf, 0);
#if 0
	ast_jpeg_reset(jpeg->regs);
	ast_jpeg_poweron(jpeg->regs);
	ast_jpeg_proc_mode(jpeg->regs, ctx->mode);
		if (ctx->out_q.fmt->fourcc == V4L2_PIX_FMT_RGB565)
			ast_jpeg_input_raw_mode(jpeg->regs,
							S5P_JPEG_RAW_IN_565);
		else
			ast_jpeg_input_raw_mode(jpeg->regs,
							S5P_JPEG_RAW_IN_422);
		ast_jpeg_subsampling_mode(jpeg->regs, ctx->subsampling);
		ast_jpeg_dri(jpeg->regs, ctx->restart_interval);
		ast_jpeg_x(jpeg->regs, ctx->out_q.w);
		ast_jpeg_y(jpeg->regs, ctx->out_q.h);
		ast_jpeg_imgadr(jpeg->regs, src_addr);
		ast_jpeg_jpgadr(jpeg->regs, dst_addr);

		/* ultimately comes from sizeimage from userspace */
		ast_jpeg_enc_stream_int(jpeg->regs, ctx->cap_q.size);

		/*
		 * JPEG IP allows storing 4 quantization tables
		 * We fill table 0 for luma and table 1 for chroma
		 */
		ast_jpeg_set_qtbl_lum(jpeg->regs, ctx->compr_quality);
		ast_jpeg_set_qtbl_chr(jpeg->regs, ctx->compr_quality);
		/* use table 0 for Y */
		ast_jpeg_qtbl(jpeg->regs, 1, 0);
		/* use table 1 for Cb and Cr*/
		ast_jpeg_qtbl(jpeg->regs, 2, 1);
		ast_jpeg_qtbl(jpeg->regs, 3, 1);


	ast_jpeg_start(jpeg->regs);
#endif
	spin_unlock_irqrestore(&ctx->jpeg->slock, flags);
}

static int ast_jpeg_job_ready(void *priv)
{
	struct ast_jpeg_ctx *ctx = priv;

	return ctx->hdr_parsed;
}

static void ast_jpeg_job_abort(void *priv)
{
}

static struct v4l2_m2m_ops ast_jpeg_m2m_ops = {
	.device_run	= ast_jpeg_device_run,
	.job_ready	= ast_jpeg_job_ready,
	.job_abort	= ast_jpeg_job_abort,
};

/*
 * ============================================================================
 * Queue operations
 * ============================================================================
 */

static int ast_jpeg_queue_setup(struct vb2_queue *vq,
			   unsigned int *nbuffers, unsigned int *nplanes,
			   unsigned int sizes[], struct device *alloc_devs[])
{
	struct ast_jpeg_ctx *ctx = vb2_get_drv_priv(vq);
	struct ast_jpeg_q_data *q_data = NULL;
	unsigned int size, count = *nbuffers;

	q_data = get_q_data(ctx, vq->type);
	BUG_ON(q_data == NULL);

	size = q_data->size;

	/*
	 * header is parsed during decoding and parsed information stored
	 * in the context so we do not allow another buffer to overwrite it
	 */
	*nbuffers = count;
	*nplanes = 1;
	sizes[0] = size;

	return 0;
}

static int ast_jpeg_buf_prepare(struct vb2_buffer *vb)
{
	struct ast_jpeg_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct ast_jpeg_q_data *q_data = NULL;

	q_data = get_q_data(ctx, vb->vb2_queue->type);
	BUG_ON(q_data == NULL);

	if (vb2_plane_size(vb, 0) < q_data->size) {
		pr_err("%s data will not fit into plane (%lu < %lu)\n",
				__func__, vb2_plane_size(vb, 0),
				(long)q_data->size);
		return -EINVAL;
	}

	vb2_set_plane_payload(vb, 0, q_data->size);

	return 0;
}

static void ast_jpeg_buf_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct ast_jpeg_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);

	v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, vbuf);
}

static int ast_jpeg_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct ast_jpeg_ctx *ctx = vb2_get_drv_priv(q);
	int ret;

	ret = pm_runtime_get_sync(ctx->jpeg->dev);

	return ret > 0 ? 0 : ret;
}

static void ast_jpeg_stop_streaming(struct vb2_queue *q)
{
	struct ast_jpeg_ctx *ctx = vb2_get_drv_priv(q);

	pm_runtime_put(ctx->jpeg->dev);
}

static const struct vb2_ops ast_jpeg_qops = {
	.queue_setup		= ast_jpeg_queue_setup,
	.buf_prepare		= ast_jpeg_buf_prepare,
	.buf_queue		= ast_jpeg_buf_queue,
	.wait_prepare		= vb2_ops_wait_prepare,
	.wait_finish		= vb2_ops_wait_finish,
	.start_streaming	= ast_jpeg_start_streaming,
	.stop_streaming		= ast_jpeg_stop_streaming,
};

static int queue_init(void *priv, struct vb2_queue *src_vq,
		      struct vb2_queue *dst_vq)
{
	struct ast_jpeg_ctx *ctx = priv;
	int ret;

	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	src_vq->io_modes = VB2_MMAP | VB2_USERPTR;
	src_vq->drv_priv = ctx;
	src_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	src_vq->ops = &ast_jpeg_qops;
	src_vq->mem_ops = &vb2_dma_contig_memops;
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->lock = &ctx->jpeg->lock;
	src_vq->dev = ctx->jpeg->dev;

	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	dst_vq->io_modes = VB2_MMAP | VB2_USERPTR;
	dst_vq->drv_priv = ctx;
	dst_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	dst_vq->ops = &ast_jpeg_qops;
	dst_vq->mem_ops = &vb2_dma_contig_memops;
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->lock = &ctx->jpeg->lock;
	dst_vq->dev = ctx->jpeg->dev;

	return vb2_queue_init(dst_vq);
}

/*
 * ============================================================================
 * ISR
 * ============================================================================
 */

static irqreturn_t ast_jpeg_irq(int irq, void *dev_id)
{
	struct ast_jpeg *jpeg = dev_id;
	struct ast_jpeg_ctx *curr_ctx;
	struct vb2_v4l2_buffer *src_buf, *dst_buf;
	unsigned long payload_size = 0;
	enum vb2_buffer_state state = VB2_BUF_STATE_DONE;
	bool enc_jpeg_too_large = false;
	bool timer_elapsed = false;
	bool op_completed = false;

	spin_lock(&jpeg->slock);
#if 0
	curr_ctx = v4l2_m2m_get_curr_priv(jpeg->m2m_dev);

	src_buf = v4l2_m2m_src_buf_remove(curr_ctx->fh.m2m_ctx);
	dst_buf = v4l2_m2m_dst_buf_remove(curr_ctx->fh.m2m_ctx);

	if (curr_ctx->mode == S5P_JPEG_ENCODE)
		enc_jpeg_too_large = ast_jpeg_enc_stream_stat(jpeg->regs);
	timer_elapsed = ast_jpeg_timer_stat(jpeg->regs);
	op_completed = ast_jpeg_result_stat_ok(jpeg->regs);
	if (curr_ctx->mode == S5P_JPEG_DECODE)
		op_completed = op_completed &&
					ast_jpeg_stream_stat_ok(jpeg->regs);

	if (enc_jpeg_too_large) {
		state = VB2_BUF_STATE_ERROR;
		ast_jpeg_clear_enc_stream_stat(jpeg->regs);
	} else if (timer_elapsed) {
		state = VB2_BUF_STATE_ERROR;
		ast_jpeg_clear_timer_stat(jpeg->regs);
	} else if (!op_completed) {
		state = VB2_BUF_STATE_ERROR;
	} else {
		payload_size = ast_jpeg_compressed_size(jpeg->regs);
	}

	dst_buf->timecode = src_buf->timecode;
	dst_buf->vb2_buf.timestamp = src_buf->vb2_buf.timestamp;
	dst_buf->flags &= ~V4L2_BUF_FLAG_TSTAMP_SRC_MASK;
	dst_buf->flags |=
		src_buf->flags & V4L2_BUF_FLAG_TSTAMP_SRC_MASK;

	v4l2_m2m_buf_done(src_buf, state);
	if (curr_ctx->mode == S5P_JPEG_ENCODE)
		vb2_set_plane_payload(&dst_buf->vb2_buf, 0, payload_size);
	v4l2_m2m_buf_done(dst_buf, state);
	v4l2_m2m_job_finish(jpeg->m2m_dev, curr_ctx->fh.m2m_ctx);

	curr_ctx->subsampling = ast_jpeg_get_subsampling_mode(jpeg->regs);
	spin_unlock(&jpeg->slock);

	ast_jpeg_clear_int(jpeg->regs);
#endif
	return IRQ_HANDLED;
}

static void *jpeg_get_drv_data(struct device *dev);

/*
 * ============================================================================
 * Driver basic infrastructure
 * ============================================================================
 */

static int ast_jpeg_probe(struct platform_device *pdev)
{
	struct ast_jpeg *jpeg;
	struct resource *res;
	int i, ret;

	/* JPEG IP abstraction struct */
	jpeg = devm_kzalloc(&pdev->dev, sizeof(struct ast_jpeg), GFP_KERNEL);
	if (!jpeg)
		return -ENOMEM;

	jpeg->variant = jpeg_get_drv_data(&pdev->dev);

	mutex_init(&jpeg->lock);
	spin_lock_init(&jpeg->slock);
	jpeg->dev = &pdev->dev;

	/* memory-mapped registers */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	jpeg->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(jpeg->regs))
		return PTR_ERR(jpeg->regs);

	/* interrupt service routine registration */
	jpeg->irq = ret = platform_get_irq(pdev, 0);
	if (ret < 0) {
		dev_err(&pdev->dev, "cannot find IRQ\n");
		return ret;
	}

	ret = devm_request_irq(&pdev->dev, jpeg->irq, jpeg->variant->jpeg_irq,
				0, dev_name(&pdev->dev), jpeg);
	if (ret) {
		dev_err(&pdev->dev, "cannot claim IRQ %d\n", jpeg->irq);
		return ret;
	}

	/* v4l2 device */
	ret = v4l2_device_register(&pdev->dev, &jpeg->v4l2_dev);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register v4l2 device\n");
		return ret;
	}

	/* mem2mem device */
	jpeg->m2m_dev = v4l2_m2m_init(jpeg->variant->m2m_ops);
	if (IS_ERR(jpeg->m2m_dev)) {
		v4l2_err(&jpeg->v4l2_dev, "Failed to init mem2mem device\n");
		ret = PTR_ERR(jpeg->m2m_dev);
		goto device_register_rollback;
	}

	vb2_dma_contig_set_max_seg_size(&pdev->dev, DMA_BIT_MASK(32));

	/* JPEG encoder /dev/videoX node */
	jpeg->vfd_encoder = video_device_alloc();
	if (!jpeg->vfd_encoder) {
		v4l2_err(&jpeg->v4l2_dev, "Failed to allocate video device\n");
		ret = -ENOMEM;
		goto m2m_init_rollback;
	}
	snprintf(jpeg->vfd_encoder->name, sizeof(jpeg->vfd_encoder->name),
				"%s-enc", "ast-jpeg");
	jpeg->vfd_encoder->fops		= &ast_jpeg_fops;
	jpeg->vfd_encoder->ioctl_ops	= &ast_jpeg_ioctl_ops;
	jpeg->vfd_encoder->minor	= -1;
	jpeg->vfd_encoder->release	= video_device_release;
	jpeg->vfd_encoder->lock		= &jpeg->lock;
	jpeg->vfd_encoder->v4l2_dev	= &jpeg->v4l2_dev;
	jpeg->vfd_encoder->vfl_dir	= VFL_DIR_M2M;

	ret = video_register_device(jpeg->vfd_encoder, VFL_TYPE_GRABBER, -1);
	if (ret) {
		v4l2_err(&jpeg->v4l2_dev, "Failed to register video device\n");
		video_device_release(jpeg->vfd_encoder);
		goto m2m_init_rollback;
	}

	video_set_drvdata(jpeg->vfd_encoder, jpeg);
	v4l2_info(&jpeg->v4l2_dev,
		  "encoder device registered as /dev/video%d\n",
		  jpeg->vfd_encoder->num);

	/* final statements & power management */
	platform_set_drvdata(pdev, jpeg);

	pm_runtime_enable(&pdev->dev);

	v4l2_info(&jpeg->v4l2_dev, "ASPEED JPEG codec\n");

	return 0;

enc_vdev_register_rollback:
	video_unregister_device(jpeg->vfd_encoder);

m2m_init_rollback:
	v4l2_m2m_release(jpeg->m2m_dev);

device_register_rollback:
	v4l2_device_unregister(&jpeg->v4l2_dev);

	return ret;
}

static int ast_jpeg_remove(struct platform_device *pdev)
{
	struct ast_jpeg *jpeg = platform_get_drvdata(pdev);
	int i;

	pm_runtime_disable(jpeg->dev);

	video_unregister_device(jpeg->vfd_encoder);
	vb2_dma_contig_clear_max_seg_size(&pdev->dev);
	v4l2_m2m_release(jpeg->m2m_dev);
	v4l2_device_unregister(&jpeg->v4l2_dev);

	if (!pm_runtime_status_suspended(&pdev->dev)) {
		for (i = jpeg->variant->num_clocks - 1; i >= 0; i--)
			clk_disable_unprepare(jpeg->clocks[i]);
	}

	return 0;
}

#ifdef CONFIG_PM
static int ast_jpeg_runtime_suspend(struct device *dev)
{
	struct ast_jpeg *jpeg = dev_get_drvdata(dev);
	int i;

	for (i = jpeg->variant->num_clocks - 1; i >= 0; i--)
		clk_disable_unprepare(jpeg->clocks[i]);

	return 0;
}

static int ast_jpeg_runtime_resume(struct device *dev)
{
	struct ast_jpeg *jpeg = dev_get_drvdata(dev);
	unsigned long flags;
	int i, ret;

	for (i = 0; i < jpeg->variant->num_clocks; i++) {
		ret = clk_prepare_enable(jpeg->clocks[i]);
		if (ret) {
			while (--i > 0)
				clk_disable_unprepare(jpeg->clocks[i]);
			return ret;
		}
	}

	spin_lock_irqsave(&jpeg->slock, flags);

	/*
	 * JPEG IP allows storing two Huffman tables for each component.
	 * We fill table 0 for each component and do this here only
	 * for S5PC210 and Exynos3250 SoCs. Exynos4x12 and Exynos542x SoC
	 * require programming their Huffman tables each time the encoding
	 * process is initialized, and thus it is accomplished in the
	 * device_run callback of m2m_ops.
	 */
	if (!jpeg->variant->htbl_reinit) {
		ast_jpeg_set_hdctbl(jpeg->regs);
		ast_jpeg_set_hdctblg(jpeg->regs);
		ast_jpeg_set_hactbl(jpeg->regs);
		ast_jpeg_set_hactblg(jpeg->regs);
	}

	spin_unlock_irqrestore(&jpeg->slock, flags);

	return 0;
}
#endif /* CONFIG_PM */

static const struct dev_pm_ops ast_jpeg_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				pm_runtime_force_resume)
	SET_RUNTIME_PM_OPS(ast_jpeg_runtime_suspend, ast_jpeg_runtime_resume,
			   NULL)
};

static struct ast_jpeg_variant ast_jpeg_drvdata = {
	.version	= SJPEG_S5P,
	.jpeg_irq	= ast_jpeg_irq,
	.m2m_ops	= &ast_jpeg_m2m_ops,
	.fmt_ver_flag	= SJPEG_FMT_FLAG_S5P,
	.clk_names	= {"jpeg"},
	.num_clocks	= 1,
};

static const struct of_device_id ast_jpeg_match[] = {
	{
		.compatible = "aspeed,ast-jpeg",
		.data = &ast_jpeg_drvdata,
	},
	{},
};

MODULE_DEVICE_TABLE(of, ast_jpeg_match);

static void *jpeg_get_drv_data(struct device *dev)
{
	struct ast_jpeg_variant *driver_data = NULL;
	const struct of_device_id *match;

	if (!IS_ENABLED(CONFIG_OF) || !dev->of_node)
		return &ast_jpeg_drvdata;

	match = of_match_node(ast_jpeg_match, dev->of_node);

	if (match)
		driver_data = (struct ast_jpeg_variant *)match->data;

	return driver_data;
}

static struct platform_driver ast_jpeg_driver = {
	.probe = ast_jpeg_probe,
	.remove = ast_jpeg_remove,
	.driver = {
		.of_match_table	= of_match_ptr(ast_jpeg_match),
		.name		= "ast-jpeg",
		.pm		= &ast_jpeg_pm_ops,
	},
};

static struct platform_device *ast_jpeg_device;

static int __init ast_jpeg_init(void)
{
	int ret;
	static const struct resource jpeg_resources[] = {
		[0] = {
			.start = AST_JPEG_BASE,
			.end = AST_JPEG_BASE + SZ_2K - 1,
			.flags = IORESOURCE_MEM,
		},
		[1] = {
			.start = IRQ_JPEG,
			.end = IRQ_JPEG,
			.flags = IORESOURCE_IRQ,
		},
		[2] = {
			.start = AST_JPEG_MEM,
			.end = AST_JPEG_MEM + AST_JPEG_MEM_SIZE - 1,
			.flags = IORESOURCE_DMA,
		},	
	};

	ret = platform_driver_register(&ast_jpeg_driver);

	ast_scu_init_jpeg(0);

	if (!ret) {
		ast_jpeg_device = platform_device_register_simple("ast-jpeg", 0,
								jpeg_resources, ARRAY_SIZE(jpeg_resources));
		if (IS_ERR(ast_jpeg_device)) {
			platform_driver_unregister(&ast_jpeg_driver);
			ret = PTR_ERR(ast_jpeg_device);
		}
	}

	return ret;
}

static void __exit ast_jpeg_exit(void)
{
	platform_device_unregister(ast_jpeg_device);
	platform_driver_unregister(&ast_jpeg_driver);
}

module_init(ast_jpeg_init);
module_exit(ast_jpeg_exit);

MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("ASPEED JPEG codec driver");
MODULE_LICENSE("GPL");
