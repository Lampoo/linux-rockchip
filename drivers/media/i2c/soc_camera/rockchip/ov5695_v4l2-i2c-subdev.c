/*
 * ov5695 sensor driver
 *
 * Copyright (C) 2016 Fuzhou Rockchip Electronics Co., Ltd.
 *
 * Copyright (C) 2012-2014 Intel Mobile Communications GmbH
 *
 * Copyright (C) 2008 Texas Instruments.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 * Note:
 *
 *v0.1.0:
 *1. Initialize version;
 */

#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <media/v4l2-subdev.h>
#include <media/videobuf-core.h>
#include <linux/slab.h>
#include "ov_camera_module.h"

#define ov5695_DRIVER_NAME "ov5695"

#define ov5695_AEC_PK_LONG_GAIN_REG 0x3509

#define ov5695_AEC_PK_LONG_EXPO_3RD_REG 0x3500 /* Exposure Bits 16-19 */
#define ov5695_AEC_PK_LONG_EXPO_2ND_REG 0x3501 /* Exposure Bits 8-15 */
#define ov5695_AEC_PK_LONG_EXPO_1ST_REG 0x3502 /* Exposure Bits 0-7 */
#define ov5695_FETCH_3RD_BYTE_EXP(VAL) (((VAL) >> 12) & 0xF)
#define ov5695_FETCH_2ND_BYTE_EXP(VAL) (((VAL) >> 4) & 0xFF)
#define ov5695_FETCH_1ST_BYTE_EXP(VAL) (((VAL) & 0x0F) << 4)

#define ov5695_AEC_GROUP_UPDATE_ADDRESS 0x3208
#define ov5695_AEC_GROUP_UPDATE_START_DATA 0x00
#define ov5695_AEC_GROUP_UPDATE_END_DATA 0x10
#define ov5695_AEC_GROUP_UPDATE_END_LAUNCH 0xA0

#define ov5695_PIDH_ADDR 0x300B
#define ov5695_PIDL_ADDR 0x300C

#define ov5695_PIDH_MAGIC 0x56
#define ov5695_PIDL_MAGIC 0x95

#define ov5695_TIMING_VTS_HIGH_REG 0x380e
#define ov5695_TIMING_VTS_LOW_REG 0x380f
#define ov5695_TIMING_HTS_HIGH_REG 0x380c
#define ov5695_TIMING_HTS_LOW_REG 0x380d
#define ov5695_INTEGRATION_TIME_MARGIN 8
#define ov5695_FINE_INTG_TIME_MIN 0
#define ov5695_FINE_INTG_TIME_MAX_MARGIN 0
#define ov5695_COARSE_INTG_TIME_MIN 16
#define ov5695_COARSE_INTG_TIME_MAX_MARGIN 4
#define ov5695_TIMING_X_INC 0x3814
#define ov5695_TIMING_Y_INC 0x3816
#define ov5695_HORIZONTAL_START_HIGH_REG 0x3800
#define ov5695_HORIZONTAL_START_LOW_REG 0x3801
#define ov5695_VERTICAL_START_HIGH_REG 0x3802
#define ov5695_VERTICAL_START_LOW_REG 0x3803
#define ov5695_HORIZONTAL_END_HIGH_REG 0x3804
#define ov5695_HORIZONTAL_END_LOW_REG 0x3805
#define ov5695_VERTICAL_END_HIGH_REG 0x3806
#define ov5695_VERTICAL_END_LOW_REG 0x3807
#define ov5695_HORIZONTAL_OUTPUT_SIZE_HIGH_REG 0x3808
#define ov5695_HORIZONTAL_OUTPUT_SIZE_LOW_REG 0x3809
#define ov5695_VERTICAL_OUTPUT_SIZE_HIGH_REG 0x380a
#define ov5695_VERTICAL_OUTPUT_SIZE_LOW_REG 0x380b
#define ov5695_FLIP_MIRROR_REG 0x3820

#define ov5695_EXT_CLK 24000000

static struct ov_camera_module ov5695;
static struct ov_camera_module_custom_config ov5695_custom_config;

/* ======================================================================== */
/* Base sensor configs */
/* ======================================================================== */

/* MCLK:24MHz  2560x1440  30fps   mipi 2lane   840Mbps/lane */
static struct ov_camera_module_reg ov5695_init_tab_2560_1440_30fps[] = {
/*global setting*/
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0103, 0x01},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0100, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0300, 0x04},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0301, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0302, 0x69},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0303, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0304, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0305, 0x01},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0307, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x030b, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x030c, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x030d, 0x1e},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x030e, 0x04},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x030f, 0x03},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0312, 0x01},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3000, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3002, 0x21},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3022, 0x51},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3106, 0x15},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3107, 0x01},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3108, 0x05},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3500, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3501, 0x7e},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3502, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3503, 0x08},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3504, 0x03},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3505, 0x8c},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3507, 0x03},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3508, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3509, 0x10},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x350c, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x350d, 0x80},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3510, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3511, 0x02},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3512, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3601, 0x55},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3602, 0x58},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3614, 0x30},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3615, 0x77},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3621, 0x08},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3624, 0x40},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3633, 0x0c},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3634, 0x0c},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3635, 0x0c},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3636, 0x0c},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3638, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3639, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x363a, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x363b, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x363c, 0xff},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x363d, 0xfa},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3650, 0x44},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3651, 0x44},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3652, 0x44},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3653, 0x44},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3654, 0x44},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3655, 0x44},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3656, 0x44},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3657, 0x44},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3660, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3661, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3662, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x366a, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x366e, 0x18},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3673, 0x04},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3700, 0x14},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3703, 0x0c},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3715, 0x01},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3733, 0x10},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3734, 0x40},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x373f, 0xa0},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3765, 0x20},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x37a1, 0x1d},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x37a8, 0x26},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x37ab, 0x14},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x37c2, 0x04},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x37cb, 0x09},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x37cc, 0x13},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x37cd, 0x1f},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x37ce, 0x1f},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3800, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3801, 0x10},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3802, 0x01},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3803, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3804, 0x0a},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3805, 0x2f},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3806, 0x06},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3807, 0xaf},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3808, 0x0a},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3809, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380a, 0x05},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380b, 0xa0},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380c, 0x02},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380d, 0xe4},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380e, 0x07},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380f, 0xe8},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3810, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3811, 0x06},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3812, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3813, 0x08},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3814, 0x01},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3815, 0x01},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3816, 0x01},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3817, 0x01},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3818, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3819, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x381a, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x381b, 0x01},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3820, 0x80},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3821, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3c80, 0x08},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3c82, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3c83, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3c88, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3d85, 0x14},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3f02, 0x08},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3f03, 0x10},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4008, 0x04},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4009, 0x13},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x404e, 0x20},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4501, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4502, 0x10},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4800, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x481f, 0x2a},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4837, 0x13},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5000, 0x17},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5780, 0x3e},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5781, 0x0f},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5782, 0x44},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5783, 0x02},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5784, 0x01},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5785, 0x01},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5786, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5787, 0x04},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5788, 0x02},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5789, 0x0f},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x578a, 0xfd},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x578b, 0xf5},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x578c, 0xf5},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x578d, 0x03},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x578e, 0x08},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x578f, 0x0c},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5790, 0x08},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5791, 0x06},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5792, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5793, 0x52},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5794, 0xa3},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5b00, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5b01, 0x1c},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5b02, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5b03, 0x7f},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5b05, 0x6c},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5e10, 0xfc},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4010, 0xf1},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3503, 0x08},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3505, 0x8c},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3507, 0x03},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3508, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3509, 0xf8}
};

/* MCLK:24MHz  2592x1944  30fps   mipi 2lane   840Mbps/lane */
static struct ov_camera_module_reg ov5695_init_tab_2592_1944_30fps[] = {
/*global setting*/
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0103, 0x01},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0100, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0300, 0x04},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0301, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0302, 0x69},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0303, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0304, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0305, 0x01},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0307, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x030b, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x030c, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x030d, 0x1e},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x030e, 0x04},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x030f, 0x03},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0312, 0x01},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3000, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3002, 0x21},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3022, 0x51},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3106, 0x15},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3107, 0x01},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3108, 0x05},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3500, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3501, 0x7e},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3502, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3503, 0x08},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3504, 0x03},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3505, 0x8c},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3507, 0x03},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3508, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3509, 0x10},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x350c, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x350d, 0x80},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3510, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3511, 0x02},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3512, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3601, 0x55},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3602, 0x58},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3614, 0x30},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3615, 0x77},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3621, 0x08},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3624, 0x40},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3633, 0x0c},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3634, 0x0c},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3635, 0x0c},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3636, 0x0c},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3638, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3639, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x363a, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x363b, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x363c, 0xff},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x363d, 0xfa},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3650, 0x44},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3651, 0x44},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3652, 0x44},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3653, 0x44},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3654, 0x44},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3655, 0x44},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3656, 0x44},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3657, 0x44},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3660, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3661, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3662, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x366a, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x366e, 0x18},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3673, 0x04},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3700, 0x14},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3703, 0x0c},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3715, 0x01},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3733, 0x10},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3734, 0x40},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x373f, 0xa0},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3765, 0x20},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x37a1, 0x1d},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x37a8, 0x26},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x37ab, 0x14},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x37c2, 0x04},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x37cb, 0x09},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x37cc, 0x13},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x37cd, 0x1f},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x37ce, 0x1f},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3800, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3801, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3802, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3803, 0x04},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3804, 0x0a},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3805, 0x3f},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3806, 0x07},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3807, 0xab},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3808, 0x0a},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3809, 0x20},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380a, 0x07},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380b, 0x98},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380c, 0x02},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380d, 0xe4},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380e, 0x07},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380f, 0xe8},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3810, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3811, 0x06},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3812, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3813, 0x08},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3814, 0x01},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3815, 0x01},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3816, 0x01},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3817, 0x01},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3818, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3819, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x381a, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x381b, 0x01},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3820, 0x80},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3821, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3c80, 0x08},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3c82, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3c83, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3c88, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3d85, 0x14},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3f02, 0x08},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3f03, 0x10},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4008, 0x04},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4009, 0x13},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x404e, 0x20},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4501, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4502, 0x10},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4800, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x481f, 0x2a},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4837, 0x13},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5000, 0x17},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5780, 0x3e},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5781, 0x0f},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5782, 0x44},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5783, 0x02},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5784, 0x01},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5785, 0x01},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5786, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5787, 0x04},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5788, 0x02},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5789, 0x0f},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x578a, 0xfd},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x578b, 0xf5},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x578c, 0xf5},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x578d, 0x03},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x578e, 0x08},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x578f, 0x0c},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5790, 0x08},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5791, 0x06},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5792, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5793, 0x52},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5794, 0xa3},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5b00, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5b01, 0x1c},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5b02, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5b03, 0x7f},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5b05, 0x6c},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5e10, 0xfc},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4010, 0xf1},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3503, 0x08},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3505, 0x8c},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3507, 0x03},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3508, 0x00},
{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3509, 0xf8}
};

/* ======================================================================== */

static struct ov_camera_module_config ov5695_configs[] = {
	{
		.name = "2560x1440_30fps",
		.frm_fmt = {
			.width = 2560,
			.height = 1440,
			.code = V4L2_MBUS_FMT_SBGGR10_1X10
		},
		.frm_intrvl = {
			.interval = {
				.numerator = 1,
				.denominator = 30
			}
		},
		.auto_exp_enabled = false,
		.auto_gain_enabled = false,
		.auto_wb_enabled = false,
		.reg_table = (void *)ov5695_init_tab_2560_1440_30fps,
		.reg_table_num_entries = ARRAY_SIZE(ov5695_init_tab_2560_1440_30fps),
		.v_blanking_time_us = 9600,
		PLTFRM_CAM_ITF_MIPI_CFG(0, 2, 840, ov5695_EXT_CLK)
	},
	{
		.name = "2592x1944_30fps",
		.frm_fmt = {
			.width = 2592,
			.height = 1944,
			.code = V4L2_MBUS_FMT_SBGGR10_1X10
		},
		.frm_intrvl = {
			.interval = {
				.numerator = 1,
				.denominator = 30
			}
		},
		.auto_exp_enabled = false,
		.auto_gain_enabled = false,
		.auto_wb_enabled = false,
		.reg_table = (void *)ov5695_init_tab_2592_1944_30fps,
		.reg_table_num_entries = ARRAY_SIZE(ov5695_init_tab_2592_1944_30fps),
		.v_blanking_time_us = 1300,
		PLTFRM_CAM_ITF_MIPI_CFG(0, 2, 840, ov5695_EXT_CLK)
	}
};

/*--------------------------------------------------------------------------*/

static int ov5695_g_VTS(struct ov_camera_module *cam_mod, u32 *vts)
{
	u32 msb, lsb;
	int ret;

	ret = ov_camera_module_read_reg_table(cam_mod,
		ov5695_TIMING_VTS_HIGH_REG,
		&msb);
	if (IS_ERR_VALUE(ret))
		goto err;

	ret = ov_camera_module_read_reg_table(cam_mod,
		ov5695_TIMING_VTS_LOW_REG,
		&lsb);
	if (IS_ERR_VALUE(ret))
		goto err;

	*vts = (msb << 8) | lsb;

	return 0;
err:
	ov_camera_module_pr_err(cam_mod, "failed with error (%d)\n", ret);
	return ret;
}

/*--------------------------------------------------------------------------*/

static int ov5695_auto_adjust_fps(struct ov_camera_module *cam_mod,
	u32 exp_time)
{
	int ret;
	u32 vts;

	if ((exp_time + ov5695_COARSE_INTG_TIME_MAX_MARGIN)
		> cam_mod->vts_min)
		vts = exp_time + ov5695_COARSE_INTG_TIME_MAX_MARGIN;
	else
		vts = cam_mod->vts_min;

	ret = ov_camera_module_write_reg(cam_mod, ov5695_TIMING_VTS_LOW_REG, vts & 0xFF);
	ret |= ov_camera_module_write_reg(cam_mod, ov5695_TIMING_VTS_HIGH_REG, (vts >> 8) & 0xFF);

	if (IS_ERR_VALUE(ret)) {
		ov_camera_module_pr_err(cam_mod, "failed with error (%d)\n", ret);
	} else {
		ov_camera_module_pr_info(cam_mod, "updated vts = %d,vts_min=%d\n", vts, cam_mod->vts_min);
		cam_mod->vts_cur = vts;
	}

	return ret;
}

/*--------------------------------------------------------------------------*/

static int ov5695_set_vts(struct ov_camera_module *cam_mod,
	u32 vts)
{
	int ret = 0;

	if (vts < cam_mod->vts_min)
		return ret;

	ret = ov_camera_module_write_reg(cam_mod, ov5695_TIMING_VTS_LOW_REG, vts & 0xFF);
	ret |= ov_camera_module_write_reg(cam_mod, ov5695_TIMING_VTS_HIGH_REG, (vts >> 8) & 0xFF);

	if (IS_ERR_VALUE(ret)) {
		ov_camera_module_pr_err(cam_mod, "failed with error (%d)\n", ret);
	} else {
		ov_camera_module_pr_debug(cam_mod, "updated vts = %d,vts_min=%d\n", vts, cam_mod->vts_min);
		cam_mod->vts_cur = vts;
	}

	return ret;
}

/*--------------------------------------------------------------------------*/

static int ov5695_write_aec(struct ov_camera_module *cam_mod)
{
	int ret = 0;
	u32 vts;

	ov_camera_module_pr_debug(cam_mod,
		  "exp_time = %d, gain = %d, flash_mode = %d\n",
		  cam_mod->exp_config.exp_time,
		  cam_mod->exp_config.gain,
		  cam_mod->exp_config.flash_mode);

	if (cam_mod->state == OV_CAMERA_MODULE_SW_STANDBY ||
		cam_mod->state == OV_CAMERA_MODULE_STREAMING) {
		u32 a_gain = cam_mod->exp_config.gain;
		u32 exp_time = cam_mod->exp_config.exp_time;

		a_gain = a_gain * cam_mod->exp_config.gain_percent / 100;
		if (a_gain < 0x10)
			a_gain = 0x10;
		else if (a_gain > 0xf8)
			a_gain = 0xf8;

		mutex_lock(&cam_mod->lock);
		vts = cam_mod->vts_cur == 0 ? cam_mod->vts_min : cam_mod->vts_cur;
		if (!cam_mod->auto_adjust_fps &&
			exp_time > vts - ov5695_COARSE_INTG_TIME_MAX_MARGIN) {
			exp_time = vts - ov5695_COARSE_INTG_TIME_MAX_MARGIN;
		}

		/* hold reg en */
		ret = ov_camera_module_write_reg(cam_mod,
			ov5695_AEC_GROUP_UPDATE_ADDRESS,
			ov5695_AEC_GROUP_UPDATE_START_DATA);

		if (!IS_ERR_VALUE(ret) && cam_mod->auto_adjust_fps)
			ret |= ov5695_auto_adjust_fps(cam_mod,
				cam_mod->exp_config.exp_time);

		ret |= ov_camera_module_write_reg(cam_mod,
			ov5695_AEC_PK_LONG_GAIN_REG, a_gain & 0xff);
		ret |= ov_camera_module_write_reg(cam_mod,
			ov5695_AEC_PK_LONG_EXPO_3RD_REG,
			ov5695_FETCH_3RD_BYTE_EXP(exp_time));
		ret |= ov_camera_module_write_reg(cam_mod,
			ov5695_AEC_PK_LONG_EXPO_2ND_REG,
			ov5695_FETCH_2ND_BYTE_EXP(exp_time));
		ret |= ov_camera_module_write_reg(cam_mod,
			ov5695_AEC_PK_LONG_EXPO_1ST_REG,
			ov5695_FETCH_1ST_BYTE_EXP(exp_time));

		if (!cam_mod->auto_adjust_fps)
			ret |= ov5695_set_vts(cam_mod, cam_mod->exp_config.vts_value);

		/* hold reg end */
		ret |= ov_camera_module_write_reg(cam_mod,
			ov5695_AEC_GROUP_UPDATE_ADDRESS,
			ov5695_AEC_GROUP_UPDATE_END_DATA);
		ret |= ov_camera_module_write_reg(cam_mod,
			ov5695_AEC_GROUP_UPDATE_ADDRESS,
			ov5695_AEC_GROUP_UPDATE_END_LAUNCH);
		mutex_unlock(&cam_mod->lock);
	}

	if (IS_ERR_VALUE(ret))
		ov_camera_module_pr_err(cam_mod, "failed with error (%d)\n", ret);
	return ret;
}

/*--------------------------------------------------------------------------*/

static int ov5695_g_ctrl(struct ov_camera_module *cam_mod, u32 ctrl_id)
{
	int ret = 0;

	ov_camera_module_pr_debug(cam_mod, "\n");

	switch (ctrl_id) {
	case V4L2_CID_GAIN:
	case V4L2_CID_EXPOSURE:
	case V4L2_CID_FLASH_LED_MODE:
		/* nothing to be done here */
		break;
	default:
		ret = -EINVAL;
		break;
	}

	if (IS_ERR_VALUE(ret))
		ov_camera_module_pr_debug(cam_mod,
			"failed with error (%d)\n", ret);
	return ret;
}

/*--------------------------------------------------------------------------*/

static int ov5695_filltimings(struct ov_camera_module_custom_config *custom)
{
	int i, j;
	struct ov_camera_module_config *config;
	struct ov_camera_module_timings *timings;
	struct ov_camera_module_reg *reg_table;
	int reg_table_num_entries;

	for (i = 0; i < custom->num_configs; i++) {
		config = &custom->configs[i];
		reg_table = config->reg_table;
		reg_table_num_entries = config->reg_table_num_entries;
		timings = &config->timings;

		for (j = 0; j < reg_table_num_entries; j++) {
			switch (reg_table[j].reg) {
			case ov5695_TIMING_VTS_HIGH_REG:
				timings->frame_length_lines = reg_table[j].val << 8;
				break;
			case ov5695_TIMING_VTS_LOW_REG:
				timings->frame_length_lines |= reg_table[j].val;
				break;
			case ov5695_TIMING_HTS_HIGH_REG:
				timings->line_length_pck =
					(reg_table[j].val << 8);
				break;
			case ov5695_TIMING_HTS_LOW_REG:
				timings->line_length_pck |= reg_table[j].val;
				break;
			case ov5695_TIMING_X_INC:
				timings->binning_factor_x = ((reg_table[j].val >> 4) + 1) / 2;
				if (timings->binning_factor_x == 0)
					timings->binning_factor_x = 1;
				break;
			case ov5695_TIMING_Y_INC:
				timings->binning_factor_y = ((reg_table[j].val >> 4) + 1) / 2;
				if (timings->binning_factor_y == 0)
					timings->binning_factor_y = 1;
				break;
			case ov5695_HORIZONTAL_START_HIGH_REG:
				timings->crop_horizontal_start = reg_table[j].val << 8;
				break;
			case ov5695_HORIZONTAL_START_LOW_REG:
				timings->crop_horizontal_start |= reg_table[j].val;
				break;
			case ov5695_VERTICAL_START_HIGH_REG:
				timings->crop_vertical_start = reg_table[j].val << 8;
				break;
			case ov5695_VERTICAL_START_LOW_REG:
				timings->crop_vertical_start |= reg_table[j].val;
				break;
			case ov5695_HORIZONTAL_END_HIGH_REG:
				timings->crop_horizontal_end = reg_table[j].val << 8;
				break;
			case ov5695_HORIZONTAL_END_LOW_REG:
				timings->crop_horizontal_end |= reg_table[j].val;
				break;
			case ov5695_VERTICAL_END_HIGH_REG:
				timings->crop_vertical_end = reg_table[j].val << 8;
				break;
			case ov5695_VERTICAL_END_LOW_REG:
				timings->crop_vertical_end |= reg_table[j].val;
				break;
			case ov5695_HORIZONTAL_OUTPUT_SIZE_HIGH_REG:
				timings->sensor_output_width = reg_table[j].val << 8;
				break;
			case ov5695_HORIZONTAL_OUTPUT_SIZE_LOW_REG:
				timings->sensor_output_width |= reg_table[j].val;
				break;
			case ov5695_VERTICAL_OUTPUT_SIZE_HIGH_REG:
				timings->sensor_output_height = reg_table[j].val << 8;
				break;
			case ov5695_VERTICAL_OUTPUT_SIZE_LOW_REG:
				timings->sensor_output_height |= reg_table[j].val;
				break;
			}
		}

		timings->vt_pix_clk_freq_hz = config->frm_intrvl.interval.denominator
					* timings->frame_length_lines
					* timings->line_length_pck;

		timings->coarse_integration_time_min = ov5695_COARSE_INTG_TIME_MIN;
		timings->coarse_integration_time_max_margin =
			ov5695_COARSE_INTG_TIME_MAX_MARGIN;

		/* OV Sensor do not use fine integration time. */
		timings->fine_integration_time_min = ov5695_FINE_INTG_TIME_MIN;
		timings->fine_integration_time_max_margin =
				ov5695_FINE_INTG_TIME_MAX_MARGIN;
	}

	return 0;
}

static int ov5695_g_timings(struct ov_camera_module *cam_mod,
	struct ov_camera_module_timings *timings)
{
	int ret = 0;
	unsigned int vts;

	if (IS_ERR_OR_NULL(cam_mod->active_config))
		goto err;

	*timings = cam_mod->active_config->timings;

	vts = (!cam_mod->vts_cur) ?
		timings->frame_length_lines :
		cam_mod->vts_cur;

	if (cam_mod->frm_intrvl_valid)
		timings->vt_pix_clk_freq_hz =
			cam_mod->frm_intrvl.interval.denominator
			* vts
			* timings->line_length_pck;
	else
		timings->vt_pix_clk_freq_hz =
			cam_mod->active_config->frm_intrvl.interval.denominator
			* vts
			* timings->line_length_pck;

	timings->frame_length_lines = vts;

	return ret;
err:
	ov_camera_module_pr_err(cam_mod, "failed with error (%d)\n", ret);
	return ret;
}

/*--------------------------------------------------------------------------*/

static int ov5695_set_flip(struct ov_camera_module *cam_mod,
	struct pltfrm_camera_module_reg reglist[],
	int len)
{
	int i, mode = 0;
	u16 match_reg;

	mode = ov_camera_module_get_flip_mirror(cam_mod);
	if (mode == -1) {
		ov_camera_module_pr_info(cam_mod, "dts don't set flip, return!\n");
		return 0;
	}

	if (!IS_ERR_OR_NULL(cam_mod->active_config)) {
		if (mode == OV_FLIP_BIT_MASK) {
			match_reg = 0xb0;
		} else if (mode == OV_MIRROR_BIT_MASK) {
			match_reg = 0x88;
		} else if (mode == (OV_MIRROR_BIT_MASK |
			OV_FLIP_BIT_MASK)) {
			match_reg = 0xb8;
		} else {
			match_reg = 0x80;
		}

		for (i = 0; i < len; i++) {
			if (reglist[i].reg == ov5695_FLIP_MIRROR_REG)
				reglist[i].val = match_reg;
		}
	}

	return 0;
}

/*--------------------------------------------------------------------------*/

static int ov5695_s_ctrl(struct ov_camera_module *cam_mod, u32 ctrl_id)
{
	int ret = 0;

	ov_camera_module_pr_debug(cam_mod, "\n");

	switch (ctrl_id) {
	case V4L2_CID_GAIN:
	case V4L2_CID_EXPOSURE:
		ret = ov5695_write_aec(cam_mod);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	if (IS_ERR_VALUE(ret))
		ov_camera_module_pr_debug(cam_mod, "failed with error (%d) 0x%x\n", ret, ctrl_id);
	return ret;
}

/*--------------------------------------------------------------------------*/

static int ov5695_s_ext_ctrls(struct ov_camera_module *cam_mod,
	struct ov_camera_module_ext_ctrls *ctrls)
{
	int ret = 0;

	/* Handles only exposure and gain together special case. */
	if ((ctrls->ctrls[0].id == V4L2_CID_GAIN ||
		ctrls->ctrls[0].id == V4L2_CID_EXPOSURE))
		ret = ov5695_write_aec(cam_mod);
	else
		ret = -EINVAL;

	if (IS_ERR_VALUE(ret))
		ov_camera_module_pr_debug(cam_mod,
			"failed with error (%d)\n", ret);
	return ret;
}

/*--------------------------------------------------------------------------*/

static int ov5695_start_streaming(struct ov_camera_module *cam_mod)
{
	int ret = 0;

	ov_camera_module_pr_info(cam_mod, "active config=%s\n", cam_mod->active_config->name);

	ret = ov5695_g_VTS(cam_mod, &cam_mod->vts_min);
	if (IS_ERR_VALUE(ret))
		goto err;

	mutex_lock(&cam_mod->lock);
	ret = ov_camera_module_write_reg(cam_mod, 0x0100, 1);
	mutex_unlock(&cam_mod->lock);
	if (IS_ERR_VALUE(ret))
		goto err;

	msleep(25);

	return 0;
err:
	ov_camera_module_pr_err(cam_mod, "failed with error (%d)\n",
		ret);
	return ret;
}

/*--------------------------------------------------------------------------*/

static int ov5695_stop_streaming(struct ov_camera_module *cam_mod)
{
	int ret = 0;

	ov_camera_module_pr_info(cam_mod, "\n");

	mutex_lock(&cam_mod->lock);
	ret = ov_camera_module_write_reg(cam_mod, 0x0100, 0);
	mutex_unlock(&cam_mod->lock);
	if (IS_ERR_VALUE(ret))
		goto err;

	msleep(25);

	return 0;
err:
	ov_camera_module_pr_err(cam_mod, "failed with error (%d)\n", ret);
	return ret;
}

/*--------------------------------------------------------------------------*/

static int ov5695_check_camera_id(struct ov_camera_module *cam_mod)
{
	u32 pidh, pidl;
	int ret = 0;

	ov_camera_module_pr_debug(cam_mod, "\n");

	ret |= ov_camera_module_read_reg(cam_mod, 1, ov5695_PIDH_ADDR, &pidh);
	ret |= ov_camera_module_read_reg(cam_mod, 1, ov5695_PIDL_ADDR, &pidl);
	if (IS_ERR_VALUE(ret)) {
		ov_camera_module_pr_err(cam_mod,
			"register read failed, camera module powered off?\n");
		goto err;
	}

	if (pidh == ov5695_PIDH_MAGIC && pidl == ov5695_PIDL_MAGIC) {
		ov_camera_module_pr_info(cam_mod,
			"successfully detected camera ID 0x%02x%02x\n",
			pidh, pidl);
	} else {
		ov_camera_module_pr_err(cam_mod,
			"wrong camera ID, expected 0x%02x%02x, detected 0x%02x%02x\n",
			ov5695_PIDH_MAGIC, ov5695_PIDL_MAGIC, pidh, pidl);
		ret = -EINVAL;
		goto err;
	}

	return 0;
err:
	ov_camera_module_pr_err(cam_mod, "failed with error (%d)\n", ret);
	return ret;
}

/* ======================================================================== */
/* This part is platform dependent */
/* ======================================================================== */

static struct v4l2_subdev_core_ops ov5695_camera_module_core_ops = {
	.g_ctrl = ov_camera_module_g_ctrl,
	.s_ctrl = ov_camera_module_s_ctrl,
	.s_ext_ctrls = ov_camera_module_s_ext_ctrls,
	.s_power = ov_camera_module_s_power,
	.ioctl = ov_camera_module_ioctl
};

static struct v4l2_subdev_video_ops ov5695_camera_module_video_ops = {
	.enum_frameintervals = ov_camera_module_enum_frameintervals,
	.s_mbus_fmt = ov_camera_module_s_fmt,
	.g_mbus_fmt = ov_camera_module_g_fmt,
	.try_mbus_fmt = ov_camera_module_try_fmt,
	.s_frame_interval = ov_camera_module_s_frame_interval,
	.g_frame_interval = ov_camera_module_g_frame_interval,
	.s_stream = ov_camera_module_s_stream
};

static struct v4l2_subdev_ops ov5695_camera_module_ops = {
	.core = &ov5695_camera_module_core_ops,
	.video = &ov5695_camera_module_video_ops,
};

static struct ov_camera_module_custom_config ov5695_custom_config = {
	.start_streaming = ov5695_start_streaming,
	.stop_streaming = ov5695_stop_streaming,
	.s_ctrl = ov5695_s_ctrl,
	.s_ext_ctrls = ov5695_s_ext_ctrls,
	.g_ctrl = ov5695_g_ctrl,
	.g_timings = ov5695_g_timings,
	.check_camera_id = ov5695_check_camera_id,
	.set_flip = ov5695_set_flip,
	.s_vts = ov5695_auto_adjust_fps,
	.configs = ov5695_configs,
	.num_configs = ARRAY_SIZE(ov5695_configs),
	.power_up_delays_ms = {5, 20, 0},
	/*
	 * 0: Exposure time valid fileds;
	 * 1: Exposure gain valid fileds;
	 * (2 fileds == 1 frames)
	 */
	.exposure_valid_frame = {4, 4}
};

static int ov5695_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	dev_info(&client->dev, "probing...\n");

	ov5695_filltimings(&ov5695_custom_config);
	v4l2_i2c_subdev_init(&ov5695.sd, client, &ov5695_camera_module_ops);
	ov5695.sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	ov5695.custom = ov5695_custom_config;

	mutex_init(&ov5695.lock);
	dev_info(&client->dev, "probing successful\n");
	return 0;
}

static int ov5695_remove(struct i2c_client *client)
{
	struct ov_camera_module *cam_mod = i2c_get_clientdata(client);

	dev_info(&client->dev, "removing device...\n");

	if (!client->adapter)
		return -ENODEV;	/* our client isn't attached */

	mutex_destroy(&cam_mod->lock);
	ov_camera_module_release(cam_mod);

	dev_info(&client->dev, "removed\n");
	return 0;
}

static const struct i2c_device_id ov5695_id[] = {
	{ ov5695_DRIVER_NAME, 0 },
	{ }
};

const static struct of_device_id ov5695_of_match[] = {
	{.compatible = "omnivision,ov5695-v4l2-i2c-subdev"},
	{},
};

MODULE_DEVICE_TABLE(i2c, ov5695_id);

static struct i2c_driver ov5695_i2c_driver = {
	.driver = {
		.name = ov5695_DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = ov5695_of_match
	},
	.probe = ov5695_probe,
	.remove = ov5695_remove,
	.id_table = ov5695_id,
};

module_i2c_driver(ov5695_i2c_driver);

MODULE_DESCRIPTION("SoC Camera driver for ov5695");
MODULE_AUTHOR("Cain");
MODULE_LICENSE("GPL");
