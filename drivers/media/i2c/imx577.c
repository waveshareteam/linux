// SPDX-License-Identifier: GPL-2.0-only
/*
 * A V4L2 driver for Sony IMX577 cameras.
 *
 * Register tables and mode timings are ported from the Rockchip BSP IMX577
 * driver, Copyright (C) 2022 Rockchip Electronics Co., Ltd.
 * The driver framework, CCI register access and V4L2 subdev state handling
 * are based on the Raspberry Pi IMX477 driver,
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * Hardware configuration:
 *   - MIPI CSI-2, two or four data lanes, 750 MHz link frequency
 *     (1500 Mbps per lane); the lane count is taken from the device tree
 *   - 24 MHz external clock
 *   - D-PHY global timing in automatic mode (0x0808 = 1)
 *
 * Supported modes (selected with the "mode_idx" module parameter):
 *   0 - 4056x3040 10-bit   (~30 fps with four lanes, ISP limited)
 *   1 - 2028x1520 10-bit   (~118 fps with four lanes, 2x2 binning)
 *
 * The lane count and the requested D-PHY bit rate are programmed from the
 * device tree "data-lanes" and "link-frequencies" properties, so modules
 * with either two or four lanes are supported by the same register tables.
 * The sensor derives its readout rate from the requested bit rate, so two
 * lanes halve the maximum frame rate.
 */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mediabus.h>

/* Chip ID */
#define IMX577_REG_CHIP_ID		CCI_REG16(0x0016)
#define IMX577_CHIP_ID			0x0577

#define IMX577_REG_MODE_SELECT		CCI_REG8(0x0100)
#define IMX577_MODE_STANDBY		0x00
#define IMX577_MODE_STREAMING		0x01

/* Flip/mirror */
#define IMX577_REG_ORIENTATION		CCI_REG8(0x0101)

#define IMX577_XCLK_FREQ		24000000

#define IMX577_LINK_FREQ_750MHZ	750000000ULL
#define IMX577_MIN_LANES		2
#define IMX577_MAX_LANES		4
#define IMX577_BITS_PER_SAMPLE		10

/*
 * Link related registers.  The lane count and the requested D-PHY bit rate
 * (16.16 fixed point Mb/s) are derived from the device tree, see
 * imx577_apply_link_config().  The sensor derives its readout rate from the
 * D-PHY bit rate, so the same per-mode register tables can be used with two
 * or four lanes; two lanes halve the maximum frame rate.
 *
 *   0x0112/0x0113  CSI-2 data type (0x0a = RAW10)
 *   0x0808         D-PHY control (0x01 = automatic global timing)
 */
#define IMX577_REG_NUM_LANES		CCI_REG8(0x0114)
#define IMX577_REG_REQ_LINK_BIT_RATE	CCI_REG32(0x0820)

/* V_TIMING internal */
#define IMX577_REG_FRAME_LENGTH		CCI_REG16(0x0340)
#define IMX577_VTS_MAX			0xffff

/* H_TIMING internal */
#define IMX577_REG_LINE_LENGTH		CCI_REG16(0x0342)

/* Exposure control */
#define IMX577_REG_EXPOSURE		CCI_REG16(0x0202)
#define IMX577_EXPOSURE_MIN		8
#define IMX577_EXPOSURE_OFFSET		22
#define IMX577_EXPOSURE_STEP		1
#define IMX577_EXPOSURE_DEFAULT		0x0c10

/* Analog gain control, raw register units: gain = 1024 / (1024 - reg) */
#define IMX577_REG_ANALOG_GAIN		CCI_REG16(0x0204)
#define IMX577_ANA_GAIN_MIN		0
#define IMX577_ANA_GAIN_MAX		978
#define IMX577_ANA_GAIN_STEP		1
#define IMX577_ANA_GAIN_DEFAULT		0

/* Digital gain control, 1/256ths of a stop, applied to all four channels */
#define IMX577_REG_DGAIN_GR		CCI_REG16(0x020e)
#define IMX577_REG_DGAIN_R		CCI_REG16(0x0210)
#define IMX577_REG_DGAIN_B		CCI_REG16(0x0212)
#define IMX577_REG_DGAIN_GB		CCI_REG16(0x0214)
#define IMX577_DGAIN_MIN		0x0100
#define IMX577_DGAIN_MAX		0x0fff
#define IMX577_DGAIN_DEFAULT		0x0100
#define IMX577_DGAIN_STEP		1

/* Test pattern control */
#define IMX577_REG_TEST_PATTERN		CCI_REG8(0x0600)
#define IMX577_TEST_PATTERN_DISABLE	0x00
#define IMX577_TEST_PATTERN_COLORBAR1	0x02
#define IMX577_TEST_PATTERN_COLORBAR2	0x03
#define IMX577_TEST_PATTERN_COLORBAR3	0x04

#define IMX577_VBLANK_MIN		48

/* Embedded metadata stream structure */
#define IMX577_EMBEDDED_LINE_WIDTH	16384
#define IMX577_NUM_EMBEDDED_LINES	1

enum pad_types {
	IMAGE_PAD,
	METADATA_PAD,
	NUM_PADS
};

/* IMX577 active pixel array size. */
#define IMX577_PIXEL_ARRAY_LEFT		0U
#define IMX577_PIXEL_ARRAY_TOP		0U
#define IMX577_PIXEL_ARRAY_WIDTH	4056U
#define IMX577_PIXEL_ARRAY_HEIGHT	3040U

#define IMX577_NATIVE_WIDTH		IMX577_PIXEL_ARRAY_WIDTH
#define IMX577_NATIVE_HEIGHT		IMX577_PIXEL_ARRAY_HEIGHT

struct imx577_reg_list {
	unsigned int num_of_regs;
	const struct cci_reg_sequence *regs;
};

/* Mode : resolution and related config&values */
struct imx577_mode {
	/* Frame width */
	unsigned int width;

	/* Frame height */
	unsigned int height;

	/* Analog crop rectangle. */
	struct v4l2_rect crop;

	/* Default frame_length value to use in this mode */
	unsigned int frm_length_default;

	/* Default line_length value to use in this mode */
	unsigned int line_length_default;

	/* Maximum frame rate */
	struct v4l2_fract max_fps;

	/* Default register values */
	struct imx577_reg_list reg_list;
};

static const struct cci_reg_sequence mode_common_regs[] = {
	{CCI_REG8(0x0136), 0x18},
	{CCI_REG8(0x0137), 0x00},
	{CCI_REG8(0x0808), 0x01},
	{CCI_REG8(0x38a8), 0x1f},
	{CCI_REG8(0x38a9), 0xff},
	{CCI_REG8(0x38aa), 0x1f},
	{CCI_REG8(0x38ab), 0xff},
	{CCI_REG8(0x3c7e), 0x01},
	{CCI_REG8(0x3c7f), 0x02},
	{CCI_REG8(0x55d4), 0x00},
	{CCI_REG8(0x55d5), 0x00},
	{CCI_REG8(0x55d6), 0x07},
	{CCI_REG8(0x55d7), 0xff},
	{CCI_REG8(0x55e8), 0x07},
	{CCI_REG8(0x55e9), 0xff},
	{CCI_REG8(0x55ea), 0x00},
	{CCI_REG8(0x55eb), 0x00},
	{CCI_REG8(0x575c), 0x07},
	{CCI_REG8(0x575d), 0xff},
	{CCI_REG8(0x575e), 0x00},
	{CCI_REG8(0x575f), 0x00},
	{CCI_REG8(0x5764), 0x00},
	{CCI_REG8(0x5765), 0x00},
	{CCI_REG8(0x5766), 0x07},
	{CCI_REG8(0x5767), 0xff},
	{CCI_REG8(0x5974), 0x04},
	{CCI_REG8(0x5975), 0x01},
	{CCI_REG8(0x5f10), 0x09},
	{CCI_REG8(0x5f11), 0x92},
	{CCI_REG8(0x5f12), 0x32},
	{CCI_REG8(0x5f13), 0x72},
	{CCI_REG8(0x5f14), 0x16},
	{CCI_REG8(0x5f15), 0xba},
	{CCI_REG8(0x5f17), 0x13},
	{CCI_REG8(0x5f18), 0x24},
	{CCI_REG8(0x5f19), 0x60},
	{CCI_REG8(0x5f1a), 0xe3},
	{CCI_REG8(0x5f1b), 0xad},
	{CCI_REG8(0x5f1c), 0x74},
	{CCI_REG8(0x5f2d), 0x25},
	{CCI_REG8(0x5f5c), 0xd0},
	{CCI_REG8(0x6a22), 0x00},
	{CCI_REG8(0x6a23), 0x1d},
	{CCI_REG8(0x7ba8), 0x00},
	{CCI_REG8(0x7ba9), 0x00},
	{CCI_REG8(0x886b), 0x00},
	{CCI_REG8(0x9002), 0x0a},
	{CCI_REG8(0x9004), 0x1a},
	{CCI_REG8(0x9214), 0x93},
	{CCI_REG8(0x9215), 0x69},
	{CCI_REG8(0x9216), 0x93},
	{CCI_REG8(0x9217), 0x6b},
	{CCI_REG8(0x9218), 0x93},
	{CCI_REG8(0x9219), 0x6d},
	{CCI_REG8(0x921a), 0x57},
	{CCI_REG8(0x921b), 0x58},
	{CCI_REG8(0x921c), 0x57},
	{CCI_REG8(0x921d), 0x59},
	{CCI_REG8(0x921e), 0x57},
	{CCI_REG8(0x921f), 0x5a},
	{CCI_REG8(0x9220), 0x57},
	{CCI_REG8(0x9221), 0x5b},
	{CCI_REG8(0x9222), 0x93},
	{CCI_REG8(0x9223), 0x02},
	{CCI_REG8(0x9224), 0x93},
	{CCI_REG8(0x9225), 0x03},
	{CCI_REG8(0x9226), 0x93},
	{CCI_REG8(0x9227), 0x04},
	{CCI_REG8(0x9228), 0x93},
	{CCI_REG8(0x9229), 0x05},
	{CCI_REG8(0x922a), 0x98},
	{CCI_REG8(0x922b), 0x21},
	{CCI_REG8(0x922c), 0xb2},
	{CCI_REG8(0x922d), 0xdb},
	{CCI_REG8(0x922e), 0xb2},
	{CCI_REG8(0x922f), 0xdc},
	{CCI_REG8(0x9230), 0xb2},
	{CCI_REG8(0x9231), 0xdd},
	{CCI_REG8(0x9232), 0xb2},
	{CCI_REG8(0x9233), 0xe1},
	{CCI_REG8(0x9234), 0xb2},
	{CCI_REG8(0x9235), 0xe2},
	{CCI_REG8(0x9236), 0xb2},
	{CCI_REG8(0x9237), 0xe3},
	{CCI_REG8(0x9238), 0xb7},
	{CCI_REG8(0x9239), 0xb9},
	{CCI_REG8(0x923a), 0xb7},
	{CCI_REG8(0x923b), 0xbb},
	{CCI_REG8(0x923c), 0xb7},
	{CCI_REG8(0x923d), 0xbc},
	{CCI_REG8(0x923e), 0xb7},
	{CCI_REG8(0x923f), 0xc5},
	{CCI_REG8(0x9240), 0xb7},
	{CCI_REG8(0x9241), 0xc7},
	{CCI_REG8(0x9242), 0xb7},
	{CCI_REG8(0x9243), 0xc9},
	{CCI_REG8(0x9244), 0x98},
	{CCI_REG8(0x9245), 0x56},
	{CCI_REG8(0x9246), 0x98},
	{CCI_REG8(0x9247), 0x55},
	{CCI_REG8(0x9380), 0x00},
	{CCI_REG8(0x9381), 0x62},
	{CCI_REG8(0x9382), 0x00},
	{CCI_REG8(0x9383), 0x56},
	{CCI_REG8(0x9384), 0x00},
	{CCI_REG8(0x9385), 0x52},
	{CCI_REG8(0x9388), 0x00},
	{CCI_REG8(0x9389), 0x55},
	{CCI_REG8(0x938a), 0x00},
	{CCI_REG8(0x938b), 0x55},
	{CCI_REG8(0x938c), 0x00},
	{CCI_REG8(0x938d), 0x41},
};

/*
 * 4056x3040 10-bit, 4 data lanes, 750 MHz link frequency (1500 Mbps per
 * lane).  The D-PHY uses automatic global timing (0x0808 = 1) and the
 * requested bit rate is set to 6000 Mb/s, matching four lanes at 1500 Mbps.
 * Frame length 3080 and line length 4400 give the sensor's own maximum of
 * about 44 fps; the usable frame rate is limited by the ISP.
 */
static const struct cci_reg_sequence mode_4056x3040_10b_regs[] = {
	{CCI_REG8(0x00e3), 0x00},
	{CCI_REG8(0x00e4), 0x00},
	{CCI_REG8(0x00e5), 0x01},
	{CCI_REG8(0x00fc), 0x0a},
	{CCI_REG8(0x00fd), 0x0a},
	{CCI_REG8(0x00fe), 0x0a},
	{CCI_REG8(0x00ff), 0x0a},
	{CCI_REG8(0x0112), 0x0a},
	{CCI_REG8(0x0113), 0x0a},
	{CCI_REG8(0x0114), 0x03},
	{CCI_REG8(0x0202), 0x0c},
	{CCI_REG8(0x0203), 0x16},
	{CCI_REG8(0x0204), 0x00},
	{CCI_REG8(0x0205), 0x00},
	{CCI_REG8(0x020e), 0x01},
	{CCI_REG8(0x020f), 0x00},
	{CCI_REG8(0x0210), 0x01},
	{CCI_REG8(0x0211), 0x00},
	{CCI_REG8(0x0212), 0x01},
	{CCI_REG8(0x0213), 0x00},
	{CCI_REG8(0x0214), 0x01},
	{CCI_REG8(0x0215), 0x00},
	{CCI_REG8(0x0220), 0x00},
	{CCI_REG8(0x0221), 0x11},
	{CCI_REG8(0x0301), 0x05},
	{CCI_REG8(0x0303), 0x02},
	{CCI_REG8(0x0305), 0x04},
	{CCI_REG8(0x0306), 0x00},
	{CCI_REG8(0x0307), 0xfa},
	{CCI_REG8(0x0309), 0x0a},
	{CCI_REG8(0x030b), 0x01},
	{CCI_REG8(0x030d), 0x02},
	{CCI_REG8(0x030e), 0x01},
	{CCI_REG8(0x030f), 0x5e},
	{CCI_REG8(0x0310), 0x00},
	{CCI_REG8(0x0340), 0x0c},
	{CCI_REG8(0x0341), 0x08},
	{CCI_REG8(0x0342), 0x11},
	{CCI_REG8(0x0343), 0x30},
	{CCI_REG8(0x0344), 0x00},
	{CCI_REG8(0x0345), 0x00},
	{CCI_REG8(0x0346), 0x00},
	{CCI_REG8(0x0347), 0x00},
	{CCI_REG8(0x0348), 0x0f},
	{CCI_REG8(0x0349), 0xd7},
	{CCI_REG8(0x034a), 0x0b},
	{CCI_REG8(0x034b), 0xdf},
	{CCI_REG8(0x034c), 0x0f},
	{CCI_REG8(0x034d), 0xd8},
	{CCI_REG8(0x034e), 0x0b},
	{CCI_REG8(0x034f), 0xe0},
	{CCI_REG8(0x0381), 0x01},
	{CCI_REG8(0x0383), 0x01},
	{CCI_REG8(0x0385), 0x01},
	{CCI_REG8(0x0387), 0x01},
	{CCI_REG8(0x0401), 0x00},
	{CCI_REG8(0x0404), 0x00},
	{CCI_REG8(0x0405), 0x10},
	{CCI_REG8(0x0408), 0x00},
	{CCI_REG8(0x0409), 0x00},
	{CCI_REG8(0x040a), 0x00},
	{CCI_REG8(0x040b), 0x00},
	{CCI_REG8(0x040c), 0x0f},
	{CCI_REG8(0x040d), 0xd8},
	{CCI_REG8(0x040e), 0x0b},
	{CCI_REG8(0x040f), 0xe0},
	{CCI_REG8(0x0808), 0x01},
	{CCI_REG8(0x0820), 0x17},
	{CCI_REG8(0x0821), 0x70},
	{CCI_REG8(0x0822), 0x00},
	{CCI_REG8(0x0823), 0x00},
	{CCI_REG8(0x0900), 0x00},
	{CCI_REG8(0x0901), 0x11},
	{CCI_REG8(0x0902), 0x00},
	{CCI_REG8(0x3140), 0x02},
	{CCI_REG8(0x3210), 0x00},
	{CCI_REG8(0x3241), 0x11},
	{CCI_REG8(0x3250), 0x03},
	{CCI_REG8(0x3c0a), 0x5a},
	{CCI_REG8(0x3c0b), 0x55},
	{CCI_REG8(0x3c0c), 0x28},
	{CCI_REG8(0x3c0d), 0x07},
	{CCI_REG8(0x3c0e), 0xff},
	{CCI_REG8(0x3c0f), 0x00},
	{CCI_REG8(0x3c10), 0x00},
	{CCI_REG8(0x3c11), 0x02},
	{CCI_REG8(0x3c12), 0x00},
	{CCI_REG8(0x3c13), 0x03},
	{CCI_REG8(0x3c14), 0x00},
	{CCI_REG8(0x3c15), 0x00},
	{CCI_REG8(0x3c16), 0x0c},
	{CCI_REG8(0x3c17), 0x0c},
	{CCI_REG8(0x3c18), 0x0c},
	{CCI_REG8(0x3c19), 0x0a},
	{CCI_REG8(0x3c1a), 0x0a},
	{CCI_REG8(0x3c1b), 0x0a},
	{CCI_REG8(0x3c1c), 0x00},
	{CCI_REG8(0x3c1d), 0x00},
	{CCI_REG8(0x3c1e), 0x00},
	{CCI_REG8(0x3c1f), 0x00},
	{CCI_REG8(0x3c20), 0x00},
	{CCI_REG8(0x3c21), 0x00},
	{CCI_REG8(0x3c22), 0x3f},
	{CCI_REG8(0x3c23), 0x0a},
	{CCI_REG8(0x3e10), 0x00},
	{CCI_REG8(0x3e11), 0x00},
	{CCI_REG8(0x3e20), 0x01},
	{CCI_REG8(0x3e35), 0x01},
	{CCI_REG8(0x3e37), 0x00},
	{CCI_REG8(0x3f0d), 0x00},
	{CCI_REG8(0x3f26), 0x00},
	{CCI_REG8(0x3f42), 0x00},
	{CCI_REG8(0x3f43), 0x00},
	{CCI_REG8(0x3f4a), 0x03},
	{CCI_REG8(0x3f4b), 0xbf},
	{CCI_REG8(0x3f50), 0x00},
	{CCI_REG8(0x3f56), 0x00},
	{CCI_REG8(0x3f57), 0x41},
};

static const struct cci_reg_sequence mode_2028x1520_regs[] = {
	{CCI_REG8(0x0112), 0x0a},
	{CCI_REG8(0x0113), 0x0a},
	{CCI_REG8(0x0114), 0x03},
	{CCI_REG8(0x0305), 0x04},
	{CCI_REG8(0x0306), 0x00},
	{CCI_REG8(0x0307), 0xfa},
	{CCI_REG8(0x0309), 0x0a},
	{CCI_REG8(0x030b), 0x01},
	{CCI_REG8(0x030d), 0x02},
	{CCI_REG8(0x030e), 0x01},
	{CCI_REG8(0x030f), 0x5e},
	{CCI_REG8(0x0310), 0x00},
	{CCI_REG8(0x0340), 0x06},
	{CCI_REG8(0x0341), 0x2c},
	{CCI_REG8(0x0342), 0x09},
	{CCI_REG8(0x0343), 0x60},
	{CCI_REG8(0x0344), 0x00},
	{CCI_REG8(0x0345), 0x00},
	{CCI_REG8(0x0346), 0x00},
	{CCI_REG8(0x0347), 0x00},
	{CCI_REG8(0x0348), 0x0f},
	{CCI_REG8(0x0349), 0xd7},
	{CCI_REG8(0x034a), 0x0b},
	{CCI_REG8(0x034b), 0xdf},
	{CCI_REG8(0x034c), 0x07},
	{CCI_REG8(0x034d), 0xec},
	{CCI_REG8(0x034e), 0x05},
	{CCI_REG8(0x034f), 0xf0},
	{CCI_REG8(0x0401), 0x00},
	{CCI_REG8(0x0404), 0x00},
	{CCI_REG8(0x0405), 0x20},
	{CCI_REG8(0x0408), 0x00},
	{CCI_REG8(0x0409), 0x00},
	{CCI_REG8(0x040a), 0x00},
	{CCI_REG8(0x040b), 0x00},
	{CCI_REG8(0x040c), 0x0f},
	{CCI_REG8(0x040d), 0xd8},
	{CCI_REG8(0x040e), 0x0b},
	{CCI_REG8(0x040f), 0xe0},
	{CCI_REG8(0x0808), 0x01},
	{CCI_REG8(0x0820), 0x17},
	{CCI_REG8(0x0821), 0x70},
	{CCI_REG8(0x0822), 0x00},
	{CCI_REG8(0x0823), 0x00},
	{CCI_REG8(0x0900), 0x01},
	{CCI_REG8(0x0901), 0x22},
	{CCI_REG8(0x3041), 0x01},
	{CCI_REG8(0x3c01), 0x03},
	{CCI_REG8(0x3c02), 0xa2},
	{CCI_REG8(0x3f56), 0x01},
	{CCI_REG8(0x3f57), 0x6c},
	{CCI_REG8(0x5748), 0x07},
	{CCI_REG8(0x5749), 0xff},
	{CCI_REG8(0x574a), 0x00},
	{CCI_REG8(0x574b), 0x00},
	{CCI_REG8(0x7b53), 0x01},
	{CCI_REG8(0x9304), 0x00},
	{CCI_REG8(0x9305), 0x00},
	{CCI_REG8(0x9369), 0x73},
	{CCI_REG8(0x936b), 0x64},
	{CCI_REG8(0x936d), 0x5f},
	{CCI_REG8(0xa2a9), 0x60},
	{CCI_REG8(0xa2b7), 0x00},
	{CCI_REG8(0xe04c), 0x00},
	{CCI_REG8(0xe04d), 0x7f},
	{CCI_REG8(0xe04e), 0x00},
	{CCI_REG8(0xe04f), 0x1f},
};

/*
 * Only one mode is enumerated at a time.  libpisp configures the CFE from
 * the enumerated mode, so advertising both would let it program one frame
 * size while the sensor outputs the other.
 */
static int mode_idx;
module_param(mode_idx, int, 0644);
MODULE_PARM_DESC(mode_idx,
		 "Sensor mode to expose: 0 = 4056x3040 (default), 1 = 2028x1520 binning, -1 = all modes");

/*
 * libcamera selects the camera helper, the static sensor properties and the
 * tuning file by the subdevice name.  There is no IMX577 entry yet, but the
 * sensor is register compatible with the IMX477, so allow reusing the IMX477
 * tuning by reporting that name.  The correct fix is an IMX577 tuning file in
 * libcamera, at which point this option can be dropped.
 */
static bool imx477_tuning;
module_param(imx477_tuning, bool, 0444);
MODULE_PARM_DESC(imx477_tuning,
		 "Report the subdevice as \"imx477\" so that libcamera reuses the IMX477 tuning file");

static const struct imx577_mode supported_modes[] = {
	{
		/* 4056x3040 full resolution, 10-bit, 4-lane */
		.width = 4056, .height = 3040,
		.crop = { .left = IMX577_PIXEL_ARRAY_LEFT, .top = IMX577_PIXEL_ARRAY_TOP,
			  .width = IMX577_PIXEL_ARRAY_WIDTH, .height = IMX577_PIXEL_ARRAY_HEIGHT },
		.frm_length_default = 3080, .line_length_default = 4400,
		.max_fps = { 1, 30 },
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_4056x3040_10b_regs),
			.regs = mode_4056x3040_10b_regs,
		},
	},
	{
		/* 2028x1520 2x2 binning, 10-bit, 4-lane */
		.width = 2028, .height = 1520,
		.crop = { .left = IMX577_PIXEL_ARRAY_LEFT, .top = IMX577_PIXEL_ARRAY_TOP,
			  .width = IMX577_PIXEL_ARRAY_WIDTH, .height = IMX577_PIXEL_ARRAY_HEIGHT },
		.frm_length_default = 1580, .line_length_default = 2400,
		.max_fps = { 1, 118 },
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_2028x1520_regs),
			.regs = mode_2028x1520_regs,
		},
	},
};

/*
 * The supported formats.  The table has one entry per flip combination,
 * in the order
 * - no flip
 * - h flip
 * - v flip
 * - h&v flips
 */
static const u32 codes[] = {
	MEDIA_BUS_FMT_SRGGB10_1X10,
	MEDIA_BUS_FMT_SGRBG10_1X10,
	MEDIA_BUS_FMT_SGBRG10_1X10,
	MEDIA_BUS_FMT_SBGGR10_1X10,
};

static const char * const imx577_test_pattern_menu[] = {
	"Disabled",
	"Vertical Color Bar Type 1",
	"Vertical Color Bar Type 2",
	"Vertical Color Bar Type 3"
};

static const int imx577_test_pattern_val[] = {
	IMX577_TEST_PATTERN_DISABLE,
	IMX577_TEST_PATTERN_COLORBAR1,
	IMX577_TEST_PATTERN_COLORBAR2,
	IMX577_TEST_PATTERN_COLORBAR3,
};

/* regulator supplies */
static const char * const imx577_supply_name[] = {
	/* Supplies can be enabled in any order */
	"VANA",  /* Analog (2.8V) supply */
	"VDIG",  /* Digital Core (1.05V) supply */
	"VDDL",  /* IF (1.8V) supply */
};

#define IMX577_NUM_SUPPLIES ARRAY_SIZE(imx577_supply_name)

/*
 * Initialisation delay between XCLR low->high and the moment when the sensor
 * can start capture (i.e. can leave software standby).
 */
#define IMX577_XCLR_MIN_DELAY_US	8000
#define IMX577_XCLR_DELAY_RANGE_US	1000

struct imx577 {
	struct v4l2_subdev sd;
	struct media_pad pad[NUM_PADS];
	struct regmap *regmap;

	unsigned int fmt_code;

	struct clk *xclk;
	u32 xclk_freq;

	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[IMX577_NUM_SUPPLIES];

	struct v4l2_ctrl_handler ctrl_handler;
	/* V4L2 Controls */
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *vflip;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;

	u64 link_freq_value;
	unsigned int lanes;

	/* Current mode */
	const struct imx577_mode *mode;

	/*
	 * Mutex for serialized access:
	 * Protect sensor module set pad format and start/stop streaming safely.
	 */
	struct mutex mutex;

	/* Streaming on/off */
	bool streaming;

	/* Rewrite common registers on stream on? */
	bool common_regs_written;
};

static inline struct imx577 *to_imx577(struct v4l2_subdev *_sd)
{
	return container_of(_sd, struct imx577, sd);
}

/*
 * Return the bayer order code for the requested code, adjusted for the
 * current flip settings.
 */
static u32 imx577_get_format_code(struct imx577 *imx577, u32 code)
{
	unsigned int i;

	lockdep_assert_held(&imx577->mutex);

	for (i = 0; i < ARRAY_SIZE(codes); i++)
		if (codes[i] == code)
			break;

	/* Preserve the requested flip combination, if any valid code. */
	if (i == ARRAY_SIZE(codes))
		i = 0;

	/* Apply the current flip settings. */
	i |= (imx577->vflip->val ? 2 : 0) | (imx577->hflip->val ? 1 : 0);

	return codes[i];
}

static void imx577_set_default_format(struct imx577 *imx577)
{
	/* Set default mode to the full resolution mode */
	imx577->mode = &supported_modes[0];
	imx577->fmt_code = MEDIA_BUS_FMT_SRGGB10_1X10;
}

static int imx577_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct imx577 *imx577 = to_imx577(sd);
	struct v4l2_mbus_framefmt *try_fmt =
		v4l2_subdev_state_get_format(fh->state, IMAGE_PAD);
	struct v4l2_rect *try_crop;

	mutex_lock(&imx577->mutex);

	/* Initialize try_fmt */
	try_fmt->width = supported_modes[0].width;
	try_fmt->height = supported_modes[0].height;
	try_fmt->code = imx577_get_format_code(imx577,
					       MEDIA_BUS_FMT_SRGGB10_1X10);
	try_fmt->field = V4L2_FIELD_NONE;

	/* Initialize try_fmt for the embedded metadata pad */
	try_fmt = v4l2_subdev_state_get_format(fh->state, METADATA_PAD);
	try_fmt->width = IMX577_EMBEDDED_LINE_WIDTH;
	try_fmt->height = IMX577_NUM_EMBEDDED_LINES;
	try_fmt->code = MEDIA_BUS_FMT_SENSOR_DATA;
	try_fmt->field = V4L2_FIELD_NONE;

	/* Initialize try_crop */
	try_crop = v4l2_subdev_state_get_crop(fh->state, IMAGE_PAD);
	*try_crop = supported_modes[0].crop;

	mutex_unlock(&imx577->mutex);

	return 0;
}

static void imx577_adjust_exposure_range(struct imx577 *imx577)
{
	int exposure_max, exposure_def;

	/* Honour the VBLANK limits when setting exposure. */
	exposure_max = imx577->mode->height + imx577->vblank->val -
		       IMX577_EXPOSURE_OFFSET;
	exposure_def = min(exposure_max, imx577->exposure->val);
	__v4l2_ctrl_modify_range(imx577->exposure, imx577->exposure->minimum,
				 exposure_max, imx577->exposure->step,
				 exposure_def);
}

static int imx577_set_frame_length(struct imx577 *imx577, unsigned int val)
{
	return cci_write(imx577->regmap, IMX577_REG_FRAME_LENGTH, val, NULL);
}

static int imx577_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx577 *imx577 =
		container_of(ctrl->handler, struct imx577, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&imx577->sd);
	int ret = 0;

	/*
	 * The VBLANK control may change the limits of usable exposure, so check
	 * and adjust if necessary.
	 */
	if (ctrl->id == V4L2_CID_VBLANK)
		imx577_adjust_exposure_range(imx577);

	/*
	 * Applying V4L2 control value only happens
	 * when power is up for streaming
	 */
	if (pm_runtime_get_if_in_use(&client->dev) == 0)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_ANALOGUE_GAIN:
		ret = cci_write(imx577->regmap, IMX577_REG_ANALOG_GAIN,
				ctrl->val, NULL);
		break;
	case V4L2_CID_EXPOSURE:
		ret = cci_write(imx577->regmap, IMX577_REG_EXPOSURE,
				ctrl->val, NULL);
		break;
	case V4L2_CID_DIGITAL_GAIN:
		ret = cci_write(imx577->regmap, IMX577_REG_DGAIN_GR,
				ctrl->val, NULL);
		ret |= cci_write(imx577->regmap, IMX577_REG_DGAIN_R,
				 ctrl->val, NULL);
		ret |= cci_write(imx577->regmap, IMX577_REG_DGAIN_B,
				 ctrl->val, NULL);
		ret |= cci_write(imx577->regmap, IMX577_REG_DGAIN_GB,
				 ctrl->val, NULL);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = cci_write(imx577->regmap, IMX577_REG_TEST_PATTERN,
				imx577_test_pattern_val[ctrl->val], NULL);
		break;
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
		ret = cci_write(imx577->regmap, IMX577_REG_ORIENTATION,
				imx577->hflip->val | imx577->vflip->val << 1,
				NULL);
		break;
	case V4L2_CID_VBLANK:
		ret = imx577_set_frame_length(imx577,
					      imx577->mode->height + ctrl->val);
		break;
	case V4L2_CID_HBLANK:
		ret = cci_write(imx577->regmap, IMX577_REG_LINE_LENGTH,
				imx577->mode->width + ctrl->val, NULL);
		break;
	default:
		dev_info(&client->dev,
			 "ctrl(id:0x%x,val:0x%x) is not handled\n",
			 ctrl->id, ctrl->val);
		ret = -EINVAL;
		break;
	}

	pm_runtime_mark_last_busy(&client->dev);
	pm_runtime_put_autosuspend(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops imx577_ctrl_ops = {
	.s_ctrl = imx577_set_ctrl,
};

static int imx577_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct imx577 *imx577 = to_imx577(sd);

	if (code->pad >= NUM_PADS)
		return -EINVAL;

	if (code->pad == METADATA_PAD) {
		if (code->index > 0)
			return -EINVAL;
		code->code = MEDIA_BUS_FMT_SENSOR_DATA;
		return 0;
	}

	if (code->index > 0)
		return -EINVAL;

	mutex_lock(&imx577->mutex);
	code->code = imx577_get_format_code(imx577, codes[0]);
	mutex_unlock(&imx577->mutex);

	return 0;
}

/*
 * Pixel rate of the sensor: the readout clock is derived from the D-PHY bit
 * rate and the internal PLL, not from the number of data lanes, so it is the
 * same with two and four lanes (the line length is scaled instead).
 */
static u64 imx577_pixel_rate(const struct imx577 *imx577)
{
	return imx577->link_freq_value * 2 * IMX577_MAX_LANES /
	       IMX577_BITS_PER_SAMPLE;
}

/*
 * The mode tables assume four lanes.  The sensor derives its readout rate
 * from the D-PHY bit rate and the internal PLL, so with fewer lanes the
 * line length has to be stretched to keep the pixel rate on the CSI-2 link
 * within the narrower link bandwidth.
 */
static unsigned int imx577_line_length(const struct imx577 *imx577,
				       const struct imx577_mode *mode)
{
	return mode->line_length_default * IMX577_MAX_LANES / imx577->lanes;
}

static struct v4l2_fract imx577_mode_max_fps(struct imx577 *imx577,
					     const struct imx577_mode *mode)
{
	u64 fps = div_u64(imx577_pixel_rate(imx577),
			  (u64)imx577_line_length(imx577, mode) *
			  mode->frm_length_default);
	struct v4l2_fract interval = { 1, 1 };

	if (fps)
		interval.denominator = min_t(u64, fps, mode->max_fps.denominator);

	return interval;
}

static int imx577_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	unsigned int idx = 0;
	unsigned int i;

	if (fse->pad >= NUM_PADS)
		return -EINVAL;

	if (fse->pad == METADATA_PAD) {
		if (fse->code != MEDIA_BUS_FMT_SENSOR_DATA || fse->index > 0)
			return -EINVAL;
		fse->min_width = IMX577_EMBEDDED_LINE_WIDTH;
		fse->max_width = fse->min_width;
		fse->min_height = IMX577_NUM_EMBEDDED_LINES;
		fse->max_height = fse->min_height;
		return 0;
	}

	/*
	 * Only the mode selected by mode_idx is enumerated.  Returning
	 * -EINVAL for a valid code with no matching mode would make
	 * libcamera's V4L2Subdevice::formats() discard the whole map.
	 */
	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		const struct imx577_mode *mode = &supported_modes[i];

		if (mode_idx >= 0 && i != (unsigned int)mode_idx)
			continue;
		if (idx == fse->index) {
			fse->min_width = mode->width;
			fse->max_width = fse->min_width;
			fse->min_height = mode->height;
			fse->max_height = fse->min_height;
			return 0;
		}
		idx++;
	}

	return -EINVAL;
}

static int imx577_enum_frame_interval(struct v4l2_subdev *sd,
				      struct v4l2_subdev_state *sd_state,
				      struct v4l2_subdev_frame_interval_enum *fie)
{
	struct imx577 *imx577 = to_imx577(sd);
	const struct imx577_mode *mode;

	unsigned int idx = 0;
	unsigned int i;

	if (fie->pad >= NUM_PADS)
		return -EINVAL;

	if (fie->pad == METADATA_PAD) {
		if (fie->code != MEDIA_BUS_FMT_SENSOR_DATA || fie->index > 0)
			return -EINVAL;
		fie->width = IMX577_EMBEDDED_LINE_WIDTH;
		fie->height = IMX577_NUM_EMBEDDED_LINES;
		fie->interval = imx577_mode_max_fps(imx577, imx577->mode);
		return 0;
	}

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		mode = &supported_modes[i];
		if (mode_idx >= 0 && i != (unsigned int)mode_idx)
			continue;
		if (idx == fie->index) {
			fie->width = mode->width;
			fie->height = mode->height;
			fie->interval = imx577_mode_max_fps(imx577, mode);
			return 0;
		}
		idx++;
	}

	return -EINVAL;
}

static void imx577_reset_colorspace(struct v4l2_mbus_framefmt *fmt)
{
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_MAP_QUANTIZATION_DEFAULT(true,
							  fmt->colorspace,
							  fmt->ycbcr_enc);
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
}

static void imx577_update_metadata_pad_format(struct v4l2_subdev_format *fmt)
{
	fmt->format.width = IMX577_EMBEDDED_LINE_WIDTH;
	fmt->format.height = IMX577_NUM_EMBEDDED_LINES;
	fmt->format.code = MEDIA_BUS_FMT_SENSOR_DATA;
	fmt->format.field = V4L2_FIELD_NONE;
}

static void imx577_update_image_pad_format(struct imx577 *imx577,
					   const struct imx577_mode *mode,
					   struct v4l2_subdev_format *fmt)
{
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	imx577_reset_colorspace(&fmt->format);
}

static int imx577_get_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct imx577 *imx577 = to_imx577(sd);

	if (fmt->pad >= NUM_PADS)
		return -EINVAL;

	mutex_lock(&imx577->mutex);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		struct v4l2_mbus_framefmt *try_fmt =
			v4l2_subdev_state_get_format(sd_state, fmt->pad);

		if (fmt->pad == IMAGE_PAD) {
			/* update the code which could change due to vflip/hflip: */
			try_fmt->code =
				imx577_get_format_code(imx577, try_fmt->code);
		} else {
			try_fmt->code = MEDIA_BUS_FMT_SENSOR_DATA;
		}
		fmt->format = *try_fmt;
	} else if (fmt->pad == IMAGE_PAD) {
		imx577_update_image_pad_format(imx577, imx577->mode, fmt);
		fmt->format.code = imx577->fmt_code;
	} else {
		imx577_update_metadata_pad_format(fmt);
	}

	mutex_unlock(&imx577->mutex);

	return 0;
}

static void imx577_set_framing_limits(struct imx577 *imx577)
{
	const struct imx577_mode *mode = imx577->mode;
	unsigned int line_length = imx577_line_length(imx577, mode);
	unsigned int hblank_min = line_length - mode->width;
	unsigned int vblank_def = mode->frm_length_default - mode->height;

	lockdep_assert_held(&imx577->mutex);

	/* Update limits and set VBLANK to the mode default. */
	__v4l2_ctrl_modify_range(imx577->vblank, vblank_def,
				 IMX577_VTS_MAX - mode->height,
				 1, vblank_def);

	/* Setting this will adjust the exposure limits as well. */
	__v4l2_ctrl_s_ctrl(imx577->vblank, vblank_def);

	/*
	 * HBLANK is adjustable so that the IPA can stretch the line length when
	 * the ISP requires a slower line rate (e.g. still capture at full
	 * resolution needs >= 10.67us per line on PiSP).
	 */
	__v4l2_ctrl_modify_range(imx577->hblank, hblank_min,
				 0xffff - mode->width, 1, hblank_min);
}

/* Find the mode closest to the requested resolution. */
static const struct imx577_mode *
imx577_find_best_fit(struct v4l2_subdev_format *fmt)
{
	const struct imx577_mode *best = &supported_modes[0];
	unsigned int w = fmt->format.width, h = fmt->format.height;
	int best_dist = -1;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		const struct imx577_mode *mode = &supported_modes[i];
		int dist = abs((int)mode->width - (int)w) +
			   abs((int)mode->height - (int)h);

		if (best_dist < 0 || dist < best_dist) {
			best_dist = dist;
			best = mode;
		}
	}

	return best;
}

static int imx577_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt;
	const struct imx577_mode *mode;
	struct imx577 *imx577 = to_imx577(sd);

	if (fmt->pad >= NUM_PADS)
		return -EINVAL;

	mutex_lock(&imx577->mutex);

	if (fmt->pad == IMAGE_PAD) {
		if (mode_idx >= 0 && mode_idx < (int)ARRAY_SIZE(supported_modes))
			mode = &supported_modes[mode_idx];
		else
			mode = imx577_find_best_fit(fmt);
		fmt->format.code =
			imx577_get_format_code(imx577, fmt->format.code);
		imx577_update_image_pad_format(imx577, mode, fmt);

		if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
			framefmt = v4l2_subdev_state_get_format(sd_state,
								fmt->pad);
			*framefmt = fmt->format;
		} else if (imx577->mode != mode ||
			   imx577->fmt_code != fmt->format.code) {
			imx577->mode = mode;
			imx577->fmt_code = fmt->format.code;
			imx577_set_framing_limits(imx577);
		}
	} else {
		/* Only one embedded data mode is supported */
		imx577_update_metadata_pad_format(fmt);
		framefmt = v4l2_subdev_state_get_format(sd_state, fmt->pad);
		if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
			*framefmt = fmt->format;
	}

	mutex_unlock(&imx577->mutex);

	return 0;
}

static const struct v4l2_rect *
__imx577_get_pad_crop(struct imx577 *imx577,
		      struct v4l2_subdev_state *sd_state,
		      unsigned int pad, enum v4l2_subdev_format_whence which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_state_get_crop(sd_state, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &imx577->mode->crop;
	}

	return NULL;
}

static int imx577_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP: {
		struct imx577 *imx577 = to_imx577(sd);

		mutex_lock(&imx577->mutex);
		sel->r = *__imx577_get_pad_crop(imx577, sd_state, sel->pad,
						sel->which);
		mutex_unlock(&imx577->mutex);

		return 0;
	}

	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = IMX577_NATIVE_WIDTH;
		sel->r.height = IMX577_NATIVE_HEIGHT;

		return 0;

	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.left = IMX577_PIXEL_ARRAY_LEFT;
		sel->r.top = IMX577_PIXEL_ARRAY_TOP;
		sel->r.width = IMX577_PIXEL_ARRAY_WIDTH;
		sel->r.height = IMX577_PIXEL_ARRAY_HEIGHT;

		return 0;
	}

	return -EINVAL;
}

/*
 * Program the number of data lanes and the D-PHY bit rate requested by the
 * device tree.
 */
static int imx577_apply_link_config(struct imx577 *imx577)
{
	u64 bitrate = imx577->link_freq_value * 2 * imx577->lanes;
	int ret;

	ret = cci_write(imx577->regmap, IMX577_REG_NUM_LANES,
			imx577->lanes - 1, NULL);
	if (ret)
		return ret;

	return cci_write(imx577->regmap, IMX577_REG_REQ_LINK_BIT_RATE,
			 (bitrate / 1000000) << 16, NULL);
}

/* Start streaming */
static int imx577_start_streaming(struct imx577 *imx577)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx577->sd);
	const struct imx577_reg_list *reg_list;
	int ret;

	if (!imx577->common_regs_written) {
		ret = cci_multi_reg_write(imx577->regmap, mode_common_regs,
					  ARRAY_SIZE(mode_common_regs), NULL);
		if (ret) {
			dev_err(&client->dev,
				"%s failed to set common settings\n",
				__func__);
			return ret;
		}
		imx577->common_regs_written = true;
	}

	/* Apply default values of current mode */
	reg_list = &imx577->mode->reg_list;
	ret = cci_multi_reg_write(imx577->regmap, reg_list->regs,
				  reg_list->num_of_regs, NULL);
	if (ret) {
		dev_err(&client->dev, "%s failed to set mode\n", __func__);
		return ret;
	}

	ret = imx577_apply_link_config(imx577);
	if (ret) {
		dev_err(&client->dev, "%s failed to configure the link\n",
			__func__);
		return ret;
	}

	/* Apply customized values from user */
	ret = __v4l2_ctrl_handler_setup(imx577->sd.ctrl_handler);
	if (ret)
		return ret;

	/* set stream on register */
	cci_write(imx577->regmap, IMX577_REG_MODE_SELECT,
		  IMX577_MODE_STREAMING, &ret);

	return ret;
}

/* Stop streaming */
static void imx577_stop_streaming(struct imx577 *imx577)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx577->sd);
	int ret;

	/* set stream off register */
	ret = cci_write(imx577->regmap, IMX577_REG_MODE_SELECT,
			IMX577_MODE_STANDBY, NULL);
	if (ret)
		dev_err(&client->dev, "%s failed to set stream\n", __func__);
}

static int imx577_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct imx577 *imx577 = to_imx577(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;

	mutex_lock(&imx577->mutex);
	if (imx577->streaming == enable) {
		mutex_unlock(&imx577->mutex);
		return 0;
	}

	if (enable) {
		ret = pm_runtime_resume_and_get(&client->dev);
		if (ret < 0)
			goto err_unlock;

		/*
		 * Apply default & customized values
		 * and then start streaming.
		 */
		ret = imx577_start_streaming(imx577);
		if (ret) {
			pm_runtime_put_sync(&client->dev);
			goto err_unlock;
		}
	} else {
		imx577_stop_streaming(imx577);
		pm_runtime_mark_last_busy(&client->dev);
		pm_runtime_put_autosuspend(&client->dev);
	}

	imx577->streaming = enable;

	/* vflip and hflip cannot change during streaming */
	__v4l2_ctrl_grab(imx577->vflip, enable);
	__v4l2_ctrl_grab(imx577->hflip, enable);

	mutex_unlock(&imx577->mutex);

	return ret;

err_unlock:
	mutex_unlock(&imx577->mutex);

	return ret;
}

static int imx577_get_frame_interval(struct v4l2_subdev *sd,
				     struct v4l2_subdev_state *state,
				     struct v4l2_subdev_frame_interval *fi)
{
	struct imx577 *imx577 = to_imx577(sd);

	mutex_lock(&imx577->mutex);
	fi->interval = imx577_mode_max_fps(imx577, imx577->mode);
	mutex_unlock(&imx577->mutex);

	return 0;
}

/* Power/clock management functions */
static int imx577_power_on(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx577 *imx577 = to_imx577(sd);
	int ret;

	ret = regulator_bulk_enable(IMX577_NUM_SUPPLIES,
				    imx577->supplies);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable regulators\n",
			__func__);
		return ret;
	}

	ret = clk_prepare_enable(imx577->xclk);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable clock\n",
			__func__);
		goto reg_off;
	}

	gpiod_set_value_cansleep(imx577->reset_gpio, 1);
	usleep_range(IMX577_XCLR_MIN_DELAY_US,
		     IMX577_XCLR_MIN_DELAY_US + IMX577_XCLR_DELAY_RANGE_US);

	return 0;

reg_off:
	regulator_bulk_disable(IMX577_NUM_SUPPLIES, imx577->supplies);
	return ret;
}

static int imx577_power_off(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx577 *imx577 = to_imx577(sd);

	gpiod_set_value_cansleep(imx577->reset_gpio, 0);
	regulator_bulk_disable(IMX577_NUM_SUPPLIES, imx577->supplies);
	clk_disable_unprepare(imx577->xclk);

	/* Force reprogramming of the common registers when powered up again. */
	imx577->common_regs_written = false;

	return 0;
}

static int __maybe_unused imx577_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx577 *imx577 = to_imx577(sd);

	if (imx577->streaming)
		imx577_stop_streaming(imx577);

	return 0;
}

static int __maybe_unused imx577_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx577 *imx577 = to_imx577(sd);
	int ret;

	if (imx577->streaming) {
		ret = imx577_start_streaming(imx577);
		if (ret)
			goto error;
	}

	return 0;

error:
	imx577_stop_streaming(imx577);
	imx577->streaming = 0;
	return ret;
}

static int imx577_get_regulators(struct imx577 *imx577)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx577->sd);
	unsigned int i;

	for (i = 0; i < IMX577_NUM_SUPPLIES; i++)
		imx577->supplies[i].supply = imx577_supply_name[i];

	return devm_regulator_bulk_get(&client->dev,
				       IMX577_NUM_SUPPLIES,
				       imx577->supplies);
}

/* Verify chip ID */
static int imx577_identify_module(struct imx577 *imx577)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx577->sd);
	int ret;
	u64 val;

	ret = cci_read(imx577->regmap, IMX577_REG_CHIP_ID, &val, NULL);
	if (ret) {
		dev_err(&client->dev,
			"failed to read chip id %x, with error %d\n",
			IMX577_CHIP_ID, ret);
		return ret;
	}

	/*
	 * A mismatch means the sensor is not answering where we expect it,
	 * so fail instead of driving an unknown device.
	 */
	if (val != IMX577_CHIP_ID) {
		dev_err(&client->dev,
			"chip id mismatch: expected 0x%04x, read 0x%04llx\n",
			IMX577_CHIP_ID, val);
		return -EIO;
	}

	dev_info(&client->dev, "Device found is imx%04llx\n", val);

	return 0;
}

static const struct v4l2_subdev_core_ops imx577_core_ops = {
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_video_ops imx577_video_ops = {
	.s_stream = imx577_set_stream,
};

static const struct v4l2_subdev_pad_ops imx577_pad_ops = {
	.enum_mbus_code = imx577_enum_mbus_code,
	.get_fmt = imx577_get_pad_format,
	.set_fmt = imx577_set_pad_format,
	.get_selection = imx577_get_selection,
	.enum_frame_size = imx577_enum_frame_size,
	.enum_frame_interval = imx577_enum_frame_interval,
	.get_frame_interval = imx577_get_frame_interval,
};

static const struct v4l2_subdev_ops imx577_subdev_ops = {
	.core = &imx577_core_ops,
	.video = &imx577_video_ops,
	.pad = &imx577_pad_ops,
};

static const struct v4l2_subdev_internal_ops imx577_internal_ops = {
	.open = imx577_open,
};

/* Initialize control handlers */
static int imx577_init_controls(struct imx577 *imx577)
{
	struct v4l2_ctrl_handler *ctrl_hdlr;
	struct i2c_client *client = v4l2_get_subdevdata(&imx577->sd);
	struct v4l2_fwnode_device_properties props;
	int ret;

	ctrl_hdlr = &imx577->ctrl_handler;
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 16);
	if (ret)
		return ret;

	mutex_init(&imx577->mutex);
	ctrl_hdlr->lock = &imx577->mutex;

	/* By default, PIXEL_RATE is read only */
	imx577->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, &imx577_ctrl_ops,
					       V4L2_CID_PIXEL_RATE,
					       imx577_pixel_rate(imx577),
					       imx577_pixel_rate(imx577), 1,
					       imx577_pixel_rate(imx577));
	if (imx577->pixel_rate)
		imx577->pixel_rate->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/* LINK_FREQ is also read only */
	imx577->link_freq =
		v4l2_ctrl_new_int_menu(ctrl_hdlr, &imx577_ctrl_ops,
				       V4L2_CID_LINK_FREQ, 0, 0,
				       &imx577->link_freq_value);
	if (imx577->link_freq)
		imx577->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/*
	 * Create the controls here, but mode specific limits are setup
	 * in the imx577_set_framing_limits() call below.
	 */
	imx577->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &imx577_ctrl_ops,
					   V4L2_CID_VBLANK, IMX577_VBLANK_MIN,
					   0xffff, 1, IMX577_VBLANK_MIN);
	/*
	 * HBLANK is writable: the IPA needs to stretch the line length when the
	 * ISP requires a slower line rate than the mode's default.
	 */
	imx577->hblank = v4l2_ctrl_new_std(ctrl_hdlr, &imx577_ctrl_ops,
					   V4L2_CID_HBLANK, 0, 0xffff, 1, 0);

	imx577->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &imx577_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     IMX577_EXPOSURE_MIN,
					     IMX577_VTS_MAX -
					     IMX577_EXPOSURE_OFFSET,
					     IMX577_EXPOSURE_STEP,
					     IMX577_EXPOSURE_DEFAULT);

	v4l2_ctrl_new_std(ctrl_hdlr, &imx577_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  IMX577_ANA_GAIN_MIN, IMX577_ANA_GAIN_MAX,
			  IMX577_ANA_GAIN_STEP, IMX577_ANA_GAIN_DEFAULT);

	v4l2_ctrl_new_std(ctrl_hdlr, &imx577_ctrl_ops, V4L2_CID_DIGITAL_GAIN,
			  IMX577_DGAIN_MIN, IMX577_DGAIN_MAX,
			  IMX577_DGAIN_STEP, IMX577_DGAIN_DEFAULT);

	imx577->hflip = v4l2_ctrl_new_std(ctrl_hdlr, &imx577_ctrl_ops,
					  V4L2_CID_HFLIP, 0, 1, 1, 0);
	if (imx577->hflip)
		imx577->hflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	imx577->vflip = v4l2_ctrl_new_std(ctrl_hdlr, &imx577_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 1, 1, 0);
	if (imx577->vflip)
		imx577->vflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	v4l2_ctrl_new_std_menu_items(ctrl_hdlr, &imx577_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(imx577_test_pattern_menu) - 1,
				     0, 0, imx577_test_pattern_menu);

	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(&client->dev, "%s control init failed (%d)\n",
			__func__, ret);
		goto error;
	}

	ret = v4l2_fwnode_device_parse(&client->dev, &props);
	if (ret)
		goto error;

	ret = v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &imx577_ctrl_ops,
					      &props);
	if (ret)
		goto error;

	imx577->sd.ctrl_handler = ctrl_hdlr;

	mutex_lock(&imx577->mutex);

	/* Setup exposure and frame/line length limits. */
	imx577_set_framing_limits(imx577);

	mutex_unlock(&imx577->mutex);

	return 0;

error:
	v4l2_ctrl_handler_free(ctrl_hdlr);
	mutex_destroy(&imx577->mutex);

	return ret;
}

static void imx577_free_controls(struct imx577 *imx577)
{
	v4l2_ctrl_handler_free(imx577->sd.ctrl_handler);
	mutex_destroy(&imx577->mutex);
}

static int imx577_check_hwcfg(struct device *dev, struct imx577 *imx577)
{
	struct fwnode_handle *endpoint;
	struct v4l2_fwnode_endpoint ep_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY
	};
	unsigned int i;
	int ret = -EINVAL;

	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (!endpoint) {
		dev_err(dev, "endpoint node not found\n");
		return -EINVAL;
	}

	if (v4l2_fwnode_endpoint_alloc_parse(endpoint, &ep_cfg)) {
		dev_err(dev, "could not parse endpoint\n");
		goto error_out;
	}

	/* Check the number of MIPI CSI2 data lanes */
	switch (ep_cfg.bus.mipi_csi2.num_data_lanes) {
	case 2:
	case 4:
		imx577->lanes = ep_cfg.bus.mipi_csi2.num_data_lanes;
		break;
	default:
		dev_err(dev, "only %u or %u data lanes are supported\n",
			IMX577_MIN_LANES, IMX577_MAX_LANES);
		goto error_out;
	}

	/* Check the link frequency set in device tree */
	if (!ep_cfg.nr_of_link_frequencies) {
		dev_err(dev, "link-frequency property not found in DT\n");
		goto error_out;
	}

	for (i = 0; i < ep_cfg.nr_of_link_frequencies; i++) {
		if (ep_cfg.link_frequencies[i] == IMX577_LINK_FREQ_750MHZ) {
			imx577->link_freq_value = ep_cfg.link_frequencies[i];
			break;
		}
	}

	if (i == ep_cfg.nr_of_link_frequencies) {
		dev_err(dev, "No supported link frequency configured\n");
		ret = -EINVAL;
		goto error_out;
	}

	ret = 0;

error_out:
	v4l2_fwnode_endpoint_free(&ep_cfg);
	fwnode_handle_put(endpoint);

	return ret;
}

static const struct of_device_id imx577_dt_ids[] = {
	{ .compatible = "sony,imx577" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx577_dt_ids);

static int imx577_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct imx577 *imx577;
	int ret;

	imx577 = devm_kzalloc(&client->dev, sizeof(*imx577), GFP_KERNEL);
	if (!imx577)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&imx577->sd, client, &imx577_subdev_ops);

	if (imx477_tuning)
		v4l2_i2c_subdev_set_name(&imx577->sd, client, "imx477", NULL);

	imx577->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(imx577->regmap)) {
		ret = PTR_ERR(imx577->regmap);
		dev_err(&client->dev, "failed to initialize CCI: %d\n", ret);
		return ret;
	}

	/* Check the hardware configuration in device tree */
	if (imx577_check_hwcfg(dev, imx577))
		return -EINVAL;

	/* Get system clock (xclk) */
	imx577->xclk = devm_clk_get(dev, NULL);
	if (IS_ERR(imx577->xclk)) {
		dev_err(dev, "failed to get xclk\n");
		return PTR_ERR(imx577->xclk);
	}

	imx577->xclk_freq = clk_get_rate(imx577->xclk);
	if (imx577->xclk_freq != IMX577_XCLK_FREQ) {
		dev_err(dev, "xclk frequency not supported: %d Hz\n",
			imx577->xclk_freq);
		return -EINVAL;
	}

	ret = imx577_get_regulators(imx577);
	if (ret) {
		dev_err(dev, "failed to get regulators\n");
		return ret;
	}

	/* Request optional enable pin */
	imx577->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						     GPIOD_OUT_HIGH);

	/*
	 * The sensor must be powered for imx577_identify_module()
	 * to be able to read the CHIP_ID register
	 */
	ret = imx577_power_on(dev);
	if (ret)
		return ret;

	ret = imx577_identify_module(imx577);
	if (ret)
		goto error_power_off;

	/* Initialize default format */
	imx577_set_default_format(imx577);

	/*
	 * Enable runtime PM with 5s autosuspend. As the device has been powered
	 * manually, mark it as active, and increase the usage count without
	 * resuming the device.
	 */
	pm_runtime_set_active(dev);
	pm_runtime_get_noresume(dev);
	pm_runtime_enable(dev);
	pm_runtime_set_autosuspend_delay(dev, 5000);
	pm_runtime_use_autosuspend(dev);

	/* This needs the pm runtime to be registered. */
	ret = imx577_init_controls(imx577);
	if (ret)
		goto error_power_off;

	/* Initialize subdev */
	imx577->sd.internal_ops = &imx577_internal_ops;
	imx577->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
			    V4L2_SUBDEV_FL_HAS_EVENTS;
	imx577->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	/* Initialize source pads */
	imx577->pad[IMAGE_PAD].flags = MEDIA_PAD_FL_SOURCE;
	imx577->pad[METADATA_PAD].flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&imx577->sd.entity, NUM_PADS,
				     imx577->pad);
	if (ret) {
		dev_err(dev, "failed to init entity pads: %d\n", ret);
		goto error_handler_free;
	}

	ret = v4l2_async_register_subdev_sensor(&imx577->sd);
	if (ret < 0) {
		dev_err(dev, "failed to register sensor sub-device: %d\n", ret);
		goto error_media_entity;
	}

	pm_runtime_mark_last_busy(&client->dev);
	pm_runtime_put_autosuspend(&client->dev);

	return 0;

error_media_entity:
	media_entity_cleanup(&imx577->sd.entity);

error_handler_free:
	imx577_free_controls(imx577);

error_power_off:
	pm_runtime_disable(&client->dev);
	pm_runtime_put_noidle(&client->dev);
	imx577_power_off(&client->dev);

	return ret;
}

static void imx577_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx577 *imx577 = to_imx577(sd);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	imx577_free_controls(imx577);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		imx577_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);
}

static const struct dev_pm_ops imx577_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(imx577_suspend, imx577_resume)
	SET_RUNTIME_PM_OPS(imx577_power_off, imx577_power_on, NULL)
};

static struct i2c_driver imx577_i2c_driver = {
	.driver = {
		.name = "imx577",
		.of_match_table	= imx577_dt_ids,
		.pm = &imx577_pm_ops,
	},
	.probe = imx577_probe,
	.remove = imx577_remove,
};

module_i2c_driver(imx577_i2c_driver);

MODULE_AUTHOR("Yu Dukai <646689853@qq.com>");
MODULE_DESCRIPTION("Sony IMX577 sensor driver");
MODULE_LICENSE("GPL");
