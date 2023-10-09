/* SPDX-License-Identifier: GPL-2.0-only
* U-Boot port of the linux touchscreen driver cyttsp5.c
*
* Linux author information:
* Parade TrueTouch(TM) Standard Product V5 Module.
*
* Copyright (C) 2015 Parade Technologies
* Copyright (C) 2012-2015 Cypress Semiconductor
* Copyright (C) 2018 Bootlin
*
* Authors: Mylène Josserand <mylene.josserand@bootlin.com>
*                Alistair Francis <alistair@alistair23.me>
*
* U-Boot port by:
* Maximilian Weigand <mweigand@mweigand.net>
*
* U-Boot notes:
* This is a hacky/investigative port and would need to be cleaned up for
* long-time use. However, it works on the PineNote, which for now is enough.
*
* Notes:
*
* * The irq handling during the probing phase is ... shaky. The linux driver
* initially triggered on LEVEL_LOG, but I'm pretty sure the currently
* implementation requires irq handling on the falling edge. At least the linux
* driver works more reliable then. For now most of the probing code uses
* hard-coded sleeps to wait for the device responses. This works on the
* PineNote, but should be fixed for wider use.
*/
#define DEBUG
#include <common.h>
#include <dm.h>
#include <i2c.h>
#include <irq-generic.h>
#include <asm/gpio.h>
#include <asm/unaligned.h>
#include <input.h>
#include <linux/input.h>
#include <stdio_dev.h>

#define CYTTSP5_NAME				"cyttsp5"
#define CY_I2C_DATA_SIZE			(2 * 256)
#define HID_VERSION				0x0100
#define CY_MAX_INPUT				512
#define CYTTSP5_PREALLOCATED_CMD_BUFFER	32
#define CY_BITS_PER_BTN			1
#define CY_NUM_BTN_EVENT_ID			GENMASK(CY_BITS_PER_BTN, 0)

#define MAX_AREA				255
#define HID_OUTPUT_BL_SOP			0x1
#define HID_OUTPUT_BL_EOP			0x17
#define HID_OUTPUT_BL_LAUNCH_APP		0x3B
#define HID_OUTPUT_BL_LAUNCH_APP_SIZE		11
#define HID_OUTPUT_GET_SYSINFO			0x2
#define HID_OUTPUT_GET_SYSINFO_SIZE		5
#define HID_OUTPUT_MAX_CMD_SIZE		12

#define HID_DESC_REG				0x1
#define HID_INPUT_REG				0x3
#define HID_OUTPUT_REG				0x4

#define REPORT_ID_TOUCH			0x1
#define REPORT_ID_BTN				0x3
#define REPORT_SIZE_5				5
#define REPORT_SIZE_8				8
#define REPORT_SIZE_16				16

/* Touch reports offsets */
/* Header offsets */
#define TOUCH_REPORT_DESC_HDR_CONTACTCOUNT	16
/* Record offsets */
#define TOUCH_REPORT_DESC_CONTACTID		8
#define TOUCH_REPORT_DESC_X			16
#define TOUCH_REPORT_DESC_Y			32
#define TOUCH_REPORT_DESC_P			48
#define TOUCH_REPORT_DESC_MAJ			56
#define TOUCH_REPORT_DESC_MIN			64

/* HID */
#define HID_TOUCH_REPORT_ID			0x1
#define HID_BTN_REPORT_ID			0x3
#define HID_APP_RESPONSE_REPORT_ID		0x1F
#define HID_APP_OUTPUT_REPORT_ID		0x2F
#define HID_BL_RESPONSE_REPORT_ID		0x30
#define HID_BL_OUTPUT_REPORT_ID		0x40

#define HID_OUTPUT_RESPONSE_REPORT_OFFSET	2
#define HID_OUTPUT_RESPONSE_CMD_OFFSET		4
#define HID_OUTPUT_RESPONSE_CMD_MASK		GENMASK(6, 0)

#define HID_SYSINFO_SENSING_OFFSET		33
#define HID_SYSINFO_BTN_OFFSET			48
#define HID_SYSINFO_BTN_MASK			GENMASK(7, 0)
#define HID_SYSINFO_MAX_BTN			8

#define CY_HID_OUTPUT_TIMEOUT_MS			200
#define CY_HID_OUTPUT_GET_SYSINFO_TIMEOUT_MS	3000
#define CY_HID_GET_HID_DESCRIPTOR_TIMEOUT_MS	4000

/* maximum number of concurrent tracks */
#define TOUCH_REPORT_SIZE			10
#define TOUCH_INPUT_HEADER_SIZE		7
#define BTN_REPORT_SIZE			9
#define BTN_INPUT_HEADER_SIZE			5

#define MAX_CY_TCH_T_IDS	32

/* All usage pages for Touch Report */
#define TOUCH_REPORT_USAGE_PG_X		0x00010030
#define TOUCH_REPORT_USAGE_PG_Y		0x00010031
#define TOUCH_REPORT_USAGE_PG_P		0x000D0030
#define TOUCH_REPORT_USAGE_PG_CONTACTID	0x000D0051
#define TOUCH_REPORT_USAGE_PG_CONTACTCOUNT	0x000D0054
#define TOUCH_REPORT_USAGE_PG_MAJ		0xFF010062
#define TOUCH_REPORT_USAGE_PG_MIN		0xFF010063
#define TOUCH_COL_USAGE_PG			0x000D0022

#define TOUCH_DEAD_TIME_MS 500

/* STDIO START*/

static struct input_config button_input;

/* STDIO END */

/* crc itu implementation, from kernel source: lib/crc-itu-t.c */
// SPDX-License-Identifier: GPL-2.0-only

/* CRC table for the CRC ITU-T V.41 0x1021 (x^16 + x^12 + x^5 + 1) */
const u16 crc_itu_t_table[256] = {
	0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
	0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
	0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
	0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
	0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
	0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
	0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
	0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
	0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
	0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
	0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
	0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
	0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
	0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
	0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
	0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
	0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
	0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
	0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
	0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
	0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
	0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
	0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
	0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
	0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
	0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
	0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
	0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
	0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
	0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
	0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
	0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
};

static inline u16 crc_itu_t_byte(u16 crc, const u8 data)
{
	return (crc << 8) ^ crc_itu_t_table[((crc >> 8) ^ data) & 0xff];
}

/**
 * crc_itu_t - Compute the CRC-ITU-T for the data buffer
 *
 * @crc:     previous CRC value
 * @buffer:  data pointer
 * @len:     number of bytes in the buffer
 *
 * Returns the updated CRC value
 */
u16 crc_itu_t(u16 crc, const u8 *buffer, size_t len)
{
	while (len--)
		crc = crc_itu_t_byte(crc, *buffer++);
	return crc;
}

/* crc_itu code import end */

/* System Information interface definitions */
struct cyttsp5_sensing_conf_data_dev {
	u8 electrodes_x;
	u8 electrodes_y;
	__le16 len_x;
	__le16 len_y;
	__le16 res_x;
	__le16 res_y;
	__le16 max_z;
	u8 origin_x;
	u8 origin_y;
	u8 btn;
	u8 scan_mode;
	u8 max_num_of_tch_per_refresh_cycle;
} __packed;

struct cyttsp5_sensing_conf_data {
	u16 res_x;
	u16 res_y;
	u16 max_z;
	u16 len_x;
	u16 len_y;
	u8 origin_x;
	u8 origin_y;
	u8 max_tch;
};

enum cyttsp5_tch_abs {	/* for ordering within the extracted touch data array */
	CY_TCH_X,	/* X */
	CY_TCH_Y,	/* Y */
	CY_TCH_P,	/* P (Z) */
	CY_TCH_T,	/* TOUCH ID */
	CY_TCH_MAJ,	/* TOUCH_MAJOR */
	CY_TCH_MIN,	/* TOUCH_MINOR */
	CY_TCH_NUM_ABS
};

struct cyttsp5_tch_abs_params {
	size_t ofs;	/* abs byte offset */
	size_t size;	/* size in bits */
	size_t min;	/* min value */
	size_t max;	/* max value */
	size_t bofs;	/* bit offset */
};

struct cyttsp5_touch {
	int abs[CY_TCH_NUM_ABS];
};

struct cyttsp5_sysinfo {
	struct cyttsp5_sensing_conf_data sensing_conf_data;
	int num_btns;
	struct cyttsp5_tch_abs_params tch_hdr;
	struct cyttsp5_tch_abs_params tch_abs[CY_TCH_NUM_ABS];
	u32 key_code[HID_SYSINFO_MAX_BTN];
};

struct cyttsp5_hid_desc {
	__le16 hid_desc_len;
	u8 packet_id;
	u8 reserved_byte;
	__le16 bcd_version;
	__le16 report_desc_len;
	__le16 report_desc_register;
	__le16 input_register;
	__le16 max_input_len;
	__le16 output_register;
	__le16 max_output_len;
	__le16 command_register;
	__le16 data_register;
	__le16 vendor_id;
	__le16 product_id;
	__le16 version_id;
	u8 reserved[4];
} __packed;

struct key_area {
	/* define the key/button area */
	u32 xmin;
	u32 xmax;
	u32 ymin;
	u32 ymax;

	/* key code to emit */
	int key_code;

	ulong last_press;
};

struct cyttsp5 {
	struct udevice *dev;
	/* struct completion cmd_done; */
	struct cyttsp5_sysinfo sysinfo;
	struct cyttsp5_hid_desc hid_desc;
	u8 cmd_buf[CYTTSP5_PREALLOCATED_CMD_BUFFER];
	u8 input_buf[CY_MAX_INPUT];
	u8 response_buf[CY_MAX_INPUT];
	struct gpio_desc *reset_gpio;
	struct gpio_desc *int_gpio;
	u32 gpio_int[2]; /* gpios[0]: gpio controller phandle, gpios[1]: pin */
	struct input_dev *input;
	/* char phys[NAME_MAX]; */
	char phys[12];
	int num_prv_rec;
	struct regmap *regmap;
	/* struct touchscreen_properties prop; */
	struct regulator *vdd;
	int irq;
	int triggered;
	/* We are implementing a VERY VERY simple trigger scheme here, basically:
	 * only allow an touch event every TOUCH_DEAD_TIME_MS ms
	 * This here is the timer */
	ulong last_touch_event;
	u8 key_count;
	struct key_area *key_areas;
	/* store last coordinates used to trigger a key press event
	 * we use this as a simple way to prevent duplicate key presses because the
	 * driver, as (incorrectly transferred from linux?) implemented sends a
	 * touch event with the same coordinates when the finger is lifted.
	 * */
	u32 last_x;
	u32 last_y;
	u32 last_pressure;
};

/* **********************************************************************/



/*
 * For what is understood in the datasheet, the register does not
 * matter. For consistency, use the Input Register address
 * but it does mean anything to the device. The important data
 * to send is the I2C address
 */
static int cyttsp5_read(struct cyttsp5 *ts, u8 *buf, u32 max)
{
	int error;
	u32 size;
	u8 temp[2];

	/* Read the frame to retrieve the size */
	error = dm_i2c_read(ts->dev, HID_INPUT_REG, temp, 2);
	/* error = regmap_bulk_read(ts->regmap, HID_INPUT_REG, temp, sizeof(temp)); */
	if (error)
		return error;

	size = get_unaligned_le16(temp);
	/* printf("NEW READ: %i %i -> %i (err:%i)\n", temp[0], temp[1], size, error); */
	if (!size || size == 2)
		return 0;

	if (size > max)
		return -EINVAL;

	/* printf("Reading frame of size: %i\n", size); */
	/* Get the real value */
	/* return regmap_bulk_read(ts->regmap, HID_INPUT_REG, buf, size); */
	return dm_i2c_read(ts->dev, HID_INPUT_REG, buf, size);
}

static int cyttsp5_write(struct cyttsp5 *ts, unsigned int reg, u8 *data,
			 size_t size)
{
	u8 cmd[HID_OUTPUT_MAX_CMD_SIZE];

	if (size + 1 > HID_OUTPUT_MAX_CMD_SIZE)
		return -E2BIG;

	/* High bytes of register address needed as first byte of cmd */
	cmd[0] = (reg >> 8) & 0xFF ;

	/* Copy the rest of the data */
	if (data)
		memcpy(&cmd[1], data, size);

	/*
	 * The hardware wants to receive a frame with the address register
	 * contained in the first two bytes. As the regmap_write function
	 * add the register adresse in the frame, we use the low byte as
	 * first frame byte for the address register and the first
	 * data byte is the high register + left of the cmd to send
	 */
	return dm_i2c_write(ts->dev, reg & 0xFF, cmd, size + 1);
	/* return regmap_bulk_write(ts->regmap, reg & 0xFF, cmd, size + 1); */
}

static void cyttsp5_get_touch_axis(int *axis, int size, int max, u8 *xy_data,
				   int bofs)
{
	int nbyte;

	for (nbyte = 0, *axis = 0; nbyte < size; nbyte++)
		*axis += ((xy_data[nbyte] >> bofs) << (nbyte * 8));

	*axis &= max - 1;
}

static void cyttsp5_get_touch_record(struct cyttsp5 *ts,
				     struct cyttsp5_touch *touch, u8 *xy_data)
{
	struct cyttsp5_sysinfo *si = &ts->sysinfo;
	enum cyttsp5_tch_abs abs;

	for (abs = CY_TCH_X; abs < CY_TCH_NUM_ABS; abs++)
		cyttsp5_get_touch_axis(&touch->abs[abs],
				       si->tch_abs[abs].size,
				       si->tch_abs[abs].max,
				       xy_data + si->tch_abs[abs].ofs,
				       si->tch_abs[abs].bofs);
}

static void update_key_timers(struct cyttsp5 *ts, int x, int y, int pressure){
	int key_nr;
	struct key_area *area_ptr;
	area_ptr = ts->key_areas;
	/* loop through keys */
	for (key_nr=1; key_nr <= ts->key_count; key_nr++){
		/* printf("Duplicate check: [%i] %i\n", key_nr, !((x == ts->last_x) && (y == ts->last_y) && (pressure == ts->last_pressure))); */
		/* printf("    %i %i: [%i/%i %i/%i]\n", x, y, */
		/* 		area_ptr->xmin, */
		/* 		area_ptr->xmax, */
		/* 		area_ptr->ymin, */
		/* 		area_ptr->ymax */
		/* 	); */

		if( !((x == ts->last_x) && (y == ts->last_y) && (pressure == ts->last_pressure)) && (x >= area_ptr->xmin) && (x < area_ptr->xmax) && (y >=
					area_ptr->ymin) && (y < area_ptr->ymax)){
			/* printf("Updating KEY[%i] = (%u/%u) (%u/%u) -> %i\n", */
			/* 		key_nr, */
			/* 		area_ptr->xmin, */
			/* 		area_ptr->xmax, */
			/* 		area_ptr->ymin, */
			/* 		area_ptr->ymax, */
			/* 		area_ptr->key_code */
			/* ); */
			area_ptr->last_press = get_timer(0);
		}
		area_ptr++;
	}
	ts->last_x = x;
	ts->last_y = y;
	ts->last_pressure = pressure;
}


static void cyttsp5_get_mt_touches(struct cyttsp5 *ts,
				   struct cyttsp5_touch *tch, int num_cur_tch)
{
	struct cyttsp5_sysinfo *si = &ts->sysinfo;
	int i;
	/* int t = 0; */
    int	offset = 0;
	/* DECLARE_BITMAP(ids, MAX_CY_TCH_T_IDS); */
	u8 *tch_addr;
	int tmp;

	/* bitmap_zero(ids, MAX_CY_TCH_T_IDS); */
	memset(tch->abs, 0, sizeof(tch->abs));

	switch (ts->input_buf[2]) {
	case HID_TOUCH_REPORT_ID:
		offset = TOUCH_INPUT_HEADER_SIZE;
		break;
	case HID_BTN_REPORT_ID:
		offset = BTN_INPUT_HEADER_SIZE;
		break;
	}

	for (i = 0; i < num_cur_tch; i++) {
		/* printf("touch nr: %i\n", i); */
		tch_addr = ts->input_buf + offset + (i * TOUCH_REPORT_SIZE);
		cyttsp5_get_touch_record(ts, tch, tch_addr);
		/* for (int i=0; i<6; i++) */
		/* 	printf("    index[%i] = %i\n", i, tch->abs[i]); */

		/* Convert MAJOR/MINOR from mm to resolution */
		tmp = tch->abs[CY_TCH_MAJ] * 100 * si->sensing_conf_data.res_x;
		tch->abs[CY_TCH_MAJ] = tmp / si->sensing_conf_data.len_x;
		tmp = tch->abs[CY_TCH_MIN] * 100 * si->sensing_conf_data.res_x;
		tch->abs[CY_TCH_MIN] = tmp / si->sensing_conf_data.len_x;
		/* for (int i=0; i<6; i++) */
		/* 	printf("    index[%i] = %i\n", i, tch->abs[i]); */
		/* printf("%i X/Y/P: (%i/%i/%i)\n", i, tch->abs[0], tch->abs[1], tch->abs[2]); */
		/* simple debounce */
		if (get_timer(ts->last_touch_event) > TOUCH_DEAD_TIME_MS){
			// trigger a key press depending on location
			/* printf("%i X/Y/P: (%i/%i/%i)\n", i, tch->abs[0], tch->abs[1], tch->abs[2]); */
			// reset timer
			ts->last_touch_event = get_timer(0);
			update_key_timers(ts, tch->abs[0], tch->abs[1], tch->abs[2]);
		}

		/* t = tch->abs[CY_TCH_T]; */
		/* printf("t: %i\n", t); */
		/* input_mt_slot(ts->input, t); */
		/* input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, true); */
		/* __set_bit(t, ids); */

		/* position and pressure fields */
		/* touchscreen_report_pos(ts->input, &ts->prop, */
		/* 		       tch->abs[CY_TCH_X], tch->abs[CY_TCH_Y], */
		/* 		       true); */
		/* input_report_abs(ts->input, ABS_MT_PRESSURE, */
		/* 		 tch->abs[CY_TCH_P]); */

		/* Get the extended touch fields */
		/* input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, */
		/* 		 tch->abs[CY_TCH_MAJ]); */
		/* input_report_abs(ts->input, ABS_MT_TOUCH_MINOR, */
		/* 		 tch->abs[CY_TCH_MIN]); */
	}

	ts->num_prv_rec = num_cur_tch;
}

static int cyttsp5_mt_attention(struct udevice *dev)
{
	/* struct cyttsp5 *ts = dev_get_drvdata(dev); */
	struct cyttsp5 *ts = dev_get_priv(dev);
	struct cyttsp5_sysinfo *si = &ts->sysinfo;
	int max_tch = si->sensing_conf_data.max_tch;
	struct cyttsp5_touch tch;
	u8 num_cur_tch;
	/* int * int_ptr = (int *) &num_cur_tch; */
	int tmp_int;

	cyttsp5_get_touch_axis(&tmp_int, si->tch_hdr.size,
			       si->tch_hdr.max,
			       ts->input_buf + 3 + si->tch_hdr.ofs,
			       si->tch_hdr.bofs);
	num_cur_tch = (u8) tmp_int;

	if (num_cur_tch > max_tch) {
		dev_err(dev, "Num touch err detected (n=%d)\n", num_cur_tch);
		num_cur_tch = max_tch;
	}

	if (num_cur_tch == 0 && ts->num_prv_rec == 0)
		return 0;

	/* extract xy_data for all currently reported touches */
	if (num_cur_tch)
		cyttsp5_get_mt_touches(ts, &tch, num_cur_tch);

	/* input_mt_sync_frame(ts->input); */
	/* input_sync(ts->input); */

	return 0;
}

/* this is meant to de-assert the interrupt line. I'm pretty sure this does not
 * work */
static int cyttsp5_deassert_int(struct cyttsp5 *ts)
{
	u16 size;
	u8 buf[2];
	int error;
	/* struct dm_i2c_chip *chip = dev_get_parent_platdata(ts->dev); */

	error = dm_i2c_read(ts->dev, HID_INPUT_REG, buf, 2);
	/* error = regmap_bulk_read(ts->regmap, HID_INPUT_REG, buf, sizeof(buf)); */
	if (error < 0)
		return error;
	printf("cyttsp5_deassert_int returned: %i %i\n", buf[0], buf[1]);

	size = get_unaligned_le16(&buf[0]);
	if (size == 2 || size == 0)
		return 0;

	return -EINVAL;
}

static int fill_tch_abs(struct cyttsp5_tch_abs_params *tch_abs, int report_size,
			int offset)
{
	tch_abs->ofs = offset / 8;
	tch_abs->size = report_size / 8;
	if (report_size % 8)
		tch_abs->size += 1;
	tch_abs->min = 0;
	tch_abs->max = 1 << report_size;
	tch_abs->bofs = offset - (tch_abs->ofs << 3);

	return 0;
}

static int cyttsp5_fill_all_touch(struct cyttsp5 *ts)
{
	struct cyttsp5_sysinfo *si = &ts->sysinfo;

	fill_tch_abs(&si->tch_abs[CY_TCH_X], REPORT_SIZE_16,
		     TOUCH_REPORT_DESC_X);
	fill_tch_abs(&si->tch_abs[CY_TCH_Y], REPORT_SIZE_16,
		     TOUCH_REPORT_DESC_Y);
	fill_tch_abs(&si->tch_abs[CY_TCH_P], REPORT_SIZE_8,
		     TOUCH_REPORT_DESC_P);
	fill_tch_abs(&si->tch_abs[CY_TCH_T], REPORT_SIZE_5,
		     TOUCH_REPORT_DESC_CONTACTID);
	fill_tch_abs(&si->tch_hdr, REPORT_SIZE_5,
		     TOUCH_REPORT_DESC_HDR_CONTACTCOUNT);
	fill_tch_abs(&si->tch_abs[CY_TCH_MAJ], REPORT_SIZE_8,
		     TOUCH_REPORT_DESC_MAJ);
	fill_tch_abs(&si->tch_abs[CY_TCH_MIN], REPORT_SIZE_8,
		     TOUCH_REPORT_DESC_MIN);

	return 0;
}

static int cyttsp5_validate_cmd_response(struct cyttsp5 *ts, u8 code)
{
	u16 size, crc;
	u8 status, report_id;
	int command_code;

	size = get_unaligned_le16(&ts->response_buf[0]);

	if (!size)
		return 0;

	report_id = ts->response_buf[HID_OUTPUT_RESPONSE_REPORT_OFFSET];

	switch (report_id) {
	case HID_BL_RESPONSE_REPORT_ID: {
		if (ts->response_buf[4] != HID_OUTPUT_BL_SOP) {
			printf("HID output response, wrong SOP\n");
			return -EPROTO;
		}

		if (ts->response_buf[size - 1] != HID_OUTPUT_BL_EOP) {
			printf("HID output response, wrong EOP\n");
			return -EPROTO;
		}

		crc = crc_itu_t(0xFFFF, &ts->response_buf[4], size - 7);
		if (get_unaligned_le16(&ts->response_buf[size - 3]) != crc) {
			printf("HID output response, wrong CRC 0x%X\n",
				crc);
			return -EPROTO;
		}

		status = ts->response_buf[5];
		if (status) {
			printf("HID output response, ERROR:%d\n",
				status);
			return -EPROTO;
		}
		break;
	}
	case HID_APP_RESPONSE_REPORT_ID: {
		command_code = ts->response_buf[HID_OUTPUT_RESPONSE_CMD_OFFSET]
			& HID_OUTPUT_RESPONSE_CMD_MASK;
		if (command_code != code) {
			printf("HID output response, wrong command_code:%X\n",
				command_code);
			return -EPROTO;
		}
		break;
	}
	}

	return 0;
}

static int cyttsp5_hid_output_bl_launch_app(struct cyttsp5 *ts)
{
	int rc;
	u8 cmd[HID_OUTPUT_BL_LAUNCH_APP_SIZE];
	u16 crc;
	int error;

	put_unaligned_le16(HID_OUTPUT_BL_LAUNCH_APP_SIZE, cmd);
	cmd[2] = HID_BL_OUTPUT_REPORT_ID;
	cmd[3] = 0x0; /* Reserved */
	cmd[4] = HID_OUTPUT_BL_SOP;
	cmd[5] = HID_OUTPUT_BL_LAUNCH_APP;
	put_unaligned_le16(0x00, &cmd[6]);
	crc = crc_itu_t(0xFFFF, &cmd[4], 4);
	put_unaligned_le16(crc, &cmd[8]);
	cmd[10] = HID_OUTPUT_BL_EOP;

	rc = cyttsp5_write(ts, HID_OUTPUT_REG, cmd,
			HID_OUTPUT_BL_LAUNCH_APP_SIZE);
	if (rc) {
		printf("%s Failed to write command %d", __func__, rc);
		return rc;
	}

	// for now, do not use the irq-handler
	mdelay(30);

	error = cyttsp5_read(ts, ts->input_buf, CY_MAX_INPUT);
	if (error)
		printf("ERROR\n");

	size_t size;
	size = get_unaligned_le16(&ts->input_buf[0]);
	printf("launch app got size: %li (expected 0)\n", size);

	// this here is not correct!
	/* if (size == 32){ */
	/* 	printf("Got correct response, copy to response_buf\n"); */
	/* 	memcpy(ts->response_buf, ts->input_buf, size); */
	/* } */

	/* rc = wait_for_completion_interruptible_timeout(&ts->cmd_done, */
	/* 					msecs_to_jiffies(CY_HID_OUTPUT_TIMEOUT_MS)); */
	/* if (rc <= 0) { */
	/* 	dev_err(ts->dev, "HID output cmd execution timed out\n"); */
	/* 	rc = -ETIMEDOUT; */
	/* 	return rc; */
	/* } */

	/* rc = cyttsp5_validate_cmd_response(ts, HID_OUTPUT_BL_LAUNCH_APP); */
	/* if (rc) { */
	/* 	dev_err(ts->dev, "Validation of the response failed\n"); */
	/* 	return rc; */
	/* } */

	return 0;
}

static int cyttsp5_get_hid_descriptor(struct cyttsp5 *ts,
				      struct cyttsp5_hid_desc *desc)
{
	/* struct device *dev = ts->dev; */
	__le16 hid_desc_register = HID_DESC_REG;
	int rc;
	u8 cmd[2];

	/* Set HID descriptor register */
	memcpy(cmd, &hid_desc_register, sizeof(hid_desc_register));

	rc = cyttsp5_write(ts, HID_DESC_REG, NULL, 0);
	if (rc) {
		/* dev_err(dev, "Failed to get HID descriptor, rc=%d\n", rc); */
		printf("Failed to get HID descriptor, rc=%d\n", rc);
		return rc;
	}

	mdelay(200);

	int error;
	memset(ts->input_buf, 0, CY_MAX_INPUT);
	error = cyttsp5_read(ts, ts->input_buf, CY_MAX_INPUT);
	if (error)
		printf("ERROR\n");

	size_t size;
	size = get_unaligned_le16(&ts->input_buf[0]);
	printf("get hid descriptor got size: %li (expected 32)\n", size);
	/* for (int i=0; i < 32; i++) */
	/* 	printf("    ts->input_buf[%i] = %i\n", i, ts->input_buf[i]); */
	memcpy(ts->response_buf, ts->input_buf, size);

	/* rc = wait_for_completion_interruptible_timeout(&ts->cmd_done, */
	/* 					msecs_to_jiffies(CY_HID_GET_HID_DESCRIPTOR_TIMEOUT_MS)); */
	/* if (rc <= 0) { */
	/* 	printf("HID get descriptor timed out\n"); */
	/* 	rc = -ETIMEDOUT; */
	/* 	return rc; */
	/* } */

	memcpy(desc, ts->response_buf, sizeof(*desc));

	/* printf("size check: %i vs %li\n", le16_to_cpu(desc->hid_desc_len), sizeof(*desc)); */
	/* printf("value check: %i vs %i\n", le16_to_cpu(desc->bcd_version) , HID_VERSION); */
	/* Check HID descriptor length and version */
	if (le16_to_cpu(desc->hid_desc_len) != sizeof(*desc) ||
	    le16_to_cpu(desc->bcd_version) != HID_VERSION) {
		printf("Unsupported HID version\n");
		return -ENODEV;
	}

	return 0;
}

static void cyttsp5_si_get_btn_data(struct cyttsp5 *ts)
{
	struct cyttsp5_sysinfo *si = &ts->sysinfo;
	unsigned int btns = ts->response_buf[HID_SYSINFO_BTN_OFFSET]
		& HID_SYSINFO_BTN_MASK;

	si->num_btns = hweight8(btns);
}

static int cyttsp5_get_sysinfo_regs(struct cyttsp5 *ts)
{
	struct cyttsp5_sensing_conf_data *scd = &ts->sysinfo.sensing_conf_data;
	struct cyttsp5_sensing_conf_data_dev *scd_dev =
		(struct cyttsp5_sensing_conf_data_dev *)
		&ts->response_buf[HID_SYSINFO_SENSING_OFFSET];
	/* u32 tmp; */

	cyttsp5_si_get_btn_data(ts);

	scd->max_tch = scd_dev->max_num_of_tch_per_refresh_cycle;

	if (scd->max_tch == 0) {
		printf("Max touch points cannot be zero\n");
		scd->max_tch = 2;
	}

	scd->res_x = dev_read_u32_default(ts->dev, "touchscreen-size-x", get_unaligned_le16(&scd_dev->res_x));
	/* if(dev_read_u32_default(ts->dev, "touchscreen-size-x", &tmp)) */
	/* 	scd->res_x = get_unaligned_le16(&scd_dev->res_x); */
	/* else */
	/* 	scd->res_x = tmp; */

	if (scd->res_x == 0) {
		printf("ABS_X cannot be zero\n");
		return -ENODATA;
	}

	scd->res_y = dev_read_u32_default(ts->dev, "touchscreen-size-y", get_unaligned_le16(&scd_dev->res_y));
	/* if(device_property_read_u32(ts->dev, "touchscreen-size-y", &tmp)) */
	/* 	scd->res_y = get_unaligned_le16(&scd_dev->res_y); */
	/* else */
	/* 	scd->res_y = tmp; */

	if (scd->res_y == 0) {
		printf("ABS_Y cannot be zero\n");
		return -ENODATA;
	}


	scd->max_z = dev_read_u32_default(ts->dev, "touchscreen-max-pressure", get_unaligned_le16(&scd_dev->max_z));
	/* if(device_property_read_u32(ts->dev, "touchscreen-max-pressure", &tmp)) */
	/* 	scd->max_z = get_unaligned_le16(&scd_dev->max_z); */
	/* else */
	/* 	scd->max_z = tmp; */

	if (scd->max_z == 0) {
		printf("ABS_PRESSURE cannot be zero\n");
		return -ENODATA;
	}

	scd->len_x = dev_read_u32_default(ts->dev, "touchscreen-x-mm", get_unaligned_le16(&scd_dev->len_x));
	/* if(device_property_read_u32(ts->dev, "touchscreen-x-mm", &tmp)) */
	/* 	scd->len_x = get_unaligned_le16(&scd_dev->len_x); */
	/* else */
	/* 	scd->len_x = tmp; */

	if (scd->len_x == 0) {
		printf("Touchscreen size x cannot be zero\n");
		scd->len_x = scd->res_x + 1;
	}

	scd->len_y = dev_read_u32_default(ts->dev, "touchscreen-y-mm", get_unaligned_le16(&scd_dev->len_y));
	/* if(device_property_read_u32(ts->dev, "touchscreen-y-mm", &tmp)) */
	/* 	scd->len_y = get_unaligned_le16(&scd_dev->len_y); */
	/* else */
	/* 	scd->len_y = tmp; */

	if (scd->len_y == 0) {
		printf("Touchscreen size y cannot be zero\n");
		scd->len_y = scd->res_y + 1;
	}

	return 0;
}

static int cyttsp5_hid_output_get_sysinfo(struct cyttsp5 *ts)
{
	int rc;
	u8 cmd[HID_OUTPUT_GET_SYSINFO_SIZE];

	/* HI bytes of Output register address */
	put_unaligned_le16(HID_OUTPUT_GET_SYSINFO_SIZE, cmd);
	cmd[2] = HID_APP_OUTPUT_REPORT_ID;
	cmd[3] = 0x0; /* Reserved */
	cmd[4] = HID_OUTPUT_GET_SYSINFO;

	rc = cyttsp5_write(ts, HID_OUTPUT_REG, cmd,
			   HID_OUTPUT_GET_SYSINFO_SIZE);
	if (rc) {
		printf("Failed to write command %d", rc);
		return rc;
	}
	mdelay(800);

	int error;
	memset(ts->input_buf, 0, CY_MAX_INPUT);
	error = cyttsp5_read(ts, ts->input_buf, CY_MAX_INPUT);
	if (error)
		printf("ERROR\n");

	size_t size;
	size = get_unaligned_le16(&ts->input_buf[0]);
	printf("get sysinfo got size: %li (expected 51)\n", size);

	memcpy(ts->response_buf, ts->input_buf, size);
	/* rc = wait_for_completion_interruptible_timeout(&ts->cmd_done, */
	/* 					msecs_to_jiffies(CY_HID_OUTPUT_GET_SYSINFO_TIMEOUT_MS)); */
	/* if (rc <= 0) { */
	/* 	dev_err(ts->dev, "HID output cmd execution timed out\n"); */
	/* 	rc = -ETIMEDOUT; */
	/* 	return rc; */
	/* } */

	rc = cyttsp5_validate_cmd_response(ts, HID_OUTPUT_GET_SYSINFO);
	if (rc) {
		printf("Validation of the response failed\n");
		return rc;
	}

	return cyttsp5_get_sysinfo_regs(ts);
}


/* **********************************************************************/

int cyttsp5_int_deassert(struct cyttsp5 *priv_data)
{
	int ret;
	/* u8 reg; */
	/* reg = HID_INPUT_REG; */
	u8 data[2];
	struct dm_i2c_chip *chip = dev_get_parent_platdata(priv_data->dev);
	/* struct i2c_msg msg[] = { */
	/* 	{ */
	/* 		.addr = chip->chip_addr, */
	/* 		.flags = 0, */
	/* 		.buf = (u8 *)&reg, */
	/* 		.len = 1, */
	/* 	}, { */
	/* 		.addr = chip->chip_addr, */
	/* 		.flags = I2C_M_RD, */
	/* 		.buf = data, */
	/* 		.len = 2, */
	/* 	} */
	/* }; */

	/* ret = dm_i2c_xfer(priv_data->dev, msg, 2); */
	struct i2c_msg msg[] = {
		{
			.addr = chip->chip_addr,
			.flags = I2C_M_RD,
			.buf = data,
			.len = 2,
		}
	};

	ret = dm_i2c_xfer(priv_data->dev, msg, 1);
	if (ret) {
		printf("cyttsp5 i2c read failed: %d\n", ret);
		return ret;
	}
	printf("Deassert returned: %i %i\n", data[0], data[1]);

	return 0;
}

int cyttsp5_i2c_write_old(struct cyttsp5 *priv_data, u8 reg, u8 *val, size_t size)
{
	int ret;
	struct dm_i2c_chip *chip = dev_get_parent_platdata(priv_data->dev);

	u8 data[2];
	data[0] = 19;
	data[1] = 19;

	u8 cmd[13];
	cmd[0] = reg;
	memcpy(&cmd[1], val, size);

	/* for (int i=0; i <= 20; i++){ */
	/* 	printf("    cmd content[%i] = %i\n", i, cmd[i]); */
	/* } */
	u8 reg2;
	reg2 = HID_INPUT_REG;

	struct i2c_msg msg[] = {
		{
			.addr = chip->chip_addr,
			.flags = 0,
			.buf = cmd,
			.len = 13,
		},
		{
			.addr = chip->chip_addr,
			.flags = 0,
			.buf = &reg2,
			.len = 1,
		},
	   	{
			.addr = chip->chip_addr,
			.flags = I2C_M_RD,
			.buf = data,
			.len = 2,
		}
	};

	/* printf("buf: %i %i\n", msg[0].buf[0], msg[0].buf[1]); */

	ret = dm_i2c_xfer(priv_data->dev, msg, 1);
	if (ret) {
		printf("cyttsp5 i2c write failed: %d\n", ret);
		return ret;
	}
	printf("cyttsp5_i2c_write done:\n");

	return 0;
}

int cyttsp5_i2c_read_old(struct cyttsp5 *priv_data)
{
	int ret;
	u8 data[2];
	data[0] = 27;
	data[1] = 27;
	u8 reg = HID_INPUT_REG;
	struct dm_i2c_chip *chip = dev_get_parent_platdata(priv_data->dev);
	size_t size;

	struct i2c_msg msg[] = {
		{
			.addr = chip->chip_addr,
			.flags = 0,
			.buf = (u8 *)&reg,
			.len = 1,
		},
		{
			.addr = chip->chip_addr,
			.flags = I2C_M_RD,
			.buf = data,
			.len = 2,
		}
	};
	/* struct i2c_msg msg[] = { */
	/* 	{ */
	/* 		.addr = chip->chip_addr, */
	/* 		.flags = 0, */
	/* 		.buf = (u8 *)&reg, */
	/* 		.len = 1, */
	/* 	} */
	/* }; */

	ret = dm_i2c_xfer(priv_data->dev, msg, 2);
	if (ret) {
		printf("cyttsp5 i2c read failed: %d\n", ret);
		return ret;
	}

	size = get_unaligned_le16(data);
	printf("i2c read returns: %i %i -> %lu\n", data[0], data[1], size);
	if (size > 2){
		// retrieve data frame
		u8 frame_data[CY_MAX_INPUT];

		struct i2c_msg msg2[] = {
		 {
				.addr = chip->chip_addr,
				.flags = I2C_M_RD,
				.buf = frame_data,
				.len = size,
			}
		};
		ret = dm_i2c_xfer(priv_data->dev, msg2, 1);
		if (ret) {
			printf("cyttsp5 i2c read 2failed: %d\n", ret);
			return ret;
		}
		if (frame_data[2] == HID_TOUCH_REPORT_ID){
			printf("TOUCH REPORT ID\n");
		} else if (frame_data[2] == HID_BTN_REPORT_ID){
			printf("BUTTON REPORT ID\n");

		} else {
			// command response
			for (int i=0; i < size; i++){
				printf("    data bit %i -> %i\n", i, frame_data[i]);
			}
		}
	}

	return 0;
}

int cyttsp5_i2c_hid_descriptor_old(struct cyttsp5 *priv_data)
{
	/* int ret; */
	/* struct dm_i2c_chip *chip = dev_get_parent_platdata(priv_data->dev); */
	/* struct cyttsp5_hid_desc desc; */
	/* u8 addr[2]; */
	/* addr[0] = HID_DESC_REG & 0xFF; */
	/* addr[1] = (HID_DESC_REG >> 8) & 0xFF; */
	/* struct i2c_msg msg[] = { */
	/* 	{ */
	/* 		.addr = chip->chip_addr, */
	/* 		.flags = 0, */
	/* 		.buf = addr, */
	/* 		.len = 2, */
	/* 	} */
	/* }; */

	/* ret = dm_i2c_xfer(priv_data->dev, HID_DESC_REG, NULL, 0); */
	/* if (ret) { */
	/* 	printf("cyttsp5 i2c hid desc failed: %d\n", ret); */
	/* 	return ret; */
	/* } */
	/* printf("RETURN: %i %i\n", data[0], data[1]); */

	return 0;
}

int cyttsp5_hid_output_get_sysinfo_old(struct cyttsp5 *priv_data)
{
	/* int ret; */
	/* struct dm_i2c_chip *chip = dev_get_parent_platdata(priv_data->dev); */
	u8 cmd[7];

	cmd[0] = HID_OUTPUT_REG & 0xFF;
	cmd[1] = (HID_OUTPUT_REG >> 8) && 0xFF;
	// size
	cmd[2] = 5;
	cmd[3] = 0;
	cmd[4] = HID_APP_OUTPUT_REPORT_ID;
	cmd[5] = 0x0;
	cmd[6] = HID_OUTPUT_GET_SYSINFO;


	/* struct i2c_msg msg[] = { */
	/* 	{ */
	/* 		.addr = chip->chip_addr, */
	/* 		.flags = 0, */
	/* 		.buf = cmd, */
	/* 		.len = 7, */
	/* 	} */
	/* }; */

	/* ret = dm_i2c_xfer(priv_data->dev, msg, 1); */
	/* if (ret) { */
	/* 	printf("cyttsp5 i2c hid get systeinfo failed: %d\n", ret); */
	/* 	return ret; */
	/* } */
	/* /1* printf("RETURN: %i %i\n", data[0], data[1]); *1/ */

	/* return 0; */
	return cyttsp5_write(priv_data, HID_OUTPUT_REG, cmd, 5);
}


static void gpio_irq_handler(int irq, void *data)
{
/* 	int ret; */
	struct udevice *dev = data;
	struct cyttsp5 *ts = dev_get_priv(dev);

	int report_id;
	int size;
	int error;
	/* printf("IRQ HANDLER\n"); */

	memset(ts->input_buf, 0, CY_MAX_INPUT);
	error = cyttsp5_read(ts, ts->input_buf, CY_MAX_INPUT);
	if (error)
		printf("IRQ Error reading frame size: %i\n", error);
	/* 	return IRQ_HANDLED; */

	size = get_unaligned_le16(&ts->input_buf[0]);
	if (size == 0) {
		/* reset */
		report_id = 0;
		size = 2;
	} else {
		report_id = ts->input_buf[2];
	}

	switch (report_id) {
	case HID_TOUCH_REPORT_ID:
		cyttsp5_mt_attention(ts->dev);
		break;
	case HID_BTN_REPORT_ID:
		/* cyttsp5_btn_attention(ts->dev); */
		break;
	default:
		/* It is not an input but a command response */
		memcpy(ts->response_buf, ts->input_buf, size);
		ts->triggered = 1;
		/* complete(&ts->cmd_done); */
	}
	// just until we know the irq won't stop u-boot
	mdelay(10);

	/* old */
/* 	irq_handler_disable(ts->irq); */
/* 	/1* u8 buf[2]; *1/ */

	/* printf("IRQ TRIGGERED %s\n", __func__); */
	/* cyttsp5_i2c_read_old(ts); */
	/* cyttsp5_mt_attention(ts->dev); */
	/* mdelay(1000); */
/* 	/1* DETERMINE SIZE OF FRAME *1/ */
/* 	/1* int size; *1/ */
/* 	/1* ret = dm_i2c_read(ts->dev, HID_INPUT_REG, buf, 2); *1/ */
/* 	/1* size = get_unaligned_le16(buf); *1/ */
/* 	/1* printf("FRAME SIZE: %i %i -> %u\n", buf[0], buf[1], size); *1/ */

/* 	/1* mdelay(20); *1/ */
/* 	/1* cyttsp5_i2c_read(ts, HID_INPUT_REG, ts->input_buf); *1/ */

/* 	ret = dm_i2c_read(ts->dev, HID_INPUT_REG, ts->input_buf, CY_MAX_INPUT); */
/* 	if (ret) { */
/* 	    printf("%s: i2c read failed, ret=%d\n", __func__, ret); */
/* 	} */
/* 	int i; */
/* 	for (i=0; i<= 10; i++){ */
/* 		printf("input_buf[%i] = %i\n", i, ts->input_buf[i]); */
/* 	} */
	/* printf("IRQ DONE\n"); */
/* 	ts->triggered = 1; */
}

/* static void wait_for_cmd(struct cyttsp5 *ts, ulong duration) */
/* { */
/* 	ulong start; */
/* 	start = get_timer(0); */
/* 	while (1) { */
/* 		if (ts->triggered == 1) */
/* 			break; */
/* 		if (get_timer(start) > duration){ */
/* 			printf("TIMER OUT\n"); */
/* 			break; */
/* 		} */
/* 		mdelay(1); */
/* 	} */
/* 	ts->triggered = 0; */
/* } */

static int cyttsp5_key_getc(struct stdio_dev *dev){
	return input_getc(&button_input);
}

static int cyttsp5_key_tstc(struct stdio_dev *dev){
	return input_tstc(&button_input);
}


static int cyttsp5_keys_read_keys(struct input_config *input){
	struct cyttsp5 *ts = dev_get_priv(input->dev);
	int key_nr;
	struct key_area *area_ptr;
	area_ptr = ts->key_areas;
	int send_count = 0;
	/* loop through keys */
	for (key_nr=1; key_nr <= ts->key_count; key_nr++){
		if (get_timer(area_ptr->last_press) < 500){
			/* printf("Sending KEY[%i] = (%u/%u) (%u/%u) -> %i\n", */
			/* 		key_nr, */
			/* 		area_ptr->xmin, */
			/* 		area_ptr->xmax, */
			/* 		area_ptr->ymin, */
			/* 		area_ptr->ymax, */
			/* 		area_ptr->key_code */
			/* ); */
			input_send_keycodes(&button_input, &area_ptr->key_code, 1);
			send_count++;
		}
		area_ptr++;
	}
	/* int key = KEY_D; */
	/* int key_send = KEY_DOWN; */
	/* if (key_is_pressed(key_read(key))){ */
	/* 	input_send_keycodes(&button_input, &key_send, 1); */
	/* 	return 1; */
	/* } */
	/* printf("cyttsp5_keys_read_keys: */
	// 0 means we got nothing
	return send_count;
}

static int cyttsp5ub_probe(struct udevice *dev)
{
	debug("cyttsp5ub_probe start\n");
	struct cyttsp5 *ts = dev_get_priv(dev);
	ts->dev = dev;
	ts->triggered = 0;
	/* struct cyttsp5_sysinfo *si; */
	int error;
	/* u8 buf[2]; */

	/* int ret; */
	//int i;

	// let's assume that the supply regulator is already active
	if (dev_read_u32_array(dev, "interrupt-gpios",
	               ts->gpio_int, ARRAY_SIZE(ts->gpio_int))) {
	    printf("%s: read 'gpio-int' failed\n", __func__);
	    return -EINVAL;
	}

	int irq;
	irq = phandle_gpio_to_irq(ts->gpio_int[0], ts->gpio_int[1]);
	if (irq < 0) {
		printf("%s: failed to request irq, ret=%d\n", __func__, irq);
		return 0;
	}
	irq_install_handler(irq, gpio_irq_handler, dev);
	irq_set_irq_type(irq, IRQ_TYPE_EDGE_FALLING);

	/* error = gpio_request_by_name(dev, "interrupt-gpios", 0, ts->int_gpio, GPIOD_IS_IN); */
	/* if (error) */
	/* 	printf("%s Failed to request int gpio\n", __func__); */
	/* printf("INT: %i\n", dm_gpio_get_value(ts->int_gpio)); */

	// reset device
	/* Reset the gpio to be in a reset state */
	// REPLACED: GPIOD_OUT_HIGH with GPIOD_IS_OUT_ACTIVE
	error = gpio_request_by_name(dev, "reset-gpios", 0, ts->reset_gpio, GPIOD_IS_OUT);
	if (error)
		printf("%s Failed to request reset gpio\n", __func__);
	dm_gpio_set_value(ts->reset_gpio, 0);
	mdelay(20);
	dm_gpio_set_value(ts->reset_gpio, 1);
	mdelay(40);
	/* printf("RESET: %i\n", dm_gpio_get_value(ts->reset_gpio)); */
	/* ts->reset_gpio = gpio_request_by_name(dev, "reset", 0, GPIOD_IS_OUT_ACTIVE); */
	/* if (IS_ERR(ts->reset_gpio)) { */
	/* 	error = PTR_ERR(ts->reset_gpio); */
	/* 	dev_err(dev, "Failed to request reset gpio, error %d\n", error); */
	/* 	return error; */
	/* } */
	dm_gpio_set_value(ts->reset_gpio, 0);
	/* printf("RESET 2: %i\n", dm_gpio_get_value(ts->reset_gpio)); */

	/* Need a delay to have device up */
	mdelay(20);
	/* printf("INT after delay: %i\n", dm_gpio_get_value(ts->int_gpio)); */


	// now we need to talk to the TS...
	// ////////////////////////////////////////////////////////////////////////
	cyttsp5_deassert_int(ts);
	/* cyttsp5_int_deassert(ts); */
	/* mdelay(10); */
	// ////////////////////////////////////////////////////////////////////////
	//
	cyttsp5_hid_output_bl_launch_app(ts);

	// launch the app
	/* u8 cmd[12]; */
	/* u16 crc; */
	/* crc = 0; */

	/* /1* cmd[0] = HID_OUTPUT_REG & 0xFF; *1/ */
	/* cmd[0] = (HID_OUTPUT_REG >> 8) & 0xFF; */
	/* /1* put_unaligned_le16(HID_OUTPUT_BL_LAUNCH_APP_SIZE, cmd[2]); *1/ */
	/* cmd[1] = HID_OUTPUT_BL_LAUNCH_APP_SIZE; */
	/* cmd[2] = 0; */
	/* cmd[3] = HID_BL_OUTPUT_REPORT_ID; */
	/* cmd[4] = 0x0; /1* Reserved *1/ */
	/* cmd[5] = HID_OUTPUT_BL_SOP; */
	/* cmd[6] = HID_OUTPUT_BL_LAUNCH_APP; */
	/* put_unaligned_le16(0x00, &cmd[7]); */

	/* crc = crc_itu_t(0xFFFF, &cmd[5], 4); */
	/* put_unaligned_le16(crc, &cmd[9]); */
	/* cmd[11] = HID_OUTPUT_BL_EOP; */

	/* printf("CMD bytes: %i %i %i %i %i %i %i %i %i %i %i %i \n", */
	/* 	(int)cmd[0], (int)cmd[1], cmd[2], cmd[3], */
	/* 	cmd[4], */
	/* 	cmd[5], */
	/* 	cmd[6], */
	/* 	cmd[7], */
	/* 	cmd[8], */
	/* 	cmd[9], */
	/* 	cmd[10], */
	/* 	cmd[11] */
	/* ); */
	/* for (int i=0; i < 40; i++) */
	/* cyttsp5_i2c_write_old(ts, HID_OUTPUT_REG & 0xFF, cmd, 12); */
	/* mdelay(10); */
	/* cyttsp5_read(ts, NULL, CY_MAX_INPUT); */
	/* cyttsp5_i2c_read_old(ts); */
	// ////////////////////////////////////////////////////////////////////////

	/* printf("did write\n"); */
	/* cyttsp5_i2c_read(ts); */

	/* mdelay(100); */
	/* printf("Getting HID descriptor\n"); */
	/* for (int i=0; i <= 8; i++){ */
	printf("Requesting HID descriptor\n");
	error = cyttsp5_get_hid_descriptor(ts, &ts->hid_desc);
	if (error < 0) {
		printf("Error on getting HID descriptor r=%d\n", error);
	}
	/* cyttsp5_i2c_hid_descriptor(ts); */
	printf("Request done\n");
	/* mdelay(100); */
	/* cyttsp5_read(ts, ts->input_buf, CY_MAX_INPUT); */
	/* printf("First hid: %i %i %i\n", */
	/* 		ts->input_buf[0], */
	/* 		ts->input_buf[1], */
	/* 		ts->input_buf[2] */
	/* 	); */
	/* cyttsp5_i2c_read_old(ts); */
	/* } */
	/* /1* wait_for_cmd(ts, 3000); *1/ */

	// ////////////////////////////////////////////////////////////////////////
	error = cyttsp5_fill_all_touch(ts);
	if (error < 0) {
		printf("Error on report descriptor r=%d\n", error);
		return error;
	}
	// ////////////////////////////////////////////////////////////////////////
	printf("get sysinfo start\n");
	/* cyttsp5_hid_output_get_sysinfo_old(ts); */
	mdelay(100);
	/* cyttsp5_i2c_read_old(ts); */
	error = cyttsp5_hid_output_get_sysinfo(ts);
	if (error) {
		printf("Error on getting sysinfo r=%d\n", error);
		return error;
	}
	printf("get sysinfo end\n");
	/* memset(ts->input_buf, 0, CY_MAX_INPUT); */
	/* error = cyttsp5_read(ts, ts->input_buf, CY_MAX_INPUT); */
	/* if (error) */
	/* 	printf("ERROR\n"); */
	/* printf("INT: %i\n", dm_gpio_get_value(ts->int_gpio)); */

	/* size_t size; */
	/* size = get_unaligned_le16(&ts->input_buf[0]); */
	/* printf("get idada got size: %li (expected ?)\n", size); */
	/* for (int i=0; i<10; i++) */
	/* 	cyttsp5_int_deassert(ts); */
	/* printf("INT: %i\n", dm_gpio_get_value(ts->int_gpio)); */

	/* mdelay(200); */
	// ////////////////////////////////////////////////////////////////////////
	printf("Setting up the stdio support\n");

	/* define some keys (3) */
	ts->key_count = 3;
	ts->key_areas = malloc(ts->key_count * sizeof(struct key_area));
	struct key_area *area_ptr;
	area_ptr = ts->key_areas;
	area_ptr->xmin = 1872 - 1550;
	area_ptr->xmax = 1872 - 1200;
	area_ptr->ymin = 50;
	area_ptr->ymax = 400;
	area_ptr->key_code = KEY_UP;
	/* it will take at least a second from here on to initialize the epd
	 * display, so no harm done in intializing the timer here */
	area_ptr->last_press = get_timer(0);

	area_ptr++;
	area_ptr->xmin = 1872 - 1550;
	area_ptr->xmax = 1872 - 1200;
	area_ptr->ymin = 500;
	area_ptr->ymax = 904;
	area_ptr->key_code = KEY_ENTER;
	area_ptr->last_press = get_timer(0);

	area_ptr++;
	area_ptr->xmax = 1872 - 1200;
	area_ptr->xmin = 1872 - 1550;
	area_ptr->ymin = 1004;
	area_ptr->ymax = 1354;
	area_ptr->key_code = KEY_DOWN;
	area_ptr->last_press = get_timer(0);

	/* loop over keys */
	area_ptr = ts->key_areas;
	printf("enumerating keys:\n");
	for (int counter=0; counter < ts->key_count; counter++){
		printf("KEY[%i] = (%u/%u) (%u/%u) -> %i\n",
				counter,
				area_ptr->xmin,
				area_ptr->xmax,
				area_ptr->ymin,
				area_ptr->ymax,
				area_ptr->key_code
		);
		area_ptr++;
	}
	printf("done\n");

	struct stdio_dev dev_stdio = {
		.name	= "touch_keys",
		.flags	= DEV_FLAGS_INPUT,
		.getc	= cyttsp5_key_getc,
		.tstc	= cyttsp5_key_tstc,
		.priv = dev,
	};

	/* * @param repeat_delay_ms   Delay before key auto-repeat starts (in ms) */
	/* * @param repeat_rate_ms    Delay between successive key repeats (in ms) */
	input_set_delays(&button_input, 500, 500);
	/* struct dm_key_uclass_platdata *uc_key; */
	/* uc_key = dev_get_uclass_platdata(dev); */
	/* if (!uc_key){ */
	/* 	printf("cyttsp5 stdio: cannot get dev_get_uclass_platdata\n"); */
	/* 	return -1; */
	/* } */
	/* printf("uc_key: %s\n", uc_key->name); */

	error = input_init(&button_input, 0);

	if (error) {
		debug("%s: CYTTSP5 KEY Cannot set up input\n", __func__);
		return -1;
	}
	button_input.dev = dev;
	input_add_tables(&button_input, false);
	button_input.read_keys = cyttsp5_keys_read_keys;

	error = input_stdio_register(&dev_stdio);
	if (error)
		return error;
	input_set_delays(&button_input, 500, 500);
	printf("Stdio setup complete\n");

	/* printf("INT: %i\n", dm_gpio_get_value(ts->int_gpio)); */
	/* printf("INT: %i\n", dm_gpio_get_value(ts->int_gpio)); */
	/* mdelay(100); */
	/* printf("INT: %i\n", dm_gpio_get_value(ts->int_gpio)); */
	/* mdelay(1000); */
	/* printf("INT: %i\n", dm_gpio_get_value(ts->int_gpio)); */
	/* printf("Enabling irq handler\n"); */
	cyttsp5_deassert_int(ts);
	ts->last_touch_event = get_timer(0);
	irq_handler_enable(irq);
	printf("enabled\n");
	/* mdelay(6000); */
	/* gpio_irq_handler(1, NULL); */

	debug("cyttsp5ub_probe finished\n");
	return 0;
}

static const struct udevice_id cyttsp5ub_of_match[] = {
	{ .compatible = "cypress,tma448" },
	{}
};

U_BOOT_DRIVER(cyttsp5ub) = {
	.name = "cyttsp5ub",
	.id = UCLASS_I2C_GENERIC,
	.of_match = cyttsp5ub_of_match,
	.probe = cyttsp5ub_probe,
	.bind = dm_scan_fdt_dev,
	.priv_auto_alloc_size = sizeof(struct cyttsp5),
};
