
/*----------------------------------------------------------------
* Author : Rubén Espínola (https://github.com/Ruben1863/)
* Contact : rubenes2003@gmail.com
* Supported device: Unknown X23
* Reversed for Jerrick Godwin
* Copyright 2022 © Rubén Espínola
*---------------------------------------------------------------*/
 
#include "lcm_drv.h"

/* Local Constants */
#define FRAME_WIDTH  (720)
#define FRAME_HEIGHT (1528)

/* Local Variables */
#define SET_RESET_PIN(v) (lcm_util.set_reset_pin((v)))
#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))

#define REGFLAG_DELAY 			0xFE
#define REGFLAG_END_OF_TABLE 	0xFFF

/* Local Functions */
#define dsi_set_cmdq_V3(para_tbl,size,force_update)         lcm_util.dsi_set_cmdq_V3(para_tbl,size,force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define read_reg_v2(cmd, buffer, buffer_size)	            lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)
#define write_regs(addr, pdata, byte_nums)	                lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)   lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define wrtie_cmd(cmd)	lcm_util.dsi_write_cmd(cmd)

/* LCM Driver Implementations */
static LCM_UTIL_FUNCS lcm_util = { 0 };

struct LCM_setting_table {
	unsigned char cmd;
	unsigned char count;
	unsigned char para_list[64];
};

static struct LCM_setting_table lcm_initialization_setting[] =
{
	{0xf0, 2, {0x02,0x5a,0x5a}},
	{0xf1, 2, {0xa5,0xa5}},
	{0xb0, 12, {0x54,0x32,0x23,0x45,0x44,0x44,0x44,0x44,0x82,0x01,0x82,0x01}},
	{0xb1, 8, {0x33,0x84,0x02,0x87,0x80,0x01,0x84,0x01}},
	{0xb2, 1, {0x73}},
	{0xb3, 20, {0x0b,0x09,0x13,0x11,0x0f,0x0d,0x00,0x00,0x00,0x03,0x00,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x05,0x07}},
	{0xb4, 20, {0x0a,0x08,0x12,0x10,0x0e,0x0c,0x00,0x00,0x00,0x03,0x00,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x04,0x06}},
	{0xb6, 2, {0x25,0x25}},
	{0xb7, 17, {0x01,0x41,0x09,0x11,0x0d,0x15,0x19,0x0d,0x21,0x1d,0x00,0x00,0x20,0x00,0x02,0xff,0x3c}},
	{0xb8, 5, {0x23,0x01,0x30,0x34,0x53}},
	{0xb9, 4, {0xa0,0x2c,0xff,0xc4}},
	{0xba, 2, {0x27,0x63}},
	{0xbd, 10, {0x54,0x14,0x60,0x60,0x21,0x1e,0x00,0x14,0x42,0x03}},
	{0xc1, 6, {0x0c,0x1c,0x04,0x0c,0x10,0x04}},
	{0xc2, 2, {0x12,0x7e}},
	{0xc3, 3, {0x22,0x31,0x04}},
	{0xc6, 8, {0x08,0x10,0xff,0x08,0x16,0xff,0x36,0x00}},
	{0xc7, 5, {0x05,0x23,0x6b,0x41,0x00}},
	{0xc8, 38, {0x7c,0x54,0x3d,0x2d,0x26,0x15,0x1a,0x06,0x24,0x27,0x2b,0x4d,0x3e,0x4a,0x3f,0x42,0x3a,0x30,0x20,0x7c,0x54,0x3d,0x2d,0x26,0x15,0x1a,0x07,0x24,0x26,0x2a,0x4d,0x3e,0x4a,0x3f,0x42,0x3a,0x30,0x20}},
	{0xd0, 3, {0x07,0x20,0x20}},
	{0xd2, 4, {0x63,0x2b,0x08,0x88}},
	{0xd4, 6, {0x00,0x01,0x00,0x0e,0x04,0x64}},
	{0xf4, 2, {0x08,0x77}},
	{0xf3, 2, {0x13,0x08}},
	{0xf1, 2, {0x5a,0x5a}},
	{0xf0, 2, {0xa5,0xa5}},
	{0x35, 1, {0x00}},
	{0x11, 1, {0x00}},
	{REGFLAG_DELAY, 200, {0X00}},
	{0x29, 1, {0x00}},
	{REGFLAG_DELAY, 20, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_suspend_setting[] =
{
	{0X28, 1, {0X00}},
	{REGFLAG_DELAY, 50, {0X00}},
	{0X10, 1, {0X00}},
	{REGFLAG_DELAY, 120, {0X00}},
	{REGFLAG_END_OF_TABLE, 0X00, {}}
};

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	int i;
	for(i = 0; i < count; i++)
	{
		switch (table[i].cmd) {
			case REGFLAG_DELAY :
			MDELAY(table[i].count);
			break;
			case REGFLAG_END_OF_TABLE :
			break;
			default:
			dsi_set_cmdq_V2(table[i].cmd, table[i].count, table[i].para_list, force_update);
		}
	}
}

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS *params)
{
	memset(params, 0, sizeof(LCM_PARAMS));

	params->dsi.mode = 3;
	params->dsi.LANE_NUM = 4;
	params->dsi.packet_size = 256;
	params->dsi.PLL_CLOCK = 227;
	params->dsi.lcm_esd_check_table[0].para_list[0] = -100;
	params->type = 2;
	params->dsi.data_format.format = 2;
	params->dsi.intermediat_buffer_num = 2;
	params->dsi.PS = 2;
	params->width = 720;
	params->dsi.horizontal_active_pixel = 720;
	params->height = 1528;
	params->dsi.vertical_active_line = 1528;
	params->dbi.te_mode = 1;
	params->dsi.esd_check_enable = 1;
	params->dsi.lcm_esd_check_table[0].count = 1;
	params->dbi.te_edge_polarity = 0;
	params->dsi.data_format.color_order = 0;
	params->dsi.data_format.trans_seq = 0;
	params->dsi.data_format.padding = 0;
	params->dsi.customization_esd_check_enable = 0;
	params->dsi.vertical_sync_active = 8;
	params->dsi.vertical_backporch = 8;
	params->dsi.horizontal_sync_active = 8;
	params->dsi.vertical_frontporch = 10;
	params->dsi.lcm_esd_check_table[0].cmd = 10;
	params->dsi.horizontal_backporch = 80;
	params->dsi.horizontal_frontporch = 80;
}

static void lcm_init(void)
{
	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(20);
	SET_RESET_PIN(1);
	MDELAY(120);

	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_suspend(void)
{
	push_table(lcm_suspend_setting, sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table), 1);
	
	SET_RESET_PIN(1);
	MDELAY(1);
	SET_RESET_PIN(0);
	MDELAY(1);
	SET_RESET_PIN(1);
	MDELAY(120);
}

static void lcm_resume(void)
{
	lcm_init();
}

static unsigned int lcm_compare_id(void)
{
	return 1;
}

/* Get LCM Driver Hooks */
LCM_DRIVER icn7517_dsi_vdo_hd_drip_lcm =
{
	.name           = "icn7517_dsi_vdo_hd_drip_lcm",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id     = lcm_compare_id,
};
