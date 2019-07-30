
/*----------------------------------------------------------------
* Author : Rubén Espínola (ruben1863@github.com)
* Contact : rubenes2003@gmail.com
* Supported device: Itel A32F
* Reversed for Landy Mkonjo
* Copyright 2019 © Rubén Espínola
 *---------------------------------------------------------------*/
 
 
#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#endif
#include "lcm_drv.h"
#ifdef BUILD_LK
	#include <platform/mt_gpio.h>
	#include <string.h>
#elif defined(BUILD_UBOOT)
	#include <asm/arch/mt_gpio.h>
#else
        #include <mt-plat/mt_gpio.h>
#endif

/* Local Constants */
#define FRAME_WIDTH  (480)
#define FRAME_HEIGHT (854)

/* Local Variables */
#define SET_RESET_PIN(v) (lcm_util.set_reset_pin((v)))
#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))

/**
 * REGFLAG_DELAY, used to trigger MDELAY,
 * REGFLAG_END_OF_TABLE, used to mark the end of LCM_setting_table.
 * their values dosen't matter until they,
 * match with any LCM_setting_table->cmd.
 */

#define REGFLAG_DELAY 			0xFE
#define REGFLAG_END_OF_TABLE 	0X00

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
	{0xBF, 3, {0x91,0x61,0xF2}},
	{0xB3, 2, {0x00,0x87}},
	{0xB4, 2, {0x00,0x87}},
	{0xB8, 6, {0x00,0x9F,0x01,0x00,0x9F,0x01}},
	{0xBA, 3, {0x3E,0x23,0x04}},
	{0xC3, 1, {0x02}},
	{0xC4, 2, {0x30,0x6A}},
	{0xC7, 9, {0x00,0x01,0x31,0x0A,0x6A,0x2A,0x13,0xA5,0xA5}},
	{0xC8, 38, {0x7F,0x73,0x5D,0x48,0x39,0x26,0x28,0x11,0x2D,0x2F,0x32,0x55,0x49,0x5A,0x54,0x5B,0x56,0x50,0x48,0x7F,0x73,0x5D,0x48,0x39,0x26,0x28,0x11,0x2D,0x2F,0x32,0x55,0x49,0x5A,0x54,0x5B,0x56,0x50,0x48}},
	{0xD4, 16, {0x1F,0x1F,0x1F,0x03,0x01,0x05,0x07,0x09,0x0B,0x11,0x13,0x1F,0x1F,0x1F,0x1F,0x1F}},
	{0xD5, 16, {0x1F,0x1F,0x1F,0x02,0x00,0x04,0x06,0x08,0x0A,0x10,0x12,0x1F,0x1F,0x1F,0x1F,0x1F}},
	{0xD6, 16, {0x1F,0x1F,0x1F,0x10,0x12,0x04,0x0A,0x08,0x06,0x02,0x00,0x1F,0x1F,0x1F,0x1F,0x1F}},
	{0xD7, 16, {0x1F,0x1F,0x1F,0x11,0x13,0x05,0x0B,0x09,0x07,0x03,0x01,0x1F,0x1F,0x1F,0x1F,0x1F}},
	{0xD8, 20, {0x20,0x00,0x00,0x30,0x03,0x30,0x01,0x02,0x30,0x01,0x02,0x06,0x70,0x73,0x5D,0x72,0x06,0x38,0x70,0x08}},
	{0xD9, 19, {0x00,0x0A,0x0A,0x88,0x00,0x00,0x06,0x7B,0x00,0x80,0x00,0x3B,0x33,0x1F,0x00,0x00,0x00,0x06,0x70}},
	{0x35, 1, {0x00}},
	{0xBE, 1, {0x01}},
	{0xC1, 1, {0x10}},
	{0xCC, 10, {0x34,0x20,0x38,0x60,0x11,0x91,0x00,0x40,0x00,0x00}},
	{0xBE, 1, {0x02}},
	{0xBE, 1, {0x00}},
	{0x11, 1, {0x00}},
	{REGFLAG_DELAY, 120, {}},
	{0x29, 1, {0x00}},
	{REGFLAG_DELAY, 50, {}},
	{0xBF, 3, {0x09,0xB1,0x7F}},
	{0x34, 120, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}  //Stop cmd is 0x00
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

	params->type = 2;
	params->dbi.te_mode = 0;
	params->dbi.te_edge_polarity = 0;
	params->dsi.LANE_NUM = 2;
	params->dsi.data_format.color_order = 0;
	params->dsi.data_format.trans_seq = 0;
	params->dsi.data_format.padding = 0;
	params->dsi.data_format.format = 2;
	params->dsi.intermediat_buffer_num = 2;
	params->dsi.PS = 2;
	params->width = 480;
	params->dsi.vertical_sync_active = 4;
	params->height = 854;
	params->dsi.vertical_backporch = 6;
	params->dsi.vertical_frontporch = 6; 
	params->dsi.ssc_disable = 1;
	params->dsi.horizontal_sync_active = 10;
	params->dsi.mode = 1;
	params->dsi.horizontal_backporch = 20;
	params->dsi.packet_size = 256;
	params->dsi.horizontal_frontporch = 23;
	params->dsi.vertical_active_line = 854;
	params->dsi.horizontal_active_pixel = 480;
	params->dsi.PLL_CLOCK = 181;
}

static void lcm_init(void)
{

	SET_RESET_PIN(1);
	MDELAY(5);
	SET_RESET_PIN(0);
	MDELAY(20);
	SET_RESET_PIN(1);
	MDELAY(120);

	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);

}

static unsigned int lcm_compare_id(void)
{
	return 1;
}

static void lcm_suspend(void)
{
	SET_RESET_PIN(1);
	MDELAY(5);
	SET_RESET_PIN(0);
	MDELAY(50);
	SET_RESET_PIN(1);
}

static void lcm_resume(void)
{
	lcm_init();
}


/* Get LCM Driver Hooks */
LCM_DRIVER jd9161ba_dsi_vdo_fwvga_jt_lcm_drv =
{
	.name           = "jd9161ba_dsi_vdo_fwvga_jt",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id     = lcm_compare_id,
};
