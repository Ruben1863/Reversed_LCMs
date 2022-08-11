
/*----------------------------------------------------------------
* Author : Rubén Espínola (https://github.com/Ruben1863)
* Contact : rubenes2003@gmail.com
* Supported device: MOBIWIRE Vyper
* Reversed for Melek Saidini
* Copyright 2022 © Rubén Espínola
*---------------------------------------------------------------*/

#include "lcm_drv.h"

/* Local Constants */
#define FRAME_WIDTH  (480)
#define FRAME_HEIGHT (960)

/* Local Variables */
#define SET_RESET_PIN(v) (lcm_util.set_reset_pin((v)))
#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))

#define REGFLAG_DELAY 			0xFE0F
#define REGFLAG_END_OF_TABLE 	0xFF0F

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

static struct LCM_setting_table lcm_initialization_setting[] = {
	{REGFLAG_DELAY, 120, {}},
	{0X11, 1, {0X00}},
	{REGFLAG_DELAY, 120, {}},
	{0XFF, 5, {0X77,0X01,0X00,0X00,0X10}},
	{0XC0, 2, {0X77,0X00}},
	{0XC1, 2, {0X0E,0X02}},
	{0XC2, 2, {0X07,0X02}},
	{0XCC, 1, {0X38}},
	{0XB0, 16, {0X00,0X04,0X0E,0X0E,0X15,0X09,0X08,0X0A,0X09,0X1D,0X07,0X14,0X0F,0X50,0X12,0XD0}},
	{0XB1, 16, {0X00,0X04,0X0C,0X0C,0X14,0X09,0X06,0X09,0X08,0X20,0X07,0X15,0X12,0X57,0X1B,0XDF}},
	{0XFF, 5, {0X77,0X01,0X00,0X00,0X11}},
	{0XB0, 1, {0X4D}},
	{0XB1, 1, {0X46}},
	{0XB2, 1, {0X07}},
	{0XB3, 1, {0X80}},
	{0XB5, 1, {0X49}},
	{0XB7, 1, {0X8A}},
	{0XB8, 1, {0X21}},
	{0XBB, 1, {0X03}},
	{0XC0, 1, {0X89}},
	{0XC1, 1, {0X78}},
	{0XC2, 1, {0X78}},
	{0XC8, 1, {0XBE}},
	{0XD0, 1, {0X88}},
	{0XE0, 3, {0X00,0X00,0X02}},
	{0XE1, 11, {0X07,0X8C,0X09,0X8C,0X06,0X8C,0X08,0X8C,0X00,0X44,0X44}},
	{0XE2, 13, {0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00}},
	{0XE3, 4, {0X00,0X00,0X33,0X33}},
	{0XE4, 2, {0X44,0X44}},
	{0XE5, 16, {0X0F,0XCA,0X8C,0X8C,0X11,0XCA,0X8C,0X8C,0X0B,0XCA,0X8C,0X8C,0X0D,0XCA,0X8C,0X8C}},
	{0XE6, 4, {0X00,0X00,0X33,0X33}},
	{0XE7, 2, {0X44,0X44}},
	{0XE8, 16, {0X0E,0XCA,0X8C,0X8C,0X10,0XCA,0X8C,0X8C,0X0A,0XCA,0X8C,0X8C,0X0C,0XCA,0X8C,0X8C}},
	{0XEB, 7, {0X02,0X01,0X00,0X00,0X00,0X00,0X00}},
	{0XED, 16, {0XAB,0X89,0X76,0X54,0X01,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X10,0X45,0X67,0X98,0XBA}},
	{0XFF, 5, {0X77,0X01,0X00,0X00,0X10}},
	{0XE5, 1, {0X00}},
	{0XFF, 5, {0X77,0X01,0X00,0X00,0X00}},
	{0X35, 1, {0X00}},
	{0X29, 0, {}},
	{REGFLAG_END_OF_TABLE, 0X00, {}}
};

static struct LCM_setting_table lcm_suspend_setting[] = {
	{0X28, 1, {0X00}},
	{REGFLAG_DELAY, 50, {0X00}},
	{0X10, 1, {0X00}},
	{REGFLAG_DELAY, 150, {0X00}},
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

	params->type = 2;
	params->dsi.LANE_NUM = 2;
	params->dsi.data_format.format = 2;
	params->dsi.PS = 2;
	params->dsi.packet_size = 256;
	params->dsi.vertical_sync_active = 8;
	params->width = 480;
	params->dsi.vertical_backporch = 16;
	params->dsi.vertical_frontporch = 16;
	params->dsi.horizontal_sync_active = 16;
	params->height = 960;
	params->dsi.horizontal_backporch = 22;
	params->dsi.horizontal_frontporch = 22;
	params->dbi.te_mode = 1;
	params->dsi.PLL_CLOCK = 208;
	params->dbi.te_edge_polarity = 0;
	params->dsi.lcm_esd_check_table[0].cmd = 10;
	params->dsi.mode = 1;
	params->dsi.data_format.color_order = 0;
	params->dsi.data_format.trans_seq = 0;
	params->dsi.data_format.padding = 0;
	params->dsi.word_count = 1440;
	params->dsi.intermediat_buffer_num = 0;
	params->dsi.vertical_active_line = 960;
	params->dsi.horizontal_active_pixel = 480;
	params->dsi.ssc_disable = 1;
	params->dsi.clk_lp_per_line_enable = 1;
	params->dsi.noncont_clock = 1;
	params->dsi.noncont_clock_period = 1;
	params->dsi.esd_check_enable = 1;
	params->dsi.customization_esd_check_enable = 1;
	params->dsi.lcm_esd_check_table[0].count = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = -100;
	params->dsi.lcm_esd_check_table[1].count = 1;
	params->dsi.lcm_esd_check_table[1].cmd = -27;
	params->dsi.lcm_esd_check_table[1].para_list[0] = 0;
}

static void lcm_init(void) {
	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(20);
	SET_RESET_PIN(1);
	MDELAY(120);

	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_suspend(void) {
	push_table(lcm_suspend_setting, sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table), 1);

	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(20);
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
LCM_DRIVER st7701_fwgaplus_dsi_vdo_helitai_vp531_lcm_drv =
{
	.name           = "st7701_fwgaplus_dsi_vdo_helitai_vp531",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id     = lcm_compare_id,
};
