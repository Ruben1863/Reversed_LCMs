/*----------------------------------------------------------------
* Warning! Do not remove this from the lcm
* Author : Rub3n1863 (ruben1863@github.com)
* Contact : rubenes2003@gmail.com
* Supported device: Intex Cloud Q11
* Copyright 2021 © Rub3n1863
 *---------------------------------------------------------------*/

#include <string.h>
#include <linux/gpio.h>
#include <linux/pinctrl/consumer.h>
#include "lcm_drv.h"
#include <mt-plat/mt_gpio.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_HEIGHT                                    (1280)
#define FRAME_WIDTH                                     (720)

#define REGFLAG_DELAY             			(0XFC)
#define REGFLAG_END_OF_TABLE      			(0xFD)

#define AUTHOR						"RUBEN1863"

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = { 0 };

#define SET_RESET_PIN(v) (lcm_util.set_reset_pin((v)))
#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V3(para_tbl,size,force_update)         lcm_util.dsi_set_cmdq_V3(para_tbl,size,force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define read_reg_v2(cmd, buffer, buffer_size)	            lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)
#define write_regs(addr, pdata, byte_nums)	                lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)   lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define wrtie_cmd(cmd)	lcm_util.dsi_write_cmd(cmd)


struct LCM_setting_table {
	unsigned char cmd;
	unsigned char count;
	unsigned char para_list[64];
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


// ---------------------------------------------------------------------------
//  Local Structs
// ---------------------------------------------------------------------------

static struct pinctrl *lcmctrl;
static struct pinctrl_state *lcd_rst_high;
static struct pinctrl_state *lcd_rst_low;


static void lcm_set_rst_output(int val)
{
	if (val == 0) {
		pinctrl_select_state(lcmctrl, lcd_rst_low);
	} else {
		pinctrl_select_state(lcmctrl, lcd_rst_high);
	}

}

/* Disable this for now
static void lcm_set_rst_output(int val)
{
	if (strstr(AUTHOR, "RUBEN1863") == 0){
		if (val == 0) {
			pinctrl_select_state(lcmctrl, lcd_rst_low);
		} else {
			pinctrl_select_state(lcmctrl, lcd_rst_high);
		}
	}
	else
	{
		printk("You removed authorship! >:( \n");
	}
}
*/

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static struct LCM_setting_table lcm_initialization_setting[] = {

	{0xB0, 1, {0x04}},
	{0x00, 1, {0x00}},
	{0xB4, 2, {0x08}},
	{0xD6, 1, {0x01}},
	{0xB0, 1, {0x03}},
	{0x00, 1, {0x00}},
	{0x29, 1, {0x00}},
	{0x11, 1, {0x00}},
	{REGFLAG_DELAY, 120, {0x00}},
	{REGFLAG_END_OF_TABLE, 0x00, {}} //Stop cmd is 0xFD
};


static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof( LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
	memset(params, 0, sizeof(LCM_PARAMS));
	
	params->dsi.LANE_NUM = 3;
	params->dsi.packet_size = 256;
	params->dsi.vertical_backporch = 11;
	params->dsi.vertical_frontporch = 13;
	params->dsi.horizontal_sync_active = 10;
	params->dsi.horizontal_backporch = 40;
	params->dsi.horizontal_frontporch = 140;
	params->dsi.lcm_esd_check_table[0].cmd = 14;
	params->type = 2;
	params->dsi.PLL_CLOCK = 286;
	params->dsi.mode = 2;
	params->dsi.data_format.format = 2;
	params->dsi.intermediat_buffer_num = 2;
	params->dsi.PS = 2;
	params->width = 720;
	params->dsi.horizontal_active_pixel = 720;
	params->height = 1280;
	params->dsi.vertical_active_line = 1280;
	params->dbi.te_mode = 0;
	params->dbi.te_edge_polarity = 0;
	params->dsi.data_format.color_order = 0;
	params->dsi.data_format.trans_seq = 0;
	params->dsi.data_format.padding = 0;
	params->dsi.clk_lp_per_line_enable = 0;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0;
	params->dsi.ssc_range = 0;
	params->dsi.vertical_sync_active = 1;
	params->dsi.esd_check_enable = 1;
	params->dsi.customization_esd_check_enable = 1;
	params->dsi.lcm_esd_check_table[0].count = 1;
}


static void lcm_init(void)
{
	lcm_set_rst_output(1);
	MDELAY(20);
	lcm_set_rst_output(0);
	MDELAY(20);
	lcm_set_rst_output(1);
	MDELAY(120);
	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_suspend(void)
{
	MDELAY(20);
	lcm_set_rst_output(0);
	MDELAY(10);
}


static void lcm_resume(void)
{
	lcm_init();
}


static unsigned int lcm_compare_id(void)
{
	return 1;
}


// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------

LCM_DRIVER r69339_sampl_lcm_drv =
{
	.name           = "r69339_sampl",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id     = lcm_compare_id,
};
