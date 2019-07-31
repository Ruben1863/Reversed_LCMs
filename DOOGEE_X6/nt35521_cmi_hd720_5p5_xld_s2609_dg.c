
/*----------------------------------------------------------------
* Author : Rubén Espínola (ruben1863@github.com)
* Contact : rubenes2003@gmail.com
* Supported device: DOOGEE X6
* Reversed for Αντίσταση στη Ντπ
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

/* ---------------------------------------------------------------------------
   Local Constants
   --------------------------------------------------------------------------- */

#define FRAME_WIDTH  (720)
#define FRAME_HEIGHT (1280)
#define REGFLAG_DELAY (0xFE)
#define REGFLAG_END_OF_TABLE (0XFF)

/**
 * REGFLAG_DELAY, used to trigger MDELAY,
 * REGFLAG_END_OF_TABLE, used to mark the end of LCM_setting_table.
 * their values dosen't matter until they,
 * match with any LCM_setting_table->cmd.
 */

/* ---------------------------------------------------------------------------
   Local Variables
   --------------------------------------------------------------------------- */

static LCM_UTIL_FUNCS lcm_util = { 0 };

#define SET_RESET_PIN(v) (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))

/* ---------------------------------------------------------------------------
   Local Functions
   --------------------------------------------------------------------------- */

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


static struct LCM_setting_table lcm_initialization_setting[] =
{
	{0xD8, 5, {0x00,0x00,0x00,0x00,0x00}},
	{0xD9, 5, {0x00,0x00,0x00,0x00,0x00}},
	{0xE7, 1, {0x00}},
	{0xF0, 5, {0x55,0xAA,0x52,0x08,0x03}},
	{0xB0, 2, {0x00,0x00}},
	{0xB1, 2, {0x00,0x00}},
	{0xB2, 5, {0x05,0x00,0x00,0x00,0x00}},
	{0xB6, 5, {0x05,0x00,0x00,0x00,0x00}},
	{0xB7, 5, {0x05,0x00,0x00,0x00,0x00}},
	{0xBA, 5, {0x57,0x00,0x00,0x00,0x00}},
	{0xBB, 5, {0x57,0x00,0x00,0x00,0x00}},
	{0xC0, 4, {0x00,0x00,0x00,0x00}},
	{0xC1, 4, {0x00,0x00,0x00,0x00}},
	{0xC4, 1, {0x60}},
	{0xC5, 1, {0x40}},
	{0xF0, 5, {0x55,0xAA,0x52,0x08,0x05}},
	{0xBD, 5, {0x03,0x01,0x03,0x03,0x03}},
	{0xB0, 2, {0x17,0x06}},
	{0xB1, 2, {0x17,0x06}},
	{0xB2, 2, {0x17,0x06}},
	{0xB3, 2, {0x17,0x06}},
	{0xB4, 2, {0x17,0x06}},
	{0xB5, 2, {0x17,0x06}},
	{0xB8, 1, {0x00}},
	{0xB9, 1, {0x00}},
	{0xBA, 1, {0x00}},
	{0xBB, 1, {0x02}},
	{0xBC, 1, {0x00}},
	{0xC0, 1, {0x07}},
	{0xC4, 1, {0x80}},
	{0xC5, 1, {0xA4}},
	{0xC8, 2, {0x05,0x30}},
	{0xC9, 2, {0x01,0x31}},
	{0xCC, 3, {0x00,0x00,0x3C}},
	{0xCD, 3, {0x00,0x00,0x3C}},
	{0xD1, 5, {0x00,0x05,0x09,0x07,0x10}},
	{0xD2, 5, {0x00,0x05,0x0E,0x07,0x10}},
	{0xE5, 1, {0x06}},
	{0xE6, 1, {0x06}},
	{0xE7, 1, {0x06}},
	{0xE8, 1, {0x06}},
	{0xE9, 1, {0x06}},
	{0xEA, 1, {0x06}},
	{0xED, 1, {0x30}},
	{0x35, 1, {0x00}},
	{0x11, 0, {0x00,0x00}},
	{REGFLAG_DELAY, 120, {}},
	{0x29, 0, {0x00,0x00}},
	{REGFLAG_DELAY, 10, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}  //Stop cmd is 0xFF
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

/* ---------------------------------------------------------------------------
   LCM Driver Implementations
   --------------------------------------------------------------------------- */

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS *params)
{
	memset(params, 0, sizeof(LCM_PARAMS));

	params->dsi.packet_size = 512;
	params->dsi.word_count = 2160;
	params->dsi.PLL_CLOCK = 250;
	params->dsi.lcm_esd_check_table[0].cmd = 10;
	params->dsi.lcm_esd_check_table[0].para_list[0] = -100;
	params->type = 2;
	params->dsi.data_format.format = 2;
	params->dsi.PS = 2;
	params->width = 720;
	params->dsi.horizontal_active_pixel = 720;
	params->height = 1280;
	params->dsi.vertical_active_line = 1280;
	params->dbi.te_mode = 1;
	params->dsi.cont_clock = 1;
	params->dsi.ssc_disable = 1;
	params->dsi.lcm_esd_check_table[0].count = 1;
	params->dbi.te_edge_polarity = 0;
	params->dsi.data_format.color_order = 0;
	params->dsi.data_format.trans_seq = 0;
	params->dsi.data_format.padding = 0;
	params->dsi.intermediat_buffer_num = 0;
	params->dsi.clk_lp_per_line_enable = 0;
	params->dsi.esd_check_enable = 0;
	params->dsi.customization_esd_check_enable = 0;
	params->dsi.mode = 3;
	params->dsi.LANE_NUM = 3;
	params->dsi.vertical_sync_active = 4;
	params->dsi.horizontal_sync_active = 4;
	params->dsi.vertical_backporch = 16;
	params->dsi.vertical_frontporch = 16;
	params->dsi.horizontal_backporch = 60;
	params->dsi.horizontal_frontporch = 60;
}
static void lcm_init(void)
{
	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(120);

	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_suspend(void)
{
	unsigned int data_array[16];
	
	data_array[0] = 0x00280500;
	dsi_set_cmdq(&data_array, 1, 1);
	MDELAY(20);
	data_array[0] = 0x00100500;
	dsi_set_cmdq(&data_array, 1, 1);
	MDELAY(120);
	SET_RESET_PIN(1);
	MDELAY(20);
	SET_RESET_PIN(0);
	MDELAY(50);
}
static void lcm_resume(void)
{
	lcm_init();
}

static unsigned int rgk_lcm_compare_id(void)
{
	return 1;
}

static unsigned int lcm_ata_check(unsigned char *buffer)
{
	return 1;
}

/* ---------------------------------------------------------------------------
   Get LCM Driver Hooks
   --------------------------------------------------------------------------- */

LCM_DRIVER nt35521_cmi_hd720_5p5_xld_s2609_dg_lcm_drv =
{
    .name           = "nt35521_cmi_hd720_5p5_xld_s2609_dg",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_init,
    .compare_id     = rgk_lcm_compare_id,
    .ata_check	    = lcm_ata_check,
};
