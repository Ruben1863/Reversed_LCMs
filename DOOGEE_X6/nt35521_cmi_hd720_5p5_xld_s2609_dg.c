
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
	{0xFF, 4, {0x55,0xA5,0x80}},
	{0x6F, 2, {0x11,0x00}},
	{0xF7, 2, {0x20,0x00}},
	{0x6F, 1, {0x06}},
	{0xF7, 1, {0xA0}},
	{0x6F, 1, {0x19}},
	{0xF7, 1, {0x12}},
	{0x6F, 1, {0x02}},
	{0xF7, 1, {0x47}},
	{0x6F, 1, {0x17}},
	{0xF4, 1, {0x70}},
	{0x6F, 1, {0x01}},
	{0xF9, 1, {0x46}},
	{0xF0, 5, {0x55,0xAA,0x52,0x08,0x00}},
	{0xBD, 5, {0x01,0xA0,0x10,0x10,0x01}},
	{0xB8, 4, {0x00,0x00,0x00,0x00}},
	{0xBB, 2, {0x24,0x24}},
	{0xBC, 2, {0x00,0x00}},
	{0xB6, 1, {0x04}},
	{0xC8, 1, {0x80}},
	{0xD9, 2, {0x01,0x01}},
	{0xD4, 1, {0xC7}},
	{0xB1, 2, {0x60,0x21}},
	{0xF0, 5, {0x55,0xAA,0x52,0x08,0x01}},
	{0xB0, 2, {0x09,0x09}},
	{0xB1, 2, {0x09,0x09}},
	{0xBC, 2, {0x90,0x00}},
	{0xBD, 2, {0x90,0x00}},
	{0xCA, 1, {0x00}},
	{0xC0, 1, {0x0C}},
	{0xB5, 2, {0x03,0x03}},
	{0xB3, 2, {0x19,0x19}},
	{0xB4, 2, {0x19,0x19}},
	{0xB9, 2, {0x36,0x36}},
	{0xBA, 2, {0x34,0x34}},
	{0xF0, 5, {0x55,0xAA,0x52,0x08,0x02}},
	{0xB0, 16, {0x00,0x0E,0x00,0x69,0x00,0x96,0x00,0xB5,0x00,0xD0,0x00,0xF4,0x01,0x11,0x01,0x3F}},
	{0xB1, 16, {0x01,0x62,0x01,0x9C,0x01,0xC9,0x02,0x0E,0x02,0x44,0x02,0x48,0x02,0x74,0x02,0xA9}},
	{0xB2, 16, {0x02,0xCB,0x02,0xF6,0x03,0x16,0x03,0x3F,0x03,0x5C,0x03,0x73,0x03,0x9B,0x03,0x9F}},
	{0xB3, 4, {0x03,0xD7,0x03,0xE8}},
	{0xB4, 16, {0x00,0x05,0x00,0x50,0x00,0x8D,0x00,0xAD,0x00,0xC4,0x00,0xEB,0x01,0x09,0x01,0x39}},
	{0xB5, 16, {0x01,0x5E,0x01,0x97,0x01,0xC4,0x02,0x08,0x02,0x3D,0x02,0x3E,0x02,0x70,0x02,0xA4}},
	{0xB6, 16, {0x02,0xC5,0x02,0xF2,0x03,0x11,0x03,0x3B,0x03,0x58,0x03,0x6C,0x03,0x96,0x03,0xCA}},
	{0xB7, 4, {0x03,0xF5,0x03,0xF8}},
	{0xB8, 16, {0x00,0x14,0x00,0x3B,0x00,0x6F,0x00,0x8E,0x00,0xA9,0x00,0xD1,0x00,0xF1,0x01,0x24}},
	{0xB9, 16, {0x01,0x4C,0x01,0x8A,0x01,0xB9,0x02,0x03,0x02,0x3A,0x02,0x3B,0x02,0x6E,0x02,0xA4}},
	{0xBA, 16, {0x02,0xC5,0x02,0xF4,0x03,0x16,0x03,0x4D,0x03,0x81,0x03,0xF9,0x03,0xFA,0x03,0xFB}},
	{0xBB, 4, {0x03,0xFD,0x03,0xFE}},
	{0xBC, 16, {0x00,0x0E,0x00,0x69,0x00,0x96,0x00,0xB5,0x00,0xD0,0x00,0xF4,0x01,0x11,0x01,0x3F}},
	{0xBD, 16, {0x01,0x62,0x01,0x9C,0x01,0xC9,0x02,0x0E,0x02,0x44,0x02,0x48,0x02,0x74,0x02,0xA9}},
	{0xBE, 16, {0x02,0xCB,0x02,0xF6,0x03,0x16,0x03,0x3F,0x03,0x5C,0x03,0x73,0x03,0x9B,0x03,0x9F}},
	{0xBF, 4, {0x03,0xD7,0x03,0xE8}},
	{0xC0, 16, {0x00,0x05,0x00,0x50,0x00,0x8D,0x00,0xAD,0x00,0xC4,0x00,0xEB,0x01,0x09,0x01,0x39}},
	{0xC1, 16, {0x01,0x5E,0x01,0x97,0x01,0xC4,0x02,0x08,0x02,0x3D,0x02,0x3E,0x02,0x70,0x02,0xA4}},
	{0xC2, 16, {0x02,0xC5,0x02,0xF2,0x03,0x11,0x03,0x3B,0x03,0x58,0x03,0x6C,0x03,0x96,0x03,0xCA}},
	{0xC3, 4, {0x03,0xF5,0x03,0xF8}},
	{0xC4, 16, {0x00,0x14,0x00,0x3B,0x00,0x6F,0x00,0x8E,0x00,0xA9,0x00,0xD1,0x00,0xF1,0x01,0x24}},
	{0xC5, 16, {0x01,0x4C,0x01,0x8A,0x01,0xB9,0x02,0x03,0x02,0x3A,0x02,0x3B,0x02,0x6E,0x02,0xA4}},
	{0xC6, 16, {0x02,0xC5,0x02,0xF4,0x03,0x16,0x03,0x4D,0x03,0x81,0x03,0xF9,0x03,0xFA,0x03,0xFB}},
	{0xC7, 4, {0x03,0xFD,0x03,0xFE}},
	{0xF0, 5, {0x55,0xAA,0x52,0x08,0x06}},
	{0xB0, 2, {0x31,0x2E}},
	{0xB1, 2, {0x10,0x12}},
	{0xB2, 2, {0x16,0x18}},
	{0xB3, 2, {0x31,0x31}},
	{0xB4, 2, {0x31,0x34}},
	{0xB5, 2, {0x34,0x34}},
	{0xB6, 2, {0x34,0x34}},
	{0xB7, 2, {0x34,0x34}},
	{0xB8, 2, {0x33,0x2D}},
	{0xB9, 2, {0x00,0x02}},
	{0xBA, 2, {0x03,0x01}},
	{0xBB, 2, {0x2D,0x33}},
	{0xBC, 2, {0x34,0x34}},
	{0xBD, 2, {0x34,0x34}},
	{0xBE, 2, {0x34,0x34}},
	{0xBF, 2, {0x34,0x31}},
	{0xC0, 2, {0x31,0x31}},
	{0xC1, 2, {0x19,0x17}},
	{0xC2, 2, {0x13,0x11}},
	{0xC3, 2, {0x2E,0x31}},
	{0xE5, 2, {0x31,0x31}},
	{0xC4, 2, {0x31,0x2D}},
	{0xC5, 2, {0x19,0x17}},
	{0xC6, 2, {0x13,0x11}},
	{0xC7, 2, {0x31,0x31}},
	{0xC8, 2, {0x31,0x34}},
	{0xC9, 2, {0x34,0x34}},
	{0xCA, 2, {0x34,0x34}},
	{0xCB, 2, {0x34,0x34}},
	{0xCC, 2, {0x33,0x2E}},
	{0xCD, 2, {0x03,0x01}},
	{0xCE, 2, {0x00,0x02}},
	{0xCF, 2, {0x2E,0x33}},
	{0xD0, 2, {0x34,0x34}},
	{0xD1, 2, {0x34,0x34}},
	{0xD2, 2, {0x34,0x34}},
	{0xD3, 2, {0x34,0x31}},
	{0xD4, 2, {0x31,0x31}},
	{0xD5, 2, {0x10,0x12}},
	{0xD6, 2, {0x16,0x18}},
	{0xD7, 2, {0x2D,0x31}},
	{0xE6, 2, {0x31,0x31}},
	{0xD8, 5, {0x00,0x00,0x00,0x00,0x00}},
	{0xD9, 5, {0x00,0x00,0x00,0x00,0x00}},
	{0xE7, 1, {0x00}},
	{0xF0, 5, {0x55,0xAA,0x52,0x08,0x05}},
	{0xED, 1, {0x30}},
	{0xB0, 2, {0x17,0x06}},
	{0xB8, 1, {0x00}},
	{0xC0, 1, {0x0D}},
	{0xC1, 1, {0x0B}},
	{0xC2, 1, {0x00}},
	{0xC3, 1, {0x00}},
	{0xC4, 1, {0x84}},
	{0xC5, 1, {0x82}},
	{0xC6, 1, {0x82}},
	{0xC7, 1, {0x80}},
	{0xC8, 2, {0x0B,0x20}},
	{0xC9, 2, {0x07,0x20}},
	{0xCA, 2, {0x01,0x10}},
	{0xD1, 5, {0x03,0x05,0x05,0x07,0x00}},
	{0xD2, 5, {0x03,0x05,0x09,0x03,0x00}},
	{0xD3, 5, {0x00,0x00,0x6A,0x07,0x10}},
	{0xD4, 5, {0x30,0x00,0x6A,0x07,0x10}},
	{0xF0, 5, {0x55,0xAA,0x52,0x08,0x03}},
	{0xB0, 2, {0x00,0x00}},
	{0xB1, 2, {0x00,0x00}},
	{0xB2, 5, {0x05,0x01,0x13,0x00,0x00}},
	{0xB3, 5, {0x05,0x01,0x13,0x00,0x00}},
	{0xB4, 5, {0x05,0x01,0x13,0x00,0x00}},
	{0xB5, 5, {0x05,0x01,0x13,0x00,0x00}},
	{0xB6, 5, {0x02,0x01,0x13,0x00,0x00}},
	{0xB7, 5, {0x02,0x01,0x13,0x00,0x00}},
	{0xB8, 5, {0x02,0x01,0x13,0x00,0x00}},
	{0xB9, 5, {0x02,0x01,0x13,0x00,0x00}},
	{0xBA, 5, {0x53,0x01,0x13,0x00,0x00}},
	{0xBB, 5, {0x53,0x01,0x13,0x00,0x00}},
	{0xBC, 5, {0x53,0x01,0x13,0x00,0x00}},
	{0xBD, 5, {0x53,0x01,0x13,0x00,0x00}},
	{0xC4, 1, {0x60}},
	{0xC5, 1, {0x40}},
	{0xC6, 1, {0x64}},
	{0xC7, 1, {0x44}},
	{0x6F, 1, {0x11}},
	{0xF3, 1, {0x01}},
	{0x35, 1, {0x00}},
	{0x11, 0, {0x00,0x00}},
    {REGFLAG_DELAY, 120, {0x00}},
	{0x29, 0, {0x00,0x00}},
    {REGFLAG_DELAY, 10, {0x00}},
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
