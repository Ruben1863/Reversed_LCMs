
/*----------------------------------------------------------------
* Author : Rubén Espínola (ruben1863@github.com)
* Contact : "rubenes2003@gmail.com" / "t.me/ruben1863"
* Supported device: Infinix Hot 4 Lite
* Reversed for Marwan M Mahboub
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

#define REGFLAG_DELAY               (0XFC)
#define REGFLAG_END_OF_TABLE        (0x00)

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


// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static struct LCM_setting_table lcm_initialization_setting[] =
    
{
	{0XFF,3,{0X98,0X81,0X03}},
	{0X01,1,{0X08}},
	{0X02,1,{0X00}},
	{0X03,1,{0X73}},
	{0X04,1,{0X73}},
	{0X05,1,{0X14}},
	{0X06,1,{0X06}},
	{0X07,1,{0X02}},
	{0X08,1,{0X05}},
	{0X09,1,{0X14}},
	{0X0A,1,{0X14}},
	{0X0B,1,{0X00}},
	{0X0C,1,{0X14}},
	{0X0D,1,{0X14}},
	{0X0E,1,{0X00}},
	{0X0F,1,{0X0C}},
	{0X10,1,{0X0C}},
	{0X11,1,{0X0C}},
	{0X12,1,{0X0C}},
	{0X13,1,{0X14}},
	{0X14,1,{0X0C}},
	{0X15,1,{0X00}},
	{0X16,1,{0X00}},
	{0X17,1,{0X00}},
	{0X18,1,{0X00}},
	{0X19,1,{0X00}},
	{0X1A,1,{0X00}},
	{0X1B,1,{0X00}},
	{0X1C,1,{0X00}},
	{0X1D,1,{0X00}},
	{0X1E,1,{0XC8}},
	{0X1F,1,{0X80}},
	{0X20,1,{0X02}},
	{0X21,1,{0X00}},
	{0X22,1,{0X02}},
	{0X23,1,{0X00}},
	{0X24,1,{0X00}},
	{0X25,1,{0X00}},
	{0X26,1,{0X00}},
	{0X27,1,{0X00}},
	{0X28,1,{0XFB}},
	{0X29,1,{0X43}},
	{0X2A,1,{0X00}},
	{0X2B,1,{0X00}},
	{0X2C,1,{0X07}},
	{0X2D,1,{0X07}},
	{0X2E,1,{0XFF}},
	{0X2F,1,{0XFF}},
	{0X30,1,{0X11}},
	{0X31,1,{0X00}},
	{0X32,1,{0X00}},
	{0X33,1,{0X00}},
	{0X34,1,{0X84}},
	{0X35,1,{0X80}},
	{0X36,1,{0X07}},
	{0X37,1,{0X00}},
	{0X38,1,{0X00}},
	{0X39,1,{0X00}},
	{0X3A,1,{0X00}},
	{0X3B,1,{0X00}},
	{0X3C,1,{0X00}},
	{0X3D,1,{0X00}},
	{0X3E,1,{0X00}},
	{0X3F,1,{0X00}},
	{0X40,1,{0X00}},
	{0X41,1,{0X88}},
	{0X42,1,{0X00}},
	{0X43,1,{0X80}},
	{0X44,1,{0X08}},
	{0X50,1,{0X01}},
	{0X51,1,{0X23}},
	{0X52,1,{0X45}},
	{0X53,1,{0X67}},
	{0X54,1,{0X89}},
	{0X55,1,{0XAB}},
	{0X56,1,{0X01}},
	{0X57,1,{0X23}},
	{0X58,1,{0X45}},
	{0X59,1,{0X67}},
	{0X5A,1,{0X89}},
	{0X5B,1,{0XAB}},
	{0X5C,1,{0XCD}},
	{0X5D,1,{0XEF}},
	{0X5E,1,{0X10}},
	{0X5F,1,{0X02}},
	{0X60,1,{0X08}},
	{0X61,1,{0X09}},
	{0X62,1,{0X10}},
	{0X63,1,{0X12}},
	{0X64,1,{0X11}},
	{0X65,1,{0X13}},
	{0X66,1,{0X0C}},
	{0X67,1,{0X02}},
	{0X68,1,{0X02}},
	{0X69,1,{0X02}},
	{0X6A,1,{0X02}},
	{0X6B,1,{0X02}},
	{0X6C,1,{0X0E}},
	{0X6D,1,{0X0D}},
	{0X6E,1,{0X0F}},
	{0X6F,1,{0X02}},
	{0X70,1,{0X02}},
	{0X71,1,{0X06}},
	{0X72,1,{0X07}},
	{0X73,1,{0X02}},
	{0X74,1,{0X02}},
	{0X75,1,{0X02}},
	{0X76,1,{0X07}},
	{0X77,1,{0X06}},
	{0X78,1,{0X11}},
	{0X79,1,{0X13}},
	{0X7A,1,{0X10}},
	{0X7B,1,{0X12}},
	{0X7C,1,{0X0F}},
	{0X7D,1,{0X02}},
	{0X7E,1,{0X02}},
	{0X7F,1,{0X02}},
	{0X80,1,{0X02}},
	{0X81,1,{0X02}},
	{0X82,1,{0X0D}},
	{0X83,1,{0X0E}},
	{0X84,1,{0X0C}},
	{0X85,1,{0X02}},
	{0X86,1,{0X02}},
	{0X87,1,{0X09}},
	{0X88,1,{0X08}},
	{0X89,1,{0X02}},
	{0X8A,1,{0X02}},
	{0XFF,3,{0X98,0X81,0X04}},
	{0X80,1,{0X00}},
	{0X6C,1,{0X15}},
	{0X6E,1,{0X2D}},
	{0X6F,1,{0X35}},
	{0X3A,1,{0XA4}},
	{0X8D,1,{0X14}},
	{0X87,1,{0XBA}},
	{0X26,1,{0X76}},
	{0XB2,1,{0XD1}},
	{0XB5,1,{0X06}},
	{0XFF,3,{0X98,0X81,0X01}},
	{0X22,1,{0X0A}},
	{0X31,1,{0X00}},
	{0X53,1,{0X7F}},
	{0X55,1,{0X5F}},
	{0X50,1,{0X95}},
	{0X51,1,{0X90}},
	{0X60,1,{0X14}},
	{0X61,1,{0X00}},
	{0X62,1,{0X19}},
	{0X63,1,{0X10}},
	{0XA0,1,{0X00}},
	{0XA1,1,{0X17}},
	{0XA2,1,{0X24}},
	{0XA3,1,{0X12}},
	{0XA4,1,{0X14}},
	{0XA5,1,{0X25}},
	{0XA6,1,{0X1A}},
	{0XA7,1,{0X1C}},
	{0XA8,1,{0X87}},
	{0XA9,1,{0X1B}},
	{0XAA,1,{0X27}},
	{0XAB,1,{0X82}},
	{0XAC,1,{0X22}},
	{0XAD,1,{0X25}},
	{0XAE,1,{0X50}},
	{0XAF,1,{0X23}},
	{0XB0,1,{0X27}},
	{0XB1,1,{0X56}},
	{0XB2,1,{0X68}},
	{0XB3,1,{0X3F}},
	{0XC0,1,{0X0A}},
	{0XC1,1,{0X17}},
	{0XC2,1,{0X24}},
	{0XC3,1,{0X12}},
	{0XC4,1,{0X14}},
	{0XC5,1,{0X25}},
	{0XC6,1,{0X1A}},
	{0XC7,1,{0X1C}},
	{0XC8,1,{0X87}},
	{0XC9,1,{0X1B}},
	{0XCA,1,{0X27}},
	{0XCB,1,{0X82}},
	{0XCC,1,{0X20}},
	{0XCD,1,{0X25}},
	{0XCE,1,{0X50}},
	{0XCF,1,{0X23}},
	{0XD0,1,{0X27}},
	{0XD1,1,{0X56}},
	{0XD2,1,{0X68}},
	{0XD3,1,{0X3F}},
	{0XFF,3,{0X98,0X81,0X00}},
	{0X11,1,{0X00}},
	{REGFLAG_DELAY, 120, {}},
	{0X29,1,{0X00}},
	{REGFLAG_DELAY, 40, {}},
	{REGFLAG_END_OF_TABLE, 0X00, {}}
};

static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = 
{
	{0x28, 1, {0x00}},
	{REGFLAG_DELAY, 20, {}},
	{0x10, 1, {0x00}},
	{REGFLAG_DELAY, 120, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
    unsigned int i;

    for(i = 0; i < count; i++) {
        
        unsigned cmd;
        cmd = table[i].cmd;
        
        switch (cmd) {
            case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;
                
            case REGFLAG_END_OF_TABLE :
                break;
                
            default:
                dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
        }
    }
    
}

static void lcm_get_params(LCM_PARAMS *params)
{
	params->dsi.LANE_NUM = 3;
	params->dsi.word_count = 2160;
	params->dsi.vertical_sync_active = 8;
	params->dsi.horizontal_sync_active = 20;
	params->dsi.horizontal_backporch = 40;
	params->dsi.horizontal_frontporch = 50;
	params->dsi.PLL_CLOCK = 258;
	params->dsi.ssc_range = 4;
	params->dsi.HS_TRAIL = 15;
	params->dsi.lcm_esd_check_table[0].cmd = 10;
	params->dsi.lcm_esd_check_table[0].para_list[0] = -100;
	params->type = 2;
	params->dsi.data_format.format = 2;
//	vC0EB78F8 = 6;
	params->dsi.PS = 2;
//	params[1].type = 6;
	params->width = 720;
	params->dsi.horizontal_active_pixel = 720;
	params->height = 1280;
	params->dsi.vertical_active_line = 1280;
	params->dbi.te_mode = 0;
	params->dbi.te_edge_polarity = 0;
	params->dsi.data_format.color_order = 0;
	params->dsi.data_format.trans_seq = 0;
	params->dsi.data_format.padding = 0;
	params->dsi.intermediat_buffer_num = 0;
	params->dsi.ssc_disable = 0;
	params->dsi.esd_check_enable = 0;
	params->dsi.customization_esd_check_enable = 0;
	params->dsi.mode = 1;
	params->dsi.lcm_esd_check_table[0].count = 1;
	params->dsi.vertical_backporch = 24;
	params->dsi.vertical_frontporch = 24;
}

static void lcm_init_power(void)
{
	MDELAY(10);
	lcd_enn_bias_setting_(0);
	MDELAY(10);
	lcd_enp_bias_setting_(0);
	MDELAY(5);
	sm5106_i2c_write_byte_(0, 10);
	MDELAY(1);
	sm5106_i2c_write_byte_(1, 10);
	MDELAY(1);
	sm5106_i2c_write_byte_(2, 3);
	MDELAY(1);
	lcd_enp_bias_setting_(1);
	MDELAY(10);
	lcd_enn_bias_setting_(1);
	MDELAY(10);
}

static void lcm_suspend_power(void)
{
	SET_RESET_PIN();
	MDELAY(5);
	lcd_enn_bias_setting_(0);
	MDELAY(10);
	lcd_enn_bias_setting_(0);
	MDELAY(5);
	pmic_set_register_value_(916, 0);
	MDELAY(5);
}

static void lcm_resume_power(void)
{
	pmic_set_register_value_(916, 1);
	MDELAY(5);
	lcd_enp_bias_setting_(1);
	MDELAY(10);
	lcd_enn_bias_setting_(1);
	MDELAY(5);
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
	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);

	SET_RESET_PIN(1);
	MDELAY(5);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(20);
}

static void lcm_resume(void)
{
	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(10);

	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}

static unsigned int lcm_compare_id(void)
{
	return 1;
}


// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------
LCM_DRIVER ili9881c_hd720_dsi_vdo_lcm_drv = 
{
    .name           = "ili9881c_hd720_dsi_vdo",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,   
    .compare_id     = lcm_compare_id,
//  .init_power	    = lcm_init_power,
//  .resume_power   = lcm_resume_power,
//  .suspend_power  = lcm_suspend_power,    
};
