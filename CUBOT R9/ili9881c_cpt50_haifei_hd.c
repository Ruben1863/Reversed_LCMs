
/*----------------------------------------------------------------
* Author : Rubén Espínola (ruben1863@github.com)
* Contact : rubenes2003@gmail.com
* Supported device: CUBOT R9
* Copyright 2019 © Rubén Espínola
 *---------------------------------------------------------------*/

#include "lcm_drv.h"

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH                                         (720)
#define FRAME_HEIGHT                                        (1280)



#define REGFLAG_DELAY             							(0XFE)
#define REGFLAG_END_OF_TABLE      							(0xFF)


// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

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
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};

static struct LCM_setting_table lcm_initialization_setting[] =  
{
	{0XFF, 3, {0X98,0X81,0X03}},
	{0X01, 1, {0X00}},
	{0X02, 1, {0X00}},
	{0X03, 1, {0X53}},
	{0X04, 1, {0X14}},
	{0X05, 1, {0X00}},
	{0X06, 1, {0X06}},
	{0X07, 1, {0X01}},
	{0X08, 1, {0X00}},
	{0X09, 1, {0X01}},
	{0X0A, 1, {0X19}},
	{0X0B, 1, {0X01}},
	{0X0C, 1, {0X00}},
	{0X0D, 1, {0X00}},
	{0X0E, 1, {0X00}},
	{0X0F, 1, {0X19}},
	{0X10, 1, {0X19}},
	{0X11, 1, {0X00}},
	{0X12, 1, {0X00}},
	{0X13, 1, {0X00}},
	{0X14, 1, {0X00}},
	{0X15, 1, {0X00}},
	{0X16, 1, {0X00}},
	{0X17, 1, {0X00}},
	{0X18, 1, {0X00}},
	{0X19, 1, {0X00}},
	{0X1A, 1, {0X00}},
	{0X1B, 1, {0X00}},
	{0X1C, 1, {0X00}},
	{0X1D, 1, {0X00}},
	{0X1E, 1, {0X40}},
	{0X1F, 1, {0X40}},
	{0X20, 1, {0X02}},
	{0X21, 1, {0X05}},
	{0X22, 1, {0X02}},
	{0X23, 1, {0X00}},
	{0X24, 1, {0X87}},
	{0X25, 1, {0X87}},
	{0X26, 1, {0X00}},
	{0X27, 1, {0X00}},
	{0X28, 1, {0X3B}},
	{0X29, 1, {0X03}},
	{0X2A, 1, {0X00}},
	{0X2B, 1, {0X00}},
	{0X2C, 1, {0X00}},
	{0X2D, 1, {0X00}},
	{0X2E, 1, {0X00}},
	{0X2F, 1, {0X00}},
	{0X30, 1, {0X00}},
	{0X31, 1, {0X00}},
	{0X32, 1, {0X00}},
	{0X33, 1, {0X00}},
	{0X34, 1, {0X04}},
	{0X35, 1, {0X00}},
	{0X36, 1, {0X00}},
	{0X37, 1, {0X00}},
	{0X38, 1, {0X01}},
	{0X39, 1, {0X01}},
	{0X3A, 1, {0X40}},
	{0X3B, 1, {0X40}},
	{0X3C, 1, {0X00}},
	{0X3D, 1, {0X00}},
	{0X3E, 1, {0X00}},
	{0X3F, 1, {0X00}},
	{0X40, 1, {0X00}},
	{0X41, 1, {0X88}},
	{0X42, 1, {0X00}},
	{0X43, 1, {0X00}},
	{0X44, 1, {0X00}},
	{0X50, 1, {0X01}},
	{0X51, 1, {0X23}},
	{0X52, 1, {0X45}},
	{0X53, 1, {0X67}},
	{0X54, 1, {0X89}},
	{0X55, 1, {0XAB}},
	{0X56, 1, {0X01}},
	{0X57, 1, {0X23}},
	{0X58, 1, {0X45}},
	{0X59, 1, {0X67}},
	{0X5A, 1, {0X89}},
	{0X5B, 1, {0XAB}},
	{0X5C, 1, {0XCD}},
	{0X5D, 1, {0XEF}},
	{0X5E, 1, {0X11}},
	{0X5F, 1, {0X06}},
	{0X60, 1, {0X0C}},
	{0X61, 1, {0X0D}},
	{0X62, 1, {0X0E}},
	{0X63, 1, {0X0F}},
	{0X64, 1, {0X02}},
	{0X65, 1, {0X02}},
	{0X66, 1, {0X02}},
	{0X67, 1, {0X02}},
	{0X68, 1, {0X02}},
	{0X69, 1, {0X02}},
	{0X6A, 1, {0X02}},
	{0X6B, 1, {0X02}},
	{0X6C, 1, {0X02}},
	{0X6D, 1, {0X02}},
	{0X6E, 1, {0X05}},
	{0X6F, 1, {0X05}},
	{0X70, 1, {0X05}},
	{0X71, 1, {0X02}},
	{0X72, 1, {0X01}},
	{0X73, 1, {0X00}},
	{0X74, 1, {0X08}},
	{0X75, 1, {0X08}},
	{0X76, 1, {0X0C}},
	{0X77, 1, {0X0D}},
	{0X78, 1, {0X0E}},
	{0X79, 1, {0X0F}},
	{0X7A, 1, {0X02}},
	{0X7B, 1, {0X02}},
	{0X7C, 1, {0X02}},
	{0X7D, 1, {0X02}},
	{0X7E, 1, {0X02}},
	{0X7F, 1, {0X02}},
	{0X80, 1, {0X02}},
	{0X81, 1, {0X02}},
	{0X82, 1, {0X02}},
	{0X83, 1, {0X02}},
	{0X84, 1, {0X05}},
	{0X85, 1, {0X05}},
	{0X86, 1, {0X05}},
	{0X87, 1, {0X02}},
	{0X88, 1, {0X01}},
	{0X89, 1, {0X00}},
	{0X8A, 1, {0X06}},
	{0XFF, 3, {0X98,0X81,0X04}},
	{0X00, 1, {0X00}},
	{0X6C, 1, {0X15}},
	{0X6E, 1, {0X2B}},
	{0X6F, 1, {0X33}},
	{0X3A, 1, {0XA4}},
	{0X8D, 1, {0X15}},
	{0X87, 1, {0XBA}},
	{0X88, 1, {0X0B}},
	{0X26, 1, {0X76}},
	{0XB2, 1, {0XD1}},
	{0X3A, 1, {0X24}},
	{0X35, 1, {0X1F}},
	{0X33, 1, {0X00}},
	{0X7A, 1, {0X0F}},
	{0XFF, 3, {0X98,0X81,0X01}},
	{0X22, 1, {0X0A}},
	{0X53, 1, {0X8B}},
	{0X55, 1, {0XA5}},
	{0X50, 1, {0XA6}},
	{0X51, 1, {0X9E}},
	{0X31, 1, {0X00}},
	{0X60, 1, {0X14}},
	{0X63, 1, {0X00}},
	{0XA0, 1, {0X0F}},
	{0XA1, 1, {0X17}},
	{0XA2, 1, {0X28}},
	{0XA3, 1, {0X11}},
	{0XA4, 1, {0X16}},
	{0XA5, 1, {0X2B}},
	{0XA6, 1, {0X1E}},
	{0XA7, 1, {0X1F}},
	{0XA8, 1, {0X8B}},
	{0XA9, 1, {0X1A}},
	{0XAA, 1, {0X27}},
	{0XAB, 1, {0X82}},
	{0XAC, 1, {0X1B}},
	{0XAD, 1, {0X1C}},
	{0XAE, 1, {0X4E}},
	{0XAF, 1, {0X22}},
	{0XB0, 1, {0X28}},
	{0XB1, 1, {0X4D}},
	{0XB2, 1, {0X58}},
	{0XB3, 1, {0X3F}},
	{0XC0, 1, {0X01}},
	{0XC1, 1, {0X2D}},
	{0XC2, 1, {0X3C}},
	{0XC3, 1, {0X18}},
	{0XC4, 1, {0X18}},
	{0XC5, 1, {0X27}},
	{0XC6, 1, {0X1C}},
	{0XC7, 1, {0X1D}},
	{0XC8, 1, {0XA6}},
	{0XC9, 1, {0X1C}},
	{0XCA, 1, {0X2A}},
	{0XCB, 1, {0X98}},
	{0XCC, 1, {0X1E}},
	{0XCD, 1, {0X1D}},
	{0XCE, 1, {0X51}},
	{0XCF, 1, {0X25}},
	{0XD0, 1, {0X2C}},
	{0XD1, 1, {0X5F}},
	{0XD2, 1, {0X76}},
	{0XD3, 1, {0X3F}},
	{0XFF, 3, {0X98,0X81,0X00}},
	{0X35, 1, {0X00}},
	{0X11, 1, {0X00}},
	{REGFLAG_DELAY, 120, {}},
	{0X29, 1, {0X00}},
	{REGFLAG_DELAY, 20, {}},
	{REGFLAG_END_OF_TABLE, 0X00, {}}
};	

static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = 
{
	{0xFF, 3, {0x98, 0x81, 0x01}},
	{0x53, 1, {0x10}},
	{0xB3, 1, {0x3F}},
	{0xD3, 1, {0x3F}},
	{0xFF, 3, {0x98, 0x81, 0x04}},
	{0x2D, 1, {0x02}},
	{0x2F, 1, {0x01}},
	{REGFLAG_DELAY, 120, {}},
	{0x2F, 1, {0x00}},
	{0xFF, 3, {0x98, 0x81, 0x00}},
	{0x2F, 1, {0x00}},
	{0x28, 0, {0x00}},
	{REGFLAG_DELAY, 20, {}},
	{0x10, 0, {0x00}},
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


// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS *params)
{
	memset(params, 0, sizeof(LCM_PARAMS));

	params->physical_width = 62;
	params->physical_height = 110;
	params->physical_width_um = 62100;
	params->physical_height_um = 110400;
	params->dsi.LANE_NUM = 3;
	params->dsi.word_count = 2160;
	params->dsi.vertical_sync_active = 8;
	params->dsi.vertical_backporch = 16;
	params->dsi.PLL_CLOCK = 280;
	params->type = 2;
	params->dsi.data_format.format = 2;
	params->dsi.PS = 2;
	params->width = 720;
	params->dsi.horizontal_active_pixel = 720;
	params->height = 1280;
	params->dsi.vertical_active_line = 1280;
	params->dbi.te_mode = 1;
	params->dsi.mode = 1;
	params->dsi.clk_lp_per_line_enable = 1;
	params->dbi.te_edge_polarity = 0;
  params->dsi.data_format.color_order = 0;
	params->dsi.data_format.trans_seq = 0;
	params->dsi.data_format.padding = 0;
	params->dsi.intermediat_buffer_num = 0;
	params->dsi.vertical_frontporch = 20;
	params->dsi.horizontal_sync_active = 20;
	params->dsi.HS_TRAIL = 20;
	params->dsi.horizontal_backporch = 60;
	params->dsi.horizontal_frontporch = 60;
	params->dsi.noncont_clock = 0;
}

static void lcm_suspend(void)
{
	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
	SET_RESET_PIN(0);
	MDELAY(20);
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

static void lcm_resume(void)
{
    lcm_init();
}

static void lcm_init_power(void)
{
}

static void lcm_suspend_power(void)
{
}

static void lcm_resume_power(void)
{
}

static unsigned int lcm_compare_id(void)
{
	return 1;
}

// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------
LCM_DRIVER ili9881c_cpt50_haifei_hd_lcm_drv =
{
    .name           = "ili9881c_cpt50_haifei_hd",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,   
    .compare_id     = lcm_compare_id,
    .init_power     = lcm_init_power,   
    .suspend_power  = lcm_suspend_power,
    .resume_power   = lcm_resume_power,
};
