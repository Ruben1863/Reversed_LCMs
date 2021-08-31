/*----------------------------------------------------------------
* Author : Ruben1863 (ruben1863@github.com)
* Supported device: LEAGOO KIICAA POWER (T591)
* Copyright 2021 © Ruben1863
 *---------------------------------------------------------------*/

#include "lcm_drv.h"
//#include <cust_adc.h>
#include <linux/string.h>
#include <linux/kernel.h>

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

#define AUXADC_LCM_VOLTAGE_CHANNEL                          12
extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);

extern atomic_t ESDCheck_byCPU;

struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};

static struct LCM_setting_table lcm_initialization_setting[] = {

	{0XFE, 1, {0X01}},
	{0X24, 1, {0X00}},
	{0X25, 1, {0X53}},
	{0X26, 1, {0X00}},
	{0X2B, 1, {0XE5}},
	{0X27, 1, {0X0A}},
	{0X29, 1, {0X0A}},
	{0X2B, 1, {0XE5}},
	{0X34, 1, {0X55}},
	{0X2F, 1, {0X54}},
	{0X34, 1, {0X59}},
	{0X1B, 1, {0X50}},
	{0X12, 1, {0X02}},
	{0X1A, 1, {0X06}},
	{0X46, 1, {0X90}},
	{0X52, 1, {0X68}},
	{0X53, 1, {0X00}},
	{0X54, 1, {0X68}},
	{0X55, 1, {0X00}},
	{0X5F, 1, {0X12}},
	{0XFE, 1, {0X03}},
	{0X00, 1, {0X05}},
	{0X01, 1, {0X16}},
	{0X02, 1, {0X0B}},
	{0X03, 1, {0X0F}},
	{0X04, 1, {0X7D}},
	{0X05, 1, {0X00}},
	{0X06, 1, {0X50}},
	{0X07, 1, {0X05}},
	{0X08, 1, {0X16}},
	{0X09, 1, {0X0D}},
	{0X0A, 1, {0X11}},
	{0X0B, 1, {0X7D}},
	{0X0C, 1, {0X00}},
	{0X0D, 1, {0X50}},
	{0X0E, 1, {0X07}},
	{0X0F, 1, {0X08}},
	{0X10, 1, {0X01}},
	{0X11, 1, {0X02}},
	{0X12, 1, {0X00}},
	{0X13, 1, {0X7D}},
	{0X14, 1, {0X00}},
	{0X15, 1, {0X85}},
	{0X16, 1, {0X08}},
	{0X17, 1, {0X03}},
	{0X18, 1, {0X04}},
	{0X19, 1, {0X05}},
	{0X1A, 1, {0X06}},
	{0X1B, 1, {0X00}},
	{0X1C, 1, {0X7D}},
	{0X1D, 1, {0X00}},
	{0X1E, 1, {0X85}},
	{0X1F, 1, {0X08}},
	{0X20, 1, {0X00}},
	{0X21, 1, {0X00}},
	{0X22, 1, {0X00}},
	{0X23, 1, {0X00}},
	{0X24, 1, {0X00}},
	{0X25, 1, {0X00}},
	{0X26, 1, {0X00}},
	{0X27, 1, {0X00}},
	{0X28, 1, {0X00}},
	{0X29, 1, {0X00}},
	{0X2A, 1, {0X07}},
	{0X2B, 1, {0X08}},
	{0X2D, 1, {0X01}},
	{0X2F, 1, {0X02}},
	{0X30, 1, {0X00}},
	{0X31, 1, {0X40}},
	{0X32, 1, {0X05}},
	{0X33, 1, {0X08}},
	{0X34, 1, {0X54}},
	{0X35, 1, {0X7D}},
	{0X36, 1, {0X00}},
	{0X37, 1, {0X03}},
	{0X38, 1, {0X04}},
	{0X39, 1, {0X05}},
	{0X3A, 1, {0X06}},
	{0X3B, 1, {0X00}},
	{0X3D, 1, {0X40}},
	{0X3F, 1, {0X05}},
	{0X40, 1, {0X08}},
	{0X41, 1, {0X54}},
	{0X42, 1, {0X7D}},
	{0X43, 1, {0X00}},
	{0X44, 1, {0X00}},
	{0X45, 1, {0X00}},
	{0X46, 1, {0X00}},
	{0X47, 1, {0X00}},
	{0X48, 1, {0X00}},
	{0X49, 1, {0X00}},
	{0X4A, 1, {0X00}},
	{0X4B, 1, {0X00}},
	{0X4C, 1, {0X00}},
	{0X4D, 1, {0X00}},
	{0X4E, 1, {0X00}},
	{0X4F, 1, {0X00}},
	{0X50, 1, {0X00}},
	{0X51, 1, {0X00}},
	{0X52, 1, {0X00}},
	{0X53, 1, {0X00}},
	{0X54, 1, {0X00}},
	{0X55, 1, {0X00}},
	{0X56, 1, {0X00}},
	{0X58, 1, {0X00}},
	{0X59, 1, {0X00}},
	{0X5A, 1, {0X00}},
	{0X5B, 1, {0X00}},
	{0X5C, 1, {0X00}},
	{0X5D, 1, {0X00}},
	{0X5E, 1, {0X00}},
	{0X5F, 1, {0X00}},
	{0X60, 1, {0X00}},
	{0X61, 1, {0X00}},
	{0X62, 1, {0X00}},
	{0X63, 1, {0X00}},
	{0X64, 1, {0X00}},
	{0X65, 1, {0X00}},
	{0X66, 1, {0X00}},
	{0X67, 1, {0X00}},
	{0X68, 1, {0X00}},
	{0X69, 1, {0X00}},
	{0X6A, 1, {0X00}},
	{0X6B, 1, {0X00}},
	{0X6C, 1, {0X00}},
	{0X6D, 1, {0X00}},
	{0X6E, 1, {0X00}},
	{0X6F, 1, {0X00}},
	{0X70, 1, {0X00}},
	{0X71, 1, {0X00}},
	{0X72, 1, {0X00}},
	{0X73, 1, {0X00}},
	{0X74, 1, {0X08}},
	{0X75, 1, {0X08}},
	{0X76, 1, {0X08}},
	{0X77, 1, {0X08}},
	{0X78, 1, {0X08}},
	{0X79, 1, {0X08}},
	{0X7A, 1, {0X00}},
	{0X7B, 1, {0X00}},
	{0X7C, 1, {0X00}},
	{0X7D, 1, {0X00}},
	{0X7E, 1, {0XBF}},
	{0X7F, 1, {0X3F}},
	{0X80, 1, {0X3F}},
	{0X81, 1, {0X3F}},
	{0X82, 1, {0X3F}},
	{0X83, 1, {0X3F}},
	{0X84, 1, {0X3F}},
	{0X85, 1, {0X02}},
	{0X86, 1, {0X06}},
	{0X87, 1, {0X3F}},
	{0X88, 1, {0X14}},
	{0X89, 1, {0X10}},
	{0X8A, 1, {0X16}},
	{0X8B, 1, {0X12}},
	{0X8C, 1, {0X08}},
	{0X8D, 1, {0X0C}},
	{0X8E, 1, {0X0A}},
	{0X8F, 1, {0X0E}},
	{0X90, 1, {0X00}},
	{0X91, 1, {0X04}},
	{0X92, 1, {0X3F}},
	{0X93, 1, {0X3F}},
	{0X94, 1, {0X3F}},
	{0X95, 1, {0X3F}},
	{0X96, 1, {0X05}},
	{0X97, 1, {0X01}},
	{0X98, 1, {0X0F}},
	{0X99, 1, {0X0B}},
	{0X9A, 1, {0X0D}},
	{0X9B, 1, {0X09}},
	{0X9C, 1, {0X13}},
	{0X9D, 1, {0X17}},
	{0X9E, 1, {0X11}},
	{0X9F, 1, {0X15}},
	{0XA0, 1, {0X3F}},
	{0XA2, 1, {0X07}},
	{0XA3, 1, {0X03}},
	{0XA4, 1, {0X3F}},
	{0XA5, 1, {0X3F}},
	{0XA6, 1, {0X3F}},
	{0XA7, 1, {0X3F}},
	{0XA9, 1, {0X3F}},
	{0XAA, 1, {0X3F}},
	{0XAB, 1, {0X3F}},
	{0XAC, 1, {0X3F}},
	{0XAD, 1, {0X3F}},
	{0XAE, 1, {0X3F}},
	{0XAF, 1, {0X3F}},
	{0XB0, 1, {0X3F}},
	{0XB1, 1, {0X3F}},
	{0XB2, 1, {0X3F}},
	{0XB3, 1, {0X05}},
	{0XB4, 1, {0X01}},
	{0XB5, 1, {0X3F}},
	{0XB6, 1, {0X17}},
	{0XB7, 1, {0X13}},
	{0XB8, 1, {0X15}},
	{0XB9, 1, {0X11}},
	{0XBA, 1, {0X0F}},
	{0XBB, 1, {0X0B}},
	{0XBC, 1, {0X0D}},
	{0XBD, 1, {0X09}},
	{0XBE, 1, {0X07}},
	{0XBF, 1, {0X03}},
	{0XC0, 1, {0X3F}},
	{0XC1, 1, {0X3F}},
	{0XC2, 1, {0X3F}},
	{0XC3, 1, {0X3F}},
	{0XC4, 1, {0X02}},
	{0XC5, 1, {0X06}},
	{0XC6, 1, {0X08}},
	{0XC7, 1, {0X0C}},
	{0XC8, 1, {0X0A}},
	{0XC9, 1, {0X0E}},
	{0XCA, 1, {0X10}},
	{0XCB, 1, {0X14}},
	{0XCC, 1, {0X12}},
	{0XCD, 1, {0X16}},
	{0XCE, 1, {0X3F}},
	{0XCF, 1, {0X00}},
	{0XD0, 1, {0X04}},
	{0XD1, 1, {0X3F}},
	{0XD2, 1, {0X3F}},
	{0XD3, 1, {0X3F}},
	{0XD4, 1, {0X3F}},
	{0XD5, 1, {0X3F}},
	{0XD6, 1, {0X3F}},
	{0XD7, 1, {0X3F}},
	{0XDC, 1, {0X02}},
	{0XDE, 1, {0X12}},
	{0XFE, 1, {0X0E}},
	{0X01, 1, {0X75}},
	{0XFE, 1, {0X04}},
	{0X60, 1, {0X00}},
	{0X61, 1, {0X08}},
	{0X62, 1, {0X0E}},
	{0X63, 1, {0X0D}},
	{0X64, 1, {0X05}},
	{0X65, 1, {0X10}},
	{0X66, 1, {0X0E}},
	{0X67, 1, {0X0A}},
	{0X68, 1, {0X16}},
	{0X69, 1, {0X0C}},
	{0X6A, 1, {0X10}},
	{0X6B, 1, {0X07}},
	{0X6C, 1, {0X0E}},
	{0X6D, 1, {0X13}},
	{0X6E, 1, {0X0C}},
	{0X6F, 1, {0X00}},
	{0X70, 1, {0X00}},
	{0X71, 1, {0X08}},
	{0X72, 1, {0X0E}},
	{0X73, 1, {0X0D}},
	{0X74, 1, {0X05}},
	{0X75, 1, {0X10}},
	{0X76, 1, {0X0E}},
	{0X77, 1, {0X0A}},
	{0X78, 1, {0X16}},
	{0X79, 1, {0X0C}},
	{0X7A, 1, {0X10}},
	{0X7B, 1, {0X07}},
	{0X7C, 1, {0X0E}},
	{0X7D, 1, {0X13}},
	{0X7E, 1, {0X0C}},
	{0X7F, 1, {0X00}},
	{0XFE, 1, {0X00}},
	{0X11, 0, {0X00}},
	{REGFLAG_DELAY, 80, {}},
	{0X29, 0, {0X00}},
	{REGFLAG_DELAY, 10, {}},
	{REGFLAG_END_OF_TABLE, 0X00, {}}	
};	

static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = 
{
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
	
	params->physical_width = 63;
	params->physical_height = 110;
	params->dsi.LANE_NUM = 3;
	params->dsi.packet_size = 256;
	params->dsi.vertical_frontporch = 12;
	params->dsi.horizontal_sync_active = 8;
	params->dsi.PLL_CLOCK = 247;
	params->dsi.lcm_esd_check_table[0].para_list[0] = -100;
	params->type = 2;
	params->dsi.mode = 2;
	params->dsi.data_format.format = 2;
	params->dsi.PS = 2;
	params->dsi.vertical_sync_active = 2;
	params->width = 720;
	params->dsi.horizontal_active_pixel = 720;
	params->height = 1280;
	params->dsi.vertical_active_line = 1280;
	params->dbi.te_mode = 1;
	params->dsi.cont_clock = 1;
	params->dsi.ssc_disable = 1;
	params->dsi.customization_esd_check_enable = 1;
	params->dsi.lcm_esd_check_table[0].count = 1;
	params->dbi.te_edge_polarity = 0;
	params->dsi.data_format.color_order = 0;
	params->dsi.data_format.trans_seq = 0;
	params->dsi.data_format.padding = 0;
	params->dsi.clk_lp_per_line_enable = 0;
	params->dsi.esd_check_enable = 0;
	params->dsi.vertical_backporch = 10;
	params->dsi.lcm_esd_check_table[0].cmd = 10;
	params->dsi.horizontal_backporch = 25;
	params->dsi.horizontal_frontporch = 25;
}

static void lcm_init(void)
{
	SET_RESET_PIN(1);
	MDELAY(1);
	SET_RESET_PIN(0);
	MDELAY(1);
	SET_RESET_PIN(1);
	MDELAY(60);

    push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_suspend(void)
{
	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
	
	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(120);
}

static void lcm_resume(void)
{
    lcm_init();
}

static unsigned int rgk_lcm_compare_id(void) {
	
	int data[4] = {0,0,0,0};
	int rawdata = 0;
	int lcm_vol = 0;
	int unknown = 0;
	
	
	unsigned int data_array[16];
	int res = 0;
	unsigned char buffer[5];
	unsigned char id_high = 0;
	unsigned char id_low = 0;

	res = IMM_GetOneChannelValue(AUXADC_LCM_VOLTAGE_CHANNEL,data,&rawdata);

	if (res < 0 || (lcm_vol = 10 * unknown + 1000 * data, printk([adc_kernel]: lcm_vol= 0x%x, lcm_vol), lcm_vol > 0x64))
    	{
		return 0;   
    	}
		else
	{
		SET_RESET_PIN(1);
		MDELAY(10);
		SET_RESET_PIN(0);
		MDELAY(10);
		SET_RESET_PIN(1);
		MDELAY(200);
		
		data_data_array[0] = 0x1FE1500;
		dsi_set_cmdq(data_array, 1, 1);
		
		data_array[1] = 0x13700;
		dsi_set_cmdq(data_array, 1, 1);
		read_reg_v2(0xDE, buffer, 1);
		
		data_array[2] = 0x13700;
		id_high = buffer[2];
		dsi_set_cmdq(data_array, 1, 1);
		read_reg_v2(0xDF, buffer, 1);
		
		id_low = buffer[3] | (id_high << 8);
		
		//printk("%s, kernel rm68200 debug:id = 0x%08x",lcm_compare_id)
		
		return id_low == 26656;
	}
}

static unsigned int lcm_ata_check(unsigned char *buffer)
{
	unsigned int data_array[16];
	unsigned char buffer[5];
	unsigned char buffer_2[5];
	unsigned char id = 0;
	
	data_array[0] = 0x1FE1500;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[1] = 0x13700;
	dsi_set_cmdq(data_array, 1, 1);
	
//	vC11A42F8 = 1;
	atomic_set(&ESDCheck_byCPU,1); 

	
	read_reg_v2(0xDE, buffer, 1);
	
//	vC11A42F8 = 0;
	atomic_set(&ESDCheck_byCPU,0);

	data_array[2] = 0x13700;
	dsi_set_cmdq(data_array, 1, 1);
	
//	vC11A42F8 = 1;
	atomic_set(&ESDCheck_byCPU,1);

	read_reg_v2(0xDF, buffer_2, 1);
	id = buffer | (buffer_2 << 8);
	data_array[3] = 0xFE1500;
	
//	vC11A42F8 = 0;
	atomic_set(&ESDCheck_byCPU,0);

	dsi_set_cmdq(data_array, 1, 1);
	
//	printk("%s, kernel debug:id = 0x%08x", __func__, id);

	return id == 26656;
}

// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------

LCM_DRIVER rm68200_auo_hd720_5p0_ry_t591_otd_lcm_drv =
{
    .name           = "rm68200_auo_hd720_5p0_ry_t591_otd",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,   
    .compare_id     = rgk_lcm_compare_id,
    .ata_check      = lcm_ata_check,
};
