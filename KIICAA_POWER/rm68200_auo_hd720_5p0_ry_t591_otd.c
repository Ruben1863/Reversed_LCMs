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

	{0xFE, 1, {0x00,0x01,0x01}},
	{0x24, 1, {0x00}},
	{0x25, 1, {0x53}},
	{0x26, 1, {0x00}},
	{0x27, 1, {0x0A}},
	{0x29, 1, {0x0A}},
	{0x2B, 1, {0xE5}},
	{0x16, 1, {0x52}},
	{0x2F, 1, {0x54}},
	{0x34, 1, {0x59}},
	{0x1B, 1, {0x50}},
	{0x12, 1, {0x02}},
	{0x1A, 1, {0x06}},
	{0x46, 1, {0x90}},
	{0x52, 1, {0x68}},
	{0x53, 1, {0x00}},
	{0x54, 1, {0x68}},
	{0x55, 1, {0x00}},
	{0x5F, 1, {0x12}},
	{0xFE, 1, {0x03}},
	{0x00, 1, {0x05}},
	{0x01, 1, {0x16}},
	{0x02, 1, {0x0B}},
	{0x03, 1, {0x0F}},
	{0x04, 1, {0x7D}},
	{0x05, 1, {0x00}},
	{0x06, 1, {0x50}},
	{0x07, 1, {0x05}},
	{0x08, 1, {0x16}},
	{0x09, 1, {0x0D}},
	{0x0A, 1, {0x11}},
	{0x0B, 1, {0x7D}},
	{0x0C, 1, {0x00}},
	{0x0D, 1, {0x50}},
	{0x0E, 1, {0x07}},
	{0x0F, 1, {0x08}},
	{0x10, 1, {0x01}},
	{0x11, 1, {0x02}},
	{0x12, 1, {0x00}},
	{0x13, 1, {0x7D}},
	{0x14, 1, {0x00}},
	{0x15, 1, {0x85}},
	{0x16, 1, {0x08}},
	{0x17, 1, {0x03}},
	{0x18, 1, {0x04}},
	{0x19, 1, {0x05}},
	{0x1A, 1, {0x06}},
	{0x1B, 1, {0x00}},
	{0x1C, 1, {0x7D}},
	{0x1D, 1, {0x00}},
	{0x1E, 1, {0x85}},
	{0x1F, 1, {0x08}},
	{0x20, 1, {0x00}},
	{0x21, 1, {0x00}},
	{0x22, 1, {0x00}},
	{0x23, 1, {0x00}},
	{0x24, 1, {0x00}},
	{0x25, 1, {0x00}},
	{0x26, 1, {0x00}},
	{0x27, 1, {0x00}},
	{0x28, 1, {0x00}},
	{0x29, 1, {0x00}},
	{0x2A, 1, {0x07}},
	{0x2B, 1, {0x08}},
	{0x2D, 1, {0x01}},
	{0x2F, 1, {0x02}},
	{0x30, 1, {0x00}},
	{0x31, 1, {0x40}},
	{0x32, 1, {0x05}},
	{0x33, 1, {0x08}},
	{0x34, 1, {0x54}},
	{0x35, 1, {0x7D}},
	{0x36, 1, {0x00}},
	{0x37, 1, {0x03}},
	{0x38, 1, {0x04}},
	{0x39, 1, {0x05}},
	{0x3A, 1, {0x06}},
	{0x3B, 1, {0x00}},
	{0x3D, 1, {0x40}},
	{0x3F, 1, {0x05}},
	{0x40, 1, {0x08}},
	{0x41, 1, {0x54}},
	{0x42, 1, {0x7D}},
	{0x43, 1, {0x00}},
	{0x44, 1, {0x00}},
	{0x45, 1, {0x00}},
	{0x46, 1, {0x00}},
	{0x47, 1, {0x00}},
	{0x48, 1, {0x00}},
	{0x49, 1, {0x00}},
	{0x4A, 1, {0x00}},
	{0x4B, 1, {0x00}},
	{0x4C, 1, {0x00}},
	{0x4D, 1, {0x00}},
	{0x4E, 1, {0x00}},
	{0x4F, 1, {0x00}},
	{0x50, 1, {0x00}},
	{0x51, 1, {0x00}},
	{0x52, 1, {0x00}},
	{0x53, 1, {0x00}},
	{0x54, 1, {0x00}},
	{0x55, 1, {0x00}},
	{0x56, 1, {0x00}},
	{0x58, 1, {0x00}},
	{0x59, 1, {0x00}},
	{0x5A, 1, {0x00}},
	{0x5B, 1, {0x00}},
	{0x5C, 1, {0x00}},
	{0x5D, 1, {0x00}},
	{0x5E, 1, {0x00}},
	{0x5F, 1, {0x00}},
	{0x60, 1, {0x00}},
	{0x61, 1, {0x00}},
	{0x62, 1, {0x00}},
	{0x63, 1, {0x00}},
	{0x64, 1, {0x00}},
	{0x65, 1, {0x00}},
	{0x66, 1, {0x00}},
	{0x67, 1, {0x00}},
	{0x68, 1, {0x00}},
	{0x69, 1, {0x00}},
	{0x6A, 1, {0x00}},
	{0x6B, 1, {0x00}},
	{0x6C, 1, {0x00}},
	{0x6D, 1, {0x00}},
	{0x6E, 1, {0x00}},
	{0x6F, 1, {0x00}},
	{0x70, 1, {0x00}},
	{0x71, 1, {0x00}},
	{0x72, 1, {0x20}},
	{0x73, 1, {0x00}},
	{0x74, 1, {0x08}},
	{0x75, 1, {0x08}},
	{0x76, 1, {0x08}},
	{0x77, 1, {0x08}},
	{0x78, 1, {0x08}},
	{0x79, 1, {0x08}},
	{0x7A, 1, {0x00}},
	{0x7B, 1, {0x00}},
	{0x7C, 1, {0x00}},
	{0x7D, 1, {0x00}},
	{0x7E, 1, {0xBF}},
	{0x7F, 1, {0x3F}},
	{0x80, 1, {0x3F}},
	{0x81, 1, {0x3F}},
	{0x82, 1, {0x3F}},
	{0x83, 1, {0x3F}},
	{0x84, 1, {0x3F}},
	{0x85, 1, {0x02}},
	{0x86, 1, {0x06}},
	{0x87, 1, {0x3F}},
	{0x88, 1, {0x14}},
	{0x89, 1, {0x10}},
	{0x8A, 1, {0x16}},
	{0x8B, 1, {0x12}},
	{0x8C, 1, {0x08}},
	{0x8D, 1, {0x0C}},
	{0x8E, 1, {0x0A}},
	{0x8F, 1, {0x0E}},
	{0x90, 1, {0x00}},
	{0x91, 1, {0x04}},
	{0x92, 1, {0x3F}},
	{0x93, 1, {0x3F}},
	{0x94, 1, {0x3F}},
	{0x95, 1, {0x3F}},
	{0x96, 1, {0x05}},
	{0x97, 1, {0x01}},
	{0x98, 1, {0x0F}},
	{0x99, 1, {0x0B}},
	{0x9A, 1, {0x0D}},
	{0x9B, 1, {0x09}},
	{0x9C, 1, {0x13}},
	{0x9D, 1, {0x17}},
	{0x9E, 1, {0x11}},
	{0x9F, 1, {0x15}},
	{0xA0, 1, {0x3F}},
	{0xA2, 1, {0x07}},
	{0xA3, 1, {0x03}},
	{0xA4, 1, {0x3F}},
	{0xA5, 1, {0x3F}},
	{0xA6, 1, {0x3F}},
	{0xA7, 1, {0x3F}},
	{0xA9, 1, {0x3F}},
	{0xAA, 1, {0x3F}},
	{0xAB, 1, {0x3F}},
	{0xAC, 1, {0x3F}},
	{0xAD, 1, {0x3F}},
	{0xAE, 1, {0x3F}},
	{0xAF, 1, {0x3F}},
	{0xB0, 1, {0x3F}},
	{0xB1, 1, {0x3F}},
	{0xB2, 1, {0x3F}},
	{0xB3, 1, {0x05}},
	{0xB4, 1, {0x01}},
	{0xB5, 1, {0x3F}},
	{0xB6, 1, {0x17}},
	{0xB7, 1, {0x13}},
	{0xB8, 1, {0x15}},
	{0xB9, 1, {0x11}},
	{0xBA, 1, {0x0F}},
	{0xBB, 1, {0x0B}},
	{0xBC, 1, {0x0D}},
	{0xBD, 1, {0x09}},
	{0xBE, 1, {0x07}},
	{0xBF, 1, {0x03}},
	{0xC0, 1, {0x3F}},
	{0xC1, 1, {0x3F}},
	{0xC2, 1, {0x3F}},
	{0xC3, 1, {0x3F}},
	{0xC4, 1, {0x02}},
	{0xC5, 1, {0x06}},
	{0xC6, 1, {0x08}},
	{0xC7, 1, {0x0C}},
	{0xC8, 1, {0x0A}},
	{0xC9, 1, {0x0E}},
	{0xCA, 1, {0x10}},
	{0xCB, 1, {0x14}},
	{0xCC, 1, {0x12}},
	{0xCD, 1, {0x16}},
	{0xCE, 1, {0x3F}},
	{0xCF, 1, {0x00}},
	{0xD0, 1, {0x04}},
	{0xD1, 1, {0x3F}},
	{0xD2, 1, {0x3F}},
	{0xD3, 1, {0x3F}},
	{0xD4, 1, {0x3F}},
	{0xD5, 1, {0x3F}},
	{0xD6, 1, {0x3F}},
	{0xD7, 1, {0x3F}},
	{0xDC, 1, {0x02}},
	{0xDE, 1, {0x12}},
	{0xFE, 1, {0x0E}},
	{0x01, 1, {0x75}},
	{0xFE, 1, {0x04}},
	{0x60, 1, {0x00}},
	{0x61, 1, {0x08}},
	{0x62, 1, {0x0E}},
	{0x63, 1, {0x0D}},
	{0x64, 1, {0x05}},
	{0x65, 1, {0x10}},
	{0x66, 1, {0x0E}},
	{0x67, 1, {0x0A}},
	{0x68, 1, {0x16}},
	{0x69, 1, {0x0C}},
	{0x6A, 1, {0x10}},
	{0x6B, 1, {0x07}},
	{0x6C, 1, {0x0E}},
	{0x6D, 1, {0x13}},
	{0x6E, 1, {0x0C}},
	{0x6F, 1, {0x00}},
	{0x70, 1, {0x00}},
	{0x71, 1, {0x08}},
	{0x72, 1, {0x0E}},
	{0x73, 1, {0x0D}},
	{0x74, 1, {0x05}},
	{0x75, 1, {0x10}},
	{0x76, 1, {0x0E}},
	{0x77, 1, {0x0A}},
	{0x78, 1, {0x16}},
	{0x79, 1, {0x0C}},
	{0x7A, 1, {0x10}},
	{0x7B, 1, {0x07}},
	{0x7C, 1, {0x0E}},
	{0x7D, 1, {0x13}},
	{0x7E, 1, {0x0C}},
	{0x7F, 1, {0x00}},
	{0xFE, 1, {0x00}},
	{0x11, 0, {0x00}},
	{REGFLAG_DELAY, 80, {}},
	{0x29, 0, {0x00,0x00}},
	{REGFLAG_DELAY, 10, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
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
	
	int result;
	unsigned int data_array[16];
	int res = 0;
	unsigned char buffer[5];
	unsigned char id_high = 0;
	unsigned char id_low = 0;

	res = IMM_GetOneChannelValue(AUXADC_LCM_VOLTAGE_CHANNEL,data,&rawdata);

	if (res < 0) {
	    printk("[adc_kernel]: get data error\n");
	    result = 0; 
    }
	    else
	{
    	lcm_vol = 10 * unknown + 1000 * data[0];
    	printk("[adc_kernel]: lcm_vol= %d\n", lcm_vol);

    	    if (lcm_vol <= 100) {

            		SET_RESET_PIN(1);
		        MDELAY(10);
		        SET_RESET_PIN(0);
		        MDELAY(10);
		        SET_RESET_PIN(1);
		        MDELAY(200);
		
	        	data_array[0] = 0x1FE1500;
	        	dsi_set_cmdq(data_array, 1, 1);
		
	        	data_array[1] = 0x13700;
	        	dsi_set_cmdq(data_array, 1, 1);
	        	read_reg_v2(0xDE, buffer, 1);
	        	
	        	data_array[2] = 0x13700;
	        	id_high = buffer[2];
	        	dsi_set_cmdq(data_array, 1, 1);
	        	read_reg_v2(0xDF, buffer, 1);
		
	        	id_low = buffer[3] | (id_high << 8);
		
	        	//printk("%s, kernel rm68200 debug:id = 0x%08x", "lcm_compare_id", id_low)
		
	        	result = id_low == 26656;
	    	}
		    	else
		{
			result = 0;	
		}
    }
    return result;
}

static unsigned int lcm_ata_check(unsigned char *buffer)
{
	unsigned int data_array[16];
	unsigned char buff[5];
	unsigned char buff_2[5];
	unsigned char id = 0;
	
	data_array[0] = 0x1FE1500;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[1] = 0x13700;
	dsi_set_cmdq(data_array, 1, 1);
	
//	vC11A42F8 = 1;
	atomic_set(&ESDCheck_byCPU,1); 

	read_reg_v2(0xDE, buff, 1);
	
//	vC11A42F8 = 0;
	atomic_set(&ESDCheck_byCPU,0);

	data_array[2] = 0x13700;
	dsi_set_cmdq(data_array, 1, 1);
	
//	vC11A42F8 = 1;
	atomic_set(&ESDCheck_byCPU,1);

	read_reg_v2(0xDF, buff_2, 1);
	id = buff[0] | (buff_2[0] << 8);
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
