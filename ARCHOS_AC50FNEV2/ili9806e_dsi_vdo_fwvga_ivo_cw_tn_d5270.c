/*----------------------------------------------------------------
 * Copyright Statement:
 *
 * Inc: MTK LCM (https://github.com/LCM-MTK)
 * Author : Jose (https://github.com/jmpfbmx) and Ruben (https://github.com/ruben1863)
 * Telegram Contact: (t.me/LCM-MTK_inc)
 * Supported device: Archos AC50FNEV2
 * Copyright (C) 2021 : JMPFBMX && Ruben1863
 *
 *---------------------------------------------------------------*/

#include "lcm_drv.h"

#include <linux/string.h>
#include <linux/kernel.h>

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH                                         (480)
#define FRAME_HEIGHT                                        (854)

#define REGFLAG_DELAY             							(0XFE)
#define REGFLAG_END_OF_TABLE      							(0xFD)

#define LCM_ID												(0x98)

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

struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};

static struct LCM_setting_table lcm_initialization_setting[] = {
	{0xFF,5,{0xFF,0x98,0x06,0x04,0x01}},
	{0x08,1,{0x10}},
	{0x21,1,{0x01}},
	{0x30,1,{0x01}},
	{0x31,1,{0x00}},
	{0x50,1,{0x88}},
	{0x51,1,{0x88}},
	{0x60,1,{0x15}},
	{0x61,1,{0x00}},
	{0x62,1,{0x07}},
	{0x63,1,{0x00}},
	{0x40,1,{0x15}},
	{0x41,1,{0x44}},
	{0x42,1,{0x03}},
	{0x43,1,{0x0A}},    
	{0x44,1,{0x06}},      
	{0x52,1,{0x00}},
	{0x53,1,{0x4A}},
	{0x57,1,{0x50}},
	{0xFF,5,{0xFF,0x98,0x06,0x04,0x01}},
	{0xA0,1,{0x00}},
	{0xA1,1,{0x16}},
	{0xA2,1,{0x26}},
	{0xA3,1,{0x13}},
	{0xA4,1,{0x0C}},
	{0xA5,1,{0x16}},
	{0xA6,1,{0x0A}},
	{0xA7,1,{0x08}},
	{0xA8,1,{0x05}},
	{0xA9,1,{0x08}},
	{0xAA,1,{0x0D}},
	{0xAB,1,{0x07}},
	{0xAC,1,{0x0D}},
	{0xAD,1,{0x23}},
	{0xAE,1,{0x1C}},
	{0xAF,1,{0x00}},
	{0xC0,1,{0x00}},
	{0xC1,1,{0x10}},
	{0xC2,1,{0x23}},
	{0xC3,1,{0x13}},
	{0xC4,1,{0x0C}},
	{0xC5,1,{0x1A}},
	{0xC6,1,{0x09}},
	{0xC7,1,{0x08}},
	{0xC8,1,{0x04}},
	{0xC9,1,{0x08}},
	{0xCA,1,{0x02}},
	{0xCB,1,{0x05}},
	{0xCC,1,{0x0A}},
	{0xCD,1,{0x22}},
	{0xCE,1,{0x1F}},
	{0xCF,1,{0x00}},
	{0xFF,5,{0xFF,0x98,0x06,0x04,0x06}},
	{0x00,1,{0x21}},
	{0x01,1,{0x06}},
	{0x02,1,{0xA0}},    
	{0x03,1,{0x02}},
	{0x04,1,{0x01}},
	{0x05,1,{0x01}},
	{0x06,1,{0x80}},
	{0x07,1,{0x03}},
	{0x08,1,{0x06}},
	{0x09,1,{0x80}},
	{0x0A,1,{0x00}},
	{0x0B,1,{0x00}},
	{0x0C,1,{0x20}},
	{0x0D,1,{0x20}},
	{0x0E,1,{0x09}},
	{0x0F,1,{0x00}},
	{0x10,1,{0xFF}},
	{0x11,1,{0xE0}},
	{0x12,1,{0x00}},
	{0x13,1,{0x00}},
	{0x14,1,{0x00}},
	{0x15,1,{0xC0}},
	{0x16,1,{0x08}},
	{0x17,1,{0x00}},
	{0x18,1,{0x00}},
	{0x19,1,{0x00}},
	{0x1A,1,{0x00}},
	{0x1B,1,{0x00}},
	{0x1C,1,{0x00}},
	{0x1D,1,{0x00}},
	{0x20,1,{0x01}},
	{0x21,1,{0x23}},
	{0x22,1,{0x45}},
	{0x23,1,{0x67}},
	{0x24,1,{0x01}},
	{0x25,1,{0x23}},
	{0x26,1,{0x45}},
	{0x27,1,{0x67}},
	{0x30,1,{0x12}},
	{0x31,1,{0x22}},
	{0x32,1,{0x22}},
	{0x33,1,{0x22}},
	{0x34,1,{0x87}},
	{0x35,1,{0x96}},
	{0x36,1,{0xAA}},
	{0x37,1,{0xDB}},
	{0x38,1,{0xCC}},
	{0x39,1,{0xBD}},
	{0x3A,1,{0x78}},
	{0x3B,1,{0x69}},
	{0x3C,1,{0x22}},
	{0x3D,1,{0x22}},
	{0x3E,1,{0x22}},
	{0x3F,1,{0x22}},
	{0x40,1,{0x22}},
	{0x52,1,{0x10}},
	{0x53,1,{0x10}},
	{0x54,1,{0x13}},
	{0xFF,5,{0xFF,0x98,0x06,0x04,0x07}},
	{0x02,1,{0x77}},
	{0x06,1,{0x13}}, 
	{0xE1,1,{0x79}},                                
	{0x17,1,{0x22}},
	{0xB3,1,{0x10}},
	{0x26,1,{0xB2}},
	{0xFF,5,{0xFF,0x98,0x06,0x04,0x00}},
	{0x11,1,{0x00}},
	{REGFLAG_DELAY, 120, {0}},
	{0x29,1,{0x00}},
	{REGFLAG_DELAY, 20, {0}},
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
	
	params->dsi.lcm_esd_check_table[0].para_list[0] = -100;
	params->dsi.packet_size = 256;
	params->dsi.word_count = 1440;
	params->dsi.vertical_sync_active = 6;
	params->physical_width = 62;
	params->physical_height = 110;
	params->type = LCM_TYPE_DSI;
	params->dsi.PLL_CLOCK = 215;
	params->dsi.LANE_NUM = LCM_TWO_LANE;
	params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;
	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;
	params->width = 480;
	params->dsi.horizontal_active_pixel = 480;
	params->height = 854;
	params->dsi.vertical_active_line = 854;
	params->dbi.te_mode = LCM_DBI_TE_MODE_VSYNC_ONLY;
	params->dsi.mode = SYNC_PULSE_VDO_MODE;
	params->dsi.esd_check_enable = 1;
	params->dsi.lcm_esd_check_table[0].count = 1;
	params->dsi.noncont_clock = 1;
	params->dsi.noncont_clock_period = 1;
	params->dsi.customization_esd_check_enable = 1;
	params->dbi.te_edge_polarity = LCM_POLARITY_RISING;
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
	params->dsi.intermediat_buffer_num = 0;
	params->dsi.lcm_esd_check_table[0].cmd = 10;
	params->dsi.vertical_frontporch = 10;
	params->dsi.vertical_backporch = 8;
	params->dsi.horizontal_sync_active = 8;
	params->dsi.horizontal_backporch = 90;
	params->dsi.horizontal_frontporch = 90;
	params->dsi.ssc_disable = 1;
}

static void lcm_init(void)
{
	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(50);
	SET_RESET_PIN(1);
	MDELAY(100);
  
  push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_suspend(void)
{
  push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
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
	unsigned char id = 0;

	res = IMM_GetOneChannelValue(AUXADC_LCM_VOLTAGE_CHANNEL,data,&rawdata);

	if (res < 0)
    {
		printk("[adc_uboot]: get data error\n");
		result = 0; 
    }
		else
	{
    	lcm_vol = 10 * unknown + 1000 * data[0];
    	printk("[adc_kernel]: lcm_vol= %d\n", lcm_vol);

    	if (lcm_vol <= 200) {

		SET_RESET_PIN(1);
		SET_RESET_PIN(0);
		MDELAY(25);
		SET_RESET_PIN(1);
		MDELAY(50);
		
		data_array[0] = 407810;
		data_array[1] = 110690303;
		data_array[2] = 260;
		dsi_set_cmdq(data_array, 3, 1);
		MDELAY(10);
		
		data_array[0] = 79616;
		dsi_set_cmdq(data_array, 1, 1);
		
		read_reg_v2(0, buffer, 1);
		id = buffer[0];
		return (LCM_ID == id) ? 1 : 0;
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
	return 1;
}

// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------

LCM_DRIVER ili9806e_dsi_vdo_fwvga_ivo_cw_tn_d5270_lcm_drv =
{
    .name           = "ili9806e_dsi_vdo_fwvga_ivo_cw_tn_d5270",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,   
    .compare_id     = rgk_lcm_compare_id,
    .ata_check      = lcm_ata_check,
};
