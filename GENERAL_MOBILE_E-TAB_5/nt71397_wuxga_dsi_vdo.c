/*----------------------------------------------------------------
 * Copyright Statement:
 *
 * Author: Ruben (https://github.com/ruben1863)
 * Telegram Contact: (t.me/ruben1863)
 * Supported device: General Mobile E-Tab 5
 * Copyright (C) 2023: Ruben1863
 *
 *---------------------------------------------------------------*/

#ifndef BUILD_LK
#include <linux/string.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
	#include <platform/mt_gpio.h>
#elif defined(BUILD_UBOOT)
	#include <asm/arch/mt_gpio.h>
#else
	#include <mach/mt_gpio.h>
#endif

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  (1920)
#define FRAME_HEIGHT (1200)

#define LCM_GPIO_1			(GPIO77|0x80000000)
#define LCM_GPIO_2			(GPIO76|0x80000000)

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V3(para_tbl,size,force_update)        lcm_util.dsi_set_cmdq_V3(para_tbl,size,force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	        lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)   

#define dsi_lcm_set_gpio_out(pin, out)										lcm_util.set_gpio_out(pin, out)
#define dsi_lcm_set_gpio_mode(pin, mode)									lcm_util.set_gpio_mode(pin, mode)
#define dsi_lcm_set_gpio_dir(pin, dir)										lcm_util.set_gpio_dir(pin, dir)
#define dsi_lcm_set_gpio_pull_enable(pin, en)								lcm_util.set_gpio_pull_enable)(pin, en)

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

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
	
	params->type = LCM_TYPE_DSI;
	params->dsi.mode = BURST_VDO_MODE;
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.LANE_NUM = LCM_FOUR_LANE;
	params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;
	params->dsi.intermediat_buffer_num = 0;
	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;
	params->dsi.vertical_sync_active = 2;
	params->width = FRAME_WIDTH;
	params->dsi.vertical_backporch = 18;
	params->height = FRAME_HEIGHT;
	params->dsi.vertical_frontporch = 15;
	params->dsi.packet_size = 256;
	params->dsi.horizontal_backporch = 32;
	params->dsi.word_count = 5760;
	params->dsi.vertical_active_line = 1200;
	params->dsi.horizontal_sync_active = 16;
	params->dsi.horizontal_frontporch = 16;
	params->dsi.horizontal_active_pixel = 1920;
	params->dsi.PLL_CLOCK = 441;
}

static void lcm_init(void)
{
	unsigned int data_array[16];

	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(10);
	
	dsi_lcm_set_gpio_mode(LCM_GPIO_1, 0);
	dsi_lcm_set_gpio_dir(LCM_GPIO_1, 1);
	dsi_lcm_set_gpio_out(LCM_GPIO_1, 1);
	MDELAY(150);
	
	data_array[0] = 0x53200;
	dsi_set_cmdq(data_array, 1, 1);
	
	dsi_lcm_set_gpio_mode(LCM_GPIO_2, 0);
	dsi_lcm_set_gpio_dir(LCM_GPIO_2, 1);
	dsi_lcm_set_gpio_out(LCM_GPIO_2, 1);
}

static void lcm_suspend(void)
{
	MDELAY(90);
	dsi_lcm_set_gpio_out(LCM_GPIO_1, 1);
}

static void lcm_resume(void)
{
	unsigned int data_array[16];

	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(10);
	
	dsi_lcm_set_gpio_mode(LCM_GPIO_1, 0);
	dsi_lcm_set_gpio_dir(LCM_GPIO_1, 1);
	dsi_lcm_set_gpio_out(LCM_GPIO_1, 1);
	MDELAY(150);
	MDELAY(50);
	
	data_array[0] = 0x53200;
	dsi_set_cmdq(data_array, 1, 1);
}

LCM_DRIVER nt71397_wuxga_dsi_vdo_lcm_drv = 
{
	.name			= "nt71397_wuxga_dsi_vdo",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
};
