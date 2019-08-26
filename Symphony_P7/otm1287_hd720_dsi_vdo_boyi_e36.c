
/*----------------------------------------------------------------
* Author : Rubén Espínola (ruben1863@github.com)
* Contact : rubenes2003@gmail.com
* Supported device: SYMPHONY P7
* Reversed for Al Helal Isram
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


// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
	params->dsi.LANE_NUM = 3;
	params->dsi.packet_size = 256;
	params->dsi.vertical_backporch = 16;
	params->dsi.PLL_CLOCK = 270;
	params->dsi.lcm_esd_check_table[0].para_list[0] = -100;
	params->dsi.lcm_esd_check_table[1].cmd = 13;
	params->width = 720;
	params->type = 2;
	params->dsi.data_format.format = 2;
	params->dsi.PS = 2;
	params->dsi.vertical_sync_active = 2;
	params->dsi.horizontal_active_pixel = 720;
	params->height = 1280;
	params->dsi.vertical_active_line = 1280;
	params->dsi.mode = 1;
	params->dsi.ssc_disable = 1;
	params->dsi.clk_lp_per_line_enable = 1;
	params->dsi.esd_check_enable = 1;
	params->dsi.customization_esd_check_enable = 1;
	params->dsi.lcm_esd_check_table[0].count = 1;
	params->dsi.lcm_esd_check_table[1].count = 1;
	params->dbi.te_mode = 0;
	params->dbi.te_edge_polarity = 0;
	params->dsi.data_format.color_order = 0;
	params->dsi.data_format.trans_seq = 0;
	params->dsi.data_format.padding = 0;
	params->dsi.intermediat_buffer_num = 0;
	params->dsi.lcm_esd_check_table[1].para_list[0] = 0;
	params->dsi.vertical_frontporch = 14;
	params->dsi.horizontal_sync_active = 10;
	params->dsi.lcm_esd_check_table[0].cmd = 10;
	params->dsi.horizontal_backporch = 60;
	params->dsi.horizontal_frontporch = 60;
	params->dsi.lcm_esd_check_table[2].cmd = 14;
	params->dsi.lcm_esd_check_table[2].count = 1;
	params->dsi.lcm_esd_check_table[2].para_list[0] = 0;
}

static void lcm_init(void)
{
	unsigned int data_array[16];

	SET_RESET_PIN(1);
	MDELAY(1);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(120);

	data_array[0] = 5376;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = 276738;
	data_array[1] = 25629439;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = -2147478272;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = 211202;
	data_array[1] = 8852223;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = -1845488384;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = 211202;
	data_array[1] = 139519;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = -2147478272;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[1] = 6553792;
	data_array[0] = 669954;
	data_array[3] = 4112;
	data_array[2] = 1677725712;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = -1879042816;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = 473346;
	data_array[1] = 6029504;
	data_array[2] = 262145;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = -1577052928;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = 12588288;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = -1560275712;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = 96474368;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = -1543498496;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = 96474368;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = -1291840256;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = 211202;
	data_array[1] = 5570752;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = -2130701056;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = 1153504512;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = -1342171904;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = 211202;
	data_array[1] = 196;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = -1862265600;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[1] = 4343493;
	data_array[0] = 211202;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = 5376;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = 211202;
	data_array[1] = 10329560;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = -2147478272;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = 211202;
	data_array[1] = 8532164;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = -2147478272;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = 211202;
	data_array[1] = 8388804;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = 5376;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = 1926829312;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = -1291840256;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = -2067458816;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = -1157622528;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = -1966795520;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = -2113923840;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = 180622592;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = -973073152;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = 61871360;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = -2147478272;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[1] = 203;
	data_array[0] = 801026;
	data_array[2] = 0;
	data_array[3] = 0;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = -1879042816;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[1] = 203;
	data_array[2] = 0;
	data_array[3] = 0;
	data_array[4] = 0;
	data_array[0] = 1063170;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = -1610607360;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = 1063170;
	data_array[1] = 203;
	data_array[2] = 0;
	data_array[3] = 0;
	data_array[4] = 0;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = -1342171904;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = 1063170;
	data_array[1] = 203;
	data_array[2] = 0;
	data_array[3] = 0;
	data_array[4] = 0;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = -1073736448;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = 1063170;
	data_array[4] = 0;
	data_array[3] = 1285;
	data_array[2] = 84215045;
	data_array[1] = 84215243;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = -805300992;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = 1063170;
	data_array[2] = 0;
	data_array[1] = 1483;
	data_array[3] = 84215045;
	data_array[4] = 84215045;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = -536865536;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[1] = 1483;
	data_array[2] = 0;
	data_array[4] = 0;
	data_array[0] = 997634;
	data_array[3] = 5;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = -268430080;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[2] = -1;
	data_array[3] = -1;
	data_array[0] = 801026;
	data_array[1] = -53;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = -2147478272;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = 1063170;
	data_array[4] = 0;
	data_array[1] = 690826956;
	data_array[2] = 235670058;
	data_array[3] = 528;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = -1879042816;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = 1063170;
	data_array[1] = 1228;
	data_array[2] = 0;
	data_array[3] = 707341614;
	data_array[4] = 252513033;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = -1610607360;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[2] = 0;
	data_array[3] = 3;
	data_array[0] = 997634;
	data_array[4] = 0;
	data_array[1] = 460;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = -1342171904;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = 1063170;
	data_array[4] = 0;
	data_array[1] = 690892236;
	data_array[2] = 185405226;
	data_array[3] = 777;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = -1073736448;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = 1063170;
	data_array[1] = 460;
	data_array[2] = 0;
	data_array[3] = 707341869;
	data_array[4] = 168562192;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = -805300992;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[2] = 0;
	data_array[0] = 997634;
	data_array[4] = 0;
	data_array[1] = 1228;
	data_array[3] = 2;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = -1258285824;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = 473346;
	data_array[1] = -8321083;
	data_array[2] = 16744711;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = -2147478272;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[4] = 0;
	data_array[1] = 232398;
	data_array[0] = 866562;
	data_array[2] = -1996487798;
	data_array[3] = 59244547;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = -1879042816;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[3] = 0;
	data_array[2] = 65332;
	data_array[0] = 997634;
	data_array[4] = 0;
	data_array[1] = 16594126;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = -1610607360;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[4] = 0;
	data_array[2] = 1;
	data_array[0] = 997634;
	data_array[1] = 84097230;
	data_array[3] = 33882680;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = -1342171904;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[2] = 3;
	data_array[0] = 997634;
	data_array[4] = 0;
	data_array[1] = 83966158;
	data_array[3] = 67436600;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = -1073736448;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[4] = 0;
	data_array[0] = 997634;
	data_array[1] = 67582158;
	data_array[2] = 253;
	data_array[3] = -33290696;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = -805300992;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[4] = 0;
	data_array[0] = 997634;
	data_array[1] = 67451086;
	data_array[2] = 255;
	data_array[3] = 328760;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = -2147478272;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[2] = 0;
	data_array[3] = 0;
	data_array[0] = 997634;
	data_array[4] = 0;
	data_array[1] = 207;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = -1879042816;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[1] = 207;
	data_array[0] = 997634;
	data_array[2] = 0;
	data_array[3] = 0;
	data_array[4] = 0;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = -1610607360;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[1] = 207;
	data_array[2] = 0;
	data_array[0] = 997634;
	data_array[3] = 0;
	data_array[4] = 0;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = -1342171904;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[1] = 207;
	data_array[2] = 0;
	data_array[0] = 997634;
	data_array[3] = 0;
	data_array[4] = 0;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = -1073736448;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = 801026;
	data_array[3] = 4224;
	data_array[2] = 16777248;
	data_array[1] = 352468431;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = -1879042816;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = 342274;
	data_array[1] = 34669301;
	data_array[2] = 21;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = -1879042816;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = 1355093248;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = -1811933952;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = 276738;
	data_array[1] = 808670661;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = -1761602304;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = 818222336;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = -1308617472;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[1] = 245;
	data_array[0] = 211202;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = -1241508608;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[1] = 245;
	data_array[0] = 211202;
	dsi_set_cmdq(data_array, 2, 1);
	
	data_array[0] = -1811933952;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[1] = 245;
	data_array[0] = 211202;
	dsi_set_cmdq(data_array, 2, 1);
	
	data_array[0] = -771746560;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = 211202;
	data_array[1] = (21 << 16) | 0x6F5;
	dsi_set_cmdq(data_array, 2, 1);
	
	data_array[0] = -1275063040;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = -859499264;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = -939518720;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = 1118835968;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = 5376;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[3] = -1888463259;
	data_array[2] = 993734690;
	data_array[0] = 1390850;
	data_array[1] = 420480481;
	data_array[6] = 5;
	data_array[4] = 1314292599;
	data_array[5] = 387590973;
	dsi_set_cmdq(data_array, 7, 1);
	
	data_array[0] = 5376;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[4] = 1314292599;
	data_array[2] = 993734690;
	data_array[3] = -1888463259;
	data_array[1] = 420480482;
	data_array[5] = 387590973;
	data_array[6] = 5;
	data_array[0] = 1390850;
	dsi_set_cmdq(data_array, 7, 1);
	
	data_array[0] = 5376;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = 145666;
	data_array[1] = 40960;
	dsi_set_cmdq(data_array, 2, 1);
	
	data_array[0] = 145666;
	data_array[1] = 705;
	dsi_set_cmdq(data_array, 2, 1);

	MDELAY(1);
	data_array[0] = -1828711168;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = -1934617344;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = -1879042816;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = -1229581056;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = -1845488384;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = 45290752;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = -2013260544;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = -2134633216;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = -2147478272;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[1] = 0xFFFFFF;
	data_array[0] = 211202;
	dsi_set_cmdq(data_array, 2, 1);
	
	data_array[0] = -2147478272;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = 868488448;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = -2030037760;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = 415503616;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = 5376;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[1] = -1;
	data_array[0] = 276738;
	dsi_set_cmdq(data_array, 2, 1);
	
	data_array[0] = 1115392;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(120);
	
	data_array[0] = 2688256;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(20);
}

static void lcm_suspend(void)
{
	data_array[0] = 2622720;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(20);

	data_array[0] = 1049856;
	dsi_set_cmdq(data_array, 1, 1);
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


// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------
LCM_DRIVER otm1287_hd720_dsi_vdo_boyi_e36_lcm_drv = 
{
    .name           = "otm1287_hd720_dsi_vdo_boyi_e36",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,   
    .compare_id     = lcm_compare_id,    
};

