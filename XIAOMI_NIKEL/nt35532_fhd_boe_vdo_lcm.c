
/*----------------------------------------------------------------
* Author : Ruben (https://github.com/ruben1863) and Roger (https://github.com/R0rt1z2)
* Contact : rubenes2003@gmail.com && rogerortizleal@gmail.com
* Telegram Contact: (t.me/ruben1863) && (t.me/R0rt1z2)
* Supported device: Redmi Note 4/4X MTK (nikel)
* Copyright 2021 © Ruben1863 && R0rt1z2
 *---------------------------------------------------------------*/

#include "lcm_drv.h"

#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/gpio.h>

#include <mt-plat/mt_gpio.h>
#include <mt-plat/mt_gpio_core.h>
//#include <cust_gpio_usage.h>

#include "tps65132_i2c.h"
#include "tps65132_iic.h"

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH					(1080)
#define FRAME_HEIGHT					(1920)

#define REGFLAG_DELAY             			(0XFC)
#define REGFLAG_END_OF_TABLE      			(0xFD)

#define LCM_ID						(0x32)

#define GPIO50                              		50
#define GPIO_LCD_BL_EN_PIN				(GPIO50 | 0x80000000)

#define GPIO180                             		180
#define GPIO_LCD_RESET_PIN				(GPIO180 | 0x80000000)

#define GPIO108                             		108
#define GPIO_LCD_BIAS_ENN_PIN				(GPIO108 | 0x80000000)

#define GPIO87                              		87
#define GPIO_LCD_BIAS_ENP_PIN				(GPIO87 | 0x80000000)

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

extern char* saved_command_line;
extern int cabc_enable_flag;

static unsigned int last_backlight_level = 0;

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
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};

static struct LCM_setting_table lcm_initialization_setting[] = {

	{0XFF, 1, {0X01}},
	{REGFLAG_DELAY, 1, {}},
	{0XFB, 1, {0X01}},
	{0X00, 1, {0X01}},
	{0X01, 1, {0X55}},
	{0X02, 1, {0X59}},
	{0X04, 1, {0X0C}},
	{0X05, 1, {0X3A}},
	{0X06, 1, {0X55}},
	{0X07, 1, {0XD5}},
	{0X0D, 1, {0X9D}},
	{0X0E, 1, {0X9D}},
	{0X0F, 1, {0XE0}},
	{0X10, 1, {0X03}},
	{0X11, 1, {0X3C}},
	{0X12, 1, {0X50}},
	{0X15, 1, {0X60}},
	{0X16, 1, {0X14}},
	{0X17, 1, {0X14}},
	{0X44, 1, {0X68}},
	{0X45, 1, {0X88}},
	{0X46, 1, {0X78}},
	{0X68, 1, {0X13}},
	{0X6D, 1, {0X33}},
	{0X75, 1, {0X00}},
	{0X76, 1, {0X00}},
	{0X77, 1, {0X00}},
	{0X78, 1, {0X22}},
	{0X79, 1, {0X00}},
	{0X7A, 1, {0X51}},
	{0X7B, 1, {0X00}},
	{0X7C, 1, {0X73}},
	{0X7D, 1, {0X00}},
	{0X7E, 1, {0X8D}},
	{0X7F, 1, {0X00}},
	{0X80, 1, {0XA4}},
	{0X81, 1, {0X00}},
	{0X82, 1, {0XB8}},
	{0X83, 1, {0X00}},
	{0X84, 1, {0XCA}},
	{0X85, 1, {0X00}},
	{0X86, 1, {0XD9}},
	{0X87, 1, {0X01}},
	{0X88, 1, {0X0D}},
	{0X89, 1, {0X01}},
	{0X8A, 1, {0X36}},
	{0X8B, 1, {0X01}},
	{0X8C, 1, {0X75}},
	{0X8D, 1, {0X01}},
	{0X8E, 1, {0XA7}},
	{0X8F, 1, {0X01}},
	{0X90, 1, {0XF2}},
	{0X91, 1, {0X02}},
	{0X92, 1, {0X2B}},
	{0X93, 1, {0X02}},
	{0X94, 1, {0X2B}},
	{0X95, 1, {0X02}},
	{0X96, 1, {0X61}},
	{0X97, 1, {0X02}},
	{0X98, 1, {0X9D}},
	{0X99, 1, {0X02}},
	{0X9A, 1, {0XC6}},
	{0X9B, 1, {0X03}},
	{0X9C, 1, {0X01}},
	{0X9D, 1, {0X03}},
	{0X9E, 1, {0X27}},
	{0X9F, 1, {0X03}},
	{0XA0, 1, {0X63}},
	{0XA2, 1, {0X03}},
	{0XA3, 1, {0X6B}},
	{0XA4, 1, {0X03}},
	{0XA5, 1, {0X72}},
	{0XA6, 1, {0X03}},
	{0XA7, 1, {0X7E}},
	{0XA9, 1, {0X03}},
	{0XAA, 1, {0X87}},
	{0XAB, 1, {0X03}},
	{0XAC, 1, {0X93}},
	{0XAD, 1, {0X03}},
	{0XAE, 1, {0X9B}},
	{0XAF, 1, {0X03}},
	{0XB0, 1, {0XA2}},
	{0XB1, 1, {0X03}},
	{0XB2, 1, {0XFF}},
	{0XB3, 1, {0X00}},
	{0XB4, 1, {0X00}},
	{0XB5, 1, {0X00}},
	{0XB6, 1, {0X22}},
	{0XB7, 1, {0X00}},
	{0XB8, 1, {0X51}},
	{0XB9, 1, {0X00}},
	{0XBA, 1, {0X73}},
	{0XBB, 1, {0X00}},
	{0XBC, 1, {0X8D}},
	{0XBD, 1, {0X00}},
	{0XBE, 1, {0XA4}},
	{0XBF, 1, {0X00}},
	{0XC0, 1, {0XB8}},
	{0XC1, 1, {0X00}},
	{0XC2, 1, {0XCA}},
	{0XC3, 1, {0X00}},
	{0XC4, 1, {0XD9}},
	{0XC5, 1, {0X01}},
	{0XC6, 1, {0X0D}},
	{0XC7, 1, {0X01}},
	{0XC8, 1, {0X36}},
	{0XC9, 1, {0X01}},
	{0XCA, 1, {0X75}},
	{0XCB, 1, {0X01}},
	{0XCC, 1, {0XA7}},
	{0XCD, 1, {0X01}},
	{0XCE, 1, {0XF2}},
	{0XCF, 1, {0X02}},
	{0XD0, 1, {0X2B}},
	{0XD1, 1, {0X02}},
	{0XD2, 1, {0X2B}},
	{0XD3, 1, {0X02}},
	{0XD4, 1, {0X61}},
	{0XD5, 1, {0X02}},
	{0XD6, 1, {0X9D}},
	{0XD7, 1, {0X02}},
	{0XD8, 1, {0XC6}},
	{0XD9, 1, {0X03}},
	{0XDA, 1, {0X01}},
	{0XDB, 1, {0X03}},
	{0XDC, 1, {0X27}},
	{0XDD, 1, {0X03}},
	{0XDE, 1, {0X63}},
	{0XDF, 1, {0X03}},
	{0XE0, 1, {0X6B}},
	{0XE1, 1, {0X03}},
	{0XE2, 1, {0X72}},
	{0XE3, 1, {0X03}},
	{0XE4, 1, {0X7E}},
	{0XE5, 1, {0X03}},
	{0XE6, 1, {0X87}},
	{0XE7, 1, {0X03}},
	{0XE8, 1, {0X93}},
	{0XE9, 1, {0X03}},
	{0XEA, 1, {0X9B}},
	{0XEB, 1, {0X03}},
	{0XEC, 1, {0XA2}},
	{0XED, 1, {0X03}},
	{0XEE, 1, {0XFF}},
	{0XEF, 1, {0X00}},
	{0XF0, 1, {0XD6}},
	{0XF1, 1, {0X00}},
	{0XF2, 1, {0XDD}},
	{0XF3, 1, {0X00}},
	{0XF4, 1, {0XEA}},
	{0XF5, 1, {0X00}},
	{0XF6, 1, {0XF7}},
	{0XF7, 1, {0X01}},
	{0XF8, 1, {0X03}},
	{0XF9, 1, {0X01}},
	{0XFA, 1, {0X0D}},
	{0XFF, 1, {0X02}},
	{REGFLAG_DELAY, 1, {}},
	{0XFB, 1, {0X01}},
	{0X00, 1, {0X01}},
	{0X01, 1, {0X17}},
	{0X02, 1, {0X01}},
	{0X03, 1, {0X21}},
	{0X04, 1, {0X01}},
	{0X05, 1, {0X2A}},
	{0X06, 1, {0X01}},
	{0X07, 1, {0X4B}},
	{0X08, 1, {0X01}},
	{0X09, 1, {0X68}},
	{0X0A, 1, {0X01}},
	{0X0B, 1, {0X98}},
	{0X0C, 1, {0X01}},
	{0X0D, 1, {0XC0}},
	{0X0E, 1, {0X02}},
	{0X0F, 1, {0X00}},
	{0X10, 1, {0X02}},
	{0X11, 1, {0X34}},
	{0X12, 1, {0X02}},
	{0X13, 1, {0X35}},
	{0X14, 1, {0X02}},
	{0X15, 1, {0X68}},
	{0X16, 1, {0X02}},
	{0X17, 1, {0XA4}},
	{0X18, 1, {0X02}},
	{0X19, 1, {0XCC}},
	{0X1A, 1, {0X03}},
	{0X1B, 1, {0X07}},
	{0X1C, 1, {0X03}},
	{0X1D, 1, {0X2F}},
	{0X1E, 1, {0X03}},
	{0X1F, 1, {0X6A}},
	{0X20, 1, {0X03}},
	{0X21, 1, {0X71}},
	{0X22, 1, {0X03}},
	{0X23, 1, {0X85}},
	{0X24, 1, {0X03}},
	{0X25, 1, {0X9A}},
	{0X26, 1, {0X03}},
	{0X27, 1, {0XB2}},
	{0X28, 1, {0X03}},
	{0X29, 1, {0XCA}},
	{0X2A, 1, {0X03}},
	{0X2B, 1, {0XE2}},
	{0X2D, 1, {0X03}},
	{0X2F, 1, {0XF5}},
	{0X30, 1, {0X03}},
	{0X31, 1, {0XFF}},
	{0X32, 1, {0X00}},
	{0X33, 1, {0XD6}},
	{0X34, 1, {0X00}},
	{0X35, 1, {0XDD}},
	{0X36, 1, {0X00}},
	{0X37, 1, {0XEA}},
	{0X38, 1, {0X00}},
	{0X39, 1, {0XF7}},
	{0X3A, 1, {0X01}},
	{0X3B, 1, {0X03}},
	{0X3D, 1, {0X01}},
	{0X3F, 1, {0X0D}},
	{0X40, 1, {0X01}},
	{0X41, 1, {0X17}},
	{0X42, 1, {0X01}},
	{0X43, 1, {0X21}},
	{0X44, 1, {0X01}},
	{0X45, 1, {0X2A}},
	{0X46, 1, {0X01}},
	{0X47, 1, {0X4B}},
	{0X48, 1, {0X01}},
	{0X49, 1, {0X68}},
	{0X4A, 1, {0X01}},
	{0X4B, 1, {0X98}},
	{0X4C, 1, {0X01}},
	{0X4D, 1, {0XC0}},
	{0X4E, 1, {0X02}},
	{0X4F, 1, {0X00}},
	{0X50, 1, {0X02}},
	{0X51, 1, {0X34}},
	{0X52, 1, {0X02}},
	{0X53, 1, {0X35}},
	{0X54, 1, {0X02}},
	{0X55, 1, {0X68}},
	{0X56, 1, {0X02}},
	{0X58, 1, {0XA4}},
	{0X59, 1, {0X02}},
	{0X5A, 1, {0XCC}},
	{0X5B, 1, {0X03}},
	{0X5C, 1, {0X07}},
	{0X5D, 1, {0X03}},
	{0X5E, 1, {0X2F}},
	{0X5F, 1, {0X03}},
	{0X60, 1, {0X6A}},
	{0X61, 1, {0X03}},
	{0X62, 1, {0X71}},
	{0X63, 1, {0X03}},
	{0X64, 1, {0X85}},
	{0X65, 1, {0X03}},
	{0X66, 1, {0X9A}},
	{0X67, 1, {0X03}},
	{0X68, 1, {0XB2}},
	{0X69, 1, {0X03}},
	{0X6A, 1, {0XCA}},
	{0X6B, 1, {0X03}},
	{0X6C, 1, {0XE2}},
	{0X6D, 1, {0X03}},
	{0X6E, 1, {0XF5}},
	{0X6F, 1, {0X03}},
	{0X70, 1, {0XFF}},
	{0X71, 1, {0X00}},
	{0X72, 1, {0XCD}},
	{0X73, 1, {0X00}},
	{0X74, 1, {0XD5}},
	{0X75, 1, {0X00}},
	{0X76, 1, {0XE3}},
	{0X77, 1, {0X00}},
	{0X78, 1, {0XEF}},
	{0X79, 1, {0X00}},
	{0X7A, 1, {0XFB}},
	{0X7B, 1, {0X01}},
	{0X7C, 1, {0X06}},
	{0X7D, 1, {0X01}},
	{0X7E, 1, {0X11}},
	{0X7F, 1, {0X01}},
	{0X80, 1, {0X1B}},
	{0X81, 1, {0X01}},
	{0X82, 1, {0X24}},
	{0X83, 1, {0X01}},
	{0X84, 1, {0X46}},
	{0X85, 1, {0X01}},
	{0X86, 1, {0X63}},
	{0X87, 1, {0X01}},
	{0X88, 1, {0X94}},
	{0X89, 1, {0X01}},
	{0X8A, 1, {0XBB}},
	{0X8B, 1, {0X01}},
	{0X8C, 1, {0xFC}},
	{0X8D, 1, {0X02}},
	{0X8E, 1, {0X32}},
	{0X8F, 1, {0X02}},
	{0X90, 1, {0X32}},
	{0X91, 1, {0X02}},
	{0X92, 1, {0X66}},
	{0X93, 1, {0X02}},
	{0X94, 1, {0XA3}},
	{0X95, 1, {0X02}},
	{0X96, 1, {0XCC}},
	{0X97, 1, {0X03}},
	{0X98, 1, {0X0B}},
	{0X99, 1, {0X03}},
	{0X9A, 1, {0X38}},
	{0X9B, 1, {0X03}},
	{0X9C, 1, {0X8A}},
	{0X9D, 1, {0X03}},
	{0X9E, 1, {0X97}},
	{0X9F, 1, {0X03}},
	{0XA0, 1, {0X97}},
	{0XA2, 1, {0X03}},
	{0XA3, 1, {0X99}},
	{0XA4, 1, {0X03}},
	{0XA5, 1, {0X9B}},
	{0XA6, 1, {0X03}},
	{0XA7, 1, {0X9D}},
	{0XA9, 1, {0X03}},
	{0XAA, 1, {0X9F}},
	{0XAB, 1, {0X03}},
	{0XAC, 1, {0XA1}},
	{0XAD, 1, {0X03}},
	{0XAE, 1, {0XA2}},
	{0XAF, 1, {0X00}},
	{0XB0, 1, {0XCD}},
	{0XB1, 1, {0X00}},
	{0XB2, 1, {0XD5}},
	{0XB3, 1, {0X00}},
	{0XB4, 1, {0XE3}},
	{0XB5, 1, {0X00}},
	{0XB6, 1, {0XEF}},
	{0XB7, 1, {0X00}},
	{0XB8, 1, {0XFB}},
	{0XB9, 1, {0X01}},
	{0XBA, 1, {0X06}},
	{0XBB, 1, {0X01}},
	{0XBC, 1, {0X11}},
	{0XBD, 1, {0X01}},
	{0XBE, 1, {0X1B}},
	{0XBF, 1, {0X01}},
	{0XC0, 1, {0X24}},
	{0XC1, 1, {0X01}},
	{0XC2, 1, {0X46}},
	{0XC3, 1, {0X01}},
	{0XC4, 1, {0X63}},
	{0XC5, 1, {0X01}},
	{0XC6, 1, {0X94}},
	{0XC7, 1, {0X01}},
	{0XC8, 1, {0XBB}},
	{0XC9, 1, {0X01}},
	{0XCA, 1, {0xFC}},
	{0XCB, 1, {0X02}},
	{0XCC, 1, {0X32}},
	{0XCD, 1, {0X02}},
	{0XCE, 1, {0X32}},
	{0XCF, 1, {0X02}},
	{0XD0, 1, {0X66}},
	{0XD1, 1, {0X02}},
	{0XD2, 1, {0XA3}},
	{0XD3, 1, {0X02}},
	{0XD4, 1, {0XCC}},
	{0XD5, 1, {0X03}},
	{0XD6, 1, {0X0B}},
	{0XD7, 1, {0X03}},
	{0XD8, 1, {0X38}},
	{0XD9, 1, {0X03}},
	{0XDA, 1, {0X8A}},
	{0XDB, 1, {0X03}},
	{0XDC, 1, {0X97}},
	{0XDD, 1, {0X03}},
	{0XDE, 1, {0X97}},
	{0XDF, 1, {0X03}},
	{0XE0, 1, {0X99}},
	{0XE1, 1, {0X03}},
	{0XE2, 1, {0X9B}},
	{0XE3, 1, {0X03}},
	{0XE4, 1, {0X9D}},
	{0XE5, 1, {0X03}},
	{0XE6, 1, {0X9F}},
	{0XE7, 1, {0X03}},
	{0XE8, 1, {0XA1}},
	{0XE9, 1, {0X03}},
	{0XEA, 1, {0XA2}},
	{0XFF, 1, {0X05}},
	{REGFLAG_DELAY, 1, {}},
	{0XFB, 1, {0X01}},
	{0X00, 1, {0X35}},
	{0X01, 1, {0X08}},
	{0X02, 1, {0X06}},
	{0X03, 1, {0X04}},
	{0X04, 1, {0X34}},
	{0X05, 1, {0X1A}},
	{0X06, 1, {0X1A}},
	{0X07, 1, {0X16}},
	{0X08, 1, {0X16}},
	{0X09, 1, {0X22}},
	{0X0A, 1, {0X22}},
	{0X0B, 1, {0X1E}},
	{0X0C, 1, {0X1E}},
	{0X0D, 1, {0X05}},
	{0X0E, 1, {0X40}},
	{0X0F, 1, {0X40}},
	{0X10, 1, {0X40}},
	{0X11, 1, {0X40}},
	{0X12, 1, {0X40}},
	{0X13, 1, {0X40}},
	{0X14, 1, {0X35}},
	{0X15, 1, {0X09}},
	{0X16, 1, {0X07}},
	{0X17, 1, {0X04}},
	{0X18, 1, {0X34}},
	{0X19, 1, {0X1C}},
	{0X1A, 1, {0X1C}},
	{0X1B, 1, {0X18}},
	{0X1C, 1, {0X18}},
	{0X1D, 1, {0X24}},
	{0X1E, 1, {0X24}},
	{0X1F, 1, {0X20}},
	{0X20, 1, {0X20}},
	{0X21, 1, {0X05}},
	{0X22, 1, {0X40}},
	{0X23, 1, {0X40}},
	{0X24, 1, {0X40}},
	{0X25, 1, {0X40}},
	{0X26, 1, {0X40}},
	{0X27, 1, {0X40}},
	{0X28, 1, {0X35}},
	{0X29, 1, {0X07}},
	{0X2A, 1, {0X09}},
	{0X2B, 1, {0X04}},
	{0X2D, 1, {0X34}},
	{0X2F, 1, {0X20}},
	{0X30, 1, {0X20}},
	{0X31, 1, {0X24}},
	{0X32, 1, {0X24}},
	{0X33, 1, {0X18}},
	{0X34, 1, {0X18}},
	{0X35, 1, {0X1C}},
	{0X36, 1, {0X1C}},
	{0X37, 1, {0X05}},
	{0X38, 1, {0X40}},
	{0X39, 1, {0X40}},
	{0X3A, 1, {0X40}},
	{0X3B, 1, {0X40}},
	{0X3D, 1, {0X40}},
	{0X3F, 1, {0X40}},
	{0X40, 1, {0X35}},
	{0X41, 1, {0X06}},
	{0X42, 1, {0X08}},
	{0X43, 1, {0X04}},
	{0X44, 1, {0X34}},
	{0X45, 1, {0X1E}},
	{0X46, 1, {0X1E}},
	{0X47, 1, {0X22}},
	{0X48, 1, {0X22}},
	{0X49, 1, {0X16}},
	{0X4A, 1, {0X16}},
	{0X4B, 1, {0X1A}},
	{0X4C, 1, {0X1A}},
	{0X4D, 1, {0X05}},
	{0X4E, 1, {0X40}},
	{0X4F, 1, {0X40}},
	{0X50, 1, {0X40}},
	{0X51, 1, {0X40}},
	{0X52, 1, {0X40}},
	{0X53, 1, {0X40}},
	{0X54, 1, {0X08}},
	{0X55, 1, {0X06}},
	{0X56, 1, {0X08}},
	{0X58, 1, {0X06}},
	{0X59, 1, {0X1B}},
	{0X5A, 1, {0X1B}},
	{0X5B, 1, {0X48}},
	{0X5C, 1, {0X0E}},
	{0X5D, 1, {0X01}},
	{0X65, 1, {0X00}},
	{0X66, 1, {0X44}},
	{0X67, 1, {0X00}},
	{0X68, 1, {0X48}},
	{0X69, 1, {0X0E}},
	{0X6A, 1, {0X06}},
	{0X6B, 1, {0X20}},
	{0X6C, 1, {0X08}},
	{0X6D, 1, {0X00}},
	{0X76, 1, {0X00}},
	{0X77, 1, {0X00}},
	{0X78, 1, {0X02}},
	{0X79, 1, {0X00}},
	{0X7A, 1, {0X0A}},
	{0X7B, 1, {0X05}},
	{0X7C, 1, {0X00}},
	{0X7D, 1, {0X0D}},
	{0X7E, 1, {0X33}},
	{0X7F, 1, {0X33}},
	{0X80, 1, {0X33}},
	{0X81, 1, {0X00}},
	{0X82, 1, {0X00}},
	{0X83, 1, {0X00}},
	{0X84, 1, {0X30}},
	{0X85, 1, {0XFF}},
	{0X86, 1, {0XFF}},
	{0XBB, 1, {0X88}},
	{0XB7, 1, {0XFF}},
	{0XB8, 1, {0X00}},
	{0XBA, 1, {0X13}},
	{0XBC, 1, {0X95}},
	{0XBD, 1, {0XAA}},
	{0XBE, 1, {0X08}},
	{0XBF, 1, {0XA3}},
	{0XC8, 1, {0X00}},
	{0XC9, 1, {0X00}},
	{0XCA, 1, {0X00}},
	{0XCB, 1, {0X00}},
	{0XCC, 1, {0X12}},
	{0XCF, 1, {0X44}},
	{0XD0, 1, {0X00}},
	{0XD1, 1, {0X00}},
	{0XD4, 1, {0X15}},
	{0XD5, 1, {0XBF}},
	{0XD6, 1, {0X22}},
	{0X90, 1, {0X78}},
	{0X91, 1, {0X10}},
	{0X92, 1, {0X10}},
	{0X97, 1, {0X08}},
	{0X98, 1, {0X00}},
	{0X99, 1, {0X00}},
	{0X9B, 1, {0X68}},
	{0X9C, 1, {0X0A}},
	{0XFF, 1, {0X00}},
	{REGFLAG_DELAY, 1, {}},
	{0X36, 1, {0X00}},
	{0XFF, 1, {0X01}},
	{REGFLAG_DELAY, 1, {}},
	{0XFB, 1, {0X01}},
	{0X6E, 1, {0X00}},
	{0XFF, 1, {0X04}},
	{REGFLAG_DELAY, 1, {}},
	{0XFB, 1, {0X01}},
	{0X08, 1, {0X06}},
	{0XFF, 1, {0X00}},
	{REGFLAG_DELAY, 1, {}},
	{0XFB, 1, {0X01}},
	{0X51, 1, {0XFF}},
	{0X53, 1, {0X24}},
	{0X55, 1, {0X00}},
	{0XD3, 1, {0X12}},
	{0XD4, 1, {0X16}},
	{0X35, 1, {0X00}},
	{0X11, 0, {}},
	{REGFLAG_DELAY, 120, {}},
	{0X29, 0, {}},
	{REGFLAG_DELAY, 20, {}},
	{REGFLAG_END_OF_TABLE, 0X00, {}}
};

static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] =
{
	{0x28, 0, {}},
	{REGFLAG_DELAY, 100, {}},
	{0x10, 0, {}},
	{REGFLAG_DELAY, 120, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_cabc_on_setting[]=
{
	{0x55, 1, {0x01}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_cabc_off_setting[] =
{
	{0x55, 1, {0x00}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_backlight_level_setting[] =
{
	{0x51, 1, {0xFF}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_cabc_on_initialization_setting[] = {

	{0xff, 1, {0x01}},
	{0xfb, 1, {0x01}},
	{0x00, 1, {0x01}},
	{0x01, 1, {0x55}},
	{0x02, 1, {0x40}},
	{0x05, 1, {0x40}},
	{0x06, 1, {0x0a}},
	{0x07, 1, {0x14}},
	{0x08, 1, {0x0c}},
	{0x0b, 1, {0x7d}},
	{0x0c, 1, {0x7d}},
	{0x0e, 1, {0xab}},
	{0x0f, 1, {0xa4}},
	{0x14, 1, {0x14}},
	{0x15, 1, {0x13}},
	{0x16, 1, {0x13}},
	{0x18, 1, {0x00}},
	{0x19, 1, {0x77}},
	{0x1a, 1, {0x55}},
	{0x1b, 1, {0x13}},
	{0x1c, 1, {0x00}},
	{0x1d, 1, {0x00}},
	{0x1e, 1, {0x13}},
	{0x1f, 1, {0x00}},
	{0x35, 1, {0x00}},
	{0x66, 1, {0x00}},
	{0x58, 1, {0x81}},
	{0x59, 1, {0x01}},
	{0x5a, 1, {0x01}},
	{0x5b, 1, {0x01}},
	{0x5c, 1, {0x82}},
	{0x5d, 1, {0x82}},
	{0x5e, 1, {0x02}},
	{0x5f, 1, {0x02}},
	{0x6d, 1, {0x22}},
	{0x72, 1, {0x31}},
	{0xff, 1, {0x05}},
	{0xfb, 1, {0x01}},
	{0x00, 1, {0x00}},
	{0x01, 1, {0x00}},
	{0x02, 1, {0x03}},
	{0x03, 1, {0x04}},
	{0x04, 1, {0x00}},
	{0x05, 1, {0x11}},
	{0x06, 1, {0x0c}},
	{0x07, 1, {0x0b}},
	{0x08, 1, {0x01}},
	{0x09, 1, {0x00}},
	{0x0a, 1, {0x18}},
	{0x0b, 1, {0x16}},
	{0x0c, 1, {0x14}},
	{0x0d, 1, {0x17}},
	{0x0e, 1, {0x15}},
	{0x0f, 1, {0x13}},
	{0x10, 1, {0x00}},
	{0x11, 1, {0x00}},
	{0x12, 1, {0x03}},
	{0x13, 1, {0x04}},
	{0x14, 1, {0x00}},
	{0x15, 1, {0x11}},
	{0x16, 1, {0x0c}},
	{0x17, 1, {0x0b}},
	{0x18, 1, {0x01}},
	{0x19, 1, {0x00}},
	{0x1a, 1, {0x18}},
	{0x1b, 1, {0x16}},
	{0x1c, 1, {0x14}},
	{0x1d, 1, {0x17}},
	{0x1e, 1, {0x15}},
	{0x1f, 1, {0x13}},
	{0x20, 1, {0x00}},
	{0x21, 1, {0x02}},
	{0x22, 1, {0x09}},
	{0x23, 1, {0x67}},
	{0x24, 1, {0x06}},
	{0x25, 1, {0x1d}},
	{0x29, 1, {0x58}},
	{0x2a, 1, {0x11}},
	{0x2b, 1, {0x04}},
	{0x2f, 1, {0x02}},
	{0x30, 1, {0x01}},
	{0x31, 1, {0x49}},
	{0x32, 1, {0x23}},
	{0x33, 1, {0x01}},
	{0x34, 1, {0x03}},
	{0x35, 1, {0x6b}},
	{0x36, 1, {0x00}},
	{0x37, 1, {0x1d}},
	{0x38, 1, {0x00}},
	{0x5d, 1, {0x23}},
	{0x61, 1, {0x15}},
	{0x65, 1, {0x00}},
	{0x69, 1, {0x04}},
	{0x6c, 1, {0x51}},
	{0x7a, 1, {0x00}},
	{0x7b, 1, {0x80}},
	{0x7c, 1, {0xd8}},
	{0x7d, 1, {0x10}},
	{0x7e, 1, {0x06}},
	{0x7f, 1, {0x1b}},
	{0x81, 1, {0x06}},
	{0x82, 1, {0x02}},
	{0x8a, 1, {0x33}},
	{0x93, 1, {0x06}},
	{0x94, 1, {0x06}},
	{0x9b, 1, {0x0f}},
	{0xa4, 1, {0x0f}},
	{0xe7, 1, {0x80}},
	{0xff, 1, {0x01}},
	{0xfb, 1, {0x01}},
	{0x75, 1, {0x00}},
	{0x76, 1, {0x00}},
	{0x77, 1, {0x00}},
	{0x78, 1, {0x21}},
	{0x79, 1, {0x00}},
	{0x7a, 1, {0x4a}},
	{0x7b, 1, {0x00}},
	{0x7c, 1, {0x66}},
	{0x7d, 1, {0x00}},
	{0x7e, 1, {0x7f}},
	{0x7f, 1, {0x00}},
	{0x80, 1, {0x94}},
	{0x81, 1, {0x00}},
	{0x82, 1, {0xa7}},
	{0x83, 1, {0x00}},
	{0x84, 1, {0xb8}},
	{0x85, 1, {0x00}},
	{0x86, 1, {0xc7}},
	{0x87, 1, {0x00}},
	{0x88, 1, {0xfb}},
	{0x89, 1, {0x01}},
	{0x8a, 1, {0x25}},
	{0x8b, 1, {0x01}},
	{0x8c, 1, {0x61}},
	{0x8d, 1, {0x01}},
	{0x8e, 1, {0x94}},
	{0x8f, 1, {0x01}},
	{0x90, 1, {0xe2}},
	{0x91, 1, {0x02}},
	{0x92, 1, {0x20}},
	{0x93, 1, {0x02}},
	{0x94, 1, {0x22}},
	{0x95, 1, {0x02}},
	{0x96, 1, {0x5c}},
	{0x97, 1, {0x02}},
	{0x98, 1, {0x9e}},
	{0x99, 1, {0x02}},
	{0x9a, 1, {0xc9}},
	{0x9b, 1, {0x03}},
	{0x9c, 1, {0x01}},
	{0x9d, 1, {0x03}},
	{0x9e, 1, {0x28}},
	{0x9f, 1, {0x03}},
	{0xa0, 1, {0x55}},
	{0xa2, 1, {0x03}},
	{0xa3, 1, {0x62}},
	{0xa4, 1, {0x03}},
	{0xa5, 1, {0x6f}},
	{0xa6, 1, {0x03}},
	{0xa7, 1, {0x7e}},
	{0xa9, 1, {0x03}},
	{0xaa, 1, {0x8f}},
	{0xab, 1, {0x03}},
	{0xac, 1, {0x9c}},
	{0xad, 1, {0x03}},
	{0xae, 1, {0xa2}},
	{0xaf, 1, {0x03}},
	{0xb0, 1, {0xab}},
	{0xb1, 1, {0x03}},
	{0xb2, 1, {0xb2}},
	{0xb3, 1, {0x00}},
	{0xb4, 1, {0x00}},
	{0xb5, 1, {0x00}},
	{0xb6, 1, {0x21}},
	{0xb7, 1, {0x00}},
	{0xb8, 1, {0x4a}},
	{0xb9, 1, {0x00}},
	{0xba, 1, {0x66}},
	{0xbb, 1, {0x00}},
	{0xbc, 1, {0x7f}},
	{0xbd, 1, {0x00}},
	{0xbe, 1, {0x94}},
	{0xbf, 1, {0x00}},
	{0xc0, 1, {0xa7}},
	{0xc1, 1, {0x00}},
	{0xc2, 1, {0xb8}},
	{0xc3, 1, {0x00}},
	{0xc4, 1, {0xc7}},
	{0xc5, 1, {0x00}},
	{0xc6, 1, {0xfb}},
	{0xc7, 1, {0x01}},
	{0xc8, 1, {0x25}},
	{0xc9, 1, {0x01}},
	{0xca, 1, {0x61}},
	{0xcb, 1, {0x01}},
	{0xcc, 1, {0x94}},
	{0xcd, 1, {0x01}},
	{0xce, 1, {0xe2}},
	{0xcf, 1, {0x02}},
	{0xd0, 1, {0x20}},
	{0xd1, 1, {0x02}},
	{0xd2, 1, {0x22}},
	{0xd3, 1, {0x02}},
	{0xd4, 1, {0x5c}},
	{0xd5, 1, {0x02}},
	{0xd6, 1, {0x9e}},
	{0xd7, 1, {0x02}},
	{0xd8, 1, {0xc9}},
	{0xd9, 1, {0x03}},
	{0xda, 1, {0x01}},
	{0xdb, 1, {0x03}},
	{0xdc, 1, {0x28}},
	{0xdd, 1, {0x03}},
	{0xde, 1, {0x55}},
	{0xdf, 1, {0x03}},
	{0xe0, 1, {0x62}},
	{0xe1, 1, {0x03}},
	{0xe2, 1, {0x6f}},
	{0xe3, 1, {0x03}},
	{0xe4, 1, {0x7e}},
	{0xe5, 1, {0x03}},
	{0xe6, 1, {0x8f}},
	{0xe7, 1, {0x03}},
	{0xe8, 1, {0x9c}},
	{0xe9, 1, {0x03}},
	{0xea, 1, {0xa2}},
	{0xeb, 1, {0x03}},
	{0xec, 1, {0xab}},
	{0xed, 1, {0x03}},
	{0xee, 1, {0xb2}},
	{0xef, 1, {0x00}},
	{0xf0, 1, {0x00}},
	{0xf1, 1, {0x00}},
	{0xf2, 1, {0x21}},
	{0xf3, 1, {0x00}},
	{0xf4, 1, {0x4a}},
	{0xf5, 1, {0x00}},
	{0xf6, 1, {0x66}},
	{0xf7, 1, {0x00}},
	{0xf8, 1, {0x7f}},
	{0xf9, 1, {0x00}},
	{0xfa, 1, {0x94}},
	{0xff, 1, {0x02}},
	{0xfb, 1, {0x01}},
	{0x00, 1, {0x00}},
	{0x01, 1, {0xa7}},
	{0x02, 1, {0x00}},
	{0x03, 1, {0xb8}},
	{0x04, 1, {0x00}},
	{0x05, 1, {0xc7}},
	{0x06, 1, {0x00}},
	{0x07, 1, {0xfb}},
	{0x08, 1, {0x01}},
	{0x09, 1, {0x25}},
	{0x0a, 1, {0x01}},
	{0x0b, 1, {0x61}},
	{0x0c, 1, {0x01}},
	{0x0d, 1, {0x94}},
	{0x0e, 1, {0x01}},
	{0x0f, 1, {0xe2}},
	{0x10, 1, {0x02}},
	{0x11, 1, {0x20}},
	{0x12, 1, {0x02}},
	{0x13, 1, {0x22}},
	{0x14, 1, {0x02}},
	{0x15, 1, {0x5c}},
	{0x16, 1, {0x02}},
	{0x17, 1, {0x9e}},
	{0x18, 1, {0x02}},
	{0x19, 1, {0xc9}},
	{0x1a, 1, {0x03}},
	{0x1b, 1, {0x01}},
	{0x1c, 1, {0x03}},
	{0x1d, 1, {0x28}},
	{0x1e, 1, {0x03}},
	{0x1f, 1, {0x55}},
	{0x20, 1, {0x03}},
	{0x21, 1, {0x62}},
	{0x22, 1, {0x03}},
	{0x23, 1, {0x6f}},
	{0x24, 1, {0x03}},
	{0x25, 1, {0x7e}},
	{0x26, 1, {0x03}},
	{0x27, 1, {0x8f}},
	{0x28, 1, {0x03}},
	{0x29, 1, {0x9c}},
	{0x2a, 1, {0x03}},
	{0x2b, 1, {0xa2}},
	{0x2d, 1, {0x03}},
	{0x2f, 1, {0xab}},
	{0x30, 1, {0x03}},
	{0x31, 1, {0xb2}},
	{0x32, 1, {0x00}},
	{0x33, 1, {0x00}},
	{0x34, 1, {0x00}},
	{0x35, 1, {0x21}},
	{0x36, 1, {0x00}},
	{0x37, 1, {0x4a}},
	{0x38, 1, {0x00}},
	{0x39, 1, {0x66}},
	{0x3a, 1, {0x00}},
	{0x3b, 1, {0x7f}},
	{0x3d, 1, {0x00}},
	{0x3f, 1, {0x94}},
	{0x40, 1, {0x00}},
	{0x41, 1, {0xa7}},
	{0x42, 1, {0x00}},
	{0x43, 1, {0xb8}},
	{0x44, 1, {0x00}},
	{0x45, 1, {0xc7}},
	{0x46, 1, {0x00}},
	{0x47, 1, {0xfb}},
	{0x48, 1, {0x01}},
	{0x49, 1, {0x25}},
	{0x4a, 1, {0x01}},
	{0x4b, 1, {0x61}},
	{0x4c, 1, {0x01}},
	{0x4d, 1, {0x94}},
	{0x4e, 1, {0x01}},
	{0x4f, 1, {0xe2}},
	{0x50, 1, {0x02}},
	{0x51, 1, {0x20}},
	{0x52, 1, {0x02}},
	{0x53, 1, {0x22}},
	{0x54, 1, {0x02}},
	{0x55, 1, {0x5c}},
	{0x56, 1, {0x02}},
	{0x58, 1, {0x9e}},
	{0x59, 1, {0x02}},
	{0x5a, 1, {0xc9}},
	{0x5b, 1, {0x03}},
	{0x5c, 1, {0x01}},
	{0x5d, 1, {0x03}},
	{0x5e, 1, {0x28}},
	{0x5f, 1, {0x03}},
	{0x60, 1, {0x55}},
	{0x61, 1, {0x03}},
	{0x62, 1, {0x62}},
	{0x63, 1, {0x03}},
	{0x64, 1, {0x6f}},
	{0x65, 1, {0x03}},
	{0x66, 1, {0x7e}},
	{0x67, 1, {0x03}},
	{0x68, 1, {0x8f}},
	{0x69, 1, {0x03}},
	{0x6a, 1, {0x9c}},
	{0x6b, 1, {0x03}},
	{0x6c, 1, {0xa2}},
	{0x6d, 1, {0x03}},
	{0x6e, 1, {0xab}},
	{0x6f, 1, {0x03}},
	{0x70, 1, {0xb2}},
	{0x71, 1, {0x00}},
	{0x72, 1, {0x00}},
	{0x73, 1, {0x00}},
	{0x74, 1, {0x1e}},
	{0x75, 1, {0x00}},
	{0x76, 1, {0x48}},
	{0x77, 1, {0x00}},
	{0x78, 1, {0x57}},
	{0x79, 1, {0x00}},
	{0x7a, 1, {0x6a}},
	{0x7b, 1, {0x00}},
	{0x7c, 1, {0x80}},
	{0x7d, 1, {0x00}},
	{0x7e, 1, {0x90}},
	{0x7f, 1, {0x00}},
	{0x80, 1, {0xa0}},
	{0x81, 1, {0x00}},
	{0x82, 1, {0xae}},
	{0x83, 1, {0x00}},
	{0x84, 1, {0xe3}},
	{0x85, 1, {0x01}},
	{0x86, 1, {0x0e}},
	{0x87, 1, {0x01}},
	{0x88, 1, {0x50}},
	{0x89, 1, {0x01}},
	{0x8a, 1, {0x88}},
	{0x8b, 1, {0x01}},
	{0x8c, 1, {0xda}},
	{0x8d, 1, {0x02}},
	{0x8e, 1, {0x19}},
	{0x8f, 1, {0x02}},
	{0x90, 1, {0x1b}},
	{0x91, 1, {0x02}},
	{0x92, 1, {0x58}},
	{0x93, 1, {0x02}},
	{0x94, 1, {0x9c}},
	{0x95, 1, {0x02}},
	{0x96, 1, {0xc6}},
	{0x97, 1, {0x03}},
	{0x98, 1, {0x01}},
	{0x99, 1, {0x03}},
	{0x9a, 1, {0x28}},
	{0x9b, 1, {0x03}},
	{0x9c, 1, {0x55}},
	{0x9d, 1, {0x03}},
	{0x9e, 1, {0x62}},
	{0x9f, 1, {0x03}},
	{0xa0, 1, {0x6f}},
	{0xa2, 1, {0x03}},
	{0xa3, 1, {0x7e}},
	{0xa4, 1, {0x03}},
	{0xa5, 1, {0x8f}},
	{0xa6, 1, {0x03}},
	{0xa7, 1, {0x9c}},
	{0xa9, 1, {0x03}},
	{0xaa, 1, {0xa2}},
	{0xab, 1, {0x03}},
	{0xac, 1, {0xab}},
	{0xad, 1, {0x03}},
	{0xae, 1, {0xb2}},
	{0xaf, 1, {0x00}},
	{0xb0, 1, {0x00}},
	{0xb1, 1, {0x00}},
	{0xb2, 1, {0x1e}},
	{0xb3, 1, {0x00}},
	{0xb4, 1, {0x48}},
	{0xb5, 1, {0x00}},
	{0xb6, 1, {0x57}},
	{0xb7, 1, {0x00}},
	{0xb8, 1, {0x6a}},
	{0xb9, 1, {0x00}},
	{0xba, 1, {0x80}},
	{0xbb, 1, {0x00}},
	{0xbc, 1, {0x90}},
	{0xbd, 1, {0x00}},
	{0xbe, 1, {0xa0}},
	{0xbf, 1, {0x00}},
	{0xc0, 1, {0xae}},
	{0xc1, 1, {0x00}},
	{0xc2, 1, {0xe3}},
	{0xc3, 1, {0x01}},
	{0xc4, 1, {0x0e}},
	{0xc5, 1, {0x01}},
	{0xc6, 1, {0x50}},
	{0xc7, 1, {0x01}},
	{0xc8, 1, {0x88}},
	{0xc9, 1, {0x01}},
	{0xca, 1, {0xda}},
	{0xcb, 1, {0x02}},
	{0xcc, 1, {0x19}},
	{0xcd, 1, {0x02}},
	{0xce, 1, {0x1b}},
	{0xcf, 1, {0x02}},
	{0xd0, 1, {0x58}},
	{0xd1, 1, {0x02}},
	{0xd2, 1, {0x9c}},
	{0xd3, 1, {0x02}},
	{0xd4, 1, {0xc6}},
	{0xd5, 1, {0x03}},
	{0xd6, 1, {0x01}},
	{0xd7, 1, {0x03}},
	{0xd8, 1, {0x28}},
	{0xd9, 1, {0x03}},
	{0xda, 1, {0x55}},
	{0xdb, 1, {0x03}},
	{0xdc, 1, {0x62}},
	{0xdd, 1, {0x03}},
	{0xde, 1, {0x6f}},
	{0xdf, 1, {0x03}},
	{0xe0, 1, {0x7e}},
	{0xe1, 1, {0x03}},
	{0xe2, 1, {0x8f}},
	{0xe3, 1, {0x03}},
	{0xe4, 1, {0x9c}},
	{0xe5, 1, {0x03}},
	{0xe6, 1, {0xa2}},
	{0xe7, 1, {0x03}},
	{0xe8, 1, {0xab}},
	{0xe9, 1, {0x03}},
	{0xea, 1, {0xb2}},
	{0xff, 1, {0x05}},
	{0xfb, 1, {0x01}},
	{0xe7, 1, {0x00}},
	{0xff, 1, {0x04}},
	{0xfb, 1, {0x01}},
	{0x08, 1, {0x06}},
	{0xff, 1, {0x00}},
	{0xfb, 1, {0x01}},
	{0xd3, 1, {0x06}},
	{0xd4, 1, {0x16}},
	{0x35, 1, {0x00}},
	{0x51, 1, {0x20}},
	{0x53, 1, {0x24}},
	{0x55, 1, {0x00}},
	{0x11, 0, {}},
	{REGFLAG_DELAY, 120, {}},
	{0x29, 0, {}},
	{REGFLAG_DELAY, 20, {}},
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

int read_boardid(void)
{
	char *result;
	int id = 0;
	result = strstr(saved_command_line, "BoardID=");

	if (result)
	{
		id = result[8] - '0';
		return ((id > 9) ? 0 : id);
    	}
    return id;
}

static void lcm_get_params(LCM_PARAMS *params)
{
	memset(params, 0, sizeof(LCM_PARAMS));

	params->module = "BOE";
	params->vendor = "BOE";
	params->ic = "nt35532";
	params->physical_width = 68;
	params->dsi.mode = SYNC_PULSE_VDO_MODE;
	params->physical_height = 121;
	params->dsi.noncont_clock = 1;
	params->type = LCM_TYPE_DSI;
	params->width = 1080;
	params->height = 1920;
	params->info = "1080*1920";
	params->dsi.LANE_NUM = LCM_FOUR_LANE;
	params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;
	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;
	params->dsi.vertical_backporch = 17;
	params->dsi.vertical_frontporch = 22;
	params->dsi.vertical_sync_active = 2;
	params->dsi.vertical_active_line = 1920;
	params->dsi.horizontal_sync_active = 4;
	params->dsi.horizontal_active_pixel = 1080;
	params->dsi.customization_esd_check_enable = 0;
	params->dsi.ssc_disable = 1;
	params->dsi.esd_check_enable = 1;

	if (read_boardid() == 3)
	{
		printk("boardid  is 3\n");
		params->dsi.horizontal_backporch = 30;
		params->dsi.horizontal_frontporch = 90;
		params->dsi.PLL_CLOCK = 448;
	}
	else
	{
		params->dsi.horizontal_backporch = 95;
		params->dsi.horizontal_frontporch = 95;
		params->dsi.PLL_CLOCK = 481;
	}
}

static void tps65132_enable(void)
{
	int ret = 0;
	int ret1 = 0;
	int ret2 = 0;
	int num = 0;

	mt_set_gpio_mode(GPIO_LCD_BIAS_ENN_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_BIAS_ENN_PIN, GPIO_DIR_OUT);
	ret1 = mt_set_gpio_out(GPIO_LCD_BIAS_ENN_PIN, GPIO_OUT_ONE);
	MDELAY(12);

	mt_set_gpio_mode(GPIO_LCD_BIAS_ENP_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_BIAS_ENP_PIN, GPIO_DIR_OUT);
	ret2 = mt_set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ONE);

	printk("tps65132_enable, ret1 =%d, ret2 =%d\n", ret1, ret2);

	for(num = 0; num < 3; num++)
	{
		ret = tps65132_write_bytes(0x00,0x0f);
		if(ret < 0)
		{
			printk("nt35532--boe--tps65132_enable-cmd=0x00-- i2c write error-num=%d\n", num);
			MDELAY(5);
		}
		else
		{
			printk("nt35532--boe--tps65132_enable-cmd=0x00-- i2c write success-num=%d\n", num);
			break;
		}
	}

	for(num = 0; num < 3; num++)
	{
		ret = tps65132_write_bytes(0x01,0x0f);
		if(ret < 0)
		{
			printk("nt35532--boe--tps65132_enable-cmd=0x01-- i2c write error-num=%d\n", num);
			MDELAY(5);
		}
		else
		{
			printk("nt35532--boe--tps65132_enable-cmd=0x01-- i2c write success-num=%d\n", num);
			break;
		}
	}
}


static void lcm_init(void)
{
	tps65132_enable();
	MDELAY(10);
	mt_set_gpio_mode(GPIO_LCD_RESET_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_RESET_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCD_RESET_PIN, GPIO_OUT_ONE);
	MDELAY(10);
	mt_set_gpio_out(GPIO_LCD_RESET_PIN, GPIO_OUT_ZERO);
	MDELAY(20);
	mt_set_gpio_out(GPIO_LCD_RESET_PIN, GPIO_OUT_ONE);
	MDELAY(20);

	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1); // FF 00 00 00 01 01
}

static void lcm_suspend(void)
{
	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
	printk("[KERNEL]nt35532--boe--tps65132_enable-----sleep--\n");

	mt_set_gpio_mode(GPIO_LCD_BIAS_ENN_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_BIAS_ENN_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCD_BIAS_ENN_PIN, GPIO_OUT_ZERO); // 0xFFFFFFC001715B68
	MDELAY(12);
	mt_set_gpio_mode(GPIO_LCD_BIAS_ENP_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_BIAS_ENP_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ZERO); // 0xFFFFFFC001715AA8
	MDELAY(10);
	mt_set_gpio_mode(GPIO_LCD_RESET_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_RESET_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCD_RESET_PIN, GPIO_OUT_ZERO);
}

static void lcm_resume(void)
{
	static unsigned int backlight_array_num = 0;
	static unsigned int lcm_initialization_count = 0;

	if(cabc_enable_flag == 0)
	{
		lcm_initialization_count = sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table);
		for(backlight_array_num = lcm_initialization_count - 1; backlight_array_num >= 0; backlight_array_num--)
		{
			if(0x51 == lcm_initialization_setting[backlight_array_num].cmd)
			{
				break;
			}
		}
	}
	else
	{
		lcm_initialization_count = sizeof(lcm_cabc_on_initialization_setting) / sizeof(struct LCM_setting_table);
		for(backlight_array_num = lcm_initialization_count - 1; backlight_array_num >= 0; backlight_array_num--)
		{
			if(0x51 == lcm_cabc_on_initialization_setting[backlight_array_num].cmd)
			{
				break;
			}
		}
	}

	printk("lcm_resume, lcm_initialization_count=%d, backlight_array_num=%d\n", lcm_initialization_count, backlight_array_num);
	tps65132_enable();

	MDELAY(15);
	mt_set_gpio_mode(GPIO_LCD_RESET_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_RESET_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCD_RESET_PIN, GPIO_OUT_ONE);
	MDELAY(5);
	mt_set_gpio_out(GPIO_LCD_RESET_PIN, GPIO_OUT_ZERO);
	MDELAY(5);
	mt_set_gpio_out(GPIO_LCD_RESET_PIN, GPIO_OUT_ONE);
	MDELAY(5);
	mt_set_gpio_out(GPIO_LCD_RESET_PIN, GPIO_OUT_ZERO);
	MDELAY(5);
	mt_set_gpio_out(GPIO_LCD_RESET_PIN, GPIO_OUT_ONE);
	MDELAY(20);

	if(cabc_enable_flag == 0) {
		if(last_backlight_level <= 32) {
			if(backlight_array_num != 0) {
				lcm_initialization_setting[backlight_array_num].para_list[0] = last_backlight_level;
			}
		} else {
			if(backlight_array_num != 0) {
				lcm_initialization_setting[backlight_array_num].para_list[0] = 32;
			}
		}
		push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
	} else {
		if(last_backlight_level <= 32) {
			if(backlight_array_num != 0) {
				lcm_cabc_on_initialization_setting[backlight_array_num].para_list[0] = last_backlight_level;
			}
		} else {
			if(backlight_array_num != 0) {
				lcm_cabc_on_initialization_setting[backlight_array_num].para_list[0] = 32;
			}
		}
		push_table(lcm_cabc_on_initialization_setting, sizeof(lcm_cabc_on_initialization_setting) / sizeof(struct LCM_setting_table), 1);
	}
}
static unsigned int lcm_compare_id(void) {

	unsigned char buffer[8];
	unsigned int array[16];
	unsigned int id = 0;

	mt_set_gpio_mode(GPIO_LCD_RESET_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_RESET_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCD_RESET_PIN, GPIO_OUT_ONE);
	MDELAY(50);
	mt_set_gpio_out(GPIO_LCD_RESET_PIN, GPIO_OUT_ZERO);
	MDELAY(20);
	mt_set_gpio_out(GPIO_LCD_RESET_PIN, GPIO_OUT_ONE);
	MDELAY(20);


	array[0] = 0x00023700;
	dsi_set_cmdq(array, 1, 1);

	array[0] = 0x00ff1500;
	dsi_set_cmdq(array, 1, 1);

	array[0] = 0x01fb1500;
	dsi_set_cmdq(array, 1, 1);
	MDELAY(10);
	read_reg_v2(0xF4, buffer, 1);
	MDELAY(20);

	id = buffer[0];

	return (LCM_ID == id) ? 1 : 0;
}

static void lcm_cabc_enable_cmdq(void* handle,unsigned int enable)
{
	if(enable == 0)
	{
		push_table(lcm_cabc_off_setting, sizeof(lcm_cabc_off_setting) / sizeof(struct LCM_setting_table), 1);
	}
	else
	{
		push_table(lcm_cabc_on_setting, sizeof(lcm_cabc_on_setting) / sizeof(struct LCM_setting_table), 1);
	}
}

static void lcm_setbacklight_cmdq(void* handle,unsigned int level)
{
	unsigned int mapped_level = 0;

	mapped_level = level;

	if ( mapped_level )
	{
		if ( mapped_level > 0xFF )
		mapped_level = 255;

		last_backlight_level = mapped_level;
		mt_set_gpio_mode(GPIO_LCD_BL_EN_PIN, GPIO_MODE_00);
		mt_set_gpio_dir(GPIO_LCD_BL_EN_PIN, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_LCD_BL_EN_PIN, GPIO_OUT_ONE);
	}
	else
	{
		mt_set_gpio_mode(GPIO_LCD_BL_EN_PIN, GPIO_MODE_00);
		mt_set_gpio_dir(GPIO_LCD_BL_EN_PIN, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_LCD_BL_EN_PIN, GPIO_OUT_ZERO);
		MDELAY(30);
	}

	lcm_backlight_level_setting[0].para_list[0] = mapped_level;
	push_table(lcm_backlight_level_setting, sizeof(lcm_backlight_level_setting) / sizeof(struct LCM_setting_table), 1);
}

// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------

LCM_DRIVER nt35532_fhd_boe_vdo_lcm_drv =
{
	.name           	= "nt35532_fhd_boe_vdo_lcm",
	.set_util_funcs 	= lcm_set_util_funcs,
	.get_params     	= lcm_get_params,
 	.init           	= lcm_init,
	.suspend       	 	= lcm_suspend,
	.resume			= lcm_resume,
	.compare_id		= lcm_compare_id,
	.set_backlight_cmdq	= lcm_setbacklight_cmdq,
	.enable_cabc_cmdq	= lcm_cabc_enable_cmdq,
};
