
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

/*********************************************************
* Gate Driver
*********************************************************/

#ifndef BUILD_LK
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>

/*****************************************************************************
 * Define
 *****************************************************************************/
#ifndef CONFIG_FPGA_EARLY_PORTING

#ifdef CONFIG_MTK_LEGACY
#define TPS_I2C_BUSNUM  I2C_I2C_LCD_BIAS_CHANNEL/*for I2C channel 0*/
#endif
#define I2C_ID_NAME "tps65132"
#define TPS_ADDR 0x3E

/*****************************************************************************
 * GLobal Variable
 *****************************************************************************/
#ifdef CONFIG_MTK_LEGACY
static struct i2c_board_info tps65132_board_info  __initdata = {I2C_BOARD_INFO(I2C_ID_NAME, TPS_ADDR)};
#endif

#if !defined(CONFIG_MTK_LEGACY)
static const struct of_device_id lcm_of_match[] = {
		{ .compatible = "mediatek,i2c_lcd_bias" },
		{},
};
#endif

static struct i2c_client *tps65132_i2c_client;

/*****************************************************************************
 * Function Prototype
 *****************************************************************************/
static int tps65132_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tps65132_remove(struct i2c_client *client);

/*****************************************************************************
 * Data Structure
 *****************************************************************************/

struct tps65132_dev	{
	struct i2c_client	*client;

};

static const struct i2c_device_id tps65132_id[] = {
	{ I2C_ID_NAME, 0 },
	{ }
};

static struct i2c_driver tps65132_iic_driver = {
	.id_table	= tps65132_id,
	.probe		= tps65132_probe,
	.remove		= tps65132_remove,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "tps65132",
#if !defined(CONFIG_MTK_LEGACY)
		.of_match_table = lcm_of_match,
#endif
	},

};

/*****************************************************************************
 * Function
 *****************************************************************************/
static int tps65132_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	pr_debug("tps65132_iic_probe\n");
	pr_debug("TPS: info==>name=%s addr=0x%x\n", client->name, client->addr);
	tps65132_i2c_client  = client;
	return 0;
}

static int tps65132_remove(struct i2c_client *client)
{
	pr_debug("tps65132_remove\n");
	tps65132_i2c_client = NULL;
	i2c_unregister_device(client);
	return 0;
}

static int tps65132_write_bytes(unsigned char addr, unsigned char value)
{
	int ret = 0;
	struct i2c_client *client = tps65132_i2c_client;
	char write_data[2] = {0};

	write_data[0] = addr;
	write_data[1] = value;
	ret = i2c_master_send(client, write_data, 2);
	if (ret < 0)
		printk("tps65132 write data fail !!\n");
		return ret;
}

/*
 * module load/unload record keeping
 */

static int __init tps65132_iic_init(void)
{

	pr_debug("tps65132_iic_init\n");
#ifdef CONFIG_MTK_LEGACY
	i2c_register_board_info(TPS_I2C_BUSNUM, &tps65132_board_info, 1);
	pr_debug("tps65132_iic_init2\n");
#endif
	i2c_add_driver(&tps65132_iic_driver);
	pr_debug("tps65132_iic_init success\n");
	return 0;
}

static void __exit tps65132_iic_exit(void)
{
	pr_debug("tps65132_iic_exit\n");
	i2c_del_driver(&tps65132_iic_driver);
}


module_init(tps65132_iic_init);
module_exit(tps65132_iic_exit);

MODULE_AUTHOR("Xiaokuan Shi");
MODULE_DESCRIPTION("MTK TPS65132 I2C Driver");
MODULE_LICENSE("GPL");

#endif
#endif

/*********************************************************
* LCM  Driver
*********************************************************/

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH					(1080)
#define FRAME_HEIGHT					(1920)

#define REGFLAG_DELAY             			(0xFC)
#define REGFLAG_END_OF_TABLE      			(0xFD)

#define LCM_ID						(0x32)

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

extern char* saved_command_line;
extern int cabc_enable_flag;
extern int lct_read_boardid(void);

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

#define set_gpio_lcd_enp(cmd)	lcm_util.set_gpio_lcd_enp_bias(cmd)
#define set_gpio_lcd_enn(cmd)	lcm_util.set_gpio_lcd_enn_bias(cmd)
#define set_gpio_lcd_bl(cmd)	lcm_util.set_gpio_lcd_backlight_en(cmd)
#define set_gpio_lcd_pwr(cmd)	lcm_util.set_gpio_lcd_pwr_en(cmd)

struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};

static struct LCM_setting_table lcm_initialization_setting[] = 
{
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
	{0X8C, 1, {0XFC}},
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
	{0X51, 1, {0X06}},
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

static struct LCM_setting_table lcm_cabc_on_setting[] =
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

static struct LCM_setting_table lcm_cabc_on_initialization_setting[] = 
{
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
	{0X8C, 1, {0XFC}},
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
	{0XCA, 1, {0XFC}},
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
	{0X51, 1, {0X06}},
	{0X53, 1, {0X24}},
	{0X55, 1, {0X01}},
	{0XD3, 1, {0X12}},
	{0XD4, 1, {0X16}},
	{0X35, 1, {0X00}},
	{0X11, 0, {}},
	{REGFLAG_DELAY, 120, {}},
	{0X29, 0, {}},
	{REGFLAG_DELAY, 20, {}},
	{REGFLAG_END_OF_TABLE, 0X00, {}}
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
	//temp params
	params->dsi.horizontal_backporch = 30;
	params->dsi.horizontal_frontporch = 90;
	params->dsi.PLL_CLOCK = 448;
	
	printk("[READ_BOARDID] BoardID = %d", lct_read_boardid());
}

static void tps65132_enable(void)
{
	int ret = 0;
	int ret1 = 0;
	int ret2 = 0;
	int num = 0;
			
	ret1 = set_gpio_lcd_enn(1);
	MDELAY(12);
	ret2 = set_gpio_lcd_enp(1);

	printk("tps65132_enable, ret1 =%d, ret2 =%d\n", ret1, ret2);
	for(num = 0; num < 3; num++) {	
		ret = tps65132_write_bytes(0x00, 0x0f);
		if(ret < 0) {
			printk("nt35532--boe--tps65132_enable-cmd=0x00-- i2c write error-num=%d\n", num);
			MDELAY(5);
		} else {
			printk("nt35532--boe--tps65132_enable-cmd=0x00-- i2c write success-num=%d\n", num);
			break;
		}
	}

	for(num = 0; num < 3; num++) {	
		ret = tps65132_write_bytes(0x01, 0x0f);
		if(ret < 0) {
			printk("nt35532--boe--tps65132_enable-cmd=0x01-- i2c write error-num=%d\n", num);
			MDELAY(5);
		} else {
			printk("nt35532--boe--tps65132_enable-cmd=0x01-- i2c write success-num=%d\n", num);
			break;
		}
	}
}

static void lcm_init(void)
{
	tps65132_enable();
	
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(20);
	SET_RESET_PIN(1);
	MDELAY(20);

	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_suspend(void)
{
	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
	
	printk("[KERNEL]nt35532--boe--tps65132_enable-----sleep--\n");
	
	set_gpio_lcd_enn(0);
	MDELAY(12);
	set_gpio_lcd_enp(0);
	MDELAY(10);
	SET_RESET_PIN(0);
}

static void lcm_resume(void)
{
	static unsigned int backlight_array_num = 0;
	static unsigned int lcm_initialization_count = 0;

	if(cabc_enable_flag == 0) {
		lcm_initialization_count = sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table);
		for(backlight_array_num = lcm_initialization_count - 1; backlight_array_num >= 0; backlight_array_num--) {
			if(0x51 == lcm_initialization_setting[backlight_array_num].cmd) {
				break;
			}
		}
	} else {
		lcm_initialization_count = sizeof(lcm_cabc_on_initialization_setting) / sizeof(struct LCM_setting_table);
		for(backlight_array_num = lcm_initialization_count - 1; backlight_array_num >= 0; backlight_array_num--) {
			if(0x51 == lcm_cabc_on_initialization_setting[backlight_array_num].cmd) {
				break;
			}
		}
	}

	printk("lcm_resume, lcm_initialization_count=%d, backlight_array_num=%d\n", lcm_initialization_count, backlight_array_num);
	tps65132_enable();

	MDELAY(15);
	SET_RESET_PIN(1);
	MDELAY(5);
	SET_RESET_PIN(0);
	MDELAY(5);
	SET_RESET_PIN(1);
	MDELAY(5);
	SET_RESET_PIN(0);
	MDELAY(5);
	SET_RESET_PIN(1);
	MDELAY(20);

    if (cabc_enable_flag == 0) {
	    if (last_backlight_level > 32) {
		    if (backlight_array_num != 0) {
			    lcm_initialization_setting[backlight_array_num].para_list[0] = 32;
			    push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
		    }
	    } else if (backlight_array_num) {
		    lcm_initialization_setting[backlight_array_num].para_list[0] = last_backlight_level;
	    }
	    push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
    } else {
	    if (last_backlight_level > 32) {
		    if (backlight_array_num != 0)
			    lcm_cabc_on_initialization_setting[backlight_array_num].para_list[0] = 32;
	    } else if (backlight_array_num != 0) {
		    lcm_cabc_on_initialization_setting[backlight_array_num].para_list[0] = last_backlight_level;
	    }
	    push_table(lcm_cabc_on_initialization_setting, sizeof(lcm_cabc_on_initialization_setting) / sizeof(struct LCM_setting_table), 1);
    }
}
static unsigned int lcm_compare_id(void) {

	unsigned char buffer[8];
	unsigned int array[16];
	unsigned int id = 0;

	SET_RESET_PIN(1);
	MDELAY(50);
	SET_RESET_PIN(0);
	MDELAY(20);
	SET_RESET_PIN(1);
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

	return LCM_ID == id;
}

static void lcm_cabc_enable_cmdq(void* handle,unsigned int enable)
{
	if(enable == 0) {
		push_table(lcm_cabc_off_setting, sizeof(lcm_cabc_off_setting) / sizeof(struct LCM_setting_table), 1);
	} else {
		push_table(lcm_cabc_on_setting, sizeof(lcm_cabc_on_setting) / sizeof(struct LCM_setting_table), 1);
	}
}

static void lcm_setbacklight_cmdq(void* handle, unsigned int level)
{
	unsigned int mapped_level = 0;

	mapped_level = level;
	
	if (mapped_level)
	{
		if (mapped_level > 255)
			mapped_level = 255;

		last_backlight_level = mapped_level;
		set_gpio_lcd_bl(1);
	}
	else
	{
		set_gpio_lcd_bl(0);
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
