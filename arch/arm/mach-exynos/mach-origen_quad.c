/*
 * linux/arch/arm/mach-exynos4/mach-origen.c
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/gpio.h>
#include <linux/gpio_event.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/unidisplay_ts.h>
#include <linux/io.h>
#include <linux/mfd/s5m87xx/s5m-pmic.h>
#include <linux/mfd/s5m87xx/s5m-core.h>
#include <linux/mmc/host.h>
#include <linux/platform_data/s3c-hsotg.h>
#include <linux/platform_device.h>
#include <linux/pwm_backlight.h>
#include <linux/regulator/machine.h>
#include <linux/serial_core.h>

#include <video/platform_lcd.h>

#include <asm/hardware/gic.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <plat/backlight.h>
#include <plat/clock.h>
#include <plat/cpu.h>
#include <plat/devs.h>
#include <plat/ehci.h>
#include <plat/fb.h>
#include <plat/fimg2d.h>
#include <plat/gpio-cfg.h>
#include <plat/hdmi.h>
#include <plat/iic.h>
#include <plat/keypad.h>
#include <plat/mfc.h>
#include <plat/pm.h>
#include <plat/regs-fb-v4.h>
#include <plat/regs-serial.h>
#include <plat/sdhci.h>

#include <mach/dev.h>
#include <mach/dwmci.h>
#include <mach/exynos-ion.h>
#include <mach/map.h>
#include <mach/ohci.h>
#include <mach/ppmu.h>
#include <mach/regs-pmu.h>

#include "common.h"

/* Following are default values for UCON, ULCON and UFCON UART registers */
#define ORIGEN_UCON_DEFAULT	(S3C2410_UCON_TXILEVEL | \
				 S3C2410_UCON_RXILEVEL | \
				 S3C2410_UCON_TXIRQMODE | \
				 S3C2410_UCON_RXIRQMODE | \
				 S3C2410_UCON_RXFIFO_TOI | \
				 S3C2443_UCON_RXERR_IRQEN)

#define ORIGEN_ULCON_DEFAULT	S3C2410_LCON_CS8

#define ORIGEN_UFCON_DEFAULT	(S3C2410_UFCON_FIFOMODE | \
				 S5PV210_UFCON_TXTRIG4 | \
				 S5PV210_UFCON_RXTRIG4)


static struct s3c2410_uartcfg origen_uartcfgs[] __initdata = {
	[0] = {
		.hwport		= 0,
		.flags		= 0,
		.ucon		= ORIGEN_UCON_DEFAULT,
		.ulcon		= ORIGEN_ULCON_DEFAULT,
		.ufcon		= ORIGEN_UFCON_DEFAULT,
	},
	[1] = {
		.hwport		= 1,
		.flags		= 0,
		.ucon		= ORIGEN_UCON_DEFAULT,
		.ulcon		= ORIGEN_ULCON_DEFAULT,
		.ufcon		= ORIGEN_UFCON_DEFAULT,
	},
	[2] = {
		.hwport		= 2,
		.flags		= 0,
		.ucon		= ORIGEN_UCON_DEFAULT,
		.ulcon		= ORIGEN_ULCON_DEFAULT,
		.ufcon		= ORIGEN_UFCON_DEFAULT,
	},
	[3] = {
		.hwport		= 3,
		.flags		= 0,
		.ucon		= ORIGEN_UCON_DEFAULT,
		.ulcon		= ORIGEN_ULCON_DEFAULT,
		.ufcon		= ORIGEN_UFCON_DEFAULT,
	},
};

static struct s3c_sdhci_platdata origen_hsmmc2_pdata __initdata = {
	.cd_type		= S3C_SDHCI_CD_GPIO,
	.ext_cd_gpio		= EXYNOS4_GPK2(2),
	.ext_cd_gpio_invert	= 1,
#ifdef CONFIG_EXYNOS4_SDHCI_CH2_8BIT
	.max_width		= 8,
	.host_caps		= MMC_CAP_8_BIT_DATA,
#endif
};

static struct regulator_consumer_supply s5m8767_buck1_consumer[] = {
	REGULATOR_SUPPLY("vdd_mif", NULL),
};

static struct regulator_consumer_supply s5m8767_buck2_consumer[] = {
	REGULATOR_SUPPLY("vdd_arm", NULL),
};

static struct regulator_consumer_supply s5m8767_buck3_consumer[] = {
	REGULATOR_SUPPLY("vdd_int", NULL),
};

static struct regulator_consumer_supply s5m8767_buck4_consumer[] = {
	REGULATOR_SUPPLY("vdd_g3d", NULL),
};

static struct regulator_consumer_supply s5m8767_ldo8_consumer[] = {
	REGULATOR_SUPPLY("vdd", "exynos4-hdmi"),
	REGULATOR_SUPPLY("vdd_pll", "exynos4-hdmi"),
	REGULATOR_SUPPLY("vdd8_mipi", NULL),
};

static struct regulator_consumer_supply s5m8767_ldo9_consumer[] = {
	REGULATOR_SUPPLY("vdd_lcd", NULL),
};

static struct regulator_consumer_supply s5m8767_ldo10_consumer[] = {
	REGULATOR_SUPPLY("vdd_osc", "exynos4-hdmi"),
	REGULATOR_SUPPLY("vdd10_mipi", NULL),
	REGULATOR_SUPPLY("vdd_tmu", NULL),
};

static struct regulator_consumer_supply s5m8767_ldo12_consumer[] = {
	REGULATOR_SUPPLY("vusb_a", NULL),
};

static struct regulator_consumer_supply s5m8767_ldo15_consumer[] = {
	REGULATOR_SUPPLY("vusb_d", NULL),
};

static struct regulator_consumer_supply s5m8767_ldo18_consumer[] = {
	REGULATOR_SUPPLY("vdd_uhost", NULL),
};

static struct regulator_init_data s5m8767_buck1_data = {
	.constraints		= {
		.name		= "vdd_mif range",
		.min_uV		=  800000,
		.max_uV		= 1100000,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE |
				  REGULATOR_CHANGE_STATUS,
		.always_on	= 1,
		.state_mem	= {
			.enabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s5m8767_buck1_consumer[0],
};

static struct regulator_init_data s5m8767_buck2_data = {
	.constraints		= {
		.name		= "vdd_arm range",
		.min_uV		=  800000,
		.max_uV		= 1350000,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE |
				  REGULATOR_CHANGE_STATUS,
		.boot_on	= 1,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s5m8767_buck2_consumer[0],
};

static struct regulator_init_data s5m8767_buck3_data = {
	.constraints		= {
		.name		= "vdd_int range",
		.min_uV		=  800000,
		.max_uV		= 1150000,
		.apply_uV	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE |
				  REGULATOR_CHANGE_STATUS,
		.always_on	= 1,
		.state_mem	= {
			.uV		= 1100000,
			.mode		= REGULATOR_MODE_NORMAL,
			.enabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s5m8767_buck3_consumer[0],
};

static struct regulator_init_data s5m8767_buck4_data = {
	.constraints		= {
		.name		= "vdd_g3d range",
		.min_uV		=  850000,
		.max_uV		= 1200000,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE |
				REGULATOR_CHANGE_STATUS,
		.boot_on	= 1,
		.state_mem	= {
			.disabled	= 1,
			},
		},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s5m8767_buck4_consumer[0],
};

static struct regulator_init_data s5m8767_ldo8_data = {
	.constraints		= {
		.name		= "vdd_ldo8 range",
		.min_uV		= 1000000,
		.max_uV         = 1000000,
		.boot_on        = 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem      = {
			.disabled       = 1,
			.mode  	        = REGULATOR_MODE_STANDBY,
		},
		.initial_state = PM_SUSPEND_MEM,
	},
	.num_consumer_supplies  = ARRAY_SIZE(s5m8767_ldo8_consumer),
	.consumer_supplies      = s5m8767_ldo8_consumer,
};

static struct regulator_init_data s5m8767_ldo9_data = {
	.constraints		= {
		.name		= "vdd_lcd fixed",
		.min_uV		= 3300000,
		.max_uV		= 3300000,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,
		.boot_on	= 1,
		.state_mem	= {
			.disabled	= 1,
			.mode		= REGULATOR_MODE_STANDBY,
		},
		.initial_state	= PM_SUSPEND_MEM,
	},
	.num_consumer_supplies	= ARRAY_SIZE(s5m8767_ldo9_consumer),
	.consumer_supplies	= &s5m8767_ldo9_consumer[0],
};

static struct regulator_init_data s5m8767_ldo10_data = {
	.constraints = {
		.name           = "vdd_ldo10 range",
		.min_uV         = 1800000,
		.max_uV         = 1800000,
		.boot_on        = 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem      = {
			.disabled       = 1,
			.mode           = REGULATOR_MODE_STANDBY,
		},
		.initial_state = PM_SUSPEND_MEM,
	},
	.num_consumer_supplies  = ARRAY_SIZE(s5m8767_ldo10_consumer),
	.consumer_supplies      = s5m8767_ldo10_consumer,
};

static struct regulator_init_data s5m8767_ldo12_data = {
	.constraints = {
		.name           = "vdd_ldo12 range",
		.min_uV         = 3000000,
		.max_uV         = 3300000,
		.boot_on        = 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_VOLTAGE,
		.always_on      = 1,
		.state_mem      = {
			.disabled       = 1,
			.mode           = REGULATOR_MODE_STANDBY,
		},
	.initial_state = PM_SUSPEND_MEM,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &s5m8767_ldo12_consumer[0],
};

static struct regulator_init_data s5m8767_ldo15_data = {
	.constraints = {
		.name           = "vdd_ldo15 range",
		.min_uV         = 1000000,
		.max_uV         = 1000000,
		.boot_on        = 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.always_on      = 1,
		.state_mem      = {
			.disabled       = 1,
			.mode           = REGULATOR_MODE_STANDBY,
		},
		.initial_state = PM_SUSPEND_MEM,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &s5m8767_ldo15_consumer[0],
};

static struct regulator_init_data s5m8767_ldo18_data = {
	.constraints = {
		.name           = "vddioperi_28",
		.min_uV         = 2800000,
		.max_uV         = 2800000,
		.boot_on        = 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem      = {
			.disabled       = 1,
			.mode           = REGULATOR_MODE_STANDBY,
		},
		.initial_state = PM_SUSPEND_MEM,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &s5m8767_ldo18_consumer[0],
};

static struct s5m_regulator_data pegasus_regulators[] = {
	{ S5M8767_BUCK1, &s5m8767_buck1_data },
	{ S5M8767_BUCK2, &s5m8767_buck2_data },
	{ S5M8767_BUCK3, &s5m8767_buck3_data },
	{ S5M8767_BUCK4, &s5m8767_buck4_data },
	{ S5M8767_LDO8,	 &s5m8767_ldo8_data },
	{ S5M8767_LDO9,	 &s5m8767_ldo9_data },
	{ S5M8767_LDO10, &s5m8767_ldo10_data },
	{ S5M8767_LDO12, &s5m8767_ldo12_data },
	{ S5M8767_LDO15, &s5m8767_ldo15_data },
	{ S5M8767_LDO18, &s5m8767_ldo18_data },
};

struct s5m_opmode_data s5m8767_opmode_data[S5M8767_REG_MAX] = {
	[S5M8767_BUCK1] = { S5M8767_BUCK1, S5M_OPMODE_SUSPEND },
	[S5M8767_BUCK2] = { S5M8767_BUCK2, S5M_OPMODE_SUSPEND },
	[S5M8767_BUCK3] = { S5M8767_BUCK3, S5M_OPMODE_SUSPEND },
	[S5M8767_BUCK4] = { S5M8767_BUCK4, S5M_OPMODE_SUSPEND },
	[S5M8767_LDO8]	= { S5M8767_LDO8,  S5M_OPMODE_SUSPEND },
	[S5M8767_LDO9]	= { S5M8767_LDO9,  S5M_OPMODE_SUSPEND },
	[S5M8767_LDO10]	= { S5M8767_LDO10, S5M_OPMODE_SUSPEND },
	[S5M8767_LDO12] = { S5M8767_LDO12, S5M_OPMODE_SUSPEND },
	[S5M8767_LDO15] = { S5M8767_LDO15, S5M_OPMODE_SUSPEND },
	[S5M8767_LDO18] = { S5M8767_LDO18, S5M_OPMODE_SUSPEND },
};

static int s5m_cfg_irq(void)
{
	/* AP_PMIC_IRQ: EINT26 */
	s3c_gpio_cfgpin(EXYNOS4_GPX2(6), S3C_GPIO_SFN(0xF));
	s3c_gpio_setpull(EXYNOS4_GPX2(6), S3C_GPIO_PULL_UP);
	return 0;
}

static struct s5m_platform_data origen_s5m8767_pdata = {
	.device_type		= S5M8767X,
	.irq_base		= IRQ_BOARD_START,
	.num_regulators		= ARRAY_SIZE(pegasus_regulators),
	.regulators		= pegasus_regulators,
	.cfg_pmic_irq		= s5m_cfg_irq,
	.wakeup			= 1,
	.opmode			= s5m8767_opmode_data,

	.buck_default_idx	= 1,
	.buck_gpios[0]		= EXYNOS4_GPX2(3),
	.buck_gpios[1]		= EXYNOS4_GPX2(4),
	.buck_gpios[2]		= EXYNOS4_GPX2(5),

	.buck_ramp_delay	= 25,
	.buck2_ramp_enable	= true,
	.buck3_ramp_enable	= true,
	.buck4_ramp_enable	= true,
};

static struct gpio_event_direct_entry origen_keypad_keymap[] = {
	{
		.gpio	= EXYNOS4_GPX2(7),
		.code	= KEY_POWER,
	}
};

static struct gpio_event_input_info origen_keypad_input_info = {
	.info.func		= gpio_event_input_func,
	.info.no_suspend	= true,
	.debounce_time.tv64	= 20 * NSEC_PER_MSEC,
	.type			= EV_KEY,
	.keymap			= origen_keypad_keymap,
	.keymap_size		= ARRAY_SIZE(origen_keypad_keymap)
};

static struct gpio_event_info *origen_keypad_info[] = {
	&origen_keypad_input_info.info,
};

static struct gpio_event_platform_data origen_keypad_pdata = {
	.names	= {
		"origen-keypad",
		NULL,
	},
	.info		= origen_keypad_info,
	.info_count	= ARRAY_SIZE(origen_keypad_info),
};

static struct platform_device origen_device_keypad = {
	.name	= GPIO_EVENT_DEV_NAME,
	.id	= 0,
	.dev	= {
		.platform_data = &origen_keypad_pdata,
	},
};

static uint32_t origen_keymap[] __initdata = {
	/* KEY(row, col, keycode) */
	KEY(0, 0, KEY_HOME), KEY(0, 1, KEY_DOWN),
	KEY(1, 0, KEY_UP),   KEY(1, 1, KEY_MENU),
	KEY(2, 0, KEY_BACK), KEY(2, 1, KEY_SEARCH)
};

static struct matrix_keymap_data origen_keymap_data __initdata = {
	.keymap	 = origen_keymap,
	.keymap_size	= ARRAY_SIZE(origen_keymap),
};

static struct samsung_keypad_platdata origen_keypad_data __initdata = {
	.keymap_data	= &origen_keymap_data,
	.rows		= 3,
	.cols		= 2,
};

#define TOUCH_INT_PIN		EXYNOS4_GPX3(1)
#define TOUCH_INT_PIN_SHIFT	1
#define TOUCH_RST_PIN		EXYNOS4X12_GPM3(3)

static int unidisplay_ts_init(void)
{
	if (gpio_request(TOUCH_INT_PIN, "TOUCH_INT_PIN")) {
		pr_err("%s : gpio request failed.\n", __func__);
		return 1;
	}
	s3c_gpio_cfgpin(TOUCH_INT_PIN, S3C_GPIO_SFN(0x0F));
	s3c_gpio_setpull(TOUCH_INT_PIN, S3C_GPIO_PULL_UP);
	gpio_direction_input(TOUCH_INT_PIN);
	gpio_free(TOUCH_INT_PIN);

	if (gpio_request(TOUCH_RST_PIN, "TOUCH_RST_PIN")) {
		pr_err("%s : gpio request failed.\n", __func__);
		return 1;
	}
	s3c_gpio_setpull(TOUCH_RST_PIN, S3C_GPIO_PULL_NONE);
	s3c_gpio_cfgpin(TOUCH_RST_PIN, S3C_GPIO_OUTPUT);
	gpio_direction_output(TOUCH_RST_PIN, 1);
	gpio_free(TOUCH_RST_PIN);
	return 0;
}

static int unidisplay_ts_reset(void)
{
	if (gpio_request(TOUCH_RST_PIN, "TOUCH_RST_PIN")) {
		pr_err("%s : gpio request failed.\n", __func__);
		return 1;
	}
	gpio_set_value(TOUCH_RST_PIN, 0);
	udelay(100);
	gpio_set_value(TOUCH_RST_PIN, 1);
	gpio_free(TOUCH_RST_PIN);
	return 0;
}

static int unidisplay_ts_pin_state(int irq)
{
	int ret = 1;
	if (gpio_request(TOUCH_INT_PIN, "TOUCH_INT_PIN") == 0) {
		ret = gpio_get_value(TOUCH_INT_PIN);
		gpio_free(TOUCH_INT_PIN);
	}
	return ret;
}

struct unidisplay_ts_platform_data origen_unidisplay_ts_pdata = {
	.init			= unidisplay_ts_init,
	.reset			= unidisplay_ts_reset,
	.pin_state		= unidisplay_ts_pin_state,
};

static struct samsung_bl_gpio_info origen_bl_gpio_info = {
	.no	= EXYNOS4_GPD0(1),
	.func	= S3C_GPIO_SFN(2),
};

static struct platform_pwm_backlight_data origen_bl_data = {
	.pwm_id		= 1,
	.pwm_period_ns	= 1000,
};

static struct s3c_fb_pd_win origen_fb_win0 = {
	.xres			= 1024,
	.yres			= 600,
        .virtual_x              = 1024,
        .virtual_y              = 600 * CONFIG_FB_S3C_NR_BUFFERS,
        .max_bpp                = 32,
        .default_bpp            = 24,
	.width			= 154,
	.height 		= 90,
};

static struct s3c_fb_pd_win origen_fb_win1 = {
	.xres			= 1024,
	.yres			= 600,
        .virtual_x              = 1024,
        .virtual_y              = 600 * CONFIG_FB_S3C_NR_BUFFERS,
        .max_bpp                = 32,
        .default_bpp            = 24,
	.width			= 154,
	.height 		= 90,
};

static struct s3c_fb_pd_win origen_fb_win2 = {
	.xres			= 1024,
	.yres			= 600,
        .virtual_x              = 1024,
        .virtual_y              = 600 * CONFIG_FB_S3C_NR_BUFFERS,
        .max_bpp                = 32,
        .default_bpp            = 24,
	.width			= 154,
	.height 		= 90,
};

static struct s3c_fb_pd_win origen_fb_win3 = {
	.xres			= 1024,
	.yres			= 600,
        .virtual_x              = 1024,
        .virtual_y              = 600 * CONFIG_FB_S3C_NR_BUFFERS,
        .max_bpp                = 32,
        .default_bpp            = 24,
	.width			= 154,
	.height 		= 90,
};

static struct s3c_fb_pd_win origen_fb_win4 = {
	.xres			= 1024,
	.yres			= 600,
        .virtual_x              = 1024,
        .virtual_y              = 600 * CONFIG_FB_S3C_NR_BUFFERS,
        .max_bpp                = 32,
        .default_bpp            = 24,
	.width			= 154,
	.height 		= 90,
};

static struct fb_videomode origen_lcd_timing = {
	.left_margin	= 213,
	.right_margin	= 105,
	.upper_margin	= 23,
	.lower_margin	= 10,
	.hsync_len	= 2,
	.vsync_len	= 2,
	.xres		= 1024,
	.yres		= 600,
};

static struct s3c_fb_platdata origen_lcd0_pdata __initdata = {
	.win[0]         = &origen_fb_win0,
	.win[1]         = &origen_fb_win1,
	.win[2]         = &origen_fb_win2,
	.win[3]         = &origen_fb_win3,
	.win[4]         = &origen_fb_win4,
	.vtiming        = &origen_lcd_timing,
	.vidcon1        = VIDCON1_INV_VCLK | VIDCON1_INV_HSYNC | VIDCON1_INV_VSYNC,
	.setup_gpio     = exynos4_fimd0_gpio_setup_24bpp,
};

static void origen_lcd_set_power(struct plat_lcd_data *pd, unsigned int power)
{
	int ret;

	if (power)
		ret = gpio_request_one(EXYNOS4X12_GPM3(4),
					GPIOF_OUT_INIT_HIGH, "GPM3_4");
	else
		ret = gpio_request_one(EXYNOS4X12_GPM3(4),
					GPIOF_OUT_INIT_LOW, "GPM3_4");

	gpio_free(EXYNOS4X12_GPM3(4));

	if (ret)
		pr_err("failed to request gpio for LCD power: %d\n", ret);
}

static struct plat_lcd_data origen_lcd_data = {
	.set_power = origen_lcd_set_power,
};

static struct platform_device origen_device_lcd = {
        .name   = "platform-lcd",
        .dev    = {
                .parent         = &s5p_device_fimd0.dev,
                .platform_data  = &origen_lcd_data,
        },
};

static struct i2c_board_info origen_i2c_devs0[] __initdata = {
	{
		I2C_BOARD_INFO("s5m87xx", 0xCC >> 1),
		.platform_data	= &origen_s5m8767_pdata,
		.irq		= IRQ_EINT(26),
	},
};

static struct i2c_board_info origen_i2c_devs1[] __initdata = {
	/* nothing here yet */
};

static struct i2c_board_info origen_i2c_devs2[] __initdata = {
	/* nothing here yet */
};

static struct i2c_board_info origen_i2c_devs3[] __initdata = {
	{
		I2C_BOARD_INFO("unidisplay_ts", 0x41),
		.irq = IRQ_EINT(25),
                .platform_data  = &origen_unidisplay_ts_pdata,
	},
};

static struct i2c_board_info origen_i2c_devs6[] __initdata = {
        {
                I2C_BOARD_INFO("s5p_ddc", (0x74 >> 1)),
        },
};

/* I2C module and id for HDMIPHY */
static struct i2c_board_info origen_i2c_hdmiphy[] __initdata = {
        { I2C_BOARD_INFO("hdmiphy-exynos4412", 0x38), }
};

#ifdef CONFIG_EXYNOS4_DEV_DWMCI
static void origen_dwmci_cfg_gpio(int width)
{
	unsigned int gpio;

	for (gpio = EXYNOS4_GPK0(0); gpio < EXYNOS4_GPK0(2); gpio++) {
		s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(3));
		s3c_gpio_setpull(gpio, S3C_GPIO_PULL_NONE);
		s5p_gpio_set_drvstr(gpio, S5P_GPIO_DRVSTR_LV4);
	}

	switch (width) {
	case 8:
		for (gpio = EXYNOS4_GPK1(3); gpio <= EXYNOS4_GPK1(6); gpio++) {
			s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(4));
			s3c_gpio_setpull(gpio, S3C_GPIO_PULL_UP);
			s5p_gpio_set_drvstr(gpio, S5P_GPIO_DRVSTR_LV4);
		}
	case 4:
		for (gpio = EXYNOS4_GPK0(3); gpio <= EXYNOS4_GPK0(6); gpio++) {
			s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(3));
			s3c_gpio_setpull(gpio, S3C_GPIO_PULL_UP);
			s5p_gpio_set_drvstr(gpio, S5P_GPIO_DRVSTR_LV4);
		}
		break;
	case 1:
		gpio = EXYNOS4_GPK0(3);
		s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(3));
		s3c_gpio_setpull(gpio, S3C_GPIO_PULL_UP);
		s5p_gpio_set_drvstr(gpio, S5P_GPIO_DRVSTR_LV4);
	default:
		break;
	}

	gpio = EXYNOS4_GPK0(2);
	s3c_gpio_cfgpin(gpio, S3C_GPIO_INPUT);
}

static struct dw_mci_board origen_dwmci_pdata __initdata = {
	.num_slots		= 1,
	.quirks			= DW_MCI_QUIRK_BROKEN_CARD_DETECTION | \
				  DW_MCI_QUIRK_HIGHSPEED,
	.bus_hz			= 100 * 1000 * 1000,
	.caps			= MMC_CAP_UHS_DDR50 | MMC_CAP_8_BIT_DATA | \
				  MMC_CAP_CMD23,
	.fifo_depth		= 0x80,
	.detect_delay_ms	= 200,
	.hclk_name		= "dwmci",
	.cclk_name		= "sclk_dwmci",
	.cfg_gpio		= origen_dwmci_cfg_gpio,
};
#endif

static struct platform_device exynos4412_busfreq = {
	.name	= "exynos4412-busfreq",
	.id	= 1,
};

/* BUSFREQ to control memory/bus */
static struct device_domain busfreq;

static struct platform_device exynos_busfreq = {
	.name	= "exynos-busfreq",
	.id	= -1,
};

#ifdef CONFIG_BATTERY_SAMSUNG
static struct platform_device samsung_device_battery = {
	.name	= "samsung-fake-battery",
	.id	= -1,
};
#endif

#ifdef CONFIG_VIDEO_EXYNOS_FIMG2D
static struct fimg2d_platdata fimg2d_data __initdata = {
	.ip_ver		= IP_VER_G2D_4P,
	.hw_ver		= 0x41,
	.parent_clkname	= "mout_g2d0",
	.clkname	= "sclk_fimg2d",
	.gate_clkname	= "fimg2d",
	.clkrate	= 200 * MHZ,
};
#endif

/* USB OTG */
static struct s3c_hsotg_plat origen_hsotg_pdata;
/* USB EHCI */
static struct s5p_ehci_platdata origen_ehci_pdata;
/* USB OHCI */
static struct exynos4_ohci_platdata origen_ohci_pdata;

static int origen_uhost_reset(void)
{
	int err;
	err = gpio_request_one(EXYNOS4_GPX3(5), 1, "GPX3");
	if (err) {
		printk(KERN_ERR "failed to request GPX3 for "
				"suspend/resume control\n");
		return -1;
	}
	s3c_gpio_setpull(EXYNOS4_GPX3(5), S3C_GPIO_PULL_NONE);
	gpio_direction_output(EXYNOS4_GPX3(5), 1);
	gpio_set_value(EXYNOS4_GPX3(5), 1);
	gpio_free(EXYNOS4_GPX3(5));
	return 0;
}

static void origen_hdmi_hdp_init(void)
{
        /* direct HPD to External Interrupt */
        WARN_ON(gpio_request_one(EXYNOS4_GPX3(7), GPIOF_IN, "hpd-plug"));
        s3c_gpio_cfgpin(EXYNOS4_GPX3(7), S3C_GPIO_SFN(0xf));
        s3c_gpio_setpull(EXYNOS4_GPX3(7), S3C_GPIO_PULL_NONE);
	gpio_free(EXYNOS4_GPX3(7));
}

static void origen_i2c6_setup(void)
{
	s5p_gpio_set_drvstr(EXYNOS4_GPC1(3), 3);
	s5p_gpio_set_drvstr(EXYNOS4_GPC1(4), 3);
}

static struct platform_device *origen_devices[] __initdata = {
	&s3c_device_hsmmc2,
	&s3c_device_i2c0,
	&s3c_device_i2c1,
	&s3c_device_i2c2,
	&s3c_device_i2c3,
	&s3c_device_i2c6,
	&s3c_device_rtc,
	&s3c_device_usb_hsotg,
	&s3c_device_wdt,
	&s5p_device_ehci,
	&s5p_device_fimc_md,
	&s5p_device_fimc0,
	&s5p_device_fimc1,
	&s5p_device_fimc2,
	&s5p_device_fimc3,
	&s5p_device_fimd0,
	&s5p_device_fimg2d,
	&s5p_device_hdmi,
	&s5p_device_i2c_hdmiphy,
	&s5p_device_mfc,
	&s5p_device_mfc_l,
	&s5p_device_mfc_r,
	&s5p_device_mixer,
	&samsung_device_battery,
	&samsung_device_keypad,
	&exynos_busfreq,
	&exynos_device_dwmci,
	&exynos_device_ion,
	&exynos4_device_ohci,
	&exynos4412_busfreq,
	&mali_gpu_device,
	&origen_device_keypad,
	&origen_device_lcd,
};

static int __init exynos4_setup_clock(struct device *dev,
						const char *clock,
						const char *parent,
						unsigned long clk_rate)
{
	struct clk *clk_parent;
	struct clk *sclk;

	sclk = clk_get(dev, clock);
	if (IS_ERR(sclk)) {
		pr_err("Unable to get clock:%s.\n", clock);
		return PTR_ERR(sclk);
	}

	clk_parent = clk_get(NULL, parent);
	if (IS_ERR(clk_parent)) {
		clk_put(sclk);
		pr_err("Unable to get parent clock:%s of clock:%s.\n",
				parent, sclk->name);
		return PTR_ERR(clk_parent);
	}

	if (clk_set_parent(sclk, clk_parent)) {
		pr_err("Unable to set parent %s of clock %s.\n", parent, clock);
		clk_put(sclk);
		clk_put(clk_parent);
		return PTR_ERR(sclk);
	}

	if (clk_rate)
		if (clk_set_rate(sclk, clk_rate)) {
			pr_err("%s rate change failed: %lu\n", sclk->name,
				clk_rate);
			clk_put(sclk);
			clk_put(clk_parent);
			return PTR_ERR(sclk);
		}

	clk_put(sclk);
	clk_put(clk_parent);

	return 0;
}

static void origen_init_clocks(void)
{
	exynos4_setup_clock(&s5p_device_fimd0.dev, "sclk_fimd",
					"mout_mpll_user", 160 * MHZ);

	exynos4_setup_clock(&s5p_device_fimc0.dev, "sclk_fimc",
					"mout_mpll_user", 160 * MHZ);
	exynos4_setup_clock(&s5p_device_fimc1.dev, "sclk_fimc",
					"mout_mpll_user", 160 * MHZ);
	exynos4_setup_clock(&s5p_device_fimc2.dev, "sclk_fimc",
					"mout_mpll_user", 160 * MHZ);
	exynos4_setup_clock(&s5p_device_fimc3.dev, "sclk_fimc",
					"mout_mpll_user", 160 * MHZ);

	exynos4_setup_clock(NULL, "mout_mfc0", "mout_mpll", 0);
	exynos4_setup_clock(&s5p_device_mfc.dev, "sclk_mfc",
					"mout_mfc0", 200 * MHZ);

	exynos4_setup_clock(&s3c_device_hsmmc2.dev, "dout_mmc2",
					"mout_mpll_user", 100 * MHZ);

	exynos4_setup_clock(&exynos_device_dwmci.dev, "dout_mmc4",
					"mout_mpll_user", 400 * MHZ);
}

static void origen_pmu_wdt_init(void)
{
	unsigned int value;

	if (soc_is_exynos4212() || soc_is_exynos4412()) {
		value = __raw_readl(S5P_AUTOMATIC_WDT_RESET_DISABLE);
		value &= ~S5P_SYS_WDTRESET;
		__raw_writel(value, S5P_AUTOMATIC_WDT_RESET_DISABLE);
		value = __raw_readl(S5P_MASK_WDT_RESET_REQUEST);
		value &= ~S5P_SYS_WDTRESET;
		__raw_writel(value, S5P_MASK_WDT_RESET_REQUEST);
	}
}

static void origen_rtc_wake_init(void)
{
#ifdef CONFIG_PM
	gic_arch_extn.irq_set_wake = s3c_irq_wake;
#endif
}

static void __init origen_powerkey_init(void)
{
	int err = 0;

	err = gpio_request_one(EXYNOS4_GPX2(7), 0, "GPX2");
	if (err) {
		printk(KERN_ERR "failed to request GPX2 for "
				"suspend/resume control\n");
		return;
	}
	s3c_gpio_setpull(EXYNOS4_GPX2(7), S3C_GPIO_PULL_NONE);
	gpio_free(EXYNOS4_GPX2(7));
}

static void __init origen_machine_init(void)
{
	s3c_i2c0_set_platdata(NULL);
	i2c_register_board_info(0, origen_i2c_devs0,
			ARRAY_SIZE(origen_i2c_devs0));

	s3c_i2c1_set_platdata(NULL);
	i2c_register_board_info(1, origen_i2c_devs1,
				ARRAY_SIZE(origen_i2c_devs1));

	s3c_i2c2_set_platdata(NULL);
	i2c_register_board_info(2, origen_i2c_devs2,
				ARRAY_SIZE(origen_i2c_devs2));

	s3c_i2c3_set_platdata(NULL);
	i2c_register_board_info(3, origen_i2c_devs3,
				ARRAY_SIZE(origen_i2c_devs3));

	s3c_i2c6_set_platdata(NULL);
	i2c_register_board_info(6, origen_i2c_devs6,
				ARRAY_SIZE(origen_i2c_devs6));

	origen_rtc_wake_init();
	origen_pmu_wdt_init();

	origen_powerkey_init();
	samsung_keypad_set_platdata(&origen_keypad_data);

	origen_hdmi_hdp_init();
	s5p_i2c_hdmiphy_set_platdata(NULL);
	s5p_hdmi_set_platdata(origen_i2c_hdmiphy, NULL, 0);
	origen_i2c6_setup();

	exynos_dwmci_set_platdata(&origen_dwmci_pdata);
	s3c_sdhci2_set_platdata(&origen_hsmmc2_pdata);

	exynos_ion_set_platdata();

	s5p_fimg2d_set_platdata(&fimg2d_data);

	s5p_fimd0_set_platdata(&origen_lcd0_pdata);
	samsung_bl_set(&origen_bl_gpio_info, &origen_bl_data);

	s3c_hsotg_set_platdata(&origen_hsotg_pdata);
	s5p_ehci_set_platdata(&origen_ehci_pdata);
	exynos4_ohci_set_platdata(&origen_ohci_pdata);
	origen_uhost_reset();

	platform_add_devices(origen_devices, ARRAY_SIZE(origen_devices));

	origen_init_clocks();

	dev_add(&busfreq, &exynos_busfreq.dev);
	ppmu_init(&exynos_ppmu[PPMU_DMC0], &exynos_busfreq.dev);
	ppmu_init(&exynos_ppmu[PPMU_DMC1], &exynos_busfreq.dev);
	ppmu_init(&exynos_ppmu[PPMU_CPU], &exynos_busfreq.dev);
}

static void __init origen_map_io(void)
{
	clk_xusbxti.rate = 24000000;

	exynos_init_io(NULL, 0);
	s3c24xx_init_clocks(clk_xusbxti.rate);
	s3c24xx_init_uarts(origen_uartcfgs, ARRAY_SIZE(origen_uartcfgs));
}

static void __init origen_reserve(void)
{
	// HACK: This reserved memory will be used for FIMC-IS
	s5p_mfc_reserve_mem(0x58000000, 32<< 20, 0x43000000, 0 << 20);
}

MACHINE_START(ORIGEN_QUAD, "ORIGEN_QUAD")
	/* Maintainer: JeongHyeon Kim <jhkim@insignal.co.kr> */
	.atag_offset	= 0x100,
	.handle_irq	= gic_handle_irq,
	.init_irq	= exynos4_init_irq,
	.init_late	= exynos_init_late,
	.init_machine	= origen_machine_init,
	.map_io		= origen_map_io,
	.reserve	= &origen_reserve,
	.restart	= exynos4_restart,
	.timer		= &exynos4_timer,
MACHINE_END
