/* linux/arch/arm/mach-exynos4/mach-smdkc210.c
 *
 * Copyright (c) 2010-2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/serial_core.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/gpio_event.h>
#include <linux/lcd.h>
#include <linux/mmc/host.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/i2c.h>
#include <linux/i2c-gpio.h>
#include <linux/i2c/mcs.h>
#include <linux/i2c/atmel_mxt_ts.h>
#include <linux/input.h>
#include <linux/switch.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_gpio.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/mfd/max8997.h>
#include <linux/mfd/max8997-private.h>
#include <linux/sensor/k3g.h>
#include <linux/sensor/k3dh.h>
#include <linux/sensor/ak8975.h>
#include <linux/sensor/cm3663.h>
#include <linux/pn544.h>
#include <linux/mfd/mc1n2_pdata.h>
#include <linux/memblock.h>
#include <linux/power_supply.h>
#include <linux/cma.h>

#include <asm/mach/arch.h>
#include <asm/mach-types.h>

#include <video/platform_lcd.h>

#include <plat/regs-serial.h>
#include <plat/regs-srom.h>
#include <plat/exynos4.h>
#include <plat/clock.h>
#include <plat/hwmon.h>
#include <plat/cpu.h>
#include <plat/devs.h>
#include <plat/fb-s5p.h>
#include <plat/fimc.h>
#include <plat/csis.h>
#include <plat/gpio-cfg.h>
#include <plat/adc.h>
#include <plat/ts.h>
#include <plat/keypad.h>
#include <plat/sdhci.h>
#include <plat/mshci.h>
#include <plat/iic.h>
#include <plat/sysmmu.h>
#include <plat/pd.h>
#include <plat/regs-fb-v4.h>
#include <plat/media.h>
#include <plat/udc-hs.h>
#include <plat/s5p-clock.h>
#include <plat/tvout.h>
#include <plat/fimg2d.h>
#include <plat/ehci.h>
#include <plat/usbgadget.h>

#include <mach/map.h>
#include <mach/exynos-clock.h>
#include <mach/media.h>
#include <plat/regs-fb.h>

#include <mach/dev-sysmmu.h>
#include <mach/dev.h>
#include <mach/regs-clock.h>
#include <mach/exynos-ion.h>

#include <media/m5mo_platform.h>
#include <media/s5k5bafx_platform.h>

#include <plat/s5p-tmu.h>
#include <mach/regs-tmu.h>

#include <linux/sec_jack.h>

#include <mach/board-bluetooth-bcm.h>

#include <linux/ld9040.h>

#include <../../../drivers/video/samsung/s3cfb.h>
#include "u1.h"

#include <mach/sec_debug.h>

#include <linux/irq.h>
#include <linux/sii9234.h>

#include <linux/power/sec_battery_u1.h>
#include <mach/sec_thermistor.h>
#include <linux/power/max17042_fuelgauge_u1.h>
#include <linux/power/max8922_charger_u1.h>

#include <linux/host_notify.h>

enum i2c_bus_ids {
	I2C_GPIO_BUS_GAUGE = 9,
	I2C_GPIO_BUS_USB = 10,
	I2C_GPIO_BUS_PROX = 11,
	I2C_GPIO_BUS_VT_CAM = 12,
	I2C_GPIO_BUS_TOUCHKEY = 13,
	I2C_GPIO_BUS_NFC = 14,
	I2C_GPIO_BUS_MHL = 15,
	I2C_GPIO_BUS_FM = 16,
	I2C_GPIO_BUS_DBT = 17,
	I2C_GPIO_BUS_SMB329 = 19,
};

/*******************************************************************************
 * UART
 ******************************************************************************/
/* Following are default values for UCON, ULCON and UFCON UART registers */
#define SMDKC210_UCON_DEFAULT	(S3C2410_UCON_TXILEVEL |	\
				 S3C2410_UCON_RXILEVEL |	\
				 S3C2410_UCON_TXIRQMODE |	\
				 S3C2410_UCON_RXIRQMODE |	\
				 S3C2410_UCON_RXFIFO_TOI |	\
				 S3C2443_UCON_RXERR_IRQEN)

#define SMDKC210_ULCON_DEFAULT	S3C2410_LCON_CS8

#define SMDKC210_UFCON_DEFAULT	(S3C2410_UFCON_FIFOMODE |	\
				 S5PV210_UFCON_TXTRIG4 |	\
				 S5PV210_UFCON_RXTRIG4)

static struct s3c2410_uartcfg smdkc210_uartcfgs[] __initdata = {
	[0] = {
		.hwport = 0,
		.flags = 0,
		.ucon = SMDKC210_UCON_DEFAULT,
		.ulcon = SMDKC210_ULCON_DEFAULT,
		.ufcon = SMDKC210_UFCON_DEFAULT,
#ifdef CONFIG_BT_BCM4330
		.wake_peer = bcm_bt_lpm_exit_lpm_locked,
#endif
	},
	[1] = {
		.hwport = 1,
		.flags = 0,
		.ucon = SMDKC210_UCON_DEFAULT,
		.ulcon = SMDKC210_ULCON_DEFAULT,
		.ufcon = SMDKC210_UFCON_DEFAULT,
		.set_runstate = set_gps_uart_op,
	},
	[2] = {
		.hwport = 2,
		.flags = 0,
		.ucon = SMDKC210_UCON_DEFAULT,
		.ulcon = SMDKC210_ULCON_DEFAULT,
		.ufcon = SMDKC210_UFCON_DEFAULT,
	},
	[3] = {
		.hwport = 3,
		.flags = 0,
		.ucon = SMDKC210_UCON_DEFAULT,
		.ulcon = SMDKC210_ULCON_DEFAULT,
		.ufcon = SMDKC210_UFCON_DEFAULT,
	},
};

/*******************************************************************************
 * Camera FIMC
 ******************************************************************************/
/*
 * External camera reset
 * Because the most of cameras take i2c bus signal, so that
 * you have to reset at the boot time for other i2c slave devices.
 * This function also called at fimc_init_camera()
 * Do optimization for cameras on your platform.
 */

#define CAM_CHECK_ERR_RET(x, msg)					\
	if (unlikely((x) < 0)) {					\
		printk(KERN_ERR "\nfail to %s: err = %d\n", msg, x);	\
		return x;						\
	}
#define CAM_CHECK_ERR(x, msg)						\
	if (unlikely((x) < 0)) {					\
		printk(KERN_ERR "\nfail to %s: err = %d\n", msg, x);	\
	}

static int m5mo_get_i2c_busnum(void)
{
	return 0;
}

static int m5mo_power_on(void)
{
	struct regulator *regulator;
	int ret = 0;

	printk(KERN_DEBUG "%s: in\n", __func__);

	ret = gpio_request(GPIO_CAM_VGA_nSTBY, "GPL2");
	if (ret) {
		printk(KERN_ERR "faile to request gpio(GPIO_CAM_VGA_nSTBY)\n");
		return ret;
	}
	ret = gpio_request(GPIO_CAM_VGA_nRST, "GPL2");
	if (ret) {
		printk(KERN_ERR "faile to request gpio(GPIO_CAM_VGA_nRST)\n");
		return ret;
	}
	ret = gpio_request(GPIO_CAM_SENSOR_CORE, "GPE2");
	if (ret) {
		printk(KERN_ERR "fail to request gpio(CAM_SENSOR_CORE)\n");
		return ret;
	}
	ret = gpio_request(GPIO_CAM_IO_EN, "GPE2");
	if (ret) {
		printk(KERN_ERR "fail to request gpio(CAM_IO_EN)\n");
		return ret;
	}
	ret = gpio_request(GPIO_VT_CAM_15V, "GPE2");
	if (ret) {
		printk(KERN_ERR "faile to request gpio(GPIO_VT_CAM_15V)\n");
		return ret;
	}
	ret = gpio_request(GPIO_ISP_RESET, "ISP_RESET");
	if (ret) {
		printk(KERN_ERR "faile to request gpio(ISP_RESET)\n");
		return ret;
	}
	ret = gpio_request(GPIO_8M_AF_EN, "GPK1");
	if (ret) {
		printk(KERN_ERR "fail to request gpio(8M_AF_EN)\n");
		return ret;
	}

	/* CAM_VT_nSTBY low */
	ret = gpio_direction_output(GPIO_CAM_VGA_nSTBY, 0);
	CAM_CHECK_ERR_RET(ret, "output VGA_nSTBY");

	/* CAM_VT_nRST	low */
	gpio_direction_output(GPIO_CAM_VGA_nRST, 0);
	CAM_CHECK_ERR_RET(ret, "output VGA_nRST");
	udelay(10);

	/* CAM_ISP_CORE_1.2V */
	regulator = regulator_get(NULL, "cam_isp_core");
	if (IS_ERR(regulator))
		return -ENODEV;
	ret = regulator_enable(regulator);
	regulator_put(regulator);
	CAM_CHECK_ERR_RET(ret, "enable cam_isp_core");
	/* No delay */

	/* CAM_SENSOR_CORE_1.2V */
	ret = gpio_direction_output(GPIO_CAM_SENSOR_CORE, 1);
	CAM_CHECK_ERR_RET(ret, "output senser_core");

	udelay(10);

	/* CAM_SENSOR_A2.8V */
	ret = gpio_direction_output(GPIO_CAM_IO_EN, 1);
	CAM_CHECK_ERR_RET(ret, "output IO_EN");
	/* it takes about 100us at least during level transition. */
	udelay(160);		/* 130us -> 160us */

	/* VT_CORE_1.5V */
	ret = gpio_direction_output(GPIO_VT_CAM_15V, 1);
	CAM_CHECK_ERR_RET(ret, "output VT_CAM_1.5V");
	udelay(20);

	/* CAM_AF_2.8V */
	ret = gpio_direction_output(GPIO_8M_AF_EN, 1);
	CAM_CHECK_ERR(ret, "output AF");
	mdelay(7);

	/* VT_CAM_1.8V */
	regulator = regulator_get(NULL, "vt_cam_1.8v");
	if (IS_ERR(regulator))
		return -ENODEV;
	ret = regulator_enable(regulator);
	regulator_put(regulator);
	CAM_CHECK_ERR_RET(ret, "enable vt_1.8v");
	udelay(10);

	/* CAM_ISP_1.8V */
	regulator = regulator_get(NULL, "cam_isp");
	if (IS_ERR(regulator))
		return -ENODEV;
	ret = regulator_enable(regulator);
	regulator_put(regulator);
	CAM_CHECK_ERR_RET(ret, "enable cam_isp");
	udelay(120);		/* at least */

	/* CAM_SENSOR_IO_1.8V */
	regulator = regulator_get(NULL, "cam_sensor_io");
	if (IS_ERR(regulator))
		return -ENODEV;
	ret = regulator_enable(regulator);
	regulator_put(regulator);
	CAM_CHECK_ERR_RET(ret, "enable sensor_io");
	udelay(30);

	/* MCLK */
	ret = s3c_gpio_cfgpin(GPIO_CAM_MCLK, S3C_GPIO_SFN(2));
	CAM_CHECK_ERR_RET(ret, "cfg mclk");
	s3c_gpio_setpull(GPIO_CAM_MCLK, S3C_GPIO_PULL_NONE);
	udelay(70);

	/* ISP_RESET */
	ret = gpio_direction_output(GPIO_ISP_RESET, 1);
	CAM_CHECK_ERR_RET(ret, "output reset");
	mdelay(4);

	gpio_free(GPIO_CAM_VGA_nSTBY);
	gpio_free(GPIO_CAM_VGA_nRST);
	gpio_free(GPIO_CAM_SENSOR_CORE);
	gpio_free(GPIO_CAM_IO_EN);
	gpio_free(GPIO_VT_CAM_15V);
	gpio_free(GPIO_ISP_RESET);
	gpio_free(GPIO_8M_AF_EN);
	printk(KERN_DEBUG "%s: out\n", __func__);

	return ret;
}

/*******************************************************************************
 * HDMI MHL
 ******************************************************************************/
static void sii9234_cfg_gpio(void)
{
	printk(KERN_INFO "%s()\n", __func__);

	s3c_gpio_cfgpin(GPIO_AP_SDA_18V, S3C_GPIO_SFN(0x0));
	s3c_gpio_setpull(GPIO_AP_SDA_18V, S3C_GPIO_PULL_NONE);

	s3c_gpio_cfgpin(GPIO_AP_SCL_18V, S3C_GPIO_SFN(0x1));
	s3c_gpio_setpull(GPIO_AP_SCL_18V, S3C_GPIO_PULL_NONE);

	s3c_gpio_cfgpin(GPIO_MHL_WAKE_UP, S3C_GPIO_INPUT);
	irq_set_irq_type(MHL_WAKEUP_IRQ, IRQ_TYPE_EDGE_RISING);
	s3c_gpio_setpull(GPIO_MHL_WAKE_UP, S3C_GPIO_PULL_DOWN);

	gpio_request(GPIO_MHL_INT, "MHL_INT");
	s5p_register_gpio_interrupt(GPIO_MHL_INT);
	s3c_gpio_setpull(GPIO_MHL_INT, S3C_GPIO_PULL_DOWN);
	irq_set_irq_type(MHL_INT_IRQ, IRQ_TYPE_EDGE_RISING);
	s3c_gpio_cfgpin(GPIO_MHL_INT, GPIO_MHL_INT_AF);

	if (system_rev < 7) {
		s3c_gpio_cfgpin(GPIO_HDMI_EN, S3C_GPIO_OUTPUT);
		gpio_set_value(GPIO_HDMI_EN, GPIO_LEVEL_LOW);
		s3c_gpio_setpull(GPIO_HDMI_EN, S3C_GPIO_PULL_NONE);
	} else {
		s3c_gpio_cfgpin(GPIO_HDMI_EN_REV07, S3C_GPIO_OUTPUT);
		gpio_set_value(GPIO_HDMI_EN_REV07, GPIO_LEVEL_LOW);
		s3c_gpio_setpull(GPIO_HDMI_EN_REV07, S3C_GPIO_PULL_NONE);
	}

	s3c_gpio_cfgpin(GPIO_MHL_RST, S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(GPIO_MHL_RST, S3C_GPIO_PULL_NONE);
	gpio_set_value(GPIO_MHL_RST, GPIO_LEVEL_LOW);

	s3c_gpio_cfgpin(GPIO_MHL_SEL, S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(GPIO_MHL_SEL, S3C_GPIO_PULL_NONE);
	gpio_set_value(GPIO_MHL_SEL, GPIO_LEVEL_LOW);

}

void sii9234_power_onoff(bool on)
{
	pr_info("%s(%d)\n", __func__, on);

	if (on) {
		if (system_rev < 7)
			gpio_set_value(GPIO_HDMI_EN, GPIO_LEVEL_HIGH);
		else
			gpio_set_value(GPIO_HDMI_EN_REV07, GPIO_LEVEL_HIGH);

		s3c_gpio_setpull(GPIO_AP_SCL_18V, S3C_GPIO_PULL_DOWN);
		s3c_gpio_setpull(GPIO_AP_SCL_18V, S3C_GPIO_PULL_NONE);

	} else {
		gpio_set_value(GPIO_MHL_RST, GPIO_LEVEL_LOW);
		usleep_range(10000, 20000);
		gpio_set_value(GPIO_MHL_RST, GPIO_LEVEL_HIGH);
		if (system_rev < 7)
			gpio_set_value(GPIO_HDMI_EN, GPIO_LEVEL_LOW);
		else
			gpio_set_value(GPIO_HDMI_EN_REV07, GPIO_LEVEL_LOW);
		gpio_set_value(GPIO_MHL_RST, GPIO_LEVEL_LOW);
	}
	pr_info("[MHL]%s : %d\n", __func__, on);
}

void sii9234_reset(void)
{
	s3c_gpio_cfgpin(GPIO_MHL_RST, S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(GPIO_MHL_RST, S3C_GPIO_PULL_NONE);

	gpio_set_value(GPIO_MHL_RST, GPIO_LEVEL_LOW);
	usleep_range(10000, 20000);
	gpio_set_value(GPIO_MHL_RST, GPIO_LEVEL_HIGH);

}

void mhl_usb_switch_control(bool on)
{
	pr_info("%s() [MHL] USB path change : %s\n",
			__func__, on ? "MHL" : "USB");
	if (on == 1) {
		if (gpio_get_value(GPIO_MHL_SEL))
			pr_info("[MHL] GPIO_MHL_SEL :already 1\n");
		else {
			gpio_set_value(GPIO_MHL_SEL, GPIO_LEVEL_HIGH);
			/* sii9234_cfg_power(1);	// onegun */
			/* sii9234_init();		// onegun */
		}
	} else {
		if (!gpio_get_value(GPIO_MHL_SEL))
			pr_info("[MHL]	GPIO_MHL_SEL :already0\n");
		else {
			/* sii9234_cfg_power(0);	// onegun */
			gpio_set_value(GPIO_MHL_SEL, GPIO_LEVEL_LOW);
		}
	}
}

static struct sii9234_platform_data sii9234_pdata = {
	.init = sii9234_cfg_gpio,
	.mhl_sel = mhl_usb_switch_control,
	.hw_onoff = sii9234_power_onoff,
	.hw_reset = sii9234_reset,
	.enable_vbus = NULL,
	.vbus_present = NULL,
};

static struct i2c_board_info __initdata tuna_i2c15_boardinfo[] = {
	{
		I2C_BOARD_INFO("sii9234_mhl_tx", 0x72>>1),
		.platform_data = &sii9234_pdata,
	},
	{
		I2C_BOARD_INFO("sii9234_tpi", 0x7A>>1),
		.platform_data = &sii9234_pdata,
	},
	{
		I2C_BOARD_INFO("sii9234_hdmi_rx", 0x92>>1),
		.platform_data = &sii9234_pdata,
	},
	{
		I2C_BOARD_INFO("sii9234_cbus", 0xC8>>1),
		.platform_data = &sii9234_pdata,
	},
};

static struct i2c_gpio_platform_data gpio_i2c_data15 = {
	.sda_pin = GPIO_MHL_SDA_18V,
	.scl_pin = GPIO_MHL_SCL_18V,
	.udelay = 2,
	.sda_is_open_drain	= 0,
	.scl_is_open_drain	= 0,
	.scl_is_output_only	= 0,
};

struct platform_device s3c_device_i2c15 = {
	.name = "i2c-gpio",
	.id = I2C_GPIO_BUS_MHL,
	.dev = {
		.platform_data = &gpio_i2c_data15,
	}
};


/*******************************************************************************
 * M5MO Rear Camera
 ******************************************************************************/
static int m5mo_power_down(void)
{
	struct regulator *regulator;
	int ret = 0;

	printk(KERN_DEBUG "%s: in\n", __func__);

	ret = gpio_request(GPIO_8M_AF_EN, "GPK1");
	if (ret) {
		printk(KERN_ERR "fail to request gpio(8M_AF_EN)\n");
		return ret;
	}
	ret = gpio_request(GPIO_ISP_RESET, "ISP_RESET");
	if (ret) {
		printk(KERN_ERR "faile to request gpio(ISP_RESET)\n");
		return ret;
	}
	ret = gpio_request(GPIO_CAM_IO_EN, "GPE2");
	if (ret) {
		printk(KERN_ERR "fail to request gpio(GPIO_CAM_IO_EN)\n");
		return ret;
	}
	ret = gpio_request(GPIO_CAM_SENSOR_CORE, "GPE2");
	if (ret) {
		printk(KERN_ERR "fail to request gpio(CAM_SENSOR_COR)\n");
		return ret;
	}
	ret = gpio_request(GPIO_VT_CAM_15V, "GPE2");
	if (ret) {
		printk(KERN_ERR "faile to request gpio(GPIO_VT_CAM_15V)\n");
		return ret;
	}

	/* s3c_i2c0_force_stop(); */

	mdelay(3);

	/* ISP_RESET */
	ret = gpio_direction_output(GPIO_ISP_RESET, 0);
	CAM_CHECK_ERR(ret, "output reset");
	mdelay(2);

	/* MCLK */
	ret = s3c_gpio_cfgpin(GPIO_CAM_MCLK, S3C_GPIO_INPUT);
	s3c_gpio_setpull(GPIO_CAM_MCLK, S3C_GPIO_PULL_DOWN);
	CAM_CHECK_ERR(ret, "cfg mclk");
	udelay(20);

	/* CAM_AF_2.8V */
	/* 8M_AF_2.8V_EN */
	ret = gpio_direction_output(GPIO_8M_AF_EN, 0);
	CAM_CHECK_ERR(ret, "output AF");

	/* CAM_SENSOR_IO_1.8V */
	regulator = regulator_get(NULL, "cam_sensor_io");
	if (IS_ERR(regulator))
		return -ENODEV;
	if (regulator_is_enabled(regulator))
		ret = regulator_force_disable(regulator);
	regulator_put(regulator);
	CAM_CHECK_ERR(ret, "disable, sensor_io");
	udelay(10);

	/* CAM_ISP_1.8V */
	regulator = regulator_get(NULL, "cam_isp");
	if (IS_ERR(regulator))
		return -ENODEV;
	if (regulator_is_enabled(regulator))
		ret = regulator_force_disable(regulator);
	regulator_put(regulator);
	CAM_CHECK_ERR(ret, "disable cam_isp");
	udelay(500);		/* 100us -> 500us */

	/* VT_CAM_1.8V */
	regulator = regulator_get(NULL, "vt_cam_1.8v");
	if (IS_ERR(regulator))
		return -ENODEV;
	if (regulator_is_enabled(regulator))
		ret = regulator_disable(regulator);
	regulator_put(regulator);
	CAM_CHECK_ERR(ret, "disable vt_1.8v");
	udelay(250);		/* 10us -> 250us */

	/* VT_CORE_1.5V */
	ret = gpio_direction_output(GPIO_VT_CAM_15V, 0);
	CAM_CHECK_ERR(ret, "output VT_CAM_1.5V");
	udelay(300);		/*10 -> 300 us */

	/* CAM_SENSOR_A2.8V */
	ret = gpio_direction_output(GPIO_CAM_IO_EN, 0);
	CAM_CHECK_ERR(ret, "output IO_EN");
	udelay(800);

	/* CAM_SENSOR_CORE_1.2V */
	ret = gpio_direction_output(GPIO_CAM_SENSOR_CORE, 0);
	CAM_CHECK_ERR(ret, "output SENSOR_CORE");
	udelay(5);

	/* CAM_ISP_CORE_1.2V */
	regulator = regulator_get(NULL, "cam_isp_core");
	if (IS_ERR(regulator))
		return -ENODEV;
	if (regulator_is_enabled(regulator))
		ret = regulator_force_disable(regulator);
	regulator_put(regulator);
	CAM_CHECK_ERR(ret, "disable isp_core");

	gpio_free(GPIO_8M_AF_EN);
	gpio_free(GPIO_ISP_RESET);
	gpio_free(GPIO_CAM_IO_EN);
	gpio_free(GPIO_CAM_SENSOR_CORE);
	gpio_free(GPIO_VT_CAM_15V);

	return ret;
}

int s3c_csis_power(int enable)
{
	struct regulator *regulator;
	int ret = 0;
	printk(KERN_DEBUG "%s: in\n", __func__);

	/* mipi_1.1v ,mipi_1.8v are always powered-on.
	 * If they are off, we then power them on.
	 */
	if (enable) {
		/* VMIPI_1.1V */
		regulator = regulator_get(NULL, "vmipi_1.1v");
		if (IS_ERR(regulator))
			goto error_out;
		if (!regulator_is_enabled(regulator)) {
			printk(KERN_WARNING "%s: vmipi_1.1v is off. so ON\n",
			       __func__);
			ret = regulator_enable(regulator);
			CAM_CHECK_ERR(ret, "enable vmipi_1.1v");
		}
		regulator_put(regulator);

		/* VMIPI_1.8V */
		regulator = regulator_get(NULL, "vmipi_1.8v");
		if (IS_ERR(regulator))
			goto error_out;
		if (!regulator_is_enabled(regulator)) {
			printk(KERN_WARNING "%s: vmipi_1.8v is off. so ON\n",
			       __func__);
			ret = regulator_enable(regulator);
			CAM_CHECK_ERR(ret, "enable vmipi_1.8v");
		}
		regulator_put(regulator);
	}
	printk(KERN_DEBUG "%s: out\n", __func__);

	return 0;

error_out:
	printk(KERN_ERR "%s: ERROR: failed to check mipi-power\n", __func__);
	return 0;
}

static int m5mo_flash_power(int enable)
{
	struct regulator *flash = regulator_get(NULL, "led_flash");
	struct regulator *movie = regulator_get(NULL, "led_movie");

	if (enable) {
		regulator_set_current_limit(flash, 490000, 530000);
		regulator_enable(flash);
		regulator_set_current_limit(movie, 90000, 110000);
		regulator_enable(movie);
	} else {

		if (regulator_is_enabled(flash))
			regulator_disable(flash);
		if (regulator_is_enabled(movie))
			regulator_disable(movie);
	}
	
	regulator_put(flash);
	regulator_put(movie);

	return 0;
}

static int m5mo_power(int enable)
{
	int ret = 0;

	printk(KERN_DEBUG "%s %s\n", __func__, enable ? "on" : "down");
	if (enable) {
		ret = m5mo_power_on();
		if (unlikely(ret))
			goto error_out;
	} else
		ret = m5mo_power_down();

	ret = s3c_csis_power(enable);
	m5mo_flash_power(enable);

error_out:
	return ret;
}

static int m5mo_config_isp_irq(void)
{
	s3c_gpio_cfgpin(GPIO_ISP_INT, S3C_GPIO_SFN(0xF));
	s3c_gpio_setpull(GPIO_ISP_INT, S3C_GPIO_PULL_NONE);
	return 0;
}

static struct m5mo_platform_data m5mo_plat = {
	.default_width = 640,	/* 1920 */
	.default_height = 480,	/* 1080 */
	.pixelformat = V4L2_PIX_FMT_UYVY,
	.freq = 24000000,
	.is_mipi = 1,
	.config_isp_irq = m5mo_config_isp_irq,
	.irq = IRQ_EINT(13),
};

static struct i2c_board_info m5mo_i2c_info = {
	I2C_BOARD_INFO("M5MO", 0x1F),
	.platform_data = &m5mo_plat,
};

static struct s3c_platform_camera m5mo = {
	.id = CAMERA_CSI_C,
	.clk_name = "sclk_cam0",
	.get_i2c_busnum = m5mo_get_i2c_busnum,
	.cam_power = m5mo_power,	/*smdkv310_mipi_cam0_reset, */
	.type = CAM_TYPE_MIPI,
	.fmt = ITU_601_YCBCR422_8BIT,	/*MIPI_CSI_YCBCR422_8BIT */
	.order422 = CAM_ORDER422_8BIT_CBYCRY,
	.info = &m5mo_i2c_info,
	.pixelformat = V4L2_PIX_FMT_UYVY,
	.srclk_name = "xusbxti",	/* "mout_mpll" */
	.clk_rate = 24000000,	/* 48000000 */
	.line_length = 1920,
	.width = 640,
	.height = 480,
	.window = {
		.left = 0,
		.top = 0,
		.width = 640,
		.height = 480,
	},

	.mipi_lanes = 2,
	.mipi_settle = 12,
	.mipi_align = 32,

	/* Polarity */
	.inv_pclk = 0,
	.inv_vsync = 1,
	.inv_href = 0,
	.inv_hsync = 0,
	.reset_camera = 0,
	.initialized = 0,
};

/*******************************************************************************
 * S5K5BAFX Front Camera
 ******************************************************************************/
static int s5k5bafx_get_i2c_busnum(void)
{
	return 12;
}

static int s5k5bafx_power_on(void)
{
	struct regulator *regulator;
	int ret = 0;

	/* printk("%s: in\n", __func__); */

	ret = gpio_request(GPIO_ISP_RESET, "ISP_RESET");
	if (ret) {
		printk(KERN_ERR "faile to request gpio(ISP_RESET)\n");
		return ret;
	}
	ret = gpio_request(GPIO_CAM_IO_EN, "GPE2");
	if (ret) {
		printk(KERN_ERR "faile to request gpio(GPIO_CAM_IO_EN)\n");
		return ret;
	}
	ret = gpio_request(GPIO_VT_CAM_15V, "GPE2");
	if (ret) {
		printk(KERN_ERR "faile to request gpio(GPIO_VT_CAM_15V)\n");
		return ret;
	}
	ret = gpio_request(GPIO_CAM_VGA_nSTBY, "GPL2");
	if (ret) {
		printk(KERN_ERR "faile to request gpio(GPIO_CAM_VGA_nSTBY)\n");
		return ret;
	}
	ret = gpio_request(GPIO_CAM_VGA_nRST, "GPL2");
	if (ret) {
		printk(KERN_ERR "faile to request gpio(GPIO_CAM_VGA_nRST)\n");
		return ret;
	}

	if (system_rev >= 9) {
		s3c_gpio_setpull(VT_CAM_SDA_18V, S3C_GPIO_PULL_NONE);
		s3c_gpio_setpull(VT_CAM_SCL_18V, S3C_GPIO_PULL_NONE);
	}

	/* ISP_RESET low */
	ret = gpio_direction_output(GPIO_ISP_RESET, 0);
	CAM_CHECK_ERR_RET(ret, "output reset");
	udelay(100);

	/* CAM_ISP_CORE_1.2V */
	regulator = regulator_get(NULL, "cam_isp_core");
	if (IS_ERR(regulator))
		return -ENODEV;
	ret = regulator_enable(regulator);
	regulator_put(regulator);
	CAM_CHECK_ERR_RET(ret, "enable isp_core");
	udelay(10);

	/* CAM_SENSOR_A2.8V */
	ret = gpio_direction_output(GPIO_CAM_IO_EN, 1);
	CAM_CHECK_ERR_RET(ret, "output io_en");
	udelay(300);		/* don't change me */

	/* VT_CORE_1.5V */
	ret = gpio_direction_output(GPIO_VT_CAM_15V, 1);
	CAM_CHECK_ERR_RET(ret, "output vt_15v");
	udelay(100);

	/* CAM_ISP_1.8V */
	regulator = regulator_get(NULL, "cam_isp");
	if (IS_ERR(regulator))
		return -ENODEV;
	ret = regulator_enable(regulator);
	regulator_put(regulator);
	CAM_CHECK_ERR_RET(ret, "enable cam_isp");
	udelay(10);

	/* VT_CAM_1.8V */
	regulator = regulator_get(NULL, "vt_cam_1.8v");
	if (IS_ERR(regulator))
		return -ENODEV;
	ret = regulator_enable(regulator);
	regulator_put(regulator);
	CAM_CHECK_ERR_RET(ret, "enable vt_1.8v");
	udelay(10);

	/* CAM_VGA_nSTBY */
	ret = gpio_direction_output(GPIO_CAM_VGA_nSTBY, 1);
	CAM_CHECK_ERR_RET(ret, "output VGA_nSTBY");
	udelay(50);

	/* Mclk */
	ret = s3c_gpio_cfgpin(GPIO_CAM_MCLK, S3C_GPIO_SFN(2));
	s3c_gpio_setpull(GPIO_CAM_MCLK, S3C_GPIO_PULL_NONE);
	CAM_CHECK_ERR_RET(ret, "cfg mclk");
	udelay(100);

	/* CAM_VGA_nRST	 */
	ret = gpio_direction_output(GPIO_CAM_VGA_nRST, 1);
	CAM_CHECK_ERR_RET(ret, "output VGA_nRST");
	mdelay(2);

	gpio_free(GPIO_ISP_RESET);
	gpio_free(GPIO_CAM_IO_EN);
	gpio_free(GPIO_VT_CAM_15V);
	gpio_free(GPIO_CAM_VGA_nSTBY);
	gpio_free(GPIO_CAM_VGA_nRST);

	return 0;
}

static int s5k5bafx_power_off(void)
{
	struct regulator *regulator;
	int ret = 0;

	/* printk("n%s: in\n", __func__); */

	ret = gpio_request(GPIO_CAM_VGA_nRST, "GPL2");
	if (ret) {
		printk(KERN_ERR "faile to request gpio(GPIO_CAM_VGA_nRST)\n");
		return ret;
	}
	ret = gpio_request(GPIO_CAM_VGA_nSTBY, "GPL2");
	if (ret) {
		printk(KERN_ERR "faile to request gpio(GPIO_CAM_VGA_nSTBY)\n");
		return ret;
	}
	ret = gpio_request(GPIO_VT_CAM_15V, "GPE2");
	if (ret) {
		printk(KERN_ERR "faile to request gpio(GPIO_VT_CAM_15V)\n");
		return ret;
	}
	ret = gpio_request(GPIO_CAM_IO_EN, "GPE2");
	if (ret) {
		printk(KERN_ERR "faile to request gpio(GPIO_CAM_IO_EN)\n");
		return ret;
	}

	/* CAM_VGA_nRST	 */
	ret = gpio_direction_output(GPIO_CAM_VGA_nRST, 0);
	CAM_CHECK_ERR(ret, "output VGA_nRST");
	udelay(100);

	/* Mclk */
	ret = s3c_gpio_cfgpin(GPIO_CAM_MCLK, S3C_GPIO_INPUT);
	s3c_gpio_setpull(GPIO_CAM_MCLK, S3C_GPIO_PULL_DOWN);
	CAM_CHECK_ERR(ret, "cfg mclk");
	udelay(20);

	/* CAM_VGA_nSTBY */
	ret = gpio_direction_output(GPIO_CAM_VGA_nSTBY, 0);
	CAM_CHECK_ERR(ret, "output VGA_nSTBY");
	udelay(20);

	/* VT_CAM_1.8V */
	regulator = regulator_get(NULL, "vt_cam_1.8v");
	if (IS_ERR(regulator))
		return -ENODEV;
	if (regulator_is_enabled(regulator))
		ret = regulator_disable(regulator);
	regulator_put(regulator);
	CAM_CHECK_ERR(ret, "disable vt_1.8v");
	udelay(10);

	/* CAM_ISP_1.8V */
	regulator = regulator_get(NULL, "cam_isp");
	if (IS_ERR(regulator))
		return -ENODEV;
	if (regulator_is_enabled(regulator))
		ret = regulator_force_disable(regulator);
	regulator_put(regulator);
	CAM_CHECK_ERR(ret, "disable cam_isp");
	udelay(10);

	/* VT_CORE_1.5V */
	ret = gpio_direction_output(GPIO_VT_CAM_15V, 0);
	CAM_CHECK_ERR(ret, "output vt_1.5v");
	udelay(10);

	/* CAM_SENSOR_A2.8V */
	ret = gpio_direction_output(GPIO_CAM_IO_EN, 0);
	CAM_CHECK_ERR(ret, "output io_en");
	udelay(10);

	/* CAM_ISP_CORE_1.2V */
	regulator = regulator_get(NULL, "cam_isp_core");
	if (IS_ERR(regulator))
		return -ENODEV;
	if (regulator_is_enabled(regulator))
		ret = regulator_force_disable(regulator);
	regulator_put(regulator);
	CAM_CHECK_ERR(ret, "disable isp_core");

	if (system_rev >= 9) {
		gpio_direction_input(VT_CAM_SDA_18V);
		s3c_gpio_setpull(VT_CAM_SDA_18V, S3C_GPIO_PULL_DOWN);
		gpio_direction_input(VT_CAM_SCL_18V);
		s3c_gpio_setpull(VT_CAM_SCL_18V, S3C_GPIO_PULL_DOWN);
	}

	gpio_free(GPIO_CAM_VGA_nRST);
	gpio_free(GPIO_CAM_VGA_nSTBY);
	gpio_free(GPIO_VT_CAM_15V);
	gpio_free(GPIO_CAM_IO_EN);

	return 0;
}

static int s5k5bafx_power(int onoff)
{
	int ret = 0;

	printk(KERN_INFO "%s(): %s\n", __func__, onoff ? "on" : "down");
	if (onoff) {
		ret = s5k5bafx_power_on();
		if (unlikely(ret))
			goto error_out;
	} else {
		ret = s5k5bafx_power_off();
		/* s3c_i2c0_force_stop(); *//* DSLIM. Should be implemented */
	}

	ret = s3c_csis_power(onoff);

error_out:
	return ret;
}

static struct s5k5bafx_platform_data s5k5bafx_plat = {
	.default_width = 640,
	.default_height = 480,
	.pixelformat = V4L2_PIX_FMT_UYVY,
	.freq = 24000000,
	.is_mipi = 1,
};

static struct i2c_board_info s5k5bafx_i2c_info = {
	I2C_BOARD_INFO("S5K5BAFX", 0x5A >> 1),
	.platform_data = &s5k5bafx_plat,
};

static struct s3c_platform_camera s5k5bafx = {
	.id = CAMERA_CSI_D,
	.type = CAM_TYPE_MIPI,
	.fmt = ITU_601_YCBCR422_8BIT,
	.order422 = CAM_ORDER422_8BIT_CBYCRY,
	.mipi_lanes = 1,
	.mipi_settle = 6,
	.mipi_align = 32,

	.get_i2c_busnum = s5k5bafx_get_i2c_busnum,
	.info = &s5k5bafx_i2c_info,
	.pixelformat = V4L2_PIX_FMT_UYVY,
	.srclk_name = "xusbxti",
	.clk_name = "sclk_cam0",
	.clk_rate = 24000000,
	.line_length = 640,
	.width = 640,
	.height = 480,
	.window = {
		.left = 0,
		.top = 0,
		.width = 640,
		.height = 480,
	},

	/* Polarity */
	.inv_pclk = 0,
	.inv_vsync = 1,
	.inv_href = 0,
	.inv_hsync = 0,
	.reset_camera = 0,
	.initialized = 0,
	.cam_power = s5k5bafx_power,
};

/*******************************************************************************
 * Writeback driver for FIMC
 ******************************************************************************/
static int get_i2c_busnum_writeback(void)
{
	return 0;
}

static struct i2c_board_info writeback_i2c_info = {
	I2C_BOARD_INFO("WriteBack", 0x0),
};

static struct s3c_platform_camera writeback = {
	.id = CAMERA_WB,
	.fmt = ITU_601_YCBCR422_8BIT,
	.order422 = CAM_ORDER422_8BIT_CBYCRY,
	.get_i2c_busnum = get_i2c_busnum_writeback,
	.info = &writeback_i2c_info,
	.pixelformat = V4L2_PIX_FMT_YUV444,
	.line_length = 800,
	.width = 480,
	.height = 800,
	.window = {
		.left = 0,
		.top = 0,
		.width = 480,
		.height = 800,
	},

	.initialized = 0,
};

void cam_cfg_gpio(struct platform_device *pdev)
{
	int ret = 0;
	printk(KERN_INFO "\n\n\n%s: pdev->id=%d\n", __func__, pdev->id);

	if (pdev->id != 0)
		return;

	if (system_rev >= 9) {
		/* Rev0.9 */
		ret = gpio_direction_input(VT_CAM_SDA_18V);
		CAM_CHECK_ERR(ret, "VT_CAM_SDA_18V");
		s3c_gpio_setpull(VT_CAM_SDA_18V, S3C_GPIO_PULL_DOWN);

		ret = gpio_direction_input(VT_CAM_SCL_18V);
		CAM_CHECK_ERR(ret, "VT_CAM_SCL_18V");
		s3c_gpio_setpull(VT_CAM_SCL_18V, S3C_GPIO_PULL_DOWN);
	}
}

/* Interface setting */
static struct s3c_platform_fimc fimc_plat = {
	.default_cam = CAMERA_CSI_C,
	.camera = {
		&m5mo,
		&s5k5bafx,
		&writeback,
	},
	.hw_ver = 0x51,
	.cfg_gpio = cam_cfg_gpio,
};

/*******************************************************************************
 * HSMMC
 ******************************************************************************/
static struct s3c_sdhci_platdata exynos4_hsmmc2_pdata __initdata = {
	.cd_type = S3C_SDHCI_CD_GPIO,
	.clk_type = S3C_SDHCI_CLK_DIV_EXTERNAL,
	.ext_cd_gpio = EXYNOS4_GPX3(4),
	.ext_cd_gpio_invert = 1,
	.host_caps = MMC_CAP_4_BIT_DATA |
				MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED |
				MMC_CAP_DISABLE,
	.max_width		= 4,
	.cfg_gpio = exynos4_setup_sdhci2_cfg_gpio,
	.vmmc_name = "vtf_2.8v",
};

static struct s3c_sdhci_platdata exynos4_hsmmc3_pdata __initdata = {
	.cd_type = S3C_SDHCI_CD_NONE,
	.clk_type = S3C_SDHCI_CLK_DIV_EXTERNAL,
	.max_width = 4,
	.host_caps		= MMC_CAP_4_BIT_DATA |
				MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED,
	.cfg_gpio = exynos4_setup_sdhci3_cfg_gpio,
	
	.pm_flags = S3C_SDHCI_PM_IGNORE_SUSPEND_RESUME,
	.cfg_card = exynos4_setup_sdhci_cfg_card,
};

static struct s3c_mshci_platdata exynos4_mshc_pdata __initdata = {
	.cd_type = S3C_MSHCI_CD_PERMANENT,
	.max_width = 8,
	.host_caps = MMC_CAP_8_BIT_DATA | MMC_CAP_1_8V_DDR |
			MMC_CAP_UHS_DDR50 | MMC_CAP_CMD23,
	.int_power_gpio		= GPIO_XMMC0_CDn,
};

/******************************************************************************
 * FIMG 2D
 *****************************************************************************/
static struct fimg2d_platdata fimg2d_data __initdata = {
	.hw_ver = 30,
	.parent_clkname = "mout_g2d0",
	.clkname = "sclk_fimg2d",
	.gate_clkname = "fimg2d",
	.clkrate = 267 * 1000000,	/* 266 Mhz */
};

/*******************************************************************************
 * LD9040 FB
 ******************************************************************************/
unsigned int lcdtype;
static int __init lcdtype_setup(char *str)
{
	get_option(&str, &lcdtype);
	return 1;
}
__setup("lcdtype=", lcdtype_setup);

unsigned int ld9040_lcdtype;
static int __init ld9040_lcdtype_setup(char *str)
{
	get_option(&str, &ld9040_lcdtype);
	return 1;
}

__setup("ld9040.get_lcdtype=0x", ld9040_lcdtype_setup);

static int lcd_cfg_gpio(void)
{
	int i, f3_end = 4;

	for (i = 0; i < 8; i++) {
		/* set GPF0,1,2[0:7] for RGB Interface and Data line (32bit) */
		s3c_gpio_cfgpin(EXYNOS4_GPF0(i), S3C_GPIO_SFN(2));
		s3c_gpio_setpull(EXYNOS4_GPF0(i), S3C_GPIO_PULL_NONE);

	}
	for (i = 0; i < 8; i++) {
		s3c_gpio_cfgpin(EXYNOS4_GPF1(i), S3C_GPIO_SFN(2));
		s3c_gpio_setpull(EXYNOS4_GPF1(i), S3C_GPIO_PULL_NONE);
	}

	for (i = 0; i < 8; i++) {
		s3c_gpio_cfgpin(EXYNOS4_GPF2(i), S3C_GPIO_SFN(2));
		s3c_gpio_setpull(EXYNOS4_GPF2(i), S3C_GPIO_PULL_NONE);
	}

	for (i = 0; i < f3_end; i++) {
		s3c_gpio_cfgpin(EXYNOS4_GPF3(i), S3C_GPIO_SFN(2));
		s3c_gpio_setpull(EXYNOS4_GPF3(i), S3C_GPIO_PULL_NONE);
	}

	/* drive strength to 2X */
	writel(0xaaaaaaaa, S5P_VA_GPIO + 0x18c);
	writel(0xaaaaaaaa, S5P_VA_GPIO + 0x1ac);
	writel(0xaaaaaaaa, S5P_VA_GPIO + 0x1cc);
	writel(readl(S5P_VA_GPIO + 0x1ec) | 0xaaaaaa, S5P_VA_GPIO + 0x1ec);

	/* MLCD_RST */
	s3c_gpio_cfgpin(EXYNOS4_GPY4(5), S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(EXYNOS4_GPY4(5), S3C_GPIO_PULL_NONE);

	/* LCD_nCS */
	s3c_gpio_cfgpin(EXYNOS4_GPY4(3), S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(EXYNOS4_GPY4(3), S3C_GPIO_PULL_NONE);
	/* LCD_SCLK */
	s3c_gpio_cfgpin(EXYNOS4_GPY3(1), S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(EXYNOS4_GPY3(1), S3C_GPIO_PULL_NONE);
	/* LCD_SDI */
	s3c_gpio_cfgpin(EXYNOS4_GPY3(3), S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(EXYNOS4_GPY3(3), S3C_GPIO_PULL_NONE);
	return 0;
}

static int lcd_power_on(struct lcd_device *ld, int enable)
{
	struct regulator *regulator;

	if (ld == NULL) {
		printk(KERN_ERR "lcd device object is NULL.\n");
		return 0;
	}

	if (enable) {
		regulator = regulator_get(NULL, "vlcd_3.0v");
		if (IS_ERR(regulator))
			return 0;

		regulator_enable(regulator);
		regulator_put(regulator);
	} else {
		regulator = regulator_get(NULL, "vlcd_3.0v");

		if (IS_ERR(regulator))
			return 0;

		if (regulator_is_enabled(regulator))
			regulator_force_disable(regulator);

		regulator_put(regulator);
	}

	return 1;
}

static int reset_lcd(struct lcd_device *ld)
{
	int reset_gpio = -1;
	int err;

	reset_gpio = EXYNOS4_GPY4(5);

	err = gpio_request(reset_gpio, "MLCD_RST");
	if (err) {
		printk(KERN_ERR "failed to request MLCD_RST for "
		       "lcd reset control\n");
		return err;
	}

	gpio_request(reset_gpio, "MLCD_RST");

	mdelay(10);
	gpio_direction_output(reset_gpio, 0);
	mdelay(10);
	gpio_direction_output(reset_gpio, 1);

	gpio_free(reset_gpio);

	return 1;
}

static int lcd_gpio_cfg_earlysuspend(struct lcd_device *ld)
{
	int reset_gpio = -1;
	int err;

	reset_gpio = EXYNOS4_GPY4(5);

	err = gpio_request(reset_gpio, "MLCD_RST");
	if (err) {
		printk(KERN_ERR "failed to request MLCD_RST for "
		       "lcd reset control\n");
		return err;
	}

	mdelay(10);
	gpio_direction_output(reset_gpio, 0);

	gpio_free(reset_gpio);

	return 0;
}

static int lcd_gpio_cfg_lateresume(struct lcd_device *ld)
{
	/* MLCD_RST */
	s3c_gpio_cfgpin(EXYNOS4_GPY4(5), S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(EXYNOS4_GPY4(5), S3C_GPIO_PULL_NONE);

	/* LCD_nCS */
	s3c_gpio_cfgpin(EXYNOS4_GPY4(3), S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(EXYNOS4_GPY4(3), S3C_GPIO_PULL_NONE);
	/* LCD_SCLK */
	s3c_gpio_cfgpin(EXYNOS4_GPY3(1), S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(EXYNOS4_GPY3(1), S3C_GPIO_PULL_NONE);
	/* LCD_SDI */
	s3c_gpio_cfgpin(EXYNOS4_GPY3(3), S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(EXYNOS4_GPY3(3), S3C_GPIO_PULL_NONE);
	return 0;
}

static struct s3cfb_lcd ld9040_info = {
	.width = 480,
	.height = 800,
	.p_width = 56,
	.p_height = 93,
	.bpp = 24,

	.freq = 60,
	.timing = {
		.h_fp = 16,
		.h_bp = 14,
		.h_sw = 2,
		.v_fp = 10,
		.v_fpe = 1,
		.v_bp = 4,
		.v_bpe = 1,
		.v_sw = 2,
	},
	.polarity = {
		.rise_vclk = 1,
		.inv_hsync = 1,
		.inv_vsync = 1,
		.inv_vden = 1,
	},
};

static struct lcd_platform_data ld9040_platform_data = {
	.reset = reset_lcd,
	.power_on = lcd_power_on,
	.gpio_cfg_earlysuspend = lcd_gpio_cfg_earlysuspend,
	.gpio_cfg_lateresume = lcd_gpio_cfg_lateresume,
	/* it indicates whether lcd panel is enabled from u-boot. */
	.lcd_enabled = 1,
	.reset_delay = 20,	/* 10ms */
	.power_on_delay = 20,	/* 20ms */
	.power_off_delay = 200,	/* 120ms */
	.pdata = &u1_panel_data,
};

#define LCD_BUS_NUM	3
#define DISPLAY_CS	EXYNOS4_GPY4(3)
static struct spi_board_info spi_board_info[] __initdata = {
	{
		.max_speed_hz = 1200000,
		.bus_num = LCD_BUS_NUM,
		.chip_select = 0,
		.mode = SPI_MODE_3,
		.controller_data = (void *)DISPLAY_CS,
	},
};

#define DISPLAY_CLK	EXYNOS4_GPY3(1)
#define DISPLAY_SI	EXYNOS4_GPY3(3)
static struct spi_gpio_platform_data lcd_spi_gpio_data = {
	.sck = DISPLAY_CLK,
	.mosi = DISPLAY_SI,
	.miso = SPI_GPIO_NO_MISO,
	.num_chipselect = 1,
};

static struct platform_device ld9040_spi_gpio = {
	.name = "spi_gpio",
	.id = LCD_BUS_NUM,
	.dev = {
		.parent = &s3c_device_fb.dev,
		.platform_data = &lcd_spi_gpio_data,
	},
};

static struct s3c_platform_fb fb_platform_data __initdata = {
	.hw_ver = 0x70,
	.clk_name = "fimd",
	.nr_wins = 5,
#ifdef CONFIG_FB_S5P_DEFAULT_WINDOW
	.default_win = CONFIG_FB_S5P_DEFAULT_WINDOW,
#else
	.default_win = 0,
#endif
	.swap = FB_SWAP_HWORD | FB_SWAP_WORD,
	.lcd = &ld9040_info,
};

/* reading with 3-WIRE SPI with GPIO */
static inline void setcs(u8 is_on)
{
	gpio_set_value(DISPLAY_CS, is_on);
}

static inline void setsck(u8 is_on)
{
	gpio_set_value(DISPLAY_CLK, is_on);
}

static inline void setmosi(u8 is_on)
{
	gpio_set_value(DISPLAY_SI, is_on);
}

static inline unsigned int getmiso(void)
{
	return !!gpio_get_value(DISPLAY_SI);
}

static inline void setmosi2miso(u8 is_on)
{
	if (is_on)
		s3c_gpio_cfgpin(DISPLAY_SI, S3C_GPIO_INPUT);
	else
		s3c_gpio_cfgpin(DISPLAY_SI, S3C_GPIO_OUTPUT);
}

struct spi_ops ops = {
	.setcs = setcs,
	.setsck = setsck,
	.setmosi = setmosi,
	.setmosi2miso = setmosi2miso,
	.getmiso = getmiso,
};

static void __init ld9040_fb_init(void)
{
	struct ld9040_panel_data *pdata;

	strcpy(spi_board_info[0].modalias, "ld9040");
	spi_board_info[0].platform_data = (void *)&ld9040_platform_data;

	lcdtype = max(ld9040_lcdtype, lcdtype);

	if (lcdtype == LCDTYPE_SM2_A2)
		ld9040_platform_data.pdata = &u1_panel_data_a2;
	else if (lcdtype == LCDTYPE_M2)
		ld9040_platform_data.pdata = &u1_panel_data_m2;

	pdata = ld9040_platform_data.pdata;
	pdata->ops = &ops;

	printk(KERN_INFO "%s :: lcdtype=%d\n", __func__, lcdtype);

	spi_register_board_info(spi_board_info, ARRAY_SIZE(spi_board_info));

	if (!ld9040_platform_data.lcd_enabled)
		lcd_cfg_gpio();
	s3cfb_set_platdata(&fb_platform_data);
}

/*******************************************************************************
 * Voltage Regulators
 ******************************************************************************/
static struct platform_device u1_regulator_consumer = {
	.name = "u1-regulator-consumer",
	.id = -1,
};

static struct regulator_consumer_supply ldo1_supply[] = {
	REGULATOR_SUPPLY("vadc_3.3v", NULL),
};

static struct regulator_consumer_supply ldo3_supply[] = {
	REGULATOR_SUPPLY("vusb_1.1v", "usb_otg"),
	REGULATOR_SUPPLY("vmipi_1.1v", "m5mo"),
	REGULATOR_SUPPLY("vmipi_1.1v", NULL),
};

static struct regulator_consumer_supply ldo4_supply[] = {
	REGULATOR_SUPPLY("vmipi_1.8v", NULL),
};

static struct regulator_consumer_supply ldo5_supply[] = {
	REGULATOR_SUPPLY("vhsic", NULL),
};

static struct regulator_consumer_supply ldo7_supply[] = {
	REGULATOR_SUPPLY("cam_isp", NULL),
};

static struct regulator_consumer_supply ldo8_supply[] = {
	REGULATOR_SUPPLY("vusb_3.3v", NULL),
};

#if defined(CONFIG_S5PV310_HI_ARMCLK_THAN_1_2GHZ)
static struct regulator_consumer_supply ldo10_supply[] = {
	REGULATOR_SUPPLY("vpll_1.2v", NULL),
};
#else
static struct regulator_consumer_supply ldo10_supply[] = {
	REGULATOR_SUPPLY("vpll_1.1v", NULL),
};
#endif

static struct regulator_consumer_supply ldo11_supply[] = {
	REGULATOR_SUPPLY("touchkey", NULL),
};

static struct regulator_consumer_supply ldo12_supply[] = {
	REGULATOR_SUPPLY("vt_cam_1.8v", NULL),
};

static struct regulator_consumer_supply ldo13_supply[] = {
	REGULATOR_SUPPLY("vlcd_3.0v", NULL),
};

static struct regulator_consumer_supply ldo14_supply[] = {
	REGULATOR_SUPPLY("vmotor", NULL),
};

static struct regulator_consumer_supply ldo15_supply[] = {
	REGULATOR_SUPPLY("vled", NULL),
};

static struct regulator_consumer_supply ldo16_supply[] = {
	REGULATOR_SUPPLY("cam_sensor_io", NULL),
};

static struct regulator_consumer_supply ldo17_supply[] = {
	REGULATOR_SUPPLY("vtf_2.8v", NULL),
};

static struct regulator_consumer_supply ldo18_supply[] = {
	REGULATOR_SUPPLY("touch_led", NULL),
};

static struct regulator_consumer_supply ldo21_supply[] = {
	REGULATOR_SUPPLY("vddq_m1m2", NULL),
};

static struct regulator_consumer_supply buck1_supply[] = {
	REGULATOR_SUPPLY("vdd_arm", NULL),
};

static struct regulator_consumer_supply buck2_supply[] = {
	REGULATOR_SUPPLY("vdd_int", NULL),
};

static struct regulator_consumer_supply buck3_supply[] = {
	REGULATOR_SUPPLY("vdd_g3d", NULL),
};

static struct regulator_consumer_supply buck4_supply[] = {
	REGULATOR_SUPPLY("cam_isp_core", NULL),
};

static struct regulator_consumer_supply buck7_supply[] = {
	REGULATOR_SUPPLY("vcc_sub", NULL),
};

static struct regulator_consumer_supply safeout1_supply[] = {
	REGULATOR_SUPPLY("safeout1", NULL),
};

static struct regulator_consumer_supply safeout2_supply[] = {
	REGULATOR_SUPPLY("safeout2", NULL),
};

static struct regulator_consumer_supply led_flash_supply[] = {
	REGULATOR_SUPPLY("led_flash", NULL),
};

static struct regulator_consumer_supply led_movie_supply[] = {
	REGULATOR_SUPPLY("led_movie", NULL),
};

#define REGULATOR_INIT(_ldo, _name, _min_uV, _max_uV, _always_on, _ops_mask, \
		       _disabled)					\
	static struct regulator_init_data _ldo##_init_data = {		\
		.constraints = {					\
			.name	= _name,				\
			.min_uV = _min_uV,				\
			.max_uV = _max_uV,				\
			.always_on	= _always_on,			\
			.boot_on	= _always_on,			\
			.apply_uV	= 1,				\
			.valid_ops_mask = _ops_mask,			\
			.state_mem	= {				\
				.disabled =				\
					(_disabled == -1 ? 0 : _disabled),\
				.enabled =				\
					(_disabled == -1 ? 0 : !(_disabled)),\
			}						\
		},							\
		.num_consumer_supplies = ARRAY_SIZE(_ldo##_supply),	\
		.consumer_supplies = &_ldo##_supply[0],			\
	};

REGULATOR_INIT(ldo1, "VADC_3.3V_C210", 3300000, 3300000, 1,
		REGULATOR_CHANGE_STATUS, 1);
REGULATOR_INIT(ldo3, "VUSB_1.1V", 1100000, 1100000, 1,
		REGULATOR_CHANGE_STATUS, 1);
REGULATOR_INIT(ldo4, "VMIPI_1.8V", 1800000, 1800000, 1,
		REGULATOR_CHANGE_STATUS, 1);
REGULATOR_INIT(ldo5, "VHSIC_1.2V", 1200000, 1200000, 1,
		REGULATOR_CHANGE_STATUS, 1);
REGULATOR_INIT(ldo7, "CAM_ISP_1.8V", 1800000, 1800000, 0,
		REGULATOR_CHANGE_STATUS, 1);
REGULATOR_INIT(ldo8, "VUSB_3.3V", 3300000, 3300000, 1,
		REGULATOR_CHANGE_STATUS, 1);
#if defined(CONFIG_S5PV310_HI_ARMCLK_THAN_1_2GHZ)
REGULATOR_INIT(ldo10, "VPLL_1.2V", 1200000, 1200000, 1,
		REGULATOR_CHANGE_STATUS, 1);
#else
REGULATOR_INIT(ldo10, "VPLL_1.1V", 1100000, 1100000, 1,
		REGULATOR_CHANGE_STATUS, 1);
#endif
REGULATOR_INIT(ldo11, "TOUCH_2.8V", 2800000, 2800000, 0,
		REGULATOR_CHANGE_STATUS, 1);
REGULATOR_INIT(ldo12, "VT_CAM_1.8V", 1800000, 1800000, 0,
		REGULATOR_CHANGE_STATUS, 1);
REGULATOR_INIT(ldo13, "VCC_3.0V_LCD", 3000000, 3000000, 1,
		REGULATOR_CHANGE_STATUS, 1);
REGULATOR_INIT(ldo14, "VCC_2.8V_MOTOR", 2800000, 2800000, 0,
		REGULATOR_CHANGE_STATUS, 1);
REGULATOR_INIT(ldo15, "LED_A_2.8V", 2800000, 2800000, 0,
		REGULATOR_CHANGE_STATUS, -1);
REGULATOR_INIT(ldo16, "CAM_SENSOR_IO_1.8V", 1800000, 1800000, 0,
		REGULATOR_CHANGE_STATUS, 1);
REGULATOR_INIT(ldo17, "VTF_2.8V", 2800000, 2800000, 0,
		REGULATOR_CHANGE_STATUS, 1);
REGULATOR_INIT(ldo18, "TOUCH_LED_3.3V", 3000000, 3300000, 0,
		REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_VOLTAGE, 1);
REGULATOR_INIT(ldo21, "VDDQ_M1M2_1.2V", 1200000, 1200000, 1,
		REGULATOR_CHANGE_STATUS, 1);


static struct regulator_init_data buck1_init_data = {
	.constraints	= {
		.name		= "vdd_arm range",
		.min_uV		= 650000,
		.max_uV		= 2225000,
		.always_on	= 1,
		.boot_on	= 1,
		.apply_uV	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE |
				  REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.mode		= REGULATOR_MODE_NORMAL,
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &buck1_supply[0],
};

static struct regulator_init_data buck2_init_data = {
	.constraints	= {
		.name		= "vdd_int range",
		.min_uV		= 650000,
		.max_uV		= 2225000,
		.always_on	= 1,
		.boot_on	= 1,
		.apply_uV	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE |
				  REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.mode		= REGULATOR_MODE_NORMAL,
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &buck2_supply[0],
};

static struct regulator_init_data buck3_init_data = {
	.constraints	= {
		.name		= "G3D_1.1V",
		.min_uV		= 900000,
		.max_uV		= 1200000,
		.always_on	= 0,
		.boot_on	= 0,
		.apply_uV	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE |
				  REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.mode		= REGULATOR_MODE_NORMAL,
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &buck3_supply[0],
};

static struct regulator_init_data buck4_init_data = {
	.constraints	= {
		.name		= "CAM_ISP_CORE_1.2V",
		.min_uV		= 1200000,
		.max_uV		= 1200000,
		.apply_uV	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &buck4_supply[0],
};

static struct regulator_init_data buck5_init_data = {
	.constraints	= {
		.name		= "VMEM_1.2V",
		.min_uV		= 1200000,
		.max_uV		= 1200000,
		.apply_uV	= 1,
		.always_on	= 1,
		.state_mem	= {
			.uV	= 1200000,
			.mode	= REGULATOR_MODE_NORMAL,
			.enabled = 1,
		},
	},
};

static struct regulator_init_data buck7_init_data = {
	.constraints	= {
		.name		= "VCC_SUB_2.0V",
		.min_uV		= 2000000,
		.max_uV		= 2000000,
		.apply_uV	= 1,
		.always_on	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.enabled = 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &buck7_supply[0],
};

static struct regulator_init_data safeout1_init_data = {
	.constraints	= {
		.name		= "safeout1 range",
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.boot_on	= 1,
		.state_mem	= {
			.enabled = 1,
		},
	},
	.num_consumer_supplies	= ARRAY_SIZE(safeout1_supply),
	.consumer_supplies	= safeout1_supply,
};

static struct regulator_init_data safeout2_init_data = {
	.constraints	= {
		.name		= "safeout2 range",
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.enabled = 1,
		},
	},
	.num_consumer_supplies	= ARRAY_SIZE(safeout2_supply),
	.consumer_supplies	= safeout2_supply,
};

static struct regulator_init_data led_flash_init_data = {
	.constraints = {
		.name	= "FLASH_CUR",
		.min_uA = 23440,
		.max_uA = 750080,
		.valid_ops_mask	= REGULATOR_CHANGE_CURRENT |
				  REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = &led_flash_supply[0],
};

static struct regulator_init_data led_movie_init_data = {
	.constraints = {
		.name	= "MOVIE_CUR",
		.min_uA = 15625,
		.max_uA = 250000,
		.valid_ops_mask	= REGULATOR_CHANGE_CURRENT |
				  REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = &led_movie_supply[0],
};

static struct max8997_regulator_data max8997_regulators[] = {
	{ MAX8997_LDO1,	 &ldo1_init_data, NULL, },
	{ MAX8997_LDO3,	 &ldo3_init_data, NULL, },
	{ MAX8997_LDO4,	 &ldo4_init_data, NULL, },
	{ MAX8997_LDO5,	 &ldo5_init_data, NULL, },
	{ MAX8997_LDO7,	 &ldo7_init_data, NULL, },
	{ MAX8997_LDO8,	 &ldo8_init_data, NULL, },
	{ MAX8997_LDO10, &ldo10_init_data, NULL, },
	{ MAX8997_LDO11, &ldo11_init_data, NULL, },
	{ MAX8997_LDO12, &ldo12_init_data, NULL, },
	{ MAX8997_LDO13, &ldo13_init_data, NULL, },
	{ MAX8997_LDO14, &ldo14_init_data, NULL, },
	{ MAX8997_LDO15, &ldo15_init_data, NULL, },
	{ MAX8997_LDO16, &ldo16_init_data, NULL, },
	{ MAX8997_LDO17, &ldo17_init_data, NULL, },
	{ MAX8997_LDO18, &ldo18_init_data, NULL, },
	{ MAX8997_LDO21, &ldo21_init_data, NULL, },
	{ MAX8997_BUCK1, &buck1_init_data, NULL, },
	{ MAX8997_BUCK2, &buck2_init_data, NULL, },
	{ MAX8997_BUCK3, &buck3_init_data, NULL, },
	{ MAX8997_BUCK4, &buck4_init_data, NULL, },
	{ MAX8997_BUCK5, &buck5_init_data, NULL, },
	{ MAX8997_BUCK7, &buck7_init_data, NULL, },
	{ MAX8997_ESAFEOUT1, &safeout1_init_data, NULL, },
	{ MAX8997_ESAFEOUT2, &safeout2_init_data, NULL, },
	{ MAX8997_FLASH_CUR, &led_flash_init_data, NULL, },
	{ MAX8997_MOVIE_CUR, &led_movie_init_data, NULL, },
};

static struct max8997_power_data max8997_power = {
	.batt_detect = 1,
};

static struct max8997_motor_data max8997_motor = {
	.max_timeout = 10000,
	.duty = 37641,
	.period = 38022,
	.init_hw = NULL,
	.motor_en = NULL,
	.pwm_id = 1,
};

static int max8997_muic_set_safeout(int path)
{
	struct regulator *regulator;

	if (path == CP_USB_MODE) {
		regulator = regulator_get(NULL, "safeout1");
		if (IS_ERR(regulator))
			return -ENODEV;
		if (regulator_is_enabled(regulator))
			regulator_force_disable(regulator);
		regulator_put(regulator);

		regulator = regulator_get(NULL, "safeout2");
		if (IS_ERR(regulator))
			return -ENODEV;
		if (!regulator_is_enabled(regulator))
			regulator_enable(regulator);
		regulator_put(regulator);
	} else {
		/* AP_USB_MODE || AUDIO_MODE */
		regulator = regulator_get(NULL, "safeout1");
		if (IS_ERR(regulator))
			return -ENODEV;
		if (!regulator_is_enabled(regulator))
			regulator_enable(regulator);
		regulator_put(regulator);

		regulator = regulator_get(NULL, "safeout2");
		if (IS_ERR(regulator))
			return -ENODEV;
		if (regulator_is_enabled(regulator))
			regulator_force_disable(regulator);
		regulator_put(regulator);
	}

	return 0;
}

static struct charging_status_callbacks {
	void (*tsp_set_charging_cable) (int type);
} charging_cbs;

bool is_cable_attached;
static int connected_cable_type = CABLE_TYPE_NONE;

static int max8997_muic_charger_cb(int cable_type)
{
	struct power_supply *psy = power_supply_get_by_name("battery");
	union power_supply_propval value;

	connected_cable_type = cable_type;

	if (!psy) {
		pr_err("%s: fail to get battery ps\n", __func__);
		return -ENODEV;
	}

	switch (cable_type) {
	case CABLE_TYPE_NONE:
	case CABLE_TYPE_OTG:
	case CABLE_TYPE_JIG_UART_OFF:
	case CABLE_TYPE_MHL:
		value.intval = POWER_SUPPLY_TYPE_BATTERY;
		is_cable_attached = false;
		break;
	case CABLE_TYPE_USB:
	case CABLE_TYPE_JIG_USB_OFF:
	case CABLE_TYPE_JIG_USB_ON:
		value.intval = POWER_SUPPLY_TYPE_USB;
		is_cable_attached = true;
		break;
	case CABLE_TYPE_MHL_VB:
		value.intval = POWER_SUPPLY_TYPE_MISC;
		is_cable_attached = true;
		break;
	case CABLE_TYPE_TA:
	case CABLE_TYPE_CARDOCK:
	case CABLE_TYPE_DESKDOCK:
	case CABLE_TYPE_JIG_UART_OFF_VB:
		value.intval = POWER_SUPPLY_TYPE_MAINS;
		is_cable_attached = true;
		break;
	default:
		pr_err("%s: invalid type:%d\n", __func__, cable_type);
		return -EINVAL;
	}

	if (charging_cbs.tsp_set_charging_cable)
		charging_cbs.tsp_set_charging_cable(value.intval);

	return psy->set_property(psy, POWER_SUPPLY_PROP_ONLINE, &value);
}

static void usb_otg_accessory_power(int enable)
{
	u8 on = (u8)!!enable;

	gpio_request(GPIO_USB_OTG_EN, "USB_OTG_EN");
	gpio_direction_output(GPIO_USB_OTG_EN, on);
	gpio_free(GPIO_USB_OTG_EN);
	pr_info("%s: otg accessory power = %d\n", __func__, on);
}

static struct host_notifier_platform_data host_notifier_pdata = {
	.ndev.name	= "usb_otg",
	.booster	= usb_otg_accessory_power,
};

struct platform_device host_notifier_device = {
	.name = "host_notifier",
	.dev.platform_data = &host_notifier_pdata,
};

#include "u1-otg.c"
static void max8997_muic_usb_cb(u8 usb_mode)
{
	struct s3c_udc *udc = platform_get_drvdata(&s3c_device_usbgadget);
	int ret = 0;

	pr_info("otg %s: usb mode=%d\n", __func__, usb_mode);


	if (udc) {
		if (usb_mode == USB_OTGHOST_ATTACHED) {
			usb_otg_accessory_power(1);
			max8997_muic_charger_cb(CABLE_TYPE_OTG);
		}

		ret = c210_change_usb_mode(udc, usb_mode);
		if (ret < 0)
			pr_err("%s: fail to change mode!!!\n", __func__);

		if (usb_mode == USB_OTGHOST_DETACHED)
			usb_otg_accessory_power(0);
	} else
		pr_info("otg error s3c_udc is null.\n");
}

static void max8997_muic_mhl_cb(int attached)
{
	pr_info("%s(%d)\n", __func__, attached);

	if (attached == MAX8997_MUIC_ATTACHED) {
		sii9234_mhl_detection_sched();
	}
}

static bool max8997_muic_is_mhl_attached(void)
{
	int val;

	gpio_request(GPIO_MHL_SEL, "MHL_SEL");
	val = gpio_get_value(GPIO_MHL_SEL);
	gpio_free(GPIO_MHL_SEL);

	return !!val;
}

static struct switch_dev switch_dock = {
	.name = "dock",
};

static void max8997_muic_deskdock_cb(bool attached)
{
	if (attached)
		switch_set_state(&switch_dock, 1);
	else
		switch_set_state(&switch_dock, 0);
}

static void max8997_muic_cardock_cb(bool attached)
{
	if (attached)
		switch_set_state(&switch_dock, 2);
	else
		switch_set_state(&switch_dock, 0);
}

static void max8997_muic_init_cb(void)
{
	int ret;

	/* for CarDock, DeskDock */
	ret = switch_dev_register(&switch_dock);
	if (ret < 0)
		pr_err("Failed to register dock switch. %d\n", ret);
}

static int max8997_muic_cfg_uart_gpio(void)
{
	int val, path;

	val = gpio_get_value(GPIO_UART_SEL);
	path = val ? UART_PATH_AP : UART_PATH_CP;

	pr_info("%s: path=%d\n", __func__, path);
	return path;
}

static void max8997_muic_jig_uart_cb(int path)
{
	int val;

	val = path == UART_PATH_AP ? GPIO_LEVEL_HIGH : GPIO_LEVEL_LOW;
	gpio_set_value(GPIO_UART_SEL, val);
	pr_info("%s: val:%d\n", __func__, val);
}

static int max8997_muic_host_notify_cb(int enable)
{
	struct host_notify_dev *ndev = &host_notifier_pdata.ndev;

	if (ndev) {
		ndev->booster = enable ? NOTIFY_POWER_ON : NOTIFY_POWER_OFF;
		pr_info("%s: mode %d, enable %d\n", __func__,
				ndev->mode, enable);
		return ndev->mode;
	} else
		pr_info("%s: host_notify_dev is null, enable %d\n",
				__func__, enable);

	return -1;
}

static struct max8997_muic_data max8997_muic = {
	.usb_cb = max8997_muic_usb_cb,
	.charger_cb = max8997_muic_charger_cb,
	.mhl_cb = max8997_muic_mhl_cb,
	.is_mhl_attached = max8997_muic_is_mhl_attached,
	.set_safeout = max8997_muic_set_safeout,
	.init_cb = max8997_muic_init_cb,
	.deskdock_cb = max8997_muic_deskdock_cb,
	.cardock_cb = max8997_muic_cardock_cb,
	.cfg_uart_gpio = max8997_muic_cfg_uart_gpio,
	.jig_uart_cb = max8997_muic_jig_uart_cb,
	.host_notify_cb = max8997_muic_host_notify_cb,
	.gpio_usb_sel = GPIO_USB_SEL,
};

static struct max8997_buck1_dvs_funcs *buck1_dvs_funcs;

void max8997_set_arm_voltage_table(int *voltage_table, int arr_size)
{
	pr_info("%s\n", __func__);
	if (buck1_dvs_funcs && buck1_dvs_funcs->set_buck1_dvs_table)
		buck1_dvs_funcs->set_buck1_dvs_table(buck1_dvs_funcs,
			 voltage_table, arr_size);
}

static void max8997_register_buck1dvs_funcs(struct max8997_buck1_dvs_funcs *ptr)
{
	buck1_dvs_funcs = ptr;
}


static struct max8997_platform_data exynos4_max8997_info = {
	.num_regulators = ARRAY_SIZE(max8997_regulators),
	.regulators	= &max8997_regulators[0],
	.irq_base	= IRQ_BOARD_START,
	.wakeup		= 1,
	.buck1_gpiodvs	= false,
	.buck1_max_vol	= 1350000,
	.buck2_max_vol	= 1150000,
	.buck5_max_vol	= 1200000,
	.buck_set1 = GPIO_BUCK1_EN_A,
	.buck_set2 = GPIO_BUCK1_EN_B,
	.buck_set3 = GPIO_BUCK2_EN,
	.buck_ramp_en = true,
	.buck_ramp_delay = 10,		/* 10.00mV /us (default) */
	.flash_cntl_val = 0x5F,	/* Flash safety timer duration: 800msec,
					   Maximum timer mode */
	.power = &max8997_power,
	.motor = &max8997_motor,
	.muic = &max8997_muic,
	.register_buck1_dvs_funcs = max8997_register_buck1dvs_funcs,
};

/******************************************************************************
 * Bluetooth
 *****************************************************************************/
static struct platform_device bcm4330_bluetooth_device = {
	.name = "bcm4330_bluetooth",
	.id = -1,
};

/*******************************************************************************
 * Sound
 ******************************************************************************/
#define SYSTEM_REV_SND 0x09

static DEFINE_SPINLOCK(mic_bias_lock);
static bool mc1n2_mainmic_bias;
static bool mc1n2_submic_bias;

static void set_shared_mic_bias(void)
{
	if (system_rev >= 0x03)
		gpio_set_value(GPIO_MIC_BIAS_EN, mc1n2_mainmic_bias
			       || mc1n2_submic_bias);
	else
		gpio_set_value(GPIO_EAR_MIC_BIAS_EN, mc1n2_mainmic_bias
			       || mc1n2_submic_bias);
}

void sec_set_sub_mic_bias(bool on)
{
	if (system_rev < SYSTEM_REV_SND) {
		unsigned long flags;
		spin_lock_irqsave(&mic_bias_lock, flags);
		mc1n2_submic_bias = on;
		set_shared_mic_bias();
		spin_unlock_irqrestore(&mic_bias_lock, flags);
	} else
		gpio_set_value(GPIO_SUB_MIC_BIAS_EN, on);

}

void sec_set_main_mic_bias(bool on)
{
	if (system_rev < SYSTEM_REV_SND) {
		unsigned long flags;
		spin_lock_irqsave(&mic_bias_lock, flags);
		mc1n2_mainmic_bias = on;
		set_shared_mic_bias();
		spin_unlock_irqrestore(&mic_bias_lock, flags);
	} else
		gpio_set_value(GPIO_MIC_BIAS_EN, on);
}

void sec_set_ldo1_constraints(int disabled)
{
}

static struct mc1n2_platform_data mc1n2_pdata = {
	.set_main_mic_bias = sec_set_main_mic_bias,
	.set_sub_mic_bias = sec_set_sub_mic_bias,
	.set_adc_power_contraints = sec_set_ldo1_constraints,
};

static void u1_sound_init(void)
{
	int err;

	err = gpio_request(GPIO_MIC_BIAS_EN, "GPE1");
	if (err) {
		pr_err(KERN_ERR "MIC_BIAS_EN GPIO set error!\n");
		return;
	}
	gpio_direction_output(GPIO_MIC_BIAS_EN, 1);
	gpio_set_value(GPIO_MIC_BIAS_EN, 0);
	gpio_free(GPIO_MIC_BIAS_EN);

	err = gpio_request(GPIO_EAR_MIC_BIAS_EN, "GPE2");
	if (err) {
		pr_err(KERN_ERR "EAR_MIC_BIAS_EN GPIO set error!\n");
		return;
	}
	gpio_direction_output(GPIO_EAR_MIC_BIAS_EN, 1);
	gpio_set_value(GPIO_EAR_MIC_BIAS_EN, 0);
	gpio_free(GPIO_EAR_MIC_BIAS_EN);

	if (system_rev >= SYSTEM_REV_SND) {
		err = gpio_request(GPIO_SUB_MIC_BIAS_EN, "submic_bias");
		if (err) {
			pr_err(KERN_ERR "SUB_MIC_BIAS_EN GPIO set error!\n");
			return;
		}
		gpio_direction_output(GPIO_SUB_MIC_BIAS_EN, 0);
		gpio_free(GPIO_SUB_MIC_BIAS_EN);
	}
}

/*******************************************************************************
 * Power Supply
 ******************************************************************************/
static int c1_charger_topoff_cb(void)
{
	struct power_supply *psy = power_supply_get_by_name("battery");
	union power_supply_propval value;

	if (!psy) {
		pr_err("%s: fail to get battery ps\n", __func__);
		return -ENODEV;
	}

	value.intval = POWER_SUPPLY_STATUS_FULL;
	return psy->set_property(psy, POWER_SUPPLY_PROP_STATUS, &value);
}

static int max8922_cfg_gpio(void)
{
	if (system_rev < HWREV_FOR_BATTERY)
		return -ENODEV;

	s3c_gpio_cfgpin(GPIO_CHG_EN, S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(GPIO_CHG_EN, S3C_GPIO_PULL_NONE);
	gpio_set_value(GPIO_CHG_EN, GPIO_LEVEL_LOW);

	s3c_gpio_cfgpin(GPIO_CHG_ING_N, S3C_GPIO_INPUT);
	s3c_gpio_setpull(GPIO_CHG_ING_N, S3C_GPIO_PULL_NONE);

	s3c_gpio_cfgpin(GPIO_TA_nCONNECTED, S3C_GPIO_INPUT);
	s3c_gpio_setpull(GPIO_TA_nCONNECTED, S3C_GPIO_PULL_NONE);

	return 0;
}

static struct max8922_platform_data max8922_pdata = {
	.topoff_cb = c1_charger_topoff_cb,
	.cfg_gpio = max8922_cfg_gpio,
	.gpio_chg_en = GPIO_CHG_EN,
	.gpio_chg_ing = GPIO_CHG_ING_N,
	.gpio_ta_nconnected = GPIO_TA_nCONNECTED,
};

static struct platform_device max8922_device_charger = {
	.name = "max8922-charger",
	.id = -1,
	.dev.platform_data = &max8922_pdata,
};

/* temperature table for ADC 6 */
static struct sec_bat_adc_table_data temper_table[] = {
	{  165,	 800 },
	{  171,	 790 },
	{  177,	 780 },
	{  183,	 770 },
	{  189,	 760 },
	{  196,	 750 },
	{  202,	 740 },
	{  208,	 730 },
	{  214,	 720 },
	{  220,	 710 },
	{  227,	 700 },
	{  237,	 690 },
	{  247,	 680 },
	{  258,	 670 },
	{  269,	 660 },
	{  281,	 650 },
	{  296,	 640 },
	{  311,	 630 },
	{  326,	 620 },
	{  341,	 610 },
	{  356,	 600 },
	{  370,	 590 },
	{  384,	 580 },
	{  398,	 570 },
	{  412,	 560 },
	{  427,	 550 },
	{  443,	 540 },
	{  457,	 530 },
	{  471,	 520 },
	{  485,	 510 },
	{  498,	 500 },
	{  507,	 490 },
	{  516,	 480 },
	{  525,	 470 },
	{  535,	 460 },
	{  544,	 450 },
	{  553,	 440 },
	{  562,	 430 },
	{  579,	 420 },
	{  596,	 410 },
	{  613,	 400 },
	{  630,	 390 },
	{  648,	 380 },
	{  665,	 370 },
	{  684,	 360 },
	{  702,	 350 },
	{  726,	 340 },
	{  750,	 330 },
	{  774,	 320 },
	{  798,	 310 },
	{  821,	 300 },
	{  844,	 290 },
	{  867,	 280 },
	{  891,	 270 },
	{  914,	 260 },
	{  937,	 250 },
	{  960,	 240 },
	{  983,	 230 },
	{ 1007,	 220 },
	{ 1030,	 210 },
	{ 1054,	 200 },
	{ 1083,	 190 },
	{ 1113,	 180 },
	{ 1143,	 170 },
	{ 1173,	 160 },
	{ 1202,	 150 },
	{ 1232,	 140 },
	{ 1262,	 130 },
	{ 1291,	 120 },
	{ 1321,	 110 },
	{ 1351,	 100 },
	{ 1357,	  90 },
	{ 1363,	  80 },
	{ 1369,	  70 },
	{ 1375,	  60 },
	{ 1382,	  50 },
	{ 1402,	  40 },
	{ 1422,	  30 },
	{ 1442,	  20 },
	{ 1462,	  10 },
	{ 1482,	   0 },
	{ 1519,	 -10 },
	{ 1528,	 -20 },
	{ 1546,	 -30 },
	{ 1563,	 -40 },
	{ 1587,	 -50 },
	{ 1601,	 -60 },
	{ 1614,	 -70 },
	{ 1625,  -80 },
	{ 1641,  -90 },
	{ 1663, -100 },
	{ 1678, -110 },
	{ 1693, -120 },
	{ 1705, -130 },
	{ 1720, -140 },
	{ 1736, -150 },
	{ 1751, -160 },
	{ 1767, -170 },
	{ 1782, -180 },
	{ 1798, -190 },
	{ 1815, -200 },
};
/* temperature table for ADC 7 */
static struct sec_bat_adc_table_data temper_table_ADC7[] = {
	{  193,	 800 },
	{  200,	 790 },
	{  207,	 780 },
	{  215,	 770 },
	{  223,	 760 },
	{  230,	 750 },
	{  238,	 740 },
	{  245,	 730 },
	{  252,	 720 },
	{  259,	 710 },
	{  266,	 700 },
	{  277,	 690 },
	{  288,	 680 },
	{  300,	 670 },
	{  311,	 660 },
	{  326,	 650 },
	{  340,	 640 },
	{  354,	 630 },
	{  368,	 620 },
	{  382,	 610 },
	{  397,	 600 },
	{  410,	 590 },
	{  423,	 580 },
	{  436,	 570 },
	{  449,	 560 },
	{  462,	 550 },
	{  475,	 540 },
	{  488,	 530 },
	{  491,	 520 },
	{  503,	 510 },
	{  535,	 500 },
	{  548,	 490 },
	{  562,	 480 },
	{  576,	 470 },
	{  590,	 460 },
	{  603,	 450 },
	{  616,	 440 },
	{  630,	 430 },
	{  646,	 420 },
	{  663,	 410 },
	{  679,	 400 },
	{  696,	 390 },
	{  712,	 380 },
	{  728,	 370 },
	{  745,	 360 },
	{  762,	 350 },
	{  784,	 340 },
	{  806,	 330 },
	{  828,	 320 },
	{  850,	 310 },
	{  872,	 300 },
	{  895,	 290 },
	{  919,	 280 },
	{  942,	 270 },
	{  966,	 260 },
	{  989,	 250 },
	{ 1013,	 240 },
	{ 1036,	 230 },
	{ 1060,	 220 },
	{ 1083,	 210 },
	{ 1107,	 200 },
	{ 1133,	 190 },
	{ 1159,	 180 },
	{ 1186,	 170 },
	{ 1212,	 160 },
	{ 1238,	 150 },
	{ 1265,	 140 },
	{ 1291,	 130 },
	{ 1316,	 120 },
	{ 1343,	 110 },
	{ 1370,	 100 },
	{ 1381,	  90 },
	{ 1393,	  80 },
	{ 1404,	  70 },
	{ 1416,	  60 },
	{ 1427,	  50 },
	{ 1453,	  40 },
	{ 1479,	  30 },
	{ 1505,	  20 },
	{ 1531,	  10 },
	{ 1557,	   0 },
	{ 1565,	 -10 },
	{ 1577,	 -20 },
	{ 1601,	 -30 },
	{ 1620,	 -40 },
	{ 1633,	 -50 },
	{ 1642,	 -60 },
	{ 1656,	 -70 },
	{ 1667,  -80 },
	{ 1674,  -90 },
	{ 1689, -100 },
	{ 1704, -110 },
	{ 1719, -120 },
	{ 1734, -130 },
	{ 1749, -140 },
	{ 1763, -150 },
	{ 1778, -160 },
	{ 1793, -170 },
	{ 1818, -180 },
	{ 1823, -190 },
	{ 1838, -200 },
};

#define ADC_CH_TEMPERATURE_PMIC	6
#define ADC_CH_TEMPERATURE_LCD	7

static unsigned int sec_bat_get_lpcharging_state(void)
{
	u32 val = __raw_readl(S5P_INFORM2);
	struct power_supply *psy = power_supply_get_by_name("max8997-charger");
	union power_supply_propval value;

	if (!psy) {
		return 0;
	}

	if (val == 1) {
		psy->get_property(psy, POWER_SUPPLY_PROP_STATUS, &value);
		pr_info("%s: charging status: %d\n", __func__, value.intval);
		if (value.intval == POWER_SUPPLY_STATUS_DISCHARGING)
			pr_warn("%s: DISCHARGING\n", __func__);
	}

	pr_info("%s: LP charging:%d\n", __func__, val);
	return val;
}

static struct sec_bat_platform_data sec_bat_pdata = {
	.fuel_gauge_name	= "fuelgauge",
	.charger_name		= "max8997-charger",
	.sub_charger_name	= "max8922-charger",
	/* TODO: should provide temperature table */
	.adc_arr_size		= ARRAY_SIZE(temper_table),
	.adc_table			= temper_table,
	.adc_channel		= ADC_CH_TEMPERATURE_PMIC,
	.adc_sub_arr_size	= ARRAY_SIZE(temper_table_ADC7),
	.adc_sub_table		= temper_table_ADC7,
	.adc_sub_channel	= ADC_CH_TEMPERATURE_LCD,
	.get_lpcharging_state	= sec_bat_get_lpcharging_state,
};

static struct platform_device sec_device_battery = {
	.name = "sec-battery",
	.id = -1,
	.dev.platform_data = &sec_bat_pdata,
};

/* temperature table for ADC CH 6 */
static struct sec_therm_adc_table adc_ch6_table[] = {
	/* ADC, Temperature */
	{  173,  800 },
	{  180,  790 },
	{  188,  780 },
	{  196,  770 },
	{  204,  760 },
	{  212,  750 },
	{  220,  740 },
	{  228,  730 },
	{  236,  720 },
	{  244,  710 },
	{  252,  700 },
	{  259,  690 },
	{  266,  680 },
	{  273,  670 },
	{  289,  660 },
	{  304,  650 },
	{  314,  640 },
	{  325,  630 },
	{  337,  620 },
	{  347,  610 },
	{  361,  600 },
	{  376,  590 },
	{  391,  580 },
	{  406,  570 },
	{  417,  560 },
	{  431,  550 },
	{  447,  540 },
	{  474,  530 },
	{  491,  520 },
	{  499,  510 },
	{  511,  500 },
	{  519,  490 },
	{  547,  480 },
	{  568,  470 },
	{  585,  460 },
	{  597,  450 },
	{  614,  440 },
	{  629,  430 },
	{  647,  420 },
	{  672,  410 },
	{  690,  400 },
	{  720,  390 },
	{  735,  380 },
	{  755,  370 },
	{  775,  360 },
	{  795,  350 },
	{  818,  340 },
	{  841,  330 },
	{  864,  320 },
	{  887,  310 },
	{  909,  300 },
	{  932,  290 },
	{  954,  280 },
	{  976,  270 },
	{  999,  260 },
	{ 1021,  250 },
	{ 1051,  240 },
	{ 1077,  230 },
	{ 1103,  220 },
	{ 1129,  210 },
	{ 1155,  200 },
	{ 1177,  190 },
	{ 1199,  180 },
	{ 1220,  170 },
	{ 1242,  160 },
	{ 1263,  150 },
	{ 1284,  140 },
	{ 1306,  130 },
	{ 1326,  120 },
	{ 1349,  110 },
	{ 1369,  100 },
	{ 1390,   90 },
	{ 1411,   80 },
	{ 1433,   70 },
	{ 1454,   60 },
	{ 1474,   50 },
	{ 1486,   40 },
	{ 1499,   30 },
	{ 1512,   20 },
	{ 1531,   10 },
	{ 1548,    0 },
	{ 1570,  -10 },
	{ 1597,  -20 },
	{ 1624,  -30 },
	{ 1633,  -40 },
	{ 1643,  -50 },
	{ 1652,  -60 },
	{ 1663,  -70 },
	{ 1687,  -80 },
	{ 1711,  -90 },
	{ 1735,  -100 },
	{ 1746,  -110 },
	{ 1757,  -120 },
	{ 1768,  -130 },
	{ 1779,  -140 },
	{ 1790,  -150 },
	{ 1801,  -160 },
	{ 1812,  -170 },
	{ 1823,  -180 },
	{ 1834,  -190 },
	{ 1845,  -200 },
};

static struct sec_therm_platform_data sec_therm_pdata = {
	.adc_channel	= 6,
	.adc_arr_size	= ARRAY_SIZE(adc_ch6_table),
	.adc_table	= adc_ch6_table,
	.polling_interval = 30 * 1000, /* msecs */
};

static struct platform_device sec_device_thermistor = {
	.name = "sec-thermistor",
	.id = -1,
	.dev.platform_data = &sec_therm_pdata,
};

/*******************************************************************************
 * GPIO Keys
 ******************************************************************************/
struct gpio_keys_button u1_buttons[] = {
	{
		.code = KEY_VOLUMEUP,
		.gpio = GPIO_VOL_UP,
		.active_low = 1,
		.type = EV_KEY,
		.wakeup = 1,
		.isr_hook = sec_debug_check_crash_key,
	},			/* vol up */
	{
		.code = KEY_VOLUMEDOWN,
		.gpio = GPIO_VOL_DOWN,
		.active_low = 1,
		.type = EV_KEY,
		.wakeup = 1,
		.isr_hook = sec_debug_check_crash_key,
	},			/* vol down */
	{
		.code = KEY_POWER,
		.gpio = GPIO_nPOWER,
		.active_low = 1,
		.type = EV_KEY,
		.wakeup = 1,
		.isr_hook = sec_debug_check_crash_key,
	},			/* power key */
	{
		.code = KEY_HOME,
		.gpio = GPIO_OK_KEY,
		.active_low = 1,
		.type = EV_KEY,
		.wakeup = 1,
	},			/* ok key */
};

struct gpio_keys_platform_data u1_keypad_platform_data = {
	u1_buttons,
	ARRAY_SIZE(u1_buttons),
};

struct platform_device u1_keypad = {
	.name = "gpio-keys",
	.dev.platform_data = &u1_keypad_platform_data,
};

/*******************************************************************************
 * Headset Jack
 ******************************************************************************/
static void sec_set_jack_micbias(bool on)
{
	if (system_rev >= 3)
		gpio_set_value(GPIO_EAR_MIC_BIAS_EN, on);
	else
		gpio_set_value(GPIO_MIC_BIAS_EN, on);
}

static struct sec_jack_zone sec_jack_zones[] = {
	{
		/* adc == 0, unstable zone, default to 3pole if it stays
		 * in this range for 300ms (15ms delays, 20 samples)
		 */
		.adc_high = 0,
		.delay_ms = 15,
		.check_count = 20,
		.jack_type = SEC_HEADSET_3POLE,
	},
	{
		/* 0 < adc <= 1200, unstable zone, default to 3pole if it stays
		 * in this range for 300ms (15ms delays, 20 samples)
		 */
		.adc_high = 1200,
		.delay_ms = 10,
		.check_count = 80,
		.jack_type = SEC_HEADSET_3POLE,
	},
	{
		/* 950 < adc <= 2600, unstable zone, default to 4pole if it
		 * stays in this range for 800ms (10ms delays, 80 samples)
		 */
		.adc_high = 2600,
		.delay_ms = 10,
		.check_count = 10,
		.jack_type = SEC_HEADSET_4POLE,
	},
	{
		/* 2600 < adc <= 3400, 3 pole zone, default to 3pole if it
		 * stays in this range for 100ms (10ms delays, 10 samples)
		 */
		.adc_high = 3800,
		.delay_ms = 10,
		.check_count = 5,
		.jack_type = SEC_HEADSET_4POLE,
	},
	{
		/* adc > 3400, unstable zone, default to 3pole if it stays
		 * in this range for two seconds (10ms delays, 200 samples)
		 */
		.adc_high = 0x7fffffff,
		.delay_ms = 10,
		.check_count = 200,
		.jack_type = SEC_HEADSET_3POLE,
	},
};

/* To support 3-buttons earjack */
static struct sec_jack_buttons_zone sec_jack_buttons_zones[] = {
	{
		/* 0 <= adc <=170, stable zone */
		.code = KEY_MEDIA,
		.adc_low = 0,
		.adc_high = 170,
	},
	{
		/* 171 <= adc <= 370, stable zone */
		.code = KEY_VOLUMEUP,
		.adc_low = 171,
		.adc_high = 370,
	},
	{
		/* 371 <= adc <= 850, stable zone */
		.code = KEY_VOLUMEDOWN,
		.adc_low = 371,
		.adc_high = 850,
	},
};

static struct sec_jack_platform_data sec_jack_data = {
	.set_micbias_state = sec_set_jack_micbias,
	.zones = sec_jack_zones,
	.num_zones = ARRAY_SIZE(sec_jack_zones),
	.buttons_zones = sec_jack_buttons_zones,
	.num_buttons_zones = ARRAY_SIZE(sec_jack_buttons_zones),
	.det_gpio = GPIO_DET_35,
	.send_end_gpio = GPIO_EAR_SEND_END,
};

static struct platform_device sec_device_jack = {
	.name = "sec_jack",
	.id = 1,		/* will be used also for gpio_event id */
	.dev.platform_data = &sec_jack_data,
};

/*******************************************************************************
 * EEPROM
 ******************************************************************************/
/* I2C0 */
static struct i2c_board_info i2c_devs0[] __initdata = {
	{I2C_BOARD_INFO("24c128", 0x50),},	/* Samsung S524AD0XD1 */
	{I2C_BOARD_INFO("24c128", 0x52),},	/* Samsung S524AD0XD1 */
};

/*******************************************************************************
 * Sensors
 ******************************************************************************/
static struct k3dh_platform_data k3dh_data = {
};

/* I2C1 */
static struct i2c_board_info i2c_devs1[] __initdata = {
	{
		I2C_BOARD_INFO("k3g", 0x69),
		.irq = IRQ_EINT(1),
	},
	{
		I2C_BOARD_INFO("k3dh", 0x19),
		.platform_data = &k3dh_data,
	},
};

static struct i2c_board_info i2c_devs5[] __initdata = {
	{
		I2C_BOARD_INFO("max8997", (0xcc >> 1)),
		.platform_data = &exynos4_max8997_info,
	},
};

static struct i2c_board_info i2c_devs6[] __initdata = {
	{
		I2C_BOARD_INFO("mc1n2", 0x3a),	/* MC1N2 */
		.platform_data = &mc1n2_pdata,
	},
};

static struct akm8975_platform_data akm8975_pdata = {
	.gpio_data_ready_int = GPIO_MSENSE_INT,
};

/* I2C7 */
static struct i2c_board_info i2c_devs7[] __initdata = {
	{
		I2C_BOARD_INFO("ak8975", 0x0C),
		.platform_data = &akm8975_pdata,
	},
	{
		I2C_BOARD_INFO("s5p_ddc", (0x74 >> 1)),
	},
};

/******************************************************************************
 * Battery gauge
 *****************************************************************************/
static struct i2c_gpio_platform_data gpio_i2c_data9 = {
	.sda_pin = GPIO_FUEL_SDA,
	.scl_pin = GPIO_FUEL_SCL,
};

struct platform_device s3c_device_i2c9 = {
	.name = "i2c-gpio",
	.id = I2C_GPIO_BUS_GAUGE,
	.dev.platform_data = &gpio_i2c_data9,
};

struct max17042_reg_data max17042_init_data[] = {
	{ MAX17042_REG_CGAIN,		0x00,	0x00 },
	{ MAX17042_REG_MISCCFG,		0x03,	0x00 },
	{ MAX17042_REG_LEARNCFG,	0x07,	0x00 },
	/* RCOMP: 0x0050 2011.02.29 from MAXIM */
	{ MAX17042_REG_RCOMP,		0x50,	0x00 },
};

struct max17042_reg_data max17042_alert_init_data[] = {
	/* SALRT Threshold setting to 2% => 1% wake lock */
	{ MAX17042_REG_SALRT_TH,	0x02,	0xFF },
	/* VALRT Threshold setting (disable) */
	{ MAX17042_REG_VALRT_TH,	0x00,	0xFF },
	/* TALRT Threshold setting (disable) */
	{ MAX17042_REG_TALRT_TH,	0x80,	0x7F },
};

bool max17042_is_low_batt(void)
{
	struct power_supply *psy = power_supply_get_by_name("battery");
	union power_supply_propval value;

	if (!psy) {
		pr_err("%s: fail to get battery ps\n", __func__);
		return -ENODEV;
	}

	if (!(psy->get_property(psy, POWER_SUPPLY_PROP_CAPACITY, &value)))
		if (value.intval > SEC_BATTERY_SOC_3_6)
			return false;

	return true;
}
EXPORT_SYMBOL(max17042_is_low_batt);

static int max17042_low_batt_cb(void)
{
	struct power_supply *psy = power_supply_get_by_name("battery");
	union power_supply_propval value;

	if (!psy) {
		pr_err("%s: fail to get battery ps\n", __func__);
		return -ENODEV;
	}

	value.intval = POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
	return psy->set_property(psy, POWER_SUPPLY_PROP_CAPACITY_LEVEL, &value);
}

static struct max17042_platform_data s5pv310_max17042_info = {
	.low_batt_cb = max17042_low_batt_cb,
	.init = max17042_init_data,
	.init_size = sizeof(max17042_init_data),
	.alert_init = max17042_alert_init_data,
	.alert_init_size = sizeof(max17042_alert_init_data),
	.alert_gpio = GPIO_FUEL_ALERT,
	.alert_irq = 0,
	.enable_current_sense = false,
	.enable_gauging_temperature = true,
};

/* I2C9 */
static struct i2c_board_info i2c_devs9_emul[] __initdata = {
	{
		I2C_BOARD_INFO("max17042", 0x36),
		.platform_data	= &s5pv310_max17042_info,
		.irq = IRQ_EINT(19),
	},
};

/******************************************************************************
 * Proximity sensor i2c
 *****************************************************************************/
static struct i2c_gpio_platform_data gpio_i2c_data11 = {
	.sda_pin = GPIO_PS_ALS_SDA,
	.scl_pin = GPIO_PS_ALS_SCL,
};

struct platform_device s3c_device_i2c11 = {
	.name = "i2c-gpio",
	.id = I2C_GPIO_BUS_PROX,
	.dev.platform_data = &gpio_i2c_data11,
};

/******************************************************************************
 * Light/Proximity sensor
 *****************************************************************************/
static int cm3663_ldo(bool on)
{
	struct regulator *regulator;

	if (on) {
		regulator = regulator_get(NULL, "vled");
		if (IS_ERR(regulator))
			return 0;
		regulator_enable(regulator);
		regulator_put(regulator);
	} else {
		regulator = regulator_get(NULL, "vled");
		if (IS_ERR(regulator))
			return 0;
		if (regulator_is_enabled(regulator))
			regulator_force_disable(regulator);
		regulator_put(regulator);
	}

	return 0;
}

static struct cm3663_platform_data cm3663_pdata = {
	.proximity_power = cm3663_ldo,
};

static struct i2c_board_info i2c_devs11_emul[] __initdata = {
	{
		I2C_BOARD_INFO("cm3663", 0x20),
		.irq = GPIO_PS_ALS_INT,
		.platform_data = &cm3663_pdata,
	},
};

/******************************************************************************
 * NFC
 *****************************************************************************/
static struct i2c_gpio_platform_data i2c14_platdata = {
	.sda_pin = GPIO_NFC_SDA,
	.scl_pin = GPIO_NFC_SCL,
	.udelay = 2,
	.sda_is_open_drain = 0,
	.scl_is_open_drain = 0,
	.scl_is_output_only = 0,
};

static struct platform_device s3c_device_i2c14 = {
	.name = "i2c-gpio",
	.id = I2C_GPIO_BUS_NFC,
	.dev.platform_data = &i2c14_platdata,
};

static struct pn544_i2c_platform_data pn544_pdata = {
	.irq_gpio = GPIO_NFC_IRQ,
	.ven_gpio = GPIO_NFC_EN,
	.firm_gpio = GPIO_NFC_FIRM,
};

static struct i2c_board_info i2c_devs14[] __initdata = {
	{
		I2C_BOARD_INFO("pn544", 0x2b),
		.irq = IRQ_EINT(15),
		.platform_data = &pn544_pdata,
	},
};

static unsigned int nfc_gpio_table[][4] = {
	{GPIO_NFC_IRQ, S3C_GPIO_INPUT, GPIO_LEVEL_NONE, S3C_GPIO_PULL_DOWN},
	{GPIO_NFC_EN, S3C_GPIO_OUTPUT, GPIO_LEVEL_LOW, S3C_GPIO_PULL_NONE},
	{GPIO_NFC_FIRM, S3C_GPIO_OUTPUT, GPIO_LEVEL_LOW, S3C_GPIO_PULL_NONE},
/*	{GPIO_NFC_SCL, S3C_GPIO_INPUT, GPIO_LEVEL_NONE, S3C_GPIO_PULL_NONE}, */
/*	{GPIO_NFC_SDA, S3C_GPIO_INPUT, GPIO_LEVEL_NONE, S3C_GPIO_PULL_NONE}, */
};

void nfc_setup_gpio(void)
{
	/* s3c_config_gpio_alive_table(ARRAY_SIZE(nfc_gpio_table),
	   nfc_gpio_table); */
	int array_size = ARRAY_SIZE(nfc_gpio_table);
	u32 i, gpio;
	for (i = 0; i < array_size; i++) {
		gpio = nfc_gpio_table[i][0];
		s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(nfc_gpio_table[i][1]));
		s3c_gpio_setpull(gpio, nfc_gpio_table[i][3]);
		if (nfc_gpio_table[i][2] != GPIO_LEVEL_NONE)
			gpio_set_value(gpio, nfc_gpio_table[i][2]);
	}

	/* s3c_gpio_cfgpin(GPIO_NFC_IRQ, EINT_MODE); */
	/* s3c_gpio_setpull(GPIO_NFC_IRQ, S3C_GPIO_PULL_DOWN); */
}

/******************************************************************************
 * Some i2c
 *****************************************************************************/
static struct i2c_gpio_platform_data i2c12_platdata = {
	.sda_pin = VT_CAM_SDA_18V,
	.scl_pin = VT_CAM_SCL_18V,
	.udelay = 2,		/* 250KHz */
	.sda_is_open_drain = 0,
	.scl_is_open_drain = 0,
	.scl_is_output_only = 0,
};

static struct platform_device s3c_device_i2c12 = {
	.name = "i2c-gpio",
	.id = I2C_GPIO_BUS_VT_CAM,
	.dev.platform_data = &i2c12_platdata,
};

/* I2C12 */
static struct i2c_board_info i2c_devs12_emul[] __initdata = {
	/* need to work here */
};

/******************************************************************************
 * FM Radio
 *****************************************************************************/
static struct i2c_gpio_platform_data i2c16_platdata = {
	.sda_pin		= GPIO_FM_SDA_28V,
	.scl_pin		= GPIO_FM_SCL_28V,
	.udelay			= 2,	/* 250KHz */
	.sda_is_open_drain	= 0,
	.scl_is_open_drain	= 0,
	.scl_is_output_only	= 0,
};

static struct platform_device s3c_device_i2c16 = {
	.name					= "i2c-gpio",
	.id						= I2C_GPIO_BUS_FM,
	.dev.platform_data	= &i2c16_platdata,
};

static struct i2c_board_info i2c_devs16[] __initdata = {
	{
		I2C_BOARD_INFO("si4709", (0x20 >> 1)),
	},
};

/*******************************************************************************
 * USB
 ******************************************************************************/
static struct s5p_ehci_platdata smdkc210_ehci_pdata;
static void __init smdkc210_ehci_init(void)
{
	struct s5p_ehci_platdata *pdata = &smdkc210_ehci_pdata;

	s5p_ehci_set_platdata(pdata);
}

static struct s5p_usbgadget_platdata smdkc210_usbgadget_pdata;
static void __init smdkc210_usbgadget_init(void)
{
	struct s5p_usbgadget_platdata *pdata = &smdkc210_usbgadget_pdata;

	s5p_usbgadget_set_platdata(pdata);

	pdata = s3c_device_usbgadget.dev.platform_data;
	if (pdata) {
		/* Enables HS Transmitter pre-emphasis [20] */
		pdata->phy_tune_mask = 0;
		pdata->phy_tune_mask |= (0x1 << 20);
		pdata->phy_tune |= (0x1 << 20);

		/* U1 OPEN : Squelch Threshold Tune [13:11] (101 : -10%) */
		pdata->phy_tune_mask |= (0x7 << 11);
		pdata->phy_tune |= (0x5 << 11);
		/* HS DC Voltage Level Adjustment [3:0] (1011 : +16%) */
		pdata->phy_tune_mask |= 0xf;
		pdata->phy_tune |= 0xb;
	}
}

/******************************************************************************
 * touch keys
 *****************************************************************************/
static struct i2c_gpio_platform_data i2c_gpio_touchkey_data = {
	.sda_pin	= GPIO_3_TOUCH_SDA,
	.scl_pin	= GPIO_3_TOUCH_SCL,
	.udelay		= 2,
};

static struct platform_device i2c_gpio_touchkey = {
	.name		= "i2c-gpio",
	.id		= I2C_GPIO_BUS_TOUCHKEY,
	.dev		= {
		.platform_data	= &i2c_gpio_touchkey_data,
	},
};

static void i9100_mcs_power(bool on) {
	struct regulator *regulator;
	regulator = regulator_get(NULL, "touchkey");
	if (IS_ERR(regulator)) {
		pr_err("%s: failed to get regulator\n", __func__);
		return;
	}

	if (on) {
		regulator_enable(regulator);
	} else {
		regulator_disable(regulator);
	}

	regulator_put(regulator);
}

static uint32_t touchkey_keymap[] = {
	MCS_KEY_MAP(0, KEY_MENU),
	MCS_KEY_MAP(1, KEY_BACK),
};

static struct mcs_platform_data touchkey_data = {
	.keymap		= touchkey_keymap,
	.keymap_size	= ARRAY_SIZE(touchkey_keymap),
	.key_maxval	= 2,
	.poweron = i9100_mcs_power,
};

static struct i2c_board_info i2c_gpio_touchkey_devs[] __initdata = {
	{
		I2C_BOARD_INFO("mcs5000_touchkey", 0x20),
		.platform_data = &touchkey_data,
	},
};

static void __init i9100_init_touchkey(void)
{
	gpio_request(GPIO_3_TOUCH_INT, "3_TOUCH_INT");
	s5p_register_gpio_interrupt(GPIO_3_TOUCH_INT);
	s3c_gpio_cfgpin(GPIO_3_TOUCH_INT, S3C_GPIO_SFN(0xf));
	s3c_gpio_setpull(GPIO_3_TOUCH_INT, S3C_GPIO_PULL_UP);
	i2c_gpio_touchkey_devs[0].irq = gpio_to_irq(GPIO_3_TOUCH_INT);

	i2c_register_board_info(I2C_GPIO_BUS_TOUCHKEY,
		i2c_gpio_touchkey_devs, ARRAY_SIZE(i2c_gpio_touchkey_devs));
}

/******************************************************************************
 * touchscreen
 *****************************************************************************/
static struct mxt_platform_data qt602240_platform_data = {
	.x_line		= 19,
	.y_line		= 11,
	.x_size		= 800,
	.y_size		= 480,
	.blen		= 0x11,
	.threshold	= 0x28,
	.voltage	= 2800000,
	.orient	= MXT_DIAGONAL,
	.irqflags = IRQF_TRIGGER_FALLING,
};

static struct i2c_board_info i2c3_devs[] __initdata = {
	{
		I2C_BOARD_INFO("qt602240_ts", 0x4a),
		.platform_data = &qt602240_platform_data,
	},
};

static struct s3c2410_platform_i2c i2c3_data __initdata = {
	.flags		= 0,
	.bus_num	= 3,
	.slave_addr	= 0x10,
	.frequency	= 400 * 1000,
	.sda_delay	= 100,
};

static void __init i9100_init_tsp(void) {
	gpio_request(GPIO_TSP_INT, "TOUCH_INT");
	s5p_register_gpio_interrupt(GPIO_TSP_INT);
	s3c_gpio_cfgpin(GPIO_TSP_INT, S3C_GPIO_SFN(0xf));
	s3c_gpio_setpull(GPIO_TSP_INT, S3C_GPIO_PULL_UP);
	i2c3_devs[0].irq = gpio_to_irq(GPIO_TSP_INT);

	gpio_request(GPIO_TSP_LDO_ON, "TOUCH_LDO");
	s3c_gpio_cfgpin(GPIO_TSP_LDO_ON, S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(GPIO_TSP_LDO_ON, S3C_GPIO_PULL_NONE);
	gpio_direction_output(GPIO_TSP_LDO_ON, 1);
	msleep(100);
}

/******************************************************************************
 * devices list
 ******************************************************************************/
static struct platform_device *smdkc210_devices[] __initdata = {
	&exynos4_device_pd[PD_MFC],
	&exynos4_device_pd[PD_G3D],
	&exynos4_device_pd[PD_LCD0],
	&exynos4_device_pd[PD_LCD1],
	&exynos4_device_pd[PD_CAM],
	&exynos4_device_pd[PD_TV],
	&exynos4_device_pd[PD_GPS],

	&s3c_device_fb,

	&s3c_device_i2c0,
	&s3c_device_i2c1,
	&s3c_device_i2c3,
	&s3c_device_i2c5,
	&s3c_device_i2c6,
	&s3c_device_i2c7,
	&i2c_gpio_touchkey,
	&s3c_device_i2c9,
	&s3c_device_i2c11,
	&s3c_device_i2c14,
	&s3c_device_i2c12,
	&s3c_device_i2c15,
	&s3c_device_i2c16,
	/* consumer driver should resume after resuming i2c drivers */
	&u1_regulator_consumer,

	&s3c_device_mshci,

	&s3c_device_hsmmc2,
	&s3c_device_hsmmc3,
	&s3c_device_adc,
	&u1_keypad,
	&s3c_device_rtc,
	&s3c_device_wdt,
	&exynos_device_i2s0,
	&sec_device_battery,
	&max8922_device_charger,
#ifdef CONFIG_S5P_SYSTEM_MMU
	&SYSMMU_PLATDEV(fimc0),
	&SYSMMU_PLATDEV(fimc1),
	&SYSMMU_PLATDEV(fimc2),
	&SYSMMU_PLATDEV(fimc3),
	&SYSMMU_PLATDEV(2d),
	&SYSMMU_PLATDEV(tv),
#endif

	&samsung_asoc_dma,
	&ld9040_spi_gpio,
	&s5p_device_tvout,
	&s5p_device_cec,
	&s5p_device_hpd,

	&s3c_device_fimc0,
	&s3c_device_fimc1,
	&s3c_device_fimc2,
	&s3c_device_fimc3,
	&s3c_device_csis0,
	&s3c_device_csis1,

	&s5p_device_fimg2d,
	&s5p_device_jpeg,
	&s3c_device_usbgadget,
	&s3c_device_timer[0],
	&s3c_device_timer[1],
	&s3c_device_timer[2],
	&s3c_device_timer[3],
	&s5p_device_tmu,
	&bcm4330_bluetooth_device,
	&sec_device_jack,
	&host_notifier_device,
	&s3c_device_usb_otghcd,
};

/* below temperature base on the celcius degree */
struct s5p_platform_tmu u1_tmu_data __initdata = {
	.ts = {
		.stop_1st_throttle  = 61,
		.start_1st_throttle = 64,
		.stop_2nd_throttle  = 87,
		.start_2nd_throttle = 103,
		.start_tripping     = 110,
		.start_emergency    = 120,
		.stop_mem_throttle  = 80,
		.start_mem_throttle = 85,
	},
	.cpufreq = {
		.limit_1st_throttle  = 800000, /* 800MHz in KHz order */
		.limit_2nd_throttle  = 200000, /* 200MHz in KHz order */
	},
};

static int __init s5p_ehci_device_initcall(void)
{
	return platform_device_register(&s5p_device_ehci);
}
late_initcall(s5p_ehci_device_initcall);

static struct s5p_platform_hpd hdmi_hpd_data __initdata = {

};

static struct s5p_platform_cec hdmi_cec_data __initdata = {

};

static void __init exynos4_cma_region_reserve(struct cma_region *regions_normal,
					      struct cma_region *regions_secure)
{
	struct cma_region *reg;
	size_t size_secure = 0, align_secure = 0;
	phys_addr_t paddr = 0;

	for (reg = regions_normal; reg->size != 0; reg++) {
		if (WARN_ON(cma_early_region_register(reg)))
			continue;

		if ((reg->alignment & (reg->alignment - 1)) || reg->reserved)
			continue;

		if (reg->start) {
			if (!memblock_is_region_reserved(reg->start, reg->size)
			    && memblock_reserve(reg->start, reg->size) >= 0)
				reg->reserved = 1;
		} else {
			paddr = __memblock_alloc_base(reg->size, reg->alignment,
						      MEMBLOCK_ALLOC_ACCESSIBLE);
			if (paddr) {
				reg->start = paddr;
				reg->reserved = 1;
			}
		}
	}

	if (regions_secure && regions_secure->size) {
		for (reg = regions_secure; reg->size != 0; reg++)
			size_secure += reg->size;

		reg--;

		align_secure = reg->alignment;
		BUG_ON(align_secure & (align_secure - 1));

		paddr -= size_secure;
		paddr &= ~(align_secure - 1);

		if (!memblock_reserve(paddr, size_secure)) {
			do {
				reg->start = paddr;
				reg->reserved = 1;
				paddr += reg->size;

				if (WARN_ON(cma_early_region_register(reg)))
					memblock_free(reg->start, reg->size);
			} while (reg-- != regions_secure);
		}
	}
}

static void __init exynos4_reserve_mem(void)
{
	static struct cma_region regions[] = {
#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMD
		{
			.name = "fimd",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMD * SZ_1K,
			.start = 0,
		},
#endif
#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMC0
		{
			.name = "fimc0",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMC0 * SZ_1K,
			.start = 0,
		},
#endif
#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMC1
		{
			.name = "fimc1",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMC1 * SZ_1K,
			.start = 0,
		},
#endif
#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMC2
		{
			.name = "fimc2",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMC2 * SZ_1K,
			.start = 0,
		},
#endif
#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMC3
		{
			.name = "fimc3",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMC3 * SZ_1K,
			.start = 0,
		},
#endif
#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_JPEG
		{
			.name = "jpeg",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_JPEG * SZ_1K,
			.start = 0,
		},
#endif
#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMG2D
		{
			.name = "fimg2d",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMG2D * SZ_1K,
			.start = 0,
		},
#endif
#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_TVOUT
		{
			.name = "tvout",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_TVOUT * SZ_1K,
			.start = 0,
		},
#endif
		{
			.size = 0,
		},
	};

	static const char map[] __initconst =
		"s3cfb.0=fimd;exynos4-fb.0=fimd;"
		"s3c-fimc.0=fimc0;s3c-fimc.1=fimc1;s3c-fimc.2=fimc2;"
		"exynos4210-fimc.0=fimc0;exynos4210-fimc.1=fimc1;"
		"exynos4210-fimc.2=fimc2;exynos4210-fimc3=fimc3;"
		"s5p-jpeg=jpeg;"
#ifdef CONFIG_VIDEO_EXYNOS_FIMC_IS
		"exynos4-fimc-is=fimc_is;"
#endif
		"s5p-fimg2d=fimg2d;"
		"s5p-tvout=tvout";

	cma_set_defaults(regions, map);
	exynos4_cma_region_reserve(regions, NULL);

}

static void __init exynos_sysmmu_init(void)
{
	ASSIGN_SYSMMU_POWERDOMAIN(fimc0, &exynos4_device_pd[PD_CAM].dev);
	ASSIGN_SYSMMU_POWERDOMAIN(fimc1, &exynos4_device_pd[PD_CAM].dev);
	ASSIGN_SYSMMU_POWERDOMAIN(fimc2, &exynos4_device_pd[PD_CAM].dev);
	ASSIGN_SYSMMU_POWERDOMAIN(fimc3, &exynos4_device_pd[PD_CAM].dev);
	ASSIGN_SYSMMU_POWERDOMAIN(jpeg, &exynos4_device_pd[PD_CAM].dev);
	ASSIGN_SYSMMU_POWERDOMAIN(fimd0, &exynos4_device_pd[PD_LCD0].dev);
	ASSIGN_SYSMMU_POWERDOMAIN(2d, &exynos4_device_pd[PD_LCD0].dev);
	ASSIGN_SYSMMU_POWERDOMAIN(rot, &exynos4_device_pd[PD_LCD0].dev);
	ASSIGN_SYSMMU_POWERDOMAIN(tv, &exynos4_device_pd[PD_TV].dev);
	sysmmu_set_owner(&SYSMMU_PLATDEV(fimc0).dev, &s3c_device_fimc0.dev);
	sysmmu_set_owner(&SYSMMU_PLATDEV(fimc1).dev, &s3c_device_fimc1.dev);
	sysmmu_set_owner(&SYSMMU_PLATDEV(fimc2).dev, &s3c_device_fimc2.dev);
	sysmmu_set_owner(&SYSMMU_PLATDEV(fimc3).dev, &s3c_device_fimc3.dev);
	sysmmu_set_owner(&SYSMMU_PLATDEV(jpeg).dev, &s5p_device_jpeg.dev);
	sysmmu_set_owner(&SYSMMU_PLATDEV(2d).dev, &s5p_device_fimg2d.dev);
	sysmmu_set_owner(&SYSMMU_PLATDEV(tv).dev, &s5p_device_tvout.dev);
}

static void __init smdkc210_map_io(void)
{
	clk_xusbxti.rate = 24000000;

	s5p_init_io(NULL, 0, S5P_VA_CHIPID);
	s3c24xx_init_clocks(24000000);
	s3c24xx_init_uarts(smdkc210_uartcfgs, ARRAY_SIZE(smdkc210_uartcfgs));

	exynos4_reserve_mem();

	/* as soon as INFORM3 is visible, sec_debug is ready to run */
	sec_debug_init();
}

static void __init smdkc210_machine_init(void)
{
	/* initialise the gpios */
	u1_config_gpio_table();
	exynos4_sleep_gpio_table_set = u1_config_sleep_gpio_table;

	s3c_i2c0_set_platdata(NULL);
	i2c_register_board_info(0, i2c_devs0, ARRAY_SIZE(i2c_devs0));

	s3c_i2c1_set_platdata(NULL);
	i2c_register_board_info(1, i2c_devs1, ARRAY_SIZE(i2c_devs1));

	i9100_init_tsp();
	s3c_i2c3_set_platdata(&i2c3_data);
	i2c_register_board_info(3, i2c3_devs, ARRAY_SIZE(i2c3_devs));

	s3c_i2c5_set_platdata(NULL);
	s3c_gpio_cfgpin(GPIO_PMIC_IRQ, S3C_GPIO_SFN(0xF));
	s3c_gpio_setpull(GPIO_PMIC_IRQ, S3C_GPIO_PULL_NONE);
	i2c_devs5[0].irq = gpio_to_irq(GPIO_PMIC_IRQ);
	i2c_register_board_info(5, i2c_devs5, ARRAY_SIZE(i2c_devs5));

	s3c_i2c6_set_platdata(NULL);
	i2c_register_board_info(6, i2c_devs6, ARRAY_SIZE(i2c_devs6));
	s3c_i2c7_set_platdata(NULL);
	i2c_register_board_info(7, i2c_devs7, ARRAY_SIZE(i2c_devs7));

	i9100_init_touchkey();
	i2c_register_board_info(15, tuna_i2c15_boardinfo,
			ARRAY_SIZE(tuna_i2c15_boardinfo));
	i2c_register_board_info(9, i2c_devs9_emul, ARRAY_SIZE(i2c_devs9_emul));

	s3c_gpio_setpull(GPIO_PS_ALS_INT, S3C_GPIO_PULL_NONE);
	i2c_register_board_info(11, i2c_devs11_emul,
				ARRAY_SIZE(i2c_devs11_emul));

	nfc_setup_gpio();
	i2c_register_board_info(14, i2c_devs14, ARRAY_SIZE(i2c_devs14));
	i2c_register_board_info(12, i2c_devs12_emul,
				ARRAY_SIZE(i2c_devs12_emul));
	i2c_register_board_info(16, i2c_devs16, ARRAY_SIZE(i2c_devs16));


	/* 400 kHz for initialization of MMC Card  */
	__raw_writel((__raw_readl(EXYNOS4_CLKDIV_FSYS3) & 0xfffffff0)
		     | 0x9, EXYNOS4_CLKDIV_FSYS3);
	__raw_writel((__raw_readl(EXYNOS4_CLKDIV_FSYS2) & 0xfff0fff0)
		     | 0x80008, EXYNOS4_CLKDIV_FSYS2);
	__raw_writel((__raw_readl(EXYNOS4_CLKDIV_FSYS1) & 0xfff0fff0)
		     | 0x90009, EXYNOS4_CLKDIV_FSYS1);

	s3c_gpio_cfgpin(GPIO_WLAN_EN, S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(GPIO_WLAN_EN, S3C_GPIO_PULL_NONE);
	gpio_direction_output(GPIO_WLAN_EN, 1);
	mdelay(50);
	
	s3c_sdhci2_set_platdata(&exynos4_hsmmc2_pdata);
	s3c_sdhci3_set_platdata(&exynos4_hsmmc3_pdata);

	s3c_mshci_set_platdata(&exynos4_mshc_pdata);

	s3cfb_set_platdata(NULL);
	s3c_device_fb.dev.parent = &exynos4_device_pd[PD_LCD0].dev;

	s5p_device_jpeg.dev.parent = &exynos4_device_pd[PD_CAM].dev;

	s5p_hdmi_hpd_set_platdata(&hdmi_hpd_data);
	s5p_hdmi_cec_set_platdata(&hdmi_cec_data);
	s5p_device_tvout.dev.parent = &exynos4_device_pd[PD_TV].dev;

	/* fimc */
	s3c_fimc0_set_platdata(&fimc_plat);
	s3c_fimc1_set_platdata(NULL);
	s3c_fimc2_set_platdata(&fimc_plat);
	s3c_fimc3_set_platdata(NULL);
	s3c_csis0_set_platdata(NULL);
	s3c_csis1_set_platdata(NULL);

	s3c_device_fimc0.dev.parent = &exynos4_device_pd[PD_CAM].dev;
	s3c_device_fimc1.dev.parent = &exynos4_device_pd[PD_CAM].dev;
	s3c_device_fimc2.dev.parent = &exynos4_device_pd[PD_CAM].dev;
	s3c_device_fimc3.dev.parent = &exynos4_device_pd[PD_CAM].dev;

	s5p_tmu_set_platdata(&u1_tmu_data);
	s5p_fimg2d_set_platdata(&fimg2d_data);
	s5p_device_fimg2d.dev.parent = &exynos4_device_pd[PD_LCD0].dev;
	smdkc210_ehci_init();
	smdkc210_usbgadget_init();
	ld9040_fb_init();
	u1_sound_init();
	exynos_sysmmu_init();

	platform_add_devices(smdkc210_devices, ARRAY_SIZE(smdkc210_devices));

	platform_device_register(&sec_device_thermistor);
}

static void __init exynos_init_reserve(void)
{
	sec_debug_magic_init();
}

MACHINE_START(SMDKC210, "i9100")
	/* Maintainer: Kukjin Kim <kgene.kim@samsung.com> */
	.boot_params	= S5P_PA_SDRAM + 0x100,
	.init_irq	= exynos4_init_irq,
	.map_io		= smdkc210_map_io,
	.init_machine	= smdkc210_machine_init,
	.timer		= &exynos4_timer,
	.init_early	= &exynos_init_reserve,
MACHINE_END
