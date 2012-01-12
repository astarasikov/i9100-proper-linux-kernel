/* linux/arch/arm/mach-exynos4/mach-sgs_i9100.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/
#include <linux/delay.h>
#include <linux/serial_core.h>
#include <linux/gpio.h>
#include <linux/mmc/host.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/input.h>
#include <linux/pwm_backlight.h>
#include <linux/gpio_keys.h>
#include <linux/i2c.h>
#include <linux/i2c-gpio.h>
#include <linux/i2c/mcs.h>
#include <linux/i2c/atmel_mxt_ts.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/mfd/max8997.h>
#include <linux/lcd.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_gpio.h>

#include <asm/mach/arch.h>
#include <asm/mach-types.h>

#include <video/platform_lcd.h>
#include <plat/regs-serial.h>
#include <plat/regs-srom.h>
#include <plat/regs-fb-v4.h>
#include <plat/exynos4.h>
#include <plat/cpu.h>
#include <plat/devs.h>
#include <plat/sdhci.h>
#include <plat/iic.h>
#include <plat/ehci.h>
#include <plat/clock.h>
#include <plat/gpio-cfg.h>
#include <plat/backlight.h>
#include <plat/pd.h>
#include <plat/fb.h>
#include <plat/mfc.h>
#include <plat/otg.h>

#include <mach/ohci.h>
#include <mach/map.h>
#include <mach/sgs_i9100.h>

enum fixed_regulator_id {
	FIXED_REG_ID_MMC = 0,
};

enum i2c_bus_ids {
	I2C_GPIO_BUS_TOUCHKEY = 8,
};

/******************************************************************************
 * UART 
 ********************************************************************************/
/* Following are default values for UCON, ULCON and UFCON UART registers */
#define I9100_UCON_DEFAULT	(S3C2410_UCON_TXILEVEL |	\
				 S3C2410_UCON_RXILEVEL |	\
				 S3C2410_UCON_TXIRQMODE |	\
				 S3C2410_UCON_RXIRQMODE |	\
				 S3C2410_UCON_RXFIFO_TOI |	\
				 S3C2443_UCON_RXERR_IRQEN)

#define I9100_ULCON_DEFAULT	S3C2410_LCON_CS8

#define I9100_UFCON_DEFAULT	(S3C2410_UFCON_FIFOMODE |	\
				 S5PV210_UFCON_TXTRIG4 |	\
				 S5PV210_UFCON_RXTRIG4)

static struct s3c2410_uartcfg i9100_uartcfgs[] __initdata = {
	[0] = {
		.hwport		= 0,
		.flags		= 0,
		.ucon		= I9100_UCON_DEFAULT,
		.ulcon		= I9100_ULCON_DEFAULT,
		.ufcon		= I9100_UFCON_DEFAULT,
	},
	[1] = {
		.hwport		= 1,
		.flags		= 0,
		.ucon		= I9100_UCON_DEFAULT,
		.ulcon		= I9100_ULCON_DEFAULT,
		.ufcon		= I9100_UFCON_DEFAULT,
	},
	[2] = {
		.hwport		= 2,
		.flags		= 0,
		.ucon		= I9100_UCON_DEFAULT,
		.ulcon		= I9100_ULCON_DEFAULT,
		.ufcon		= I9100_UFCON_DEFAULT,
	},
	[3] = {
		.hwport		= 3,
		.flags		= 0,
		.ucon		= I9100_UCON_DEFAULT,
		.ulcon		= I9100_ULCON_DEFAULT,
		.ufcon		= I9100_UFCON_DEFAULT,
	},
};

/******************************************************************************
 * voltage regulator
 *****************************************************************************/
static struct regulator_consumer_supply ldo1_supply[] = {
	REGULATOR_SUPPLY("vdd", "s5p-adc"), /* Used by CPU's ADC drv */
};

static struct regulator_consumer_supply ldo3_supply[] = {
	REGULATOR_SUPPLY("vdd11", "s5p-mipi-csis.0"), /* MIPI */
};

static struct regulator_consumer_supply ldo4_supply[] = {
	REGULATOR_SUPPLY("vdd18", "s5p-mipi-csis.0"), /* MIPI */
};

static struct regulator_consumer_supply ldo5_supply[] = {
	REGULATOR_SUPPLY("vhsic", "modemctl"), /* MODEM */
};

static struct regulator_consumer_supply ldo7_supply[] = {
	REGULATOR_SUPPLY("cam_isp", NULL),
};

static struct regulator_consumer_supply ldo8_supply[] = {
	REGULATOR_SUPPLY("vusb_d", NULL), /* Used by CPU */
	REGULATOR_SUPPLY("vdac", NULL), /* Used by CPU */
};

static struct regulator_consumer_supply ldo10_supply[] = {
	REGULATOR_SUPPLY("vpll_1.2v", NULL),
};

static struct regulator_consumer_supply ldo11_supply[] = {
	REGULATOR_SUPPLY("touch", NULL),
};

static struct regulator_consumer_supply ldo12_supply[] = {
	REGULATOR_SUPPLY("vt_cam_1.8v", NULL),
};

static struct regulator_consumer_supply ldo13_supply[] = {
	REGULATOR_SUPPLY("vlcd_3.0v", NULL),
};

static struct regulator_consumer_supply ldo14_supply[] = {
	REGULATOR_SUPPLY("inmotor", "max8997-haptic"),
};

static struct regulator_consumer_supply ldo15_supply[] = {
	REGULATOR_SUPPLY("vled", NULL),
};

static struct regulator_consumer_supply ldo16_supply[] = {
	REGULATOR_SUPPLY("cam_sensor_io", NULL),
};

static struct regulator_consumer_supply ldo17_supply[] = {
	REGULATOR_SUPPLY("vt_cam_core_1.8v", NULL),
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
	REGULATOR_SUPPLY("vdd_int", "exynos4210-busfreq.0"), /* CPUFREQ */
};

static struct regulator_consumer_supply buck3_supply[] = {
	REGULATOR_SUPPLY("vdd", "mali_dev.0"), /* G3D of Exynos 4 */
};

static struct regulator_consumer_supply buck4_supply[] = {
	REGULATOR_SUPPLY("cam_isp_core", NULL),
};

static struct regulator_consumer_supply buck7_supply[] = {
	REGULATOR_SUPPLY("vcc_sub", NULL),
};

static struct regulator_consumer_supply safeout1_supply[] = {
	REGULATOR_SUPPLY("usb_vbus", NULL), /* CPU's USB OTG */
};

static struct regulator_consumer_supply safeout2_supply[] = {
	REGULATOR_SUPPLY("usb_vbus", "modemctl"), /* VBUS of Modem */
};

static struct regulator_consumer_supply led_flash_supply[] = {
	REGULATOR_SUPPLY("led_flash", NULL),
};

static struct regulator_consumer_supply led_movie_supply[] = {
	REGULATOR_SUPPLY("led_movie", NULL),
};

#define VREG(_ldo, _name, _min_uV, _max_uV, _always_on, _ops_mask) \
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
				.disabled	= 1,		\
				.enabled	= 0,		\
			}						\
		},							\
		.num_consumer_supplies = ARRAY_SIZE(_ldo##_supply),	\
		.consumer_supplies = &_ldo##_supply[0],			\
	};

VREG(ldo1, "VADC_3.3V_C210", 3300000, 3300000, 1,  REGULATOR_CHANGE_STATUS);
VREG(ldo3, "VUSB_1.1V", 1100000, 1100000, 1,  REGULATOR_CHANGE_STATUS);
VREG(ldo4, "VMIPI_1.8V", 1800000, 1800000, 1,  REGULATOR_CHANGE_STATUS);
VREG(ldo5, "VHSIC_1.2V", 1200000, 1200000, 1,  REGULATOR_CHANGE_STATUS);
VREG(ldo7, "CAM_ISP_1.8V", 1800000, 1800000, 0,  REGULATOR_CHANGE_STATUS);
VREG(ldo8, "VUSB_3.3V", 3300000, 3300000, 1,  REGULATOR_CHANGE_STATUS);
VREG(ldo10, "VPLL_1.2V", 1200000, 1200000, 1,  REGULATOR_CHANGE_STATUS);
VREG(ldo11, "TOUCH_2.8V", 2800000, 2800000, 0,  REGULATOR_CHANGE_STATUS);
VREG(ldo12, "VT_CAM_1.8V", 1800000, 1800000, 0,  REGULATOR_CHANGE_STATUS);
VREG(ldo13, "VCC_3.0V_LCD", 3000000, 3000000, 1,  REGULATOR_CHANGE_STATUS);
VREG(ldo14, "VCC_2.8V_MOTOR", 2800000, 2800000, 0,  REGULATOR_CHANGE_STATUS);
VREG(ldo15, "LED_A_2.8V", 2800000, 2800000, 1,  REGULATOR_CHANGE_STATUS);
VREG(ldo16, "CAM_SENSOR_IO_1.8V", 1800000, 1800000, 0, REGULATOR_CHANGE_STATUS);
VREG(ldo17, "VT_CAM_CORE_1.8V", 1800000, 1800000, 0, REGULATOR_CHANGE_STATUS);
VREG(ldo18, "TOUCH_LED_3.3V", 3000000, 3300000, 0,
	REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_VOLTAGE);
VREG(ldo21, "VDDQ_M1M2_1.2V", 1200000, 1200000, 1,  REGULATOR_CHANGE_STATUS);

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
		.min_uV		= 800000,
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
			.mode		= REGULATOR_MODE_NORMAL,
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
		.always_on	= 0,
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
		.always_on	= 0,
		.boot_on	= 0,
		.state_mem	= {
			.enabled = 1,
		},
	},
	.num_consumer_supplies	= ARRAY_SIZE(safeout2_supply),
	.consumer_supplies	= safeout2_supply,
};

static struct regulator_init_data led_flash_init_data = {
	.constraints = {
		.name	= "CHARGER_CV",
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
		.name	= "CHARGER",
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

static struct max8997_regulator_data __initdata i9100_max8997_regulators[] = {
	{ MAX8997_LDO1,	 &ldo1_init_data },
	{ MAX8997_LDO3,	 &ldo3_init_data },
	{ MAX8997_LDO4,	 &ldo4_init_data },
	{ MAX8997_LDO5,	 &ldo5_init_data },
	{ MAX8997_LDO7,	 &ldo7_init_data },
	{ MAX8997_LDO8,	 &ldo8_init_data },
	{ MAX8997_LDO10, &ldo10_init_data },
	{ MAX8997_LDO11, &ldo11_init_data },
	{ MAX8997_LDO12, &ldo12_init_data },
	{ MAX8997_LDO13, &ldo13_init_data },
	{ MAX8997_LDO14, &ldo14_init_data },
	{ MAX8997_LDO15, &ldo15_init_data },
	{ MAX8997_LDO16, &ldo16_init_data },
	{ MAX8997_LDO17, &ldo17_init_data },
	{ MAX8997_LDO18, &ldo18_init_data },
	{ MAX8997_LDO21, &ldo21_init_data },
	{ MAX8997_BUCK1, &buck1_init_data },
	{ MAX8997_BUCK2, &buck2_init_data },
	{ MAX8997_BUCK3, &buck3_init_data },
	{ MAX8997_BUCK4, &buck4_init_data },
	{ MAX8997_BUCK5, &buck5_init_data },
	{ MAX8997_BUCK7, &buck7_init_data },
	{ MAX8997_ESAFEOUT1, &safeout1_init_data },
	{ MAX8997_ESAFEOUT2, &safeout2_init_data },
	{ MAX8997_CHARGER_CV, &led_flash_init_data },
	{ MAX8997_CHARGER, &led_movie_init_data },
};

static struct max8997_platform_data __initdata i9100_max8997_pdata = {
	.wakeup			= 1,

	.num_regulators		= ARRAY_SIZE(i9100_max8997_regulators),
	.regulators		= i9100_max8997_regulators,

	.buck125_gpios = {GPIO_BUCK1_EN_A, GPIO_BUCK1_EN_B, GPIO_BUCK2_EN},

	.buck1_voltage[0]	= 1350000,
	.buck1_voltage[1]	= 1300000,
	.buck1_voltage[2]	= 1250000,
	.buck1_voltage[3]	= 1200000,
	.buck1_voltage[4]	= 1150000,
	.buck1_voltage[5]	= 1100000,
	.buck1_voltage[6]	= 1000000,
	.buck1_voltage[7]	= 950000,

	.buck2_voltage[0]	= 1100000,
	.buck2_voltage[1]	= 1100000,
	.buck2_voltage[2]	= 1100000,
	.buck2_voltage[3]	= 1100000,
	.buck2_voltage[4]	= 1000000,
	.buck2_voltage[5]	= 1000000,
	.buck2_voltage[6]	= 1000000,
	.buck2_voltage[7]	= 1000000,

	.buck5_voltage[0]	= 1200000,
	.buck5_voltage[1]	= 1200000,
	.buck5_voltage[2]	= 1200000,
	.buck5_voltage[3]	= 1200000,
	.buck5_voltage[4]	= 1200000,
	.buck5_voltage[5]	= 1200000,
	.buck5_voltage[6]	= 1200000,
	.buck5_voltage[7]	= 1200000,
};

static struct i2c_board_info i2c5_devs[] __initdata = {
	{
		I2C_BOARD_INFO("max8997", 0xCC >> 1),
		.platform_data	= &i9100_max8997_pdata,
	},
};

static struct regulator_consumer_supply emmc_supplies[] = {
	REGULATOR_SUPPLY("vmmc", "s3c-sdhci.0"),
	REGULATOR_SUPPLY("vmmc", "dw_mmc"),
};

static struct regulator_init_data emmc_fixed_voltage_init_data = {
	.constraints		= {
		.name		= "VMEM_VDD_2.8V",
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= ARRAY_SIZE(emmc_supplies),
	.consumer_supplies	= emmc_supplies,
};

static struct fixed_voltage_config emmc_fixed_voltage_config = {
	.supply_name		= "MASSMEMORY_EN (inverted)",
	.microvolts		= 2800000,
	.gpio			= GPIO_MASSMEM_EN,
	.enable_high		= false,
	.init_data		= &emmc_fixed_voltage_init_data,
};

static struct platform_device emmc_fixed_voltage = {
	.name			= "reg-fixed-voltage",
	.id			= FIXED_REG_ID_MMC,
	.dev			= {
		.platform_data	= &emmc_fixed_voltage_config,
	},
};

/******************************************************************************
 * gpio keys
 ******************************************************************************/
static struct gpio_keys_button i9100_gpio_keys[] = {
	{
		.code = KEY_LEFTCTRL,
		.gpio = GPIO_VOL_UP,
		.active_low = 1,
		.type = EV_KEY,
		.wakeup = 1,
		.can_disable = 1,
	}, {
		.code = KEY_LEFTALT,
		.gpio = GPIO_VOL_DOWN,
		.active_low = 1,
		.type = EV_KEY,
		.wakeup = 1,
		.can_disable = 1,
	}, {
		.code = KEY_DELETE,
		.gpio = GPIO_nPOWER,
		.active_low = 1,
		.type = EV_KEY,
		.wakeup = 1,
	}, {
		.code = KEY_HOME,
		.gpio = GPIO_OK_KEY,
		.active_low = 1,
		.type = EV_KEY,
		.wakeup = 1,
	}
};

static struct gpio_keys_platform_data i9100_gpio_keys_data = {
	.buttons	= i9100_gpio_keys,
	.nbuttons	= ARRAY_SIZE(i9100_gpio_keys),
};

static struct platform_device i9100_device_gpio_keys = {
	.name		= "gpio-keys",
	.dev		= {
		.platform_data	= &i9100_gpio_keys_data,
	},
};

/******************************************************************************
 * touch keys
 *****************************************************************************/
static struct i2c_gpio_platform_data i2c_gpio_touchkey_data = {
	.sda_pin	= GPIO_3_TOUCH_SDA,
	.scl_pin	= GPIO_3_TOUCH_SCL,
};

static struct platform_device i2c_gpio_touchkey = {
	.name		= "i2c-gpio",
	.id		= I2C_GPIO_BUS_TOUCHKEY,
	.dev		= {
		.platform_data	= &i2c_gpio_touchkey_data,
	},
};

static uint32_t touchkey_keymap[] = {
	MCS_KEY_MAP(0, KEY_MENU),
	MCS_KEY_MAP(1, KEY_BACK),
};

static struct mcs_platform_data touchkey_data = {
	.keymap		= touchkey_keymap,
	.keymap_size	= ARRAY_SIZE(touchkey_keymap),
	.key_maxval	= 2,
};

static struct i2c_board_info i2c_gpio_touchkey_devs[] __initdata = {
	{
		I2C_BOARD_INFO("mcs5080_touchkey", 0x20),
		.platform_data = &touchkey_data,
	},
};

static void __init i9100_init_touchkey(void)
{
	gpio_request(GPIO_3_TOUCH_INT, "3_TOUCH_INT");
	s5p_register_gpio_interrupt(GPIO_3_TOUCH_INT);
	s3c_gpio_cfgpin(GPIO_3_TOUCH_INT, S3C_GPIO_SFN(0xf));
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
};

static struct i2c_board_info i2c3_devs[] __initdata = {
	{
		I2C_BOARD_INFO("qt602240_ts", 0x4a),
		.platform_data = &qt602240_platform_data,
	},
};

static void __init i9100_init_tsp(void) {
	gpio_request(GPIO_TSP_INT, "TOUCH_INT");
	s3c_gpio_cfgpin(GPIO_TSP_INT, S3C_GPIO_SFN(0xf));
	s3c_gpio_setpull(GPIO_TSP_INT, S3C_GPIO_PULL_NONE);


	gpio_request(GPIO_TSP_LDO_ON, "TOUCH_LDO");
	s3c_gpio_cfgpin(GPIO_TSP_LDO_ON, S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(GPIO_TSP_LDO_ON, S3C_GPIO_PULL_NONE);
	gpio_set_value(GPIO_TSP_LDO_ON, 1);
}

/******************************************************************************
 * framebuffer
 *******************************************************************************/
static void i9100_fimd0_gpio_setup(void)
{
	unsigned int reg;

	s3c_gpio_cfgrange_nopull(EXYNOS4_GPF0(0), 8, S3C_GPIO_SFN(2));
	s3c_gpio_cfgrange_nopull(EXYNOS4_GPF1(0), 8, S3C_GPIO_SFN(2));
	s3c_gpio_cfgrange_nopull(EXYNOS4_GPF2(0), 8, S3C_GPIO_SFN(2));
	s3c_gpio_cfgrange_nopull(EXYNOS4_GPF3(0), 4, S3C_GPIO_SFN(2));

	/*
	 * Set DISPLAY_CONTROL register for Display path selection.
	 *
	 * DISPLAY_CONTROL[1:0]
	 * ---------------------
	 *  00 | MIE
	 *  01 | MDNIE
	 *  10 | FIMD
	 *  11 | FIMD
	 *  TODO: fix s3c-fb driver to disable MDNIE
	 */
	reg = __raw_readl(S3C_VA_SYS + 0x0210);
	reg &= (1 << 1);
	reg |= 1;
	reg &= ~(1 << 13);
	reg &= ~(3 << 10);
	reg &= ~(1 << 12);
	__raw_writel(reg, S3C_VA_SYS + 0x0210);
}

 static struct s3c_fb_pd_win i9100_fb_win0 = {
	.win_mode = {
		.left_margin	= 16,
		.right_margin	= 14,
		.upper_margin	= 4,
		.lower_margin	= 10,
		.hsync_len	= 2,
		.vsync_len	= 2,
		.xres		= 480,
		.yres		= 800,
		.refresh	= 55,
	},
	.max_bpp	= 24,
	.default_bpp = 16,
	.virtual_x	= 480,
	.virtual_y	= 800,
};

static struct s3c_fb_platdata i9100_fb_pdata __initdata = {
	.win[0]		= &i9100_fb_win0,
	.vidcon0	= VIDCON0_VIDOUT_RGB | VIDCON0_PNRMODE_RGB |
		VIDCON0_CLKSEL_LCD,
	.vidcon1	= VIDCON1_INV_HSYNC | VIDCON1_INV_VSYNC |
		VIDCON1_INV_VDEN | VIDCON1_INV_VCLK,
	.setup_gpio	= i9100_fimd0_gpio_setup,
};

static int ld9040_cfg_gpio(void)
{
	/* drive strength to max[4X] */
	writel(0xffffffff, S5P_VA_GPIO + 0x18c);
	writel(0xffffffff, S5P_VA_GPIO + 0x1ac);
	writel(0xffffffff, S5P_VA_GPIO + 0x1cc);
	writel(readl(S5P_VA_GPIO + 0x1ec) | 0xffffff, S5P_VA_GPIO + 0x1ec);

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

static int ld9040_reset(struct lcd_device *ld) {
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
	mdelay(10);
	gpio_direction_output(reset_gpio, 1);

	gpio_free(reset_gpio);

	return 0;
}

static int ld9040_power(struct lcd_device *ld, int enable) {
	struct regulator *regulator;
	int rc = 0;

	if (ld == NULL) {
		printk(KERN_ERR "lcd device object is NULL.\n");
		rc = -EINVAL;
		goto fail;
	}

	regulator = regulator_get(NULL, "vlcd_3.0v");
	if (IS_ERR(regulator)) {
		rc = -ENODEV;
		goto fail;
	}
	
	if (enable) {
		regulator_enable(regulator);
		regulator_put(regulator);
	} else {
		if (regulator_is_enabled(regulator)) {
			regulator_force_disable(regulator);
		}
	}

	regulator_put(regulator);
fail:
	return rc;
}

static struct lcd_platform_data ld9040_platform_data = {
	.power_on	= ld9040_power,
	.reset	= ld9040_reset,

	.lcd_enabled	= 0,
	.reset_delay	= 20,
	.power_on_delay	= 50,
	.power_off_delay	= 300,
};

static struct spi_gpio_platform_data lcd_spi_gpio_pdata = {
	.sck	= GPIO_LCD_SPI_SCK,
	.mosi	= GPIO_LCD_SPI_MOSI,
	.miso	= SPI_GPIO_NO_MISO,
	.num_chipselect	= 1,
};

static struct platform_device lcd_spi_gpio = {
	.name	= "spi_gpio",
	.id = I9100_SPI_LCD_BUS,
	.dev = {
		.platform_data = &lcd_spi_gpio_pdata,
	}
};

static struct spi_board_info spi_board_info[] __initdata = {
	{
		.max_speed_hz	= 1200000,
		.bus_num	= I9100_SPI_LCD_BUS,
		.chip_select	= 0,
		.mode	= SPI_MODE_3,
		.controller_data = (void*)GPIO_LCD_SPI_CS,
		.platform_data	= &ld9040_platform_data,
	}
};

static void __init i9100_init_fb(void) {
	ld9040_cfg_gpio();
	spi_register_board_info(spi_board_info, ARRAY_SIZE(spi_board_info));
	s5p_fimd0_set_platdata(&i9100_fb_pdata);
}

/******************************************************************************
 * sdhci
 ******************************************************************************/
static struct s3c_sdhci_platdata i9100_hsmmc0_pdata __initdata = {
	.max_width		= 8,
	.host_caps		= (MMC_CAP_8_BIT_DATA | MMC_CAP_4_BIT_DATA |
				MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED |
				MMC_CAP_DISABLE | MMC_CAP_ERASE),
	.cd_type		= S3C_SDHCI_CD_PERMANENT,
	.clk_type		= S3C_SDHCI_CLK_DIV_EXTERNAL,
};

static struct s3c_sdhci_platdata i9100_hsmmc2_pdata __initdata = {
	.cd_type		= S3C_SDHCI_CD_PERMANENT,
	.clk_type		= S3C_SDHCI_CLK_DIV_EXTERNAL,
//	.ext_cd_gpio	= GPIO_HSMMC2_CD,
//	.ext_cd_gpio_invert = true,
#ifdef CONFIG_EXYNOS4_SDHCI_CH2_8BIT
	.max_width		= 8,
	.host_caps		= MMC_CAP_8_BIT_DATA |
				MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED |
				MMC_CAP_DISABLE,
#else
	.host_caps = MMC_CAP_4_BIT_DATA |
				MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED |
				MMC_CAP_DISABLE,
	.max_width		= 4,
#endif
};

static struct s3c_sdhci_platdata i9100_hsmmc3_pdata __initdata = {
	.cd_type		= S3C_SDHCI_CD_PERMANENT,
	.clk_type		= S3C_SDHCI_CLK_DIV_EXTERNAL,
};

/******************************************************************************
 * gpio table
 ******************************************************************************/
typedef enum {
	SGS_GPIO_SETPIN_ZERO,
	SGS_GPIO_SETPIN_ONE,
	SGS_GPIO_SETPIN_NONE
} sgs_gpio_initval;

static struct {
	unsigned int num;
	unsigned int cfg;
	sgs_gpio_initval val;
	samsung_gpio_pull_t pull;
	s5p_gpio_drvstr_t drv;
} i9100_init_gpios[] = {
	{
		.num	= GPIO_FM_RST,
		.cfg	= S3C_GPIO_INPUT,
		.val	= SGS_GPIO_SETPIN_ZERO,
		.pull	= S3C_GPIO_PULL_DOWN,
		.drv	= S5P_GPIO_DRVSTR_LV1,
	},
	 {
		.num	= GPIO_TDMB_RST_N,
		.cfg	= S3C_GPIO_INPUT,
		.val	= SGS_GPIO_SETPIN_ZERO,
		.pull	= S3C_GPIO_PULL_DOWN,
		.drv	= S5P_GPIO_DRVSTR_LV1,
	},
	{
		.num	= EXYNOS4_GPC1(3),	/* CODEC_SDA_1.8V */
		.cfg	= S3C_GPIO_INPUT,
		.val	= SGS_GPIO_SETPIN_NONE,
		.pull	= S3C_GPIO_PULL_NONE,
		.drv	= S5P_GPIO_DRVSTR_LV1,
	}, {
		.num	= EXYNOS4_GPC1(4),	/* CODEC_SCL_1.8V */
		.cfg	= S3C_GPIO_INPUT,
		.val	= SGS_GPIO_SETPIN_NONE,
		.pull	= S3C_GPIO_PULL_NONE,
		.drv	= S5P_GPIO_DRVSTR_LV1,
	}, {
		.num	= EXYNOS4_GPD0(2),	/* MSENSOR_MHL_SDA_2.8V */
		.cfg	= S3C_GPIO_INPUT,
		.val	= SGS_GPIO_SETPIN_NONE,
		.pull	= S3C_GPIO_PULL_NONE,
		.drv	= S5P_GPIO_DRVSTR_LV1,
	}, {
		.num	= EXYNOS4_GPD0(3),	/* MSENSOR_MHL_SCL_2.8V */
		.cfg	= S3C_GPIO_INPUT,
		.val	= SGS_GPIO_SETPIN_NONE,
		.pull	= S3C_GPIO_PULL_NONE,
		.drv	= S5P_GPIO_DRVSTR_LV1,
	}, {
		.num	= EXYNOS4_GPD1(0),	/* 8M_CAM_SDA_2.8V */
		.cfg	= S3C_GPIO_INPUT,
		.val	= SGS_GPIO_SETPIN_NONE,
		.pull	= S3C_GPIO_PULL_NONE,
		.drv	= S5P_GPIO_DRVSTR_LV1,
	}, {
		.num	= EXYNOS4_GPD1(1),	/* 8M_CAM_SCL_2.8V */
		.cfg	= S3C_GPIO_INPUT,
		.val	= SGS_GPIO_SETPIN_NONE,
		.pull	= S3C_GPIO_PULL_NONE,
		.drv	= S5P_GPIO_DRVSTR_LV1,
	}, {
		.num	= EXYNOS4_GPD1(2),	/* SENSE_SDA_2.8V */
		.cfg	= S3C_GPIO_INPUT,
		.val	= SGS_GPIO_SETPIN_NONE,
		.pull	= S3C_GPIO_PULL_NONE,
		.drv	= S5P_GPIO_DRVSTR_LV1,
	}, {
		.num	= EXYNOS4_GPD1(3),	/* SENSE_SCL_2.8V */
		.cfg	= S3C_GPIO_INPUT,
		.val	= SGS_GPIO_SETPIN_NONE,
		.pull	= S3C_GPIO_PULL_NONE,
		.drv	= S5P_GPIO_DRVSTR_LV1,
	},
	{
		.num	= EXYNOS4_GPK1(1),
		.cfg	= S3C_GPIO_OUTPUT,
		.val	= SGS_GPIO_SETPIN_ZERO,
		.pull	= S3C_GPIO_PULL_NONE,
		.drv	= S5P_GPIO_DRVSTR_LV2,
	}, {
		.num	= EXYNOS4_GPK2(2),	/* PS_ALS_SDA_2.8V */
		.cfg	= S3C_GPIO_INPUT,
		.val	= SGS_GPIO_SETPIN_NONE,
		.pull	= S3C_GPIO_PULL_NONE,
		.drv	= S5P_GPIO_DRVSTR_LV1,
	},
	{
		.num	= EXYNOS4_GPK3(1),	/* WLAN_SDIO_CMD */
		.cfg	= S3C_GPIO_INPUT,
		.val	= SGS_GPIO_SETPIN_NONE,
		.pull	= S3C_GPIO_PULL_NONE,
		.drv	= S5P_GPIO_DRVSTR_LV1,
	}, {
		.num	= EXYNOS4_GPK3(2),	/* PS_ALS_SCL_2.8V */
		.cfg	= S3C_GPIO_INPUT,
		.val	= SGS_GPIO_SETPIN_NONE,
		.pull	= S3C_GPIO_PULL_NONE,
		.drv	= S5P_GPIO_DRVSTR_LV1,
	}, {
		.num	= EXYNOS4_GPK3(3),	/* WLAN_SDIO_D(0) */
		.cfg	= S3C_GPIO_INPUT,
		.val	= SGS_GPIO_SETPIN_NONE,
		.pull	= S3C_GPIO_PULL_NONE,
		.drv	= S5P_GPIO_DRVSTR_LV1,
	}, {
		.num	= EXYNOS4_GPK3(4),	/* WLAN_SDIO_D(1) */
		.cfg	= S3C_GPIO_INPUT,
		.val	= SGS_GPIO_SETPIN_NONE,
		.pull	= S3C_GPIO_PULL_NONE,
		.drv	= S5P_GPIO_DRVSTR_LV1,
	}, {
		.num	= EXYNOS4_GPK3(5),	/* WLAN_SDIO_D(2) */
		.cfg	= S3C_GPIO_INPUT,
		.val	= SGS_GPIO_SETPIN_NONE,
		.pull	= S3C_GPIO_PULL_NONE,
		.drv	= S5P_GPIO_DRVSTR_LV1,
	}, {
		.num	= EXYNOS4_GPK3(6),	/* WLAN_SDIO_D(3) */
		.cfg	= S3C_GPIO_INPUT,
		.val	= SGS_GPIO_SETPIN_NONE,
		.pull	= S3C_GPIO_PULL_NONE,
		.drv	= S5P_GPIO_DRVSTR_LV1,
	}, {
		.num	= EXYNOS4_GPX0(1),	/* VOL_UP */
		.cfg	= S3C_GPIO_INPUT,
		.val	= SGS_GPIO_SETPIN_NONE,
		.pull	= S3C_GPIO_PULL_NONE,
		.drv	= S5P_GPIO_DRVSTR_LV1,
	},
	{
		.num	= EXYNOS4_GPX0(2),	/* VOL_DOWN */
		.cfg	= S3C_GPIO_INPUT,
		.val	= SGS_GPIO_SETPIN_NONE,
		.pull	= S3C_GPIO_PULL_NONE,
		.drv	= S5P_GPIO_DRVSTR_LV1,
	}, {
		.num	= EXYNOS4_GPX0(3),	/* GPIO_BOOT_MODE */
		.cfg	= S3C_GPIO_INPUT,
		.val	= SGS_GPIO_SETPIN_NONE,
		.pull	= S3C_GPIO_PULL_NONE,
		.drv	= S5P_GPIO_DRVSTR_LV1,
	}, {
		.num	= EXYNOS4_GPX2(3),	/* GPIO_FUEL_ALERT */
		.cfg	= S3C_GPIO_INPUT,
		.val	= SGS_GPIO_SETPIN_NONE,
		.pull	= S3C_GPIO_PULL_NONE,
		.drv	= S5P_GPIO_DRVSTR_LV1,
	}, {
		.num	= EXYNOS4_GPX3(1),
		.cfg	= S3C_GPIO_OUTPUT,
		.val	= SGS_GPIO_SETPIN_ZERO,
		.pull	= S3C_GPIO_PULL_NONE,
		.drv	= S5P_GPIO_DRVSTR_LV1,
	}, {
		.num	= EXYNOS4_GPX3(2),	/* GPIO_DET_35 */
		.cfg	= S3C_GPIO_SFN(GPIO_DET_35_AF),
		.val	= SGS_GPIO_SETPIN_NONE,
		.pull	= S3C_GPIO_PULL_NONE,
		.drv	= S5P_GPIO_DRVSTR_LV1,
	},
	{
		.num	= EXYNOS4_GPX3(3),
		.cfg	= S3C_GPIO_OUTPUT,
		.val	= SGS_GPIO_SETPIN_ZERO,
		.pull	= S3C_GPIO_PULL_NONE,
		.drv	= S5P_GPIO_DRVSTR_LV1,
	},
	{
		.num	= EXYNOS4_GPX3(4),
		.cfg	= S3C_GPIO_INPUT,
		.val	= SGS_GPIO_SETPIN_NONE,
		.pull	= S3C_GPIO_PULL_NONE,
		.drv	= S5P_GPIO_DRVSTR_LV1,
	},
	{	/*GPY0 */
		.num	= EXYNOS4_GPY0(2),
		.cfg	= S3C_GPIO_INPUT,
		.val	= SGS_GPIO_SETPIN_ZERO,
		.pull	= S3C_GPIO_PULL_DOWN,
		.drv	= S5P_GPIO_DRVSTR_LV1,
	}, {
		.num	= EXYNOS4_GPY0(3),
		.cfg	= S3C_GPIO_INPUT,
		.val	= SGS_GPIO_SETPIN_ZERO,
		.pull	= S3C_GPIO_PULL_DOWN,
		.drv	= S5P_GPIO_DRVSTR_LV1,
	}, {
		.num	= EXYNOS4_GPY0(4),
		.cfg	= S3C_GPIO_INPUT,
		.val	= SGS_GPIO_SETPIN_ZERO,
		.pull	= S3C_GPIO_PULL_DOWN,
		.drv	= S5P_GPIO_DRVSTR_LV1,
	}, {
		.num	= EXYNOS4_GPY0(5),
		.cfg	= S3C_GPIO_INPUT,
		.val	= SGS_GPIO_SETPIN_ZERO,
		.pull	= S3C_GPIO_PULL_DOWN,
		.drv	= S5P_GPIO_DRVSTR_LV1,
	},
	{	/*GPY1 */
		.num	= EXYNOS4_GPY1(0),
		.cfg	= S3C_GPIO_INPUT,
		.val	= SGS_GPIO_SETPIN_ZERO,
		.pull	= S3C_GPIO_PULL_DOWN,
		.drv	= S5P_GPIO_DRVSTR_LV1,
	}, {
		.num	= EXYNOS4_GPY1(1),
		.cfg	= S3C_GPIO_INPUT,
		.val	= SGS_GPIO_SETPIN_ZERO,
		.pull	= S3C_GPIO_PULL_DOWN,
		.drv	= S5P_GPIO_DRVSTR_LV1,
	},
	{
		.num	= EXYNOS4_GPY1(2),
		.cfg	= S3C_GPIO_INPUT,
		.val	= SGS_GPIO_SETPIN_ZERO,
		.pull	= S3C_GPIO_PULL_DOWN,
		.drv	= S5P_GPIO_DRVSTR_LV1,
	}, {
		.num	= EXYNOS4_GPY1(3),
		.cfg	= S3C_GPIO_INPUT,
		.val	= SGS_GPIO_SETPIN_ZERO,
		.pull	= S3C_GPIO_PULL_DOWN,
		.drv	= S5P_GPIO_DRVSTR_LV1,
	}, {	/*GPY2 */
		.num	= EXYNOS4_GPY2(0),
		.cfg	= S3C_GPIO_INPUT,
		.val	= SGS_GPIO_SETPIN_ZERO,
		.pull	= S3C_GPIO_PULL_DOWN,
		.drv	= S5P_GPIO_DRVSTR_LV1,
	}, {
		.num	= EXYNOS4_GPY2(1),
		.cfg	= S3C_GPIO_INPUT,
		.val	= SGS_GPIO_SETPIN_ZERO,
		.pull	= S3C_GPIO_PULL_DOWN,
		.drv	= S5P_GPIO_DRVSTR_LV1,
	}, {
		.num	= EXYNOS4_GPY2(2),
		.cfg	= S3C_GPIO_INPUT,
		.val	= SGS_GPIO_SETPIN_ZERO,
		.pull	= S3C_GPIO_PULL_DOWN,
		.drv	= S5P_GPIO_DRVSTR_LV1,
	}, {
		.num	= EXYNOS4_GPY2(3),
		.cfg	= S3C_GPIO_INPUT,
		.val	= SGS_GPIO_SETPIN_ZERO,
		.pull	= S3C_GPIO_PULL_DOWN,
		.drv	= S5P_GPIO_DRVSTR_LV1,
	}, {
		.num	= EXYNOS4_GPY2(4),
		.cfg	= S3C_GPIO_INPUT,
		.val	= SGS_GPIO_SETPIN_ZERO,
		.pull	= S3C_GPIO_PULL_DOWN,
		.drv	= S5P_GPIO_DRVSTR_LV1,
	}, {
		.num	= EXYNOS4_GPY2(5),
		.cfg	= S3C_GPIO_INPUT,
		.val	= SGS_GPIO_SETPIN_ZERO,
		.pull	= S3C_GPIO_PULL_DOWN,
		.drv	= S5P_GPIO_DRVSTR_LV1,
	}, {	/*GPY4 */
		.num	= EXYNOS4_GPY4(4),
		.cfg	= S3C_GPIO_INPUT,
		.val	= SGS_GPIO_SETPIN_ZERO,
		.pull	= S3C_GPIO_PULL_DOWN,
		.drv	= S5P_GPIO_DRVSTR_LV1,
	}, {	/*GPY5 */
		.num	= EXYNOS4_GPY5(0),
		.cfg	= S3C_GPIO_INPUT,
		.val	= SGS_GPIO_SETPIN_ZERO,
		.pull	= S3C_GPIO_PULL_DOWN,
		.drv	= S5P_GPIO_DRVSTR_LV1,
	}, {
		.num	= EXYNOS4_GPY5(1),
		.cfg	= S3C_GPIO_INPUT,
		.val	= SGS_GPIO_SETPIN_ZERO,
		.pull	= S3C_GPIO_PULL_DOWN,
		.drv	= S5P_GPIO_DRVSTR_LV1,
	}, {
		.num	= EXYNOS4_GPY5(2),
		.cfg	= S3C_GPIO_INPUT,
		.val	= SGS_GPIO_SETPIN_ZERO,
		.pull	= S3C_GPIO_PULL_DOWN,
		.drv	= S5P_GPIO_DRVSTR_LV1,
	}, {
		.num	= EXYNOS4_GPY5(3),
		.cfg	= S3C_GPIO_INPUT,
		.val	= SGS_GPIO_SETPIN_ZERO,
		.pull	= S3C_GPIO_PULL_DOWN,
		.drv	= S5P_GPIO_DRVSTR_LV1,
	}, {
		.num	= EXYNOS4_GPY5(4),
		.cfg	= S3C_GPIO_INPUT,
		.val	= SGS_GPIO_SETPIN_ZERO,
		.pull	= S3C_GPIO_PULL_DOWN,
		.drv	= S5P_GPIO_DRVSTR_LV1,
	}, {
		.num	= EXYNOS4_GPY5(5),
		.cfg	= S3C_GPIO_INPUT,
		.val	= SGS_GPIO_SETPIN_ZERO,
		.pull	= S3C_GPIO_PULL_DOWN,
		.drv	= S5P_GPIO_DRVSTR_LV1,
	}, {
		.num	= EXYNOS4_GPY5(6),
		.cfg	= S3C_GPIO_INPUT,
		.val	= SGS_GPIO_SETPIN_ZERO,
		.pull	= S3C_GPIO_PULL_DOWN,
		.drv	= S5P_GPIO_DRVSTR_LV1,
	}, {
		.num	= EXYNOS4_GPY5(7),
		.cfg	= S3C_GPIO_INPUT,
		.val	= SGS_GPIO_SETPIN_ZERO,
		.pull	= S3C_GPIO_PULL_DOWN,
		.drv	= S5P_GPIO_DRVSTR_LV1,
	}, {	/* GPY6 */
		.num	= EXYNOS4_GPY6(0),
		.cfg	= S3C_GPIO_INPUT,
		.val	= SGS_GPIO_SETPIN_ZERO,
		.pull	= S3C_GPIO_PULL_DOWN,
		.drv	= S5P_GPIO_DRVSTR_LV1,
	}, {
		.num	= EXYNOS4_GPY6(1),
		.cfg	= S3C_GPIO_INPUT,
		.val	= SGS_GPIO_SETPIN_ZERO,
		.pull	= S3C_GPIO_PULL_DOWN,
		.drv	= S5P_GPIO_DRVSTR_LV1,
	}, {
		.num	= EXYNOS4_GPY6(2),
		.cfg	= S3C_GPIO_INPUT,
		.val	= SGS_GPIO_SETPIN_ZERO,
		.pull	= S3C_GPIO_PULL_DOWN,
		.drv	= S5P_GPIO_DRVSTR_LV1,
	}, {
		.num	= EXYNOS4_GPY6(3),
		.cfg	= S3C_GPIO_INPUT,
		.val	= SGS_GPIO_SETPIN_ZERO,
		.pull	= S3C_GPIO_PULL_DOWN,
		.drv	= S5P_GPIO_DRVSTR_LV1,
	}, {
		.num	= EXYNOS4_GPY6(4),
		.cfg	= S3C_GPIO_INPUT,
		.val	= SGS_GPIO_SETPIN_ZERO,
		.pull	= S3C_GPIO_PULL_DOWN,
		.drv	= S5P_GPIO_DRVSTR_LV1,
	}, {
		.num	= EXYNOS4_GPY6(5),
		.cfg	= S3C_GPIO_INPUT,
		.val	= SGS_GPIO_SETPIN_ZERO,
		.pull	= S3C_GPIO_PULL_DOWN,
		.drv	= S5P_GPIO_DRVSTR_LV1,
	}, {
		.num	= EXYNOS4_GPY6(6),
		.cfg	= S3C_GPIO_INPUT,
		.val	= SGS_GPIO_SETPIN_ZERO,
		.pull	= S3C_GPIO_PULL_DOWN,
		.drv	= S5P_GPIO_DRVSTR_LV1,
	}, {
		.num	= EXYNOS4_GPY6(7),
		.cfg	= S3C_GPIO_INPUT,
		.val	= SGS_GPIO_SETPIN_ZERO,
		.pull	= S3C_GPIO_PULL_DOWN,
		.drv	= S5P_GPIO_DRVSTR_LV1,
	},
};

static void i9100_config_gpio_table(void)
{
	u32 i, gpio;

	for (i = 0; i < ARRAY_SIZE(i9100_init_gpios); i++) {
		gpio = i9100_init_gpios[i].num;
		s3c_gpio_cfgpin(gpio, i9100_init_gpios[i].cfg);
		s3c_gpio_setpull(gpio, i9100_init_gpios[i].pull);

		if (i9100_init_gpios[i].val != SGS_GPIO_SETPIN_NONE)
			gpio_set_value(gpio, i9100_init_gpios[i].val);

		s5p_gpio_set_drvstr(gpio, i9100_init_gpios[i].drv);
	}
}
/******************************************************************************
 * USB
 ******************************************************************************/
static struct s5p_ehci_platdata i9100_ehci_pdata;
static struct exynos4_ohci_platdata i9100_ohci_pdata;
static struct s5p_otg_platdata i9100_otg_pdata;

static void __init i9100_init_usb(void) {
	s5p_ehci_set_platdata(&i9100_ehci_pdata);
	exynos4_ohci_set_platdata(&i9100_ohci_pdata);
	s5p_otg_set_platdata(&i9100_otg_pdata);
}

/******************************************************************************
 * platform devices
 ******************************************************************************/
static struct platform_device *i9100_devices[] __initdata = {
	&s3c_device_i2c0,
	&s3c_device_i2c5,
	&s3c_device_i2c3,
	&emmc_fixed_voltage,
	&s3c_device_rtc,
	&s3c_device_hsmmc0,
	&s3c_device_hsmmc2,
	&s3c_device_hsmmc3,
	&s5p_device_fimc0,
	&s5p_device_fimc1,
	&s5p_device_fimc2,
	&s5p_device_fimc3,
	&s5p_device_fimc_md,
	&s5p_device_fimd0,
	&s5p_device_g2d,
	&s5p_device_mfc,
	&s5p_device_mfc_l,
	&s5p_device_mfc_r,
	&exynos4_device_pd[PD_MFC],
	&exynos4_device_pd[PD_G3D],
	&exynos4_device_pd[PD_LCD0],
	&exynos4_device_pd[PD_LCD1],
	&exynos4_device_pd[PD_CAM],
	&exynos4_device_pd[PD_GPS],
	&exynos4_device_pd[PD_TV],
	&exynos4_device_tmu,

	&lcd_spi_gpio,
	&s5p_device_ehci,
	&exynos4_device_ohci,
	&s3c_device_usbgadget,
	&i9100_device_gpio_keys,
	&i2c_gpio_touchkey,
};

static void __init i9100_pmic_init(void) {
	gpio_request(GPIO_PMIC_IRQ, "PMIC_IRQ");
	s3c_gpio_setpull(GPIO_PMIC_IRQ, S3C_GPIO_PULL_NONE);
}

/******************************************************************************
 * machine initialization
 ******************************************************************************/
static void __init i9100_map_io(void) {
	s5p_init_io(NULL, 0, S5P_VA_CHIPID);
	s3c24xx_init_clocks(24000000);
	s3c24xx_init_uarts(i9100_uartcfgs, ARRAY_SIZE(i9100_uartcfgs));
}

static void __init i9100_reserve(void)
{
	s5p_mfc_reserve_mem(0x43000000, 8 << 20, 0x51000000, 8 << 20);
}

static void __init i9100_machine_init(void) {
	i9100_pmic_init();

	i9100_config_gpio_table();
	
	s3c_sdhci0_set_platdata(&i9100_hsmmc0_pdata);
	s3c_sdhci2_set_platdata(&i9100_hsmmc2_pdata);
	s3c_sdhci3_set_platdata(&i9100_hsmmc3_pdata);
	
	s3c_i2c3_set_platdata(NULL);
	i2c_register_board_info(3, i2c3_devs, ARRAY_SIZE(i2c3_devs));

	s3c_i2c5_set_platdata(NULL);
	i2c5_devs[0].irq = gpio_to_irq(GPIO_PMIC_IRQ);
	i2c_register_board_info(5, i2c5_devs, ARRAY_SIZE(i2c5_devs));
	
	i9100_init_fb();
	i9100_init_touchkey();
	i9100_init_tsp();
	i9100_init_usb();
	clk_xusbxti.rate = 24000000,
	
	platform_add_devices(i9100_devices, ARRAY_SIZE(i9100_devices));

	s5p_device_mfc.dev.parent = &exynos4_device_pd[PD_MFC].dev;
	s5p_device_fimd0.dev.parent = &exynos4_device_pd[PD_LCD0].dev;
}

MACHINE_START(SGS_I9100, "i9100")
	.atag_offset	= 0x100,
	.init_irq	= exynos4_init_irq,
	.map_io		= i9100_map_io,
	.init_machine	= i9100_machine_init,
	.timer		= &exynos4_timer,
	.reserve        = &i9100_reserve,
MACHINE_END
