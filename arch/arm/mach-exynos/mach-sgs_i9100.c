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
#include <linux/leds-regulator.h>
#include <linux/mfd/max8997.h>
#include <linux/lcd.h>
#include <linux/power/max17042_battery.h>
#include <linux/rfkill-gpio.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_gpio.h>
#include <linux/platform_data/fsa9480.h>

#include <media/s5p_fimc.h>
#include <media/m5mols.h>
#include <media/v4l2-mediabus.h>

#include <asm/mach/arch.h>
#include <asm/hardware/gic.h>
#include <asm/mach-types.h>

#include <video/platform_lcd.h>
#include <plat/regs-serial.h>
#include <plat/regs-srom.h>
#include <plat/regs-fb-v4.h>
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
#include <plat/fimc-core.h>
#include <plat/camport.h>
#include <plat/mipi_csis.h>

#include <mach/map.h>
#include <mach/sgs_i9100.h>

#include "common.h"

enum fixed_regulator_id {
	FIXED_REG_ID_MMC = 0,
	FIXED_REG_ID_WLAN,
	FIXED_REG_ID_CAM_A28V,
	FIXED_REG_ID_CAM_12V,
	FIXED_REG_ID_CAM_28V,
};

enum i2c_bus_ids {
	I2C_GPIO_BUS_TOUCHKEY = 10,
	I2C_GPIO_BUS_PROX = 11,
	I2C_GPIO_BUS_GAUGE,
	I2C_GPIO_BUS_USB,
	I2C_GPIO_BUS_MHL,
	I2C_GPIO_BUS_FM,
};

enum sgs2_rfk_id {
	RFK_ID_BT,
	RFK_ID_GPS,
};

#define M5MO_VREG_CONSUMER "0-001f"
#define CM3663_VREG_CONSUMER "11-0020"
#define MODEM_VREG "vhsic-modem"

static struct max8997_muic_platform_data i9100_max8997_muic_pdata;
static struct max8997_led_platform_data i9100_max8997_led_pdata;
static struct max8997_haptic_platform_data i9100_max8997_haptic_pdata;
static void i9100_set_usb_path(bool to_ap);

//Will be filled during probe
static unsigned i9100_hw_revision = 0;

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
	REGULATOR_SUPPLY("vdd", "samsung-adc-v3")
};

static struct regulator_consumer_supply ldo3_supply[] = {
	REGULATOR_SUPPLY("pd_io", NULL),
	REGULATOR_SUPPLY("vdd11", NULL), /* MIPI */
	//REGULATOR_SUPPLY("vdd11", "s5p-mipi-csis.0"), /* MIPI */
	REGULATOR_SUPPLY("vdd", "exynos4-hdmi"), /* HDMI */
	REGULATOR_SUPPLY("vdd_pll", "exynos4-hdmi"), /* HDMI */
};

static struct regulator_consumer_supply ldo4_supply[] = {
	REGULATOR_SUPPLY("vdd18", NULL), /* MIPI */
	//REGULATOR_SUPPLY("vdd18", "s5p-mipi-csis.0"), /* MIPI */
};

static struct regulator_consumer_supply ldo5_supply[] = {
	REGULATOR_SUPPLY("vhsic", NULL), /* MODEM */
	REGULATOR_SUPPLY(MODEM_VREG, NULL),
};

static struct regulator_consumer_supply ldo7_supply[] = {
	REGULATOR_SUPPLY("dig_18", M5MO_VREG_CONSUMER),
};

static struct regulator_consumer_supply ldo8_supply[] = {
	//vusb_3.3v
	REGULATOR_SUPPLY("pd_core", NULL),
	REGULATOR_SUPPLY("vusb_3.3v", NULL),
	REGULATOR_SUPPLY("vusb_d", NULL), /* Used by CPU */
	REGULATOR_SUPPLY("vdac", NULL), /* Used by CPU */
	REGULATOR_SUPPLY("vdd_osc", "exynos4-hdmi"), /* HDMI */
};

static struct regulator_consumer_supply ldo10_supply[] = {
	REGULATOR_SUPPLY("vpll_1.2v", NULL),
	REGULATOR_SUPPLY("vpll_1.1v", NULL),
};

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
	REGULATOR_SUPPLY("inmotor", "max8997-haptic"),
};

static struct regulator_consumer_supply ldo15_supply[] = {
	REGULATOR_SUPPLY("vled", CM3663_VREG_CONSUMER),
};

static struct regulator_consumer_supply ldo16_supply[] = {
	REGULATOR_SUPPLY("d_sensor", M5MO_VREG_CONSUMER),
};

static struct regulator_consumer_supply ldo17_rev04_supply[] = {
	REGULATOR_SUPPLY("vmmc", "s3c-sdhci.2"),
};

static struct regulator_consumer_supply ldo18_supply[] = {
	REGULATOR_SUPPLY("vled", "leds-regulator.0"),
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
	REGULATOR_SUPPLY("core", M5MO_VREG_CONSUMER),
};

static struct regulator_consumer_supply buck7_supply[] = {
	REGULATOR_SUPPLY("vcc_sub", NULL),
};

static struct regulator_consumer_supply safeout1_supply[] = {
	REGULATOR_SUPPLY("safeout1", NULL),
	//REGULATOR_SUPPLY("usb_vbus", NULL), /* CPU's USB OTG */
};

static struct regulator_consumer_supply safeout2_supply[] = {
	REGULATOR_SUPPLY("safeout2", NULL),
	//REGULATOR_SUPPLY("usb_vbus", "modemctl"), /* VBUS of Modem */
};

static struct regulator_consumer_supply led_flash_supply[] = {
	REGULATOR_SUPPLY("led_flash", NULL),
};

static struct regulator_consumer_supply led_movie_supply[] = {
	REGULATOR_SUPPLY("led_movie", NULL),
};

static struct regulator_init_data ldo1_init_data = {
	.constraints = {
		.name = "VADC_3.3V_C210",
		.min_uV = 3300000,
		.max_uV = 3300000,
		.always_on = 1,
		.boot_on = 1,
		.apply_uV = 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled = 1,
		}
	},
	.num_consumer_supplies = ARRAY_SIZE(ldo1_supply),
	.consumer_supplies = ldo1_supply,
};

static struct regulator_init_data ldo3_init_data = {
	.constraints = {
		.name = "VUSB_1.1V",
		.min_uV = 1100000,
		.max_uV = 1100000,
		.always_on = 1,
		.boot_on = 1,
		.apply_uV = 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled = 1,
		}
	},
	.num_consumer_supplies = ARRAY_SIZE(ldo3_supply),
	.consumer_supplies = ldo3_supply,
};

static struct regulator_init_data ldo4_init_data = {
	.constraints = {
		.name = "VMIPI_1.8V",
		.min_uV = 1800000,
		.max_uV = 1800000,
		.always_on = 1,
		.boot_on = 1,
		.apply_uV = 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled = 1,
		}
	},
	.num_consumer_supplies = ARRAY_SIZE(ldo4_supply),
	.consumer_supplies = ldo4_supply,
};

static struct regulator_init_data ldo5_init_data = {
	.constraints = {
		.name = "VHSIC_1.2V",
		.min_uV = 1200000,
		.max_uV = 1200000,
		.always_on = 1,
		.boot_on = 1,
		.apply_uV = 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled = 1,
		}
	},
	.num_consumer_supplies = ARRAY_SIZE(ldo5_supply),
	.consumer_supplies = ldo5_supply,
};

static struct regulator_init_data ldo7_init_data = {
	.constraints = {
		.name = "CAM_ISP_1.8V",
		.min_uV = 1800000,
		.max_uV = 1800000,
		.apply_uV = 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled = 1,
		}
	},
	.num_consumer_supplies = ARRAY_SIZE(ldo7_supply),
	.consumer_supplies = ldo7_supply,
};

static struct regulator_init_data ldo8_init_data = {
	.constraints = {
		.name = "VUSB_3.3V",
		.min_uV = 3300000,
		.max_uV = 3300000,
		.always_on = 1,
		.boot_on = 1,
		.apply_uV = 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled = 1,
		}
	},
	.num_consumer_supplies = ARRAY_SIZE(ldo8_supply),
	.consumer_supplies = ldo8_supply,
};

static struct regulator_init_data ldo10_init_data = {
	.constraints = {
		.name = "VPLL_1.2V",
		.min_uV = 1200000,
		.max_uV = 1200000,
		.always_on = 1,
		.boot_on = 1,
		.apply_uV = 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled = 1,
		}
	},
	.num_consumer_supplies = ARRAY_SIZE(ldo10_supply),
	.consumer_supplies = ldo10_supply,
};

static struct regulator_init_data ldo11_init_data = {
	.constraints = {
		.name = "TOUCH_2.8V",
		.min_uV = 2800000,
		.max_uV = 2800000,
		.always_on = 1,
		.boot_on = 1,
		.apply_uV = 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled = 1,
		}
	},
	.num_consumer_supplies = ARRAY_SIZE(ldo11_supply),
	.consumer_supplies = ldo11_supply,
};

static struct regulator_init_data ldo12_init_data = {
	.constraints = {
		.name = "VT_CAM_1.8V",
		.min_uV = 1800000,
		.max_uV = 1800000,
		.apply_uV = 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled = 1,
		}
	},
	.num_consumer_supplies = ARRAY_SIZE(ldo12_supply),
	.consumer_supplies = ldo12_supply,
};

static struct regulator_init_data ldo13_init_data = {
	.constraints = {
		.name = "VCC_3.0V_LCD",
		.min_uV = 3000000,
		.max_uV = 3000000,
		.always_on = 1,
		.boot_on = 1,
		.apply_uV = 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled = 1,
		}
	},
	.num_consumer_supplies = ARRAY_SIZE(ldo13_supply),
	.consumer_supplies = ldo13_supply,
};

static struct regulator_init_data ldo14_init_data = {
	.constraints = {
		.name = "VCC_2.8V_MOTOR",
		.min_uV = 2800000,
		.max_uV = 2800000,
		.apply_uV = 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled = 1,
		}
	},
	.num_consumer_supplies = ARRAY_SIZE(ldo14_supply),
	.consumer_supplies = ldo14_supply,
};

static struct regulator_init_data ldo15_init_data = {
	.constraints = {
		.name = "LED_A_2.8V",
		.min_uV = 2800000,
		.max_uV = 2800000,
		.always_on = 1,
		.boot_on = 1,
		.apply_uV = 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled = 1,
		}
	},
	.num_consumer_supplies = ARRAY_SIZE(ldo15_supply),
	.consumer_supplies = ldo15_supply,
};

static struct regulator_init_data ldo16_init_data = {
	.constraints = {
		.name = "CAM_SENSOR_IO_1.8V",
		.min_uV = 1800000,
		.max_uV = 1800000,
		.apply_uV = 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled = 1,
		}
	},
	.num_consumer_supplies = ARRAY_SIZE(ldo16_supply),
	.consumer_supplies = ldo16_supply,
};

static struct regulator_init_data ldo17_rev04_init_data = {
	.constraints = {
		.name = "VTF_2.8V",
		.min_uV = 2800000,
		.max_uV = 2800000,
		.apply_uV = 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled = 1,
		}
	},
	.num_consumer_supplies = ARRAY_SIZE(ldo17_rev04_supply),
	.consumer_supplies = ldo17_rev04_supply,
};

static struct regulator_init_data ldo18_init_data = {
	.constraints = {
		.name = "TOUCH_LED_3.3V",
		.min_uV = 3000000,
		.max_uV = 3300000,
		.apply_uV = 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_VOLTAGE,
		.state_mem = {
			.disabled = 1,
		}
	},
	.num_consumer_supplies = ARRAY_SIZE(ldo18_supply),
	.consumer_supplies = ldo18_supply,
};

static struct regulator_init_data ldo21_init_data = {
	.constraints = {
		.name = "VDDQ_M1M2_1.2V",
		.min_uV = 1200000,
		.max_uV = 1200000,
		.always_on = 1,
		.boot_on = 1,
		.apply_uV = 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled = 1,
		}
	},
	.num_consumer_supplies = ARRAY_SIZE(ldo21_supply),
	.consumer_supplies = ldo21_supply,
};

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
		.name		= "SAFEOUT1",
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.boot_on = 1,
		.state_mem	= {
			.enabled = 1,
		},
	},
	.num_consumer_supplies	= ARRAY_SIZE(safeout1_supply),
	.consumer_supplies	= safeout1_supply,
};

static struct regulator_init_data safeout2_init_data = {
	.constraints	= {
		.name		= "SAFEOUT2",
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
	{ MAX8997_LDO17, &ldo17_rev04_init_data },
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
	.num_regulators		= ARRAY_SIZE(i9100_max8997_regulators),
	.regulators		= i9100_max8997_regulators,

	.wakeup			= true,
	.irq_base		= IRQ_GPIO_END + 1,
	
	.buck125_default_idx = 0x1,
	.buck125_gpios = {GPIO_BUCK1_EN_A, GPIO_BUCK1_EN_B, GPIO_BUCK2_EN},
	.buck1_gpiodvs	= true,
	.buck2_gpiodvs	= true,

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

	.muic_pdata = &i9100_max8997_muic_pdata,
	.led_pdata = &i9100_max8997_led_pdata,
	.haptic_pdata = &i9100_max8997_haptic_pdata,
};

static struct i2c_board_info i2c5_devs[] __initdata = {
	{
		I2C_BOARD_INFO("max8997", 0xCC >> 1),
		.platform_data	= &i9100_max8997_pdata,
	},
};

static void __init i9100_init_pmic(void) {
	gpio_request(GPIO_PMIC_IRQ, "PMIC_IRQ");
	s5p_register_gpio_interrupt(GPIO_PMIC_IRQ);
	s3c_gpio_cfgpin(GPIO_PMIC_IRQ, S3C_GPIO_SFN(0xf));
	s3c_gpio_setpull(GPIO_PMIC_IRQ, S3C_GPIO_PULL_UP);
	i2c5_devs[0].irq = gpio_to_irq(GPIO_PMIC_IRQ);

}

/******************************************************************************
 * EMMC voltage regulator
 ******************************************************************************/
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
 * WLAN voltage regulator
 ******************************************************************************/
static struct regulator_consumer_supply wlan_supplies[] = {
	REGULATOR_SUPPLY("vmmc", "s3c-sdhci.3"),
};

static struct regulator_init_data wlan_fixed_voltage_init_data = {
	.constraints		= {
		.name		= "WLAN_VDD",
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= ARRAY_SIZE(wlan_supplies),
	.consumer_supplies	= wlan_supplies,
};

static struct fixed_voltage_config wlan_fixed_voltage_config = {
	.supply_name		= "WLAN_EN",
	.microvolts		= 3300000,
	.gpio			= GPIO_WLAN_EN,
	.enable_high		= true,
	.init_data		= &wlan_fixed_voltage_init_data,
};

static struct platform_device wlan_fixed_voltage = {
	.name			= "reg-fixed-voltage",
	.id			= FIXED_REG_ID_WLAN,
	.dev			= {
		.platform_data	= &wlan_fixed_voltage_config,
	},
};

/******************************************************************************
 * max8997 devices
 ******************************************************************************/
static void i9100_muic_uart_callback(bool attached) {
	gpio_direction_output(GPIO_UART_SEL, !attached);
}

static void i9100_muic_mhl_callback(bool attached) {
}

static void i9100_muic_cardock_callback(bool attached) {
}

static void i9100_muic_deskdock_callback(bool attached) {
}

static void i9100_muic_usb_callback(enum max8997_muic_usb_type type,
	bool attached)
{
	//i9100_set_usb_path(attached);
}

static void i9100_muic_chg_callback(bool attached,
	enum max8997_muic_charger_type type)
{
	printk("%s: type %d, attached %d\n", __func__, type, attached);
}

static struct max8997_muic_platform_data i9100_max8997_muic_pdata = {
	.usb_callback = i9100_muic_usb_callback,
	.charger_callback = i9100_muic_chg_callback,
	.deskdock_callback = i9100_muic_deskdock_callback,
	.cardock_callback = i9100_muic_cardock_callback,
	.mhl_callback = i9100_muic_mhl_callback,
	.uart_callback = i9100_muic_uart_callback,
};

static struct max8997_led_platform_data i9100_max8997_led_pdata = {
	.mode = {
		MAX8997_FLASH_MODE,
		MAX8997_MOVIE_MODE,
	},
	.brightness = {
		0,
		0,
	}
};

static struct max8997_haptic_platform_data i9100_max8997_haptic_pdata = {
	.pwm_channel_id = 1,
	.pwm_period = 30000,
	.type = MAX8997_HAPTIC_ERM,
	.mode = MAX8997_EXTERNAL_MODE,
	.pwm_divisor = MAX8997_PWM_DIVISOR_128,
};

/******************************************************************************
 * gpio keys
 ******************************************************************************/
static struct gpio_keys_button i9100_gpio_keys[] = {
	{
		.code = KEY_VOLUMEUP,
		.gpio = GPIO_VOL_UP,
		.active_low = 1,
		.type = EV_KEY,
		.wakeup = 1,
		.can_disable = 1,
	}, {
		.code = KEY_VOLUMEDOWN,
		.gpio = GPIO_VOL_DOWN,
		.active_low = 1,
		.type = EV_KEY,
		.wakeup = 1,
		.can_disable = 1,
	}, {
		.code = KEY_POWER,
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
 * gyroscope and accelerometer
 ******************************************************************************/
enum i9100_i2c1_ids {
	I2C1_GYRO,
};

static struct i2c_board_info i2c1_devs[] __initdata = {
	[I2C1_GYRO] = {
		I2C_BOARD_INFO("k3g", 0x69),
	},
};

static void __init i9100_init_k3_sensors(void) {
	gpio_request(GPIO_GYRO_FIFOP_INT, "GYRO FIFO INT");
	s5p_register_gpio_interrupt(GPIO_GYRO_FIFOP_INT);
	s3c_gpio_cfgpin(GPIO_GYRO_FIFOP_INT, S3C_GPIO_SFN(0xf));
	s3c_gpio_setpull(GPIO_GYRO_FIFOP_INT, S3C_GPIO_PULL_UP);
	i2c1_devs[I2C1_GYRO].irq = gpio_to_irq(GPIO_GYRO_FIFOP_INT);
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
	reg |= 3;
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

static int ld9040_reset(struct lcd_device *ld) {
	gpio_direction_output(GPIO_LCD_RESET, 0);
	mdelay(10);
	gpio_direction_output(GPIO_LCD_RESET, 1);
	mdelay(10);

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
	} else {
		regulator_disable(regulator);
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
		.modalias = "ld9040",
		.max_speed_hz	= 1200000,
		.bus_num	= I9100_SPI_LCD_BUS,
		.chip_select	= 0,
		.mode	= SPI_MODE_3,
		.controller_data = (void*)GPIO_LCD_SPI_CS,
		.platform_data	= &ld9040_platform_data,
	}
};

static void __init ld9040_cfg_gpio(void)
{
	s3c_gpio_cfgpin(GPIO_LCD_RESET, S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(GPIO_LCD_RESET, S3C_GPIO_PULL_NONE);

	s3c_gpio_cfgpin(GPIO_LCD_SPI_CS, S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(GPIO_LCD_SPI_CS, S3C_GPIO_PULL_NONE);
	
	s3c_gpio_cfgpin(GPIO_LCD_SPI_SCK, S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(GPIO_LCD_SPI_SCK, S3C_GPIO_PULL_NONE);
	
	s3c_gpio_cfgpin(GPIO_LCD_SPI_MOSI, S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(GPIO_LCD_SPI_MOSI, S3C_GPIO_PULL_NONE);
}

static void __init i9100_init_fb(void) {
	if (gpio_request(GPIO_LCD_RESET, "LCD Reset")) {
		pr_err("%s: failed to request LCD Reset gpio\n", __func__);
	}
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
	.cfg_gpio = exynos4_setup_sdhci0_cfg_gpio,
};

static struct s3c_sdhci_platdata i9100_hsmmc2_pdata __initdata = {
	.cd_type		= S3C_SDHCI_CD_GPIO,
	.clk_type		= S3C_SDHCI_CLK_DIV_EXTERNAL,
	.ext_cd_gpio	= GPIO_HSMMC2_CD,
	.ext_cd_gpio_invert = 1,
	.host_caps = MMC_CAP_4_BIT_DATA |
				MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED |
				MMC_CAP_DISABLE,
	.max_width		= 4,
	.cfg_gpio = exynos4_setup_sdhci2_cfg_gpio,
};

static struct s3c_sdhci_platdata i9100_hsmmc3_pdata __initdata = {
	.cd_type		= S3C_SDHCI_CD_PERMANENT,
	.clk_type		= S3C_SDHCI_CLK_DIV_EXTERNAL,
	.max_width		= 4,
	.host_caps		= MMC_CAP_4_BIT_DATA |
				MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED,
	.cfg_gpio = exynos4_setup_sdhci3_cfg_gpio,
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
static void i9100_set_usb_path(bool to_ap) {
	struct regulator *s1, *s2;

	s1 = regulator_get(NULL, "safeout1");
	if (!s1) {
		pr_err("%s: failed to get safeout1\n", __func__);
		return;
	}

	s2 = regulator_get(NULL, "safeout2");
	if (!s2) {
		pr_err("%s: failed to get safeout2\n", __func__);
		goto fail_safeout2;
	}

	if (to_ap) {
		regulator_enable(s1);
		regulator_disable(s2);
	}
	else {
		regulator_disable(s1);
		regulator_enable(s2);
	}

	regulator_put(s2);
fail_safeout2:
	regulator_put(s1);
}

static int i9100_set_usb_mipi(bool enable)
{
	struct regulator *mipi18_regulator;
	struct regulator *hsic12_regulator;
	int ret = 0;

	mipi18_regulator = regulator_get(NULL, "vdd18");
	if (IS_ERR(mipi18_regulator)) {
		pr_err("%s: failed to get %s\n", __func__, "vdd18");
		ret = -ENODEV;
		goto out1;
	}

	hsic12_regulator = regulator_get(NULL, "vhsic");
	if (IS_ERR(hsic12_regulator)) {
		pr_err("%s: failed to get %s\n", __func__, "vhsic 1.2v");
		ret = -ENODEV;
		goto out2;
	}

	if (enable) {
		regulator_enable(hsic12_regulator);
		regulator_enable(mipi18_regulator);
	} else {
		regulator_disable(mipi18_regulator);
		regulator_disable(hsic12_regulator);
	}

	regulator_put(hsic12_regulator);
out2:
	regulator_put(mipi18_regulator);
out1:
	return ret;
}

static struct s5p_ehci_platdata i9100_ehci_pdata;
static struct s5p_otg_platdata i9100_otg_pdata;

static void __init i9100_init_usb(void) {
	gpio_request(GPIO_USB_SEL, "USB Route");
	s3c_gpio_cfgpin(GPIO_USB_SEL, S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(GPIO_USB_SEL, S3C_GPIO_PULL_NONE);
	gpio_direction_output(GPIO_USB_SEL, 0);
	
	s5p_ehci_set_platdata(&i9100_ehci_pdata);
	s5p_otg_set_platdata(&i9100_otg_pdata);
}
/******************************************************************************
 * FM radio
 ******************************************************************************/
static struct i2c_gpio_platform_data i2c_gpio_fm_data = {
	.sda_pin	= GPIO_FM_SDA_28V,
	.scl_pin	= GPIO_FM_SCL_28V,
	.udelay		= 2,
};

struct platform_device i2c_gpio_fm = {
	.name = "i2c-gpio",
	.id = I2C_GPIO_BUS_FM,
	.dev.platform_data = &i2c_gpio_fm_data,
};

static struct i2c_board_info i2c_gpio_fm_devs[] __initdata = {
	{
		I2C_BOARD_INFO("si470x", 0x10),
	},
};

static void __init i9100_init_fm(void)
{
	int gpio_int = GPIO_FM_INT;
	if (i9100_hw_revision > 7) {
		gpio_int = GPIO_FM_INT_REV07;
	}
	gpio_request(gpio_int, "FM IRQ");
	s5p_register_gpio_interrupt(gpio_int);
	s3c_gpio_cfgpin(gpio_int, S3C_GPIO_SFN(0xF));
	s3c_gpio_setpull(gpio_int, S3C_GPIO_PULL_UP);
	i2c_gpio_fm_devs[0].irq = gpio_to_irq(gpio_int);
	
	gpio_request(GPIO_FM_RST, "FM Reset");
	s3c_gpio_cfgpin(GPIO_FM_RST, S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(GPIO_FM_RST, S3C_GPIO_PULL_NONE);
	gpio_direction_output(GPIO_FM_RST, 0);
	msleep(1);
	gpio_direction_output(GPIO_FM_RST, 1);
	msleep(2);

	i2c_register_board_info(I2C_GPIO_BUS_FM,
		i2c_gpio_fm_devs, ARRAY_SIZE(i2c_gpio_fm_devs));
}
/******************************************************************************
 * USB switch
 ******************************************************************************/
static struct i2c_gpio_platform_data i2c_gpio_usb_data = {
	.sda_pin	= GPIO_USB_SDA,
	.scl_pin	= GPIO_USB_SCL,
};

struct platform_device i2c_gpio_usb = {
	.name = "i2c-gpio",
	.id = I2C_GPIO_BUS_USB,
	.dev.platform_data = &i2c_gpio_usb_data,
};

static struct fsa9480_platform_data fsa9480_info = {
};

static struct i2c_board_info i2c_gpio_usb_devs[] __initdata = {
	{
		I2C_BOARD_INFO("fsa9480", 0x25),
		.platform_data = &fsa9480_info,
	},
};

static void __init i9100_init_usb_switch(void)
{
	i2c_register_board_info(I2C_GPIO_BUS_USB,
		i2c_gpio_usb_devs, ARRAY_SIZE(i2c_gpio_usb_devs));
}

/******************************************************************************
 * Battery gauge
 ******************************************************************************/
static struct i2c_gpio_platform_data i2c_gpio_gauge_data = {
	.sda_pin	= GPIO_FUEL_SDA,
	.scl_pin	= GPIO_FUEL_SCL,
};

struct platform_device i2c_gpio_gauge = {
	.name = "i2c-gpio",
	.id = I2C_GPIO_BUS_GAUGE,
	.dev.platform_data = &i2c_gpio_gauge_data,
};

static struct max17042_reg_data i9100_max17042_regs[] = {
	{
		.addr = MAX17042_CGAIN,
		.data = 0,
	},
	{
		.addr = MAX17042_MiscCFG,
		.data = 0x0003,
	},
	{
		.addr = MAX17042_LearnCFG,
		.data = 0x0007,
	},
	{
		.addr = MAX17042_RCOMP0,
		.data = 0x0050,
	},
	{
		.addr = MAX17042_SALRT_Th,
		.data = 0xff02,
	},
	{
		.addr = MAX17042_VALRT_Th,
		.data = 0xff00,
	},
	{
		.addr = MAX17042_TALRT_Th,
		.data = 0x7f80,
	}
};

static struct max17042_platform_data i9100_max17042_data = {
	.init_data = i9100_max17042_regs,
	.num_init_data = ARRAY_SIZE(i9100_max17042_regs),
};

static struct i2c_board_info i2c_gpio_gauge_devs[] __initdata = {
	{
		I2C_BOARD_INFO("max17042", 0x36),
		.platform_data = &i9100_max17042_data,
	},
};

static void __init i9100_init_battery_gauge(void)
{
	gpio_request(GPIO_FUEL_ALERT, "FUEL_ALERT");
	s5p_register_gpio_interrupt(GPIO_FUEL_ALERT);
	s3c_gpio_cfgpin(GPIO_FUEL_ALERT, S3C_GPIO_SFN(0xf));
	s3c_gpio_setpull(GPIO_FUEL_ALERT, S3C_GPIO_PULL_UP);
	i2c_gpio_gauge_devs[0].irq = gpio_to_irq(GPIO_FUEL_ALERT);
	
	i2c_register_board_info(I2C_GPIO_BUS_GAUGE,
		i2c_gpio_gauge_devs, ARRAY_SIZE(i2c_gpio_gauge_devs));
}
/******************************************************************************
 * camera
 ******************************************************************************/
static struct regulator_consumer_supply cam_vdda_supply[] = {
	REGULATOR_SUPPLY("a_sensor", M5MO_VREG_CONSUMER),
};

static struct regulator_init_data cam_vdda_reg_init_data = {
	.constraints = {
		.valid_ops_mask = REGULATOR_CHANGE_STATUS
	},
	.num_consumer_supplies = ARRAY_SIZE(cam_vdda_supply),
	.consumer_supplies = cam_vdda_supply,
};

static struct fixed_voltage_config cam_vdda_fixed_voltage_cfg = {
	.supply_name	= "CAM_IO_EN",
	.microvolts	= 2800000,
	.gpio		= GPIO_CAM_IO_EN,
	.enable_high	= 1,
	.init_data	= &cam_vdda_reg_init_data,
};

static struct platform_device cam_vdda_fixed_rdev = {
	.name = "reg-fixed-voltage",
	.id = FIXED_REG_ID_CAM_A28V,
	.dev = {
		.platform_data	= &cam_vdda_fixed_voltage_cfg
	},
};

static struct regulator_consumer_supply camera_8m_12v_supply =
	REGULATOR_SUPPLY("dig_12", M5MO_VREG_CONSUMER);

static struct regulator_init_data cam_8m_12v_reg_init_data = {
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &camera_8m_12v_supply,
	.constraints = {
		.valid_ops_mask = REGULATOR_CHANGE_STATUS
	},
};

static struct fixed_voltage_config cam_8m_12v_fixed_voltage_cfg = {
	.supply_name	= "8M_1.2V",
	.microvolts	= 1200000,
	.gpio		= GPIO_CAM_SENSOR_CORE,
	.enable_high	= 1,
	.init_data	= &cam_8m_12v_reg_init_data,
};

static struct platform_device cam_8m_12v_fixed_rdev = {
	.name = "reg-fixed-voltage",
	.id = FIXED_REG_ID_CAM_12V,
	.dev = {
		.platform_data = &cam_8m_12v_fixed_voltage_cfg
	},
};

static struct regulator_consumer_supply camera_8m_28v_supply =
	REGULATOR_SUPPLY("dig_28", M5MO_VREG_CONSUMER);

static struct regulator_init_data cam_8m_28v_reg_init_data = {
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &camera_8m_28v_supply,
	.constraints = {
		.valid_ops_mask = REGULATOR_CHANGE_STATUS
	},
};

static struct fixed_voltage_config cam_8m_28v_fixed_voltage_cfg = {
	.supply_name	= "8M_2.8V",
	.microvolts	= 2800000,
	.gpio		= GPIO_8M_AF_EN,
	.enable_high	= 1,
	.init_data	= &cam_8m_28v_reg_init_data,
};

static struct platform_device cam_8m_28v_fixed_rdev = {
	.name = "reg-fixed-voltage",
	.id = FIXED_REG_ID_CAM_28V,
	.dev = {
		.platform_data = &cam_8m_28v_fixed_voltage_cfg
	},
};

static int m5mols_set_power(struct device *dev, int on)
{
	gpio_set_value(GPIO_VT_CAM_15V, !!on);
	return 0;
}

static struct m5mols_platform_data m5mols_platdata = {
	.gpio_reset	= GPIO_ISP_RESET,
	.set_power	= m5mols_set_power,
};

static struct i2c_board_info m5mols_board_info = {
	I2C_BOARD_INFO("M5MOLS", 0x1F),
	.platform_data = &m5mols_platdata,
};

static struct s5p_fimc_isp_info i9100_camera_sensors[] = {
	{
		.flags		= V4L2_MBUS_PCLK_SAMPLE_FALLING |
				  V4L2_MBUS_VSYNC_ACTIVE_LOW,
		.bus_type	= FIMC_MIPI_CSI2,
		.board_info	= &m5mols_board_info,
		.clk_frequency	= 24000000UL,
		.csi_data_align	= 32,
	},
};

static struct s5p_platform_fimc fimc_md_platdata = {
	.isp_info	= i9100_camera_sensors,
	.num_clients	= ARRAY_SIZE(i9100_camera_sensors),
};

static struct s5p_platform_mipi_csis mipi_csis_platdata = {
	.clk_rate	= 166000000UL,
	.lanes		= 2,
	.alignment	= 32,
	.hs_settle	= 12,
	.phy_enable	= s5p_csis_phy_enable,
};

static struct gpio i9100_cam_gpios[] = {
	{
		.gpio = GPIO_CAM_8M_ISP_INT,
		.flags = GPIOF_IN,
		.label = "8M_ISP_INT"
	},
	{
		.gpio = GPIO_VT_CAM_15V,
		.flags = GPIOF_OUT_INIT_LOW,
		.label = "CAM_VT_1.5V",
	},
};

static void __init i9100_init_camera(void) {
	s3c_set_platdata(&mipi_csis_platdata, sizeof(mipi_csis_platdata),
			&s5p_device_mipi_csis0);
	s3c_set_platdata(&fimc_md_platdata,  sizeof(fimc_md_platdata),
			&s5p_device_fimc_md);
	
	if (gpio_request_array(i9100_cam_gpios, ARRAY_SIZE(i9100_cam_gpios))) {
		pr_err("%s: failed to request GPIO\n", __func__);
		return;
	}

	if (!s3c_gpio_cfgpin(GPIO_CAM_8M_ISP_INT, S3C_GPIO_SFN(0xf))) {
		s5p_register_gpio_interrupt(GPIO_CAM_8M_ISP_INT);
		m5mols_board_info.irq = gpio_to_irq(GPIO_CAM_8M_ISP_INT);
	}
	else {
		pr_err("Failed to configure 8M_ISP_INT GPIO\n");
	}

	if (exynos4_fimc_setup_gpio(S5P_CAMPORT_A)) {
		pr_err("%s: Camera port A setup failed\n", __func__);
		return;
	}
 }

static struct s3c2410_platform_i2c i9100_i2c0_platdata __initdata = {
	.frequency	= 400000U,
	.sda_delay	= 200,
};

/******************************************************************************
 * Bluetooth
 ******************************************************************************/
struct rfkill_gpio_platform_data i9100_bt_pdata = {
	.reset_gpio	= GPIO_BT_nRST,
	.shutdown_gpio	= GPIO_BT_EN,
	.type		= RFKILL_TYPE_BLUETOOTH,
	.name		= "bcm4330-bt",
};

static struct platform_device i9100_device_bt = {
	.name = "rfkill_gpio",
	.id = RFK_ID_BT,
	.dev = {
		.platform_data	= &i9100_bt_pdata,
	},
};
static void __init i9100_init_bt(void) {
	gpio_request(EXYNOS4_GPA0(0), "BT_RXD");
	gpio_request(EXYNOS4_GPA0(1), "BT_TXD");
	gpio_request(EXYNOS4_GPA0(2), "BT_CTS");
	gpio_request(EXYNOS4_GPA0(3), "BT_RTS");
	s3c_gpio_cfgrange_nopull(EXYNOS4_GPA0(0), 4, S3C_GPIO_SFN(2));
	
	s3c_gpio_cfgpin(GPIO_BT_EN, S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(GPIO_BT_EN, S3C_GPIO_PULL_NONE);
	
	s3c_gpio_cfgpin(GPIO_BT_nRST, S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(GPIO_BT_nRST, S3C_GPIO_PULL_NONE);
}

/******************************************************************************
 * GPS rfkill
 ******************************************************************************/
struct rfkill_gpio_platform_data i9100_gps_pdata = {
	.reset_gpio	= GPIO_GPS_nRST,
	.shutdown_gpio	= GPIO_GPS_PWR_EN,
	.type		= RFKILL_TYPE_GPS,
	.name		= "bcm4751-gps",
};

static struct platform_device i9100_device_gps = {
	.name = "rfkill_gpio",
	.id = RFK_ID_GPS,
	.dev = {
		.platform_data	= &i9100_gps_pdata,
	},
};

static void __init i9100_init_gps(void) {
	gpio_request(EXYNOS4_GPA0(4), "GPS_RXD");
	gpio_request(EXYNOS4_GPA0(5), "GPS_TXD");
	gpio_request(EXYNOS4_GPA0(6), "GPS_CTS");
	gpio_request(EXYNOS4_GPA0(7), "GPS_RTS");
	s3c_gpio_cfgrange_nopull(EXYNOS4_GPA0(4), 4, S3C_GPIO_SFN(2));
	
	s3c_gpio_cfgpin(GPIO_GPS_PWR_EN, S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(GPIO_GPS_PWR_EN, S3C_GPIO_PULL_NONE);
	
	s3c_gpio_cfgpin(GPIO_GPS_nRST, S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(GPIO_GPS_nRST, S3C_GPIO_PULL_NONE);
}

/******************************************************************************
 * KEY LED
 ******************************************************************************/
static struct led_regulator_platform_data i9100_keyled_data = {
	.name = "i9100::keyled",
};

static struct platform_device i9100_keyled = {
	.name = "leds-regulator",
	.id = 0,
	.dev = {
		.platform_data = &i9100_keyled_data,
	}
};

/******************************************************************************
 * proximity and light sensor
 ******************************************************************************/
static struct i2c_gpio_platform_data i2c_gpio_prox_data = {
	.sda_pin	= GPIO_PS_ALS_SDA,
	.scl_pin	= GPIO_PS_ALS_SCL,
};

struct platform_device i2c_gpio_prox = {
	.name = "i2c-gpio",
	.id = I2C_GPIO_BUS_PROX,
	.dev.platform_data = &i2c_gpio_prox_data,
};

static struct i2c_board_info i2c_gpio_prox_devs[] __initdata = {
	{
		I2C_BOARD_INFO("cm3663", 0x20),
	},
};

static void __init i9100_init_proximity_sensor(void)
{
	gpio_request(GPIO_PS_ALS_INT, "P/LS SENSOR INT");
	s5p_register_gpio_interrupt(GPIO_PS_ALS_INT);
	s3c_gpio_cfgpin(GPIO_PS_ALS_INT, S3C_GPIO_SFN(0xf));
	s3c_gpio_setpull(GPIO_PS_ALS_INT, S3C_GPIO_PULL_UP);
	i2c_gpio_prox_devs[0].irq = gpio_to_irq(GPIO_PS_ALS_INT);

	i2c_register_board_info(I2C_GPIO_BUS_PROX,
		i2c_gpio_prox_devs, ARRAY_SIZE(i2c_gpio_prox_devs));
}


/******************************************************************************
 * xmm6260 modem
 ******************************************************************************/
#include <linux/../../drivers/staging/samsung_modem/modemctl/modemctl.h>

static struct gpio i9100_modem_gpios[] = {
	{
		.gpio = GPIO_PHONE_ON,
		.flags = GPIOF_OUT_INIT_LOW,
		.label = "PHONE ON",
	},
	{
		.gpio = GPIO_PHONE_ACTIVE,
		.flags = GPIOF_IN,
		.label = "PHONE ACTIVE"
	},
	{
		.gpio = GPIO_PDA_ACTIVE,
		.flags = GPIOF_OUT_INIT_LOW,
		.label = "PDA ACTIVE",
	},
	{
		.gpio = GPIO_CP_DUMP_INT,
		.flags = GPIOF_IN,
		.label = "CP IRQ",
	},
	{
		.gpio = GPIO_CP_RST,
		.flags = GPIOF_OUT_INIT_LOW,
		.label = "CP RESET",
	},
	{
		.gpio = GPIO_CP_REQ_RESET,
		.flags = GPIOF_OUT_INIT_LOW,
		.label = "CP RESET REQ",
	},
	{
		.gpio = GPIO_IPC_SLAVE_WAKEUP,
		.flags = GPIOF_IN,
		.label = "IPC SLAVE IRQ",
	},
	{
		.gpio = GPIO_IPC_HOST_WAKEUP,
		.flags = GPIOF_IN,
		.label = "IPC HOST IRQ",
	},
	{
		.gpio = GPIO_SUSPEND_REQUEST,
		.flags = GPIOF_IN,
		.label = "MODEM SUSPEND",
	},
	{
		.gpio = GPIO_ISP_INT,
		.flags = GPIOF_IN,
		.label = "ISP IRQ",
	},
	{
		.gpio = GPIO_ACTIVE_STATE,
		.flags = GPIOF_OUT_INIT_LOW,
		.label = "MODEM ACTIVE",
	},
};

static int i9100_modem_ctl_init(struct device *dev) {
	if (gpio_request_array(i9100_modem_gpios, ARRAY_SIZE(i9100_modem_gpios))) {
		//dev_err(dev, "%s: failed to request GPIO\n", __func__);
		return -ENODEV;
	}
	return 0;
}

static int i9100_modem_set_regulator(struct device *dev, bool enable) {
	struct regulator *reg;
	int err = 0;
	reg = regulator_get(NULL, MODEM_VREG);
	//reg = regulator_get(dev, MODEM_VREG);
	if (IS_ERR(reg)) {
		err = PTR_ERR(reg);
		//dev_err(dev, "failed to get modem regulator: %d\n", err);
		pr_err("%s: failed to get modem regulator: %d\n", __func__, err);
		goto fail;
	}

	if (enable) {
		err = regulator_enable(reg);
		//dev_err(dev, "failed to enable regulator: %d\n", err);
		pr_err("%s: failed to enable regulator: %d\n", __func__, err);
	}
	else {
		err = regulator_disable(reg);
		if (err) {
			//dev_err(dev, "failed to disable regulator: %d\n", err);
			pr_err("%s: failed to disable regulator: %d\n", __func__, err);
		}
	}

	regulator_put(reg);
fail:
	return err;
}

static int i9100_modem_suspend(struct device *dev) {
	return i9100_modem_set_regulator(dev, false);
}

static int i9100_modem_resume(struct device *dev) {
	return i9100_modem_set_regulator(dev, true);
}

static enum modem_state i9100_modem_get_state(struct device *dev) {
	enum modem_state ret = 0;
	if (gpio_get_value(GPIO_PHONE_ON)) {
		ret |= MODEM_ON;
	}
	else {
		return ret;
	}
	
	if (gpio_get_value(GPIO_PHONE_ACTIVE)) {
		ret |= MODEM_ACTIVE;
	}
	return ret;
}

static int i9100_modem_set_power(struct device *dev, bool enabled) {
	if (enabled) {
		gpio_set_value(GPIO_PHONE_ON, 0);
		gpio_set_value(GPIO_CP_RST, 0);
		udelay(160);

		gpio_set_value(GPIO_PDA_ACTIVE, 0);
		gpio_set_value(GPIO_ACTIVE_STATE, 0);
		msleep(100);

		gpio_set_value(GPIO_CP_RST, 1);
		udelay(160);
		gpio_set_value(GPIO_CP_REQ_RESET, 1);
		udelay(160);

		gpio_set_value(GPIO_PHONE_ON, 1);
		msleep(20);
		gpio_set_value(GPIO_ACTIVE_STATE, 1);
		gpio_set_value(GPIO_PDA_ACTIVE, 1);
	}
	else {
		gpio_set_value(GPIO_PDA_ACTIVE, 0);
		gpio_set_value(GPIO_ACTIVE_STATE, 0);
		gpio_set_value(GPIO_PHONE_ON, 0);
		gpio_set_value(GPIO_CP_RST, 0);
	}
	return 0;
}

static void i9100_modem_ctl_exit(struct device *dev) {
	i9100_modem_set_power(dev, false);
	gpio_free_array(i9100_modem_gpios, ARRAY_SIZE(i9100_modem_gpios));
}

static int i9100_modem_reset(struct device *dev) {
	gpio_set_value(GPIO_CP_RST, 0);
	gpio_set_value(GPIO_CP_REQ_RESET, 0);
	msleep(100);
	
	gpio_set_value(GPIO_CP_RST, 1);
	udelay(160);
	gpio_set_value(GPIO_CP_REQ_RESET, 1);
	return 0;
}

static struct modemctl_platform_data i9100_modem_data = {
	.init = i9100_modem_ctl_init,
	.exit = i9100_modem_ctl_exit,
	.reset = i9100_modem_reset,
	.set_power = i9100_modem_set_power,
	.get_state = i9100_modem_get_state,
	.suspend = i9100_modem_suspend,
	.resume = i9100_modem_resume,
};

static struct resource i9100_modem_resource[] = {
	{
		.start = IRQ_PHONE_ACTIVE,
		.end = IRQ_PHONE_ACTIVE,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = IRQ_SUSPEND_REQUEST,
		.end = IRQ_SUSPEND_REQUEST,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = IRQ_IPC_HOST_WAKEUP,
		.end = IRQ_IPC_HOST_WAKEUP,
		.flags = IORESOURCE_IRQ,
	}
};

static struct platform_device i9100_modem_device = {
	.name = "modem-control",
	.resource = i9100_modem_resource,
	.num_resources = ARRAY_SIZE(i9100_modem_resource),
	.dev = {
		.platform_data = &i9100_modem_data,
	},
};

static void __init i9100_init_modem(void) {
	gpio_request(GPIO_FLM_RXD, "FLM_RXD");
	gpio_request(GPIO_FLM_TXD, "FLM_TXD");
	s3c_gpio_cfgrange_nopull(EXYNOS4_GPA1(4), 2, S3C_GPIO_SFN(2));

	s3c_gpio_cfgpin(GPIO_PHONE_ON, S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(GPIO_PHONE_ON, S3C_GPIO_PULL_NONE);
	s3c_gpio_cfgpin(GPIO_CP_RST, S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(GPIO_CP_RST, S3C_GPIO_PULL_NONE);
	s3c_gpio_cfgpin(GPIO_PDA_ACTIVE, S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(GPIO_PDA_ACTIVE, S3C_GPIO_PULL_NONE);
	s3c_gpio_cfgpin(GPIO_ACTIVE_STATE, S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(GPIO_ACTIVE_STATE, S3C_GPIO_PULL_NONE);
	s3c_gpio_cfgpin(GPIO_CP_REQ_RESET, S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(GPIO_CP_REQ_RESET, S3C_GPIO_PULL_NONE);
	s3c_gpio_cfgpin(GPIO_IPC_SLAVE_WAKEUP, S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(GPIO_IPC_SLAVE_WAKEUP, S3C_GPIO_PULL_NONE);
	s3c_gpio_cfgpin(GPIO_ACTIVE_STATE, S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(GPIO_ACTIVE_STATE, S3C_GPIO_PULL_NONE);

	s5p_register_gpio_interrupt(GPIO_IPC_HOST_WAKEUP);
	s3c_gpio_cfgpin(GPIO_IPC_HOST_WAKEUP, S3C_GPIO_SFN(0xf));
	s3c_gpio_setpull(GPIO_IPC_HOST_WAKEUP, S3C_GPIO_PULL_NONE);

	s5p_register_gpio_interrupt(GPIO_PHONE_ON);
	s3c_gpio_cfgpin(GPIO_PHONE_ON, S3C_GPIO_SFN(0xf));
	s3c_gpio_setpull(GPIO_PHONE_ON, S3C_GPIO_PULL_NONE);

	s5p_register_gpio_interrupt(GPIO_CP_DUMP_INT);
	s3c_gpio_cfgpin(GPIO_CP_DUMP_INT, S3C_GPIO_SFN(0xf));
	s3c_gpio_setpull(GPIO_CP_DUMP_INT, S3C_GPIO_PULL_DOWN);

	//FIXME: enabling statically until modemctl is reimplemented
	i9100_modem_ctl_init(NULL);
	i9100_modem_set_regulator(NULL, true);
	i9100_modem_set_power(NULL, true);
}
/******************************************************************************
 * Sound
 ******************************************************************************/
static struct i2c_board_info i2c6_devs[] __initdata = {
	{
		I2C_BOARD_INFO("ymu823", 0x3a),
	},
};

/******************************************************************************
 * DEVFREQ controlling memory/bus
 ******************************************************************************/
static struct platform_device exynos4_bus_devfreq = {
	.name			= "exynos4210-busfreq",
};

/******************************************************************************
 * platform devices
 ******************************************************************************/
static struct platform_device *i9100_devices[] __initdata = {
	&exynos4_device_sysmmu,
	&s3c_device_i2c5,
	&s3c_device_i2c0,
	&emmc_fixed_voltage,
	&wlan_fixed_voltage,
	&s3c_device_i2c1,
	&s3c_device_i2c3,
	&s3c_device_i2c6,
	&s3c_device_rtc,
	&s3c_device_hsmmc0,
	&s3c_device_hsmmc2,
	&s3c_device_hsmmc3,
	&s3c_device_timer[1],
	&s3c_device_usbgadget,
	&s3c_device_wdt,
	&s5p_device_ehci,
	&s5p_device_fimc0,
	&s5p_device_fimc1,
	&s5p_device_fimc2,
	&s5p_device_fimc3,
	&s5p_device_fimc_md,
	&s5p_device_fimd0,
	&s5p_device_mipi_csis0,
	&s5p_device_g2d,
	&s5p_device_mfc,
	&s5p_device_mfc_l,
	&s5p_device_mfc_r,
	&samsung_asoc_dma,
	&exynos4_bus_devfreq,
	&exynos4_device_i2s0,
	&exynos4_device_pd[PD_MFC],
	&exynos4_device_pd[PD_G3D],
	&exynos4_device_pd[PD_LCD0],
	&exynos4_device_pd[PD_LCD1],
	&exynos4_device_pd[PD_CAM],
	&exynos4_device_pd[PD_GPS],
	&exynos4_device_pd[PD_TV],
	&exynos4_device_tmu,
	
	&lcd_spi_gpio,
	&i2c_gpio_touchkey,
	&i2c_gpio_usb,
	&i2c_gpio_prox,
	&i2c_gpio_gauge,
	&i2c_gpio_fm,

	&cam_vdda_fixed_rdev,
	&cam_8m_12v_fixed_rdev,
	&cam_8m_28v_fixed_rdev,
	
	&i9100_device_gpio_keys,
	&i9100_device_gps,
	&i9100_device_bt,

	&i9100_keyled,
	&i9100_modem_device,
};


/******************************************************************************
 * machine initialization
 ******************************************************************************/
static void __init i9100_map_io(void) {
	exynos_init_io(NULL, 0);
	s3c24xx_init_clocks(24000000);
	s3c24xx_init_uarts(i9100_uartcfgs, ARRAY_SIZE(i9100_uartcfgs));
}

static void __init i9100_reserve(void)
{
	s5p_mfc_reserve_mem(0x43000000, 8 << 20, 0x51000000, 8 << 20);
}

static void __init i9100_get_revision(void) {
	unsigned revision = 0;
	int i;
	int gpios_rev[] = {
		GPIO_HW_REV0,
		GPIO_HW_REV1,
		GPIO_HW_REV2,
		GPIO_HW_REV3,
	};
	for (i = 0; i < ARRAY_SIZE(gpios_rev); i++) {
		gpio_request(gpios_rev[i], "HW_REV");
		revision |= ((gpio_get_value(gpios_rev[i])) << i);
		gpio_free(gpios_rev[i]);
	}
	i9100_hw_revision = revision;
	printk(KERN_INFO "I9100 board revision %d\n", revision);
}

static int __init i9100_late_init(void) {
	i9100_set_usb_mipi(1);
	i9100_set_usb_path(0);
	i9100_init_modem();
	return 0;
}
device_initcall(i9100_late_init);

static void __init i9100_machine_init(void) {
	i9100_get_revision();
	i9100_config_gpio_table();

	s3c_gpio_cfgpin(GPIO_WLAN_EN, S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(GPIO_WLAN_EN, S3C_GPIO_PULL_NONE);
	
	s3c_sdhci0_set_platdata(&i9100_hsmmc0_pdata);
	s3c_sdhci2_set_platdata(&i9100_hsmmc2_pdata);
	s3c_sdhci3_set_platdata(&i9100_hsmmc3_pdata);
	
	s3c_i2c0_set_platdata(&i9100_i2c0_platdata);
	
	i9100_init_k3_sensors();
	s3c_i2c1_set_platdata(NULL);
	i2c_register_board_info(1, i2c1_devs, ARRAY_SIZE(i2c1_devs));
	
	i9100_init_tsp();
	s3c_i2c3_set_platdata(&i2c3_data);
	i2c_register_board_info(3, i2c3_devs, ARRAY_SIZE(i2c3_devs));

	i9100_init_pmic();
	s3c_i2c5_set_platdata(NULL);
	i2c_register_board_info(5, i2c5_devs, ARRAY_SIZE(i2c5_devs));

	s3c_i2c6_set_platdata(NULL);
	i2c_register_board_info(6, i2c6_devs, ARRAY_SIZE(i2c6_devs));
	
	i9100_init_fb();
	i9100_init_touchkey();
	i9100_init_usb();
	i9100_init_usb_switch();
	clk_xusbxti.rate = 24000000;
	i9100_init_battery_gauge();
	i9100_init_fm();
	i9100_init_camera();
	i9100_init_bt();
	i9100_init_gps();
	i9100_init_proximity_sensor();
	
	platform_add_devices(i9100_devices, ARRAY_SIZE(i9100_devices));

	s5p_device_mfc.dev.parent = &exynos4_device_pd[PD_MFC].dev;
	s5p_device_fimd0.dev.parent = &exynos4_device_pd[PD_LCD0].dev;

	s5p_device_fimc0.dev.parent = &exynos4_device_pd[PD_CAM].dev;
	s5p_device_fimc1.dev.parent = &exynos4_device_pd[PD_CAM].dev;
	s5p_device_fimc2.dev.parent = &exynos4_device_pd[PD_CAM].dev;
	s5p_device_fimc3.dev.parent = &exynos4_device_pd[PD_CAM].dev;
	s5p_device_mipi_csis0.dev.parent = &exynos4_device_pd[PD_CAM].dev;
}

MACHINE_START(SGS_I9100, "i9100")
	.atag_offset	= 0x100,
	.init_irq	= exynos4_init_irq,
	.map_io		= i9100_map_io,
	.handle_irq	= gic_handle_irq,
	.init_machine	= i9100_machine_init,
	.timer		= &exynos4_timer,
	.reserve        = &i9100_reserve,
	.restart	= exynos4_restart,
MACHINE_END
