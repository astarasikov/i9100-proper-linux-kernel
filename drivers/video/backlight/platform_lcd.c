/* drivers/video/backlight/platform_lcd.c
 *
 * Copyright 2008 Simtec Electronics
 *	Ben Dooks <ben@simtec.co.uk>
 *
 * Generic platform-device LCD power control interface.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
*/

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/lcd.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>

#include <video/platform_lcd.h>

struct platform_lcd {
	struct device		*us;
	struct lcd_device	*lcd;
	struct plat_lcd_data	*pdata;

	unsigned int		 power;
	unsigned int		 suspended : 1;
};

static inline struct platform_lcd *to_our_lcd(struct lcd_device *lcd)
{
	return lcd_get_data(lcd);
}

static int platform_lcd_get_power(struct lcd_device *lcd)
{
	struct platform_lcd *plcd = to_our_lcd(lcd);

	return plcd->power;
}

static int platform_lcd_set_power(struct lcd_device *lcd, int power)
{
	struct platform_lcd *plcd = to_our_lcd(lcd);
	struct regulator *lcd_regulator;
	int lcd_power = 1;

	if (power == FB_BLANK_POWERDOWN || plcd->suspended)
		lcd_power = 0;

	/*
	 * If power to lcd and/or lcd interface is controlled using a regulator,
	 * enable or disable the regulator based in the power setting.
	 */
	lcd_regulator = regulator_get(plcd->us, "vcc_lcd");
	if (IS_ERR(lcd_regulator)) {
		dev_info(plcd->us, "could not get regulator\n");
		goto set_power;
	}

	if (lcd_power) {
		if (plcd->pdata->min_uV || plcd->pdata->max_uV)
			if (regulator_set_voltage(lcd_regulator,
				plcd->pdata->min_uV, plcd->pdata->max_uV))
				dev_info(plcd->us,
					"regulator voltage set failed\n");

		if (regulator_enable(lcd_regulator))
			dev_info(plcd->us, "failed to enable regulator\n");
	} else {
		regulator_disable(lcd_regulator);
	}

	regulator_put(lcd_regulator);

set_power:
	plcd->pdata->set_power(plcd->pdata, lcd_power);
	plcd->power = power;

	return 0;
}

static int platform_lcd_match(struct lcd_device *lcd, struct fb_info *info)
{
	struct platform_lcd *plcd = to_our_lcd(lcd);
	struct plat_lcd_data *pdata = plcd->pdata;

	if (pdata->match_fb)
		return pdata->match_fb(pdata, info);

	return plcd->us->parent == info->device;
}

static struct lcd_ops platform_lcd_ops = {
	.get_power	= platform_lcd_get_power,
	.set_power	= platform_lcd_set_power,
	.check_fb	= platform_lcd_match,
};

static int __devinit platform_lcd_probe(struct platform_device *pdev)
{
	struct plat_lcd_data *pdata;
	struct platform_lcd *plcd;
	struct device *dev = &pdev->dev;
	int err;

	pdata = pdev->dev.platform_data;
	if (!pdata) {
		dev_err(dev, "no platform data supplied\n");
		return -EINVAL;
	}

	plcd = devm_kzalloc(&pdev->dev, sizeof(struct platform_lcd),
			    GFP_KERNEL);
	if (!plcd) {
		dev_err(dev, "no memory for state\n");
		return -ENOMEM;
	}

	plcd->us = dev;
	plcd->pdata = pdata;
	plcd->lcd = lcd_device_register(dev_name(dev), dev,
					plcd, &platform_lcd_ops);
	if (IS_ERR(plcd->lcd)) {
		dev_err(dev, "cannot register lcd device\n");
		err = PTR_ERR(plcd->lcd);
		goto err;
	}

	platform_set_drvdata(pdev, plcd);
	platform_lcd_set_power(plcd->lcd, FB_BLANK_NORMAL);

	return 0;

 err:
	return err;
}

static int __devexit platform_lcd_remove(struct platform_device *pdev)
{
	struct platform_lcd *plcd = platform_get_drvdata(pdev);

	lcd_device_unregister(plcd->lcd);

	return 0;
}

#ifdef CONFIG_PM
static int platform_lcd_suspend(struct platform_device *pdev, pm_message_t st)
{
	struct platform_lcd *plcd = platform_get_drvdata(pdev);

	plcd->suspended = 1;
	platform_lcd_set_power(plcd->lcd, plcd->power);

	return 0;
}

static int platform_lcd_resume(struct platform_device *pdev)
{
	struct platform_lcd *plcd = platform_get_drvdata(pdev);

	plcd->suspended = 0;
	platform_lcd_set_power(plcd->lcd, plcd->power);

	return 0;
}
#else
#define platform_lcd_suspend NULL
#define platform_lcd_resume NULL
#endif

static struct platform_driver platform_lcd_driver = {
	.driver		= {
		.name	= "platform-lcd",
		.owner	= THIS_MODULE,
	},
	.probe		= platform_lcd_probe,
	.remove		= __devexit_p(platform_lcd_remove),
	.suspend        = platform_lcd_suspend,
	.resume         = platform_lcd_resume,
};

module_platform_driver(platform_lcd_driver);

MODULE_AUTHOR("Ben Dooks <ben-linux@fluff.org>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:platform-lcd");
