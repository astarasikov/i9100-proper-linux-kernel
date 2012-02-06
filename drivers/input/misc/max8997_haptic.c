/*
 *  max8997_haptic.c - MAX8997-haptic controller driver
 *
 *  Copyright (C) 2012 Samsung Electronics
 *  Donggeun Kim <dg77.kim@samsung.com>
 *
 * This program is not provided / owned by Maxim Integrated Products.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/pwm.h>
#include <linux/input.h>
#include <linux/mfd/max8997-private.h>
#include <linux/mfd/max8997.h>
#include <linux/regulator/consumer.h>

/* Haptic configuration 2 register */
#define MAX8997_MOTOR_TYPE_SHIFT	7
#define MAX8997_ENABLE_SHIFT		6
#define MAX8997_MODE_SHIFT		5

/* Haptic driver configuration register */
#define MAX8997_CYCLE_SHIFT		6
#define MAX8997_SIG_PERIOD_SHIFT	4
#define MAX8997_SIG_DUTY_SHIFT		2
#define MAX8997_PWM_DUTY_SHIFT		0

struct max8997_haptic {
	struct device *dev;
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct regulator *regulator;

	bool enabled;

	int level;

	struct pwm_device *pwm;
	int pwm_period;
	enum max8997_haptic_pwm_divisor pwm_divisor;

	enum max8997_haptic_motor_type type;
	enum max8997_haptic_pulse_mode mode;

	int internal_mode_pattern;
	int pattern_cycle;
	int pattern_signal_period;
};

static int max8997_haptic_set_duty_cycle(struct max8997_haptic *chip)
{
	int duty, i;
	int ret;
	u8 duty_index;

	if (chip->mode == MAX8997_EXTERNAL_MODE) {
		duty = chip->pwm_period * chip->level / 100;
		ret = pwm_config(chip->pwm, duty, chip->pwm_period);
	} else {
		for (i = 0; i <= 64; i++) {
			if (chip->level <= i * 100 / 64) {
				duty_index = i;
				break;
			}
		}
		switch (chip->internal_mode_pattern) {
		case 0:
			max8997_write_reg(chip->client,
				MAX8997_HAPTIC_REG_SIGPWMDC1, duty_index);
			break;
		case 1:
			max8997_write_reg(chip->client,
				MAX8997_HAPTIC_REG_SIGPWMDC2, duty_index);
			break;
		case 2:
			max8997_write_reg(chip->client,
				MAX8997_HAPTIC_REG_SIGPWMDC3, duty_index);
			break;
		case 3:
			max8997_write_reg(chip->client,
				MAX8997_HAPTIC_REG_SIGPWMDC4, duty_index);
			break;
		default:
			break;
		}
	}
	return ret;
}

static void max8997_haptic_configure(struct max8997_haptic *chip)
{
	u8 value;

	value = chip->type << MAX8997_MOTOR_TYPE_SHIFT |
		chip->enabled << MAX8997_ENABLE_SHIFT |
		chip->mode << MAX8997_MODE_SHIFT | chip->pwm_divisor;
	max8997_write_reg(chip->client, MAX8997_HAPTIC_REG_CONF2, value);

	if (chip->mode == MAX8997_INTERNAL_MODE && chip->enabled) {
		value = chip->internal_mode_pattern << MAX8997_CYCLE_SHIFT |
		      chip->internal_mode_pattern << MAX8997_SIG_PERIOD_SHIFT |
		      chip->internal_mode_pattern << MAX8997_SIG_DUTY_SHIFT |
		      chip->internal_mode_pattern << MAX8997_PWM_DUTY_SHIFT;
		max8997_write_reg(chip->client,
			MAX8997_HAPTIC_REG_DRVCONF, value);

		switch (chip->internal_mode_pattern) {
		case 0:
			value = chip->pattern_cycle << 4;
			max8997_write_reg(chip->client,
				MAX8997_HAPTIC_REG_CYCLECONF1, value);
			value = chip->pattern_signal_period;
			max8997_write_reg(chip->client,
				MAX8997_HAPTIC_REG_SIGCONF1, value);
			break;
		case 1:
			value = chip->pattern_cycle;
			max8997_write_reg(chip->client,
				MAX8997_HAPTIC_REG_CYCLECONF1, value);
			value = chip->pattern_signal_period;
			max8997_write_reg(chip->client,
				MAX8997_HAPTIC_REG_SIGCONF2, value);
			break;
		case 2:
			value = chip->pattern_cycle << 4;
			max8997_write_reg(chip->client,
				MAX8997_HAPTIC_REG_CYCLECONF2, value);
			value = chip->pattern_signal_period;
			max8997_write_reg(chip->client,
				MAX8997_HAPTIC_REG_SIGCONF3, value);
			break;
		case 3:
			value = chip->pattern_cycle;
			max8997_write_reg(chip->client,
				MAX8997_HAPTIC_REG_CYCLECONF2, value);
			value = chip->pattern_signal_period;
			max8997_write_reg(chip->client,
				MAX8997_HAPTIC_REG_SIGCONF4, value);
			break;
		default:
			break;
		}
	}
}

static void max8997_haptic_enable(struct max8997_haptic *chip, bool enable)
{
	if (chip->enabled == enable)
		return;

	chip->enabled = enable;

	if (enable) {
		regulator_enable(chip->regulator);
		max8997_haptic_configure(chip);
		if (chip->mode == MAX8997_EXTERNAL_MODE)
			pwm_enable(chip->pwm);
	} else {
		max8997_haptic_configure(chip);
		if (chip->mode == MAX8997_EXTERNAL_MODE)
			pwm_disable(chip->pwm);
		regulator_disable(chip->regulator);
	}
}

static void max8997_haptic_close(struct input_dev *dev)
{
	struct max8997_haptic *chip = input_get_drvdata(dev);

	if (chip->enabled)
		max8997_haptic_enable(chip, false);
}

static int max8997_haptic_play_effect(struct input_dev *dev, void *data,
				  struct ff_effect *effect)
{
	struct max8997_haptic *chip = input_get_drvdata(dev);
	int ret;

	chip->level = effect->u.rumble.strong_magnitude;
	if (!chip->level)
		chip->level = effect->u.rumble.weak_magnitude;

	if (chip->level) {
		ret = max8997_haptic_set_duty_cycle(chip);
		if (ret) {
			dev_err(chip->dev, "set_pwm_cycle failed\n");
			return ret;
		}
		max8997_haptic_enable(chip, true);
	} else {
		max8997_haptic_enable(chip, false);
	}

	return 0;
}

static int __devinit max8997_haptic_probe(struct platform_device *pdev)
{
	struct max8997_dev *iodev = dev_get_drvdata(pdev->dev.parent);
	struct max8997_platform_data *pdata = dev_get_platdata(iodev->dev);
	struct max8997_haptic_platform_data *haptic_pdata =
						pdata->haptic_pdata;
	struct max8997_haptic *chip;
	struct input_dev *input_dev;
	int ret;

	if (!haptic_pdata) {
		dev_err(&pdev->dev, "no haptic platform data\n");
		return -EINVAL;
	}

	chip = kzalloc(sizeof(struct max8997_haptic), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!chip || !input_dev) {
		dev_err(&pdev->dev, "unable to allocate memory\n");
		return -ENOMEM;
	}

	chip->client = iodev->haptic;
	chip->dev = &pdev->dev;
	chip->input_dev = input_dev;
	chip->pwm_period = haptic_pdata->pwm_period;
	chip->type = haptic_pdata->type;
	chip->mode = haptic_pdata->mode;
	chip->pwm_divisor = haptic_pdata->pwm_divisor;
	if (chip->mode == MAX8997_INTERNAL_MODE) {
		chip->internal_mode_pattern =
				haptic_pdata->internal_mode_pattern;
		chip->pattern_cycle = haptic_pdata->pattern_cycle;
		chip->pattern_signal_period =
				haptic_pdata->pattern_signal_period;
	}

	if (chip->mode == MAX8997_EXTERNAL_MODE) {
		chip->pwm = pwm_request(haptic_pdata->pwm_channel_id,
					"max8997-haptic");
		if (IS_ERR(chip->pwm)) {
			dev_err(&pdev->dev,
				"unable to request PWM for haptic\n");
			ret = PTR_RET(chip->pwm);
			goto err_pwm;
		}
	}

	chip->regulator = regulator_get(&pdev->dev, "inmotor");
	if (IS_ERR(chip->regulator)) {
		dev_err(&pdev->dev, "unable to get regulator\n");
		ret = PTR_RET(chip->regulator);
		goto err_regulator;
	}

	platform_set_drvdata(pdev, chip);

	input_dev->name = "max8997-haptic";
	input_dev->id.version = 1;
	input_dev->dev.parent = &pdev->dev;
	input_dev->close = max8997_haptic_close;
	input_set_drvdata(input_dev, chip);
	input_set_capability(input_dev, EV_FF, FF_RUMBLE);

	ret = input_ff_create_memless(input_dev, NULL,
				max8997_haptic_play_effect);
	if (ret) {
		dev_err(&pdev->dev,
			"unable to create FF device(ret : %d)\n", ret);
		goto err_ff_memless;
	}

	ret = input_register_device(input_dev);
	if (ret) {
		dev_err(&pdev->dev,
			"unable to register input device(ret : %d)\n", ret);
		goto err_input_register;
	}

	return 0;

err_input_register:
	input_ff_destroy(input_dev);
err_ff_memless:
	regulator_put(chip->regulator);
err_regulator:
	if (chip->mode == MAX8997_EXTERNAL_MODE)
		pwm_free(chip->pwm);
err_pwm:
	kfree(chip);

	return ret;
}

static int __devexit max8997_haptic_remove(struct platform_device *pdev)
{
	struct max8997_haptic *chip = platform_get_drvdata(pdev);

	input_unregister_device(chip->input_dev);
	regulator_put(chip->regulator);

	if (chip->mode == MAX8997_EXTERNAL_MODE)
		pwm_free(chip->pwm);

	kfree(chip);

	return 0;
}

static int max8997_haptic_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct max8997_haptic *chip = platform_get_drvdata(pdev);

	max8997_haptic_enable(chip, false);

	return 0;
}

static int max8997_haptic_resume(struct device *dev)
{
	return 0;
}

static SIMPLE_DEV_PM_OPS(max8997_haptic_pm_ops, max8997_haptic_suspend,
	max8997_haptic_resume);

static const struct platform_device_id max8997_haptic_id[] = {
	{ "max8997-haptic", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, max8997_haptic_id);

static struct platform_driver max8997_haptic_driver = {
	.driver	= {
		.name	= "max8997-haptic",
		.owner	= THIS_MODULE,
		.pm	= &max8997_haptic_pm_ops,
	},
	.probe		= max8997_haptic_probe,
	.remove		= __devexit_p(max8997_haptic_remove),
	.id_table	= max8997_haptic_id,
};

module_platform_driver(max8997_haptic_driver);

MODULE_ALIAS("platform:max8997-haptic");
MODULE_AUTHOR("Donggeun Kim <dg77.kim@samsung.com>");
MODULE_DESCRIPTION("max8997_haptic driver");
MODULE_LICENSE("GPL");
