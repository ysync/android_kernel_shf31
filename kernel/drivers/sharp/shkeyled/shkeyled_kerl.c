 /* drivers/sharp/shkeyled/shkeyled_kerl.c  (Key LED Driver)
 *
 * Copyright (C) 2011 SHARP CORPORATION
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/gpio.h>




#define SHKEYLED_LOG_TAG "SHKEYLEDkerl"

int shkeyled_err_log  = 1;
int shkeyled_dbg_log  = 0;

#if defined (CONFIG_ANDROID_ENGINEERING)
module_param(shkeyled_err_log,  int, 0600);
module_param(shkeyled_dbg_log,  int, 0600);
#endif /* CONFIG_ANDROID_ENGINEERING */


#define SHKEYLED_DEBUG_LOG(fmt, args...)\
		if(shkeyled_dbg_log == 1) { \
			printk(KERN_INFO "[%s][%s(%d)] " fmt"\n", SHKEYLED_LOG_TAG, __func__, __LINE__, ## args);\
		}


#define SHKEYLED_ERR_LOG(fmt, args...)\
		if(shkeyled_err_log == 1) { \
			printk(KERN_ERR "[%s][%s(%d)] " fmt"\n", SHKEYLED_LOG_TAG, __func__, __LINE__, ## args);\
		}

#define GPIO_KEY_BL_EN 28
#define KEY_BL_ON 1
#define KEY_BL_OFF 0

/* LED trigger */
//DEFINE_LED_TRIGGER(buttonbacklight_trigger);

static void shkeyled_set(struct led_classdev *led_cdev, enum led_brightness value)
{

	SHKEYLED_DEBUG_LOG("value = %d", value);
	if(value == LED_OFF)
	{
		gpio_set_value(GPIO_KEY_BL_EN, KEY_BL_OFF);
	}
	else if(value == LED_FULL)
	{
		gpio_set_value(GPIO_KEY_BL_EN, KEY_BL_ON);
	}

}

static struct led_classdev shkeyled_dev =
{
	.name			= "keyboard-backlight",
	.brightness_set	= shkeyled_set,
	.brightness		= LED_OFF,
};

static struct of_device_id key_backlight_of_match[] = {
	{ .compatible = "keyboard_backlight", },
	{ },
};
MODULE_DEVICE_TABLE(of, key_backlight_of_match);

static int shkeyled_probe(struct platform_device *pdev)
{
	int error = 0;
	int pin = 0;
	
	pin = gpio_get_value(GPIO_KEY_BL_EN);
	SHKEYLED_DEBUG_LOG("shkeyled_dev.name = %s : pin = %d", shkeyled_dev.name,pin);

	error = led_classdev_register(&pdev->dev, &shkeyled_dev);
	if (error)
	{
		SHKEYLED_ERR_LOG("led_classdev_register Error");
		return error;
	}
	
	shkeyled_set(&shkeyled_dev, LED_OFF);

	return error;
}

static int __devexit shkeyled_remove(struct platform_device *pdev)
{
	int pin = 0;
	
	pin = gpio_get_value(GPIO_KEY_BL_EN);
	shkeyled_set(&shkeyled_dev, LED_OFF);
	led_classdev_unregister(&shkeyled_dev);
	
	return 0;
}

static struct platform_driver shkeyled_driver = {
	.probe		= shkeyled_probe,
	.remove		= __devexit_p(shkeyled_remove),
	.driver		= {
		.name	= "shkeyled",
		.owner	= THIS_MODULE,
		.of_match_table = key_backlight_of_match,
	},
};

static int __init shkeyled_init(void)
{
	return platform_driver_register(&shkeyled_driver);
}

static void __exit shkeyled_exit(void)
{
	platform_driver_unregister(&shkeyled_driver);
}

module_exit(shkeyled_exit);
module_init(shkeyled_init);

MODULE_DESCRIPTION("SHARP KEYLED DRIVER MODULE");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.0");
