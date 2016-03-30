/* drivers\sharp\shflip/shflip.c  (Flip Driver)
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
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/switch.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <sharp/shtrpd_dev.h>
#include <linux/wakelock.h>

#define SHFLIP_LOG_TAG "SHFLIPSWITCH"

static struct wake_lock shflip_wakelock;

int shflip_err_log  = 1;
int shflip_dbg_log  = 0;

#if defined (CONFIG_ANDROID_ENGINEERING)
module_param(shflip_err_log,  int, 0600);
module_param(shflip_dbg_log,  int, 0600);
#endif /* CONFIG_ANDROID_ENGINEERING */


#define SHFLIP_DEBUG_LOG(fmt, args...)\
		if(shflip_dbg_log == 1) { \
			printk(KERN_INFO "[%s][%s(%d)] " fmt"\n", SHFLIP_LOG_TAG, __func__, __LINE__, ## args);\
		}

#define SHFLIP_ERR_LOG(fmt, args...)\
		if(shflip_err_log == 1) { \
			printk(KERN_ERR "[%s][%s(%d)] " fmt"\n", SHFLIP_LOG_TAG, __func__, __LINE__, ## args);\
		}

#define SH_FLIP_IRQ 21
#define GPIO_OPEN_XSHUT 21

enum {
	FLIP_CLOSE	= 0,
	FLIP_OPEN,
};

struct flip_data {
	struct input_dev *input;
	unsigned int irq;
	struct switch_dev sdev_flip;

};

int msm_flip_get_state(void)
{
	return gpio_get_value(GPIO_OPEN_XSHUT);
}

static ssize_t sh_flip_print_name(struct switch_dev *sdev, char *buf)
{
	SHFLIP_DEBUG_LOG("sdev->name = %s",sdev->name);

	switch (switch_get_state(sdev)) {
	case FLIP_OPEN:
		return sprintf(buf, "OPEN\n");
	case FLIP_CLOSE:
		return sprintf(buf, "CLOSE\n");
	}
	return -EINVAL;
}


static struct of_device_id flip_switch_of_match[] = {
	{ .compatible = "flip_switch", },
	{ },
};
MODULE_DEVICE_TABLE(of, flip_switch_of_match);

static irqreturn_t sh_flip_irq_isr(int irq, void *dev_id)
{
	wake_lock(&shflip_wakelock);
	return IRQ_WAKE_THREAD;
}

irqreturn_t sh_flip_irq_thread(int irq, void *dev_id)
{
	struct flip_data *fd = dev_id;
	struct input_dev *input = fd->input;
	int value;

	/* Get the value of the GPIO pin */
	value = gpio_get_value(GPIO_OPEN_XSHUT);

	/* Decision of opening and closing state */
	if(value == FLIP_OPEN) {
		printk("flip: open\n");
		input_report_switch(input, SW_LID, 0);
		switch_set_state(&fd->sdev_flip, value);
		msm_trpd_set_flip_state(1);
	}
	else if(value == FLIP_CLOSE) {
		printk("flip: close\n");
		input_report_switch(input, SW_LID, 1);
		switch_set_state(&fd->sdev_flip, value);
		msm_trpd_set_flip_state(0);
	}

	input_sync(input);

	wake_unlock(&shflip_wakelock);

	return IRQ_HANDLED;
}


static int shflip_probe(struct platform_device *pdev)
{
	struct flip_data *fd;
	struct input_dev *input;
	int error = 0;
	int wakeup = 1;
	unsigned long irqflags;
	int value;

	/* memory allocate */
	fd = kzalloc(sizeof(struct flip_data), GFP_KERNEL);
	if (!fd) {
		SHFLIP_ERR_LOG("failed to allocate state");
		error = -ENOMEM;
		goto fail0;
	}

	platform_set_drvdata(pdev, fd);
	
	wake_lock_init(&shflip_wakelock, WAKE_LOCK_SUSPEND, "shflip_wake_lock");

	/* flip data setting */
	fd->sdev_flip.name	= "flip";
	fd->sdev_flip.print_name = sh_flip_print_name;
	
	error = switch_dev_register(&fd->sdev_flip);
	if (error) {
		goto fail1;
	}

	/* Get the value of the GPIO pin */
	value = gpio_get_value(GPIO_OPEN_XSHUT);
	switch_set_state(&fd->sdev_flip, value);
	SHFLIP_DEBUG_LOG("Switching condition value = %d",value);

	/* driver's data entry */
	input = input_allocate_device();
	if (!input) {
		SHFLIP_ERR_LOG("failed to input allocate device state");
		error = -ENOMEM;
		goto fail2;
	}
	input_set_drvdata(input, fd);

	fd->input = input;
	input->name	= pdev->name;
	input->id.vendor	= 0x0001;
	input->id.product	= 1;
	input->id.version	= 1;

	if(value == FLIP_CLOSE) {
		__change_bit(SW_LID, input->sw);
	}

	fd->irq = gpio_to_irq(SH_FLIP_IRQ);
	irqflags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING;

	error = request_threaded_irq(fd->irq, sh_flip_irq_isr, sh_flip_irq_thread, irqflags, pdev->name, fd);
	if (error) {
		SHFLIP_ERR_LOG("Unable to request_irq error = %d", error);
		goto fail2;
	}

	input_set_capability(input, EV_SW, SW_LID);

	error = input_register_device(input);
	if (error) {
		SHFLIP_ERR_LOG("input_register_device error=%d", error);
		goto fail3;
	}

	device_init_wakeup(&pdev->dev, wakeup);

	return error;
fail3:
	free_irq(fd->irq, pdev);
	input_free_device(input);
fail2:
	switch_dev_unregister(&fd->sdev_flip);
fail1:
	kfree(fd);
fail0:
	return error;

}

static int shflip_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	SHFLIP_DEBUG_LOG("flip suspend");
	disable_irq_nosync(gpio_to_irq(SH_FLIP_IRQ));
	enable_irq_wake(gpio_to_irq(SH_FLIP_IRQ));
	return 0;
}

static int shflip_resume(struct platform_device *pdev)
{
	SHFLIP_DEBUG_LOG("flip resume");
	enable_irq(gpio_to_irq(SH_FLIP_IRQ));
	disable_irq_wake(gpio_to_irq(SH_FLIP_IRQ));
	return 0;
}

static int __devexit shflip_remove(struct platform_device *pdev)
{
	struct flip_data *fd = platform_get_drvdata(pdev);
	struct input_dev *input = fd->input;

    wake_lock_destroy(&shflip_wakelock);

	free_irq(fd->irq, pdev);
	input_free_device(input);
	switch_dev_unregister(&fd->sdev_flip);
	kfree(fd);

	return 0;
}

static struct platform_driver shflip_driver = {
	.probe		= shflip_probe,
	.remove		= __devexit_p(shflip_remove),
    .suspend    = shflip_suspend,
    .resume     = shflip_resume,
	.driver		= {
		.name	= "shflip",
		.owner	= THIS_MODULE,
		.of_match_table = flip_switch_of_match,
	},
};

static int __init shflip_init(void)
{
	return platform_driver_register(&shflip_driver);
}

static void __exit shflip_exit(void)
{
	platform_driver_unregister(&shflip_driver);
}

module_exit(shflip_exit);
module_init(shflip_init);

MODULE_DESCRIPTION("SHARP FLIP DRIVER MODULE");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.0");
