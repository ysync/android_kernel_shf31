/* kernel/drivers/sharp/shswic.c  (SwitchingIC Driver)
 *
 * Copyright (C) 2012 SHARP CORPORATION
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

/* ____________________________________________________________________________________________________________________________ */
/**
 *	Include
 */
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/input.h>
#include <linux/err.h>
#include <asm/atomic.h>
#include <asm/uaccess.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/jiffies.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <sharp/sh_boot_manager.h>
#include <linux/wakelock.h>
#include <sharp/shswic_kerl.h>
#ifdef CONFIG_SII8334_MHL_TX
#include <sharp/shmhl_kerl.h>
#endif /* CONFIG_SII8334_MHL_TX */
#ifdef CONFIG_HEADSET_BUTTON_SWIC
#include <sound/jack.h>
#endif /* CONFIG_HEADSET_BUTTON_SWIC */

#include "shswic_kerl_local.h"
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/mfd/pm8xxx/pm8921.h>

/* ____________________________________________________________________________________________________________________________ */
/**
 *	Internal Define
 */
/* Driver Name */
#define SHSWIC_DRIVER_NAME 				"shswic"
#define SHSWIC_DIAG_DRIVER_NAME 		"shswic_diag"
#define SHSWIC_I2C_DRIVER_NAME 			"shswic_i2c"
#define SHSWIC_REG_DRIVER_NAME			"msm_shswic"

#ifdef CONFIG_HEADSET_BUTTON_SWIC
#define SH_SWIC_KEY_DEVICE_NAME	"shswic_key"
#endif /* CONFIG_HEADSET_BUTTON_SWIC */

#define SHSWIC_SIGNAL_PKT_COUNTS		10
#define SHSWIC_TIMER_SIGNAL_PKT_COUNTS	10

#define SHSWIC_MHL_PASS				(0x49)	/* (0x00)VBUS  :Open			*/
													/* (0x40)CBUS  :connect			*/
													/* (0x08)HDPR  :HDP2 connect	*/
													/* (0x01)HDML  :HDM2 connect	*/

/* INT_STAT */
#define SHSWIC_INT_STA_USBPORT_MASK	(0xF4)
#define SHSWIC_INT_STA_VBUS_MASK		(0x10)
#ifdef CONFIG_SII8334_MHL_TX
#define SHSWIC_INT_STA_CHGDET_MASK	(0x80)
#endif /* CONFIG_SII8334_MHL_TX */

/* PWR_STAT */
#define SHSWIC_STATUS_CHGPORT_MASK	(0x60)
#ifdef CONFIG_SII8334_MHL_TX
#define SHSWIC_STATUS_PUPDET_MASK		(0x10)
#endif /*CONFIG_SII8334_MHL_TX */
#define SHSWIC_STATUS_DCDFAIL_MASK	(0x80)

/* ID_STAT */
#define SHSWIC_ID_STA_IDRDET_MASK		(0x10)
#define SHSWIC_ID_STA_INDO_MASK		(0x0F)
#define SHSWIC_ID_STA_MASK			(0x7F)
#ifdef CONFIG_HOST_SWIC
#define SHSWIC_ID_STA_HOST_MASK		(0x1F)
#endif /* CONFIG_HOST_SWIC */

#define SHSWIC_SUSPEND_WAKE_LOCK_TIMEOUT	1*HZ/2

enum
{
	SHSWIC_RECHECK_NON,
	SHSWIC_RECHECK_SECOND,
	SHSWIC_RECHECK_HEADSET,
	SHSWIC_RECHECK_CANCEL,
};

enum
{
	SHSWIC_CB_INIT_NONE = 0,
	SHSWIC_CB_INIT_INITIALIZING,
	SHSWIC_CB_INIT_COMPLETE,
};

#ifdef CONFIG_HEADSET_BUTTON_SWIC
enum
{
	SHSWIC_SEND_HEADSET_SW_ON,
	SHSWIC_SEND_HEADSET_SW_OFF,
};
#endif /* CONFIG_HEADSET_BUTTON_SWIC */

typedef struct shswic_cmd_tag
{
	struct work_struct	work;
	int					sig;
} shswic_cmd_t;

typedef struct shswic_delay_cmd_tag
{
	struct delayed_work	dwork;
	int					sig;
} shswic_delay_cmd_t;

/* ____________________________________________________________________________________________________________________________ */
/**
 *	Internal variable
 */

typedef void (*shswic_cb_func_t)(uint8_t device, void* user_data);
static shswic_cb_func_t			shswic_cb_func[SHSWIC_DEVICE_NUM];
static uint32_t					shswic_cb_irq[SHSWIC_DEVICE_NUM];
static void*					shswic_cb_userdata[SHSWIC_DEVICE_NUM];
static uint8_t					shswic_cb_last_usb_port[SHSWIC_DEVICE_NUM];
#ifdef CONFIG_CRADLE_SWIC
static uint8_t					shswic_cb_last_cradle_port[SHSWIC_DEVICE_NUM];
#endif /* CONFIG_CRADLE_SWIC */
static uint32_t					shswic_last_detect = SHSWIC_ID_NONE;
static struct i2c_client 		*shswic_i2c_client;
static struct workqueue_struct	*shswic_wq = NULL;
static struct mutex				shswic_task_lock;
static struct mutex				shswic_cb_lock;
static atomic_t					shswic_cb_init_state = ATOMIC_INIT(SHSWIC_CB_INIT_NONE);
static atomic_t					shswic_delay_cancel_sig = ATOMIC_INIT(0);
static uint8_t					shswic_init_flag = SHSWIC_INIT;
uint8_t							shswic_detect_state = SHSWIC_STATE_NONE;
uint8_t							shswic_detect_id = 0x0d;
uint8_t							shswic_detect_port = SHSWIC_ID_NONE;
#ifdef CONFIG_CRADLE_SWIC
static uint8_t					shswic_cradle_port = SHSWIC_ID_CRADLE_UNKNOWN;
#endif /* CONFIG_CRADLE_SWIC */
#ifdef CONFIG_SII8334_MHL_TX
shswic_read_status_t			shswic_read_data;
static struct device			*g_swic_regulator_dev = NULL;
#endif /* CONFIG_SII8334_MHL_TX */
#ifdef CONFIG_HEADSET_BUTTON_SWIC
static struct input_dev*		shswic_input_dev = NULL;
static int						shswic_last_send_sw = SHSWIC_SEND_HEADSET_SW_OFF;
#endif /* CONFIG_HEADSET_BUTTON_SWIC */
static uint16_t					shswic_signal_pkt_use_bitflg = 0;
static shswic_cmd_t*			shswic_signal_pkt = NULL;
static uint16_t					shswic_timer_signal_pkt_use_bitflg = 0;
static shswic_delay_cmd_t*		shswic_timer_signal_pkt = NULL;
#ifdef CONFIG_HEADSET_BUTTON_SWIC
bool						shswic_switch_earphone = false;
#endif /* CONFIG_HEADSET_BUTTON_SWIC */

/* ____________________________________________________________________________________________________________________________ */
/**
 *	Internal prototypes
 */

static void shswic_signal_handler(struct work_struct *poWork);
static void shswic_signal_delay_handler(struct work_struct *poWork);
static void shswic_signal_handler_core(long sig);
static void shswic_send_signal(long sig);
static void shswic_detect_cb_init(void);
static void shswic_status_init(void);
static void shswic_detect_retry_sig_handler(void);

static void shswic_detect_cb_call(uint8_t detect);
static void shswic_detect_cb_call_func(uint8_t detect);

static int8_t shswic_state_usb_cable_proc(uint8_t detect);
#ifdef CONFIG_HOST_SWIC
static int8_t shswic_state_usb_host_cable_proc(uint8_t detect);
#endif /* CONFIG_HOST_SWIC */
#ifdef CONFIG_AC_ADAPTER_SWIC
static int8_t shswic_state_ac_adapter_proc(uint8_t detect);
#endif /* CONFIG_AC_ADAPTER_SWIC */
#ifdef CONFIG_IRREGULAR_CHARGER_SWIC
static int8_t shswic_state_irregular_charger_proc(uint8_t detect);
#endif /* CONFIG_IRREGULAR_CHARGER_SWIC */
static int8_t shswic_state_headset_proc(uint8_t detect);
static int8_t shswic_state_headset_sw_proc(uint8_t detect);
static int8_t shswic_state_none_proc(uint8_t detect);
#ifdef CONFIG_SII8334_MHL_TX
static int8_t shswic_state_mhl_proc(uint8_t detect);
#endif /* CONFIG_SII8334_MHL_TX */
int8_t (* shswic_state_proc[])(uint8_t detect) =
{
	shswic_state_usb_cable_proc,
#ifdef CONFIG_HOST_SWIC
	shswic_state_usb_host_cable_proc,
#endif /* CONFIG_HOST_SWIC */
#ifdef CONFIG_AC_ADAPTER_SWIC
	shswic_state_ac_adapter_proc,
#endif /* CONFIG_AC_ADAPTER_SWIC */
#ifdef CONFIG_IRREGULAR_CHARGER_SWIC
	shswic_state_irregular_charger_proc,
#endif /* CONFIG_IRREGULAR_CHARGER_SWIC */
	shswic_state_headset_proc,
	shswic_state_headset_sw_proc,
	shswic_state_none_proc,
#ifdef CONFIG_SII8334_MHL_TX
	shswic_state_mhl_proc,
#endif /* CONFIG_SII8334_MHL_TX */
};

static void shswic_set_shswic_cb_init_state(int state);
static int shswic_get_shswic_cb_init_state(void);
static void shswic_set_delay_cancel_sig(int sig);
static void shswic_clear_delay_cancel_sig(int sig);
static int shswic_get_delay_cancel_sig(void);

static shswic_result_t shswic_diag_get_id_data(uint8_t* id_data);
static int shswic_diag_open(struct inode *inode, struct file *file);
static int shswic_diag_release(struct inode *inode, struct file *file);
static ssize_t shswic_diag_read(struct file *file, char __user *buf, size_t count, loff_t *ppos);
static long shswic_diag_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);

static int shswic_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int shswic_i2c_remove(struct i2c_client *client);

static int shswic_pm_driver_probe(struct platform_device *pdev);

#ifdef CONFIG_SII8334_MHL_TX
static void shswic_detect_cb_mhl_proc(void);
#endif /* CONFIG_SII8334_MHL_TX */

#ifdef CONFIG_HEADSET_BUTTON_SWIC
static void shswic_register_input_dev(void);
static void shswic_release_input_dev(void);
#endif /* CONFIG_HEADSET_BUTTON_SWIC */

#ifdef CONFIG_SII8334_MHL_TX
static void shswic_call_mhl_func(uint8_t id_status, uint8_t status, uint8_t int_status);
#endif /* CONFIG_SII8334_MHL_TX */
static bool shswic_is_connected_headset(uint8_t int_status, uint8_t id_status);
static bool shswic_is_connected_switch_headset(uint8_t int_status, uint8_t id_status);
static void shswic_update_headset(uint8_t id_status, uint8_t int_status);
static bool shswic_is_headset_indo(uint8_t id_status);
#ifdef SHSWIC_UCDCNT_USE
static void shswic_setup_ucdcnt(void);
#endif /* SHSWIC_UCDCNT_USE */
static uint8_t shswic_detect_get_without_mhl(uint8_t id_status, uint8_t status, uint8_t int_status);
static void shswic_send_register_recheck_signal(long sig);
static void shswic_register_recheck_handler(struct work_struct *poWork);
static void shswic_register_recheck_headset_sig_handler(void);
static void shswic_register_recheck_sig_handler(void);

void shswic_detect_isr_sig_handler(void);
void shswic_detect_init(void);
#ifdef CONFIG_SII8334_MHL_TX
void shswic_detect_mhl_proc(void);
#endif /* CONFIG_SII8334_MHL_TX */
shswic_result_t shswic_write_vbsw_reg(uint8_t vbsw_ctl);
shswic_result_t shswic_read_vbsw_reg(uint8_t* vbsw_ctl);
void shswic_pre_detect_cb_call(uint8_t detect);
void shswic_pre_do_exit(void);
#ifdef CONFIG_CRADLE_SWIC
bool shswic_is_connected_cradle(void);
#endif /* CONFIG_CRADLE_SWIC */
bool shswic_is_push_headset_button(uint8_t int_status);
#ifdef CONFIG_CRADLE_SWIC
int shswic_cradle_state(void);
#endif /* CONFIG_CRADLE_SWIC */
static void shswic_do_exit(void);

static int __init shswic_init(void);
static void __exit shswic_exit(void);

static int shswic_pm_suspend(struct device *dev);
static int shswic_pm_resume(struct device *dev);
static int __devexit shswic_pm_remove( struct platform_device* dev_p );

static struct file_operations shswic_diag_fops = {
	.owner			= THIS_MODULE,
	.open			= shswic_diag_open,
	.release		= shswic_diag_release,
	.read			= shswic_diag_read,
	.unlocked_ioctl	= shswic_diag_ioctl,

};
static struct miscdevice shswic_diag_device = {
	.minor			= MISC_DYNAMIC_MINOR,
	.name			= SHSWIC_DIAG_DRIVER_NAME,
	.fops			= &shswic_diag_fops,
};

static const struct i2c_device_id shswic_i2c_id[] = {
	{ SHSWIC_I2C_DRIVER_NAME, 0 },
	{ }
};

#ifdef CONFIG_OF
static const struct of_device_id shswic_i2c_table[] = {
	{ .compatible = "sharp,shswic_i2c",},
	{}
};
#else
#define shswic_i2c_table NULL;
#endif /* CONFIG_OF */

static const struct dev_pm_ops shswic_pm_ops = {
    .suspend =  shswic_pm_suspend,
    .resume  =  shswic_pm_resume,
}; 

static struct i2c_driver shswic_i2c_driver = {
	.driver = {
		.owner		= THIS_MODULE,
		.name		= SHSWIC_I2C_DRIVER_NAME,
		.pm			= &shswic_pm_ops,
#ifdef CONFIG_OF
		.of_match_table = shswic_i2c_table,
#endif /* CONFIG_OF */
	},
	.class			= I2C_CLASS_HWMON,
	.probe			= shswic_i2c_probe,
	.id_table		= shswic_i2c_id,
	.remove			= shswic_i2c_remove,
};

static struct platform_driver shswic_pm_driver = {
	.driver = {
		.name		= SHSWIC_REG_DRIVER_NAME,
		.owner		= THIS_MODULE,
//		.pm			= &shswic_pm_ops,
	},
	.probe			= shswic_pm_driver_probe,
    .remove			= __devexit_p(shswic_pm_remove),
};

static uint8_t					shswic_last_detect_state = SHSWIC_STATE_NONE;
//static uint8_t					shswic_last_int_status = 0x10;
#ifdef CONFIG_CRADLE_SWIC
static int						shswic_last_cradle_status = SHSWIC_ID_CRADLE_UNKNOWN;
#endif /* CONFIG_CRADLE_SWIC */
static bool						shswic_interrupt_init_state = false;
static uint8_t					shswic_1th_check_detect_port = SHSWIC_ID_NONE;
static struct delayed_work		shswic_register_recheck_signal_pkt;
static int						shswic_register_recheck_signal_pkt_state = SHSWIC_RECHECK_NON;
static long						shswic_register_recheck_signal_pkt_waiting_sig = SHSWIC_NO_SIG;
static struct wake_lock 		shswic_delayed_work_wake_lock;
static struct wake_lock 		shswic_delayed_work_wake_lock_hw;
static struct wake_lock			shswic_sleep_wake_lock;
#ifdef SHSWIC_UCDCNT_USE
static int						shswic_dcdfail_retry_cnt = 0;
#endif /* SHSWIC_UCDCNT_USE */
static struct wake_lock 		shswic_suspend_work_wake_lock;

#ifdef CONFIG_CRADLE_SWIC
#ifdef SWIC_CRADLE_DETECT_GPIO_VCDET
static bool						shswic_interrupt_cradle_state = false;
#endif /* SWIC_CRADLE_DETECT_GPIO_VCDET */
#endif /* CONFIG_CRADLE_SWIC */

shswic_headset_result_t shswic_swicheadset_state = SHSWIC_SWICDETECT_NONE;
uint8_t shswic_audioheadset_state = SHSWIC_AUDIODETECT_NONE;
uint8_t shswic_audioheadset_initflg = false;

int shswic_suspend_state = 0;

/* Debug Log */
int shswic_log_h   = 1;
int shswic_log_l   = 0;
int shswic_log_i2c = 0;

#if defined (CONFIG_ANDROID_ENGINEERING)
module_param(shswic_log_h,  int, 0600);
module_param(shswic_log_l, int, 0600);
module_param(shswic_log_i2c, int, 0600);
#endif /* CONFIG_ANDROID_ENGINEERING */

#define SHSWIC_LOG_TAG_H   "SHSWIC:H"
#define SHSWIC_LOG_TAG_L   "SHSWIC:L"
#define SHSWIC_LOG_TAG_I2C "SHSWIC:I"

#define SHSWIC_DEBUG_LOG_H(fmt, args...) \
		if(shswic_log_h == 1) { \
			printk(KERN_INFO "%s " fmt"\n", SHSWIC_LOG_TAG_H, ## args); \
		}

#define SHSWIC_DEBUG_LOG_L(fmt, args...) \
		if(shswic_log_l == 1) { \
			printk(KERN_INFO "[%s][%s(%d)] " fmt"\n", SHSWIC_LOG_TAG_L, __func__, __LINE__, ## args); \
		}

#define SHSWIC_DEBUG_LOG_I2C(fmt, args...) \
		if(shswic_log_i2c == 1) { \
			printk(KERN_INFO "[%s][%s(%d)] " fmt"\n", SHSWIC_LOG_TAG_I2C, __func__, __LINE__, ## args); \
		}

/* ____________________________________________________________________________________________________________________________ */
/**
 *	Internal Functions
 */

static void shswic_headset_inform_swoff(void)
{
	uint8_t switch_earphone_detect;

	if(shswic_switch_earphone){
		switch_earphone_detect = SHSWIC_ID_HEADSET_SW_OFF;
		if (!shswic_cb_func[SHSWIC_CODEC_DEVICE])
		{
			return;
		}
		if (!(shswic_cb_irq[SHSWIC_CODEC_DEVICE] & SHSWIC_ID_HEADSET_SW))
		{
			return;
		}
		if(shswic_cb_last_usb_port[SHSWIC_CODEC_DEVICE] != switch_earphone_detect)
		{
			SHSWIC_DEBUG_LOG_H("CallBack Call[detect = 0x%x]", switch_earphone_detect);
			shswic_cb_func[SHSWIC_CODEC_DEVICE]((uint8_t)switch_earphone_detect, shswic_cb_userdata[SHSWIC_CODEC_DEVICE]);
			shswic_cb_last_usb_port[SHSWIC_CODEC_DEVICE] = switch_earphone_detect;
		}
	}
}


shswic_headset_result_t shswic_headset_inform_state(uint8_t detect)
{
	uint8_t sw_ctl = 0x00;

	shswic_audioheadset_initflg = true;
	shswic_audioheadset_state = detect;

	if(shswic_init_flag != SHSWIC_IMPLEMENT){
		shswic_swicheadset_state = SHSWIC_SWICDETECT_NONE;
		return shswic_swicheadset_state;
	}
	
	if((shswic_last_detect_state == SHSWIC_STATE_HEADSET) || (shswic_last_detect_state == SHSWIC_STATE_HEADSET_SW))
	{
		shswic_swicheadset_state =SHSWIC_SWICDETECT_HEADSET;
	}
	else
	{
		shswic_swicheadset_state =SHSWIC_SWICDETECT_NONE;
	}

	if((shswic_swicheadset_state == SHSWIC_SWICDETECT_HEADSET) && (shswic_audioheadset_state == SHSWIC_AUDIODETECT_NONE))
	{
		sw_ctl = SHSWIC_NONE_PASS;
		shswic_i2c_write(&sw_ctl, 1, SHSWIC_REG_MUXSW_CONTROL);
		
		usleep(100000);
		
		sw_ctl = SHSWIC_HEADSET_PASS;
		shswic_i2c_write(&sw_ctl, 1, SHSWIC_REG_MUXSW_CONTROL);
	}
	else if((shswic_swicheadset_state == SHSWIC_SWICDETECT_HEADSET) && (shswic_audioheadset_state == SHSWIC_AUDIODETECT_HEADSET))
	{
#ifdef CONFIG_HEADSET_BUTTON_SWIC
		if(SHSWIC_SEND_HEADSET_SW_ON == shswic_last_send_sw)
		{
			input_report_key(shswic_input_dev, KEY_MEDIA, 0);
			input_sync(shswic_input_dev);
			shswic_last_send_sw = SHSWIC_SEND_HEADSET_SW_OFF;
		}
#endif /* CONFIG_HEADSET_BUTTON_SWIC */
		shswic_send_signal(SHSWIC_HEADSET_INFORM_SWOFF_SIG);

		sw_ctl = 0x00;
		shswic_i2c_write(&sw_ctl, 1, SHSWIC_REG_MUXSW_CONTROL);
	}

	return shswic_swicheadset_state;
}

shswic_result_t shswic_detect_cb_regist(uint8_t cb_dev, uint32_t cb_irq, void* cb_func, void* user_data)
{
	shswic_result_t ret = SHSWIC_SUCCESS;

	SHSWIC_DEBUG_LOG_L("shswic_detect_cb_regist Start[cb_irq:0x%x][cb_dev:0x%x]", cb_irq, cb_dev);

	shswic_detect_cb_init();

	mutex_lock(&shswic_cb_lock);

	switch (cb_dev)
	{

		case SHSWIC_VBUS_DEVICE:
		case SHSWIC_CHG_DEVICE:
		case SHSWIC_OFFCHG_DEVICE:
		case SHSWIC_CODEC_DEVICE:
			shswic_cb_func[cb_dev]     = (shswic_cb_func_t)cb_func;
			shswic_cb_irq[cb_dev]      = cb_irq;
			shswic_cb_userdata[cb_dev] = user_data;

			if((!shswic_cb_func[cb_dev]) || (!shswic_cb_irq[cb_dev]))
			{
				SHSWIC_DEBUG_LOG_L("shswic_detect_cb_regist callback null");
			}else{
				if(SHSWIC_IMPLEMENT == shswic_init_flag)
				{
					SHSWIC_DEBUG_LOG_L("SHSWIC_CALLBACK_SIG");
					shswic_send_timer_signal(SHSWIC_CALLBACK_SIG, SHSWIC_CALLBACK_RETRY_TIME);
				}
			}
			SHSWIC_DEBUG_LOG_L("shswic_detect_cb_regist Success[cb_dev:0x%x]", cb_dev);
			break;
		default:
			SHSWIC_DEBUG_LOG_L("shswic_detect_cb_regist Param Error[cb_dev:0x%x]", cb_dev);
			ret =  SHSWIC_PARAM_ERROR;
			break;
	}

	mutex_unlock(&shswic_cb_lock);
	return ret;
}

shswic_result_t shswic_get_usb_port_status(uint8_t* device)
{
	SHSWIC_DEBUG_LOG_L("shswic_get_usb_port_status Start[State:0x%x]", shswic_detect_port);

	if (device == NULL)
	{
		SHSWIC_DEBUG_LOG_H("shswic_get_usb_port_status SHSWIC_PARAM_ERROR");
		return SHSWIC_PARAM_ERROR;
	}
	*device = shswic_detect_port;
	return SHSWIC_SUCCESS;
}

shswic_result_t shswic_get_cradle_status(uint8_t* device)
{
#ifdef CONFIG_CRADLE_SWIC
	SHSWIC_DEBUG_LOG_L("shswic_get_cradle_status Start[State:0x%x]", shswic_cradle_port);
#endif /* CONFIG_CRADLE_SWIC */

	if (device == NULL)
	{
		SHSWIC_DEBUG_LOG_H("shswic_get_cradle_status SHSWIC_PARAM_ERROR");
		return SHSWIC_PARAM_ERROR;
	}

#ifdef CONFIG_CRADLE_SWIC
	if (shswic_cradle_port == SHSWIC_ID_CRADLE)
	{
		*device = SHSWIC_ID_CRADLE;
	}
	else if(shswic_cradle_port == SHSWIC_ID_CRADLE_UNKNOWN)
	{
		*device = SHSWIC_ID_CRADLE_UNKNOWN;
	}
	else
	{
		*device = SHSWIC_ID_NONE;
	}
	return SHSWIC_SUCCESS;
#else /* CONFIG_CRADLE_SWIC */
	*device = SHSWIC_ID_NONE;
	return SHSWIC_SUCCESS;
#endif /* CONFIG_CRADLE_SWIC */
}

shswic_result_t shswic_write_vbsw_reg(uint8_t vbsw_ctl)
{
	uint8_t ovp_ctl = 0x00;
	shswic_result_t result = SHSWIC_FAILURE;

	if (vbsw_ctl > SHSWIC_VBSW_OFF)
	{
		SHSWIC_DEBUG_LOG_H("shswic_write_vbsw_reg SHSWIC_PARAM_ERROR");
		return SHSWIC_PARAM_ERROR;
	}

	if (shswic_init_flag == SHSWIC_IMPLEMENT)
	{
		SHSWIC_DEBUG_LOG_L("shswic_write_vbsw_reg vbsw_ctl = %d", vbsw_ctl);
	
		result = shswic_i2c_read(&ovp_ctl, 1, SHSWIC_REG_VBSW_CONTROL);
		if(SHSWIC_SUCCESS != result)
		{
			return result;
		}

		SHSWIC_DEBUG_LOG_L("shswic_write_vbsw_reg ovp_ctl: 0x%x", ovp_ctl);
		
		switch(vbsw_ctl)
		{
			case SHSWIC_VBSW_OFF:
				ovp_ctl |= 0x01; /* VBSWDISEN FLG is ON. */
				break;
				
			case SHSWIC_VBSW_AUTO:
				ovp_ctl &= 0xFE; /* VBSWDISEN FLG is OFF. */
				break;
				
			default:
				return SHSWIC_PARAM_ERROR;
		}

		SHSWIC_DEBUG_LOG_L("shswic_write_vbsw_reg ovp_ctl: 0x%x", ovp_ctl);
		result = shswic_i2c_write(&ovp_ctl, 1, SHSWIC_REG_VBSW_CONTROL);
	}
	return result;
}

shswic_result_t shswic_read_vbsw_reg(uint8_t* vbsw_ctl)
{
	shswic_result_t result = SHSWIC_FAILURE;

	if (shswic_init_flag == SHSWIC_IMPLEMENT)
	{
		SHSWIC_DEBUG_LOG_L("shswic_read_vbsw_reg vbsw_ctl = %d", *vbsw_ctl);
		if (vbsw_ctl == NULL)
		{
			SHSWIC_DEBUG_LOG_H("shswic_read_vbsw_reg SHSWIC_PARAM_ERROR");
			return SHSWIC_PARAM_ERROR;
		}
		return shswic_i2c_read(vbsw_ctl, 1, SHSWIC_REG_VBSW_CONTROL);
	}
	return result;
}

void shswic_pre_detect_cb_call(uint8_t detect)
{
#ifdef CONFIG_CRADLE_SWIC
	if((SHSWIC_ID_CRADLE == detect)
	|| (SHSWIC_ID_CRADLE_NONE == detect))
	{
		return;
	}
#endif /* CONFIG_CRADLE_SWIC */

#ifdef CONFIG_HOST_SWIC
	if (detect == SHSWIC_ID_USB_HOST_CABLE)
	{
		shswic_write_vbsw_reg(SHSWIC_VBSW_OFF);
	}
	else
	{
		shswic_write_vbsw_reg(SHSWIC_VBSW_AUTO);
	}
#endif /* CONFIG_HOST_SWIC */
}

static void shswic_update_headset(uint8_t id_status, uint8_t int_status)
{
	uint8_t sw_ctl = 0x00;

	if((shswic_last_detect_state != SHSWIC_STATE_HEADSET_SW)
	&& (shswic_last_detect_state != SHSWIC_STATE_HEADSET))
	{
#if 0
		sw_ctl = SHSWIC_NONE_PASS;
		shswic_i2c_write(&sw_ctl, 1, SHSWIC_REG_MUXSW_CONTROL);
		
		wake_lock(&shswic_sleep_wake_lock);
		usleep(100000);
		wake_unlock(&shswic_sleep_wake_lock);
#endif
		if((id_status & SHSWIC_ID_STA_INDO_MASK) == 0x08)
		{
			/* stereo */
			sw_ctl = SHSWIC_HEADSET_PASS;
		} else if((id_status & SHSWIC_ID_STA_INDO_MASK) == 0x0C)
		{
			/* mono */
			sw_ctl = SHSWIC_HEADSET_PASS;
		} else
		{
			sw_ctl = SHSWIC_HEADSET_PASS;
		}
		shswic_i2c_write(&sw_ctl, 1, SHSWIC_REG_MUXSW_CONTROL);
		
		gpio_set_value(SHSWIC_GPIO_HS_DET, SHSWIC_GPIO_LOW);
	}
#ifdef CONFIG_HEADSET_BUTTON_SWIC
	else
	{
		shswic_send_headset_sw_button(int_status);
	}
#endif /* CONFIG_HEADSET_BUTTON_SWIC */
}

#ifdef SHSWIC_UCDCNT_USE
void shswic_setup_ucdcnt(void)
{
	int ret = 0;
	uint8_t ucdcnt = 0x44;	/* 0x40 : INTBEN */
							/* 0x04 : USBDETCTRL */
	ret = shswic_i2c_read(&ucdcnt, 1, SHSWIC_REG_UCDCNT);
	if (ret == SHSWIC_SUCCESS)
	{
		ucdcnt |= 0x04;	/* set USBDETCTRL FLG */
	}
	else
	{
		ucdcnt = 0x44;
	}

	ret = shswic_i2c_write(&ucdcnt, 1, SHSWIC_REG_UCDCNT);
	SHSWIC_DEBUG_LOG_L("shswic_setup_ucdcnt ret(%d) ucdcnt(0x%x)", ret, ucdcnt);

	wake_lock(&shswic_sleep_wake_lock);
	usleep(30000);
	wake_unlock(&shswic_sleep_wake_lock);

	ucdcnt |= 0x01;	/* set DCDRETRY FLG */
	ret = shswic_i2c_write(&ucdcnt, 1, SHSWIC_REG_UCDCNT);
	SHSWIC_DEBUG_LOG_L("shswic_setup_ucdcnt ret(%d) ucdcnt(0x%x)", ret, ucdcnt);
}
#endif /* SHSWIC_UCDCNT_USE */

/* ____________________________________________________________________________________________________________________________ */
/**
 *	Main Functions
 */
 
static void shswic_register_recheck_sig_handler(void)
{
	uint8_t id_status, status, int_status;
	uint8_t detect_port = shswic_detect_port;
	uint8_t sw_ctl = 0x00;

	SHSWIC_DEBUG_LOG_L("start");

	shswic_status_read(&id_status, &status, &int_status);
	shswic_status_read(&id_status, &status, &int_status);

	detect_port = shswic_detect_get(id_status, status, int_status);

	if(detect_port != shswic_1th_check_detect_port)
	{
		SHSWIC_DEBUG_LOG_L("detect_port != shswic_1th_check_detect_port");
		shswic_1th_check_detect_port = detect_port;
		shswic_send_register_recheck_signal(SHSWIC_REG_RECHECK_SIG);
		return;
	}
	
	if((int_status & SHSWIC_INT_STA_VBUS_MASK) == 0x00) {
		SHSWIC_DEBUG_LOG_L("usb-pc charging stop.");
#ifdef CONFIG_SII8334_MHL_TX
		shmhl_notify_usb_pc_charge(false);
#endif /* CONFIG_SII8334_MHL_TX */
	}
	
	switch(detect_port)
	{
		case SHSWIC_ID_MHL:
#ifdef CONFIG_SII8334_MHL_TX
			/* mhl application without vbus */
			shswic_call_mhl_func(id_status, status, int_status);
#endif /* CONFIG_SII8334_MHL_TX */
			return;

		case SHSWIC_ID_HEADSET:
			if(SHSWIC_STATE_HEADSET_SW == shswic_last_detect_state)
			{
				SHSWIC_DEBUG_LOG_L("shswic_register_recheck_sig_handler shswic_detect_get:HEADSET");
				shswic_detect_state = SHSWIC_STATE_HEADSET;
				shswic_update_headset(id_status, int_status);
			}
			else
			{
				shswic_send_register_recheck_signal(SHSWIC_REG_RECHECK_HEADSET_SIG);
				return;
			}
			break;
		
		case SHSWIC_ID_HEADSET_SW:
			SHSWIC_DEBUG_LOG_L("shswic_register_recheck_sig_handler:HEADSET_SW");
			shswic_detect_state = SHSWIC_STATE_HEADSET_SW;
#ifdef CONFIG_HEADSET_BUTTON_SWIC
			shswic_switch_earphone = true;
#endif /* CONFIG_HEADSET_BUTTON_SWIC */
			shswic_update_headset(id_status, int_status);
			break;
		
#ifdef CONFIG_HOST_SWIC
		case SHSWIC_ID_USB_HOST_CABLE:
			SHSWIC_DEBUG_LOG_L("shswic_register_recheck_sig_handler:USB_HOST_CABLE");
			shswic_detect_state = SHSWIC_STATE_USB_HOST_CABLE;
#ifdef CONFIG_HEADSET_BUTTON_SWIC
			shswic_switch_earphone = false;
#endif /* CONFIG_HEADSET_BUTTON_SWIC */
			break;
#endif /* CONFIG_HOST_SWIC */

		case SHSWIC_ID_USB_CABLE:
			SHSWIC_DEBUG_LOG_L("shswic_register_recheck_sig_handler:USB_CABLE");
			shswic_detect_state = SHSWIC_STATE_USB_CABLE;
#ifdef CONFIG_HEADSET_BUTTON_SWIC
			shswic_switch_earphone = false;
#endif /* CONFIG_HEADSET_BUTTON_SWIC */
			sw_ctl = SHSWIC_USB_PASS;
			shswic_i2c_write(&sw_ctl, 1, SHSWIC_REG_MUXSW_CONTROL);
			break;
		
#ifdef CONFIG_AC_ADAPTER_SWIC
		case SHSWIC_ID_AC_ADAPTER:
			SHSWIC_DEBUG_LOG_L("shswic_register_recheck_sig_handler:AC_ADAPTER");
			shswic_detect_state = SHSWIC_STATE_AC_ADAPTER;
#ifdef CONFIG_HEADSET_BUTTON_SWIC
			shswic_switch_earphone = false;
#endif /* CONFIG_HEADSET_BUTTON_SWIC */
			sw_ctl = SHSWIC_USB_PASS;
			shswic_i2c_write(&sw_ctl, 1, SHSWIC_REG_MUXSW_CONTROL);
			break;
#endif /* CONFIG_AC_ADAPTER_SWIC */

#ifdef CONFIG_IRREGULAR_CHARGER_SWIC
		case SHSWIC_ID_IRREGULAR_CHARGER:
			SHSWIC_DEBUG_LOG_L("shswic_register_recheck_sig_handler:IRREGULAR_CHARGER");
			shswic_detect_state = SHSWIC_STATE_IRREGULAR_CHARGER;
#ifdef CONFIG_HEADSET_BUTTON_SWIC
			shswic_switch_earphone = false;
#endif /* CONFIG_HEADSET_BUTTON_SWIC */
			break;
#endif /* CONFIG_IRREGULAR_CHARGER_SWIC */
		
		case SHSWIC_ID_NONE:
			SHSWIC_DEBUG_LOG_L("shswic_register_recheck_sig_handler:NONE");
			shswic_detect_state = SHSWIC_STATE_NONE;
#ifdef CONFIG_HEADSET_BUTTON_SWIC
			shswic_switch_earphone = false;
#endif /* CONFIG_HEADSET_BUTTON_SWIC */
			sw_ctl = SHSWIC_NONE_PASS;
			shswic_i2c_write(&sw_ctl, 1, SHSWIC_REG_MUXSW_CONTROL);
			gpio_set_value(SHSWIC_GPIO_HS_DET, SHSWIC_GPIO_HIGH);
			break;
		
		case SHSWIC_ID_RECHECK:
			return;
		
		default:
			break;
	}

	shswic_detect_id = (id_status & SHSWIC_ID_STA_MASK);

	shswic_state_proc[shswic_last_detect_state](detect_port);

	shswic_last_detect_state = shswic_detect_state;

	return;
}

static void shswic_register_recheck_headset_sig_handler(void)
{
	uint8_t id_status, status, int_status;
	uint8_t detect_port = shswic_detect_port;

	SHSWIC_DEBUG_LOG_L("start");

	shswic_status_read(&id_status, &status, &int_status);
	shswic_status_read(&id_status, &status, &int_status);

	detect_port = shswic_detect_get(id_status, status, int_status);

	if(detect_port != SHSWIC_ID_HEADSET)
	{
		SHSWIC_DEBUG_LOG_L("detect_port != SHSWIC_ID_HEADSET");
		return;
	}

	shswic_detect_state = SHSWIC_STATE_HEADSET;
	shswic_update_headset(id_status, int_status);

	shswic_detect_id = (id_status & SHSWIC_ID_STA_MASK);

	shswic_state_proc[shswic_last_detect_state](detect_port);

	shswic_last_detect_state = shswic_detect_state;
}

static void shswic_send_register_recheck_signal(long sig)
{
	int msec;
	struct delayed_work* pkt_p;
	bool non_delayed_work = false;

	if(wake_lock_active(&shswic_delayed_work_wake_lock_hw))
	{
		wake_unlock(&shswic_delayed_work_wake_lock_hw);
	}

	pkt_p = &(shswic_register_recheck_signal_pkt);

	if(SHSWIC_RECHECK_NON != shswic_register_recheck_signal_pkt_state)
	{
		non_delayed_work = cancel_delayed_work(pkt_p);
		SHSWIC_DEBUG_LOG_L("cancel_delayed_work(%d)", non_delayed_work ? 1 : 0);
	}
	else
	{
		non_delayed_work = true;
	}

	shswic_register_recheck_signal_pkt_waiting_sig = SHSWIC_NO_SIG;

	switch(sig)
	{
		case SHSWIC_REG_RECHECK_SIG:
			shswic_register_recheck_signal_pkt_state = SHSWIC_RECHECK_SECOND;
			msec = 50;
			break;
			
		case SHSWIC_REG_RECHECK_HEADSET_SIG:
			shswic_register_recheck_signal_pkt_state = SHSWIC_RECHECK_HEADSET;
			msec = 300;
			break;
		
		default:
			/* SHSWIC_REG_RECHECK_NON_SIG */
			/* no supported SIG */
			return;
	}

	if(non_delayed_work == false)
	{
		shswic_register_recheck_signal_pkt_waiting_sig = sig;
		shswic_register_recheck_signal_pkt_state = SHSWIC_RECHECK_CANCEL;
		return;
	}
	
	INIT_DELAYED_WORK(pkt_p, shswic_register_recheck_handler);
	wake_lock(&shswic_delayed_work_wake_lock_hw);
	queue_delayed_work(shswic_wq, pkt_p, msecs_to_jiffies(msec));

	return;
}

static void shswic_register_recheck_handler(struct work_struct *poWork)
{
	struct delayed_work* pkt_p;
	int state;
	
	if(wake_lock_active(&shswic_delayed_work_wake_lock_hw))
	{
		wake_unlock(&shswic_delayed_work_wake_lock_hw);
	}
	
	state = shswic_register_recheck_signal_pkt_state;
	shswic_register_recheck_signal_pkt_state = SHSWIC_RECHECK_NON;

	pkt_p = (struct delayed_work*)poWork;

	switch(state)
	{
		case SHSWIC_RECHECK_SECOND:
			SHSWIC_DEBUG_LOG_L("SHSWIC_REG_RECHECK_SIG");
			shswic_register_recheck_sig_handler();
			break;
		
		case SHSWIC_RECHECK_HEADSET:
			SHSWIC_DEBUG_LOG_L("SHSWIC_REG_RECHECK_HEADSET_SIG");
			shswic_register_recheck_headset_sig_handler();
			break;

		case SHSWIC_RECHECK_CANCEL:
			SHSWIC_DEBUG_LOG_L("SHSWIC_RECHECK_CANCEL");
			shswic_send_register_recheck_signal(shswic_register_recheck_signal_pkt_waiting_sig);
			break;
			
		default:
			break;
	}
}

static void shswic_signal_handler(struct work_struct *poWork)
{
	shswic_cmd_t* pkt_p;
	int idx = 0;

	mutex_lock(&shswic_task_lock);

	pkt_p = (shswic_cmd_t*)poWork;
	if(!shswic_suspend_state) {
	    shswic_signal_handler_core(pkt_p->sig);
	}
	idx = ((int)pkt_p - (int)shswic_signal_pkt) / sizeof(shswic_cmd_t);
	shswic_signal_pkt_use_bitflg &= ~(0x01 << idx);
	SHSWIC_DEBUG_LOG_L("shswic_signal_handler idx(%d) bit(0x%x)", idx, shswic_signal_pkt_use_bitflg);

	mutex_unlock(&shswic_task_lock);
	return;
}

static void shswic_signal_delay_handler(struct work_struct *poWork)
{
	shswic_delay_cmd_t* pkt_p;
	int idx = 0;

	mutex_lock(&shswic_task_lock);

	pkt_p = (shswic_delay_cmd_t*)poWork;

	if ((shswic_get_delay_cancel_sig() & pkt_p->sig) == 0)
	{
		shswic_signal_handler_core(pkt_p->sig);
	}
	shswic_clear_delay_cancel_sig(pkt_p->sig);

	idx = ((int)pkt_p - (int)shswic_timer_signal_pkt) / sizeof(shswic_delay_cmd_t);
	shswic_timer_signal_pkt_use_bitflg &= ~(0x01 << idx);
	SHSWIC_DEBUG_LOG_L("shswic_signal_delay_handler idx(%d) bit(0x%x)", idx, shswic_timer_signal_pkt_use_bitflg);

	if(0x00 == shswic_timer_signal_pkt_use_bitflg)
	{
		/* all delayed event completed */
		if(wake_lock_active(&shswic_delayed_work_wake_lock))
		{
			SHSWIC_DEBUG_LOG_L("delayed_wake_unlock");
			wake_unlock(&shswic_delayed_work_wake_lock);
		}
	}

	mutex_unlock(&shswic_task_lock);
	return;
}

static void shswic_signal_handler_core(long sig)
{
	switch (sig)
	{
		case SHSWIC_DETECT_ISR_SIG:
			SHSWIC_DEBUG_LOG_L("Call shswic_detect_isr_sig_handler");
			shswic_detect_isr_sig_handler();
			break;

		case SHSWIC_DETECT_INIT_SIG:
			SHSWIC_DEBUG_LOG_L("Call shswic_detect_init");
			shswic_detect_init();
			break;

		case SHSWIC_INIT_SIG:
			SHSWIC_DEBUG_LOG_L("Call shswic_status_init");
			shswic_status_init();
			break;

		case SHSWIC_DETECT_RETRY_SIG:
			SHSWIC_DEBUG_LOG_L("Call shswic_detect_retry_sig_handler");
			shswic_detect_retry_sig_handler();
			break;

#ifdef CONFIG_CRADLE_SWIC
		case SHSWIC_CRADLE_SIG:
			SHSWIC_DEBUG_LOG_L("Call shswic_state_cradle_proc");
			shswic_state_cradle_proc();
			break;
#endif /* CONFIG_CRADLE_SWIC */

#ifdef CONFIG_SII8334_MHL_TX
		case SHSWIC_DETECT_CB_MHL_SIG:
			SHSWIC_DEBUG_LOG_L("Call shswic_detect_cb_mhl_proc");
			shswic_detect_cb_mhl_proc();
			SHSWIC_DEBUG_LOG_L("Call shswic_detect_mhl_proc");
			shswic_detect_mhl_proc();
			break;
#endif /* CONFIG_SII8334_MHL_TX */

		case SHSWIC_CALLBACK_SIG:
			SHSWIC_DEBUG_LOG_L("Call shswic_detect_cb_call shswic_detect_port");
			shswic_detect_cb_call(shswic_detect_port);
#ifdef CONFIG_CRADLE_SWIC
			SHSWIC_DEBUG_LOG_L("Call shswic_detect_cb_call shswic_cradle_port");
			shswic_detect_cb_call(shswic_cradle_port);
#endif /* CONFIG_CRADLE_SWIC */
			break;

		case SHSWIC_HEADSET_INFORM_SWOFF_SIG:
			SHSWIC_DEBUG_LOG_L("SHSWIC_HEADSET_INFORM_SWOFF_SIG ");
			shswic_headset_inform_swoff();
			break;

		default:
			break;
	}
	return;
}

static void shswic_send_signal(long sig)
{
	shswic_cmd_t* pkt_p;
	int idx = 0;
	
	SHSWIC_DEBUG_LOG_L("Set Sig 0x%lx", sig);

	for(idx = 0; idx < SHSWIC_SIGNAL_PKT_COUNTS; ++idx)
	{
		if(!(shswic_signal_pkt_use_bitflg & (0x01 << idx)))
		{
		break;
		}
	}

	if(idx >= SHSWIC_SIGNAL_PKT_COUNTS)
	{
		SHSWIC_DEBUG_LOG_L("shswic_send_signal sig(0x%lx)", sig);
		return;
	}
	pkt_p = &(shswic_signal_pkt[idx]);
	shswic_signal_pkt_use_bitflg |= (0x01 << idx);
	SHSWIC_DEBUG_LOG_L("shswic_send_signal idx(%d) bit(0x%x)", idx, shswic_signal_pkt_use_bitflg);

	pkt_p->sig = sig;
	INIT_WORK((struct work_struct*)pkt_p, shswic_signal_handler);
	queue_work(shswic_wq, (struct work_struct*)pkt_p);
	return;
}

void shswic_send_timer_signal(long sig, int msec)
{
	shswic_delay_cmd_t* pkt_p;
	int idx = 0;

	SHSWIC_DEBUG_LOG_L("Set Delay Sig 0x%lx(%dms)", sig, msec);
	for(idx = 0; idx < SHSWIC_TIMER_SIGNAL_PKT_COUNTS; ++idx)
	{
		if(!(shswic_timer_signal_pkt_use_bitflg & (0x01 << idx)))
		{
		break;
		}
	}

	if(idx >= SHSWIC_TIMER_SIGNAL_PKT_COUNTS)
	{
		SHSWIC_DEBUG_LOG_L("shswic_send_timer_signal sig(0x%lx)", sig);
		return;
	}
	pkt_p = &(shswic_timer_signal_pkt[idx]);
	shswic_timer_signal_pkt_use_bitflg |= (0x01 << idx);
	SHSWIC_DEBUG_LOG_L("shswic_send_timer_signal idx(%d) bit(0x%x)", idx, shswic_timer_signal_pkt_use_bitflg);

	if(wake_lock_active(&shswic_delayed_work_wake_lock) == false)
	{
		SHSWIC_DEBUG_LOG_L("delayed_wake_lock");
		wake_lock(&shswic_delayed_work_wake_lock);
	}

	pkt_p->sig = sig;
	INIT_DELAYED_WORK((struct delayed_work*)pkt_p, shswic_signal_delay_handler);
	queue_delayed_work(shswic_wq, (struct delayed_work*)pkt_p, msecs_to_jiffies(msec));
	return;
}

void shswic_clear_timer_signal(long sig)
{
	SHSWIC_DEBUG_LOG_L("Cancel Delay Sig 0x%lx Start", sig);

	shswic_set_delay_cancel_sig(sig);

	return;
}

static void shswic_detect_cb_init(void)
{
	int i;

	if (shswic_get_shswic_cb_init_state() == SHSWIC_CB_INIT_INITIALIZING)
	{
		for (i = 0; i < 10; i++)
		{
			/* Don't call wake_lock. */
			usleep(10000);
			if (shswic_get_shswic_cb_init_state() == SHSWIC_CB_INIT_COMPLETE)
			{
				break;
			}
		}
	}

	if (shswic_get_shswic_cb_init_state() == SHSWIC_CB_INIT_NONE)
	{
		SHSWIC_DEBUG_LOG_L("SHSWIC_CB_INIT_NONE");

		shswic_set_shswic_cb_init_state(SHSWIC_CB_INIT_INITIALIZING);

		mutex_init(&shswic_cb_lock);

		for (i = 0; i < SHSWIC_DEVICE_NUM; i++)
		{
			shswic_cb_func[i]     = NULL;
			shswic_cb_irq[i]      = 0;
			shswic_cb_userdata[i] = NULL;
			shswic_cb_last_usb_port[i] = SHSWIC_ID_NONE;
#ifdef CONFIG_CRADLE_SWIC
			shswic_cb_last_cradle_port[i] = SHSWIC_ID_NONE;
#endif /* CONFIG_CRADLE_SWIC */
		}

		shswic_set_shswic_cb_init_state(SHSWIC_CB_INIT_COMPLETE);
	}
	return;
}

static void shswic_status_init(void)
{
	static uint8_t shswic_init_retry = 0;
	uint8_t id_status = 0;
	uint8_t status = 0;
	uint8_t int_status = 0;
	int8_t ret;

	SHSWIC_DEBUG_LOG_L("Start");

	ret = shswic_status_read(&id_status, &status, &int_status);
	if (ret == SHSWIC_SUCCESS)
	{
		shswic_init_flag = SHSWIC_IMPLEMENT;
		SHSWIC_DEBUG_LOG_L("SHSWIC_IMPLEMENT");
		if(SHSWIC_IMPLEMENT == shswic_init_flag)
		{
			shswic_send_signal(SHSWIC_DETECT_INIT_SIG);
			SHSWIC_DEBUG_LOG_L("shswic_status_init Success");
		}
	}
	else
	{
		if (shswic_init_retry < SHSWIC_INIT_RETRY_CNT)
		{
			shswic_init_retry++;
			shswic_send_timer_signal(SHSWIC_INIT_SIG, SHSWIC_INIT_RETRY_TIME);
		}
		else
		{
			shswic_init_flag = SHSWIC_NOT_IMPLEMENT;
			SHSWIC_DEBUG_LOG_L("SHSWIC_NOT_IMPLEMENT");
			shswic_do_exit();
		}
	}
	return;
}

irqreturn_t shswic_detect_isr(int irq, void *dev_id)
{
	SHSWIC_DEBUG_LOG_L("shswic_detect_isr Start");

	wake_lock_timeout(&shswic_suspend_work_wake_lock, SHSWIC_SUSPEND_WAKE_LOCK_TIMEOUT);

	shswic_send_signal(SHSWIC_DETECT_ISR_SIG);
	return IRQ_HANDLED;
}

#ifdef CONFIG_CRADLE_SWIC
irqreturn_t shswic_cradle_isr(int irq, void *dev_id)
{
	SHSWIC_DEBUG_LOG_L("shswic_cradle_isr Start");

	shswic_send_signal(SHSWIC_CRADLE_SIG);
	return IRQ_HANDLED;
}
#endif /* CONFIG_CRADLE_SWIC */

void shswic_detect_isr_sig_handler(void)
{
	uint8_t int_status = 0x00;
	uint8_t status = 0x00;
	uint8_t id_status = 0x0d;
	uint8_t detect = SHSWIC_ID_NONE;
#ifdef CONFIG_CRADLE_SWIC
	int		cradle_status = SHSWIC_ID_CRADLE_NONE;
#endif /* CONFIG_CRADLE_SWIC */

	SHSWIC_DEBUG_LOG_L("shswic_detect_isr_sig_handler Start");

	if(wake_lock_active(&shswic_delayed_work_wake_lock_hw))
	{
		wake_unlock(&shswic_delayed_work_wake_lock_hw);
	}

	shswic_status_read(&id_status, &status, &int_status);
	shswic_status_read(&id_status, &status, &int_status);

#ifdef CONFIG_CRADLE_SWIC
	if(int_status & 0x08)
	{
		cradle_status = SHSWIC_ID_CRADLE;
	}
	else
	{
		cradle_status = SHSWIC_ID_CRADLE_NONE;
	}
	
	if(cradle_status != shswic_last_cradle_status)
	{
		shswic_last_cradle_status = cradle_status;
		shswic_state_cradle_proc();
	}
#endif /* CONFIG_CRADLE_SWIC */

	detect = shswic_detect_get(id_status, status, int_status);

	if(SHSWIC_ID_RECHECK != detect)
	{
		shswic_1th_check_detect_port = detect;
		shswic_send_register_recheck_signal(SHSWIC_REG_RECHECK_SIG);
	}

	return;
}

uint8_t shswic_detect_get(uint8_t id_status, uint8_t status, uint8_t int_status)
{
#ifdef SHSWIC_UCDCNT_USE
	uint8_t id_det = 0x00;

	if((status & SHSWIC_STATUS_DCDFAIL_MASK) != 0x00)
	{
#ifdef CONFIG_SII8334_MHL_TX
		if(((int_status & SHSWIC_INT_STA_CHGDET_MASK) == 0x00)
		&& ((status & SHSWIC_STATUS_CHGPORT_MASK) == 0x20))
		{
			SHSWIC_DEBUG_LOG_L("shswic_detect_get:SHSWIC_ID_MHL");
			shswic_dcdfail_retry_cnt = 0;
			return SHSWIC_ID_MHL;
		}
#endif /* CONFIG_SII8334_MHL_TX */

#ifdef CONFIG_IRREGULAR_CHARGER_SWIC
		if (((status & SHSWIC_STATUS_CHGPORT_MASK) == 0x60)
		|| ((status & SHSWIC_STATUS_CHGPORT_MASK) == 0x40))
		{
			SHSWIC_DEBUG_LOG_L("shswic_detect_get:SHSWIC_ID_IRREGULAR_CHARGER");
			shswic_dcdfail_retry_cnt = 0;
			return SHSWIC_ID_IRREGULAR_CHARGER;
		}
#endif /* CONFIG_IRREGULAR_CHARGER_SWIC */

		SHSWIC_DEBUG_LOG_L("DCDFAIL");

		if (shswic_dcdfail_retry_cnt < SHSWIC_DCDFAIL_RETRY_CNT_MAX)
		{
			uint8_t ucdcnt = 0x44;
			shswic_result_t retUcdcnt = SHSWIC_FAILURE;

			shswic_dcdfail_retry_cnt++;
			
			/* ADCRETRY */
			shswic_i2c_read(&id_det, 1, SHSWIC_REG_IDDET);
			id_det &= ~0x02;
			shswic_i2c_write(&id_det, 1, SHSWIC_REG_IDDET);

			/* BCSRETRY */
			retUcdcnt = shswic_i2c_read(&ucdcnt, 1, SHSWIC_REG_UCDCNT);
			if (retUcdcnt == SHSWIC_SUCCESS)
			{
				ucdcnt &= ~0x01;
				shswic_i2c_write(&ucdcnt, 1, SHSWIC_REG_UCDCNT);
			}

			wake_lock(&shswic_sleep_wake_lock);
			usleep(30000);
			wake_unlock(&shswic_sleep_wake_lock);

			/* ADCRETRY */
			id_det |= 0x12;
			shswic_i2c_write(&id_det, 1, SHSWIC_REG_IDDET);

			/* BCSRETRY */
			if (retUcdcnt == SHSWIC_SUCCESS)
			{
				ucdcnt |= 0x01;
				shswic_i2c_write(&ucdcnt, 1, SHSWIC_REG_UCDCNT);
			}
			
			SHSWIC_DEBUG_LOG_L("DCDFAIL retry : %d", shswic_dcdfail_retry_cnt);

			if(wake_lock_active(&shswic_delayed_work_wake_lock_hw) == false)
			{
				wake_lock(&shswic_delayed_work_wake_lock_hw);
			}

			return SHSWIC_ID_RECHECK;
		}

		SHSWIC_DEBUG_LOG_L("DCDFAIL retry over");
		return SHSWIC_ID_NONE;
	}

	shswic_dcdfail_retry_cnt = 0;
#endif  /* SHSWIC_UCDCNT_USE */

	if ((id_status & SHSWIC_ID_STA_IDRDET_MASK) == 0x10)
	{
		SHSWIC_DEBUG_LOG_L("shswic_detect_isr_sig_handler IDRDET(1)");
#ifdef CONFIG_HOST_SWIC
		if ((shswic_last_detect_state == SHSWIC_STATE_USB_HOST_CABLE)
		&& ((id_status & SHSWIC_ID_STA_INDO_MASK) == 0x00))
		{
			SHSWIC_DEBUG_LOG_L("shswic_detect_get:SHSWIC_ID_USB_HOST_CABLE");
			return SHSWIC_ID_USB_HOST_CABLE;
		}
#endif /* CONFIG_HOST_SWIC */

#ifdef CONFIG_SII8334_MHL_TX
		if ((id_status & SHSWIC_ID_STA_INDO_MASK) == 0x0E)
		{
			SHSWIC_DEBUG_LOG_L("shswic_detect_get:SHSWIC_ID_MHL");
			return SHSWIC_ID_MHL;
		}
#endif /* CONFIG_SII8334_MHL_TX */
	}

	return shswic_detect_get_without_mhl(id_status, status, int_status);
}

uint8_t shswic_detect_get_without_mhl(uint8_t id_status, uint8_t status, uint8_t int_status)
{
	SHSWIC_DEBUG_LOG_L("shswic_detect_get INT = 0x%02x, STATUS = 0x%02x, ID = 0x%02x", int_status, status, id_status);

	if (shswic_is_connected_headset(int_status, id_status))
	{
		SHSWIC_DEBUG_LOG_L("shswic_detect_get:SHSWIC_ID_HEADSET");
		return SHSWIC_ID_HEADSET;
	}
	
	if(shswic_is_connected_switch_headset(int_status, id_status))
	{
		if((shswic_last_detect_state == SHSWIC_STATE_HEADSET)
		|| (shswic_last_detect_state == SHSWIC_STATE_HEADSET_SW))
		{
			SHSWIC_DEBUG_LOG_L("shswic_detect_get:SHSWIC_ID_HEADSET_SW");
			return SHSWIC_ID_HEADSET_SW;
		}

		SHSWIC_DEBUG_LOG_L("shswic_detect_get:SHSWIC_ID_HEADSET");
		return SHSWIC_ID_HEADSET;
	}

#ifdef CONFIG_HOST_SWIC
	if((id_status & SHSWIC_ID_STA_HOST_MASK) == 0x10)
	{
		SHSWIC_DEBUG_LOG_L("shswic_detect_get:USB_HOST_CABLE");
		return SHSWIC_ID_USB_HOST_CABLE;
	}
#endif /* CONFIG_HOST_SWIC */

#ifdef SHSWIC_UCDCNT_USE
	if((status & SHSWIC_STATUS_CHGPORT_MASK) == 0x20)
	{
		SHSWIC_DEBUG_LOG_L("shswic_detect_get:USB_CABLE");
		return SHSWIC_ID_USB_CABLE;
	}

#ifdef CONFIG_AC_ADAPTER_SWIC
	if ((status & SHSWIC_STATUS_CHGPORT_MASK) == 0x60)
	{
		SHSWIC_DEBUG_LOG_L("shswic_detect_get:AC_ADAPTER");
		return SHSWIC_ID_AC_ADAPTER;
	}
#endif /* CONFIG_AC_ADAPTER_SWIC */

	if ((status & SHSWIC_STATUS_CHGPORT_MASK) == 0x00)
	{
		SHSWIC_DEBUG_LOG_L("shswic_detect_get:NONE");
		return SHSWIC_ID_NONE;
	}

#else /* SHSWIC_UCDCNT_USE */

	if(((id_status & SHSWIC_ID_STA_INDO_MASK) == 0x0d) || ((id_status & SHSWIC_ID_STA_INDO_MASK) == 0x06))
	{
		if((int_status & SHSWIC_INT_STA_VBUS_MASK) == 0x10) 
		{
			SHSWIC_DEBUG_LOG_L("shswic_detect_get:USB_CABLE");
			return SHSWIC_ID_USB_CABLE;
		} else {
			SHSWIC_DEBUG_LOG_L("shswic_detect_get:NONE");
			return SHSWIC_ID_NONE;
		}
	}

#endif  /* SHSWIC_UCDCNT_USE */

	SHSWIC_DEBUG_LOG_L("shswic_detect_get:%d", shswic_detect_port);
	return shswic_detect_port;
}

void shswic_detect_init(void)
{
	int ret = 0;
	uint8_t id_status = 0x0d;
	uint8_t status = 0x00;
	uint8_t int_status = 0x00;
	uint8_t detect = SHSWIC_ID_NONE;

	SHSWIC_DEBUG_LOG_L("shswic_detect_init Start");

	wake_lock_init(&shswic_delayed_work_wake_lock_hw, WAKE_LOCK_SUSPEND, "shswic_delayed_work_lock_hw");
	wake_lock_init(&shswic_sleep_wake_lock, WAKE_LOCK_SUSPEND, "shswic_sleep_lock");

#ifdef CONFIG_SII8334_MHL_TX
	shswic_read_data.shswic_mhl_result = SHMHL_DEVICE_MAX;
#endif /* CONFIG_SII8334_MHL_TX */

	shswic_detect_cb_init();

#ifdef SHSWIC_UCDCNT_USE
	shswic_setup_ucdcnt();
#endif /* SHSWIC_UCDCNT_USE */
	ret = gpio_request(SHSWIC_GPIO_HS_DET, "swic_hs");
	if (ret != 0)
	{
		SHSWIC_DEBUG_LOG_H("Error SHSWIC_GPIO_HS_DET request");
	}

	ret = gpio_request(SHSWIC_GPIO_INT, "swic_int");
	if (ret != 0)
	{
		SHSWIC_DEBUG_LOG_H("Error SHSWIC_GPIO_INT request");
	}
	
	ret = request_irq(gpio_to_irq(SHSWIC_GPIO_INT), shswic_detect_isr, IRQF_TRIGGER_FALLING, "shswic_irq", NULL);
	if (ret != 0)
	{
		SHSWIC_DEBUG_LOG_H("Error USB-Int request_irq:Ret %d", ret);
	}
	else
	{
		shswic_interrupt_init_state = true;
		SHSWIC_DEBUG_LOG_L("Success USB-Int request_irq");
	}

	shswic_status_read(&id_status, &status, &int_status);
	shswic_status_read(&id_status, &status, &int_status);

	detect = shswic_detect_get(id_status, status, int_status);

	if(SHSWIC_ID_RECHECK != detect)
	{
		shswic_1th_check_detect_port = detect;
		shswic_send_register_recheck_signal(SHSWIC_REG_RECHECK_SIG);
	}

#ifdef CONFIG_CRADLE_SWIC
	if(int_status & 0x08)
	{
		shswic_last_cradle_status = SHSWIC_ID_CRADLE;
	}
	else
	{
		shswic_last_cradle_status = SHSWIC_ID_CRADLE_NONE;
	}

	shswic_state_cradle_proc();
#endif /* CONFIG_CRADLE_SWIC */

#ifdef CONFIG_CRADLE_SWIC
#ifdef SWIC_CRADLE_DETECT_GPIO_VCDET
	ret = request_irq(gpio_to_irq(SHSWIC_GPIO_VCDET), shswic_cradle_isr, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "shswic_irq", NULL);
	if (ret != 0)
	{
		SHSWIC_DEBUG_LOG_H("Error CRADLE-Int request_irq:Ret %d", ret);
	}
	else
	{
		shswic_interrupt_cradle_state = true;
		SHSWIC_DEBUG_LOG_L("Success CRADLE-Int request_irq");
	}
	
	
	shswic_state_cradle_proc();
#endif  /* SWIC_CRADLE_DETECT_GPIO_VCDET */
#endif /* CONFIG_CRADLE_SWIC */
	return;
}

static void shswic_detect_retry_sig_handler(void)
{

}

#ifdef CONFIG_SII8334_MHL_TX
void shswic_detect_cb_mhl(shmhl_detect_device_t device, void* user_data)
{
	SHSWIC_DEBUG_LOG_L("shswic_shmhl_detect_cb Called [device=%d]", device);

	shswic_read_data.shswic_mhl_result = device;
	shswic_send_signal(SHSWIC_DETECT_CB_MHL_SIG);
}

static void shswic_detect_cb_mhl_proc(void)
{
	shswic_detect_mhl_proc();
}
#endif /* CONFIG_SII8334_MHL_TX */

int8_t shswic_status_read(uint8_t* id_status, uint8_t* status, uint8_t* int_status)
{
	int8_t ret;
	uint8_t temp_id;
	uint8_t temp_status;
	uint8_t temp_int;

//	SHSWIC_DEBUG_LOG_L("shswic_status_read Start");

	*id_status = *status = *int_status = 0x00;
	temp_id = temp_status = temp_int = 0x00;

	ret = shswic_i2c_read(&temp_id, 1, SHSWIC_REG_ID_STA);
	if (ret == SHSWIC_SUCCESS)
	{
		ret = shswic_i2c_read(&temp_status, 1, SHSWIC_REG_STATUS);
		if (ret == SHSWIC_SUCCESS)
		{
			ret = shswic_i2c_read(&temp_int, 1, SHSWIC_REG_INT_STA);
			if (ret == SHSWIC_SUCCESS)
			{
				*id_status = temp_id;
				*status = temp_status;
				*int_status = temp_int;
			}
		}
	}

	SHSWIC_DEBUG_LOG_L("shswic_status_read ret %d, id = 0x%02x, status = 0x%02x, int = 0x%02x", ret, temp_id, temp_status, temp_int);

	if ((ret != SHSWIC_SUCCESS)
	&&  (shswic_init_flag == SHSWIC_IMPLEMENT) )
	{
		SHSWIC_DEBUG_LOG_H("shswic_status_read error %d", ret);
	}
	return ret;
}

static void shswic_detect_cb_call(uint8_t detect)
{
	SHSWIC_DEBUG_LOG_L("shswic_detect_cb_call detect = 0x%02x, shswic_detect_port = 0x%02x", detect, shswic_detect_port);
	SHSWIC_DEBUG_LOG_H("detect 0x%02x --> 0x%02x", shswic_detect_port,detect);

	shswic_pre_detect_cb_call(detect);

#ifdef CONFIG_HEADSET_BUTTON_SWIC
	if((SHSWIC_ID_HEADSET != detect)
	&& (SHSWIC_ID_HEADSET_SW != detect)
	&& (SHSWIC_ID_CRADLE != detect)
	&& (SHSWIC_ID_CRADLE_NONE != detect))
	{
		if(SHSWIC_SEND_HEADSET_SW_ON == shswic_last_send_sw)
		{
		input_report_key(shswic_input_dev, KEY_MEDIA, 0);
		input_sync(shswic_input_dev);
		shswic_last_send_sw = SHSWIC_SEND_HEADSET_SW_OFF;
		}
	}
#endif /* CONFIG_HEADSET_BUTTON_SWIC */
	switch (detect)
	{
		case SHSWIC_ID_USB_CABLE:
#ifdef CONFIG_AC_ADAPTER_SWIC
		case SHSWIC_ID_AC_ADAPTER:
#endif /* CONFIG_AC_ADAPTER_SWIC */

		case SHSWIC_ID_IRREGULAR_CHARGER:
#ifdef CONFIG_HOST_SWIC
		case SHSWIC_ID_USB_HOST_CABLE:
#endif /* CONFIG_HOST_SWIC */
#ifdef CONFIG_SII8334_MHL_TX
		case SHSWIC_ID_MHL:
#endif /* CONFIG_SII8334_MHL_TX */
			shswic_detect_port = detect;
			shswic_detect_cb_call_func(detect);
			break;

		case SHSWIC_ID_HEADSET:
			shswic_detect_port = detect;
			shswic_detect_cb_call_func(detect);
			break;

		case SHSWIC_ID_HEADSET_SW:
			shswic_detect_port = detect;
#ifdef CONFIG_HEADSET_BUTTON_SWIC
			shswic_detect_cb_call_func(detect);
#else /* CONFIG_HEADSET_BUTTON_SWIC */
			shswic_last_detect = detect;
#endif /* CONFIG_HEADSET_BUTTON_SWIC */
			break;
		case SHSWIC_ID_NONE:
			switch (shswic_detect_port)
			{
				case SHSWIC_ID_USB_CABLE:
#ifdef CONFIG_HOST_SWIC
				case SHSWIC_ID_USB_HOST_CABLE:
#endif /* CONFIG_HOST_SWIC */
#ifdef CONFIG_AC_ADAPTER_SWIC
				case SHSWIC_ID_AC_ADAPTER:
#endif /* CONFIG_AC_ADAPTER_SWIC */
				case SHSWIC_ID_IRREGULAR_CHARGER:
#ifdef CONFIG_SII8334_MHL_TX
				case SHSWIC_ID_MHL:
#endif /* CONFIG_SII8334_MHL_TX */
					shswic_detect_port = SHSWIC_ID_NONE;
					shswic_detect_cb_call_func(detect);
					break;

				case SHSWIC_ID_HEADSET:
					shswic_detect_port = SHSWIC_ID_NONE;
					shswic_detect_cb_call_func(detect);
					break;

				case SHSWIC_ID_HEADSET_SW:
					shswic_detect_port = SHSWIC_ID_NONE;
#ifdef CONFIG_HEADSET_BUTTON_SWIC
					shswic_detect_cb_call_func(detect);
#endif /* CONFIG_HEADSET_BUTTON_SWIC */
					break;

				default:
					break;
			}
			break;

#ifdef CONFIG_CRADLE_SWIC
		case SHSWIC_ID_CRADLE:
		case SHSWIC_ID_CRADLE_NONE:
			shswic_cradle_port = detect;
			shswic_detect_cb_call_func(detect);
			break;
#endif /* CONFIG_CRADLE_SWIC */

		default:
			break;
	}
	return;
}

static void shswic_detect_cb_call_func(uint8_t detect)
{
	int i;
#ifdef CONFIG_HEADSET_BUTTON_SWIC
	int switch_earphone_detect;
#endif /* CONFIG_HEADSET_BUTTON_SWIC */

	SHSWIC_DEBUG_LOG_L("shswic_detect_cb_call_func Start [detect = 0x%x]", detect);

	mutex_lock(&shswic_cb_lock);

	switch (detect)
	{
		case SHSWIC_ID_USB_CABLE:
#ifdef CONFIG_HOST_SWIC
		case SHSWIC_ID_USB_HOST_CABLE:
#endif /* CONFIG_HOST_SWIC */
#ifdef CONFIG_AC_ADAPTER_SWIC
		case SHSWIC_ID_AC_ADAPTER:
#endif /* CONFIG_AC_ADAPTER_SWIC */
		case SHSWIC_ID_IRREGULAR_CHARGER:
#ifdef CONFIG_SII8334_MHL_TX
		case SHSWIC_ID_MHL:
#endif /* CONFIG_SII8334_MHL_TX */
			for (i = 0; i < SHSWIC_DEVICE_NUM; i++)
			{
				if (!shswic_cb_func[i])
				{
					continue;
				}

				if (!(shswic_cb_irq[i] & detect))
				{
					continue;
				}
				
				if(shswic_cb_last_usb_port[i] != detect)
				{
					SHSWIC_DEBUG_LOG_H("CallBack Call[detect = 0x%x]", detect);
					shswic_cb_func[i]((uint8_t)detect, shswic_cb_userdata[i]);
					shswic_cb_last_usb_port[i] = detect;
				}
			}
			shswic_last_detect = detect;
			break;
#ifndef CONFIG_HEADSET_BUTTON_SWIC
		case SHSWIC_ID_HEADSET:
		case SHSWIC_ID_HEADSET_SW:
				for (i = 0; i < SHSWIC_DEVICE_NUM; i++)
				{
					if (!shswic_cb_func[i])
					{
						continue;
					}

					if (!(shswic_cb_irq[i] & detect))
					{
						continue;
					}
					
					if(shswic_cb_last_usb_port[i] != detect)
					{
						SHSWIC_DEBUG_LOG_H("CallBack Call[detect = 0x%x]", detect);
						shswic_cb_func[i]((uint8_t)detect, shswic_cb_userdata[i]);
						shswic_cb_last_usb_port[i] = detect;
					}
				}
			shswic_last_detect = detect;
			break;
#endif /* CONFIG_HEADSET_BUTTON_SWIC */
#ifdef CONFIG_HEADSET_BUTTON_SWIC
		case SHSWIC_ID_HEADSET:
		case SHSWIC_ID_HEADSET_SW:
			for (i = 0; i < SHSWIC_DEVICE_NUM; i++)
			{
				if (!shswic_cb_func[i])
				{
					continue;
				}

				if (!(shswic_cb_irq[i] & detect))
				{
					continue;
				}
					/* Can't judgment earphone implement swittch button,	*/
					/* if shswic_switch_earphone is false.					*/
					if((SHSWIC_CODEC_DEVICE != i)
					|| (!shswic_switch_earphone))
					{
						if(shswic_cb_last_usb_port[i] != detect)
						{
							SHSWIC_DEBUG_LOG_H("CallBack Call[detect = 0x%x]", detect);
							shswic_cb_func[i]((uint8_t)detect, shswic_cb_userdata[i]);
							shswic_cb_last_usb_port[i] = detect;
						}
					}
					else
					{
						/* SHSWIC_CODEC_DEVICE request customeize event type, 			*/
						/* it's SHSWIC_ID_HEADSET_SW_OFF or SHSWIC_ID_HEADSET_SW_ON.	*/
						switch_earphone_detect = SHSWIC_ID_HEADSET_SW_OFF;
						if(detect == SHSWIC_ID_HEADSET_SW)
						{
							switch_earphone_detect = SHSWIC_ID_HEADSET_SW_ON;
						}

						if(shswic_cb_last_usb_port[i] != switch_earphone_detect)
						{
							SHSWIC_DEBUG_LOG_H("CallBack Call[detect = 0x%x]", switch_earphone_detect);
							shswic_cb_func[i]((uint8_t)switch_earphone_detect, shswic_cb_userdata[i]);
							shswic_cb_last_usb_port[i] = switch_earphone_detect;
						}
					}
			}
			shswic_last_detect = detect;
			break;
#endif /* CONFIG_HEADSET_BUTTON_SWIC */
		case SHSWIC_ID_NONE:
			if((shswic_last_detect_state == SHSWIC_STATE_HEADSET) || (shswic_last_detect_state == SHSWIC_STATE_HEADSET_SW)){
					for (i = 0; i < SHSWIC_DEVICE_NUM; i++)
					{
						if (!shswic_cb_func[i])
						{
							continue;
						}
						
						if(shswic_cb_last_usb_port[i] != detect)
						{
							SHSWIC_DEBUG_LOG_H("CallBack Call[detect = 0x%x]", detect);
							shswic_cb_func[i]((uint8_t)detect, shswic_cb_userdata[i]);
							shswic_cb_last_usb_port[i] = detect;
						}
					}
			}else{
				for (i = 0; i < SHSWIC_DEVICE_NUM; i++)
				{
					if (!shswic_cb_func[i])
					{
						continue;
					}
					
					if(shswic_cb_last_usb_port[i] != detect)
					{
						SHSWIC_DEBUG_LOG_H("CallBack Call[detect = 0x%x]", detect);
						shswic_cb_func[i]((uint8_t)detect, shswic_cb_userdata[i]);
						shswic_cb_last_usb_port[i] = detect;
					}
				}
			}
			shswic_last_detect = detect;
			break;

#ifdef CONFIG_CRADLE_SWIC
		case SHSWIC_ID_CRADLE:
			for (i = 0; i < SHSWIC_DEVICE_NUM; i++)
			{
				if (!shswic_cb_func[i])
				{
					continue;
				}
				
				if (!(shswic_cb_irq[i] & detect))
				{
					continue;
				}
				
				if(shswic_cb_last_cradle_port[i] != detect)
				{
					SHSWIC_DEBUG_LOG_H("CallBack Call[cradle = 0x%x]", detect);
					shswic_cb_func[i]((uint8_t)detect, shswic_cb_userdata[i]);
					shswic_cb_last_cradle_port[i] = detect;
				}
			}
			break;

		case SHSWIC_ID_CRADLE_NONE:
			for (i = 0; i < SHSWIC_DEVICE_NUM; i++)
			{
				if (!shswic_cb_func[i])
				{
					continue;
				}
				
				if (!(shswic_cb_irq[i] & SHSWIC_ID_CRADLE))
				{
					continue;
				}
				
				if(shswic_cb_last_cradle_port[i] != SHSWIC_ID_NONE)
				{
					SHSWIC_DEBUG_LOG_H("CallBack Call[cradle = 0x%x]", detect);
					shswic_cb_func[i]((uint8_t)SHSWIC_ID_NONE, shswic_cb_userdata[i]);
					shswic_cb_last_cradle_port[i] = SHSWIC_ID_NONE;
				}
			}
#endif /* CONFIG_CRADLE_SWIC */

		default:
			break;
	}

	mutex_unlock(&shswic_cb_lock);
	return;
}

int8_t shswic_state_usb_cable_proc(uint8_t detect)
{
	SHSWIC_DEBUG_LOG_L("shswic_state_usb_cable_proc detect = 0x%02x", detect);

	if (detect != SHSWIC_ID_USB_CABLE)
	{
		shswic_detect_cb_call(SHSWIC_ID_NONE);

		if (detect != SHSWIC_ID_NONE)
		{
			shswic_detect_cb_call(detect);
		}
	}

	return SHSWIC_SUCCESS;
}

#ifdef CONFIG_HOST_SWIC
int8_t shswic_state_usb_host_cable_proc(uint8_t detect)
{
	SHSWIC_DEBUG_LOG_L("shswic_state_usb_host_cable_proc detect = 0x%02x", detect);

	if (detect != SHSWIC_ID_USB_HOST_CABLE)
	{
		shswic_detect_cb_call(SHSWIC_ID_NONE);

		if (detect != SHSWIC_ID_NONE)
		{
			shswic_detect_cb_call(detect);
		}
	}

	return SHSWIC_SUCCESS;
}
#endif /* CONFIG_HOST_SWIC */

#ifdef CONFIG_AC_ADAPTER_SWIC
int8_t shswic_state_ac_adapter_proc(uint8_t detect)
{
	SHSWIC_DEBUG_LOG_L("shswic_state_ac_adapter_proc detect = 0x%02x", detect);

	if (detect != SHSWIC_ID_AC_ADAPTER)
	{
		shswic_detect_cb_call(SHSWIC_ID_NONE);

		if (detect != SHSWIC_ID_NONE)
		{
			shswic_detect_cb_call(detect);
		}
	}

	return SHSWIC_SUCCESS;
}
#endif /* CONFIG_AC_ADAPTER_SWIC */

int8_t shswic_state_irregular_charger_proc(uint8_t detect)
{
	SHSWIC_DEBUG_LOG_L("shswic_state_irregular_charger_proc detect = 0x%02x", detect);

	if (detect != SHSWIC_ID_IRREGULAR_CHARGER)
	{
		shswic_detect_cb_call(SHSWIC_ID_NONE);

		if (detect != SHSWIC_ID_NONE)
		{
			shswic_detect_cb_call(detect);
		}
	}

	return SHSWIC_SUCCESS;
}

int8_t shswic_state_headset_proc(uint8_t detect)
{
	SHSWIC_DEBUG_LOG_L("shswic_state_headset_proc detect = 0x%02x", detect);
	
	if (detect == SHSWIC_ID_HEADSET)
	{
		return SHSWIC_SUCCESS;
	}
	else
	{
		if (detect == SHSWIC_ID_HEADSET_SW)
		{
			shswic_detect_cb_call(SHSWIC_ID_HEADSET_SW);
		}
		else
		{
			shswic_detect_cb_call(SHSWIC_ID_NONE);

			if (detect != SHSWIC_ID_NONE)
			{
				shswic_detect_cb_call(detect);
			}
		}
	}

	return SHSWIC_SUCCESS;
}

int8_t shswic_state_headset_sw_proc(uint8_t detect)
{
	SHSWIC_DEBUG_LOG_L("shswic_state_headset_sw_proc detect = 0x%02x", detect);

	if (detect == SHSWIC_ID_HEADSET_SW)
	{
		return SHSWIC_SUCCESS;
	}
	else
	{
		if (detect == SHSWIC_ID_HEADSET)
		{
			shswic_detect_cb_call(SHSWIC_ID_HEADSET);
		}
		else
		{
			shswic_detect_cb_call(SHSWIC_ID_NONE);

			if (detect != SHSWIC_ID_NONE)
			{
				shswic_detect_cb_call(detect);
			}
		}
	}

	return SHSWIC_SUCCESS;
}

#ifdef CONFIG_CRADLE_SWIC
void shswic_state_cradle_proc(void)
{
	SHSWIC_DEBUG_LOG_L("shswic_state_cradle_proc Start");

	switch(shswic_cradle_state())
	{
		case SHSWIC_ID_CRADLE:
			SHSWIC_DEBUG_LOG_L("shswic_state_cradle_proc Cradle-In");
			if (shswic_cradle_port != SHSWIC_ID_CRADLE)
			{
				SHSWIC_DEBUG_LOG_H("Cradle-In");
				shswic_detect_cb_call(SHSWIC_ID_CRADLE);
			}
			break;
		
		case SHSWIC_ID_CRADLE_NONE:
			SHSWIC_DEBUG_LOG_L("shswic_state_cradle_proc Cradle-None");
			if (shswic_cradle_port != SHSWIC_ID_CRADLE_NONE)
			{
				SHSWIC_DEBUG_LOG_H("Cradle-None");
				shswic_detect_cb_call(SHSWIC_ID_CRADLE_NONE);
			}
			break;
		
		case SHSWIC_ID_CRADLE_UNKNOWN:
			SHSWIC_DEBUG_LOG_L("shswic_state_cradle_proc Cradle-Unknown");
			SHSWIC_DEBUG_LOG_H("Cradle-Unknown");
			break;
		
		default:
			break;
	}

		return;
}
#endif /* CONFIG_CRADLE_SWIC */

int8_t shswic_state_none_proc(uint8_t detect)
{
	SHSWIC_DEBUG_LOG_L("shswic_state_none_proc detect = 0x%02x", detect);

	if (detect != SHSWIC_ID_NONE)
	{
		shswic_detect_cb_call(detect);
	}
	return SHSWIC_SUCCESS;
}

#ifdef CONFIG_SII8334_MHL_TX
int8_t shswic_state_mhl_proc(uint8_t detect)
{
	SHSWIC_DEBUG_LOG_L("shswic_state_mhl_proc detect = 0x%02x", detect);

	if (detect != SHSWIC_ID_MHL)
	{
		SHSWIC_DEBUG_LOG_L("MHL_DISCONNECTED");
		shmhl_notify_id_vbus_state(true, false);
		shswic_detect_cb_call(detect);
	}
	return SHSWIC_SUCCESS;
}
#endif /* CONFIG_SII8334_MHL_TX */

#ifdef CONFIG_SII8334_MHL_TX
void shswic_detect_mhl_proc(void)
{
	uint8_t detect_port = SHSWIC_ID_NONE;
	uint8_t sw_ctl = 0x00;
	uint8_t id_det = 0x00;
	uint8_t ucdcnt = 0x44;
	shswic_result_t retUcdcnt = SHSWIC_FAILURE;

	SHSWIC_DEBUG_LOG_L("shswic_detect_mhl_proc Called [device=%d]", shswic_read_data.shswic_mhl_result);

	shswic_detect_id = (shswic_read_data.shswic_id_status & SHSWIC_ID_STA_MASK);

	switch(shswic_read_data.shswic_mhl_result)
	{
		case SHMHL_DEVICE_MHL:
			detect_port = SHSWIC_ID_MHL;
			shswic_detect_state = SHSWIC_STATE_MHL;
			SHSWIC_DEBUG_LOG_L("shswic_detect_mhl_proc:MHL");
			break;
		case SHMHL_DEVICE_USB_B:
			if((shswic_read_data.shswic_status & SHSWIC_STATUS_DCDFAIL_MASK) == 0x00)
			{
				detect_port = shswic_detect_get_without_mhl(shswic_read_data.shswic_id_status, shswic_read_data.shswic_status, shswic_read_data.shswic_int_status);
			}
			else
			{
				detect_port = shswic_detect_port;
			}
			shswic_i2c_write(&sw_ctl, 1, SHSWIC_REG_MUXSW_CONTROL);
			SHSWIC_DEBUG_LOG_L("shswic_cb_detect_mhl_proc:SHMHL_DEVICE_USB_B");
			break;
		case SHMHL_DEVICE_UNKNOWN:
		case SHMHL_DEVICE_ACA_MCPC_CHG:
		case SHMHL_DEVICE_USB_A_CHG:
		case SHMHL_DEVICE_STANDBY:
		case SHMHL_DEVICE_USB_B_ACA:
		case SHMHL_DEVICE_MCPC_ACA_OPN:
			detect_port = shswic_detect_port;
			shswic_i2c_write(&sw_ctl, 1, SHSWIC_REG_MUXSW_CONTROL);
			SHSWIC_DEBUG_LOG_L("shswic_detect_mhl_proc:OTHER");
			break;
		case SHMHL_DEVICE_NONE:
			shswic_i2c_write(&sw_ctl, 1, SHSWIC_REG_MUXSW_CONTROL);

			wake_lock(&shswic_sleep_wake_lock);
			usleep(30000);
			wake_unlock(&shswic_sleep_wake_lock);

			/* ADCRETRY */
			shswic_i2c_read(&id_det, 1, SHSWIC_REG_IDDET);
			id_det &= ~0x02;
			shswic_i2c_write(&id_det, 1, SHSWIC_REG_IDDET);

			/* BCSRETRY */
			retUcdcnt = shswic_i2c_read(&ucdcnt, 1, SHSWIC_REG_UCDCNT);
			if (retUcdcnt == SHSWIC_SUCCESS)
			{
				ucdcnt &= ~0x01;
				shswic_i2c_write(&ucdcnt, 1, SHSWIC_REG_UCDCNT);
			}

			wake_lock(&shswic_sleep_wake_lock);
			usleep(30000);
			wake_unlock(&shswic_sleep_wake_lock);

			/* ADCRETRY */
			id_det |= 0x12;
			shswic_i2c_write(&id_det, 1, SHSWIC_REG_IDDET);

			/* BCSRETRY */
			if (retUcdcnt == SHSWIC_SUCCESS)
			{
				ucdcnt |= 0x01;
				shswic_i2c_write(&ucdcnt, 1, SHSWIC_REG_UCDCNT);
			}

			SHSWIC_DEBUG_LOG_L("shswic_cb_detect_mhl_proc:NONE");
			return;
		case SHMHL_DEVICE_MAX:
			/* request MHL chipset check, now. */
			SHSWIC_DEBUG_LOG_L("shswic_cb_detect_mhl_proc:SHMHL_DEVICE_MAX");
			break;
		case SHMHL_DEVICE_DISC_RETRY_OVER:
			shswic_i2c_write(&sw_ctl, 1, SHSWIC_REG_MUXSW_CONTROL);
			SHSWIC_DEBUG_LOG_L("shswic_cb_detect_mhl_proc:SHMHL_DEVICE_DISC_RETRY_OVER");
			break;
		default:
			shswic_i2c_write(&sw_ctl, 1, SHSWIC_REG_MUXSW_CONTROL);
			SHSWIC_DEBUG_LOG_L("shswic_cb_detect_mhl_proc:default");
			break;
	}

	shswic_state_proc[shswic_last_detect_state](detect_port);
	shswic_last_detect_state = shswic_detect_state;
//	shswic_last_int_status = shswic_read_data.shswic_int_status;
}

static void shswic_call_mhl_func(uint8_t id_status, uint8_t status, uint8_t int_status)
{
	uint8_t sw_ctl = SHSWIC_MHL_PASS;
	/* idrdet flg 1 : id line low */
	/*            0 : id line hi  */
	bool id_is_hi = ((id_status & SHSWIC_ID_STA_IDRDET_MASK) == 0x00);
	/* vbusdet flg 1 : vbus line hi  */
	/*             0 : vbus line low */
	bool vbas_is_hi = ((int_status & SHSWIC_INT_STA_VBUS_MASK) != 0x00);

	shswic_read_data.shswic_id_status = id_status;
	shswic_read_data.shswic_status = status;
	shswic_read_data.shswic_int_status = int_status;
	shswic_read_data.shswic_mhl_result = SHMHL_DEVICE_MAX;

	shswic_i2c_write(&sw_ctl, 1, SHSWIC_REG_MUXSW_CONTROL);
	
	if(((int_status & SHSWIC_INT_STA_CHGDET_MASK) == 0x00)
		&& ((status & SHSWIC_STATUS_CHGPORT_MASK) == 0x20)
		&& ((id_status & SHSWIC_ID_STA_IDRDET_MASK) == 0x00)) {
		SHSWIC_DEBUG_LOG_L("usb-pc charging start.");
		shmhl_notify_usb_pc_charge(true);
	}
	shmhl_detect_cb_regist(shswic_detect_cb_mhl, NULL);
	shmhl_notify_id_vbus_state(id_is_hi, vbas_is_hi);
}
#endif /* CONFIG_SII8334_MHL_TX */


#ifdef CONFIG_CRADLE_SWIC
int shswic_cradle_state(void)
{
	return shswic_last_cradle_status;
}
#endif /* CONFIG_CRADLE_SWIC */

static bool shswic_is_connected_headset(uint8_t int_status, uint8_t id_status)
{
	if((int_status & SHSWIC_INT_STA_USBPORT_MASK) != 0x00)
	{
		/* not headset */
		SHSWIC_DEBUG_LOG_L("not headset: int_status != 0x00");
		return false;
	}

	return shswic_is_headset_indo(id_status);
}

static bool shswic_is_connected_switch_headset(uint8_t int_status, uint8_t id_status)
{
	if((id_status & SHSWIC_ID_STA_INDO_MASK) == 0x02)
	{
		/* push headset button */
		return true;
	}

	if((int_status & SHSWIC_INT_STA_USBPORT_MASK) != 0x20)
	{
		/* no COMPL flg */
		SHSWIC_DEBUG_LOG_L("not COMPL flag: int_status != 0x20");
		return false;
	}

	return shswic_is_headset_indo(id_status);
}

static bool shswic_is_headset_indo(uint8_t id_status)
{
	if((id_status & SHSWIC_ID_STA_INDO_MASK) == 0x0C)
	{
		/* mono */
		return true;
	}

	if((id_status & SHSWIC_ID_STA_INDO_MASK) == 0x08)
	{
		/* stereo */
		return true;
	}
	
	return false;
}

bool shswic_is_push_headset_button(uint8_t int_status)
{
	return ((int_status & SHSWIC_INT_STA_USBPORT_MASK) == 0x20);
}

static void shswic_set_shswic_cb_init_state(int state)
{
	atomic_set(&shswic_cb_init_state, state);
	return;
}

static int shswic_get_shswic_cb_init_state(void)
{
	return (atomic_read(&shswic_cb_init_state));
}

static void shswic_set_delay_cancel_sig(int sig)
{
	int l_sig = atomic_read(&shswic_delay_cancel_sig);
	l_sig |= sig;
	atomic_set(&shswic_delay_cancel_sig, l_sig);
	return;
}

static void shswic_clear_delay_cancel_sig(int sig)
{
	int l_sig = atomic_read(&shswic_delay_cancel_sig);
	l_sig &= ~sig;
	atomic_set(&shswic_delay_cancel_sig, l_sig);
	return;
}

static int shswic_get_delay_cancel_sig(void)
{
	return (atomic_read(&shswic_delay_cancel_sig));
}

static shswic_result_t shswic_diag_get_id_data(uint8_t* id_data)
{
	shswic_result_t result = SHSWIC_FAILURE;

	if (id_data == NULL)
	{
		SHSWIC_DEBUG_LOG_H("shswic_diag_get_device_id SHSWIC_PARAM_ERROR");
		return SHSWIC_PARAM_ERROR;
	}

	if (shswic_init_flag == SHSWIC_IMPLEMENT)
	{
		result = shswic_i2c_read(id_data, 1, SHSWIC_REG_DEVICE_ID);
		SHSWIC_DEBUG_LOG_L("shswic_diag_get_device_id = %d", *id_data);
	}
	return result;
}

static int shswic_diag_open(struct inode *inode, struct file *file)
{
	SHSWIC_DEBUG_LOG_L();
	return SHSWIC_SUCCESS;
}

static int shswic_diag_release(struct inode *inode, struct file *file)
{
	SHSWIC_DEBUG_LOG_L();
	return SHSWIC_SUCCESS;
}

static ssize_t shswic_diag_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	uint8_t id = 0x00;

	SHSWIC_DEBUG_LOG_L();

	shswic_diag_get_id_data(&id);

	if (id != 0x00)
	{
		if (copy_to_user(buf, &id, sizeof(id)))
		{
			return 0;
		}
	}
	else
	{
		return 0;
	}
	return 1;
}

static long shswic_diag_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	uint8_t detect_id;

	SHSWIC_DEBUG_LOG_L();

	if (argp == NULL)
	{
		SHSWIC_DEBUG_LOG_H("SHSWIC_PARAM_ERROR");
		return -EIO;
	}

	switch (cmd)
	{
		case SHSWIC_IOCTL_ID_READ:
			SHSWIC_DEBUG_LOG_L("SHSWIC_IOCTL_ID_READ");
			detect_id = shswic_detect_id;
			if (copy_to_user(argp, &detect_id, sizeof(detect_id)))
			{
				SHSWIC_DEBUG_LOG_H("copy_to_user Error");
				return -EFAULT;
			}
			break;

		default:
			break;
	}

	return SHSWIC_SUCCESS;
}

shswic_result_t shswic_i2c_write(uint8_t* write_buf, uint8_t len, uint8_t addr)
{
	int ret;
	uint8_t buf[2];
	struct i2c_msg msg;

	if (shswic_i2c_client == NULL)
	{
		SHSWIC_DEBUG_LOG_H("shswic_i2c_write errer null");
		return SHSWIC_PARAM_ERROR;
	}
	SHSWIC_DEBUG_LOG_I2C("shswic_i2c_write I2CAddr = 0x%x, RegAddr = 0x%x, Data = 0x%x", shswic_i2c_client->addr, addr, *write_buf);

	msg.addr  = shswic_i2c_client->addr,
	msg.flags = 0,
	msg.buf   = buf,
	msg.len   = 2,

	buf[0] = addr;
	buf[1] = *write_buf;

	ret = i2c_transfer(shswic_i2c_client->adapter, &msg, 1);

	if (ret < 0)
	{
		SHSWIC_DEBUG_LOG_H("shswic_i2c_write errer = %d", ret);
		return SHSWIC_FAILURE;
	}
	SHSWIC_DEBUG_LOG_I2C("shswic_i2c_write Success");

	return SHSWIC_SUCCESS;
}

shswic_result_t shswic_i2c_read(uint8_t* read_buf, uint8_t len, uint8_t addr)
{
	int ret;
	uint8_t buf[2];
	struct i2c_msg msg[2];

	if (shswic_i2c_client == NULL)
	{
		SHSWIC_DEBUG_LOG_H("shswic_i2c_read errer null");
		return SHSWIC_PARAM_ERROR;
	}

	SHSWIC_DEBUG_LOG_I2C("shswic_i2c_read I2CAddr = 0x%x, RegAddr = 0x%x", shswic_i2c_client->addr, addr);

	msg[0].addr  = shswic_i2c_client->addr,
	msg[0].flags = 0,
	msg[0].buf   = buf,
	msg[0].len   = 1,

	msg[1].addr  = shswic_i2c_client->addr,
	msg[1].flags = I2C_M_RD,
	msg[1].buf   = read_buf,
	msg[1].len   = len,

	buf[0] = addr;

	ret = i2c_transfer(shswic_i2c_client->adapter, msg, 2);
	if (ret < 0)
	{
		if(shswic_init_flag == SHSWIC_IMPLEMENT)
		{
			SHSWIC_DEBUG_LOG_H("shswic_i2c_read errer = %d", ret);
		}
		return SHSWIC_FAILURE;
	}
	SHSWIC_DEBUG_LOG_I2C("shswic_i2c_read Success Data = 0x%x", *read_buf);

	return SHSWIC_SUCCESS;
}

#ifdef CONFIG_ANDROID_ENGINEERING
void test_func(uint8_t device, void* user_data)
{
	printk("******[SWIC][TEST] test_func device:%d*****\n", device);
}

static ssize_t shswic_dev_store(struct device *dev, struct device_attribute *attr, 
			const char *buf, size_t count)
{
	uint32_t cb_irq = (SHSWIC_ID_USB_CABLE | SHSWIC_ID_AC_ADAPTER | SHSWIC_ID_HEADSET
					| SHSWIC_ID_HEADSET_SW | SHSWIC_ID_HEADSET_SW_ON | SHSWIC_ID_HEADSET_SW_OFF
					| SHSWIC_ID_CRADLE | SHSWIC_ID_CRADLE_NONE | SHSWIC_ID_CRADLE_UNKNOWN | SHSWIC_ID_NONE );
	
	shswic_detect_cb_regist(SHSWIC_VBUS_DEVICE, cb_irq, (void*)test_func, NULL);
	
	return strlen(buf);
}

static DEVICE_ATTR( shswic_dev, ( S_IWUSR | S_IWGRP ) , NULL, shswic_dev_store );
#endif /* CONFIG_ANDROID_ENGINEERING */

static int shswic_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	SHSWIC_DEBUG_LOG_L("shswic_i2c_probe Start");
	shswic_i2c_client = client;

#ifdef CONFIG_ANDROID_ENGINEERING
	device_create_file(&client->dev, &dev_attr_shswic_dev);
#endif /* #ifdef CONFIG_ANDROID_ENGINEERING */

	return SHSWIC_SUCCESS;
}

static int shswic_i2c_remove(struct i2c_client *client)
{
	SHSWIC_DEBUG_LOG_L("shswic_i2c_remove Start");
#ifdef CONFIG_ANDROID_ENGINEERING
	device_remove_file(&client->dev, &dev_attr_shswic_dev);
#endif /* #ifdef CONFIG_ANDROID_ENGINEERING */
	return SHSWIC_SUCCESS;
}

static int shswic_pm_driver_probe(struct platform_device *pdev)
{
	SHSWIC_DEBUG_LOG_L("Start");
#ifdef CONFIG_SII8334_MHL_TX
	g_swic_regulator_dev = &pdev->dev;
#endif /* CONFIG_SII8334_MHL_TX */
	return SHSWIC_SUCCESS;
}

#ifdef CONFIG_SII8334_MHL_TX
int shswic_regulator_power(bool on)
{
	static bool swic_power = false;
	static struct regulator *swic_regulator = NULL;

	SHSWIC_DEBUG_LOG_L("Start(%d)", on);

	if (g_swic_regulator_dev == NULL)
	{
		SHSWIC_DEBUG_LOG_H("shswic_regulator_power NULL");
		return SHSWIC_REGULATOR_ERROR_PARAM;
	}

	if (swic_power == on)
	{
		return SHSWIC_REGULATOR_SUCCESS;
	}

	if (on == true)
	{
		if (swic_regulator == NULL)
		{
			swic_regulator = regulator_get(g_swic_regulator_dev, "vbus_shswic");
			if (IS_ERR(swic_regulator))
			{
				SHSWIC_DEBUG_LOG_H("shswic_regulator_power get errer");
				return SHSWIC_REGULATOR_ERROR_GET;
			}
		}

		if (regulator_enable(swic_regulator))
		{
			SHSWIC_DEBUG_LOG_H("shswic_regulator_power ON enable errer");
			regulator_put(swic_regulator);
			swic_regulator = NULL;
			return SHSWIC_REGULATOR_ERROR_ENABLE;
		}
		swic_power = true;
		SHSWIC_DEBUG_LOG_L("shswic_regulator_power ON Success");
	}
	else
	{
		if (swic_regulator == NULL)
		{
			return SHSWIC_REGULATOR_ERROR_STATE;
		}

		if (regulator_disable(swic_regulator))
		{
			SHSWIC_DEBUG_LOG_H("shswic_regulator_power OFF disable errer");
			return SHSWIC_REGULATOR_ERROR_DISABLE;
		}
		regulator_put(swic_regulator);
		swic_regulator = NULL;

		swic_power = false;
		SHSWIC_DEBUG_LOG_L("shswic_regulator_power OFF Success");
	}

	return SHSWIC_REGULATOR_SUCCESS;
}
#endif /* CONFIG_SII8334_MHL_TX */

#ifdef CONFIG_HEADSET_BUTTON_SWIC
void shswic_send_headset_sw_button(uint8_t int_status)
{
	bool is_push_headset_button = false;
	
	if(!shswic_input_dev)
	{
		SHSWIC_DEBUG_LOG_L("shswic_send_headset_sw_button shswic_input_dev[NULL]");
		return;
	}

	is_push_headset_button = shswic_is_push_headset_button(int_status);
	
	if((is_push_headset_button)
	&& (SHSWIC_SEND_HEADSET_SW_OFF == shswic_last_send_sw))
	{
		input_report_key(shswic_input_dev, KEY_MEDIA, 1);
		input_sync(shswic_input_dev);
		shswic_last_send_sw = SHSWIC_SEND_HEADSET_SW_ON;
		SHSWIC_DEBUG_LOG_L("shswic_send_headset_sw_button input_report_key[1]");
	}
	else if((!is_push_headset_button)
	&& (SHSWIC_SEND_HEADSET_SW_ON == shswic_last_send_sw))
	{
		input_report_key(shswic_input_dev, KEY_MEDIA, 0);
		input_sync(shswic_input_dev);
		shswic_last_send_sw = SHSWIC_SEND_HEADSET_SW_OFF;
		SHSWIC_DEBUG_LOG_L("shswic_send_headset_sw_button input_report_key[0]");
	}
}

static void shswic_register_input_dev(void)
{
	int ret = 0;
	if(!shswic_input_dev)
	{
		shswic_input_dev = input_allocate_device();
		if(shswic_input_dev)
		{
			shswic_input_dev->name = SH_SWIC_KEY_DEVICE_NAME;
			shswic_input_dev->id.vendor	= 0x0001;
			shswic_input_dev->id.product	= 1;
			shswic_input_dev->id.version	= 1;

			input_set_capability(shswic_input_dev, EV_KEY, KEY_MEDIA);
			
			ret = input_register_device(shswic_input_dev);
			if(ret)
			{
				SHSWIC_DEBUG_LOG_L("%s ret[%d]", __func__, ret);
				input_free_device(shswic_input_dev);
				shswic_input_dev = NULL;
			}
		}
		else
		{
			SHSWIC_DEBUG_LOG_H("%s FAILED input_allocate_device", __func__);
		}
	}
	else
	{
		SHSWIC_DEBUG_LOG_L("%s Allready registered", __func__);
	}
}

static void shswic_release_input_dev(void)
{
	if(!shswic_input_dev)
	{
		SHSWIC_DEBUG_LOG_L("shswic_release_input_dev shswic_input_dev[NULL]");
		return;
	}

	input_unregister_device(shswic_input_dev);
	input_free_device(shswic_input_dev);
	shswic_input_dev = NULL;
}
#endif /* CONFIG_HEADSET_BUTTON_SWIC */

int shswic_is_implement(void)
{
	return shswic_init_flag;
}

void shswic_pre_do_exit(void)
{
	if(shswic_interrupt_init_state)
	{
		shswic_interrupt_init_state = false;
		disable_irq_nosync(gpio_to_irq(SHSWIC_GPIO_INT));
		free_irq(gpio_to_irq(SHSWIC_GPIO_INT), 0);
	}

	gpio_free(SHSWIC_GPIO_INT);

	gpio_free(SHSWIC_GPIO_HS_DET);

#ifdef CONFIG_CRADLE_SWIC
#ifdef SWIC_CRADLE_DETECT_GPIO_VCDET
	if(shswic_interrupt_cradle_state)
	{
		shswic_interrupt_cradle_state = false;
		disable_irq_nosync(gpio_to_irq(SHSWIC_GPIO_VCDET));
		free_irq(gpio_to_irq(SHSWIC_GPIO_VCDET), 0);
	}

	gpio_free(SHSWIC_GPIO_VCDET);
#endif  /* SWIC_CRADLE_DETECT_GPIO_VCDET */
#endif  /* CONFIG_CRADLE_SWIC */
}

static void shswic_do_exit(void)
{
	shswic_pre_do_exit();

	misc_deregister(&shswic_diag_device);

	i2c_del_driver(&shswic_i2c_driver);

	platform_driver_unregister(&shswic_pm_driver);

#ifdef CONFIG_HEADSET_BUTTON_SWIC
	shswic_release_input_dev();
#endif /* CONFIG_HEADSET_BUTTON_SWIC */

	return;
}

static int __init shswic_init(void)
{
	int err = -ENODEV;

	SHSWIC_DEBUG_LOG_L("shswic_init");

	shswic_wq = create_singlethread_workqueue(SHSWIC_DRIVER_NAME);
	if (!shswic_wq)
	{
		SHSWIC_DEBUG_LOG_H("create_singlethread_workqueue ERROR");
		shswic_init_flag = SHSWIC_NOT_IMPLEMENT;
		SHSWIC_DEBUG_LOG_L("SHSWIC_NOT_IMPLEMENT");
		return -1;
	}

	mutex_init(&shswic_task_lock);

	wake_lock_init(&shswic_delayed_work_wake_lock, WAKE_LOCK_SUSPEND, "shswic_delayed_work_lock");
	wake_lock_init(&shswic_suspend_work_wake_lock, WAKE_LOCK_SUSPEND, "shswic_suspend_work_lock");

	shswic_signal_pkt = (shswic_cmd_t*)kzalloc(sizeof(shswic_cmd_t) * SHSWIC_SIGNAL_PKT_COUNTS, GFP_KERNEL);
	if(!shswic_signal_pkt)
	{
		SHSWIC_DEBUG_LOG_H("shswic_signal_pkt is NULL");
		shswic_init_flag = SHSWIC_NOT_IMPLEMENT;
		return -1;
	}
	shswic_signal_pkt_use_bitflg = 0;
	
	shswic_timer_signal_pkt = (shswic_delay_cmd_t*)kzalloc(sizeof(shswic_delay_cmd_t) * SHSWIC_TIMER_SIGNAL_PKT_COUNTS, GFP_KERNEL);
	if(!shswic_timer_signal_pkt)
	{
		SHSWIC_DEBUG_LOG_H("shswic_timer_signal_pkt is NULL");
		kfree(shswic_signal_pkt);
		shswic_signal_pkt = NULL;

		shswic_init_flag = SHSWIC_NOT_IMPLEMENT;
		return -1;
	}
	shswic_timer_signal_pkt_use_bitflg = 0;

	err = misc_register(&shswic_diag_device);
	if (err)
	{
		SHSWIC_DEBUG_LOG_H("shswic_diag_device: register failed");
		misc_deregister(&shswic_diag_device);
	}

	SHSWIC_DEBUG_LOG_L("i2c_add_driver");
	err = i2c_add_driver(&shswic_i2c_driver);
	if (err < 0)
	{
		SHSWIC_DEBUG_LOG_H("i2c_add_driver: err %d", err);
		i2c_del_driver(&shswic_i2c_driver);
	}

	err = platform_driver_register(&shswic_pm_driver);
	if (err < 0)
	{
		SHSWIC_DEBUG_LOG_H("platform_driver_register: err %d", err);
		platform_driver_unregister(&shswic_pm_driver);
	}

#ifdef CONFIG_HEADSET_BUTTON_SWIC
	shswic_register_input_dev();
#endif /* CONFIG_HEADSET_BUTTON_SWIC */

	shswic_send_signal(SHSWIC_INIT_SIG);

	return 0;
}

static void __exit shswic_exit(void)
{
	shswic_do_exit();

	if(shswic_signal_pkt)
	{
		kfree(shswic_signal_pkt);
		shswic_signal_pkt = NULL;
		shswic_signal_pkt_use_bitflg = 0;
	}

	if(shswic_timer_signal_pkt)
	{
		kfree(shswic_timer_signal_pkt);
		shswic_timer_signal_pkt = NULL;
		shswic_timer_signal_pkt_use_bitflg = 0;
	}
	
	if(shswic_wq)
	{
		destroy_workqueue(shswic_wq);
	}
}

static int shswic_pm_suspend(struct device *dev)
{
    mutex_lock(&shswic_task_lock);

    shswic_suspend_state = 1;

    disable_irq_nosync(gpio_to_irq(SHSWIC_GPIO_INT));
    disable_irq_nosync(gpio_to_irq(SHSWIC_GPIO_VCDET));
    enable_irq_wake(gpio_to_irq(SHSWIC_GPIO_INT));
    enable_irq_wake(gpio_to_irq(SHSWIC_GPIO_VCDET));

    mutex_unlock(&shswic_task_lock);

    return 0;
}

static int shswic_pm_resume(struct device *dev)
{
    mutex_lock(&shswic_task_lock);

    shswic_suspend_state = 0;

    disable_irq_wake(gpio_to_irq(SHSWIC_GPIO_INT));
    disable_irq_wake(gpio_to_irq(SHSWIC_GPIO_VCDET));
    enable_irq(gpio_to_irq(SHSWIC_GPIO_INT));
    enable_irq(gpio_to_irq(SHSWIC_GPIO_VCDET));

    mutex_unlock(&shswic_task_lock);

    return 0;
}

static int __devexit shswic_pm_remove( struct platform_device* dev_p )
{
	return 0;
}

module_init(shswic_init);
module_exit(shswic_exit);

MODULE_DESCRIPTION("SH Swic Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.0");
