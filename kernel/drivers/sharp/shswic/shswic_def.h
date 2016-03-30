/* kernel/include/sharp/shswic_def.h  (SwitchingIC Driver)
 *
 * Copyright (C) 2014 SHARP CORPORATION
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

 
#ifndef _SHSWIC_DEF_
#define _SHSWIC_DEF_

#include <mach/gpio.h>
#include <linux/mfd/pm8xxx/pm8921.h>
#include <mach/irqs.h>

#define SWIC_CRADLE_DETECT_GPIO_VCDET
// #define SHSWIC_UCDCNT_USE

#define PM8921_GPIO_BASE		NR_GPIO_IRQS
#define PM8921_GPIO_PM_TO_SYS(pm_gpio)  (pm_gpio - 1 + PM8921_GPIO_BASE)
#define PM8921_IRQ_BASE (NR_MSM_IRQS + NR_GPIO_IRQS)
#define PM8921_GPIO_PM_TO_SYS_IRQ(pm_gpio)  (pm_gpio - 1 + NR_PM8921_IRQS)

/* Signal */
#define SHSWIC_NO_SIG					0x00000000
#define SHSWIC_DETECT_ISR_SIG			0x00000001
#define SHSWIC_INIT_SIG					0x00000002
#define SHSWIC_HS_TYPE_POLL_SIG			0x00000004
#define SHSWIC_HSSW_POLL_SIG			0x00000008
#define SHSWIC_HS_POLL_SIG				0x00000010
#define SHSWIC_DETECT_RETRY_SIG			0x00000020
#define SHSWIC_DETECT_INIT_SIG			0x00000040
#ifdef CONFIG_CRADLE_SWIC
#define SHSWIC_CRADLE_SIG				0x00000080
#endif /* CONFIG_CRADLE_SWIC */
#ifdef CONFIG_SII8334_MHL_TX
#define SHSWIC_DETECT_CB_MHL_SIG		0x00000100
#endif /* CONFIG_SII8334_MHL_TX */
#define SHSWIC_CALLBACK_SIG				0x00000200
#define SHSWIC_REG_RECHECK_SIG			0x00000400
#define SHSWIC_REG_RECHECK_HEADSET_SIG	0x00000800
#define SHSWIC_CRADLE_CALLBACK_SIG		0x00001000
#define SHSWIC_HEADSET_INFORM_SWOFF_SIG		0x00004000

/* Register Addr */
#define SHSWIC_REG_DEVICE_ID			0x00
#define SHSWIC_REG_IDDET				0x01
#define SHSWIC_REG_UCDCNT				0x02
#define SHSWIC_REG_MUXSW_CONTROL		0x03
#define SHSWIC_REG_VBSW_CONTROL			0x04
#define SHSWIC_REG_INT_STA				0x05
#define SHSWIC_REG_STATUS				0x06
#define SHSWIC_REG_ID_STA				0x07

/* Retry */
#define SHSWIC_INIT_RETRY_CNT			10
#define SHSWIC_DETECT_RETRY_CNT			1
#define SHSWIC_INIT_RETRY_TIME			100
#define SHSWIC_DETECT_DELAY				250
#define SHSWIC_DETECT_RETRY_TIME		500
#define SHSWIC_CALLBACK_RETRY_TIME		1000
#define SHSWIC_DCDFAIL_RETRY_CNT_MAX	10

/* Hard */
#define SHSWIC_GPIO_INT					(109)
#ifdef CONFIG_CRADLE_SWIC
#define SHSWIC_GPIO_VCDET				(67)
#endif /* CONFIG_CRADLE_SWIC */
#define SHSWIC_GPIO_HS_DET				(22)
#define SHSWIC_GPIO_HIGH				1
#define SHSWIC_GPIO_LOW					0

/* SHSWIC_REG_MUXSW_CONTROL set parameter */
#define SHSWIC_USB_PASS					0x00
#define SHSWIC_HEADSET_PASS_MONO		0x97	/* (0x80)VBUS  :MICOUT Connect	*/
												/* (0x10)DPRXR :EARR Connect	*/
												/* (0x07)DMTXL :None			*/
#define SHSWIC_HEADSET_PASS				0x92	/* (0x80)VBUS  :MICOUT Connect	*/
												/* (0x10)DPRXR :EARR Connect	*/
												/* (0x02)DMTXL :EARL Connect	*/
#define SHSWIC_NONE_PASS				0x3F	/* (0x00)VBUS  :Open			*/
												/* (0x38)DPRXR :None			*/
												/* (0x07)DMTXL :None			*/

#ifdef CONFIG_SII8334_MHL_TX
enum
{
	SHSWIC_REGULATOR_SUCCESS = 0,
	SHSWIC_REGULATOR_ERROR_PARAM,
	SHSWIC_REGULATOR_ERROR_STATE,
	SHSWIC_REGULATOR_ERROR_GET,
	SHSWIC_REGULATOR_ERROR_ENABLE,
	SHSWIC_REGULATOR_ERROR_DISABLE,
};
#endif /* CONFIG_SII8334_MHL_TX */

enum
{
	SHSWIC_STATE_USB_CABLE = 0,
#ifdef CONFIG_HOST_SWIC
	SHSWIC_STATE_USB_HOST_CABLE,
#endif /* CONFIG_HOST_SWIC */
#ifdef CONFIG_AC_ADAPTER_SWIC
	SHSWIC_STATE_AC_ADAPTER,
#endif /* CONFIG_AC_ADAPTER_SWIC */
#ifdef CONFIG_IRREGULAR_CHARGER_SWIC
	SHSWIC_STATE_IRREGULAR_CHARGER,
#endif /* CONFIG_IRREGULAR_CHARGER_SWIC */
	SHSWIC_STATE_HEADSET,
	SHSWIC_STATE_HEADSET_SW,
	SHSWIC_STATE_NONE,
#ifdef CONFIG_SII8334_MHL_TX
	SHSWIC_STATE_MHL,
#endif /* CONFIG_SII8334_MHL_TX */
};

#ifdef CONFIG_SII8334_MHL_TX
typedef struct shswic_read_status
{
	uint8_t			shswic_id_status;
	uint8_t			shswic_status;
	uint8_t			shswic_int_status;
	shmhl_detect_device_t	shswic_mhl_result;
}shswic_read_status_t;
#endif /* CONFIG_SII8334_MHL_TX */

#endif /* _SHSWIC_DEF_ */
