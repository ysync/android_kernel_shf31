/* kernel/include/sharp/shswic.h  (SwitchingIC Driver)
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

 
#ifndef _SHSWIC_KERL_
#define _SHSWIC_KERL_

#define SHSWIC_IO					0xA2
#define SHSWIC_IOCTL_ID_READ		_IO(SHSWIC_IO, 0x01)

typedef enum
{
	SHSWIC_SUCCESS =		0,
	SHSWIC_FAILURE =		-1,
	SHSWIC_PARAM_ERROR =	-2,
}shswic_result_t;

enum
{
	SHSWIC_VBUS_DEVICE =	0,
	SHSWIC_OFFCHG_DEVICE,
	SHSWIC_CHG_DEVICE,
	SHSWIC_CODEC_DEVICE,
	SHSWIC_DEVICE_NUM,
};

enum
{
	SHSWIC_VBSW_AUTO =	0,
	SHSWIC_VBSW_OFF,
};

enum
{
	SHSWIC_ID_USB_CABLE			= 0x01,
	SHSWIC_ID_USB_HOST_CABLE	= 0x02,
	SHSWIC_ID_AC_ADAPTER		= 0x04,
	SHSWIC_ID_HEADSET			= 0x08,
	SHSWIC_ID_HEADSET_SW		= 0x10,
	SHSWIC_ID_HEADSET_SW_ON		= 0x11,		/* for SHSWIC_CODEC_DEVICE */
	SHSWIC_ID_HEADSET_SW_OFF	= 0x12,		/* for SHSWIC_CODEC_DEVICE */
	SHSWIC_ID_CRADLE			= 0x20,
	SHSWIC_ID_CRADLE_NONE		= 0x21,		/* Internal Define */
	SHSWIC_ID_CRADLE_UNKNOWN	= 0x22,		/* shswic_get_cradle_status only */
	SHSWIC_ID_NONE				= 0x40,
	SHSWIC_ID_RECHECK			= 0x41,		/* Internal Define */
	SHSWIC_ID_MHL				= 0x80,
	SHSWIC_ID_IRREGULAR_CHARGER	= 0x81,
};

enum
{
	SHSWIC_NOT_IMPLEMENT,
	SHSWIC_IMPLEMENT,
	SHSWIC_INIT,
};

typedef enum
{
	SHSWIC_SWICDETECT_HEADSET,
	SHSWIC_SWICDETECT_NONE,
}shswic_headset_result_t;

enum
{
	SHSWIC_AUDIODETECT_HEADSET,
	SHSWIC_AUDIODETECT_NONE,
};

shswic_result_t shswic_detect_cb_regist(uint8_t cb_dev, uint32_t cb_irq, void* cb_func, void* user_data );
shswic_result_t shswic_get_usb_port_status( uint8_t* device );
shswic_result_t shswic_get_cradle_status( uint8_t* device );
shswic_result_t shswic_write_vbsw_reg( uint8_t vbsw_ctl );
shswic_result_t shswic_read_vbsw_reg( uint8_t* vbsw_ctl );

int shswic_is_implement(void);
shswic_headset_result_t shswic_headset_inform_state(uint8_t detect);

#endif /* _SHSWIC_KERL_ */
