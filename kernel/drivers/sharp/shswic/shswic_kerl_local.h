/* kernel/drivers/sharp/shswic_kerl_local.h  (SwitchingIC Driver)
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

/* ____________________________________________________________________________________________________________________________ */

#ifndef _SHSWIC_KERL_LOCAL_
#define _SHSWIC_KERL_LOCAL_

#include "shswic_def.h"

extern int8_t (* shswic_state_proc[])(uint8_t detect);
extern uint8_t					shswic_detect_state;
extern uint8_t					shswic_detect_id;
extern uint8_t					shswic_detect_port;
#ifdef CONFIG_SII8334_MHL_TX
extern shswic_read_status_t			shswic_read_data;
#endif /* CONFIG_SII8334_MHL_TX */
#ifdef CONFIG_HEADSET_BUTTON_SWIC
extern bool						shswic_switch_earphone;
#endif /* CONFIG_HEADSET_BUTTON_SWIC */

#ifdef CONFIG_SII8334_MHL_TX
int shswic_regulator_power(bool on);
void shswic_detect_cb_mhl(shmhl_detect_device_t device, void* user_data);
#endif /* CONFIG_SII8334_MHL_TX */

#ifdef CONFIG_CRADLE_SWIC
irqreturn_t shswic_cradle_isr(int irq, void *dev_id);
void shswic_state_cradle_proc(void);
#endif /* CONFIG_CRADLE_SWIC */

#ifdef CONFIG_HEADSET_BUTTON_SWIC
void shswic_send_headset_sw_button(uint8_t int_status);
#endif /* CONFIG_HEADSET_BUTTON_SWIC */

void shswic_clear_timer_signal(long sig);
int8_t shswic_status_read(uint8_t* id_status, uint8_t* status, uint8_t* int_status);
shswic_result_t shswic_i2c_write(uint8_t* write_buf, uint8_t len, uint8_t reg);
shswic_result_t shswic_i2c_read(uint8_t* read_buf, uint8_t len, uint8_t reg);
uint8_t shswic_detect_get(uint8_t id_status, uint8_t status, uint8_t int_status);
void shswic_send_timer_signal(long sig, int msec);
irqreturn_t shswic_detect_isr(int irq, void *dev_id);

#endif /* _SHSWIC_KERL_LOCAL_ */
