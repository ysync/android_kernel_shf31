/* drivers/sharp/shdisp/shdisp_pm.h  (Display Driver)
 *
 * Copyright (C) 2013 SHARP CORPORATION
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

/* ------------------------------------------------------------------------- */
/* SHARP DISPLAY DRIVER FOR KERNEL MODE                                      */
/* ------------------------------------------------------------------------- */

#ifndef SHDISP_PM_H
#define SHDISP_PM_H

/* ------------------------------------------------------------------------- */
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */
#include <sharp/shdisp_kerl.h>
/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */
#define SHDISP_DEV_TYPE_NONE        (0x00000000)
#define SHDISP_DEV_TYPE_LCD         (0x00000001)
#define SHDISP_DEV_TYPE_BKL         (0x00000002)
#define SHDISP_DEV_TYPE_LED         (0x00000004)
#ifndef SHDISP_NOT_SUPPORT_PSALS
#define SHDISP_DEV_TYPE_PS          (0x00000010)
#define SHDISP_DEV_TYPE_ALS_APP     (0x00000020)
#define SHDISP_DEV_TYPE_ALS_BKL     (0x00000040)
#define SHDISP_DEV_TYPE_ALS_CAMERA  (0x00000080)
#define SHDISP_DEV_TYPE_ALS_DIAG    (0x00000200)
#endif
#define SHDISP_DEV_TYPE_SUBDISPLAY  (0x00000400)
#define SHDISP_DEV_TYPE_RECOVERY    (0x01000000)
#define SHDISP_DEV_TYPE_INIT        (0x02000000)
#define SHDISP_DEV_TYPE_DEBUG       (0x10000000)
#define SHDISP_DEV_TYPE_MASK        (0x130007F7)
#ifndef SHDISP_NOT_SUPPORT_PSALS
#define SHDISP_DEV_TYPE_ALS_MASK    (SHDISP_DEV_TYPE_LCD | \
                                     SHDISP_DEV_TYPE_ALS_APP | \
                                     SHDISP_DEV_TYPE_ALS_BKL | \
                                     SHDISP_DEV_TYPE_ALS_CAMERA | \
                                     SHDISP_DEV_TYPE_ALS_DIAG)
#endif
/* ------------------------------------------------------------------------- */
/* TYPES                                                                     */
/* ------------------------------------------------------------------------- */
enum {
    SHDISP_DEV_STATE_NOINIT = 0,
    SHDISP_DEV_STATE_OFF,
    SHDISP_DEV_STATE_ON,
    SHDISP_DEV_STATE_INIT,
    NUM_SHDISP_DEV_STATE
};

#ifndef SHDISP_NOT_SUPPORT_PSALS
enum {
    SHDISP_SENSOR_STATE_POWER_OFF = 0,
    SHDISP_SENSOR_STATE_POWER_ON,
    SHDISP_SENSOR_STATE_PROX_ON_ALS_OFF,
    SHDISP_SENSOR_STATE_PROX_OFF_ALS_ON,
    SHDISP_SENSOR_STATE_PROX_ON_ALS_ON,
    NUM_SHDISP_SENSOR_STATE
};
#endif
/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */
void shdisp_pm_init(struct shdisp_boot_context *shdisp_boot_ctx);
int shdisp_pm_bdic_power_manager(int type, int state);
#ifndef SHDISP_NOT_SUPPORT_PSALS
int shdisp_pm_psals_power_manager(int type, int state);
int shdisp_pm_is_psals_active(void);
int shdisp_pm_is_als_active(void);
int shdisp_pm_is_ps_active(void);
#endif
void shdisp_pm_bdic_shutdown(void);
int  shdisp_pm_bdic_resume(void);
#if defined (CONFIG_ANDROID_ENGINEERING)
void shdisp_pm_power_manager_users_dump(void);
#endif /* CONFIG_ANDROID_ENGINEERING */
#ifndef SHDISP_NOT_SUPPORT_PSALS
void shdisp_pm_psals_error_power_off(void);
int shdisp_pm_psals_error_power_recovery(void);
#endif
int shdisp_pm_is_led_active(void);
#endif /* SHDISP_PM_H */

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
