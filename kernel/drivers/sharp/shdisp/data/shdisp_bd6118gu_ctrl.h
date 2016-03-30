/* drivers/sharp/shdisp/data/shdisp_bd6118gu_ctrl.h  (Display Driver)
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

/* ------------------------------------------------------------------------- */
/* SHARP DISPLAY DRIVER FOR KERNEL MODE                                      */
/* ------------------------------------------------------------------------- */

#ifndef SHDISP_BD6118GU_CTRL_H
#define SHDISP_BD6118GU_CTRL_H

/* ------------------------------------------------------------------------- */
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */

#include "../shdisp_bd6118gu.h"

/* ------------------------------------------------------------------------- */
/* VARIABLES                                                                 */
/* ------------------------------------------------------------------------- */

static const shdisp_bdicRegSetting_t shdisp_bdic_init[] = {
     {BDIC_REG_PSCONT,              SHDISP_BDIC_STR,    0x24,                       0xFF,  10000}
    ,{BDIC_REG_GPOUT,               SHDISP_BDIC_CLR,    0x00,                       0xFF,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_active[] = {
     {BDIC_REG_PSCONT,              SHDISP_BDIC_CLR,    0x00,                       0x04,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_standby[] = {
     {BDIC_REG_PSCONT,              SHDISP_BDIC_SET,    0x04,                       0x04,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_bkl_on[] = {
     {BDIC_REG_PSCONT,              SHDISP_BDIC_RMW,    0x01,                       0x09,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_bkl_off[] = {
     {BDIC_REG_PSCONT,              SHDISP_BDIC_CLR,    0x00,                       0x01,      0}
};

static shdisp_bdicRegSetting_t shdisp_bdic_bkl_led_value[] = {
     {BDIC_REG_MLEDCNT,             SHDISP_BDIC_STR,    0x7F,                        0xFF,     0}
};

static shdisp_bdicRegSetting_t shdisp_bdic_led_fix_on[] = {
     {BDIC_REG_LEDCNTR,               SHDISP_BDIC_STR,  0x00,                       0x00,       0}
    ,{BDIC_REG_LEDCNTG,               SHDISP_BDIC_STR,  0x00,                       0x00,       0}
    ,{BDIC_REG_LEDCNTB,               SHDISP_BDIC_STR,  0x00,                       0x00,       0}
    ,{BDIC_REG_LEDDRVCNT,             SHDISP_BDIC_SET,  0x00,                       0x00,       0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_led_ani_on[] = {
     {BDIC_REG_LEDDRVCNT,             SHDISP_BDIC_SET,   0x80,                       0x80,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_led_ani_off[] = {
     {BDIC_REG_LEDDRVCNT,             SHDISP_BDIC_CLR,   0x00,                       0x80,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_led_set_ontime_on[] = {
     {BDIC_REG_LEDDRVCNT,             SHDISP_BDIC_SET,   0x08,                       0x08,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_led_set_ontime_off[] = {
     {BDIC_REG_LEDDRVCNT,             SHDISP_BDIC_CLR,   0x00,                       0x08,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_led_lposc_enable[] = {
     {BDIC_REG_GPOUT,             SHDISP_BDIC_SET,       0x40,                       0x40,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_led_lposc_disable[] = {
     {BDIC_REG_GPOUT,             SHDISP_BDIC_CLR,       0x00,                       0x40,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_led_off[] = {
     {BDIC_REG_LEDDRVCNT,             SHDISP_BDIC_CLR,   0x00,                       0x07,      0}
    ,{BDIC_REG_GPOUT,                 SHDISP_BDIC_CLR,   0x00,                       0x40,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_subdisplay_power_on[] = {
     {BDIC_REG_PSCONT,              SHDISP_BDIC_SET,    0x29,                         0x29,     0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_subdisplay_power_off[] = {
     {BDIC_REG_PSCONT,              SHDISP_BDIC_CLR,    0x00,                         0x09,     0}
};
#endif /* SHDISP_BD6118GU_CTRL_H */

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
