/* drivers/sharp/shdisp/data/shdisp_columbus_data.h  (Display Driver)
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

static char cmd_set_page_1_payloads[] = {
    0xF0,0x55,0xAA,0x52,0x08,0x01
};
static struct shdisp_dsi_cmd_desc cmd_set_page_1_cmds[] = {
    { SHDISP_DTYPE_DCS_LWRITE, 6, &cmd_set_page_1_payloads[0],0}
};


static char voltage_setting_1_payloads[] = {
    0xB0,0x0A,
    0xB1,0x0A,
    0xB2,0x02,
    0xB3,0x0E,
    0xB4,0x0E
};
static struct shdisp_dsi_cmd_desc voltage_setting_1_cmds[] = {
    { SHDISP_DTYPE_DCS_WRITE1, 2, &voltage_setting_1_payloads[0],0},
    { SHDISP_DTYPE_DCS_WRITE1, 2, &voltage_setting_1_payloads[2],0},
    { SHDISP_DTYPE_DCS_WRITE1, 2, &voltage_setting_1_payloads[4],0},
    { SHDISP_DTYPE_DCS_WRITE1, 2, &voltage_setting_1_payloads[6],0},
    { SHDISP_DTYPE_DCS_WRITE1, 2, &voltage_setting_1_payloads[8],0}
};


static char power_ctl_payloads[] ={
    0xB6,0x44,
    0xB7,0x34,
    0xB8,0x14,
    0xB9,0x34,
    0xBA,0x34
};
static struct shdisp_dsi_cmd_desc power_ctl_cmds[] = {
    { SHDISP_DTYPE_DCS_WRITE1, 2, &power_ctl_payloads[0],0},
    { SHDISP_DTYPE_DCS_WRITE1, 2, &power_ctl_payloads[2],0},
    { SHDISP_DTYPE_DCS_WRITE1, 2, &power_ctl_payloads[4],0},
    { SHDISP_DTYPE_DCS_WRITE1, 2, &power_ctl_payloads[6],0},
    { SHDISP_DTYPE_DCS_WRITE1, 2, &power_ctl_payloads[8],0}
};


static char voltage_setting_2_payloads[] = {
    0xBC,0x00,0xA0,0x00,
    0xBD,0x00,0xA0,0x00
};
static struct shdisp_dsi_cmd_desc voltage_setting_2_cmds[] = {
    { SHDISP_DTYPE_DCS_LWRITE, 4, &voltage_setting_2_payloads[0],0},
    { SHDISP_DTYPE_DCS_LWRITE, 4, &voltage_setting_2_payloads[4],0}
};


static char vcom_setting_payloads[] = {
    0xBE,0x82
};
static struct shdisp_dsi_cmd_desc vcom_setting_cmds[] = {
    { SHDISP_DTYPE_DCS_WRITE1, 2, &vcom_setting_payloads[0],0}
};


static char output_pin_ctl_payloads[] = {
    0xC0,0x00,0x08
};
static struct shdisp_dsi_cmd_desc output_pin_ctl_cmds[] = {
    { SHDISP_DTYPE_DCS_LWRITE, 3, &output_pin_ctl_payloads[0],0}
};


static char gamma_curve_ctl_payloads[] = {
    0xCF,0x08
};
static struct shdisp_dsi_cmd_desc gamma_curve_ctl_cmds[] = {
    { SHDISP_DTYPE_DCS_WRITE1, 2, &gamma_curve_ctl_payloads[0],0}
};


static char gamma_setting_d1_payloads[] = {
    0xD1,0x00,0x34,0x00,0x3B,0x00,0x51,0x00,0x65,0x00,0x74,0x00,0x91,0x00,0xAA,0x00,0xD2
};
static char gamma_setting_d2_payloads[] = {
    0xD2,0x00,0xF0,0x01,0x22,0x01,0x48,0x01,0x81,0x01,0xAE,0x01,0xAF,0x01,0xD5,0x01,0xFE
};
static char gamma_setting_d3_payloads[] = {
    0xD3,0x02,0x13,0x02,0x32,0x02,0x49,0x02,0x6D,0x02,0x8A,0x02,0xB7,0x02,0xD9,0x03,0x84
};
static char gamma_setting_d4_payloads[] = {
    0xD4,0x03,0xEA,0x03,0xEE
};
static char gamma_setting_e0_payloads[] = {
    0xE0,0x00,0x1B,0x00,0x24,0x00,0x3C,0x00,0x50,0x00,0x5F,0x00,0x7C,0x00,0x95,0x00,0xBF
};
static char gamma_setting_e1_payloads[] = {
    0xE1,0x00,0xE1,0x01,0x15,0x01,0x3D,0x01,0x7C,0x01,0xAD,0x01,0xAF,0x01,0xDA,0x02,0x03
};
static char gamma_setting_e2_payloads[] = {
    0xE2,0x02,0x1C,0x02,0x3B,0x02,0x52,0x02,0x7C,0x02,0x9F,0x02,0xD6,0x02,0xF8,0x03,0xA3
};
static char gamma_setting_e3_payloads[] = {
    0xE3,0x03,0xFF,0x03,0xFF
};
static struct shdisp_dsi_cmd_desc gamma_setting_pic_adj_cmds[] = {
    { SHDISP_DTYPE_DCS_LWRITE, 17, &gamma_setting_d1_payloads[0],0},
    { SHDISP_DTYPE_DCS_LWRITE, 17, &gamma_setting_d2_payloads[0],0},
    { SHDISP_DTYPE_DCS_LWRITE, 17, &gamma_setting_d3_payloads[0],0},
    { SHDISP_DTYPE_DCS_LWRITE, 5, &gamma_setting_d4_payloads[0],0},
    { SHDISP_DTYPE_DCS_LWRITE, 17, &gamma_setting_e0_payloads[0],0},
    { SHDISP_DTYPE_DCS_LWRITE, 17, &gamma_setting_e1_payloads[0],0},
    { SHDISP_DTYPE_DCS_LWRITE, 17, &gamma_setting_e2_payloads[0],0},
    { SHDISP_DTYPE_DCS_LWRITE, 5, &gamma_setting_e3_payloads[0],0}
};


static char cmd_set_page_0_payloads[] = {
    0xF0,0x55,0xAA,0x52,0x08,0x00
};
static struct shdisp_dsi_cmd_desc cmd_set_page_0_cmds[] = {
    { SHDISP_DTYPE_DCS_LWRITE, 6, &cmd_set_page_0_payloads[0],0}
};


static char disp_setting_1_2_payloads[] = {
    0xB1,0x2B,0x00,0x01,
    0xB4,0x78,
    0xB6,0x0E,
    0xB7,0x00,0x55,
    0xB8,0x00,0x05,0x05,0x00,
    0xBA,0x02,
    0xBB,0x93,0x90,
    0xBC,0x02,
    0xBD,0x01,0xC1,0x10,0x38,
    0xC7,0x00,0x0F,0x0F,0x74,0x85,0x97,0xA8,0xF0,0xF0,0x00,0x00,
    0xFF,0xAA,0x55,0x25,0x81,
    0x6F,0x26,
    0xF9,0x11,0x12,0x0F,0x10
};
static struct shdisp_dsi_cmd_desc disp_setting_1_2_cmds[] = {
    { SHDISP_DTYPE_DCS_LWRITE, 4, &disp_setting_1_2_payloads[0],0},
    { SHDISP_DTYPE_DCS_WRITE1, 2, &disp_setting_1_2_payloads[4],0},
    { SHDISP_DTYPE_DCS_WRITE1, 2, &disp_setting_1_2_payloads[6],0},
    { SHDISP_DTYPE_DCS_LWRITE, 3, &disp_setting_1_2_payloads[8],0},
    { SHDISP_DTYPE_DCS_LWRITE, 5, &disp_setting_1_2_payloads[11],0},
    { SHDISP_DTYPE_DCS_WRITE1, 2, &disp_setting_1_2_payloads[16],0},
    { SHDISP_DTYPE_DCS_LWRITE, 3, &disp_setting_1_2_payloads[18],0},
    { SHDISP_DTYPE_DCS_WRITE1, 2, &disp_setting_1_2_payloads[21],0},
    { SHDISP_DTYPE_DCS_LWRITE, 5, &disp_setting_1_2_payloads[23],0},
    { SHDISP_DTYPE_DCS_LWRITE, 12, &disp_setting_1_2_payloads[28],0},
    { SHDISP_DTYPE_DCS_LWRITE, 5, &disp_setting_1_2_payloads[40],0},
    { SHDISP_DTYPE_DCS_WRITE1, 2, &disp_setting_1_2_payloads[45],0},
    { SHDISP_DTYPE_DCS_LWRITE, 5, &disp_setting_1_2_payloads[47],0}
};




static char exit_sleep_payloads[] = {
    0x11
};
static struct shdisp_dsi_cmd_desc exit_sleep_cmds[] = {
    { SHDISP_DTYPE_DCS_WRITE, 1, &exit_sleep_payloads[0],0}
};


static char disp_on_payloads[] = {
    0x29
};
static struct shdisp_dsi_cmd_desc disp_on_cmds[] = {
    { SHDISP_DTYPE_DCS_WRITE, 1, &disp_on_payloads[0],0}
};


static char enter_sleep_payloads[] = {
    0x10
};
static struct shdisp_dsi_cmd_desc enter_sleep_cmds[] = {
    { SHDISP_DTYPE_DCS_WRITE, 1, &enter_sleep_payloads[0],80000}
};


static char disp_off_payloads[] = {
    0x28
};
static struct shdisp_dsi_cmd_desc disp_off_cmds[] = {
    { SHDISP_DTYPE_DCS_WRITE, 1, &disp_off_payloads[0],34000}
};


static char deep_standby_payloads[] = {
    0x4F,0x01
};
static struct shdisp_dsi_cmd_desc deep_standby_cmds[] = {
    { SHDISP_DTYPE_DCS_WRITE1, 2, &deep_standby_payloads[0],5000}
};
