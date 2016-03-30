/* drivers/sharp/shdisp/shdisp_columbus.c  (Display Driver)
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

/* ------------------------------------------------------------------------- */
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */

#include <linux/compiler.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/mm.h>
#include <linux/idr.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/ioctl.h>
#include <linux/gpio.h>
#include <linux/hrtimer.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/wakelock.h>
#include <linux/qpnp/qpnp-api.h>
#include <linux/regulator/consumer.h>
#include <sharp/shdisp_kerl.h>
#include <mach/rpm-regulator.h>
#include <mach/rpm-regulator-smd.h>
#include "shdisp_kerl_priv.h"
#include "shdisp_pm.h"
#include "shdisp_panel.h"
#include "shdisp_columbus.h"
#include "shdisp_system.h"
#include "shdisp_bdic.h"
#include "shdisp_type.h"
#include "shdisp_dbg.h"
#include "data/shdisp_columbus_data.h"



/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */
#define SHDISP_COLUMBUS_VCOM_MIN                0x0000
#define SHDISP_COLUMBUS_VCOM_MAX                0x00FF
#define SHDISP_COLUMBUS_ALPHA_DEFAULT           0x0082

#define SHDISP_COLUMBUS_GAMMA_B0_REG                     0xB0
#define SHDISP_COLUMBUS_GAMMA_B1_REG                     0xB1
#define SHDISP_COLUMBUS_GAMMA_B2_REG                     0xB2
#define SHDISP_COLUMBUS_GAMMA_B3_REG                     0xB3
#define SHDISP_COLUMBUS_GAMMA_B4_REG                     0xB4
#define SHDISP_COLUMBUS_GAMMA_BC_REG                     0xBC
#define SHDISP_COLUMBUS_GAMMA_BD_REG                     0xBD

#define SHDISP_COLUMBUS_VCOM_REG                         0xBE

#define SHDISP_COLUMBUS_GAMMA_D1_REG                     0xD1
#define SHDISP_COLUMBUS_GAMMA_D2_REG                     0xD2
#define SHDISP_COLUMBUS_GAMMA_D3_REG                     0xD3
#define SHDISP_COLUMBUS_GAMMA_D4_REG                     0xD4
#define SHDISP_COLUMBUS_GAMMA_E0_REG                     0xE0
#define SHDISP_COLUMBUS_GAMMA_E1_REG                     0xE1
#define SHDISP_COLUMBUS_GAMMA_E2_REG                     0xE2
#define SHDISP_COLUMBUS_GAMMA_E3_REG                     0xE3

/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */

static int shdisp_columbus_API_init_io(void);
static int shdisp_columbus_API_exit_io(void);
static int shdisp_columbus_API_power_on(int mode);
static int shdisp_columbus_API_power_off(int mode);
static int shdisp_columbus_API_disp_on(void);
static int shdisp_columbus_API_disp_off(void);
static int shdisp_columbus_API_start_display(void);
static int shdisp_columbus_API_post_video_start(void);
static int shdisp_columbus_API_check_flicker_param(unsigned short alpha_in, unsigned short *alpha_out);
static int shdisp_columbus_API_diag_write_reg(unsigned char addr, unsigned char *write_data, unsigned char size);
static int shdisp_columbus_API_diag_read_reg(unsigned char addr, unsigned char *read_data, unsigned char size);
static int shdisp_columbus_API_diag_set_flicker_param(struct shdisp_diag_flicker_param alpha);
static int shdisp_columbus_API_diag_get_flicker_param(struct shdisp_diag_flicker_param *alpha);
static int shdisp_columbus_API_diag_get_flicker_low_param(struct shdisp_diag_flicker_param *alpha);
static int shdisp_columbus_API_check_recovery(void);
static int shdisp_columbus_API_diag_set_gamma_info(struct shdisp_diag_gamma_info *gamma_info);
static int shdisp_columbus_API_diag_get_gamma_info(struct shdisp_diag_gamma_info *gamma_info);
static int shdisp_columbus_API_diag_set_gamma(struct shdisp_diag_gamma *gamma);
static int shdisp_columbus_API_shutdown(void);
static void shdisp_columbus_API_dump(int type);
static int shdisp_columbus_API_set_irq(int enable);


static int shdisp_columbus_mipi_cmd_lcd_on(void);
static int shdisp_columbus_mipi_cmd_lcd_off(void);
static int shdisp_columbus_mipi_cmd_start_display(void);
static int shdisp_columbus_mipi_cmd_post_video_start(void);
#if 0
static int shdisp_columbus_check_mipi_error(void);
static char shdisp_columbus_mipi_interface_id_setting(void);
static int shdisp_columbus_mipi_checksum_and_ecc_error_count(char *out);
#endif

#ifndef SHDISP_NOT_SUPPORT_FLICKER
static int shdisp_columbus_diag_set_flicker_param_internal(struct shdisp_diag_flicker_param flicker_param);
static int shdisp_columbus_diag_set_flicker_param_ctx(struct shdisp_diag_flicker_param flicker_param);
#endif /* SHDISP_NOT_SUPPORT_FLICKER */

int shdisp_columbus_init_phy_gamma(struct shdisp_lcddr_phy_gamma_reg *phy_gamma);
static int shdisp_columbus_request_irq(void);
static irqreturn_t shdisp_columbus_int_isr(int irq_num, void *data);
static int shdisp_columbus_set_irq(int enable);
static int shdisp_columbus_register_driver(void);
static void shdisp_workqueue_handler_columbus(struct work_struct *work);

/* ------------------------------------------------------------------------- */
/* VARIABLES                                                                 */
/* ------------------------------------------------------------------------- */
struct rpm_regulator *rpm_vdd_vreg;

#ifndef SHDISP_NOT_SUPPORT_FLICKER
static unsigned char columbus_wdata;
static unsigned char columbus_rdata;
#endif /* SHDISP_NOT_SUPPORT_FLICKER */

static struct shdisp_panel_context shdisp_panel_ctx;
static bool lcd_first_on = false;


static struct shdisp_panel_operations shdisp_columbus_fops = {
    shdisp_columbus_API_init_io,
    shdisp_columbus_API_exit_io,
    NULL,
    shdisp_columbus_API_power_on,
    shdisp_columbus_API_power_off,
    shdisp_columbus_API_disp_on,
    shdisp_columbus_API_disp_off,
    shdisp_columbus_API_start_display,
    shdisp_columbus_API_post_video_start,
    shdisp_columbus_API_check_flicker_param,
    shdisp_columbus_API_diag_write_reg,
    shdisp_columbus_API_diag_read_reg,
    shdisp_columbus_API_diag_set_flicker_param,
    shdisp_columbus_API_diag_get_flicker_param,
    shdisp_columbus_API_diag_get_flicker_low_param,
    shdisp_columbus_API_check_recovery,
    shdisp_columbus_API_diag_set_gamma_info,
    shdisp_columbus_API_diag_get_gamma_info,
    shdisp_columbus_API_diag_set_gamma,
    shdisp_columbus_API_shutdown,
    shdisp_columbus_API_dump,
    shdisp_columbus_API_set_irq
};

static struct workqueue_struct    *shdisp_wq_columbus = NULL;
static struct work_struct         shdisp_wq_columbus_wk;
static int shdisp_columbus_irq = 0;
static spinlock_t shdisp_columbus_spin_lock;
static struct wake_lock shdisp_columbus_wake_lock;
static struct platform_device *shdisp_columbus_int_irq_port_pdev = NULL;
static int shdisp_columbus_int_irq_port_status = SHDISP_IRQ_DISABLE;
static spinlock_t shdisp_columbus_set_irq_spinlock;


/* ------------------------------------------------------------------------- */
/* DEBUG MACROS                                                              */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */
#define MIPI_DSI_COMMAND_TX(x)         (shdisp_panel_API_mipi_dsi_cmds_tx(0,x, ARRAY_SIZE(x)))
#define MIPI_DSI_COMMAND_TX_COMMIT(x)  (shdisp_panel_API_mipi_dsi_cmds_tx(1,x, ARRAY_SIZE(x)))

/* ------------------------------------------------------------------------- */
/* FUNCTIONS                                                                 */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* API                                                                       */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_columbus_API_create                                                */
/* ------------------------------------------------------------------------- */

struct shdisp_panel_operations *shdisp_columbus_API_create(void)
{
    SHDISP_TRACE("\n");
    return &shdisp_columbus_fops;
}

/* ------------------------------------------------------------------------- */
/* shdisp_columbus_init_flicker_param                                            */
/* ------------------------------------------------------------------------- */
static int shdisp_columbus_init_flicker_param(unsigned short vcom)
{
#ifndef SHDISP_NOT_SUPPORT_FLICKER
    SHDISP_TRACE("in\n");
    SHDISP_DEBUG("vcom=0x%04x\n", vcom);

    vcom_setting_payloads[1] = vcom & 0xFF;

    SHDISP_DEBUG("VCOM=0x%02x \n", vcom_setting_payloads[1]);

    SHDISP_TRACE("out\n");

#endif
    return SHDISP_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* shdisp_columbus_API_init_io                                               */
/* ------------------------------------------------------------------------- */

static int shdisp_columbus_API_init_io(void)
{
    int rc = SHDISP_RESULT_SUCCESS;
#ifndef SHDISP_NOT_SUPPORT_GAMMA
    int ret = 0;
    struct shdisp_lcddr_phy_gamma_reg *lcddr_phy_gamma_tmp;

    SHDISP_TRACE("in\n");

#ifndef SHDISP_NOT_SUPPORT_FLICKER
    shdisp_panel_ctx.vcom = shdisp_api_get_alpha();
    shdisp_panel_ctx.vcom_low = shdisp_api_get_alpha_low();
#ifndef SHDISP_NOT_SUPPORT_NO_OS
    shdisp_columbus_init_flicker_param(shdisp_panel_ctx.vcom);
#endif /* SHDISP_NOT_SUPPORT_NO_OS */
#endif /* SHDISP_NOT_SUPPORT_FLICKER */

    lcddr_phy_gamma_tmp = shdisp_api_get_lcddr_phy_gamma();
    memcpy(&(shdisp_panel_ctx.lcddr_phy_gamma), lcddr_phy_gamma_tmp, sizeof(struct shdisp_lcddr_phy_gamma_reg));
    ret = shdisp_columbus_init_phy_gamma(&shdisp_panel_ctx.lcddr_phy_gamma);
    if (ret) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_columbus_init_phy_gamma.\n");
    }
#endif /* SHDISP_NOT_SUPPORT_GAMMA */
    shdisp_wq_columbus = create_singlethread_workqueue("shdisp_columbus_queue");
    if (!shdisp_wq_columbus) {
        SHDISP_ERR("shdisp_columbus_workqueue create failed.\n");
        rc = SHDISP_RESULT_FAILURE;
        goto workq_create_error;
    }
    INIT_WORK(&shdisp_wq_columbus_wk, shdisp_workqueue_handler_columbus);

    spin_lock_init(&shdisp_columbus_spin_lock);
    spin_lock_init(&shdisp_columbus_set_irq_spinlock);
    wake_lock_init(&shdisp_columbus_wake_lock, WAKE_LOCK_SUSPEND, "columbus_wake_lock");

    shdisp_columbus_register_driver();

    if(rpm_vdd_vreg != NULL) {
        rc = rpm_regulator_set_mode(rpm_vdd_vreg,RPM_REGULATOR_MODE_HPM);
        if (rc < 0) {
            SHDISP_ERR("%s: Failed to set for rpm regulator: %d\n",__func__,rc);
            rpm_regulator_put(rpm_vdd_vreg);
        }
     }

    rc = shdisp_columbus_request_irq();
    if (rc != 0) {
        SHDISP_ERR("shdisp_columbus_request_irq() failed. rc = %d\n", rc);
        rc = SHDISP_RESULT_FAILURE;
        goto request_irq_error;
    }

    SHDISP_TRACE("out\n");
    return rc;

request_irq_error:
    destroy_workqueue(shdisp_wq_columbus);
    shdisp_wq_columbus = NULL;

workq_create_error:

    SHDISP_TRACE("out\n");
    return rc;
}

/* ------------------------------------------------------------------------- */
/* shdisp_columbus_API_exit_io                                               */
/* ------------------------------------------------------------------------- */

static int shdisp_columbus_API_exit_io(void)
{
    int ret;
    SHDISP_TRACE("in\n");

    if(rpm_vdd_vreg != NULL) {
        ret = rpm_regulator_set_mode(rpm_vdd_vreg,RPM_REGULATOR_MODE_IPEAK);
        if (ret < 0) {
            SHDISP_ERR("%s: Failed to set for rpm regulator: %d\n",__func__,ret);
            rpm_regulator_put(rpm_vdd_vreg);
        }
    }
    if (shdisp_wq_columbus) {
        flush_workqueue(shdisp_wq_columbus);
        destroy_workqueue(shdisp_wq_columbus);
        shdisp_wq_columbus = NULL;
    }

    free_irq(shdisp_columbus_irq, 0);
    SHDISP_TRACE("out\n");

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_columbus_API_power_on                                              */
/* ------------------------------------------------------------------------- */

static int shdisp_columbus_API_power_on(int mode)
{
    int rc;
    SHDISP_TRACE("in\n");

    if(rpm_vdd_vreg != NULL) {
        rc = rpm_regulator_set_mode(rpm_vdd_vreg,RPM_REGULATOR_MODE_HPM);
        if (rc < 0) {
            SHDISP_ERR("%s: Failed to set for rpm regulator: %d\n",__func__,rc);
            rpm_regulator_put(rpm_vdd_vreg);
        }
    }

    if (shdisp_api_get_boot_disp_status() == 0 && !lcd_first_on) {
        SHDISP_DEBUG("%s is called when lcd first on\n",__func__);
        shdisp_bdic_API_LCD_release_hw_reset();
        lcd_first_on = true;
    } else {
        SHDISP_DEBUG("%s is called when lcd resume\n",__func__);
        shdisp_bdic_API_LCD_release_deep_standby();
    }

    SHDISP_TRACE("out\n");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_columbus_API_power_off                                             */
/* ------------------------------------------------------------------------- */

static int shdisp_columbus_API_power_off(int mode)
{
    int ret;
    SHDISP_TRACE("in\n");
    if(mode != SHDISP_PANEL_POWER_RECOVERY_OFF){
    }

    if(rpm_vdd_vreg != NULL) {
        ret = rpm_regulator_set_mode(rpm_vdd_vreg,RPM_REGULATOR_MODE_IPEAK);
        if (ret < 0) {
            SHDISP_ERR("%s: Failed to set for rpm regulator: %d\n",__func__,ret);
            rpm_regulator_put(rpm_vdd_vreg);
        }
    }

    SHDISP_TRACE("out\n");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_columbus_API_disp_on                                               */
/* ------------------------------------------------------------------------- */

static int shdisp_columbus_API_disp_on(void)
{
    int ret = 0;

    SHDISP_TRACE("in\n");

    ret = shdisp_columbus_mipi_cmd_lcd_on();

    SHDISP_TRACE("out ret=%d\n", ret);

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_columbus_API_disp_off                                              */
/* ------------------------------------------------------------------------- */

static int shdisp_columbus_API_disp_off(void)
{
    SHDISP_TRACE("in\n");

    shdisp_columbus_set_irq(SHDISP_IRQ_DISABLE);

    shdisp_columbus_mipi_cmd_lcd_off();

    SHDISP_TRACE("out\n");
    return 0;
}

/* ------------------------------------------------------------------------- */
/* shdisp_columbus_API_start_display                                         */
/* ------------------------------------------------------------------------- */

static int shdisp_columbus_API_start_display(void)
{
    SHDISP_TRACE("in\n");
    shdisp_columbus_mipi_cmd_start_display();
    SHDISP_TRACE("out\n");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_columbus_API_post_video_start                                      */
/* ------------------------------------------------------------------------- */

static int shdisp_columbus_API_post_video_start(void)
{

    SHDISP_TRACE("in\n");

    shdisp_SYS_delay_us(120*1000);

    shdisp_columbus_mipi_cmd_post_video_start();

    shdisp_columbus_set_irq(SHDISP_IRQ_ENABLE);

    SHDISP_TRACE("out\n");

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_columbus_API_check_flicker_param                                   */
/* ------------------------------------------------------------------------- */

static int shdisp_columbus_API_check_flicker_param(unsigned short alpha_in, unsigned short *alpha_out)
{
#ifndef SHDISP_NOT_SUPPORT_FLICKER
    unsigned short tmp_alpha = alpha_in;

    SHDISP_TRACE("in\n");
    if (alpha_out == NULL){
        SHDISP_ERR("<NULL_POINTER> alpha_out.\n");
        return SHDISP_RESULT_FAILURE;
    }

    if ((tmp_alpha & 0xF000) != 0x9000) {
        *alpha_out = SHDISP_COLUMBUS_ALPHA_DEFAULT;
        SHDISP_DEBUG("out1\n");
        return SHDISP_RESULT_SUCCESS;
    }

    tmp_alpha = tmp_alpha & 0x00FF;
    if ((tmp_alpha < SHDISP_COLUMBUS_VCOM_MIN) || (tmp_alpha > SHDISP_COLUMBUS_VCOM_MAX)) {
        *alpha_out = SHDISP_COLUMBUS_ALPHA_DEFAULT;
        SHDISP_DEBUG("out2\n");
        return SHDISP_RESULT_SUCCESS;
    }

    *alpha_out = tmp_alpha;

    SHDISP_TRACE("out\n");
#endif /* SHDISP_NOT_SUPPORT_FLICKER */
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_columbus_API_diag_write_reg                                        */
/* ------------------------------------------------------------------------- */

static int shdisp_columbus_API_diag_write_reg(unsigned char addr, unsigned char *write_data, unsigned char size)
{
    int ret = 0;
    char dtype;

    SHDISP_TRACE("in\n");

    if (size == 0) {
        dtype = SHDISP_DTYPE_DCS_WRITE;
    } else if(size == 1) {
        dtype = SHDISP_DTYPE_DCS_WRITE1;
    } else {
        dtype = SHDISP_DTYPE_DCS_LWRITE;
    }

    ret = shdisp_panel_API_mipi_diag_write_reg(dtype,addr, write_data, size);
    if(ret) {
        SHDISP_DEBUG("out1\n");
        return SHDISP_RESULT_FAILURE;
    }

    SHDISP_TRACE("out\n");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_columbus_API_diag_read_reg                                         */
/* ------------------------------------------------------------------------- */

static int shdisp_columbus_API_diag_read_reg(unsigned char addr, unsigned char *read_data, unsigned char size)
{
    int ret = 0;
    char dtype;

    SHDISP_TRACE("in\n");

    dtype = SHDISP_DTYPE_DCS_READ;

    ret = shdisp_panel_API_mipi_diag_read_reg(dtype,addr, read_data,size);

    if(ret) {
        SHDISP_DEBUG("out1\n");
        return SHDISP_RESULT_FAILURE;
    }
    SHDISP_TRACE("out\n");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_columbus_API_diag_set_flicker_param                                */
/* ------------------------------------------------------------------------- */

static int shdisp_columbus_API_diag_set_flicker_param(struct shdisp_diag_flicker_param flicker_param)
{
#ifndef SHDISP_NOT_SUPPORT_FLICKER
    int ret = 0;

    SHDISP_TRACE("in\n");

    ret = shdisp_columbus_diag_set_flicker_param_internal(flicker_param);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_aria_diag_set_flicker_param_internal.\n");
        return SHDISP_RESULT_FAILURE;
    } else {
        shdisp_columbus_diag_set_flicker_param_ctx(flicker_param);
    }

    SHDISP_TRACE("out\n");
#endif /* SHDISP_NOT_SUPPORT_FLICKER */
    return SHDISP_RESULT_SUCCESS;
}

#ifndef SHDISP_NOT_SUPPORT_FLICKER
/* ------------------------------------------------------------------------- */
/* shdisp_columbus_diag_set_flicker_param_internal                               */
/* ------------------------------------------------------------------------- */
static int shdisp_columbus_diag_set_flicker_param_internal(struct shdisp_diag_flicker_param flicker_param)
{
    int vcom = flicker_param.master_alpha;
    int vcom_low = flicker_param.master_alpha;
    int ret = 0;
    unsigned char columbus_rdata_tmp;

    SHDISP_TRACE("in\n");

    if (flicker_param.request & SHDISP_REG_WRITE) {
        columbus_rdata = 0;
        columbus_rdata_tmp = 0;
        columbus_wdata = 0;

        columbus_rdata_tmp = (unsigned char)(vcom & 0xFF);

        ret = MIPI_DSI_COMMAND_TX_COMMIT(cmd_set_page_1_cmds);
        if (ret != SHDISP_RESULT_SUCCESS){
            SHDISP_ERR("<RESULT_FAILURE> cmd_set_page_1_cmds!!\n" );
            return SHDISP_RESULT_FAILURE;
        }

        columbus_wdata = columbus_rdata_tmp;
        ret = shdisp_panel_API_mipi_diag_write_reg(SHDISP_DTYPE_DCS_WRITE1,SHDISP_COLUMBUS_VCOM_REG, &columbus_wdata, 1);
        if (ret) {
            SHDISP_ERR("<RESULT_FAILURE> write vcom register failure!!\n" );
        }

        SHDISP_DEBUG("VCOM=0x%02x \n", columbus_rdata_tmp);
        SHDISP_DEBUG("vcom=0x%04x\n", vcom);
        if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_DEBUG("out dokick err ret=%d\n", ret);
            return ret;
        }
    }

    if (flicker_param.request & (SHDISP_SAVE_VALUE | SHDISP_SAVE_VALUE_LOW)) {
        if (!(flicker_param.request & SHDISP_SAVE_VALUE)) {
            vcom = shdisp_panel_ctx.vcom;
        }
        if (!(flicker_param.request & SHDISP_SAVE_VALUE_LOW)) {
            vcom_low = shdisp_panel_ctx.vcom_low;
        }
        if (shdisp_columbus_init_flicker_param(vcom)) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_aria_init_flicker_param.");
        }
    }
    if (flicker_param.request & SHDISP_RESET_VALUE) {
        if (shdisp_columbus_init_flicker_param(flicker_param.master_alpha)) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_aria_init_flicker_param.");
        }
    }

    SHDISP_TRACE("out");

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_columbus_diag_set_flicker_param_ctx                                    */
/* ------------------------------------------------------------------------- */
static int shdisp_columbus_diag_set_flicker_param_ctx(struct shdisp_diag_flicker_param flicker_param)
{
    SHDISP_TRACE("in");

    if (flicker_param.request & SHDISP_SAVE_VALUE) {
        shdisp_panel_ctx.vcom = flicker_param.master_alpha;
        shdisp_panel_ctx.vcom_nvram = 0x9000 | flicker_param.master_alpha;
    }
    if (flicker_param.request & SHDISP_SAVE_VALUE_LOW) {
        shdisp_panel_ctx.vcom_low = flicker_param.master_alpha;
    }
    if (flicker_param.request & SHDISP_RESET_VALUE) {
        shdisp_panel_ctx.vcom = 0;
        shdisp_panel_ctx.vcom_low = 0;
        shdisp_panel_ctx.vcom_nvram = 0;
    }

    SHDISP_TRACE("out");

    return SHDISP_RESULT_SUCCESS;
}
#endif /* SHDISP_NOT_SUPPORT_FLICKER */

/* ------------------------------------------------------------------------- */
/* shdisp_columbus_API_diag_get_flicker_param                                */
/* ------------------------------------------------------------------------- */

static int shdisp_columbus_API_diag_get_flicker_param(struct shdisp_diag_flicker_param *flicker_param)
{
#ifndef SHDISP_NOT_SUPPORT_FLICKER
    int ret = 0;

    SHDISP_TRACE("in\n");
    if (flicker_param == NULL){
        SHDISP_ERR("<NULL_POINTER> alpha.\n");
        return SHDISP_RESULT_FAILURE;
    }

    columbus_rdata = 0;

    ret = MIPI_DSI_COMMAND_TX_COMMIT(cmd_set_page_1_cmds);
    if (ret != SHDISP_RESULT_SUCCESS){
        SHDISP_ERR("<RESULT_FAILURE> set page 1 failure!!\n" );
        return SHDISP_RESULT_FAILURE;
    }

    ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,SHDISP_COLUMBUS_VCOM_REG, &columbus_rdata, 1);
    if(ret) {
        SHDISP_ERR("<RESULT_FAILURE> read vcom register failure.\n");
        return SHDISP_RESULT_FAILURE;
    }

    flicker_param->master_alpha =  columbus_rdata;
    SHDISP_TRACE("out\n");
#endif /* SHDISP_NOT_SUPPORT_FLICKER */
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_columbus_API_diag_get_flicker_low_param                            */
/* ------------------------------------------------------------------------- */

static int shdisp_columbus_API_diag_get_flicker_low_param(struct shdisp_diag_flicker_param *flicker_param)
{
#ifndef SHDISP_NOT_SUPPORT_FLICKER
    SHDISP_TRACE("\n");
#endif /* SHDISP_NOT_SUPPORT_FLICKER */
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_columbus_API_check_recovery                                        */
/* ------------------------------------------------------------------------- */

static int shdisp_columbus_API_check_recovery(void)
{
    int ret;
#ifdef SHDISP_RESET_LOG
    struct shdisp_dbg_error_code err_code;
#endif /* SHDISP_RESET_LOG */

    SHDISP_TRACE("in\n");
    ret = shdisp_bdic_API_RECOVERY_check_restoration();

#if defined(CONFIG_ANDROID_ENGINEERING)
    if (shdisp_dbg_get_recovery_check_error() == SHDISP_DBG_RECOVERY_ERROR_DETLOW) {
        shdisp_dbg_update_recovery_check_error(SHDISP_DBG_RECOVERY_ERROR_NONE);
        SHDISP_DEBUG("force lcd det low.\n");
        ret = SHDISP_RESULT_FAILURE;
    }
#endif /* CONFIG_ANDROID_ENGINEERING */

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_bdic_API_RECOVERY_check_restoration.\n");
#ifdef SHDISP_RESET_LOG
        err_code.mode = SHDISP_DBG_MODE_LINUX;
        err_code.type = SHDISP_DBG_TYPE_PANEL;
        err_code.code = SHDISP_DBG_CODE_ERROR_DETECT;
        err_code.subcode = SHDISP_DBG_SUBCODE_DET_LOW;
        shdisp_dbg_api_err_output(&err_code, 0);
        shdisp_dbg_set_subcode(SHDISP_DBG_SUBCODE_DET_LOW);
#endif /* SHDISP_RESET_LOG */
        return SHDISP_RESULT_FAILURE;
    }

    SHDISP_TRACE("out\n");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_columbus_API_diag_set_gamma_info                                   */
/* ------------------------------------------------------------------------- */

static int shdisp_columbus_API_diag_set_gamma_info(struct shdisp_diag_gamma_info *gamma_info)
{
#ifndef SHDISP_NOT_SUPPORT_GAMMA
    int ret = 0;
    int i = 0;
    unsigned char columbus_wdata[104];
    unsigned char columbus_wdata_tmp;
    char voltage_setting_1[] = {
        0xB0,0x0A,
        0xB1,0x0A,
        0xB2,0x02,
        0xB3,0x0E,
        0xB4,0x0E
    };
    struct shdisp_dsi_cmd_desc voltage_setting_1_cmds_tmp[] = {
        { SHDISP_DTYPE_DCS_WRITE1, 2, &voltage_setting_1[0],0},
        { SHDISP_DTYPE_DCS_WRITE1, 2, &voltage_setting_1[2],0},
        { SHDISP_DTYPE_DCS_WRITE1, 2, &voltage_setting_1[4],0},
        { SHDISP_DTYPE_DCS_WRITE1, 2, &voltage_setting_1[6],0},
        { SHDISP_DTYPE_DCS_WRITE1, 2, &voltage_setting_1[8],0}
    };
    char voltage_setting_2[] = {
        0xBC,0x00,0xA0,0x00,
        0xBD,0x00,0xA0,0x00
    };
    struct shdisp_dsi_cmd_desc voltage_setting_2_cmds_tmp[] = {
        { SHDISP_DTYPE_DCS_LWRITE, 4, &voltage_setting_2[0],0},
        { SHDISP_DTYPE_DCS_LWRITE, 4, &voltage_setting_2[4],0}
    };
    char gamma_setting_d1[] = {
        0xD1,0x00,0x47,0x00,0x60,0x00,0x81,0x00,0x9B,0x00,0xAC,0x00,0xCE,0x00,0xEB,0x01,0x13
    };
    char gamma_setting_d2[] = {
        0xD2,0x01,0x34,0x01,0x6B,0x01,0x97,0x01,0xD9,0x02,0x0F,0x02,0x10,0x02,0x46,0x02,0x82
    };
    char gamma_setting_d3[] = {
        0xD3,0x02,0xA8,0x02,0xDF,0x03,0x03,0x03,0x2D,0x03,0x48,0x03,0x61,0x03,0x62,0x03,0x63
    };
    char gamma_setting_d4[] = {
        0xD4,0x03,0x78,0x03,0x7B
    };
    char gamma_setting_e0[] = {
        0xE0,0x00,0x47,0x00,0x60,0x00,0x81,0x00,0x9B,0x00,0xAC,0x00,0xCE,0x00,0xEB,0x01,0x13
    };
    char gamma_setting_e1[] = {
        0xE1,0x01,0x34,0x01,0x6B,0x01,0x97,0x01,0xD9,0x02,0x0F,0x02,0x10,0x02,0x46,0x02,0x82
    };
    char gamma_setting_e2[] = {
        0xE2,0x02,0xA8,0x02,0xDF,0x03,0x03,0x03,0x2D,0x03,0x48,0x03,0x61,0x03,0x62,0x03,0x63
    };
    char gamma_setting_e3[] = {
        0xE3,0x03,0x78,0x03,0x7B
    };
    struct shdisp_dsi_cmd_desc gamma_setting_pic_adj_cmds_tmp[] = {
        { SHDISP_DTYPE_DCS_LWRITE, 17, &gamma_setting_d1[0],0},
        { SHDISP_DTYPE_DCS_LWRITE, 17, &gamma_setting_d2[0],0},
        { SHDISP_DTYPE_DCS_LWRITE, 17, &gamma_setting_d3[0],0},
        { SHDISP_DTYPE_DCS_LWRITE, 5, &gamma_setting_d4[0],0},
        { SHDISP_DTYPE_DCS_LWRITE, 17, &gamma_setting_e0[0],0},
        { SHDISP_DTYPE_DCS_LWRITE, 17, &gamma_setting_e1[0],0},
        { SHDISP_DTYPE_DCS_LWRITE, 17, &gamma_setting_e2[0],0},
        { SHDISP_DTYPE_DCS_LWRITE, 5, &gamma_setting_e3[0],0}
    };
    voltage_setting_1[1] = gamma_info->avdd;
    voltage_setting_1[3] = gamma_info->avee;
    voltage_setting_1[5] = gamma_info->val;
    voltage_setting_1[7] = gamma_info->vgh;
    voltage_setting_1[9] = gamma_info->vgl;
    voltage_setting_2[1] = gamma_info->vgmp_h;
    voltage_setting_2[2] = gamma_info->vgmp;
    voltage_setting_2[3] = gamma_info->vgsp;
    voltage_setting_2[5] = gamma_info->vgmn_h;
    voltage_setting_2[6] = gamma_info->vgmn;
    voltage_setting_2[7] = gamma_info->vgsn;

    memset(columbus_wdata, 0, sizeof(columbus_wdata));
    memcpy(columbus_wdata, gamma_info->gamma, 104);
    for( i = 0; i < 104; i += 2 ) {
        columbus_wdata_tmp = columbus_wdata[i];
        columbus_wdata[i] = columbus_wdata[i+1];
        columbus_wdata[i+1] = columbus_wdata_tmp;
    }

    memcpy(&gamma_setting_d1[1], &columbus_wdata[0], 16);
    memcpy(&gamma_setting_d2[1], &columbus_wdata[16], 16);
    memcpy(&gamma_setting_d3[1], &columbus_wdata[32], 16);
    memcpy(&gamma_setting_d4[1], &columbus_wdata[48], 4);
    memcpy(&gamma_setting_e0[1], &columbus_wdata[52], 16);
    memcpy(&gamma_setting_e1[1], &columbus_wdata[68], 16);
    memcpy(&gamma_setting_e2[1], &columbus_wdata[84], 16);
    memcpy(&gamma_setting_e3[1], &columbus_wdata[100], 4);

    SHDISP_TRACE("in\n");

    ret = MIPI_DSI_COMMAND_TX_COMMIT(cmd_set_page_1_cmds);
    if (ret != SHDISP_RESULT_SUCCESS){
        SHDISP_ERR("<RESULT_FAILURE> cmd_set_page_1_cmds.\n");
        return SHDISP_RESULT_FAILURE;
    }

    ret = MIPI_DSI_COMMAND_TX_COMMIT(voltage_setting_1_cmds_tmp);
    if (ret != SHDISP_RESULT_SUCCESS){
        SHDISP_ERR("<RESULT_FAILURE> voltage_setting_1_cmds_tmp.\n");
        return SHDISP_RESULT_FAILURE;
    }
    ret = MIPI_DSI_COMMAND_TX_COMMIT(voltage_setting_2_cmds_tmp);
    if (ret != SHDISP_RESULT_SUCCESS){
        SHDISP_ERR("<RESULT_FAILURE> voltage_setting_2_cmds_tmp.\n");
        return SHDISP_RESULT_FAILURE;
    }
    ret = MIPI_DSI_COMMAND_TX_COMMIT(gamma_setting_pic_adj_cmds_tmp);
    if (ret != SHDISP_RESULT_SUCCESS){
        SHDISP_ERR("<RESULT_FAILURE> gamma_setting_pic_adj_cmds_tmp.\n");
        return SHDISP_RESULT_FAILURE;
    }

    voltage_setting_1_payloads[1] = gamma_info->avdd;
    voltage_setting_1_payloads[3] = gamma_info->avee;
    voltage_setting_1_payloads[5] = gamma_info->val;
    voltage_setting_1_payloads[7] = gamma_info->vgh;
    voltage_setting_1_payloads[9] = gamma_info->vgl;
    voltage_setting_2_payloads[1] = gamma_info->vgmp_h;
    voltage_setting_2_payloads[2] = gamma_info->vgmp;
    voltage_setting_2_payloads[3] = gamma_info->vgsp;
    voltage_setting_2_payloads[5] = gamma_info->vgmn_h;
    voltage_setting_2_payloads[6] = gamma_info->vgmn;
    voltage_setting_2_payloads[7] = gamma_info->vgsn;
    memcpy(&gamma_setting_d1_payloads[1], &columbus_wdata[0], 16);
    memcpy(&gamma_setting_d2_payloads[1], &columbus_wdata[16], 16);
    memcpy(&gamma_setting_d3_payloads[1], &columbus_wdata[32], 16);
    memcpy(&gamma_setting_d4_payloads[1], &columbus_wdata[48], 4);
    memcpy(&gamma_setting_e0_payloads[1], &columbus_wdata[52], 16);
    memcpy(&gamma_setting_e1_payloads[1], &columbus_wdata[68], 16);
    memcpy(&gamma_setting_e2_payloads[1], &columbus_wdata[84], 16);
    memcpy(&gamma_setting_e3_payloads[1], &columbus_wdata[100], 4);

    SHDISP_TRACE("out\n");
#endif /* SHDISP_NOT_SUPPORT_GAMMA */
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_columbus_API_diag_get_gamma_info                                   */
/* ------------------------------------------------------------------------- */

static int shdisp_columbus_API_diag_get_gamma_info(struct shdisp_diag_gamma_info *gamma_info)
{
#ifndef SHDISP_NOT_SUPPORT_GAMMA
    int ret = 0;
    int i = 0;
    unsigned char columbus_rdata[52];
    unsigned char columbus_rdata_tmp;

    SHDISP_TRACE("in\n");
    if (gamma_info == NULL){
        SHDISP_ERR("<NULL_POINTER> gamma_info.\n");
        return SHDISP_RESULT_FAILURE;
    }

    ret = MIPI_DSI_COMMAND_TX_COMMIT(cmd_set_page_1_cmds);
    if (ret != SHDISP_RESULT_SUCCESS){
        SHDISP_ERR("<RESULT_FAILURE> cmd_set_page_1_cmds.\n");
        return SHDISP_RESULT_FAILURE;
    }

    memset(columbus_rdata, 0, 52);
    ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,SHDISP_COLUMBUS_GAMMA_D1_REG,&columbus_rdata[0],16);
    if(ret) {
        SHDISP_ERR("<RESULT_FAILURE> read 0xD1 register failure.\n");
        return SHDISP_RESULT_FAILURE;
    }
    ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,SHDISP_COLUMBUS_GAMMA_D2_REG,&columbus_rdata[16],16);
    if(ret) {
        SHDISP_ERR("<RESULT_FAILURE> read 0xD2 register failure.\n");
        return SHDISP_RESULT_FAILURE;
    }
    ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,SHDISP_COLUMBUS_GAMMA_D3_REG,&columbus_rdata[32],16);
    if(ret) {
        SHDISP_ERR("<RESULT_FAILURE> read 0xD3 register failure.\n");
        return SHDISP_RESULT_FAILURE;
    }
    ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,SHDISP_COLUMBUS_GAMMA_D4_REG,&columbus_rdata[48],4);
    if(ret) {
        SHDISP_ERR("<RESULT_FAILURE> read 0xD4 register failure.\n");
        return SHDISP_RESULT_FAILURE;
    }
    for( i = 0; i < 52; i += 2 ) {
        columbus_rdata_tmp = columbus_rdata[i];
        columbus_rdata[i] = columbus_rdata[i+1];
        columbus_rdata[i+1] = columbus_rdata_tmp;
    }
    memcpy(&gamma_info->gamma[0], columbus_rdata, 52);

    memset(columbus_rdata, 0, 52);
    ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,SHDISP_COLUMBUS_GAMMA_E0_REG,&columbus_rdata[0],16);
    if(ret) {
        SHDISP_ERR("<RESULT_FAILURE> read 0xE0 register failure.\n");
        return SHDISP_RESULT_FAILURE;
    }
    ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,SHDISP_COLUMBUS_GAMMA_E1_REG,&columbus_rdata[16],16);
    if(ret) {
        SHDISP_ERR("<RESULT_FAILURE> read 0xE1 register failure.\n");
        return SHDISP_RESULT_FAILURE;
    }
    ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,SHDISP_COLUMBUS_GAMMA_E2_REG,&columbus_rdata[32],16);
    if(ret) {
        SHDISP_ERR("<RESULT_FAILURE> read 0xE2 register failure.\n");
        return SHDISP_RESULT_FAILURE;
    }
    ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,SHDISP_COLUMBUS_GAMMA_E3_REG,&columbus_rdata[48],4);
    if(ret) {
        SHDISP_ERR("<RESULT_FAILURE> read 0xE3 register failure.\n");
        return SHDISP_RESULT_FAILURE;
    }
    for( i = 0; i < 52; i += 2 ) {
        columbus_rdata_tmp = columbus_rdata[i];
        columbus_rdata[i] = columbus_rdata[i+1];
        columbus_rdata[i+1] = columbus_rdata_tmp;
    }
    memcpy(&gamma_info->gamma[26], columbus_rdata, 52);

    memset(columbus_rdata, 0, 11);
    ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,SHDISP_COLUMBUS_GAMMA_B0_REG,&columbus_rdata[0],1);
    if(ret) {
        SHDISP_ERR("<RESULT_FAILURE> read 0xB0 register failure.\n");
        return SHDISP_RESULT_FAILURE;
    }
    ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,SHDISP_COLUMBUS_GAMMA_B1_REG,&columbus_rdata[1],1);
    if(ret) {
        SHDISP_ERR("<RESULT_FAILURE> read 0xB1 register failure.\n");
        return SHDISP_RESULT_FAILURE;
    }
    ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,SHDISP_COLUMBUS_GAMMA_B2_REG,&columbus_rdata[2],1);
    if(ret) {
        SHDISP_ERR("<RESULT_FAILURE> read 0xB2 register failure.\n");
        return SHDISP_RESULT_FAILURE;
    }
    ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,SHDISP_COLUMBUS_GAMMA_B3_REG,&columbus_rdata[3],1);
    if(ret) {
        SHDISP_ERR("<RESULT_FAILURE> read 0xB3 register failure.\n");
        return SHDISP_RESULT_FAILURE;
    }
    ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,SHDISP_COLUMBUS_GAMMA_B4_REG,&columbus_rdata[4],1);
    if(ret) {
        SHDISP_ERR("<RESULT_FAILURE> read 0xB4 register failure.\n");
        return SHDISP_RESULT_FAILURE;
    }
    ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,SHDISP_COLUMBUS_GAMMA_BC_REG,&columbus_rdata[5],3);
    if(ret) {
        SHDISP_ERR("<RESULT_FAILURE> read 0xBC register failure.\n");
        return SHDISP_RESULT_FAILURE;
    }
    ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,SHDISP_COLUMBUS_GAMMA_BD_REG,&columbus_rdata[8],3);
    if(ret) {
        SHDISP_ERR("<RESULT_FAILURE> read 0xBD register failure.\n");
        return SHDISP_RESULT_FAILURE;
    }

    gamma_info->avdd   = columbus_rdata[0];
    gamma_info->avee   = columbus_rdata[1];
    gamma_info->val    = columbus_rdata[2];
    gamma_info->vgh    = columbus_rdata[3];
    gamma_info->vgl    = columbus_rdata[4];
    gamma_info->vgmp_h = columbus_rdata[5];
    gamma_info->vgmp   = columbus_rdata[6];
    gamma_info->vgsp   = columbus_rdata[7];
    gamma_info->vgmn_h = columbus_rdata[8];
    gamma_info->vgmn   = columbus_rdata[9];
    gamma_info->vgsn   = columbus_rdata[10];

    SHDISP_TRACE("out\n");
#endif /* SHDISP_NOT_SUPPORT_GAMMA */
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_columbus_API_diag_set_gamma                                        */
/* ------------------------------------------------------------------------- */

static int shdisp_columbus_API_diag_set_gamma(struct shdisp_diag_gamma *gamma)
{
#ifndef SHDISP_NOT_SUPPORT_GAMMA
    SHDISP_TRACE("in\n");

    SHDISP_TRACE("out\n");
#endif /* SHDISP_NOT_SUPPORT_GAMMA */
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_columbus_API_shutdown                                              */
/* ------------------------------------------------------------------------- */

static int  shdisp_columbus_API_shutdown(void)
{
    SHDISP_TRACE("in\n");

    shdisp_bdic_API_LCD_set_hw_reset();

    SHDISP_TRACE("out\n");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_columbus_API_dump                                                  */
/* ------------------------------------------------------------------------- */

static void shdisp_columbus_API_dump(int type)
{

#define PANEL_DUMP_RECORD(a) {a, sizeof(a)/sizeof(*a)}
    static struct {
        struct shdisp_dsi_cmd_desc * cmds;
        int                          len;
    } const dumpary[] = {
        PANEL_DUMP_RECORD(voltage_setting_1_cmds),
        PANEL_DUMP_RECORD(power_ctl_cmds),
        PANEL_DUMP_RECORD(voltage_setting_2_cmds),
        PANEL_DUMP_RECORD(vcom_setting_cmds),
        PANEL_DUMP_RECORD(output_pin_ctl_cmds),
        PANEL_DUMP_RECORD(gamma_curve_ctl_cmds),
        PANEL_DUMP_RECORD(gamma_setting_pic_adj_cmds),
    };

    int i;
    char readbuf[64];
    const int reclen = sizeof(dumpary)/sizeof(*dumpary);
    int reccnt;
    int cmdcnt;
    int dumpparams = 0;
    struct shdisp_dsi_cmd_desc *cmdpos;
    struct shdisp_dsi_cmd_desc readcmd;
    char payload[2];
    char valstr[(6*64)+1];
    char *pvalstr;
    int  len;
    int res;

    SHDISP_TRACE("in\n");

    printk("[SHDISP] PANEL PARAMETER INFO ->>\n");

    printk("[SHDISP] shdisp_panel_ctx.device_code = %d\n", shdisp_panel_ctx.device_code);
    printk("[SHDISP] shdisp_panel_ctx.vcom       = 0x%04X\n", shdisp_panel_ctx.vcom);
    printk("[SHDISP] shdisp_panel_ctx.vcom_low   = 0x%04X\n", shdisp_panel_ctx.vcom_low);
    printk("[SHDISP] shdisp_panel_ctx.vcom_nvram = 0x%04X\n", shdisp_panel_ctx.vcom_nvram);
    printk("[SHDISP] shdisp_panel_ctx.lcddr_phy_gamma.status = 0x%02X\n", shdisp_panel_ctx.lcddr_phy_gamma.status);
    printk("[SHDISP] shdisp_panel_ctx.lcddr_phy_gamma.buf = ");
    for (i = 0; i < SHDISP_LCDDR_PHY_GAMMA_BUF_MAX; i++) {
        printk("0x%03X,", shdisp_panel_ctx.lcddr_phy_gamma.buf[i]);
    }
    printk("\n");
    printk("[SHDISP] shdisp_panel_ctx.lcddr_phy_gamma.applied_voltage = ");
    for (i = 0; i < SHDISP_LCDDR_APPLIED_VOLTAGE_SIZE; i++) {
        printk("0x%02X,", shdisp_panel_ctx.lcddr_phy_gamma.applied_voltage[i]);
    }
    printk("\n");
    printk("[SHDISP] shdisp_panel_ctx.lcddr_phy_gamma.chksum = 0x%04X\n", shdisp_panel_ctx.lcddr_phy_gamma.chksum);


    readcmd.payload = payload;
    readcmd.dtype = SHDISP_DTYPE_DCS_READ;
    readcmd.dlen   = 2;
    readcmd.wait  = 0;
    payload[1] = 0;

    res = MIPI_DSI_COMMAND_TX(cmd_set_page_1_cmds);
    if (res != SHDISP_RESULT_SUCCESS){
        SHDISP_DEBUG("out1 res=%d\n", res);
    }

    for(reccnt = 0; reccnt < reclen; reccnt++) {
        cmdpos = dumpary[reccnt].cmds;

        for(cmdcnt = 0; cmdcnt < dumpary[reccnt].len; ++cmdcnt, ++cmdpos){
            payload[0] = cmdpos->payload[0];
            memset(readbuf, 0, sizeof(readbuf));
            res = shdisp_panel_API_mipi_dsi_cmds_rx(readbuf, &readcmd, cmdpos->dlen-1);
            if (res != SHDISP_RESULT_SUCCESS) {
                SHDISP_ERR("columbus dump[%02d][%02d](addr=0x%02x) read failure.\n", reccnt, cmdcnt, payload[0]);
                continue;
            }
            pvalstr = valstr;
            len = sizeof(valstr)/sizeof(*valstr);
            for(dumpparams = 0; dumpparams < cmdpos->dlen-1; dumpparams++){
                pvalstr += snprintf(pvalstr, len, " 0x%02x,", readbuf[dumpparams]);
                len = (sizeof(valstr)/sizeof(*valstr)) - (valstr - pvalstr);
            }
            SHDISP_DEBUG("columbus dump[%02d][%02d](addr=0x%02x) = %s\n", reccnt, cmdcnt, payload[0], valstr);

            if( memcmp(readbuf, &cmdpos->payload[1], cmdpos->dlen-1) ){
                pvalstr = valstr;
                len = sizeof(valstr)/sizeof(*valstr);
                for(dumpparams = 0; dumpparams < cmdpos->dlen-1; dumpparams++){
                    pvalstr += snprintf(pvalstr, len, " 0x%02x,", cmdpos->payload[1+dumpparams]);
                    len = (sizeof(valstr)/sizeof(*valstr)) - (valstr - pvalstr);
                }
                SHDISP_DEBUG("error cmds[%02d][%02d](addr=0x%02x) = %s", reccnt, cmdcnt, payload[0], valstr);
            }
        }
    }

    printk("[SHDISP] PANEL PARAMETER INFO <<-\n");
    SHDISP_TRACE("out\n");
}

/* ------------------------------------------------------------------------- */
/* shdisp_andy_API_set_irq                                                   */
/* ------------------------------------------------------------------------- */

static int shdisp_columbus_API_set_irq(int enable)
{
    int funcret = SHDISP_RESULT_SUCCESS;

    int ret;

    SHDISP_TRACE("in (enable=%d)", enable);

    switch (enable) {
    case SHDISP_IRQ_ENABLE:
        ret = shdisp_columbus_set_irq(SHDISP_IRQ_ENABLE);
        if (ret) {
            funcret = ret;
            SHDISP_ERR("failed to enable IRQ. (ret=%d irq=%d enable=%d irqstat=0x%08x)",
                            ret, shdisp_columbus_irq, enable, shdisp_columbus_int_irq_port_status);
            goto exit;
        }
        break;
    case SHDISP_IRQ_DISABLE:
        ret = shdisp_columbus_set_irq(SHDISP_IRQ_DISABLE);
        if (ret) {
            funcret = ret;
            SHDISP_ERR("failed to disable IRQ. (ret=%d irq=%d enable=%d irqstat=0x%08x)",
                            ret, shdisp_columbus_irq, enable, shdisp_columbus_int_irq_port_status);
            goto exit;
        }
        break;
    default:
        funcret = SHDISP_RESULT_FAILURE;
        SHDISP_ERR("invalid argument. (enable=%d)", enable);
        goto exit;
    }

exit:

    SHDISP_TRACE("out (ret=%d)", funcret);
    return funcret;

}
/* ------------------------------------------------------------------------- */
/* shdisp_columbus_mipi_cmd_start_display                                    */
/* ------------------------------------------------------------------------- */

static int shdisp_columbus_mipi_cmd_start_display(void)
{
    int ret = SHDISP_RESULT_SUCCESS;
    SHDISP_TRACE("in\n");
    ret = MIPI_DSI_COMMAND_TX_COMMIT(exit_sleep_cmds);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("exit_sleep_cmds error!!\n");
        return ret;
    }

    SHDISP_TRACE("out\n");
    return ret;
    }

/* ------------------------------------------------------------------------- */
/* shdisp_columbus_mipi_cmd_post_video_start                                    */
/* ------------------------------------------------------------------------- */

static int shdisp_columbus_mipi_cmd_post_video_start(void)
{
    int ret = SHDISP_RESULT_SUCCESS;
    SHDISP_TRACE("in\n");
    ret = MIPI_DSI_COMMAND_TX_COMMIT(disp_on_cmds);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("disp_on_cmds error!!\n");
        return ret;
    }
    SHDISP_TRACE("out\n");

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_columbus_mipi_cmd_lcd_off                                          */
/* ------------------------------------------------------------------------- */

static int shdisp_columbus_mipi_cmd_lcd_off(void)
{
    int ret = SHDISP_RESULT_SUCCESS;

    SHDISP_TRACE("in\n");
    ret = MIPI_DSI_COMMAND_TX(disp_off_cmds);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("disp_off_cmds error!!\n");
        return ret;
    }
    ret = MIPI_DSI_COMMAND_TX_COMMIT(enter_sleep_cmds);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("enter_sleep_cmds error!!\n");
    }
    ret = MIPI_DSI_COMMAND_TX_COMMIT(deep_standby_cmds);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("deep_standby_cmds error!!\n");
    }
    SHDISP_TRACE("out\n");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_columbus_mipi_cmd_lcd_on                                           */
/* ------------------------------------------------------------------------- */

static int shdisp_columbus_mipi_cmd_lcd_on(void)
{
    int ret = SHDISP_RESULT_SUCCESS;

    SHDISP_TRACE("in\n");

    ret = MIPI_DSI_COMMAND_TX(cmd_set_page_1_cmds);
    if (ret != SHDISP_RESULT_SUCCESS){
        SHDISP_DEBUG("out1 ret=%d\n", ret);
        return ret;
    }
    ret = MIPI_DSI_COMMAND_TX(voltage_setting_1_cmds);
    if (ret != SHDISP_RESULT_SUCCESS){
        SHDISP_DEBUG("out2 ret=%d\n", ret);
        return ret;
    }
    ret = MIPI_DSI_COMMAND_TX(power_ctl_cmds);
    if (ret != SHDISP_RESULT_SUCCESS){
        SHDISP_DEBUG("out3 ret=%d\n", ret);
        return ret;
    }
    ret = MIPI_DSI_COMMAND_TX(voltage_setting_2_cmds);
    if (ret != SHDISP_RESULT_SUCCESS){
        SHDISP_DEBUG("out4 ret=%d\n", ret);
        return ret;
    }
    ret = MIPI_DSI_COMMAND_TX(vcom_setting_cmds);
    if (ret != SHDISP_RESULT_SUCCESS){
        SHDISP_DEBUG("out5 ret=%d\n", ret);
        return ret;
    }
    ret = MIPI_DSI_COMMAND_TX_COMMIT(output_pin_ctl_cmds);
    if (ret != SHDISP_RESULT_SUCCESS){
        SHDISP_DEBUG("out6 ret=%d\n", ret);
        return ret;
    }
    ret = MIPI_DSI_COMMAND_TX(gamma_curve_ctl_cmds);
    if (ret != SHDISP_RESULT_SUCCESS){
        SHDISP_DEBUG("out7 ret=%d\n", ret);
        return ret;
    }
    ret = MIPI_DSI_COMMAND_TX_COMMIT(gamma_setting_pic_adj_cmds);
    if (ret != SHDISP_RESULT_SUCCESS){
        SHDISP_DEBUG("out8 ret=%d\n", ret);
        return ret;
    }
    ret = MIPI_DSI_COMMAND_TX(cmd_set_page_0_cmds);
    if (ret != SHDISP_RESULT_SUCCESS){
        SHDISP_DEBUG("out9 ret=%d\n", ret);
        return ret;
    }
    ret = MIPI_DSI_COMMAND_TX_COMMIT(disp_setting_1_2_cmds);
    if (ret != SHDISP_RESULT_SUCCESS){
        SHDISP_DEBUG("out10 ret=%d\n", ret);
        return ret;
    }
    SHDISP_TRACE("out ret=%d\n", ret);
    return ret;
}


/* ------------------------------------------------------------------------- */
/* shdisp_columbus_init_phy_gamma                                            */
/* ------------------------------------------------------------------------- */

int shdisp_columbus_init_phy_gamma(struct shdisp_lcddr_phy_gamma_reg *phy_gamma)
{

    int ret = 0;
#ifndef SHDISP_NOT_SUPPORT_GAMMA
    int cnt;
    int checksum;

    SHDISP_TRACE("in\n");

    if(phy_gamma == NULL) {
        SHDISP_ERR("phy_gamma is NULL.\n");
        ret = -1;
    }
    else if(phy_gamma->status != SHDISP_LCDDR_GAMMA_STATUS_OK) {
        SHDISP_ERR("gammg status invalid. status=%02x\n", phy_gamma->status);
        ret = -1;
    }
    else {
        checksum = phy_gamma->status;
        for(cnt = 0; cnt < SHDISP_LCDDR_PHY_GAMMA_BUF_MAX; cnt++) {
            checksum = checksum + phy_gamma->buf[cnt];
        }
        for(cnt = 0; cnt < SHDISP_LCDDR_APPLIED_VOLTAGE_SIZE; cnt++) {
            checksum = checksum + phy_gamma->applied_voltage[cnt];
        }
        if((checksum & 0xFFFF) != phy_gamma->chksum) {
            pr_err("%s: gammg chksum NG. chksum=%04x calc_chksum=%04x\n", __func__, phy_gamma->chksum, (checksum & 0xFFFF));
            ret = -1;
        }
        else {
            for(cnt = 0; cnt < 8; cnt++) {
                gamma_setting_d1_payloads[cnt*2 + 1]  = (unsigned char)(phy_gamma->buf[cnt] >> 8);
                gamma_setting_d1_payloads[cnt*2 + 2]  = (unsigned char)(phy_gamma->buf[cnt] & 0x00FF);
                gamma_setting_d2_payloads[cnt*2 + 1]  = (unsigned char)(phy_gamma->buf[cnt + 8] >> 8);
                gamma_setting_d2_payloads[cnt*2 + 2]  = (unsigned char)(phy_gamma->buf[cnt + 8] & 0x00FF);
                gamma_setting_d3_payloads[cnt*2 + 1]  = (unsigned char)(phy_gamma->buf[cnt + 16] >> 8);
                gamma_setting_d3_payloads[cnt*2 + 2]  = (unsigned char)(phy_gamma->buf[cnt + 16] & 0x00FF);
                gamma_setting_e0_payloads[cnt*2 + 1]  = (unsigned char)(phy_gamma->buf[cnt + 26] >> 8);
                gamma_setting_e0_payloads[cnt*2 + 2]  = (unsigned char)(phy_gamma->buf[cnt + 26] & 0x00FF);
                gamma_setting_e1_payloads[cnt*2 + 1]  = (unsigned char)(phy_gamma->buf[cnt + 34] >> 8);
                gamma_setting_e1_payloads[cnt*2 + 2]  = (unsigned char)(phy_gamma->buf[cnt + 34] & 0x00FF);
                gamma_setting_e2_payloads[cnt*2 + 1]  = (unsigned char)(phy_gamma->buf[cnt + 42] >> 8);
                gamma_setting_e2_payloads[cnt*2 + 2]  = (unsigned char)(phy_gamma->buf[cnt + 42] & 0x00FF);

            }
            for(cnt = 0; cnt < 2; cnt++) {
                gamma_setting_d4_payloads[cnt*2 + 1]  = (unsigned char)(phy_gamma->buf[cnt + 24] >> 8);
                gamma_setting_d4_payloads[cnt*2 + 2]  = (unsigned char)(phy_gamma->buf[cnt + 24] & 0x00FF);
                gamma_setting_e3_payloads[cnt*2 + 1]  = (unsigned char)(phy_gamma->buf[cnt + 50] >> 8);
                gamma_setting_e3_payloads[cnt*2 + 2]  = (unsigned char)(phy_gamma->buf[cnt + 50] & 0x00FF);
            }
            cnt = 0;
            voltage_setting_1_payloads[1]  = phy_gamma->applied_voltage[cnt++];
            voltage_setting_1_payloads[3]  = phy_gamma->applied_voltage[cnt++];
            voltage_setting_1_payloads[5]  = phy_gamma->applied_voltage[cnt++];
            voltage_setting_1_payloads[7]  = phy_gamma->applied_voltage[cnt++];
            voltage_setting_1_payloads[9]  = phy_gamma->applied_voltage[cnt++];
            voltage_setting_2_payloads[1]  = phy_gamma->applied_voltage[cnt++];
            voltage_setting_2_payloads[2]  = phy_gamma->applied_voltage[cnt++];
            voltage_setting_2_payloads[3]  = phy_gamma->applied_voltage[cnt++];
            voltage_setting_2_payloads[5]  = phy_gamma->applied_voltage[cnt++];
            voltage_setting_2_payloads[6]  = phy_gamma->applied_voltage[cnt++];
            voltage_setting_2_payloads[7]  = phy_gamma->applied_voltage[cnt++];
        }
    }

    SHDISP_TRACE("out ret=%04x\n", ret);
#endif /* SHDISP_NOT_SUPPORT_GAMMA */
    return ret;
}

/*---------------------------------------------------------------------------*/
/*shdisp_columbus_request_irq                                                */
/*---------------------------------------------------------------------------*/

static int shdisp_columbus_request_irq(void)
{
    int rc;

    SHDISP_TRACE("in\n");

    rc = devm_request_irq(&shdisp_columbus_int_irq_port_pdev->dev, shdisp_columbus_irq, shdisp_columbus_int_isr,
                                                                IRQF_TRIGGER_RISING | IRQF_DISABLED,
                                                                "shdisp_columbus", NULL);
    if (rc) {
        SHDISP_ERR("request_irq() failed. irq = 0x%x\n", shdisp_columbus_irq);
    } else {
        shdisp_columbus_int_irq_port_status = SHDISP_IRQ_ENABLE;
        if (shdisp_api_get_boot_disp_status() == SHDISP_MAIN_DISP_ON) {
            shdisp_columbus_set_irq(SHDISP_IRQ_ENABLE);
        } else {
            shdisp_columbus_set_irq(SHDISP_IRQ_DISABLE);
        }
    }

    SHDISP_TRACE("out rc = %d\n", rc);

    return rc;
}

/*---------------------------------------------------------------------------*/
/* shdisp_workqueue_handler_columbus                                         */
/*---------------------------------------------------------------------------*/

static void shdisp_workqueue_handler_columbus(struct work_struct *work)
{
#ifdef SHDISP_RESET_LOG
    struct shdisp_dbg_error_code err_code;
#endif /* SHDISP_RESET_LOG */

    SHDISP_TRACE("in\n");

    shdisp_semaphore_start();

    SHDISP_ERR("MIPI Error\n");
#ifdef SHDISP_RESET_LOG
    err_code.mode = SHDISP_DBG_MODE_LINUX;
    err_code.type = SHDISP_DBG_TYPE_PANEL;
    err_code.code = SHDISP_DBG_CODE_ERROR_DETECT;
    err_code.subcode = SHDISP_DBG_SUBCODE_ESD_MIPI;
    shdisp_dbg_api_err_output(&err_code, 0);
    shdisp_dbg_set_subcode(SHDISP_DBG_SUBCODE_ESD_MIPI);
#endif /* SHDISP_RESET_LOG */

    if (shdisp_api_do_lcd_det_recovery() != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("recovery request error!!\n");
    } else {
        shdisp_columbus_set_irq(SHDISP_IRQ_ENABLE);
    }

    shdisp_semaphore_end(__func__);

    SHDISP_TRACE("out\n");

    wake_unlock(&shdisp_columbus_wake_lock);
}

/*---------------------------------------------------------------------------*/
/* shdisp_columbus_int_isr                                                   */
/*---------------------------------------------------------------------------*/

static irqreturn_t shdisp_columbus_int_isr(int irq_num, void *data)
{
    unsigned long flags;
    int ret;

    SHDISP_TRACE("in\n");

    shdisp_columbus_set_irq(SHDISP_IRQ_DISABLE);

    spin_lock_irqsave(&shdisp_columbus_spin_lock, flags);
    if (shdisp_wq_columbus) {
        wake_lock(&shdisp_columbus_wake_lock);
        ret = queue_work(shdisp_wq_columbus, &shdisp_wq_columbus_wk);
        if (ret == 0) {
            wake_unlock(&shdisp_columbus_wake_lock);
            SHDISP_DEBUG("queue_work failed.\n");
        }
    }
    spin_unlock_irqrestore(&shdisp_columbus_spin_lock, flags);

    SHDISP_TRACE("out\n");

    return IRQ_HANDLED;
}

/*---------------------------------------------------------------------------*/
/*      shdisp_columbus_set_irq                                              */
/*---------------------------------------------------------------------------*/

static int shdisp_columbus_set_irq(int enable)
{
    int ret = SHDISP_RESULT_SUCCESS;
    unsigned long flags = 0;
    SHDISP_TRACE("in enable=%d.\n", enable);

    spin_lock_irqsave(&shdisp_columbus_set_irq_spinlock, flags);

    if (enable == shdisp_columbus_int_irq_port_status) {
        spin_unlock_irqrestore(&shdisp_columbus_set_irq_spinlock, flags);
        return SHDISP_RESULT_SUCCESS;
    }

    if (enable == SHDISP_IRQ_ENABLE) {
        enable_irq(shdisp_columbus_irq);
        shdisp_columbus_int_irq_port_status = enable;
    } else if (enable == SHDISP_IRQ_DISABLE) {
        disable_irq_nosync(shdisp_columbus_irq);
        shdisp_columbus_int_irq_port_status = enable;
    } else {
        SHDISP_ERR("<INVALID_VALUE> enable=%d.\n", enable);
        ret = SHDISP_RESULT_FAILURE;
    }
    spin_unlock_irqrestore(&shdisp_columbus_set_irq_spinlock, flags);
    SHDISP_TRACE("out\n");
    return ret;
}

/* ------------------------------------------------------------------------- */
/*      shdisp_columbus_probe                                                */
/* ------------------------------------------------------------------------- */

static int shdisp_columbus_probe(struct platform_device *pdev)
{
#ifdef CONFIG_OF
    struct resource *res;
    int rc = 0;

    SHDISP_TRACE("in pdev = 0x%p\n", pdev );

    if (pdev) {
        res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
        if (!res) {
            SHDISP_ERR("irq resouce err!!\n");
            rc = 0;
            goto probe_done;
        } else {
            shdisp_columbus_irq = res->start;
            shdisp_columbus_int_irq_port_pdev = pdev;
        }
        rpm_vdd_vreg = rpm_regulator_get(&(pdev->dev),"vdd");
        rc = PTR_RET(rpm_vdd_vreg);
        if(rc){
            SHDISP_ERR("%s: rpm_vdd_vreg get failed. rc=%d\n", __func__,rc);
            rpm_vdd_vreg = NULL;
        }

    }

probe_done:
    SHDISP_TRACE("out rc = %d\n", rc);

    return rc;
#else
    return SHDISP_RESULT_SUCCESS;
#endif /* CONFIG_OF */
}

/* ------------------------------------------------------------------------- */
/*      shdisp_columbus_remove                                               */
/* ------------------------------------------------------------------------- */

static int shdisp_columbus_remove(struct platform_device *pdev)
{
    return SHDISP_RESULT_SUCCESS;
}

#ifdef CONFIG_OF
static const struct of_device_id shdisp_columbus_dt_match[] = {
    { .compatible = "sharp,shdisp_columbus", },
    {}
};
#endif /* CONFIG_OF */

static struct platform_driver shdisp_columbus_driver = {
    .probe = shdisp_columbus_probe,
    .remove = shdisp_columbus_remove,
    .shutdown = NULL,
    .driver = {
        /*
         * Driver name must match the device name added in
         * platform.c.
         */
        .name = "shdisp_columbus",
#ifdef CONFIG_OF
        .of_match_table = shdisp_columbus_dt_match,
#endif /* CONFIG_OF */
    },
};

/* ------------------------------------------------------------------------- */
/*      shdisp_columbus_register_driver                                      */
/* ------------------------------------------------------------------------- */

static int shdisp_columbus_register_driver(void)
{
    SHDISP_TRACE("\n");
    return platform_driver_register(&shdisp_columbus_driver);
}


MODULE_DESCRIPTION("SHARP DISPLAY DRIVER MODULE");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.00");

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
