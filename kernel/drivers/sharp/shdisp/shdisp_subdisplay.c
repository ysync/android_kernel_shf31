/* drivers/sharp/shdisp/shdisp_subdisplay.c  (Display Driver)
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
#include "shdisp_kerl_priv.h"
#include "shdisp_pm.h"
#include "shdisp_panel.h"
#include "shdisp_columbus.h"
#include "shdisp_system.h"
#include "shdisp_bdic.h"
#include "shdisp_type.h"
#include "shdisp_dbg.h"
#include "shdisp_subdisplay.h"


/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */
static int shdisp_subdisplay_cmds_initial_setting(void);

/* ------------------------------------------------------------------------- */
/* VARIABLES                                                                 */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/* API                                                                       */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/* shdisp_subdisplay_API_power_on                                            */
/* ------------------------------------------------------------------------- */

int shdisp_subdisplay_API_power_on(void)
{
    SHDISP_TRACE("in\n");

    shdisp_pm_bdic_power_manager(SHDISP_DEV_TYPE_SUBDISPLAY, SHDISP_DEV_STATE_ON);
    shdisp_bdic_API_subdisplay_release_hw_reset();

    SHDISP_TRACE("out\n");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_subdisplay_API_power_off                                           */
/* ------------------------------------------------------------------------- */

int shdisp_subdisplay_API_power_off(void)
{
    SHDISP_TRACE("in\n");
    shdisp_pm_bdic_power_manager(SHDISP_DEV_TYPE_SUBDISPLAY, SHDISP_DEV_STATE_OFF);

    SHDISP_TRACE("out\n");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_subdisplay_API_disp_on                                             */
/* ------------------------------------------------------------------------- */

int shdisp_subdisplay_API_disp_on(void)
{
    unsigned char buf[SHDISP_SUBDISPLAY_HEIGHT_BUFFER_SIZE][SHDISP_SUBDISPLAY_WIDTH_BUFFER_SIZE];
    SHDISP_TRACE("in\n");
    memset(buf,0,sizeof(buf));
    shdisp_subdisplay_cmds_initial_setting();
    shdisp_subdisplay_API_update_image(&buf[0][0]);
    shdisp_bdic_API_subdisplay_power_on();
    shdisp_SYS_delay_us(4000);
    shdisp_SYS_subdisplay_spi_command_set(DDIPON_OFF,0x01);
    SHDISP_TRACE("out\n");
    return SHDISP_RESULT_SUCCESS;

}

/* ------------------------------------------------------------------------- */
/* shdisp_subdisplay_API_disp_off                                            */
/* ------------------------------------------------------------------------- */

int shdisp_subdisplay_API_disp_off(void)
{
    SHDISP_TRACE("in\n");
    shdisp_SYS_subdisplay_spi_command_set(DDIPON_OFF,0x00);
    shdisp_SYS_subdisplay_spi_command_set(DSTBYON,0x01);
    shdisp_bdic_API_subdisplay_power_off();
    shdisp_bdic_API_subdisplay_set_hw_reset();
    SHDISP_TRACE("out\n");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_subdisplay_API_update_image                                        */
/* ------------------------------------------------------------------------- */

int shdisp_subdisplay_API_update_image(unsigned char *data)
{
    SHDISP_TRACE("in\n");
    shdisp_SYS_subdisplay_spi_transfer_data(data,SHDISP_SUBDISPLAY_HEIGHT_BUFFER_SIZE * SHDISP_SUBDISPLAY_WIDTH_BUFFER_SIZE);
    SHDISP_TRACE("out\n");
    return SHDISP_RESULT_SUCCESS;
}
/* ------------------------------------------------------------------------- */
/* Logical Driver                                                            */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_subdisplay_cmds_initial_setting                                    */
/* ------------------------------------------------------------------------- */

static int shdisp_subdisplay_cmds_initial_setting(void)
{
    unsigned char data_x[2];
    unsigned char data_y[2];
    data_x[0] = 0x00;
    data_x[1] = 0x7F;
    data_y[0] = 0x06;
    data_y[1] = 0x1D;

    SHDISP_TRACE("in\n");
    shdisp_SYS_subdisplay_spi_command_set(SOFTRES,0x00);
    shdisp_SYS_subdisplay_spi_command_set(DDIPON_OFF,0x00);
    shdisp_SYS_subdisplay_spi_command_set(DISPDIRECTION,0x00);
    shdisp_SYS_subdisplay_spi_command_set(RESERVED1,0x00);
    shdisp_SYS_subdisplay_spi_command_set(RESERVED2,0x24);
    shdisp_SYS_subdisplay_spi_command_set(RESERVED3,0x00);
    shdisp_SYS_subdisplay_spi_command_set(DSTBYON,0x00);
    shdisp_SYS_subdisplay_spi_command_set(RESERVED4,0x00);
    shdisp_SYS_subdisplay_spi_command_set(RESERVED5,0x00);
    shdisp_SYS_subdisplay_spi_command_set(RESERVED6,0x07);
    shdisp_SYS_subdisplay_spi_command_set(RESERVED7,0x05);
    shdisp_SYS_subdisplay_spi_command_set(RESERVED8,0x00);
    shdisp_SYS_subdisplay_spi_command_set(WRITEDIRECTION,0x0B);
    shdisp_SYS_subdisplay_spi_multi_command_set(DISPSIZEX,data_x,2);
    shdisp_SYS_subdisplay_spi_multi_command_set(DISPSIZEY,data_y,2);
    shdisp_SYS_subdisplay_spi_command_set(XBOXADRSSTART,0x00);
    shdisp_SYS_subdisplay_spi_command_set(XBOXADRSEND,0x0F);
    shdisp_SYS_subdisplay_spi_command_set(YBOXADRSSTART,0x06);
    shdisp_SYS_subdisplay_spi_command_set(YBOXADRSEND,0x1D);
    shdisp_SYS_subdisplay_spi_command_set(XDISPSTART,0x00);
    shdisp_SYS_subdisplay_spi_command_set(YDISPSTART,0x00);
    shdisp_SYS_subdisplay_spi_command_set(RESERVED9,0x03);
    shdisp_SYS_subdisplay_spi_command_set(S_STEPTIMER,0x00);
    shdisp_SYS_subdisplay_spi_command_set(S_STEPUNIT,0x00);
    shdisp_SYS_subdisplay_spi_command_set(S_CONDITION,0x00);
    shdisp_SYS_subdisplay_spi_command_set(S_START_STOP,0x00);
    shdisp_SYS_subdisplay_spi_command_set(SCLK,0x81);
    shdisp_SYS_subdisplay_spi_command_set(DDEN,0x00);
    shdisp_SYS_subdisplay_spi_command_set(RESERVED10,0x00);
    shdisp_SYS_subdisplay_spi_command_set(FDIM,0x0F);
    shdisp_SYS_subdisplay_spi_command_set(RESERVED11,0x86);
    SHDISP_TRACE("out\n");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_subdisplay_probe                                                   */
/* ------------------------------------------------------------------------- */

static int shdisp_subdisplay_probe(struct platform_device *pdev)
{
#ifdef CONFIG_OF

    SHDISP_TRACE("in pdev = 0x%p\n", pdev );
    SHDISP_TRACE("out\n");

    return SHDISP_RESULT_SUCCESS;
#else
    return SHDISP_RESULT_SUCCESS;
#endif /* CONFIG_OF */
}

/* ------------------------------------------------------------------------- */
/*      shdisp_subdisplay_remove                                             */
/* ------------------------------------------------------------------------- */

static int shdisp_subdisplay_remove(struct platform_device *pdev)
{
    return SHDISP_RESULT_SUCCESS;
}

#ifdef CONFIG_OF
static const struct of_device_id shdisp_subdisplay_dt_match[] = {
    { .compatible = "sharp,shdisp_subdisplay", },
    {}
};
#endif /* CONFIG_OF */

static struct platform_driver shdisp_subdisplay_driver = {
    .probe = shdisp_subdisplay_probe,
    .remove = shdisp_subdisplay_remove,
    .shutdown = NULL,
    .driver = {
        /*
         * Driver name must match the device name added in
         * platform.c.
         */
        .name = "shdisp_subdisplay",
#ifdef CONFIG_OF
        .of_match_table = shdisp_subdisplay_dt_match,
#endif /* CONFIG_OF */
    },
};

/* ------------------------------------------------------------------------- */
/* shdisp_subdisplay_API_init                                                */
/* ------------------------------------------------------------------------- */

void shdisp_subdisplay_API_init(void)
{
    platform_driver_register(&shdisp_subdisplay_driver);
}
