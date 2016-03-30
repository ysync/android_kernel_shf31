/* drivers/sharp/shtrpd/shtrpd.h  (Trackpad driver)
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
#ifndef SHTRPD_H
#define SHTRPD_H

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/earlysuspend.h>

/*  i2c slave device address    */
#define SHTRPD_ADDR 0x74

/*  Definitions for the driver  */
#define IQS_MAJOR               248
#define IQS_MINOR               0
#define IQS_DEV_NUMS            1
#define IQS_NUM_RW_REGS         26
#define DEVICE_NAME             "shtrpd"

/*  Definitions of Address-commands implemented on SHTRPD   */
#define VERSION_INFO        0x00    /*  Read    */
#define XY_DATA             0x01    /*  Read    */
#define PROX_STATUS         0x02    /*  Read    */
#define TOUCH_STATUS        0x03    /*  Read    */
#define COUNT_VALUES        0x04    /*  Read    */
#define LTA_VALUES          0x05    /*  Read    */
#define ATI_COMP            0x06    /*  Read / Write    */
#define PORT_CONTROL        0x07    /*  Write   */
#define SNAP_STATUS         0x08    /*  Read    */

#define CONTROL_SETTINGS    0x10    /*  Write   */
#define THRESHOLD_SETTINGS  0x11    /*  Write   */
#define ATI_SETTINGS        0x12    /*  Write   */
#define FILTER_SETTINGS     0x13    /*  Write   */
#define TIMING_SETTINGS     0x14    /*  Write   */
#define CHANNEL_SETUP       0x15    /*  Write   */
#define HW_CONFIG_SETTINGS  0x16    /*  Write   */
#define ACTIVE_CHANNELS     0x17    /*  Write   */
#define DB_SETTINGS         0x18    /*  Write   */

#define PM_PROX_STATUS      0x20    /*  Read    */
#define PM_COUNT_VALUES     0x21    /*  Read    */
#define PM_LTA_VALUES       0x22    /*  Read    */
#define PM_ATI_COMP         0x23    /*  Read / Write    */
#define PM_ATI_SETTINGS     0x24    /*  Write   */

/*  BIT DEFINITIONS FOR SHTRPD  */
/*  XYInfoByte0 */
/*  Indicates how many co-ordinates are available   */
#define NO_OF_FINGERS0      0x01
/*  Indicates how many co-ordinates are available   */
#define NO_OF_FINGERS1      0x02
/*  Indicates how many co-ordinates are available   */
#define NO_OF_FINGERS2      0x04
/*  Determine how many touches occured  */
#define NO_OF_FINGERS       0x07
/*  0 = no snap outputs / 1 = at least one snap output  */
#define SNAP_OUTPUT         0x08
/*  0 = Charging full-speed  /  1 = Charging in LP duty cycle   */
#define LP_STATUS           0x10
/*  0 = No ATI Errors  /  1 = ATI Error    */
#define ATI_ERROR           0x20
/*  0 = Normal Charging  /  1 = ProxMode charging   */
#define MODE_INDICATOR      0x40
/*  Indicates reset has occurred    */
#define SHOW_RESET          0x80

/*  Bit definitions - ControlSettings0  */
/*  0= no event mode / 1=event mode active  */
#define EVENT_MODE          0x01
/*  Reseed all the normal mode channels */
#define TRACKPAD_RESEED     0x02
/*  Perform AutoATI routine (depend on Mode selected)   */
#define AUTO_ATI            0x04
/*  0 = Normal Mode  /  1 = ProxMode    */
#define MODE_SELECT         0x08
/*  Reseed the Prox Mode channels   */
#define PM_RESEED           0x10

/*  0 = Normal/PM manual, 1=Auto switch between NM and PM   */
#define AUTO_MODES          0x40
/*  clear the SHOW_RESET flag   */
#define ACK_RESET           0x80

/*  Bit definitions - ControlSettings1  */
/*  0= snaps calculated / 1=not calculated  */
#define SNAP_EN             0x01
/*  0= normal power charging 1=low power charging   */
#define LOW_POWER           0x02
/*  0= no sleep added / 1=permanent sleep time added    */
#define SLEEP_EN            0x04
/*  0= disabled (conventional prox detection) /
 *  1=enabled (prox trips both ways)
 */
#define REVERSE_EN          0x08
/*  0 = PMProx event enabled / 1 = enabled for EventMode    */
#define DIS_PMPROX_EVENT    0x10
/*  0 = Snap event enabled / 1 = disabled for EventMode */
#define DIS_SNAP_EVENT      0x20
/* 0 = Touch event enabled / 1 = disabled for EventMode */
#define DIS_TOUCH_EVENT     0x40

/*  Bit definitions - ControlSettings2  */
#define NM_RE_ATI           0x01
#define PM_RE_ATI           0x02
#define NM_LTA_HALT         0x04
#define PM_LTA_HALT         0x08
#define ATIC_ADJ_DISABLE    0x10

/*  Bit definitions - FilterSettings0   */
/*  0=enabled    1=disabled */
#define DIS_IIR_FILTER      0x01
/*  0=enabled    1=disabled */
#define DIS_MAV_FILTER      0x02
/*  0=Dynamic filter  1=fixed beta  */
#define SELECT_TOUCH_FILTER 0x04
/*  0 = CS filtered in PM 1= CS raw in PM   */
#define DIS_PM_FILTER       0x08

/*  Bit definitions - PMSetup0  */
/*  Decimal value selects an INDIVIDUAL Rx for ProxMode */
#define RX_SELECT0          0x01
#define RX_SELECT1          0x02
#define RX_SELECT2          0x04
#define RX_SELECT3          0x08

/*  0 = RxA  /  1 = RxB */
#define RX_GROUP            0x40
/*  0 = Projected  /  1 = Self / surface    */
#define CHARGE_TYPE         0x80

/*  Bit definitions - ProxSettings0 */
/*  0 = noise detection disabled / 1 = noise detection enabled  */
#define NOISE_EN            0x20

/*  max number of touches   */
#define MAX_TOUCHES         5

/* Structure to keep track of all the touches   */
struct shtrpd_fingers_structure {
    /*  the X-coordinate of the reported touch  */
    u16 XPos;
    /*  the Y-coordinate of the reported touch  */
    u16 YPos;
    /*  the previous X-coordinate for filters   */
    u16 prev_x;
    /*  the previous Y-coordinate for filters   */
    u16 prev_y;
    /*  the 'hardness' of the touch */
    u16 touchStrength;
    /*  the 'area' of the touch */
    u8  area;
};

#define RESET_DISABLE       0
#define RESET_ENABLE        1

#endif
