/* drivers/sharp/shtrpd/shtrpd.c  (Trackpad driver)
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

//#define SHTRPD_DBG_INITIAL_BOOT_CHK

#include <linux/delay.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/miscdevice.h>
#include <linux/ioctl.h>
#include <linux/wakelock.h>
#include <asm/uaccess.h>
#include "sharp/shflip_dev.h"
#include "sharp/sh_boot_manager.h"

#include "shtrpd.h"
#include "shtrpd_Init.h"
#include "shtrpd_HEX_Array.h"
#include "shtrpd_keycode.h"

/* ------------------------------------------------------------------------- */
/* DEFINE                                                                    */
/* ------------------------------------------------------------------------- */
#ifndef SHTRPD_FACTORY_MODE_ENABLE
#define SHTRPD_ENABLE_INIT_FW_DL
#define SHTRPD_ENABLE_INIT_FW_DL_CRC_CHECK
#endif /* SHTRPD_FACTORY_MODE_ENABLE */

#define SHTRPD_DRAG_SMOOTH_ENABLE
#define SHTRPD_LOG_DEBUG_ENABLE
#define SHTRPD_TOUCH_UP_DELAY_ENABLE
#define SHTRPD_TOUCH_DOWN_THRESHOLD_ENABLE
#define SHTRPD_EDGE_FAIL_TOUCH_REJECTION_ENABLE
#define SHTRPD_EDGE_TD_MASK_ENABLE
#define SHTRPD_TOP_OF_SCREEN_GHOST_REJECTION_ENABLE
#define SHTRPD_TU_JITTER_FILTER_ENABLE
#define SHTRPD_CPU_IDLE_SLEEP_CONTROL_ENABLE
#define SHTRPD_CPU_CLOCK_CONTROL_ENABLE
#define SHTRPD_DRUMMING_SPLIT_ENABLE
#define SHTRPD_TAP_JITTER_FILTER_ENABLE
#define SHTRPD_GRIP_FAIL_FLICK_REJECTION_ENABLE
#define SHTRPD_SPLIT_FINGER_REJECTION_ENABLE
#define SHTRPD_SNAP_MASK
#define SHTRPD_FAIL_TOUCH_CLEAR_ENABLE
#define SHTRPD_NOINTERRUPT_IN_TOUCH_CHECK_ENABLE
#define SHTRPD_INTERRUPT_TIMING_CHECK_ENABLE
#define SHTRPD_SNAP_USER_MASK
#define SHTRPD_CHECK_CPU_TEMP_ENABLE
#define SHTRPD_INTOUCH_GHOST_REJECTION_ENABLE

#if defined(SHTRPD_TOUCH_UP_DELAY_ENABLE)
	#define SHTRPD_FAIL_FLICK_RECOVER_ENABLE
#endif /* SHTRPD_TOUCH_UP_DELAY_ENABLE */

/*  Device specific resolutions */
#define SHTRPD_MAX_X            540	//2048
#define SHTRPD_MAX_Y            960	//3584

#define SHTRPD_NOTIFY_MAX_X     2048
#define SHTRPD_NOTIFY_MAX_Y     3584

#define SHTRPD_NOTIFY_POS_SCALE_X(x) \
	((x * 3792) / 1000) /* scale factor (2048 / 540 ) = 3.792592593 */

#define SHTRPD_NOTIFY_POS_SCALE_Y(y) \
	((y * 3733) / 1000) /* scale factor (3584 / 960 ) = 3.733333333 */


/*  Pins used in this setup */
#define RDY_LINE_PIN            119
#define NRST_LINE_PIN           120

/*
 *  36 bytes to read form the XY Status register -
 *  all the necessary info resides here
 */
#define SHTRPD_BLOCK_SIZE       40

#define MAX_SNAPS               (SHTRPD_XY_SNAP_NUM * 8)

#define SHTRPDIO   0x5F
#define SHTRPDIO_GET_FW_VERSION _IOR(SHTRPDIO , 5 , int)
#define SHTRPDIO_FW_UPDATE      _IO (SHTRPDIO , 6)
#define SHTRPDIO_WRITE_SENSOR   _IOW(SHTRPDIO , 7 , int)
#define SHTRPDIO_READ_SENSOR    _IOR(SHTRPDIO , 8 , int)
#define SHTRPDIO_WRITE_IMAGE    _IOW(SHTRPDIO , 9 , struct shtrpd_ioctl_fwparam)
#define SHTRPDIO_RESET          _IOW(SHTRPDIO , 10, struct shtrpd_ioctl_resetparam)
#define SHTRPDIO_GET_COUNTVALUE _IOWR(SHTRPDIO, 11, struct shtrpd_ioctl_countval)
#define SHTRPDIO_PROPERTY       _IOWR(SHTRPDIO, 12, struct shtrpd_ioctl_properties)
#define SHTRPDIO_SET_CHARGERARMOR _IOW (SHTRPDIO, 13 , int)
#define SHTRPDIO_SET_TOUCHEVENT _IOW (SHTRPDIO, 14 , int)
#define SHTRPDIO_SET_REATI      _IO  (SHTRPDIO,  15)
#define SHTRPDIO_SET_KEYMODE    _IOW (SHTRPDIO, 16, int)
#define SHTRPDIO_SET_TOUCHCRUISER _IOW ( SHTRPDIO,  17, int)

#define SHTRPD_SNAP_OFF_ON      1
#define SHTRPD_SNAP_ON_ON       2
#define SHTRPD_SNAP_ON_OFF      3

#define	SHTRPD_CMD_RESPONSE     0xA3
#define	SHTRPD_FW_VER_SIZE      10

#define RE_ATI_RANGE                3
#define PM_RE_ATI_RANGE             4
#define PMRESEED_TIME               16

#define SHTRPD_NRST_DELAY               10000
#define SHTRPD_EXEC_APP_DELAY           20000
#define SHTRPD_HANDSHAKE_DELAY          300000
#define SHTRPD_ATIC_ADJ_DELAY           1
#define	SHTRPD_BOOTLOADER_RESET_DELAY   50000
#define SHTRPD_BOOT_ENTRY_CMD           0xFF
#define SHTRPD_BOOT_ENTRY_DAT           0xA5

#define SHTRPD_BOOT_SADDR               0x34

#define SHTRPD_GET_BOOT_VER             0x00
#define SHTRPD_GET_BOOT_DATA            0x01
#define SHTRPD_EXEC_APP                 0x02
#define SHTRPD_CRC_CECK                 0x03

#define SHTRPD_BOOT_LOADER_START_ADR    0x83c0
#define SHTRPD_BOOT_WRITES              240
#define SHTRPD_BOOT_BLOCK_SIZE          64
#define SHTRPD_BOOT_BLOCK_WRITE_DELAY   7000
#define SHTRPD_BOOT_VERSION_READ_DELAY  10000

#define SHTRPD_BOOT_SUCCESS             0
#define SHTRPD_BOOT_ERR                 (-1)
#define SHTRPD_BOOT_ERR_VERIFY          (-2)
#define SHTRPD_BOOT_ERR_READ            (-3)
#define SHTRPD_BOOT_ERR_EXEC_APP        (-3)
#define SHTRPD_BOOT_ERR_ACK_NONE        (-4)
#define SHTRPD_BOOT_ERR_CRC             (-5)

#define SHTRPD_INITIALSETUP_COMPLETE_DELAY 10000
#define SHTRPD_FIRSTXYINFOREAD_COMPLETE_DELAY 10000
#define SHTRPD_CRC_CHECK_DELAY          40000
#define SHTRPD_BOOT_ACK_POLL_DELAY      2000
#define SHTRPD_I2C_RETRY_DELAY          (100 * 1000)
#define SHTRPD_I2C_USER_DELAY           1000
#define SHTRPD_INITIAL_DOWN_DELAY       20000

#define SHTRPD_I2C_REQ_READ_BYTE        0
#define SHTRPD_I2C_REQ_READ_BLOCK       1
#define SHTRPD_I2C_REQ_WRITE_BYTE       2
#define SHTRPD_I2C_REQ_WRITE_BLOCK      3

#define SHTRPD_FM_CHK_BOOTLOADER_MODE   0
#define SHTRPD_FM_CHK_APP_EXEC          1

#define SHTRPD_INITIALSETUP_RETRY_MAX   200
#define SHTRPD_FIRSTXYINFOREAD_RETRY_MAX 700
#define SHTRPD_ACK_POLL_RETRY_MAX       2
#define SHTRPD_CRC_RETRY_MAX            50
#define SHTRPD_I2C_RTETRY_MAX           3
#define SHTRPD_USER_I2C_RTETRY_MAX      5

#define SHTRPD_FW_IMAGE_MAX             0x00003C00

#define SHTRPD_CONT_DATA_READ_SIZE      203
#define SHTRPD_XY_POS_DISABLE           ((u16)0xFFFF)

#define SHTRPD_PROPERTY_CMD_READ        0
#define SHTRPD_PROPERTY_CMD_WRITE       1
#define SHTRPD_PROPERTY_REQ_PARAM_MAX   16
#define SHTRPD_PROPERTY_INIT            0
#define SHTRPD_PROPERTY_ATI_VAL         1
#define SHTRPD_PROPERTY_CONTROLSETTINGS 2
#define SHTRPD_PROPERTY_THRESHOLD       3
#define SHTRPD_PROPERTY_FILTER          4
#define SHTRPD_PROPERTY_TIMING          5
#define SHTRPD_PROPERTY_CHANNELSETUP    6
#define SHTRPD_PROPERTY_PROXSETTINGS    7
#define SHTRPD_PROPERTY_ACTIVECHANNELS  8
#define SHTRPD_PROPERTY_DEBOUNCE        9
#define SHTRPD_PROPERTY_PMATI           10
#define SHTRPD_PROPERTY_TEMPDRIFT       11

#define SHTRDP_HW_REVISION_ES0          0
#define SHTRDP_HW_REVISION_ES1          1
#define SHTRDP_HW_REVISION_PP1          2
#define SHTRDP_HW_REVISION_PP2          3
#define SHTRDP_HW_REVISION_PMP          3

#define SNAPTHRESHOLD_VAL_ES1           250
#define SNAPTHRESHOLD_VAL_PP1           300
#define SNAPTHRESHOLD_VAL_PP2           310

/* ------------------------------------------------------------------------- */
/* VARIABLES                                                                 */
/* ------------------------------------------------------------------------- */
/*  Boolean value used for the initial setup */
bool initialSetup;
bool initialATI;
bool initReseed;
/*  Flag to indicate whether a reset has occured    */
u8 reset;

/*  A counter to run through the Setup States   */
u8 setupCounter;

/*  Structure to keep the info of the reported touches  */
struct shtrpd_fingers_structure shtrpd_touches[MAX_TOUCHES];

/*  Structure to keep the info of the reported touches (old)    */
struct shtrpd_fingers_structure shtrpd_touches_old[MAX_TOUCHES];

/*  Platform data for the Trackpad Device touchscreen driver    */
struct shtrpd_ts_platform_data {
    void (*cfg_pin)(void);
    int x_size;
    int y_size;
};

/* Each client has this additional data */
struct shtrpd_ts_data {
    struct i2c_client *client;
    struct input_dev *input_dev;
    const struct shtrpd_ts_platform_data *platform_data;
    struct early_suspend early_suspend;
};

static struct workqueue_struct *shtrpd_work_queue;
static struct work_struct       shtrpd_work_flip_open;
static struct work_struct       shtrpd_work_flip_close;

#define SHTRPD_IRQ_DISABLED					(0)
#define SHTRPD_IRQ_ENABLED					(1)
static spinlock_t shtrpd_spinlock;
static int shtrpd_irq_enable   = SHTRPD_IRQ_DISABLED;
static int shtrpd_irq_enable_wake = SHTRPD_IRQ_DISABLED;

/*  Register this driver for Early suspend/Late Resume  */
#ifdef CONFIG_HAS_EARLYSUSPEND
static void shtrpd_ts_early_suspend(struct early_suspend *h);
static void shtrpd_ts_late_resume(struct early_suspend *h);
#endif

static DEFINE_MUTEX(shtrpd_ioctl_lock);
static DEFINE_MUTEX(shtrpd_i2c_lock);
static DEFINE_MUTEX(shtrpd_flip_lock);

/* old NO_OF_FINGERS */
u8 noOfTouches_old = 0;

/* finger enable bit bit0:ID1 bit1:ID2 bit2:ID3 bit3:ID4 bit4:ID5 */
u8 finger_enable = 0;

/* old output slot no */
int old_output_slot_no = -1;

u16 old_keycode[MAX_SNAPS];
u8 old_keycode_onoff[MAX_SNAPS];

bool bootLoaderMode = true;
bool firmWritting = false;
bool firstXYInfoRead = false;

int shtrpd_touchevent_sw = 1;
int shtrpd_showreset_enable = 1;
int shtrpd_i2cerror_reset_enable = 0;

int shtrpd_fw_poll_retry_cnt = 10;
int shtrpd_fw_poll_delay = 1000;

int shtrpd_flip_state = 1;
int shtrpd_flip_reset_state = 0;
int shtrpd_flip_old_state   = 0;

#ifdef SHTRPD_ENABLE_INIT_FW_DL
static int shtrpd_fwdl_fail_flg = 0;
#endif /* SHTRPD_ENABLE_INIT_FW_DL */

static int shtrpd_touchcruiser_state = 0;

struct shtrpd_property_type {
    u8   controlsetting_val[4];

    u8   proxthreshold_val;
    u8   touchmultiplier_val;
    u8   touchshifter_val;
    u8   pmproxthreshold_val;
    u16  snapthreshold_val;
    u8   proxthreshold2_val;
    u8   touchmultiplier2_val;
    u8   touchshifter2_val;
    u8   intouchmultiplier_val;

    u16  atitarget_val;
    u8   atic_val;
    u16  atitarget2_val;
    u8   atic2_val;
    u8   re_ati_range;

    u8   filtersettings0_val;
    u8   touchdamping_val;
    u8   pmcountdamping_val;
    u8   lppmcountdamping_val;

    u8   reseedtime_val;
    u8   commstimeout_val;
    u8   modetime_val;
    u8   lptime_val;
    u8   sleeptime_val;
    u8   pmreseedtime;

    u8   totalrxs_val;
    u8   totaltxs_val;
    u8   trackpadrxs_val;
    u8   trackpadtxs_val;
    u8   pmsetup0_val;
    u8   txhigh_val;
    u8   txlow_val;

    u8   proxsettings_val[4];

    u16  activechannels_val[15];

    u8   proxdb_val;
    u8   touchsnapdb_val;
    u8   touchsnapdb2_val;

    u16  pmatitarget_val;
    u8   pmatic_val;
    u8   pm_re_ati_range;

    u8   temp_non_detect_avg;
    u8   temp_min_delta;
    u8   temp_lta_adj_interval;
};

struct shtrpd_property_type shtrpd_properties;

struct IoctlDiagCmdReq {
    int  rtn;
    uint8_t  m_Cmd;
    uint8_t  m_req_size;
    uint8_t  m_res_size;
    uint8_t  m_buf[512];
};

struct shtrpd_ioctl_fwparam {
    int             size;
    unsigned char*  data;
};

struct shtrpd_ioctl_resetparam {
    int     data;
};

struct shtrpd_ioctl_countval {
    int select;
    unsigned short data[TRACKPADTXS_VAL][TRACKPADRXS_VAL];
};

struct shtrpd_ioctl_properties {
    int id;
    int cmd;
    int param[SHTRPD_PROPERTY_REQ_PARAM_MAX];
};

struct i2c_client *shtrpd_client = NULL;

struct shtrpd_UserRequest {
   u8   req_flag;
   u8   read_write;
   const struct i2c_client *client;
   u8   cmd;
   int  length;
   u8   values;
   u8   *pbuf;
   int  ret;
};
struct shtrpd_UserRequest userRequest;

enum {
	SHTRPD_FINGER_STATE_NONE,
	SHTRPD_FINGER_STATE_TD,
	SHTRPD_FINGER_STATE_DRAG,
	SHTRPD_FINGER_STATE_TU,
};

u8 shtrpd_finger_fw_state = 0;
u8 shtrpd_finger_fw_state_old = 0;

#if defined( CONFIG_ANDROID_ENGINEERING )
	#define SHTRPD_PARAM_DEF(name, val) \
		int name = val; \
		module_param(name, int, S_IRUGO | S_IWUSR);
#else
	#define SHTRPD_PARAM_DEF(name, val) \
		const int name = val;
#endif /* CONFIG_ANDROID_ENGINEERING */

#if defined( SHTRPD_DRAG_SMOOTH_ENABLE )
#define SHTRPD_POSTYPE_X (0)
#define SHTRPD_POSTYPE_Y (1)
#define SHTRPD_DRAG_HISTORY_SIZE_MAX 30

SHTRPD_PARAM_DEF(SHTRPD_DRAG_SMOOTH, 				1);
SHTRPD_PARAM_DEF(SHTRPD_DRAG_SMOOTH_LEAVE_MAX_DOT, 	10);
SHTRPD_PARAM_DEF(SHTRPD_DRAG_SMOOTH_COUNT_MIN,		5);
SHTRPD_PARAM_DEF(SHTRPD_DRAG_SMOOTH_COUNT_MAX,		20);
SHTRPD_PARAM_DEF(SHTRPD_DRAG_SMOOTH_COUNT_UP_STEP,	5);
SHTRPD_PARAM_DEF(SHTRPD_DRAG_SMOOTH_FIXED_SHIFT, 	8);

#define SHTRPD_DRAG_SMOOTH_INT_TO_FIXED(val)	((val) << SHTRPD_DRAG_SMOOTH_FIXED_SHIFT)
#define SHTRPD_DRAG_SMOOTH_FIXED_TO_INT(val)	((val) >> SHTRPD_DRAG_SMOOTH_FIXED_SHIFT)

#if defined( SHTRPD_LOG_DEBUG_ENABLE )
	SHTRPD_PARAM_DEF( SHTRPD_DRAG_SMOOTH_LOG_ENABLE,				0);
	#define	SHTRPD_LOG_DRAG_SMOOTH(...)									\
		if(SHTRPD_DRAG_SMOOTH_LOG_ENABLE != 0){							\
			printk(KERN_DEBUG "[shtrpd] [drag_smooth]" __VA_ARGS__);	\
		}
#else
	#define	SHTRPD_LOG_DRAG_SMOOTH(...)
#endif /* SHTRPD_LOG_DEBUG_ENABLE */

enum{
	SHTRPD_DRAG_DIR_NONE = 0,
	SHTRPD_DRAG_DIR_PLUS,
	SHTRPD_DRAG_DIR_MINUS,
};

u8 shtrpd_finger_state_old = 0;

struct shtrpd_drag_hist{
	int		pre;
	u8		dir;
	u8		count;
	int		history[SHTRPD_DRAG_HISTORY_SIZE_MAX];
	int		pre_comp_history_FIXED;
	int		history_old;
	int		count_up_base;
};

struct shtrpd_drag_hist	drag_hist[MAX_TOUCHES][2];
#endif /* SHTRPD_DRAG_SMOOTH_ENABLE */

#if defined(SHTRPD_EDGE_FAIL_TOUCH_REJECTION_ENABLE)
SHTRPD_PARAM_DEF( SHTRPD_EDGE_FAIL_TOUCH_REJECT_ENABLE,         1);
SHTRPD_PARAM_DEF( SHTRPD_EDGE_FAIL_TOUCH_REJECT_AREA_THRESH_X,  63);
SHTRPD_PARAM_DEF( SHTRPD_EDGE_FAIL_TOUCH_REJECT_AREA_THRESH_Y,  375);
SHTRPD_PARAM_DEF( SHTRPD_EDGE_FAIL_TOUCH_REJECT_AREA_THRESH_X2, 170);
SHTRPD_PARAM_DEF( SHTRPD_EDGE_FAIL_TOUCH_REJECT_AREA_THRESH_Y2, 750);
SHTRPD_PARAM_DEF( SHTRPD_EDGE_FAIL_TOUCH_REJECT_DISTANCE,       300);
SHTRPD_PARAM_DEF( SHTRPD_EDGE_FAIL_TOUCH_REJECT_Z_RATIO,        89);
SHTRPD_PARAM_DEF( SHTRPD_EDGE_FAIL_TOUCH_REJECT_Z_RATIO_2,      95);
SHTRPD_PARAM_DEF( SHTRPD_EDGE_FAIL_TOUCH_REJECT_EFFECT_TIME,    1544);

#if defined( SHTRPD_LOG_DEBUG_ENABLE )
    SHTRPD_PARAM_DEF( SHTRPD_EDGE_FAIL_TOUCH_REJECT_LOG_ENABLE, 0);
    #define SHTRPD_LOG_DBG_EDGE_FAIL_TOUCH_PRINT(...)                       \
        if(SHTRPD_EDGE_FAIL_TOUCH_REJECT_LOG_ENABLE != 0){                  \
            printk(KERN_DEBUG "[shtrpd] [edge_fail_reject]" __VA_ARGS__);   \
        }
#else
    #define SHTRPD_LOG_DBG_EDGE_FAIL_TOUCH_PRINT(...)
#endif /* SHTRPD_LOG_DEBUG_ENABLE */

#define SHTRPD_TOUCH_CANCEL_COORDINATES_X (0)
#define SHTRPD_TOUCH_CANCEL_COORDINATES_Y (9999)

struct shtrpd_filter_edge_fail_touch_rej_info{
    u8              enable;
    u8              inhibit_id;
    unsigned long   td_time[MAX_TOUCHES];
};
static struct shtrpd_filter_edge_fail_touch_rej_info edge_fail_touch_rej = 
{ 
    .enable = 1, 
    .inhibit_id = 0 
};
#endif /* SHTRPD_EDGE_FAIL_TOUCH_REJECTION_ENABLE */

#if defined(SHTRPD_EDGE_TD_MASK_ENABLE)
SHTRPD_PARAM_DEF( SHTRPD_EDGE_TD_MASK,         1);
SHTRPD_PARAM_DEF( SHTRPD_EDGE_TD_MASK_AREA_X,  20);
SHTRPD_PARAM_DEF( SHTRPD_EDGE_TD_MASK_AREA_Y,  20);
#endif /* SHTRPD_EDGE_TD_MASK_ENABLE */

#if defined( SHTRPD_TOUCH_UP_DELAY_ENABLE )
SHTRPD_PARAM_DEF(SHTRPD_TOUCH_UP_DELAY,   1);
SHTRPD_PARAM_DEF(SHTRPD_TOUCH_UP_DELAY_HISTORY_DIST,  5);
SHTRPD_PARAM_DEF(SHTRPD_TOUCH_UP_DELAY_HISTORY_COUNT, 3);
SHTRPD_PARAM_DEF(SHTRPD_TOUCH_UP_DELAY_IGNORE_COUNT,  10);

SHTRPD_PARAM_DEF(SHTRPD_TOUCH_UP_DELAY_AREA_ENABLE,        1);
SHTRPD_PARAM_DEF(SHTRPD_TOUCH_UP_DELAY_AREA_HISTORY_COUNT, 1);
SHTRPD_PARAM_DEF(SHTRPD_TOUCH_UP_DELAY_IGNORE_AREA_Y,      370);
SHTRPD_PARAM_DEF(SHTRPD_TOUCH_UP_DELAY_IGNORE_PWR_AREA_X,  370);
SHTRPD_PARAM_DEF(SHTRPD_TOUCH_UP_DELAY_IGNORE_PWR_AREA_Y1, 370);
SHTRPD_PARAM_DEF(SHTRPD_TOUCH_UP_DELAY_IGNORE_PWR_AREA_Y2, 550);

struct shtrpd_td_history{
	u16		x_bk;
	u16		y_bk;
	u16		z_bk;
	u16		a_bk;
	int		count_x;
	int		count_y;
	u8		state;
};

static u8 tu_queue_work_flg = 0;

int shtrpd_delay_count[MAX_TOUCHES];
int shtrpd_ignore_count[MAX_TOUCHES];
struct shtrpd_td_history td_history[MAX_TOUCHES];

static struct delayed_work shtrpd_work_tu_delay;
static struct delayed_work shtrpd_work_snap_err_delay;

static DEFINE_MUTEX(shtrpd_tu_delay_lock);
#endif /* SHTRPD_TOUCH_UP_DELAY_ENABLE */

#if defined( SHTRPD_TOUCH_DOWN_THRESHOLD_ENABLE )
struct shtrpd_th_history{
	u16		x;
	u16		y;
	int		count;
};

struct shtrpd_th_history th_history[MAX_TOUCHES];
static u8 shtrpd_touch_down_threshold_flg = 0;

SHTRPD_PARAM_DEF(SHTRPD_TOUCH_DOWN_THRESHOLD,        0);
SHTRPD_PARAM_DEF(SHTRPD_TOUCH_DOWN_THRESHOLD_COUNT,  10);
SHTRPD_PARAM_DEF(SHTRPD_TOUCH_DOWN_THRESHOLD_DIST,   30);
SHTRPD_PARAM_DEF(SHTRPD_TOUCH_DOWN_THRESHOLD_ADJUST, 7);

static int shtrpd_i2c_write_block_data(const struct i2c_client *client, u8 command, int length, u8 *values);
#endif /* SHTRPD_TOUCH_DOWN_THRESHOLD_ENABLE */

#if defined(SHTRPD_TOP_OF_SCREEN_GHOST_REJECTION_ENABLE)
SHTRPD_PARAM_DEF( SHTRPD_TOP_OF_SCREEN_GHOST_REJECT_ENABLE,				1);
SHTRPD_PARAM_DEF( SHTRPD_TOP_OF_SCREEN_GHOST_REJECT_CLEAR_COUNT,		1);
SHTRPD_PARAM_DEF( SHTRPD_TOP_OF_SCREEN_GHOST_REJECT_XDIFF_THRESH,		100);
//SHTRPD_PARAM_DEF( SHTRPD_TOP_OF_SCREEN_GHOST_REJECT_Z_RATIO_THRESH,		60);

#if defined( SHTRPD_LOG_DEBUG_ENABLE )
	SHTRPD_PARAM_DEF( SHTRPD_TOP_OF_SCREEN_GHOST_LOG_ENABLE, 	0);
    #define	SHTRPD_LOG_DBG_TOP_OF_SCREEN_GHOST_PRINT(...)				\
        if(SHTRPD_TOP_OF_SCREEN_GHOST_LOG_ENABLE != 0){	\
            printk(KERN_DEBUG "[shtrpd] [top of screen ghost]" __VA_ARGS__);	\
        }
#else
	#define	SHTRPD_LOG_DBG_TOP_OF_SCREEN_GHOST_PRINT(...)
#endif /* SHTRPD_LOG_DEBUG_ENABLE */

enum{
	TOP_OF_SCREEN_GHOST_NOT_DETECTED,
	TOP_OF_SCREEN_GHOST_DETECTED,
};

enum{
	TOP_OF_SCREEN_GHOST_NOT_CLEARED,
	TOP_OF_SCREEN_GHOST_CLEARED,
};

struct shtrpd_filter_topofscreen_ghost_reject_info{
	u8						is_ghost;
	u8						reject_count;
};
static struct shtrpd_filter_topofscreen_ghost_reject_info topofscreen_ghost_reject[MAX_TOUCHES];
#endif /* SHTRPD_TOP_OF_SCREEN_GHOST_REJECTION_ENABLE */

#if defined(SHTRPD_TU_JITTER_FILTER_ENABLE)
SHTRPD_PARAM_DEF( SHTRPD_TU_JITTER_FILTER,						1);
SHTRPD_PARAM_DEF( SHTRPD_TU_JITTER_FILTER_AREA_PREMIN_TH,		3);
SHTRPD_PARAM_DEF( SHTRPD_TU_JITTER_FILTER_AREA_MAX_TH,			3);
SHTRPD_PARAM_DEF( SHTRPD_TU_JITTER_FILTER_AREA_DECAY_TH,		50);
SHTRPD_PARAM_DEF( SHTRPD_TU_JITTER_FILTER_MOVE_RET_TH,			100);
SHTRPD_PARAM_DEF( SHTRPD_TU_JITTER_FILTER_TIMEOUT,				50);
SHTRPD_PARAM_DEF( SHTRPD_PINCHIN_JITTER_FILTER,					1);
SHTRPD_PARAM_DEF( SHTRPD_PINCHIN_JITTER_FILTER_DIST_HYSTERESIS,	3);
SHTRPD_PARAM_DEF( SHTRPD_PINCHIN_JITTER_FILTER_START_DIST,		300);
SHTRPD_PARAM_DEF( SHTRPD_PINCHIN_JITTER_FILTER_EXIT_MOVE,		250);
SHTRPD_PARAM_DEF( SHTRPD_PINCHIN_JITTER_FILTER_EXIT_DIST,		350);

#if defined( SHTRPD_LOG_DEBUG_ENABLE )
	SHTRPD_PARAM_DEF( SHTRPD_TU_JITTER_FILTER_LOG_ENABLE, 	0);
    #define	SHTRPD_LOG_DBG_TU_JITTER_FILTER_PRINT(...)				\
        if(SHTRPD_TU_JITTER_FILTER_LOG_ENABLE != 0){	\
            printk(KERN_DEBUG "[shtrpd] [tu jitter]" __VA_ARGS__);	\
        }
#else
	#define	SHTRPD_LOG_DBG_TU_JITTER_FILTER_PRINT(...)
#endif /* SHTRPD_LOG_DEBUG_ENABLE */

enum{
	SHTRPD_TU_JITTER_FILTER_NO_FIXED = 0,
	SHTRPD_TU_JITTER_FILTER_FIXED_BY_AREA,
	SHTRPD_TU_JITTER_FILTER_FIXED_BY_PINCHIIN,
};

struct shtrpd_tu_jitter_filter_info{
	u8								is_fixed;
	int								pre_fixed_ts;
	int								pre_fixed_area;
	unsigned long					pre_fixed_time;
	struct shtrpd_fingers_structure	pre_touch;
	
	
	u8								dist_cond;
	unsigned long					dist_x[MAX_TOUCHES];
	unsigned long					dist_y[MAX_TOUCHES];
};
static struct shtrpd_tu_jitter_filter_info tu_jitter_filter[MAX_TOUCHES];
#endif /* SHTRPD_TU_JITTER_FILTER_ENABLE */

#if defined(SHTRPD_CPU_IDLE_SLEEP_CONTROL_ENABLE)
#include <linux/pm_qos.h>
#define SHTRPD_QOS_LATENCY_DEF_VALUE	 			34
static DEFINE_MUTEX(shtrpd_cpu_idle_sleep_ctrl_lock);

struct shtrpd_idle_sleep_ctrl_info{
	struct wake_lock            wake_lock;
	struct pm_qos_request		qos_cpu_latency;
	int							wake_lock_idle_state;
};
static struct shtrpd_idle_sleep_ctrl_info idle_sleep_ctrl_info;
#endif /* SHTRPD_CPU_IDLE_SLEEP_CONTROL_ENABLE */

#if defined(SHTRPD_CPU_CLOCK_CONTROL_ENABLE)
#include <mach/perflock.h>
static DEFINE_MUTEX(shtrpd_cpu_clock_ctrl_lock);
SHTRPD_PARAM_DEF( SHTRPD_PERF_LOCK_ENABLE_TIME_MS,	100);

struct shtrpd_cpu_clock_ctrl_info{
	struct perf_lock			perf_lock;
	struct delayed_work			perf_lock_disable_delayed_work;
	int							report_event;
};
static struct shtrpd_cpu_clock_ctrl_info cpu_clock_ctrl_info;
#endif /* SHTRPD_CPU_CLOCK_CONTROL_ENABLE */

#if defined(SHTRPD_DRUMMING_SPLIT_ENABLE)

#define SHTRPD_DRUMMING_SPLIT_SWITCH_FINGER_ENABLE
#define SHTRPD_DRUMMING_SPLIT_DIFF_SAMPLE_MAX	(5)

SHTRPD_PARAM_DEF( SHTRPD_DRUMMING_SPLIT,						1);
SHTRPD_PARAM_DEF( SHTRPD_DRUMMING_SPLIT_MOVE_TH,				200);
SHTRPD_PARAM_DEF( SHTRPD_DRUMMING_SPLIT_FLICK_TH,				200);
SHTRPD_PARAM_DEF( SHTRPD_DRUMMING_SPLIT_MULTI_MOVE_TH,			400);
SHTRPD_PARAM_DEF( SHTRPD_DRUMMING_SPLIT_TAP_PRE_MOVE_TH,		20);
SHTRPD_PARAM_DEF( SHTRPD_DRUMMING_SPLIT_TAP_POST_MOVE_TH,		20);
SHTRPD_PARAM_DEF( SHTRPD_DRUMMING_SPLIT_RJUMP_MOVE_TH,			500);
SHTRPD_PARAM_DEF( SHTRPD_DRUMMING_SPLIT_RJUMP_CHECK_TIME,		100);
SHTRPD_PARAM_DEF( SHTRPD_DRUMMING_SPLIT_DIFF_SAMPLE_NUM,		3);
SHTRPD_PARAM_DEF( SHTRPD_DRUMMING_SPLIT_MTAP_POS_TH,			100);
SHTRPD_PARAM_DEF( SHTRPD_DRUMMING_SPLIT_MTAP_DEAD_AREA_X,		539);
SHTRPD_PARAM_DEF( SHTRPD_DRUMMING_SPLIT_MTAP_DEAD_AREA_Y,		0);
SHTRPD_PARAM_DEF( SHTRPD_DRUMMING_SPLIT_MTAP_DEAD_AREA_SIZE,	20);

#if defined( SHTRPD_LOG_DEBUG_ENABLE )
	SHTRPD_PARAM_DEF( SHTRPD_DRUMMING_SPLIT_LOG_ENABLE, 	0);
    #define	SHTRPD_LOG_DBG_DRUMMING_SPLIT_PRINT(...)				\
        if(SHTRPD_DRUMMING_SPLIT_LOG_ENABLE != 0){	\
            printk(KERN_DEBUG "[shtrpd] [drumming split]" __VA_ARGS__);	\
        }
#else
	#define	SHTRPD_LOG_DBG_DRUMMING_SPLIT_PRINT(...)
#endif /* SHTRPD_LOG_DEBUG_ENABLE */

struct shtrpd_drumming_split_info{
	int								diff_num;
	int								diff_hist_x[SHTRPD_DRUMMING_SPLIT_DIFF_SAMPLE_MAX];
	int								diff_hist_y[SHTRPD_DRUMMING_SPLIT_DIFF_SAMPLE_MAX];

	u8								is_hold;
	u8								is_flick;
	struct shtrpd_fingers_structure	hold_touch;
};
static struct shtrpd_drumming_split_info	drumming_split[MAX_TOUCHES];
static unsigned long						drumming_split_latest_mt_time;

#if defined(SHTRPD_DRUMMING_SPLIT_SWITCH_FINGER_ENABLE)
struct shtrpd_replace_fingers_info{
	u8								fw_finger_state;
	u8								order[MAX_TOUCHES];
};
static struct shtrpd_replace_fingers_info replace_fingers;
#endif /* SHTRPD_DRUMMING_SPLIT_SWITCH_FINGER_ENABLE */

#endif /* SHTRPD_DRUMMING_SPLIT_ENABLE */

#if defined(SHTRPD_TAP_JITTER_FILTER_ENABLE)
SHTRPD_PARAM_DEF( SHTRPD_TAP_JITTER_FILTER,                     1);
SHTRPD_PARAM_DEF( SHTRPD_TAP_JITTER_FILTER_TSDIFF_THRESH,       4);
SHTRPD_PARAM_DEF( SHTRPD_TAP_JITTER_FILTER_MOVE_OVER_THRESH,    20);
SHTRPD_PARAM_DEF( SHTRPD_TAP_JITTER_FILTER_TIME_THRESH,         50);
SHTRPD_PARAM_DEF( SHTRPD_TAP_JITTER_FILTER_JUMPUP_Y_THRESH,     50);
SHTRPD_PARAM_DEF( SHTRPD_TAP_JITTER_FILTER_JUMPUP_Z_THRESH,     25);
SHTRPD_PARAM_DEF( SHTRPD_TAP_JITTER_FILTER_JUMPDOWN_Y_THRESH,   50);
SHTRPD_PARAM_DEF( SHTRPD_TAP_JITTER_FILTER_JUMPDOWN_AREA_THRESH,3);
SHTRPD_PARAM_DEF( SHTRPD_TAP_JITTER_FILTER_JUMPDOWN_Z_THRESH,   60);

#if defined( SHTRPD_LOG_DEBUG_ENABLE )
    SHTRPD_PARAM_DEF( SHTRPD_TAP_JITTER_FILTER_LOG_ENABLE, 0);
    #define SHTRPD_LOG_DBG_SHTRPD_TAP_JITTER_FILTER_PRINT(...)               \
        if(SHTRPD_TAP_JITTER_FILTER_LOG_ENABLE != 0){                        \
            printk(KERN_DEBUG "[shtrpd] [tap_jitter_filter]" __VA_ARGS__);   \
        }
#else
    #define SHTRPD_LOG_DBG_SHTRPD_TAP_JITTER_FILTER_PRINT(...)
#endif /* SHTRPD_LOG_DEBUG_ENABLE */

struct shtrpd_tap_jitter_info{
    u8                               enable;
    u8                               jump_check_enable;
    unsigned long                    td_time;
    struct shtrpd_fingers_structure  hold_touch;
};
static struct shtrpd_tap_jitter_info tap_jitter[MAX_TOUCHES];
#endif /* SHTRPD_TAP_JITTER_FILTER_ENABLE */

#if defined(SHTRPD_GRIP_FAIL_FLICK_REJECTION_ENABLE)
SHTRPD_PARAM_DEF( SHTRPD_GRIP_FAIL_FLICK_REJECT_ENABLE, 			1);
SHTRPD_PARAM_DEF( SHTRPD_GRIP_FAIL_FLICK_REJECT_AREA, 				30);
SHTRPD_PARAM_DEF( SHTRPD_GRIP_FAIL_FLICK_REJECT_DIST, 				350);
SHTRPD_PARAM_DEF( SHTRPD_GRIP_FAIL_FLICK_REJECT_AREA_TH,			3);
SHTRPD_PARAM_DEF( SHTRPD_GRIP_FAIL_FLICK_REJECT_Z_TH, 				500);

#if defined( SHTRPD_LOG_DEBUG_ENABLE )
	SHTRPD_PARAM_DEF( SHTRPD_GRIP_FAIL_FLICK_REJECT_LOG_ENABLE, 0);
    #define	SHTRPD_LOG_DBG_GRIP_FAIL_FLICK_PRINT(...)							\
        if(SHTRPD_GRIP_FAIL_FLICK_REJECT_LOG_ENABLE != 0){						\
            printk(KERN_DEBUG "[shtrpd] [grip_fail_flick_reject]" __VA_ARGS__);	\
        }
#else
	#define	SHTRPD_LOG_DBG_GRIP_FAIL_FLICK_PRINT(...)
#endif /* SHTRPD_LOG_DEBUG_ENABLE */

static u8 grip_fail_flick_inhibit_id;
#endif /* SHTRPD_GRIP_FAIL_FLICK_REJECTION_ENABLE */

#if defined(SHTRPD_SPLIT_FINGER_REJECTION_ENABLE)
SHTRPD_PARAM_DEF( SHTRPD_SPLIT_FINGER_REJECT_ENABLE,				1);
SHTRPD_PARAM_DEF( SHTRPD_SPLIT_FINGER_REJECT_SINGLE_RANGE,			250);
SHTRPD_PARAM_DEF( SHTRPD_SPLIT_FINGER_REJECT_SINGLE_HYSTERESIS,		30);
SHTRPD_PARAM_DEF( SHTRPD_SPLIT_FINGER_REJECT_MULTI_RANGE,			300);
SHTRPD_PARAM_DEF( SHTRPD_SPLIT_FINGER_REJECT_MULTI_HYSTERESIS,		50);
SHTRPD_PARAM_DEF( SHTRPD_SPLIT_FINGER_REJECT_TU_HOLD_TIME,			30);

#if defined( SHTRPD_LOG_DEBUG_ENABLE )
	SHTRPD_PARAM_DEF( SHTRPD_SPLIT_FINGER_REJECT_LOG_ENABLE, 	0);
    #define	SHTRPD_LOG_DBG_SPLIT_FINGER_REJECT_PRINT(...)				\
        if(SHTRPD_SPLIT_FINGER_REJECT_LOG_ENABLE != 0){	\
            printk(KERN_DEBUG "[shtrpd] [split finger reject]" __VA_ARGS__);	\
        }
#else
	#define	SHTRPD_LOG_DBG_SPLIT_FINGER_REJECT_PRINT(...)
#endif /* SHTRPD_LOG_DEBUG_ENABLE */

struct shtrpd_split_finger_reject_info{
	u8								is_ghost;
	u8								is_hold;
	u8								pair_fingers;
	unsigned long					hold_time;
};
static struct shtrpd_split_finger_reject_info split_finger_reject[MAX_TOUCHES];
#endif /* SHTRPD_SPLIT_FINGER_REJECTION_ENABLE */

#if defined(SHTRPD_SNAP_MASK)
SHTRPD_PARAM_DEF(SHTRPD_SNAP_MASK_ENABLE,				0);
SHTRPD_PARAM_DEF(SHTRPD_SNAP_MASK_DIAGONAL_KEY,			0);
SHTRPD_PARAM_DEF(SHTRPD_SNAP_MASK_COUNT,				3);
SHTRPD_PARAM_DEF(SHTRPD_SNAP_MASK_KEY_TO_ENTER_MASK_ENABLE,		0);
SHTRPD_PARAM_DEF(SHTRPD_SNAP_MASK_ENTER_TO_KEY_MASK_ENABLE,		0);

SHTRPD_PARAM_DEF(SHTRPD_SNAP_CHATTERING_REMOVAL_ENABLE,		0);
SHTRPD_PARAM_DEF(SHTRPD_SNAP_CHATTERING_REMOVAL_COUNT,		3);

enum {
    MASK_KEY_S1,
    MASK_KEY_UPLEFT,
    MASK_KEY_UP,
    MASK_KEY_UPRIGHT,
    MASK_KEY_S2,
    MASK_KEY_LEFT,
    MASK_KEY_ENTER,
    MASK_KEY_RIGHT,
    MASK_KEY_S3,
    MASK_KEY_DNLEFT,
    MASK_KEY_DOWN,
    MASK_KEY_DNRIGHT,
    MASK_KEY_S4,
    MASK_KEY_PHONE,
    MASK_KEY_BACKSPACE,
    MASK_KEY_1,
    MASK_KEY_2,
    MASK_KEY_3,
    MASK_KEY_4,
    MASK_KEY_5,
    MASK_KEY_6,
    MASK_KEY_7,
    MASK_KEY_8,
    MASK_KEY_9,
    MASK_KEY_STAR,
    MASK_KEY_0,
    MASK_KEY_POUND,
    MASK_KEY_S5,
    MASK_KEY_S6,
    MASK_MAX_COUNT
};

static u8 shtrpd_key_mask_count[MAX_SNAPS];
static u8 shtrpd_key_chattering_removal_count[MAX_SNAPS];

#if defined( SHTRPD_LOG_DEBUG_ENABLE )
	SHTRPD_PARAM_DEF( SHTRPD_SNAP_MASK_LOG_ENABLE,	0);
	#define	SHTRPD_LOG_SNAP_MASK(...)									\
		if(SHTRPD_SNAP_MASK_LOG_ENABLE != 0){							\
			printk(KERN_DEBUG "[shtrpd] [snap_mask]" __VA_ARGS__);	\
		}
#else
	#define	SHTRPD_LOG_SNAP_MASK(...)
#endif /* SHTRPD_LOG_DEBUG_ENABLE */
#endif /* SHTRPD_SNAP_MASK */

#if defined(SHTRPD_FAIL_TOUCH_CLEAR_ENABLE)
SHTRPD_PARAM_DEF( SHTRPD_FAIL_TOUCH_CLEAR,					1);
SHTRPD_PARAM_DEF( SHTRPD_FAIL_TOUCH_CLEAR_AREA_TH,			255);
SHTRPD_PARAM_DEF( SHTRPD_FAIL_TOUCH_CLEAR_TS_TH,			500);
SHTRPD_PARAM_DEF( SHTRPD_FAIL_TOUCH_CLEAR_XPOS_BASE,		400);
SHTRPD_PARAM_DEF( SHTRPD_FAIL_TOUCH_CLEAR_XPOS_RANGE,		9999);
SHTRPD_PARAM_DEF( SHTRPD_FAIL_TOUCH_CLEAR_EVENTNUM_TH,		2);
SHTRPD_PARAM_DEF( SHTRPD_FAIL_TOUCH_CLEAR_REATI_TIME_TH,	5000);
SHTRPD_PARAM_DEF( SHTRPD_FAIL_TOUCH_CLEAR_REATI_COUNT_TH,	5);
SHTRPD_PARAM_DEF( SHTRPD_FAIL_TOUCH_CLEAR_REATI_INTERVAL_TH,2000);

#if defined( SHTRPD_LOG_DEBUG_ENABLE )
	SHTRPD_PARAM_DEF( SHTRPD_FAIL_TOUCH_CLEAR_LOG_ENABLE, 	0);
    #define	SHTRPD_LOG_DBG_FAIL_TOUCH_CLEAR_PRINT(...)				\
        if(SHTRPD_FAIL_TOUCH_CLEAR_LOG_ENABLE != 0){	\
            printk(KERN_DEBUG "[shtrpd] [fail touch clear]" __VA_ARGS__);	\
        }
#else
	#define	SHTRPD_LOG_DBG_FAIL_TOUCH_CLEAR_PRINT(...)
#endif /* SHTRPD_LOG_DEBUG_ENABLE */

struct shtrpd_fail_touch_clear_info{
	u8								invalid_count;
	u8								count;
	unsigned long					start_time;
};
static struct shtrpd_fail_touch_clear_info fail_touch_clear[MAX_TOUCHES];
static int fail_touch_clear_reati_req = 0;
#endif /* SHTRPD_FAIL_TOUCH_CLEAR_ENABLE */

int shtrpd_touchmultiplier_val   = TOUCHMULTIPLIER_VAL;
int shtrpd_touchshifter_val      = TOUCHSHIFTER_VAL;
int shtrpd_touchmultiplier2_val  = TOUCHMULTIPLIER2_VAL;
int shtrpd_touchshifter2_val     = TOUCHSHIFTER2_VAL;
int shtrpd_intouchmultiplier_val = INTOUCHMULTIPLIER_VAL;
int shtrpd_snapthreshold_val     = SNAPTHRESHOLD_VAL;
int shtrpd_proxthreshold_flipclose_val     = 0xFF;
int shtrpd_touchmultiplier_flipclose_val   = 0xFF;
int shtrpd_pmproxthreshold_flipclose_val   = 0xFF;
int shtrpd_intouchmultiplier_flipclose_val = 0xFF;
int shtrpd_lptime_flipclose_val  = 13;
int shtrpd_bootloader_key_wait   = SHTRPD_BOOTLOADER_RESET_DELAY;
int shtrpd_exec_app_wait         = SHTRPD_EXEC_APP_DELAY;
int shtrpd_handshake_wait        = SHTRPD_HANDSHAKE_DELAY;

int shtrpd_atic_adj_disable = 0;
int shtrpd_atic_adj_wait    = SHTRPD_ATIC_ADJ_DELAY;
int shtrpd_temp_drift       = 1;

#if defined(SHTRPD_NOINTERRUPT_IN_TOUCH_CHECK_ENABLE)
SHTRPD_PARAM_DEF( SHTRPD_NOINTERRUPT_IN_TOUCH_CHECK_DELAY,		5000);
static struct delayed_work shtrpd_work_noint_check_delay;
static struct delayed_work shtrpd_work_pwkey_delay;
int shtrpd_noint_check_state = 0;
static DEFINE_MUTEX(shtrpd_noint_check_delay_lock);
#endif /* SHTRPD_NOINTERRUPT_IN_TOUCH_CHECK_ENABLE */

#if defined(SHTRPD_INTERRUPT_TIMING_CHECK_ENABLE)
SHTRPD_PARAM_DEF( SHTRPD_INTERRUPT_TIMING_CHECK_TIME,		100);
SHTRPD_PARAM_DEF( SHTRPD_TU_DELAY_REATI_ENABLE,				0);
SHTRPD_PARAM_DEF( SHTRPD_TU_DELAY_REATI_TIME,				5000);
SHTRPD_PARAM_DEF( SHTRPD_PWKEY_FORCE_REATI_ENABLE,			0);
SHTRPD_PARAM_DEF( SHTRPD_INTERRUPT_TIMING_CHECK_TIME_DBG,	0);
SHTRPD_PARAM_DEF( SHTRPD_TCH_KEEP_CHECK_TIME_CNT,			100);
SHTRPD_PARAM_DEF( SHTRPD_TCH_KEEP_CHECK_POS_DELTA,			10);
SHTRPD_PARAM_DEF( SHTRPD_TCH_KEEP_CHECK_AREA_MAX,			2);
SHTRPD_PARAM_DEF( SHTRPD_SNAP_TCH_UNMATCH_CHECK,			0);
int int_timing_check_state = 0;
int int_timing_check_reati = 0;
int force_reati_req = 0;
unsigned long int_timing_check_old_time = 0;
static struct delayed_work shtrpd_work_int_timing_delay;
int shtrpd_int_check_touches_state = 0;
int shtrpd_int_check_touches_time_count[MAX_TOUCHES];
struct shtrpd_fingers_structure shtrpd_int_check_touches_old[MAX_TOUCHES];
#endif /* SHTRPD_INTERRUPT_TIMING_CHECK_ENABLE */

#if defined(SHTRPD_FAIL_FLICK_RECOVER_ENABLE)
SHTRPD_PARAM_DEF( SHTRPD_FAIL_FLICK_RECOVER,				1);
SHTRPD_PARAM_DEF( SHTRPD_FAIL_FLICK_RECOVER_VRATIO,			50);
SHTRPD_PARAM_DEF( SHTRPD_FAIL_FLICK_RECOVER_VMIN,			5);

struct shtrpd_fail_flick_recover_info{
	int hist_cnt;
	int hist_x[3];
	int hist_y[3];
	unsigned long hist_time[3];

	int acc_x;
	int acc_y;
	unsigned long pre_time;
};
static struct shtrpd_fail_flick_recover_info fail_flick_recover[MAX_TOUCHES];
#endif /* SHTRPD_FAIL_FLICK_RECOVER_ENABLE */

#if defined(SHTRPD_SNAP_USER_MASK)
#define	TRPD_KEYMODE_NORMAL					(0x0000)
#define	TRPD_KEYMODE_DISALBE_DIAGONAL_KEY	(0x0001)

SHTRPD_PARAM_DEF( SHTRPD_SNAP_USER_MASK_DIAGONAL_KEY,  1);
SHTRPD_PARAM_DEF( SHTRPD_SNAP_USER_MASK_CLEAR_IN_FLIP, 0);

static DEFINE_MUTEX(shtrpd_snap_keymode_lock);
static int shtrpd_snap_keymode = TRPD_KEYMODE_NORMAL;

#if defined (SHTRPD_FAIL_TOUCH_CLEAR_ENABLE)
module_param(shtrpd_snap_keymode,  int, 0600);
#endif /* SHTRPD_FAIL_TOUCH_CLEAR_ENABLE */

#endif /* SHTRPD_SNAP_USER_MASK */

#if defined( SHTRPD_CHECK_CPU_TEMP_ENABLE )
#define SHTRPD_CPU_TEMP_FILE "sys/class/power_supply/shbatt_adc/cpu_temp" 

SHTRPD_PARAM_DEF( SHTRPD_CHECK_CPU_TEMP_INTERVAL,  60000);

int SHTRPD_CHECK_CPU_TEMP = 0;
int shtrpd_new_snap_threshold = SNAPTHRESHOLD_VAL;
int shtrpd_new_touchmultiplier_val = TOUCHMULTIPLIER_VAL;
int shtrpd_new_touchshifter_val = TOUCHSHIFTER_VAL;
int shtrpd_new_touchmultiplier2_val = TOUCHMULTIPLIER2_VAL;
int shtrpd_new_touchshifter2_val = TOUCHSHIFTER2_VAL;
int shtrpd_new_intouchmultiplier_val = INTOUCHMULTIPLIER_VAL;
int shtrpd_old_cpu_temp = -9999;

static struct delayed_work shtrpd_work_cpu_temp;

struct shtrpd_snap_thresh_tbl_info {
	int temp;
	int snap_threshold;
	int touch_multiplier_val;
	int touch_shifter_val;
	int touch_multiplier2_val;
	int touch_shifter2_val;
	int in_touch_multiplier_val;
};

static struct shtrpd_snap_thresh_tbl_info SHTRPD_UP_TEMP_SNAP_THRESH_TBL[] =
{
	{  -4, 310, 25, 7, 32, 7, 20 },
	{  39, 310, 20, 7, 25, 7, 16 },
	{  45, 310, 16, 7, 20, 7, 12 },
	{  49, 250, 16, 7, 20, 7, 12 },
	{  54, 250, 11, 7, 15, 7, 9  },
	{ 999, 200, 11, 7, 15, 7, 9  },
	{ 999, 200, 11, 7, 15, 7, 9  },
	{ 999, 200, 11, 7, 15, 7, 9  },
	{ 999, 200, 11, 7, 15, 7, 9  },
	{ 999, 200, 11, 7, 15, 7, 9  },
	{ 999, 200, 11, 7, 15, 7, 9  },
	{ 999, 200, 11, 7, 15, 7, 9  },
};
static const int SHTRPD_UP_TEMP_SNAP_THRESH_TBL_SIZE = 
	sizeof(SHTRPD_UP_TEMP_SNAP_THRESH_TBL) / sizeof(struct shtrpd_snap_thresh_tbl_info);

static struct shtrpd_snap_thresh_tbl_info SHTRPD_DOWN_TEMP_SNAP_THRESH_TBL[] =
{
	{  -5, 310, 25, 7, 32, 7, 20 },
	{  38, 310, 20, 7, 25, 7, 16 },
	{  44, 310, 16, 7, 20, 7, 12 },
	{  48, 250, 16, 7, 20, 7, 12 },
	{  53, 250, 11, 7, 15, 7, 9  },
	{ 999, 200, 11, 7, 15, 7, 9  },
	{ 999, 200, 11, 7, 15, 7, 9  },
	{ 999, 200, 11, 7, 15, 7, 9  },
	{ 999, 200, 11, 7, 15, 7, 9  },
	{ 999, 200, 11, 7, 15, 7, 9  },
	{ 999, 200, 11, 7, 15, 7, 9  },
	{ 999, 200, 11, 7, 15, 7, 9  },
};
static const int SHTRPD_DOWN_TEMP_SNAP_THRESH_TBL_SIZE = 
	sizeof(SHTRPD_DOWN_TEMP_SNAP_THRESH_TBL) / sizeof(struct shtrpd_snap_thresh_tbl_info);

#if defined( SHTRPD_LOG_DEBUG_ENABLE )
    SHTRPD_PARAM_DEF( SHTRPD_CHECK_CPU_TEMP_LOG_ENABLE, 	0);
    #define	SHTRPD_LOG_CHECK_CPU_TEMP(...)				\
        if(SHTRPD_CHECK_CPU_TEMP_LOG_ENABLE != 0){	\
            printk(KERN_DEBUG "[shtrpd] [cpu_temp]" __VA_ARGS__);	\
        }
#else
	#define	SHTRPD_LOG_CHECK_CPU_TEMP(...)
#endif /* SHTRPD_LOG_DEBUG_ENABLE */
#endif /* SHTRPD_CHECK_CPU_TEMP_ENABLE */

#ifdef SHTRPD_INTOUCH_GHOST_REJECTION_ENABLE
SHTRPD_PARAM_DEF( SHTRPD_INTOUCH_GHOST_REJECT_ENABLE,          1);
SHTRPD_PARAM_DEF( SHTRPD_INTOUCH_GHOST_REJECT_Z_THRESH,        650);
SHTRPD_PARAM_DEF( SHTRPD_INTOUCH_GHOST_REJECT_ZRATIO_THRESH,   20);
SHTRPD_PARAM_DEF( SHTRPD_INTOUCH_GHOST_REJECT_MOVE_THRESH,     10);
SHTRPD_PARAM_DEF( SHTRPD_INTOUCH_GHOST_REJECT_AREA_MAX,        2);

struct shtrpd_filter_intouch_ghost_reject_info{
	u8						is_ghost;
	u16						td_x;
	u16						td_y;
};
static struct shtrpd_filter_intouch_ghost_reject_info intouch_ghost_reject[MAX_TOUCHES];

#if defined( SHTRPD_LOG_DEBUG_ENABLE )
    SHTRPD_PARAM_DEF( SHTRPD_INTOUCH_GHOST_REJECT_LOG_ENABLE, 	0);
    #define	SHTRPD_LOG_INTOUCH_GHOST_REJECT(...) \
        if(SHTRPD_INTOUCH_GHOST_REJECT_LOG_ENABLE != 0){	\
            printk(KERN_DEBUG "[shtrpd] [intouch_ghost_reject]" __VA_ARGS__);	\
        }
#else
	#define	SHTRPD_LOG_INTOUCH_GHOST_REJECT(...)
#endif /* SHTRPD_LOG_DEBUG_ENABLE */

#endif /* SHTRPD_INTOUCH_GHOST_REJECTION_ENABLE */

int shtrpd_err_log  = 1;
int shtrpd_warn_log = 0;
int shtrpd_info_log = 0;
int shtrpd_dbg_log  = 0;
int shtrpd_dump_log = 0;
int shtrpd_snap_log = 0;

#if defined (CONFIG_ANDROID_ENGINEERING)
module_param(shtrpd_err_log,  int, 0600);
module_param(shtrpd_warn_log, int, 0600);
module_param(shtrpd_info_log, int, 0600);
module_param(shtrpd_dbg_log,  int, 0600);
module_param(shtrpd_dump_log, int, 0600);
module_param(shtrpd_snap_log, int, 0600);
module_param(shtrpd_touchevent_sw,  int, 0600);
module_param(shtrpd_showreset_enable,  int, 0600);
module_param(shtrpd_i2cerror_reset_enable,  int, 0600);
module_param(shtrpd_fw_poll_retry_cnt,  int, 0600);
module_param(shtrpd_fw_poll_delay,  int, 0600);
module_param(shtrpd_bootloader_key_wait,  int, 0600);
module_param(shtrpd_exec_app_wait,  int, 0600);
module_param(shtrpd_handshake_wait,  int, 0600);

module_param(shtrpd_touchmultiplier_val,   int, 0600);
module_param(shtrpd_intouchmultiplier_val, int, 0600);
module_param(shtrpd_snapthreshold_val,     int, 0600);
module_param(shtrpd_proxthreshold_flipclose_val,   int, 0600);
module_param(shtrpd_touchmultiplier_flipclose_val, int, 0600);
module_param(shtrpd_pmproxthreshold_flipclose_val,   int, 0600);
module_param(shtrpd_intouchmultiplier_flipclose_val, int, 0600);
module_param(shtrpd_lptime_flipclose_val, int, 0600);

module_param(shtrpd_atic_adj_disable, int, 0600);
module_param(shtrpd_atic_adj_wait,  int, 0600);
module_param(shtrpd_temp_drift,  int, 0600);

#if defined (SHTRPD_FAIL_TOUCH_CLEAR_ENABLE)
module_param(fail_touch_clear_reati_req,  int, 0600);
#endif /* SHTRPD_FAIL_TOUCH_CLEAR_ENABLE */
#if defined (SHTRPD_CHECK_CPU_TEMP_ENABLE)
module_param(SHTRPD_CHECK_CPU_TEMP, int, 0600);
#endif /* SHTRPD_CHECK_CPU_TEMP_ENABLE */

#endif /* CONFIG_ANDROID_ENGINEERING */

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */
#define SHTRPD_ERR(fmt, args...) \
        if(shtrpd_err_log == 1) { \
            printk("[SHTRPD_ERROR][%s] " fmt, __func__, ## args); \
        }

#define SHTRPD_WARN(fmt, args...) \
        if(shtrpd_warn_log == 1) { \
            printk("[SHTRPD_WARN][%s] " fmt, __func__, ## args); \
        }

#define SHTRPD_INFO(fmt, args...) \
        if(shtrpd_info_log == 1) { \
            printk("[SHTRPD_INFO][%s] " fmt, __func__, ## args); \
        }

#define SHTRPD_DBG(fmt, args...) \
        if(shtrpd_dbg_log == 1) { \
            printk("[SHTRPD_DBG][%s] " fmt, __func__, ## args); \
        }

#define SHTRPD_SNAP(fmt, args...) \
        if(shtrpd_snap_log == 1) { \
            printk("[SHTRPD_SNAP][%s] " fmt, __func__, ## args); \
        }

/* ------------------------------------------------------------------------- */
/* FUNCTIONS                                                                 */
/* ------------------------------------------------------------------------- */
static void shtrpd_user_request(void);
static int shtrpd_reati(struct shtrpd_ts_data *data);
static int shtrpd_switch_event_mode(struct shtrpd_ts_data *data, bool event);
static void shtrpd_reset_setting(void);
static int shtrpd_flip_close_proc(void);
static int shtrpd_user_thresholds(struct shtrpd_ts_data *data, u8 enable);
static int shtrpd_user_switch_event_mode(struct shtrpd_ts_data *data, u8 enable);

#if (defined(SHTRPD_INTERRUPT_TIMING_CHECK_ENABLE) || defined(SHTRPD_FAIL_TOUCH_CLEAR_ENABLE))
static long shtrpd_int_timing_check_reati(void);
#endif

#if defined( SHTRPD_CHECK_CPU_TEMP_ENABLE )
/* ------------------------------------------------------------------------- */
/* shtrpd_read_cpu_temp_file                                                 */
/* ------------------------------------------------------------------------- */
static int shtrpd_read_cpu_temp_file(char *filename, int *temp_p)
{
    int ret = 0;
    struct file *file;
    mm_segment_t fs;
    char buf[16];
    int nr_read;
    long cur_temp;

    file = filp_open(filename, O_RDONLY, 0); 

    if(IS_ERR(file)){
        return -1;
    }

    fs = get_fs();
    set_fs(get_ds());

    nr_read = file->f_op->read(file, buf, sizeof(buf)-1, &file->f_pos);
    buf[nr_read] = '\0';

    if(nr_read == 0){
        ret = -1;
    }else{
        ret = kstrtol(buf, 10, &cur_temp);
        *temp_p = (int)cur_temp;
    }

    set_fs(fs);
    filp_close(file, NULL);

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_get_up_temp_snap_threshold                                         */
/* ------------------------------------------------------------------------- */
static void shtrpd_get_up_temp_snap_threshold(long temp,
    int *snap, int *mult, int *shift, int *mult2, int *shift2, int *intouch)
{
    int i;
    
    for(i = 0;i < SHTRPD_UP_TEMP_SNAP_THRESH_TBL_SIZE;i++){
        if(temp <= SHTRPD_UP_TEMP_SNAP_THRESH_TBL[i].temp){
            *snap   = SHTRPD_UP_TEMP_SNAP_THRESH_TBL[i].snap_threshold;
            *mult   = SHTRPD_UP_TEMP_SNAP_THRESH_TBL[i].touch_multiplier_val;
            *shift  = SHTRPD_UP_TEMP_SNAP_THRESH_TBL[i].touch_shifter_val;
            *mult2  = SHTRPD_UP_TEMP_SNAP_THRESH_TBL[i].touch_multiplier2_val;
            *shift2 = SHTRPD_UP_TEMP_SNAP_THRESH_TBL[i].touch_shifter2_val;
            *intouch= SHTRPD_UP_TEMP_SNAP_THRESH_TBL[i].in_touch_multiplier_val;
            return;
        }
    }
    
    *snap   = SHTRPD_DOWN_TEMP_SNAP_THRESH_TBL[SHTRPD_UP_TEMP_SNAP_THRESH_TBL_SIZE - 1].snap_threshold;
    *mult   = SHTRPD_DOWN_TEMP_SNAP_THRESH_TBL[SHTRPD_UP_TEMP_SNAP_THRESH_TBL_SIZE - 1].touch_multiplier_val;
    *shift  = SHTRPD_DOWN_TEMP_SNAP_THRESH_TBL[SHTRPD_UP_TEMP_SNAP_THRESH_TBL_SIZE - 1].touch_shifter_val;
    *mult2  = SHTRPD_DOWN_TEMP_SNAP_THRESH_TBL[SHTRPD_UP_TEMP_SNAP_THRESH_TBL_SIZE - 1].touch_multiplier2_val;
    *shift2 = SHTRPD_DOWN_TEMP_SNAP_THRESH_TBL[SHTRPD_UP_TEMP_SNAP_THRESH_TBL_SIZE - 1].touch_shifter2_val;
    *intouch= SHTRPD_DOWN_TEMP_SNAP_THRESH_TBL[SHTRPD_UP_TEMP_SNAP_THRESH_TBL_SIZE - 1].in_touch_multiplier_val;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_get_down_temp_snap_threshold                                       */
/* ------------------------------------------------------------------------- */
static void shtrpd_get_down_temp_snap_threshold(long temp,
    int *snap, int *mult, int *shift, int *mult2, int *shift2, int *intouch)
{
    int i;
    
    for(i = 0;i < SHTRPD_DOWN_TEMP_SNAP_THRESH_TBL_SIZE;i++){
        if(temp <= SHTRPD_DOWN_TEMP_SNAP_THRESH_TBL[i].temp){
            *snap   = SHTRPD_DOWN_TEMP_SNAP_THRESH_TBL[i].snap_threshold;
            *mult   = SHTRPD_DOWN_TEMP_SNAP_THRESH_TBL[i].touch_multiplier_val;
            *shift  = SHTRPD_DOWN_TEMP_SNAP_THRESH_TBL[i].touch_shifter_val;
            *mult2  = SHTRPD_DOWN_TEMP_SNAP_THRESH_TBL[i].touch_multiplier2_val;
            *shift2 = SHTRPD_DOWN_TEMP_SNAP_THRESH_TBL[i].touch_shifter2_val;
            *intouch= SHTRPD_DOWN_TEMP_SNAP_THRESH_TBL[i].in_touch_multiplier_val;
            return;
        }
    }
    
    *snap   = SHTRPD_DOWN_TEMP_SNAP_THRESH_TBL[SHTRPD_DOWN_TEMP_SNAP_THRESH_TBL_SIZE - 1].snap_threshold;
    *mult   = SHTRPD_DOWN_TEMP_SNAP_THRESH_TBL[SHTRPD_DOWN_TEMP_SNAP_THRESH_TBL_SIZE - 1].touch_multiplier_val;
    *shift  = SHTRPD_DOWN_TEMP_SNAP_THRESH_TBL[SHTRPD_DOWN_TEMP_SNAP_THRESH_TBL_SIZE - 1].touch_shifter_val;
    *mult2  = SHTRPD_DOWN_TEMP_SNAP_THRESH_TBL[SHTRPD_DOWN_TEMP_SNAP_THRESH_TBL_SIZE - 1].touch_multiplier2_val;
    *shift2 = SHTRPD_DOWN_TEMP_SNAP_THRESH_TBL[SHTRPD_DOWN_TEMP_SNAP_THRESH_TBL_SIZE - 1].touch_shifter2_val;
    *intouch= SHTRPD_DOWN_TEMP_SNAP_THRESH_TBL[SHTRPD_DOWN_TEMP_SNAP_THRESH_TBL_SIZE - 1].in_touch_multiplier_val;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_thresholds_check_thresh_change                                     */
/* ------------------------------------------------------------------------- */
static int shtrpd_thresholds_check_thresh_change(int snap, int mult, int shift, int mult2, int shift2, int intouch)
{
    if((shtrpd_snapthreshold_val    != snap)   ||
       (shtrpd_touchmultiplier_val  != mult)   ||
       (shtrpd_touchshifter_val     != shift)  ||
       (shtrpd_touchmultiplier2_val != mult2)  ||
       (shtrpd_touchshifter2_val    != shift2) ||
       (shtrpd_intouchmultiplier_val!= intouch))
	{
		return 1;
	}
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_thresholds_check_cpu_temp                                          */
/* ------------------------------------------------------------------------- */
static int shtrpd_thresholds_check_cpu_temp(int force_update)
{
    struct shtrpd_ts_data *data = i2c_get_clientdata(shtrpd_client);
    int ret = 0;
    int cur_cpu_temp;
    int up_snap, up_mult, up_shift, up_mult2, up_shift2, up_intouch;
    int down_snap, down_mult, down_shift, down_mult2, down_shift2, down_intouch;

    
    if(!SHTRPD_CHECK_CPU_TEMP){
        if(force_update){
            ret = shtrpd_user_thresholds(data, 1);
            if(ret < 0) {
                SHTRPD_ERR("shtrpd_user_thresholds err, ret:%d\n", ret);
            }
        }
        return ret;
    }
    
    if(shtrpd_read_cpu_temp_file(SHTRPD_CPU_TEMP_FILE, &cur_cpu_temp) == 0){
        SHTRPD_LOG_CHECK_CPU_TEMP("cpu_temp:%d\n", (int)cur_cpu_temp);
        if(cur_cpu_temp != shtrpd_old_cpu_temp){
            shtrpd_get_up_temp_snap_threshold(cur_cpu_temp,
                                              &up_snap,
                                              &up_mult,
                                              &up_shift,
                                              &up_mult2,
                                              &up_shift2,
                                              &up_intouch);

            shtrpd_get_down_temp_snap_threshold(cur_cpu_temp,
                                              &down_snap,
                                              &down_mult,
                                              &down_shift,
                                              &down_mult2,
                                              &down_shift2,
                                              &down_intouch);
            
	        if(shtrpd_thresholds_check_thresh_change(up_snap, up_mult, up_shift, up_mult2, up_shift2, up_intouch) != 0 &&
	           shtrpd_thresholds_check_thresh_change(down_snap, down_mult, down_shift, down_mult2, down_shift2, down_intouch) != 0)
	        {
	            if(cur_cpu_temp > shtrpd_old_cpu_temp){
					shtrpd_new_snap_threshold		= up_snap;
					shtrpd_new_touchmultiplier_val	= up_mult;
					shtrpd_new_touchshifter_val		= up_shift;
					shtrpd_new_touchmultiplier2_val	= up_mult2;
					shtrpd_new_touchshifter2_val	= up_shift2;
					shtrpd_new_intouchmultiplier_val= up_intouch;
				}else{
					shtrpd_new_snap_threshold		= down_snap;
					shtrpd_new_touchmultiplier_val	= down_mult;
					shtrpd_new_touchshifter_val		= down_shift;
					shtrpd_new_touchmultiplier2_val	= down_mult2;
					shtrpd_new_touchshifter2_val	= down_shift2;
					shtrpd_new_intouchmultiplier_val= down_intouch;
				}
			}
		}

        shtrpd_old_cpu_temp = cur_cpu_temp;
    }
    
    if(force_update == 0 && initialSetup == true){
        return ret;
    }
    
    if(force_update || 
        (shtrpd_snapthreshold_val    != shtrpd_new_snap_threshold) ||
        (shtrpd_touchmultiplier_val  != shtrpd_new_touchmultiplier_val) ||
        (shtrpd_touchshifter_val     != shtrpd_new_touchshifter_val) ||
        (shtrpd_touchmultiplier2_val != shtrpd_new_touchmultiplier2_val) ||
        (shtrpd_touchshifter2_val    != shtrpd_new_touchshifter2_val) ||
        (shtrpd_intouchmultiplier_val!= shtrpd_new_intouchmultiplier_val))
    {
        SHTRPD_LOG_CHECK_CPU_TEMP("Change SNAPTHRESHOLD %d -> %d\n",
                                    shtrpd_snapthreshold_val, shtrpd_new_snap_threshold);
        SHTRPD_LOG_CHECK_CPU_TEMP("Change TOUCHMULTIPLIER_VAL %d -> %d\n",
                                    shtrpd_touchmultiplier_val, shtrpd_new_touchmultiplier_val);
        SHTRPD_LOG_CHECK_CPU_TEMP("Change TOUCHSHIFTER_VAL %d -> %d\n",
                                    shtrpd_touchshifter_val, shtrpd_new_touchshifter_val);
        SHTRPD_LOG_CHECK_CPU_TEMP("Change TOUCHMULTIPLIER2_VAL %d -> %d\n",
                                    shtrpd_touchmultiplier2_val, shtrpd_new_touchmultiplier2_val);
        SHTRPD_LOG_CHECK_CPU_TEMP("Change TOUCHSHIFTER2_VAL %d -> %d\n",
                                    shtrpd_touchshifter2_val, shtrpd_new_touchshifter2_val);
        SHTRPD_LOG_CHECK_CPU_TEMP("Change INTOUCHMULTIPLIER_VAL %d -> %d\n",
                                    shtrpd_intouchmultiplier_val, shtrpd_new_intouchmultiplier_val);
                                    
        shtrpd_snapthreshold_val    = shtrpd_new_snap_threshold;
        shtrpd_touchmultiplier_val  = shtrpd_new_touchmultiplier_val;
        shtrpd_touchshifter_val     = shtrpd_new_touchshifter_val;
        shtrpd_touchmultiplier2_val = shtrpd_new_touchmultiplier2_val;
        shtrpd_touchshifter2_val    = shtrpd_new_touchshifter2_val;
        shtrpd_intouchmultiplier_val= shtrpd_new_intouchmultiplier_val;

        ret = shtrpd_user_thresholds(data, 1);
        if(ret < 0) {
            SHTRPD_ERR("shtrpd_user_thresholds err, ret:%d\n", ret);
        }
    }
    
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_check_cpu_temp_func                                                */
/* ------------------------------------------------------------------------- */
static void shtrpd_check_cpu_temp_func(struct work_struct *work)
{
    mutex_lock(&shtrpd_ioctl_lock);
    if(shtrpd_flip_state) {
        shtrpd_thresholds_check_cpu_temp(0);
        SHTRPD_LOG_CHECK_CPU_TEMP("cpu temp check timer re-start\n");
        queue_delayed_work(shtrpd_work_queue, &shtrpd_work_cpu_temp, msecs_to_jiffies(SHTRPD_CHECK_CPU_TEMP_INTERVAL));
    }
    mutex_unlock(&shtrpd_ioctl_lock);
}
#if defined (CONFIG_ANDROID_ENGINEERING)
static struct kobject   *shtrpd_snapdebug_kobj;

static ssize_t shtrpd_snapdebug_uptable_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    int i;
    char tmp[128];
    
    strcpy(buf, "[UP TEMP] SNAP THRESHOLD TABLE\n");
    
    for(i = 0;i < SHTRPD_UP_TEMP_SNAP_THRESH_TBL_SIZE;i++){
        sprintf(tmp, "[%d]  <= %4d : snap = %d, mult = %d, shift = %d, mult2 = %d, shift2 = %d, intouch = %d\n",
            i,
            SHTRPD_UP_TEMP_SNAP_THRESH_TBL[i].temp,
            SHTRPD_UP_TEMP_SNAP_THRESH_TBL[i].snap_threshold,
            SHTRPD_UP_TEMP_SNAP_THRESH_TBL[i].touch_multiplier_val,
            SHTRPD_UP_TEMP_SNAP_THRESH_TBL[i].touch_shifter_val,
            SHTRPD_UP_TEMP_SNAP_THRESH_TBL[i].touch_multiplier2_val,
            SHTRPD_UP_TEMP_SNAP_THRESH_TBL[i].touch_shifter2_val,
            SHTRPD_UP_TEMP_SNAP_THRESH_TBL[i].in_touch_multiplier_val);
        strcat(buf, tmp);
    }
    
    return strlen(buf);
}
static ssize_t shtrpd_snapdebug_uptable_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
    int index, temp, snap, mult, shift, mult2, shift2, intouch;
    
    sscanf(buf, "%d %d %d %d %d %d %d %d", &index, &temp, &snap, &mult, &shift, &mult2, &shift2, &intouch );
    
    if(index >= SHTRPD_UP_TEMP_SNAP_THRESH_TBL_SIZE){
        return -1;
    }
    
    SHTRPD_UP_TEMP_SNAP_THRESH_TBL[index].temp = temp;
    SHTRPD_UP_TEMP_SNAP_THRESH_TBL[index].snap_threshold = snap;
    SHTRPD_UP_TEMP_SNAP_THRESH_TBL[index].touch_multiplier_val = mult;
    SHTRPD_UP_TEMP_SNAP_THRESH_TBL[index].touch_shifter_val = shift;
    SHTRPD_UP_TEMP_SNAP_THRESH_TBL[index].touch_multiplier2_val = mult2;
    SHTRPD_UP_TEMP_SNAP_THRESH_TBL[index].touch_shifter2_val = shift2;
    SHTRPD_UP_TEMP_SNAP_THRESH_TBL[index].in_touch_multiplier_val = intouch;
    
    return count;
}
static struct kobj_attribute shtrpd_snapdebug_uptable = 
    __ATTR(up_table, (S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP), shtrpd_snapdebug_uptable_show, shtrpd_snapdebug_uptable_store);

static ssize_t shtrpd_snapdebug_downtable_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    int i;
    char tmp[128];
    
    strcpy(buf, "[DOWN TEMP] SNAP THRESHOLD TABLE\n");
    
    for(i = 0;i < SHTRPD_DOWN_TEMP_SNAP_THRESH_TBL_SIZE;i++){
        sprintf(tmp, "[%d]  <= %4d : snap = %d, mult = %d, shift = %d, mult2 = %d, shift2 = %d, intouch = %d\n",
            i,
            SHTRPD_DOWN_TEMP_SNAP_THRESH_TBL[i].temp,
            SHTRPD_DOWN_TEMP_SNAP_THRESH_TBL[i].snap_threshold,
            SHTRPD_DOWN_TEMP_SNAP_THRESH_TBL[i].touch_multiplier_val,
            SHTRPD_DOWN_TEMP_SNAP_THRESH_TBL[i].touch_shifter_val,
            SHTRPD_DOWN_TEMP_SNAP_THRESH_TBL[i].touch_multiplier2_val,
            SHTRPD_DOWN_TEMP_SNAP_THRESH_TBL[i].touch_shifter2_val,
            SHTRPD_DOWN_TEMP_SNAP_THRESH_TBL[i].in_touch_multiplier_val);
        strcat(buf, tmp);
    }
    
    return strlen(buf);
}
static ssize_t shtrpd_snapdebug_downtable_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
    int index, temp, snap, mult, shift, mult2, shift2, intouch;
    
    sscanf(buf, "%d %d %d %d %d %d %d %d", &index, &temp, &snap, &mult, &shift, &mult2, &shift2, &intouch );
    
    if(index >= SHTRPD_DOWN_TEMP_SNAP_THRESH_TBL_SIZE){
        return -1;
    }
    
    SHTRPD_DOWN_TEMP_SNAP_THRESH_TBL[index].temp = temp;
    SHTRPD_DOWN_TEMP_SNAP_THRESH_TBL[index].snap_threshold = snap;
    SHTRPD_DOWN_TEMP_SNAP_THRESH_TBL[index].touch_multiplier_val = mult;
    SHTRPD_DOWN_TEMP_SNAP_THRESH_TBL[index].touch_shifter_val = shift;
    SHTRPD_DOWN_TEMP_SNAP_THRESH_TBL[index].touch_multiplier2_val = mult2;
    SHTRPD_DOWN_TEMP_SNAP_THRESH_TBL[index].touch_shifter2_val = shift2;
    SHTRPD_DOWN_TEMP_SNAP_THRESH_TBL[index].in_touch_multiplier_val = intouch;
    
    return count;
}
static struct kobj_attribute shtrpd_snapdebug_downtable = 
    __ATTR(down_table, (S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP), shtrpd_snapdebug_downtable_show, shtrpd_snapdebug_downtable_store);

static struct attribute *attrs_snapdebug[] = {
    &shtrpd_snapdebug_uptable.attr,
    &shtrpd_snapdebug_downtable.attr,
    NULL
};

static struct attribute_group shtrpd_attr_grp_snapdebug = {
    .name = "snap_debug",
    .attrs = attrs_snapdebug,
};


static void shtrpd_snapdebug_init(void)
{
    shtrpd_snapdebug_kobj = kobject_create_and_add("shtrpd", kernel_kobj);
    
    if(shtrpd_snapdebug_kobj == NULL){
        SHTRPD_ERR("kobj create failed : shtrpd\n");
    }else{
        if(sysfs_create_group(shtrpd_snapdebug_kobj, &shtrpd_attr_grp_snapdebug)){
            SHTRPD_ERR("kobj create failed : shtrpd_attr_grp_snapdebug\n");
        }
    }
}
#endif /* CONFIG_ANDROID_ENGINEERING */

#endif /* SHTRPD_CHECK_CPU_TEMP_ENABLE */

#if defined( SHTRPD_DRAG_SMOOTH_ENABLE )
#if defined( SHTRPD_LOG_DEBUG_ENABLE )
/* ------------------------------------------------------------------------- */
/* SHTRPD_DRAG_SMOOTH_GET_DEC_FROM_FIXED                                     */
/* ------------------------------------------------------------------------- */
int SHTRPD_DRAG_SMOOTH_GET_DEC_FROM_FIXED(int val)
{
	int i;
	int dec_val = (val) - SHTRPD_DRAG_SMOOTH_INT_TO_FIXED(SHTRPD_DRAG_SMOOTH_FIXED_TO_INT((val)));
	int ret = 0;
	int add_val = 10;
	for(i = 0; i < SHTRPD_DRAG_SMOOTH_FIXED_SHIFT; i++){
		add_val *= 10;
	}
	for(i = 1; i < SHTRPD_DRAG_SMOOTH_FIXED_SHIFT; i++){
		add_val /= 2;
		if((dec_val >> (SHTRPD_DRAG_SMOOTH_FIXED_SHIFT - i)) & 0x01){
			ret += add_val;
		}
	}
	return ret/1000000;
}
#endif /* SHTRPD_LOG_DEBUG_ENABLE */

/* ------------------------------------------------------------------------- */
/* shtrpd_filter_pos_compensation                                            */
/* ------------------------------------------------------------------------- */
static void shtrpd_filter_pos_compensation(int xy, u8 finger_state)
{
	int inc_ave_FIXED,temp_FIXED;
	int i;
	int drag_smooth_leave_max_dot_FIXED;
	int last_history;
	int last_history_FIXED;

	if(!SHTRPD_DRAG_SMOOTH){
		return;
	}

	for(i = 0;i < MAX_TOUCHES ;i++){
		if((finger_state & (1<<i))){
			if((shtrpd_finger_state_old & (1<<i))){
				if(drag_hist[i][xy].count >= SHTRPD_DRAG_SMOOTH_COUNT_MIN){
					drag_smooth_leave_max_dot_FIXED = SHTRPD_DRAG_SMOOTH_INT_TO_FIXED(SHTRPD_DRAG_SMOOTH_LEAVE_MAX_DOT);
					last_history		= drag_hist[i][xy].history[drag_hist[i][xy].count - 1];
					last_history_FIXED	= SHTRPD_DRAG_SMOOTH_INT_TO_FIXED(last_history);

					inc_ave_FIXED = 
						SHTRPD_DRAG_SMOOTH_INT_TO_FIXED(last_history- drag_hist[i][xy].history[0]) / (drag_hist[i][xy].count-1);
					SHTRPD_LOG_DRAG_SMOOTH("   [X]inc_ave=%d.%03d history[0]=%d\n", 
						SHTRPD_DRAG_SMOOTH_FIXED_TO_INT(inc_ave_FIXED), 
						SHTRPD_DRAG_SMOOTH_GET_DEC_FROM_FIXED(inc_ave_FIXED), 
						drag_hist[i][xy].history[0]);
						
					if(xy == SHTRPD_POSTYPE_X){
						if(drag_hist[i][xy].history_old == last_history){
							shtrpd_touches[i].XPos = SHTRPD_DRAG_SMOOTH_FIXED_TO_INT(drag_hist[i][xy].pre_comp_history_FIXED);
						}else{
							if(((drag_hist[i][xy].pre_comp_history_FIXED + inc_ave_FIXED) - last_history_FIXED ) > drag_smooth_leave_max_dot_FIXED){
								temp_FIXED = last_history_FIXED + drag_smooth_leave_max_dot_FIXED;
								SHTRPD_LOG_DRAG_SMOOTH("   [X]temp=%d.%03d(upper limit)\n", SHTRPD_DRAG_SMOOTH_FIXED_TO_INT(temp_FIXED), SHTRPD_DRAG_SMOOTH_GET_DEC_FROM_FIXED(temp_FIXED));
							}else if(((drag_hist[i][xy].pre_comp_history_FIXED + inc_ave_FIXED) - last_history_FIXED ) < (0-drag_smooth_leave_max_dot_FIXED)){
								temp_FIXED = last_history_FIXED - drag_smooth_leave_max_dot_FIXED;
								SHTRPD_LOG_DRAG_SMOOTH("   [X]temp=%d.%03d(lower limit)\n", SHTRPD_DRAG_SMOOTH_FIXED_TO_INT(temp_FIXED), SHTRPD_DRAG_SMOOTH_GET_DEC_FROM_FIXED(temp_FIXED));
							}else{
								temp_FIXED = drag_hist[i][xy].pre_comp_history_FIXED + inc_ave_FIXED;
								SHTRPD_LOG_DRAG_SMOOTH("   [X]temp=%d.%03d\n", SHTRPD_DRAG_SMOOTH_FIXED_TO_INT(temp_FIXED), SHTRPD_DRAG_SMOOTH_GET_DEC_FROM_FIXED(temp_FIXED));
							}
							
							if(temp_FIXED < 0){
								shtrpd_touches[i].XPos = 0;
								drag_hist[i][xy].pre_comp_history_FIXED = 0;
							}else if(temp_FIXED >= SHTRPD_DRAG_SMOOTH_INT_TO_FIXED(SHTRPD_MAX_X)){
								shtrpd_touches[i].XPos = SHTRPD_MAX_X -1;
								drag_hist[i][xy].pre_comp_history_FIXED = SHTRPD_DRAG_SMOOTH_INT_TO_FIXED(shtrpd_touches[i].XPos);
							} else{
								shtrpd_touches[i].XPos = SHTRPD_DRAG_SMOOTH_FIXED_TO_INT(temp_FIXED);
								drag_hist[i][xy].pre_comp_history_FIXED = temp_FIXED;
							}
							drag_hist[i][xy].history_old = last_history;
						}
					}else{
						if(drag_hist[i][xy].history_old == last_history){
							shtrpd_touches[i].YPos = SHTRPD_DRAG_SMOOTH_FIXED_TO_INT(drag_hist[i][xy].pre_comp_history_FIXED);
						}else{
							if(((drag_hist[i][xy].pre_comp_history_FIXED + inc_ave_FIXED) - last_history_FIXED ) > drag_smooth_leave_max_dot_FIXED){
								temp_FIXED = last_history_FIXED + drag_smooth_leave_max_dot_FIXED;
								SHTRPD_LOG_DRAG_SMOOTH("   [Y]temp=%d.%03d(upper limit)\n", SHTRPD_DRAG_SMOOTH_FIXED_TO_INT(temp_FIXED), SHTRPD_DRAG_SMOOTH_GET_DEC_FROM_FIXED(temp_FIXED));
							}else if(((drag_hist[i][xy].pre_comp_history_FIXED + inc_ave_FIXED) - last_history_FIXED ) < (0-drag_smooth_leave_max_dot_FIXED)){
								temp_FIXED = last_history_FIXED - drag_smooth_leave_max_dot_FIXED;
								SHTRPD_LOG_DRAG_SMOOTH("   [Y]temp=%d.%03d(lower limit)\n", SHTRPD_DRAG_SMOOTH_FIXED_TO_INT(temp_FIXED), SHTRPD_DRAG_SMOOTH_GET_DEC_FROM_FIXED(temp_FIXED));
							}else{
								temp_FIXED = drag_hist[i][xy].pre_comp_history_FIXED + inc_ave_FIXED;
								SHTRPD_LOG_DRAG_SMOOTH("   [Y]temp=%d.%03d\n", SHTRPD_DRAG_SMOOTH_FIXED_TO_INT(temp_FIXED), SHTRPD_DRAG_SMOOTH_GET_DEC_FROM_FIXED(temp_FIXED));
							}

							if(temp_FIXED < 0){
								shtrpd_touches[i].YPos = 0;
								drag_hist[i][xy].pre_comp_history_FIXED = 0;
							}else if(temp_FIXED >= SHTRPD_DRAG_SMOOTH_INT_TO_FIXED(SHTRPD_MAX_Y)){
								shtrpd_touches[i].YPos = SHTRPD_MAX_Y -1;
								drag_hist[i][xy].pre_comp_history_FIXED = SHTRPD_DRAG_SMOOTH_INT_TO_FIXED(shtrpd_touches[i].YPos);
							} else{
								shtrpd_touches[i].YPos = SHTRPD_DRAG_SMOOTH_FIXED_TO_INT(temp_FIXED);
								drag_hist[i][xy].pre_comp_history_FIXED = temp_FIXED;
							}
							drag_hist[i][xy].history_old = last_history;
						}
					}
				}
			}
		}
	}
}

/* ------------------------------------------------------------------------- */
/* shtrpd_init_drag_hist                                                     */
/* ------------------------------------------------------------------------- */
static void shtrpd_init_drag_hist(int xy, int finger, int pos)
{
	drag_hist[finger][xy].pre   = pos;
	drag_hist[finger][xy].count = 0;
	drag_hist[finger][xy].count_up_base = 0;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_init_drag_hist                                                     */
/* ------------------------------------------------------------------------- */
static void shtrpd_add_drag_hist(int xy, int finger, int pos)
{
	int pre = drag_hist[finger][xy].pre;
	u8 dir  = (pos > pre)? SHTRPD_DRAG_DIR_PLUS :
			  (pos < pre)? SHTRPD_DRAG_DIR_MINUS :
						   SHTRPD_DRAG_DIR_NONE;

	int i;
	int drag_smooth_count_limit;
	int drag_smooth_count_limit_new;

	SHTRPD_DBG("add drag hist[%d][%s] pre = %d, cur = %d, dir = %s, cnt = %d\n",
		finger, (xy == SHTRPD_POSTYPE_X)? "X" : "Y",
		pre, pos, 
		(dir == SHTRPD_DRAG_DIR_PLUS)? "PLUS" : (dir == SHTRPD_DRAG_DIR_MINUS)? "MINUS" : "NONE",
		drag_hist[finger][xy].count);

	if(dir != SHTRPD_DRAG_DIR_NONE){
		if(drag_hist[finger][xy].count == 0){
			if(SHTRPD_DRAG_SMOOTH){
				drag_hist[finger][xy].history[drag_hist[finger][xy].count] = pos;
			}
			drag_hist[finger][xy].dir   = dir;
			drag_hist[finger][xy].count = 1;
		}else{
			if(SHTRPD_DRAG_SMOOTH){
				drag_smooth_count_limit = drag_hist[finger][xy].count;
				drag_smooth_count_limit_new = SHTRPD_DRAG_SMOOTH_COUNT_MIN + (drag_hist[finger][xy].count_up_base / SHTRPD_DRAG_SMOOTH_COUNT_UP_STEP);
				if(drag_smooth_count_limit < drag_smooth_count_limit_new){
					drag_smooth_count_limit = drag_smooth_count_limit_new;
				}
				if(drag_smooth_count_limit > SHTRPD_DRAG_SMOOTH_COUNT_MAX){
					drag_smooth_count_limit = SHTRPD_DRAG_SMOOTH_COUNT_MAX;
				}

				if(drag_hist[finger][xy].dir != dir){
					drag_smooth_count_limit = SHTRPD_DRAG_SMOOTH_COUNT_MIN;
					if(drag_smooth_count_limit < drag_hist[finger][xy].count){
						for(i= 0; i < drag_smooth_count_limit; i++){
							drag_hist[finger][xy].history[i] = 
								drag_hist[finger][xy].history[drag_hist[finger][xy].count - drag_smooth_count_limit + i];
						}
						drag_hist[finger][xy].count = drag_smooth_count_limit;
					}
					drag_hist[finger][xy].count_up_base = 0;
				}
					
				if(drag_hist[finger][xy].count < SHTRPD_DRAG_SMOOTH_COUNT_MIN-1){
					drag_hist[finger][xy].history[drag_hist[finger][xy].count] = pos;
					drag_hist[finger][xy].pre_comp_history_FIXED = SHTRPD_DRAG_SMOOTH_INT_TO_FIXED(pos);
					drag_hist[finger][xy].count++;
				}else{
					if(drag_hist[finger][xy].count == drag_smooth_count_limit-1){
						drag_hist[finger][xy].count++;
					}else{
						for(i= 0; i < drag_smooth_count_limit-1; i++){
							drag_hist[finger][xy].history[i] = drag_hist[finger][xy].history[i+1];
						}
					}
					drag_hist[finger][xy].history[drag_smooth_count_limit-1] = pos;
					drag_hist[finger][xy].count_up_base++;
				}
			}
			
			if(!SHTRPD_DRAG_SMOOTH){
				drag_hist[finger][xy].count = 1;
			}
			drag_hist[finger][xy].dir = dir;
		}
		drag_hist[finger][xy].pre = pos;
	}
}
#endif /* SHTRPD_DRAG_SMOOTH_ENABLE */

#if defined(SHTRPD_EDGE_FAIL_TOUCH_REJECTION_ENABLE)
/* ------------------------------------------------------------------------- */
/* shtrpd_filter_edge_fail_touch_init                                        */
/* ------------------------------------------------------------------------- */
static void shtrpd_filter_edge_fail_touch_init(void)
{
    SHTRPD_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("Init.\n");

    edge_fail_touch_rej.enable     = 1;
    edge_fail_touch_rej.inhibit_id = 0;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_filter_edge_fail_touch_cancel                                      */
/* ------------------------------------------------------------------------- */
static void shtrpd_filter_edge_fail_touch_cancel(void *dev_id, int id)
{
    struct shtrpd_ts_data *data = dev_id;

    input_mt_slot(data->input_dev, id);
    input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, id+1);
    input_report_abs(data->input_dev, ABS_MT_POSITION_X, SHTRPD_TOUCH_CANCEL_COORDINATES_X);
    input_report_abs(data->input_dev, ABS_MT_POSITION_Y, SHTRPD_TOUCH_CANCEL_COORDINATES_Y);
    input_sync(data->input_dev);

    SHTRPD_INFO("ABS_MT_TRACKING_ID ID=%d ; STATE=1 ; X=%d ; Y=%d ; Z=%d (touch cancel)\n" ,
                 id, SHTRPD_TOUCH_CANCEL_COORDINATES_X, SHTRPD_TOUCH_CANCEL_COORDINATES_Y,
                 shtrpd_touches[id].touchStrength);
}

/* ------------------------------------------------------------------------- */
/* shtrpd_filter_edge_fail_touch_area_check                                  */
/* ------------------------------------------------------------------------- */
static u8 shtrpd_filter_edge_fail_touch_area_check(unsigned short x, unsigned short y)
{
    u8 area = 0;

    if(y >= SHTRPD_EDGE_FAIL_TOUCH_REJECT_AREA_THRESH_Y2){
        if(x <= SHTRPD_EDGE_FAIL_TOUCH_REJECT_AREA_THRESH_X2){
            area = 1;
        }
        else if(x >= (SHTRPD_MAX_X - 1 - SHTRPD_EDGE_FAIL_TOUCH_REJECT_AREA_THRESH_X2)){
            area = 2;
        }
    }else if(y >= SHTRPD_EDGE_FAIL_TOUCH_REJECT_AREA_THRESH_Y){
        if(x <= SHTRPD_EDGE_FAIL_TOUCH_REJECT_AREA_THRESH_X){
            area = 1;
        }
        else if(x >= (SHTRPD_MAX_X - 1 - SHTRPD_EDGE_FAIL_TOUCH_REJECT_AREA_THRESH_X)){
            area = 2;
        }
    }

    return area;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_filter_edge_fail_touch_zratio_check                                */
/* ------------------------------------------------------------------------- */
int shtrpd_filter_edge_fail_touch_zratio_check(int z0, int z1, int threshold)
{
    if(z0 <= 0){
        z0 = 1;
    }
    if(z1 <= 0){
        z1 = 1;
    }
    
    SHTRPD_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("z ratio check. z0 = %d / z1 = %d / ratio = %d / thresh = %d\n", 
                                                z0, z1, (z0 * 100) / z1, threshold);

    return (((z0 * 100) / z1) < threshold);
}

/* ------------------------------------------------------------------------- */
/* shtrpd_filter_edge_fail_touch_check                                       */
/* ------------------------------------------------------------------------- */
void shtrpd_filter_edge_fail_touch_check(void *dev_id, u8 *finger_state_p)
{
    int i;
    int fingerNum = 0;
    int inhibitId;
    int ids[2] = {0, 0};
    int area[2] = {0, 0};
    int z[2] = {1, 1};
    int tmpId;
    int timeCondition = 0;
    int distCondition = 0;
    int zCondition = 0;
    unsigned long curTime = jiffies;

    if(SHTRPD_EDGE_FAIL_TOUCH_REJECT_ENABLE == 0){
        return;
    }

    /* Check inhibit bit clear */
    if(edge_fail_touch_rej.inhibit_id){
        for(i = 0;i < MAX_TOUCHES ;i++){
            if((edge_fail_touch_rej.inhibit_id & (1 << i)) != 0){
                if((*finger_state_p & (1<<i)) == 0 ||
                    shtrpd_filter_edge_fail_touch_area_check(shtrpd_touches[i].XPos, shtrpd_touches[i].YPos) == 0)
                {
                    edge_fail_touch_rej.inhibit_id &= ~(1 << i);
                    SHTRPD_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("inhibit clear id : %d\n", i);
                }
            }
        }
    }
    
    /* Set touch-down time */
    for(i = 0, fingerNum = 0;i < MAX_TOUCHES ;i++){
        if((*finger_state_p & (1<<i))){
            if((shtrpd_finger_state_old & (1<<i)) == 0){
                edge_fail_touch_rej.td_time[i] = curTime;
            }
            
            if(fingerNum < 2){
                ids[fingerNum] = i;
            }
            fingerNum++;
        }
    }

    SHTRPD_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("is check enable. enable flag = %d / touch num = %d\n", 
        edge_fail_touch_rej.enable, fingerNum);

    if(edge_fail_touch_rej.enable == 0){
        if(fingerNum < 2){
            edge_fail_touch_rej.enable = 1;
        }

    }else if(fingerNum > 2){
        edge_fail_touch_rej.enable = 0;
   
    }else if(fingerNum == 2){
        SHTRPD_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("check start.\n");
        
        if(edge_fail_touch_rej.td_time[ids[0]] > edge_fail_touch_rej.td_time[ids[1]]){
            tmpId = ids[0];
            ids[0] = ids[1];
            ids[1] = tmpId;
        }

        SHTRPD_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("td sequence = [%d] -> [%d].\n", ids[0], ids[1]);
        
        area[0] = shtrpd_filter_edge_fail_touch_area_check(shtrpd_touches[ids[0]].XPos, shtrpd_touches[ids[0]].YPos);
        area[1] = shtrpd_filter_edge_fail_touch_area_check(shtrpd_touches[ids[1]].XPos, shtrpd_touches[ids[1]].YPos);
        
        if(area[0] != 0 || area[1] != 0){

            /* Touch down time check */
            if(area[0] != 0){
                timeCondition = time_after(edge_fail_touch_rej.td_time[ids[1]],
                                    edge_fail_touch_rej.td_time[ids[0]] + msecs_to_jiffies(SHTRPD_EDGE_FAIL_TOUCH_REJECT_EFFECT_TIME));

                SHTRPD_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("time condition = %d.\n", timeCondition);
            }
            
            /* Finger distance check */
            if(area[0] == area[1] ||
                (area[0] == 0 && area[1] != 0) || (area[0] != 0 && area[1] == 0))
            {
                unsigned long xdiff, ydiff;

                xdiff = abs(shtrpd_touches[ids[0]].XPos - shtrpd_touches[ids[1]].XPos);
                ydiff = abs(shtrpd_touches[ids[0]].YPos - shtrpd_touches[ids[1]].YPos);
                distCondition = (((xdiff * xdiff) + (ydiff * ydiff)) < 
                                    (SHTRPD_EDGE_FAIL_TOUCH_REJECT_DISTANCE * SHTRPD_EDGE_FAIL_TOUCH_REJECT_DISTANCE));

                SHTRPD_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("distance condition = %d.\n", distCondition);
            }
            
            /* Z ratio check */
            z[0] = shtrpd_touches[ids[0]].touchStrength;
            z[0] = (z[0] == 0)? 1: z[0];
            z[1] = shtrpd_touches[ids[1]].touchStrength;
            z[1] = (z[1] == 0)? 1: z[1];

            if(area[0] != 0 && area[1] != 0){
                if(z[0] < z[1]){
                    inhibitId = ids[0];
                    zCondition = shtrpd_filter_edge_fail_touch_zratio_check(z[1], z[0], SHTRPD_EDGE_FAIL_TOUCH_REJECT_Z_RATIO_2);
                }else{
                    inhibitId = ids[1];
                    zCondition = shtrpd_filter_edge_fail_touch_zratio_check(z[0], z[1], SHTRPD_EDGE_FAIL_TOUCH_REJECT_Z_RATIO);
                }
            }else if(area[0] == 0){
                /* area[1] != 0 && time[0] <= time[1] */
                inhibitId = ids[1];
                zCondition = shtrpd_filter_edge_fail_touch_zratio_check(z[0], z[1], SHTRPD_EDGE_FAIL_TOUCH_REJECT_Z_RATIO_2);
            }else{
                /* area[0] != 0 && area[1] == 0 && time[0] <= time[1] */
                inhibitId = ids[0];
                zCondition = shtrpd_filter_edge_fail_touch_zratio_check(z[1], z[0], SHTRPD_EDGE_FAIL_TOUCH_REJECT_Z_RATIO);
            }
            SHTRPD_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("z condition = %d.\n", zCondition);
            
            /* inhibit finger check */
            if(timeCondition || distCondition || zCondition){
                SHTRPD_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("disable check. time: %d / dist: %d / z: %d\n",
                                                        timeCondition, distCondition, zCondition);
                edge_fail_touch_rej.enable = 0;
            }else{
                SHTRPD_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("inhibit add id : %d\n", inhibitId);
                edge_fail_touch_rej.inhibit_id |= (1 << inhibitId);
                edge_fail_touch_rej.enable = 0;
                
                if((shtrpd_finger_state_old & (1 << inhibitId)) != 0){
                    shtrpd_filter_edge_fail_touch_cancel(dev_id, inhibitId);
                }
            }
        }else{
            SHTRPD_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("disable check by touch area.\n");
            edge_fail_touch_rej.enable = 0;
        }
    }

    /* reject inhibit finger */
    for(i = 0; i < MAX_TOUCHES; i++){
        if( (edge_fail_touch_rej.inhibit_id & (1 << i)) != 0 ){
            *finger_state_p &= ~(1 << i);
            SHTRPD_LOG_DBG_EDGE_FAIL_TOUCH_PRINT("[edge_fail_reject][%d] is inhibited\n", i);
        }
    }
}

#if defined(SHTRPD_GRIP_FAIL_FLICK_REJECTION_ENABLE)
/* ------------------------------------------------------------------------- */
/* shtps_filter_grip_fail_flick_check                                        */
/* ------------------------------------------------------------------------- */
void shtps_filter_grip_fail_flick_init(void)
{
	grip_fail_flick_inhibit_id = 0;
}

/* ------------------------------------------------------------------------- */
/* shtps_filter_grip_fail_flick_check                                        */
/* ------------------------------------------------------------------------- */
void shtps_filter_grip_fail_flick_check(u8 *finger_state_p)
{
	int i;
	int diff_x;
	int diff_y;

	if(SHTRPD_GRIP_FAIL_FLICK_REJECT_ENABLE == 0){
		return;
	}

	for(i = 0; i < MAX_TOUCHES; i++){
		if( (grip_fail_flick_inhibit_id & (1 << i)) != 0 ){
			if((*finger_state_p & (1 << i)) == 0 ||
				shtrpd_filter_edge_fail_touch_area_check(shtrpd_touches[i].XPos, shtrpd_touches[i].YPos) == 0)
			{
				grip_fail_flick_inhibit_id &= ~(1 << i);
			}
		}

		if( (grip_fail_flick_inhibit_id & (1 << i)) == 0 &&
			(*finger_state_p & (1 << i)) != 0 && (shtrpd_finger_state_old & (1 << i)) != 0)
		{
			if(shtrpd_filter_edge_fail_touch_area_check(shtrpd_touches[i].XPos, shtrpd_touches[i].YPos) != 0){
				diff_x = abs(shtrpd_touches[i].XPos - shtrpd_touches_old[i].XPos);
				diff_y = abs(shtrpd_touches[i].YPos - shtrpd_touches_old[i].YPos);

				if(((diff_x * diff_x) + (diff_y * diff_y)) > (SHTRPD_GRIP_FAIL_FLICK_REJECT_DIST * SHTRPD_GRIP_FAIL_FLICK_REJECT_DIST)){
					int pre_z = (shtrpd_touches_old[i].area == 0)?
						shtrpd_touches_old[i].touchStrength : (shtrpd_touches_old[i].touchStrength / shtrpd_touches_old[i].area);
					int cur_z = (shtrpd_touches[i].area == 0)?
						shtrpd_touches[i].touchStrength : (shtrpd_touches[i].touchStrength / shtrpd_touches[i].area);
					unsigned long areadiff = abs(shtrpd_touches[i].area - shtrpd_touches_old[i].area);
					unsigned long zdiff    = abs(pre_z - cur_z);
					
					if(areadiff >= SHTRPD_GRIP_FAIL_FLICK_REJECT_AREA_TH && zdiff >= SHTRPD_GRIP_FAIL_FLICK_REJECT_Z_TH){
						grip_fail_flick_inhibit_id |= (1 << i);
						SHTRPD_LOG_DBG_GRIP_FAIL_FLICK_PRINT("inhibit(id : %d)\n", i);
					}
				}
			}
		}
	}

	for(i = 0; i < MAX_TOUCHES; i++){
		if( (grip_fail_flick_inhibit_id & (1 << i)) != 0 ){
			*finger_state_p &= ~(1 << i);
			SHTRPD_LOG_DBG_GRIP_FAIL_FLICK_PRINT("[%d] is inhibited\n", i);
		}
	}
}
#endif /* SHTRPD_GRIP_FAIL_FLICK_REJECTION_ENABLE */

#endif /* SHTRPD_EDGE_FAIL_TOUCH_REJECTION_ENABLE */

#if defined(SHTRPD_EDGE_TD_MASK_ENABLE)
/* ------------------------------------------------------------------------- */
/* shtrpd_edge_td_mask_check                                                 */
/* ------------------------------------------------------------------------- */
void shtrpd_edge_td_mask_check(u8 *finger_state_p)
{
    int i;

    if(SHTRPD_EDGE_TD_MASK == 0){
        return;
    }
    
    for(i = 0;i < MAX_TOUCHES ;i++){
        if((*finger_state_p & (1<<i)) && (shtrpd_finger_state_old & (1<<i)) == 0){
			if(shtrpd_touches[i].XPos <= SHTRPD_EDGE_TD_MASK_AREA_X ||
			   shtrpd_touches[i].XPos >= (SHTRPD_MAX_X - 1 - SHTRPD_EDGE_TD_MASK_AREA_X) ||
			   shtrpd_touches[i].YPos <= SHTRPD_EDGE_TD_MASK_AREA_Y ||
			   shtrpd_touches[i].YPos >= (SHTRPD_MAX_Y - 1 - SHTRPD_EDGE_TD_MASK_AREA_Y))
			{
				*finger_state_p &= ~(1 << i);
				SHTRPD_DBG("[edge_td_mask] ignore td pos(%d, %d)\n", shtrpd_touches[i].XPos, shtrpd_touches[i].YPos);
			} 
        }
    }
}
#endif /* SHTRPD_EDGE_TD_MASK_ENABLE */

#if defined(SHTRPD_FAIL_FLICK_RECOVER_ENABLE)
/* ------------------------------------------------------------------------- */
/* shtrpd_fail_flick_recover_init                                            */
/* ------------------------------------------------------------------------- */
static void shtrpd_fail_flick_recover_init(void)
{
	memset(fail_flick_recover, 0, sizeof(fail_flick_recover));
}

/* ------------------------------------------------------------------------- */
/* shtrpd_fail_flick_recover_check                                           */
/* ------------------------------------------------------------------------- */
static void shtrpd_fail_flick_recover_check(u8 finger_state)
{
	int i, j;
    struct timeval tv;
    unsigned long time;
	
	if(!SHTRPD_FAIL_FLICK_RECOVER){
		return;
	}

    do_gettimeofday(&tv);
    time = tv.tv_sec * 1000 + tv.tv_usec / 1000;
	
	for(i = 0;i < MAX_TOUCHES;i++){
		if(td_history[i].state == 0){
			if(finger_state & (1<<i)){
				int idx;
				if(fail_flick_recover[i].hist_cnt < 3){
					idx = fail_flick_recover[i].hist_cnt;
					fail_flick_recover[i].hist_cnt++;
				}else{
					idx = 2;
					for(j = 0;j < 2;j++){
						fail_flick_recover[i].hist_x[j] = fail_flick_recover[i].hist_x[j+1];
						fail_flick_recover[i].hist_y[j] = fail_flick_recover[i].hist_y[j+1];
					}
				}
				fail_flick_recover[i].hist_time[idx] = time;
				fail_flick_recover[i].hist_x[idx] = shtrpd_touches[i].XPos;
				fail_flick_recover[i].hist_y[idx] = shtrpd_touches[i].YPos;
			}else{
				fail_flick_recover[i].hist_cnt = 0;
			}
		}
	}
}

/* ------------------------------------------------------------------------- */
/* shtrpd_fail_flick_recover_getva                                           */
/* ------------------------------------------------------------------------- */
static void shtrpd_fail_flick_recover_getva(int *hist, int *a, int *v, int interval, int update_acc)
{
	int v0;
	
	v0 = hist[1] - hist[0];
	*v = hist[2] - hist[1];
	
	if(abs(v0) <= SHTRPD_FAIL_FLICK_RECOVER_VMIN || abs(*v) <= SHTRPD_FAIL_FLICK_RECOVER_VMIN){
		*v = 0;
	}else{
		*v = (*v * SHTRPD_FAIL_FLICK_RECOVER_VRATIO) / 100;
	}
	
	if(update_acc){
		if(abs(v0) <= SHTRPD_FAIL_FLICK_RECOVER_VMIN || abs(*v) <= SHTRPD_FAIL_FLICK_RECOVER_VMIN){
			*a = 0;
		}else{
			*a  = (*v * 1000) / (v0 * interval);
		}
	}
}

/* ------------------------------------------------------------------------- */
/* shtrpd_fail_flick_recover_convpos                                         */
/* ------------------------------------------------------------------------- */
static int shtrpd_fail_flick_recover_convpos(int finger, u16 *x, u16 *y, int update)
{
	int ret = 0;
	int v, conv_x, conv_y;
	struct timeval tv;
	unsigned long cur_time;
	int cal_interval = 0, interval;
	
	if(!SHTRPD_FAIL_FLICK_RECOVER || fail_flick_recover[finger].hist_cnt < 3){
		return 0;
	}
	
    do_gettimeofday(&tv);
    cur_time = tv.tv_sec * 1000 + tv.tv_usec / 1000;
	interval = cur_time - fail_flick_recover[finger].pre_time;

	if(update){
		interval = cur_time - fail_flick_recover[finger].hist_time[2];
		cal_interval = fail_flick_recover[finger].hist_time[2] - fail_flick_recover[finger].hist_time[1];
	}
	
	/* x pos */
	shtrpd_fail_flick_recover_getva(fail_flick_recover[finger].hist_x, 
	                                &fail_flick_recover[finger].acc_x,
	                                &v, cal_interval, update);
	
	conv_x = *x + (v + (fail_flick_recover[finger].acc_x * interval)/1000);
	if(conv_x <= 0){
		conv_x = 0;
		if(*x > 0){
			ret = 1;
		}
	}else if(conv_x >= SHTRPD_MAX_X - 1){
		conv_x = SHTRPD_MAX_X - 1;
		if(*x < SHTRPD_MAX_X - 1){
			ret = 1;
		}
	}
	*x = conv_x;
	
	/* y pos */
	shtrpd_fail_flick_recover_getva(fail_flick_recover[finger].hist_y, 
	                                &fail_flick_recover[finger].acc_y,
	                                &v, cal_interval, update);

	conv_y = *y + (v + (fail_flick_recover[finger].acc_y * interval)/1000);
	if(conv_y <= 0){
		conv_y = 0;
		if(*y > 0){
			ret = 1;
		}
	}else if(conv_y >= SHTRPD_MAX_Y - 1){
		conv_y = SHTRPD_MAX_Y - 1;
		if(*y < SHTRPD_MAX_Y - 1){
			ret = 1;
		}
	}
	*y = conv_y;
	
	fail_flick_recover[finger].pre_time = cur_time;
	
	return ret;
}
#endif /* SHTRPD_FAIL_FLICK_RECOVER_ENABLE */

#if defined( SHTRPD_TOUCH_UP_DELAY_ENABLE )
/* ------------------------------------------------------------------------- */
/* shtrpd_touch_down_check_area                                              */
/* ------------------------------------------------------------------------- */
static int shtrpd_touch_down_check_area(u16 x, u16 y)
{
	if(SHTRPD_TOUCH_UP_DELAY_AREA_ENABLE == 0) {
		return 0;
	}

	if ((y >= 0) && (y <= SHTRPD_TOUCH_UP_DELAY_IGNORE_AREA_Y)) {
		return 1;
	} 
	
	if ((x >= SHTRPD_TOUCH_UP_DELAY_IGNORE_PWR_AREA_X) && (x <= SHTRPD_MAX_X)) {
		if((y > SHTRPD_TOUCH_UP_DELAY_IGNORE_PWR_AREA_Y1)
		&& (y <= SHTRPD_TOUCH_UP_DELAY_IGNORE_PWR_AREA_Y2)) {
			return 1;
		}
	}
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_tu_delay_func                                                      */
/* ------------------------------------------------------------------------- */
static void shtrpd_tu_delay_func(struct work_struct *work)
{
	int i;
	u8 finger_state = 0;
	u8 delay_end_count = 0;
	struct shtrpd_ts_data *data = i2c_get_clientdata(shtrpd_client);

	if(!SHTRPD_TOUCH_UP_DELAY){
		return;
	}

	mutex_lock(&shtrpd_tu_delay_lock);

	SHTRPD_DBG("start\n");

	for (i=0; i<MAX_TOUCHES; i++) {
		SHTRPD_DBG("delay_count[%d]:%d\n", i, shtrpd_delay_count[i]);
		if(td_history[i].state == 1) {
			shtrpd_delay_count[i]++;
			
			#if defined(SHTRPD_FAIL_FLICK_RECOVER_ENABLE)
			if(shtrpd_fail_flick_recover_convpos(i, &td_history[i].x_bk, &td_history[i].y_bk, 0) != 0){
				shtrpd_delay_count[i] = SHTRPD_TOUCH_UP_DELAY_IGNORE_COUNT;
			}else{
				shtrpd_add_drag_hist(SHTRPD_POSTYPE_X, i, td_history[i].x_bk);
				shtrpd_add_drag_hist(SHTRPD_POSTYPE_Y, i, td_history[i].y_bk);
			    shtrpd_filter_pos_compensation(SHTRPD_POSTYPE_X ,(1<<i));
			    shtrpd_filter_pos_compensation(SHTRPD_POSTYPE_Y ,(1<<i));

                if(shtrpd_touchcruiser_state){
  				    input_mt_slot(data->input_dev, i);
				    input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, i+1);
				    input_report_abs(data->input_dev, ABS_MT_POSITION_X, SHTRPD_NOTIFY_POS_SCALE_X(shtrpd_touches[i].XPos));
				    input_report_abs(data->input_dev, ABS_MT_POSITION_Y, SHTRPD_NOTIFY_POS_SCALE_Y(shtrpd_touches[i].YPos));
			    	input_report_abs(data->input_dev, ABS_MT_PRESSURE, td_history[i].z_bk);
				    input_sync(data->input_dev);
			    
				    SHTRPD_INFO("ABS_MT_TRACKING_ID ID=%d ; STATE=1 ; X=%d ; Y=%d ; Z=%d (fail flick recover)\n" ,
				                 i, td_history[i].x_bk, td_history[i].y_bk, td_history[i].z_bk);
				}
			}
			#endif /* SHTRPD_FAIL_FLICK_RECOVER_ENABLE */
		} else {
			shtrpd_delay_count[i] = 0;
			finger_state |= (1<<i);
		}
		
		if( shtrpd_delay_count[i] == SHTRPD_TOUCH_UP_DELAY_IGNORE_COUNT) {
			td_history[i].count_x = 0;
			td_history[i].count_y = 0;
			td_history[i].state = 0;
			shtrpd_delay_count[i] = 0;
			drag_hist[i][SHTRPD_POSTYPE_X].count = 0;
			drag_hist[i][SHTRPD_POSTYPE_X].count_up_base = 0;
			drag_hist[i][SHTRPD_POSTYPE_Y].count = 0;
			drag_hist[i][SHTRPD_POSTYPE_Y].count_up_base = 0;
			SHTRPD_DBG("input_report -1: ID=%d\n", i);
			input_mt_slot(data->input_dev, i);
			input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, -1);
			input_sync(data->input_dev);
			shtrpd_finger_state_old &= ~(1<<i);
			finger_state |= (1<<i);
		}
	}
	
	for (i=0; i<MAX_TOUCHES; i++){
		if(finger_state & (1<<i)) {
			delay_end_count++;
		}
	}
	
	if (delay_end_count != MAX_TOUCHES) {
		queue_delayed_work(shtrpd_work_queue, &shtrpd_work_tu_delay, msecs_to_jiffies(10));
	}else {
		tu_queue_work_flg = 0;
	}

	mutex_unlock(&shtrpd_tu_delay_lock);
}

/* ------------------------------------------------------------------------- */
/* shtrpd_touch_up_check_func                                                */
/* ------------------------------------------------------------------------- */
static void shtrpd_touch_up_check_func(u8 *state, u16 *x, u16 *y, u16 *z, u8 *a)
{
	int i;
	u8 finger_state[MAX_TOUCHES];
	
	if(!SHTRPD_TOUCH_UP_DELAY){
		return;
	}
	
	for (i = 0; i < MAX_TOUCHES; i++) {
		finger_state[i] = SHTRPD_FINGER_STATE_NONE;
		
		if (!(*state & (1<<i))) {
			if (!(shtrpd_finger_state_old  & (1<<i))) {
				finger_state[i] = SHTRPD_FINGER_STATE_NONE;
			} else if ((shtrpd_finger_state_old  & (1<<i))) {
				finger_state[i] = SHTRPD_FINGER_STATE_TU;
			}
		} else if ((*state & (1<<i))) {
			if (!(shtrpd_finger_state_old  & (1<<i))) {
				finger_state[i] = SHTRPD_FINGER_STATE_TD;
			} else if ((shtrpd_finger_state_old  & (1<<i))) {
				finger_state[i] = SHTRPD_FINGER_STATE_DRAG;
			}
		}
		SHTRPD_DBG("finger_state[%d]:%d\n", i, finger_state[i]);
	}
	
	for (i = 0; i < MAX_TOUCHES; i++) {
		if(finger_state[i] == SHTRPD_FINGER_STATE_TD) {
			SHTRPD_DBG("finger_state[%d]:SHTRPD_FINGER_STATE_TD\n", i);
			td_history[i].count_x = 0;
			td_history[i].count_y = 0;
			td_history[i].x_bk = x[i];
			td_history[i].y_bk = y[i];
			td_history[i].z_bk = z[i];
			td_history[i].a_bk = a[i];
		} else if(finger_state[i] == SHTRPD_FINGER_STATE_DRAG) {
			SHTRPD_DBG("finger_state[%d]:diff x:%ld, y:%ld\n", i , abs(td_history[i].x_bk - x[i]), abs(td_history[i].y_bk - y[i]));
			if (td_history[i].state) {
				td_history[i].state = 0;
			}
			if(abs(td_history[i].x_bk - x[i]) > SHTRPD_TOUCH_UP_DELAY_HISTORY_DIST) {
				if(td_history[i].count_x < SHTRPD_TOUCH_UP_DELAY_HISTORY_COUNT){
					td_history[i].count_x++;
					SHTRPD_DBG("DRAG:td_history[%d].count_x:%d\n", i, td_history[i].count_x);
				}
			}
			if(abs(td_history[i].y_bk - y[i]) > SHTRPD_TOUCH_UP_DELAY_HISTORY_DIST) {
				if(td_history[i].count_y < SHTRPD_TOUCH_UP_DELAY_HISTORY_COUNT){
					td_history[i].count_y++;
					SHTRPD_DBG("DRAG:td_history[%d].count_y:%d\n", i, td_history[i].count_y);
				}
			}
#if 0
			if ((abs(td_history[i].x_bk - x[i]) == 0) && (abs(td_history[i].y_bk - y[i]) == 0)) {
					td_history[i].count_x = 0;
					td_history[i].count_y = 0;
					SHTRPD_DBG("DRAG:td_history[%d].STOP\n", i);
			}
#endif
			td_history[i].x_bk = x[i];
			td_history[i].y_bk = y[i];
			td_history[i].z_bk = z[i];
			td_history[i].a_bk = a[i];
		} else if(finger_state[i] == SHTRPD_FINGER_STATE_TU) {
			if(shtrpd_touch_down_check_area(td_history[i].x_bk, td_history[i].y_bk)) {
				if (td_history[i].count_x >= SHTRPD_TOUCH_UP_DELAY_AREA_HISTORY_COUNT) {
				    td_history[i].count_x = SHTRPD_TOUCH_UP_DELAY_HISTORY_COUNT;
				}
				if (td_history[i].count_y >= SHTRPD_TOUCH_UP_DELAY_AREA_HISTORY_COUNT) {
					td_history[i].count_y = SHTRPD_TOUCH_UP_DELAY_HISTORY_COUNT;
				}
			}
			SHTRPD_DBG("TOUCHUP:td_history[%d].count_x:%d\n", i, td_history[i].count_x);
			SHTRPD_DBG("TOUCHUP:td_history[%d].count_y:%d\n", i, td_history[i].count_y);
			if((td_history[i].count_x == SHTRPD_TOUCH_UP_DELAY_HISTORY_COUNT)
			|| (td_history[i].count_y == SHTRPD_TOUCH_UP_DELAY_HISTORY_COUNT)) {
				
				#if defined(SHTRPD_FAIL_FLICK_RECOVER_ENABLE)
				if(td_history[i].state || shtrpd_fail_flick_recover_convpos(i, &td_history[i].x_bk, &td_history[i].y_bk, 1) == 0){
				#endif /* SHTRPD_FAIL_FLICK_RECOVER_ENABLE */
				
				x[i] = td_history[i].x_bk;
				y[i] = td_history[i].y_bk;
				z[i] = td_history[i].z_bk;
				a[i] = td_history[i].a_bk;
				shtrpd_touches[i].XPos          = td_history[i].x_bk;
				shtrpd_touches[i].YPos          = td_history[i].y_bk;
				shtrpd_touches[i].touchStrength = td_history[i].z_bk;
				shtrpd_touches[i].area          = td_history[i].a_bk;
				*state |= (1<<i);
				td_history[i].state = 1;
				if (!tu_queue_work_flg) {
					SHTRPD_DBG("delayed_work\n");
					queue_delayed_work(shtrpd_work_queue, &shtrpd_work_tu_delay, msecs_to_jiffies(10));
					tu_queue_work_flg = 1;
				}

				#if defined(SHTRPD_FAIL_FLICK_RECOVER_ENABLE)
				}
				#endif /* SHTRPD_FAIL_FLICK_RECOVER_ENABLE */
			} else {
				SHTRPD_DBG("SHTRPD_FINGER_STATE_TU\n");
				td_history[i].count_x = 0;
				td_history[i].count_y = 0;
			}
		} else {
			SHTRPD_DBG("SHTRPD_FINGER_STATE_NONE\n");
			td_history[i].count_x = 0;
			td_history[i].count_y = 0;
		}
	}
}
#endif /* SHTRPD_TOUCH_UP_DELAY_ENABLE */

#if defined( SHTRPD_TOUCH_DOWN_THRESHOLD_ENABLE )
/* ------------------------------------------------------------------------- */
/* shtrpd_change_touch_down_threshold                                        */
/* ------------------------------------------------------------------------- */
static void shtrpd_change_touch_down_threshold(struct shtrpd_ts_data *data, u8 state, u16 *x, u16 *y)
{
	int i;
    int ret = 0;
	u8 finger_state[MAX_TOUCHES];
	u8 data_buffer[10];
	
	if(!SHTRPD_TOUCH_DOWN_THRESHOLD){
		return;
	}

	if(state) {
		if(!shtrpd_touch_down_threshold_flg) {
			for (i = 0; i < MAX_TOUCHES; i++) {
				finger_state[i] = SHTRPD_FINGER_STATE_NONE;
				
				if (!(state & (1<<i))) {
					if (!(shtrpd_finger_state_old  & (1<<i))) {
						finger_state[i] = SHTRPD_FINGER_STATE_NONE;
					} else if ((shtrpd_finger_state_old  & (1<<i))) {
						finger_state[i] = SHTRPD_FINGER_STATE_TU;
					}
				} else if ((state & (1<<i))) {
					if (!(shtrpd_finger_state_old  & (1<<i))) {
						finger_state[i] = SHTRPD_FINGER_STATE_TD;
					} else if ((shtrpd_finger_state_old  & (1<<i))) {
						finger_state[i] = SHTRPD_FINGER_STATE_DRAG;
					}
				}
				
				SHTRPD_DBG("finger_state[%d]:%d, th_history[%d].count:%d\n", i, finger_state[i], i, th_history[i].count);
				
				if(finger_state[i] == SHTRPD_FINGER_STATE_TD) {
					th_history[i].x = x[i];
					th_history[i].y = y[i];
				} else if(finger_state[i] == SHTRPD_FINGER_STATE_DRAG) {
					if(th_history[i].count < SHTRPD_TOUCH_DOWN_THRESHOLD_COUNT){
						th_history[i].count++;
					}
				} else {
					th_history[i].count = 0;
				}
				
				if( th_history[i].count == SHTRPD_TOUCH_DOWN_THRESHOLD_COUNT ) {
					SHTRPD_DBG("diff x:%ld, diff y:%ld\n", abs(th_history[i].x - x[i]), abs(th_history[i].y - y[i]));
					if((abs(th_history[i].x - x[i]) > SHTRPD_TOUCH_DOWN_THRESHOLD_DIST) 
					|| (abs(th_history[i].y - y[i]) > SHTRPD_TOUCH_DOWN_THRESHOLD_DIST)) {
						if(!shtrpd_touch_down_threshold_flg) {
							SHTRPD_DBG("THRESHOLD_SETTINGS DOWN WRITE\n");
							/*  Prox Threshold  */
							data_buffer[0] = shtrpd_properties.proxthreshold_val;
							/*  Touch Multiplier    */
							data_buffer[1] = shtrpd_touchmultiplier_val - SHTRPD_TOUCH_DOWN_THRESHOLD_ADJUST;
							/*  Touch Shifter   */
							data_buffer[2] = shtrpd_touchshifter_val;
							/*  PM Prox Threshold   */
							data_buffer[3] = shtrpd_properties.pmproxthreshold_val;
							/*  Snap threshold  */
							data_buffer[4] = (unsigned char)(shtrpd_snapthreshold_val>>8);
							/*  Snap threshold  */
							data_buffer[5] = (unsigned char)shtrpd_snapthreshold_val;
							/*  Non-trackpad channels prox threshold    */
							data_buffer[6] = shtrpd_properties.proxthreshold2_val;
							/*  Non-trackpad channels Touch Multiplier  */
							data_buffer[7] = shtrpd_touchmultiplier2_val;
							/*  Non-trackpad channels Touch Shifter */
							data_buffer[8] = shtrpd_touchshifter2_val;
							/*  (in-touch) Touch Threshold Multiplier */
							data_buffer[9] = shtrpd_intouchmultiplier_val;
							ret = shtrpd_i2c_write_block_data(data->client, THRESHOLD_SETTINGS,
																			10, data_buffer);
							if(ret >= 0){
								SHTRPD_DBG("shtrpd_touch_down_threshold_flg = 1\n");
								shtrpd_touch_down_threshold_flg = 1;
							}
						}
					}
				}
			}
		}
	} else {
		if (shtrpd_touch_down_threshold_flg) {
			SHTRPD_DBG("THRESHOLD_SETTINGS UP WRITE\n");
			/*  Prox Threshold  */
			data_buffer[0] = shtrpd_properties.proxthreshold_val;
			/*  Touch Multiplier    */
			data_buffer[1] = shtrpd_touchmultiplier_val;
			/*  Touch Shifter   */
			data_buffer[2] = shtrpd_touchshifter_val;
			/*  PM Prox Threshold   */
			data_buffer[3] = shtrpd_properties.pmproxthreshold_val;
			/*  Snap threshold  */
			data_buffer[4] = (unsigned char)(shtrpd_snapthreshold_val>>8);
			/*  Snap threshold  */
			data_buffer[5] = (unsigned char)shtrpd_snapthreshold_val;
			/*  Non-trackpad channels prox threshold    */
			data_buffer[6] = shtrpd_properties.proxthreshold2_val;
			/*  Non-trackpad channels Touch Multiplier  */
			data_buffer[7] = shtrpd_touchmultiplier2_val;
			/*  Non-trackpad channels Touch Shifter */
			data_buffer[8] = shtrpd_touchshifter2_val;
			/*  (in-touch) Touch Threshold Multiplier */
			data_buffer[9] = shtrpd_intouchmultiplier_val;
			ret = shtrpd_i2c_write_block_data(data->client, THRESHOLD_SETTINGS,
																10, data_buffer);
			if(ret >= 0){
				SHTRPD_DBG("shtrpd_touch_down_threshold_flg = 0\n");
				shtrpd_touch_down_threshold_flg = 0;
				memset(th_history, 0 ,sizeof(th_history));
			}
		}
	}
}
#endif /* SHTRPD_TOUCH_DOWN_THRESHOLD_ENABLE */

#if defined(SHTRPD_TOP_OF_SCREEN_GHOST_REJECTION_ENABLE)
/* ------------------------------------------------------------------------- */
/* shtrpd_filter_topofscreen_ghost_reject_ghost_check                        */
/* ------------------------------------------------------------------------- */
static void shtrpd_filter_topofscreen_ghost_reject_init(void)
{
	memset(topofscreen_ghost_reject, 0, sizeof(topofscreen_ghost_reject));
}

/* ------------------------------------------------------------------------- */
/* shtrpd_filter_topofscreen_ghost_reject_ghost_check                        */
/* ------------------------------------------------------------------------- */
static int shtrpd_filter_topofscreen_ghost_reject_ghost_check(u8 id, u8 *finger_state_p)
{
	int ret = TOP_OF_SCREEN_GHOST_NOT_DETECTED;
	int i;
	unsigned long diff;
	
	if(topofscreen_ghost_reject[id].is_ghost != 0){
		return TOP_OF_SCREEN_GHOST_DETECTED;
	}
	
	if((shtrpd_finger_state_old & (1 << id)) == 0 && (*finger_state_p & (1 << id)) != 0){
		for(i = 0;i < MAX_TOUCHES;i++){
			if(i == id || (*finger_state_p & (1 << i)) == 0){
				continue;
			}
			
			diff = abs(shtrpd_touches[i].XPos - shtrpd_touches[id].XPos);

			SHTRPD_LOG_DBG_TOP_OF_SCREEN_GHOST_PRINT("[%d](%d, %d) [%d](%d, %d) / x diff = %lu\n", 
				id, shtrpd_touches[id].XPos, shtrpd_touches[id].YPos, i, shtrpd_touches[i].XPos, shtrpd_touches[i].YPos, diff);
			
			if(diff <= SHTRPD_TOP_OF_SCREEN_GHOST_REJECT_XDIFF_THRESH &&
			   shtrpd_touches[i].YPos > shtrpd_touches[id].YPos /* &&
			   shtrpd_touches[i].touchStrength > shtrpd_touches[id].touchStrength  &&
			   ((shtrpd_touches[id].touchStrength * 100) / shtrpd_touches[i].touchStrength) < SHTRPD_TOP_OF_SCREEN_GHOST_REJECT_Z_RATIO_THRESH */)
			{
				ret = TOP_OF_SCREEN_GHOST_DETECTED;
				SHTRPD_LOG_DBG_TOP_OF_SCREEN_GHOST_PRINT("[%d] detect ghost\n", id);
				break;
			}
		}
	}
	
	return ret;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_filter_topofscreen_ghost_reject_ghost_clear_check                  */
/* ------------------------------------------------------------------------- */
static int shtrpd_filter_topofscreen_ghost_reject_ghost_clear_check(u8 id, u8 *finger_state_p)
{
	int i;
	unsigned long diff;

	if(topofscreen_ghost_reject[id].is_ghost == 0){
		return TOP_OF_SCREEN_GHOST_CLEARED;
	}
	
	if((*finger_state_p & (1 << id)) == 0){
		SHTRPD_LOG_DBG_TOP_OF_SCREEN_GHOST_PRINT("[%d] clear ghost by touch-state\n", id);
		return TOP_OF_SCREEN_GHOST_CLEARED;
	}
	
	if(topofscreen_ghost_reject[id].reject_count >= SHTRPD_TOP_OF_SCREEN_GHOST_REJECT_CLEAR_COUNT){
		SHTRPD_LOG_DBG_TOP_OF_SCREEN_GHOST_PRINT("[%d] clear ghost by count\n", id);
		return TOP_OF_SCREEN_GHOST_CLEARED;
	}
	
	for(i = 0;i < MAX_TOUCHES;i++){
		if(i == id || (*finger_state_p & (1 << i)) == 0){
			continue;
		}
		
		diff = abs(shtrpd_touches[i].XPos - shtrpd_touches[id].XPos);
		
		if(diff <= SHTRPD_TOP_OF_SCREEN_GHOST_REJECT_XDIFF_THRESH){
			return TOP_OF_SCREEN_GHOST_NOT_CLEARED;
		}
	}
	
	SHTRPD_LOG_DBG_TOP_OF_SCREEN_GHOST_PRINT("[%d] clear ghost by other finger\n", id);
	return TOP_OF_SCREEN_GHOST_CLEARED;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_filter_topofscreen_ghost_reject_check                              */
/* ------------------------------------------------------------------------- */
static void shtrpd_filter_topofscreen_ghost_reject_check(u8 *finger_state_p)
{
	int i;

	if(!SHTRPD_TOP_OF_SCREEN_GHOST_REJECT_ENABLE){
		return;
	}
	
	for(i = 0;i < MAX_TOUCHES;i++){
		if(topofscreen_ghost_reject[i].is_ghost == 0){
			if(shtrpd_filter_topofscreen_ghost_reject_ghost_check(i, finger_state_p) == TOP_OF_SCREEN_GHOST_DETECTED){
				topofscreen_ghost_reject[i].is_ghost = 1;
				topofscreen_ghost_reject[i].reject_count = 0;
			}
		}else{
			if(shtrpd_filter_topofscreen_ghost_reject_ghost_clear_check(i, finger_state_p) == TOP_OF_SCREEN_GHOST_CLEARED){
				topofscreen_ghost_reject[i].is_ghost = 0;
			}
		}
	}

	for(i = 0;i < MAX_TOUCHES;i++){
		if(topofscreen_ghost_reject[i].is_ghost != 0){
			*finger_state_p &= ~(1 << i);
			topofscreen_ghost_reject[i].reject_count++;
			SHTRPD_LOG_DBG_TOP_OF_SCREEN_GHOST_PRINT("[%d] reject ghost\n", i);
		}
	}
}
#endif /* SHTRPD_TOP_OF_SCREEN_GHOST_REJECTION_ENABLE */

#if defined(SHTRPD_TU_JITTER_FILTER_ENABLE)
/* ------------------------------------------------------------------------- */
/* shtrpd_tu_jitter_filter_init                                              */
/* ------------------------------------------------------------------------- */
static void shtrpd_tu_jitter_filter_init(void)
{
	memset(tu_jitter_filter, 0, sizeof(tu_jitter_filter));
}

/* ------------------------------------------------------------------------- */
/* shtrpd_tu_jitter_filter_check_area                                        */
/* ------------------------------------------------------------------------- */
static int shtrpd_tu_jitter_filter_check_area(u8 finger)
{
	unsigned long pre;
	unsigned long diff;
	
	if(tu_jitter_filter[finger].pre_touch.area <= shtrpd_touches[finger].area){
		return 0;
	}
	
	diff = abs(tu_jitter_filter[finger].pre_touch.area - shtrpd_touches[finger].area);
	pre = tu_jitter_filter[finger].pre_touch.area;
	pre = (pre == 0)? 1 : pre;
	
	if(((diff * 100) / pre) >= SHTRPD_TU_JITTER_FILTER_AREA_DECAY_TH){
		SHTRPD_LOG_DBG_TU_JITTER_FILTER_PRINT("[%d] pos fix by area. diff=%lu / pre=%lu / ratio=%lu\n",
			finger, diff, pre, ((diff * 100) / pre));
		return 1;
	}
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_tu_jitter_filter_check_return                                      */
/* ------------------------------------------------------------------------- */
static int shtrpd_tu_jitter_filter_check_return(u8 finger)
{
	if(shtrpd_touches[finger].area > SHTRPD_TU_JITTER_FILTER_AREA_MAX_TH){
		SHTRPD_LOG_DBG_TU_JITTER_FILTER_PRINT("[%d] disable by area\n", finger);
		return 1;
	}
	
	if(abs(tu_jitter_filter[finger].pre_touch.XPos - shtrpd_touches[finger].XPos) >= SHTRPD_TU_JITTER_FILTER_MOVE_RET_TH ||
	   abs(tu_jitter_filter[finger].pre_touch.YPos - shtrpd_touches[finger].YPos) >= SHTRPD_TU_JITTER_FILTER_MOVE_RET_TH)
	{
		SHTRPD_LOG_DBG_TU_JITTER_FILTER_PRINT("[%d] disable by move\n", finger);
		return 1;
	}
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_pin_jitter_filter_dist_check                                       */
/* ------------------------------------------------------------------------- */
static void shtrpd_pin_jitter_filter_dist_check(u8 finger, u8 finger_state)
{
	int i;
	u8 dist_cond = 0;
	u8 enable_jitter = 0;
	u8 area_fingers = 0;

	if(!SHTRPD_PINCHIN_JITTER_FILTER){
		return;
	}

	if((finger_state & (1 << finger)) == 0){
		return;
	}
	
	if(tu_jitter_filter[finger].is_fixed == SHTRPD_TU_JITTER_FILTER_FIXED_BY_PINCHIIN){
		return;
	}
	
	if((shtrpd_finger_state_old & (1 << finger)) == 0 && (finger_state & (1 << finger)) != 0){
		tu_jitter_filter[finger].dist_cond = 0;
		for(i = 0;i < MAX_TOUCHES;i++){
			tu_jitter_filter[finger].dist_x[i] = 0xFFFF;
			tu_jitter_filter[finger].dist_y[i] = 0xFFFF;
		}
		return;
	}
	
	for(i = 0;i < MAX_TOUCHES;i++){
		if(i == finger){
			tu_jitter_filter[finger].dist_x[i] = 0xFFFF;
			tu_jitter_filter[finger].dist_y[i] = 0xFFFF;

		}else{
			if((finger_state & (1 << i)) != 0){
				unsigned long dist_x = abs(shtrpd_touches[i].XPos - shtrpd_touches[finger].XPos);
				unsigned long dist_y = abs(shtrpd_touches[i].YPos - shtrpd_touches[finger].YPos);
				
				SHTRPD_LOG_DBG_TU_JITTER_FILTER_PRINT("[%d] (%d) pre dist(%lu, %lu), cur dist(%lu, %lu)\n", 
					finger, i, tu_jitter_filter[finger].dist_x[i], tu_jitter_filter[finger].dist_y[i], dist_x, dist_y);

				if(tu_jitter_filter[finger].dist_x[i] != 0xFFFF && 
				   tu_jitter_filter[finger].dist_y[i] != 0xFFFF)
				{
					unsigned long pre_dist_x = tu_jitter_filter[finger].dist_x[i];
					unsigned long pre_dist_y = tu_jitter_filter[finger].dist_y[i];
					int in_hyst, out_hyst;
					
					if(tu_jitter_filter[finger].dist_cond){
						in_hyst = SHTRPD_PINCHIN_JITTER_FILTER_DIST_HYSTERESIS;
						out_hyst= -SHTRPD_PINCHIN_JITTER_FILTER_DIST_HYSTERESIS;
					}else{
						in_hyst = -SHTRPD_PINCHIN_JITTER_FILTER_DIST_HYSTERESIS;
						out_hyst= SHTRPD_PINCHIN_JITTER_FILTER_DIST_HYSTERESIS;
					}
					
					if(!((dist_x > dist_y && pre_dist_x < (dist_x + out_hyst)) || (dist_y > dist_x && pre_dist_y < (dist_y + out_hyst)))){
					
						if((pre_dist_x + in_hyst) > dist_x || (pre_dist_y + in_hyst) > dist_y){
							dist_cond = 1;
							SHTRPD_LOG_DBG_TU_JITTER_FILTER_PRINT("[%d] dist condition = true with %d\n", finger, i);
						}
					}
				}
				
				if(dist_x <= SHTRPD_PINCHIN_JITTER_FILTER_START_DIST &&
				   dist_y <= SHTRPD_PINCHIN_JITTER_FILTER_START_DIST)
				{
					area_fingers++;
				}

				tu_jitter_filter[finger].dist_x[i] = dist_x;
				tu_jitter_filter[finger].dist_y[i] = dist_y;
				
			}else{
				if(tu_jitter_filter[finger].dist_x[i] != 0xFFFF && tu_jitter_filter[finger].dist_y[i] != 0xFFFF){
					if(tu_jitter_filter[finger].dist_cond){
						if(tu_jitter_filter[finger].dist_x[i] <= SHTRPD_PINCHIN_JITTER_FILTER_START_DIST &&
						   tu_jitter_filter[finger].dist_y[i] <= SHTRPD_PINCHIN_JITTER_FILTER_START_DIST)
						{
							enable_jitter = 1;
						}
					}
					SHTRPD_LOG_DBG_TU_JITTER_FILTER_PRINT("[%d][tu finger=%d] check jitter condition. dist condition = %d, dist x = %lu, dist y = %lu\n", 
						finger, i, tu_jitter_filter[finger].dist_cond, tu_jitter_filter[finger].dist_x[i], tu_jitter_filter[finger].dist_y[i]);
				}
				
				tu_jitter_filter[finger].dist_x[i] = 0xFFFF;
				tu_jitter_filter[finger].dist_y[i] = 0xFFFF;
			}
		}
	}
	
	tu_jitter_filter[finger].dist_cond = dist_cond;
	
	if(enable_jitter && !area_fingers){
		tu_jitter_filter[finger].is_fixed = SHTRPD_TU_JITTER_FILTER_FIXED_BY_PINCHIIN;
		memcpy(&tu_jitter_filter[finger].pre_touch, &shtrpd_touches[finger], sizeof(tu_jitter_filter[finger].pre_touch));
		SHTRPD_LOG_DBG_TU_JITTER_FILTER_PRINT("[%d] enable jitter by pinch-in\n", finger);
	}
}

/* ------------------------------------------------------------------------- */
/* shtrpd_pin_jitter_filter_dist_rel_check                                   */
/* ------------------------------------------------------------------------- */
static int shtrpd_pin_jitter_filter_dist_rel_check(u8 finger, u8 finger_state)
{
	int i;
	unsigned long dist_x;
	unsigned long dist_y;
	
	if((finger_state & (1 << finger)) == 0 ||
			tu_jitter_filter[finger].is_fixed != SHTRPD_TU_JITTER_FILTER_FIXED_BY_PINCHIIN)
	{
		return 1;
	}
	
	for(i = 0;i < MAX_TOUCHES;i++){
		if(i != finger){
			if((finger_state & (1 << i)) != 0){
				dist_x = abs(shtrpd_touches[i].XPos - shtrpd_touches[finger].XPos);
				dist_y = abs(shtrpd_touches[i].YPos - shtrpd_touches[finger].YPos);
				
				if(dist_x <= SHTRPD_PINCHIN_JITTER_FILTER_EXIT_DIST && 
				   dist_y <= SHTRPD_PINCHIN_JITTER_FILTER_EXIT_DIST)
				{
					SHTRPD_LOG_DBG_TU_JITTER_FILTER_PRINT("[%d] disable by multi\n", finger);
					return 1;
				}
			}
		}
	}

	dist_x = abs(tu_jitter_filter[finger].pre_touch.XPos - shtrpd_touches[finger].XPos);
	dist_y = abs(tu_jitter_filter[finger].pre_touch.YPos - shtrpd_touches[finger].YPos);

	if(dist_x >= SHTRPD_PINCHIN_JITTER_FILTER_EXIT_MOVE || 
	   dist_y >= SHTRPD_PINCHIN_JITTER_FILTER_EXIT_MOVE)
	{
		SHTRPD_LOG_DBG_TU_JITTER_FILTER_PRINT("[%d] disable by move\n", finger);
		return 1;
	}
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_tu_jitter_filter_check                                             */
/* ------------------------------------------------------------------------- */
static void shtrpd_tu_jitter_filter_check(u8 finger_state, u16 *x, u16 *y)
{
	int i;
	
	if(!SHTRPD_TU_JITTER_FILTER){
		return;
	}
	
	for(i = 0;i < MAX_TOUCHES;i++){
		shtrpd_pin_jitter_filter_dist_check(i, finger_state);
	}
	
	for(i = 0;i < MAX_TOUCHES;i++){
		if((shtrpd_finger_state_old & finger_state) & (1 << i)){
			if(tu_jitter_filter[i].is_fixed == SHTRPD_TU_JITTER_FILTER_NO_FIXED){
				if(shtrpd_touches[i].area <= SHTRPD_TU_JITTER_FILTER_AREA_MAX_TH &&
				   tu_jitter_filter[i].pre_touch.area >= SHTRPD_TU_JITTER_FILTER_AREA_PREMIN_TH)
				{
					if(shtrpd_tu_jitter_filter_check_area(i) != 0){
						tu_jitter_filter[i].is_fixed = SHTRPD_TU_JITTER_FILTER_FIXED_BY_AREA;
						tu_jitter_filter[i].pre_fixed_time = jiffies;
						tu_jitter_filter[i].pre_fixed_ts = tu_jitter_filter[i].pre_touch.touchStrength;
						tu_jitter_filter[i].pre_fixed_area = tu_jitter_filter[i].pre_touch.area;
					}
				}
			}
			
			if(tu_jitter_filter[i].is_fixed != SHTRPD_TU_JITTER_FILTER_NO_FIXED){
				if(tu_jitter_filter[i].is_fixed == SHTRPD_TU_JITTER_FILTER_FIXED_BY_AREA){
					if(time_after(jiffies, tu_jitter_filter[i].pre_fixed_time + 
												msecs_to_jiffies(SHTRPD_TU_JITTER_FILTER_TIMEOUT)) == 0)
					{
						if(shtrpd_tu_jitter_filter_check_return(i) == 0){
							SHTRPD_LOG_DBG_TU_JITTER_FILTER_PRINT("[%d] pos fix. (%d, %d) -> (%d, %d)\n",
								i, 
								shtrpd_touches[i].XPos, shtrpd_touches[i].YPos,
								tu_jitter_filter[i].pre_touch.XPos, tu_jitter_filter[i].pre_touch.YPos);
							
							x[i] = tu_jitter_filter[i].pre_touch.XPos;
							y[i] = tu_jitter_filter[i].pre_touch.YPos;
							shtrpd_touches[i].XPos = tu_jitter_filter[i].pre_touch.XPos;
							shtrpd_touches[i].YPos = tu_jitter_filter[i].pre_touch.YPos;
						}else{
							tu_jitter_filter[i].is_fixed = SHTRPD_TU_JITTER_FILTER_NO_FIXED;
						}
					}else{
						SHTRPD_LOG_DBG_TU_JITTER_FILTER_PRINT("[%d] disable by timeout\n", i);
						tu_jitter_filter[i].is_fixed = SHTRPD_TU_JITTER_FILTER_NO_FIXED;
					}
				}else if(tu_jitter_filter[i].is_fixed == SHTRPD_TU_JITTER_FILTER_FIXED_BY_PINCHIIN){
					if(shtrpd_pin_jitter_filter_dist_rel_check(i, finger_state) == 0){
						SHTRPD_LOG_DBG_TU_JITTER_FILTER_PRINT("[%d] pos fix. (%d, %d) -> (%d, %d)\n",
							i, 
							shtrpd_touches[i].XPos, shtrpd_touches[i].YPos,
							tu_jitter_filter[i].pre_touch.XPos, tu_jitter_filter[i].pre_touch.YPos);
						
						x[i] = tu_jitter_filter[i].pre_touch.XPos;
						y[i] = tu_jitter_filter[i].pre_touch.YPos;
						shtrpd_touches[i].XPos = tu_jitter_filter[i].pre_touch.XPos;
						shtrpd_touches[i].YPos = tu_jitter_filter[i].pre_touch.YPos;
					}else{
						tu_jitter_filter[i].is_fixed = SHTRPD_TU_JITTER_FILTER_NO_FIXED;
					}
				}
			}
		}else{
			if(tu_jitter_filter[i].is_fixed != SHTRPD_TU_JITTER_FILTER_NO_FIXED){
				SHTRPD_LOG_DBG_TU_JITTER_FILTER_PRINT("[%d] disable by touchup\n", i);
			}
			tu_jitter_filter[i].is_fixed = SHTRPD_TU_JITTER_FILTER_NO_FIXED;
		}
		
		if((finger_state & (1 << i)) != 0){
			memcpy(&tu_jitter_filter[i].pre_touch, &shtrpd_touches[i], sizeof(tu_jitter_filter[i].pre_touch));
		}
	}
}
#endif /* SHTRPD_TU_JITTER_FILTER_ENABLE */

#if defined(SHTRPD_CPU_IDLE_SLEEP_CONTROL_ENABLE)
/* ------------------------------------------------------------------------- */
/* shtrpd_wake_lock_idle                                                     */
/* ------------------------------------------------------------------------- */
static void shtrpd_wake_lock_init(void)
{
	idle_sleep_ctrl_info.wake_lock_idle_state = 0;
	wake_lock_init(&idle_sleep_ctrl_info.wake_lock, WAKE_LOCK_SUSPEND, "shtrpd_int_wake_lock");
	pm_qos_add_request(&idle_sleep_ctrl_info.qos_cpu_latency, PM_QOS_CPU_DMA_LATENCY, PM_QOS_DEFAULT_VALUE);
}

/* ------------------------------------------------------------------------- */
/* shtrpd_wake_lock_deidle                                                   */
/* ------------------------------------------------------------------------- */
static void shtrpd_wake_lock_deinit(void)
{
	pm_qos_remove_request(&idle_sleep_ctrl_info.qos_cpu_latency);
}

/* ------------------------------------------------------------------------- */
/* shtrpd_wake_lock_idle                                                     */
/* ------------------------------------------------------------------------- */
static void shtrpd_wake_lock_idle(void)
{
	mutex_lock(&shtrpd_cpu_idle_sleep_ctrl_lock);
	if(idle_sleep_ctrl_info.wake_lock_idle_state == 0){
		SHTRPD_DBG("wake_lock_idle on\n");
		wake_lock(&idle_sleep_ctrl_info.wake_lock);
		pm_qos_update_request(&idle_sleep_ctrl_info.qos_cpu_latency, SHTRPD_QOS_LATENCY_DEF_VALUE);
		idle_sleep_ctrl_info.wake_lock_idle_state = 1;
	}
	mutex_unlock(&shtrpd_cpu_idle_sleep_ctrl_lock);
}

/* ------------------------------------------------------------------------- */
/* shtrpd_wake_unlock_idle                                                   */
/* ------------------------------------------------------------------------- */
static void shtrpd_wake_unlock_idle(void)
{
	mutex_lock(&shtrpd_cpu_idle_sleep_ctrl_lock);
	if(idle_sleep_ctrl_info.wake_lock_idle_state != 0){
		SHTRPD_DBG("wake_lock_idle off\n");
		wake_unlock(&idle_sleep_ctrl_info.wake_lock);
		pm_qos_update_request(&idle_sleep_ctrl_info.qos_cpu_latency, PM_QOS_DEFAULT_VALUE);
		idle_sleep_ctrl_info.wake_lock_idle_state = 0;
	}
	mutex_unlock(&shtrpd_cpu_idle_sleep_ctrl_lock);
}

#endif /* SHTRPD_CPU_IDLE_SLEEP_CONTROL_ENABLE */

#if defined(SHTRPD_CPU_CLOCK_CONTROL_ENABLE)
/* ------------------------------------------------------------------------- */
/* shtrpd_perf_lock_enable                                                   */
/* ------------------------------------------------------------------------- */
static void shtrpd_perf_lock_enable(void)
{
	mutex_lock(&shtrpd_cpu_clock_ctrl_lock);

	if ( is_perf_lock_active(&cpu_clock_ctrl_info.perf_lock) == 0 ){
		perf_lock(&cpu_clock_ctrl_info.perf_lock);
		SHTRPD_DBG("perf_lock start (%d ms)\n", SHTRPD_PERF_LOCK_ENABLE_TIME_MS);
	}

	mutex_unlock(&shtrpd_cpu_clock_ctrl_lock);
}

/* ------------------------------------------------------------------------- */
/* shtrpd_perf_lock_disable                                                  */
/* ------------------------------------------------------------------------- */
static void shtrpd_perf_lock_disable(void)
{
	mutex_lock(&shtrpd_cpu_clock_ctrl_lock);

	if (is_perf_lock_active(&cpu_clock_ctrl_info.perf_lock)){
		perf_unlock(&cpu_clock_ctrl_info.perf_lock);
		SHTRPD_DBG("perf_lock end\n");
	}

	mutex_unlock(&shtrpd_cpu_clock_ctrl_lock);
}

/* ------------------------------------------------------------------------- */
/* shtrpd_perf_lock_disable_timer_start                                      */
/* ------------------------------------------------------------------------- */
static int shtrpd_perf_lock_disable_timer_start(void)
{
	cancel_delayed_work(&cpu_clock_ctrl_info.perf_lock_disable_delayed_work);
	schedule_delayed_work(&cpu_clock_ctrl_info.perf_lock_disable_delayed_work, 
								msecs_to_jiffies(SHTRPD_PERF_LOCK_ENABLE_TIME_MS));
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_perf_lock_disable_delayed_work_function                            */
/* ------------------------------------------------------------------------- */
static void shtrpd_perf_lock_disable_delayed_work_function(struct work_struct *work)
{
	shtrpd_perf_lock_disable();
	SHTRPD_DBG("perf_lock end by Timer\n");
}

/* ------------------------------------------------------------------------- */
/* shtrpd_perf_lock_init                                                    */
/* ------------------------------------------------------------------------- */
static void shtrpd_perf_lock_init(void)
{
	cpu_clock_ctrl_info.report_event             = SHTRPD_FINGER_STATE_TU;
	perf_lock_init(&cpu_clock_ctrl_info.perf_lock, PERF_LOCK_1190400KHz, "shtrpd_perf_lock");
	INIT_DELAYED_WORK(&cpu_clock_ctrl_info.perf_lock_disable_delayed_work, shtrpd_perf_lock_disable_delayed_work_function);
}

/* ------------------------------------------------------------------------- */
/* shtrpd_perf_lock_sleep                                                    */
/* ------------------------------------------------------------------------- */
static void shtrpd_perf_lock_sleep(void)
{
	cpu_clock_ctrl_info.report_event = SHTRPD_FINGER_STATE_TU;
	cancel_delayed_work(&cpu_clock_ctrl_info.perf_lock_disable_delayed_work);
	shtrpd_perf_lock_disable();
	SHTRPD_DBG("perf_lock end by sleep\n");
}

/* ------------------------------------------------------------------------- */
/* shtrpd_perf_lock_check                                                    */
/* ------------------------------------------------------------------------- */
static void shtrpd_perf_lock_check(u8 finger_state)
{
	if(finger_state == 0){
		if(shtrpd_finger_state_old != 0){
			cpu_clock_ctrl_info.report_event = SHTRPD_FINGER_STATE_TU;
			cancel_delayed_work(&cpu_clock_ctrl_info.perf_lock_disable_delayed_work);
			shtrpd_perf_lock_disable();
			SHTRPD_DBG("perf_lock end by TouchUp\n");
		}
	}else{
		if(shtrpd_finger_state_old == 0){
			cpu_clock_ctrl_info.report_event = SHTRPD_FINGER_STATE_TD;
			shtrpd_perf_lock_enable();
			shtrpd_perf_lock_disable_timer_start();
			SHTRPD_DBG("perf_lock start by TouchDown\n");
		}else{
			if(cpu_clock_ctrl_info.report_event == SHTRPD_FINGER_STATE_TD){
				int i;
				for(i = 0;i < MAX_TOUCHES;i++){
					if(shtrpd_touches[i].XPos != shtrpd_touches_old[i].XPos ||
					   shtrpd_touches[i].YPos != shtrpd_touches_old[i].YPos)
					{
						break;
					}
				}
				if(i < MAX_TOUCHES){
					cpu_clock_ctrl_info.report_event = SHTRPD_FINGER_STATE_DRAG;
					shtrpd_perf_lock_enable();
					shtrpd_perf_lock_disable_timer_start();
					SHTRPD_DBG("perf_lock start by Drag\n");
				}
			}
		}
	}
}
#endif /* SHTRPD_CPU_CLOCK_CONTROL_ENABLE */

#if defined(SHTRPD_DRUMMING_SPLIT_ENABLE)

#if defined(SHTRPD_DRUMMING_SPLIT_SWITCH_FINGER_ENABLE)
/* ------------------------------------------------------------------------- */
/* shtrpd_replace_fingers_init                                               */
/* ------------------------------------------------------------------------- */
static void shtrpd_replace_fingers_init(void)
{
	int i;
	for(i = 0;i < MAX_TOUCHES;i++){
		replace_fingers.order[i] = i;
	}
}

/* ------------------------------------------------------------------------- */
/* shtrpd_replace_fingers                                                    */
/* ------------------------------------------------------------------------- */
static void shtrpd_replace_fingers(u8 *finger_state_p)
{
	int i;
	u8 tmp_state;
	struct shtrpd_fingers_structure	tmp_touches[MAX_TOUCHES];
	
	tmp_state = *finger_state_p;
	replace_fingers.fw_finger_state = *finger_state_p;
	
	memcpy(tmp_touches, shtrpd_touches, sizeof(tmp_touches));
	
	for(i = 0;i < MAX_TOUCHES;i++){
		if(i != replace_fingers.order[i]){
			if((*finger_state_p & (1 << replace_fingers.order[i])) != 0){
				tmp_state |= (1 << i);
			}else{
				tmp_state &= ~(1 << i);
			}
			memcpy(&tmp_touches[i], &shtrpd_touches[replace_fingers.order[i]], sizeof(tmp_touches[i]));

			SHTRPD_LOG_DBG_DRUMMING_SPLIT_PRINT("replace finger [%d] <- [%d]\n", i, replace_fingers.order[i]);
		}
	}

	if(*finger_state_p != tmp_state)
		SHTRPD_LOG_DBG_DRUMMING_SPLIT_PRINT("replace finger state[0x%02x] -> [0x%02x]\n", *finger_state_p, tmp_state);
	
	*finger_state_p = tmp_state;
	memcpy(shtrpd_touches, tmp_touches, sizeof(shtrpd_touches));
}

/* ------------------------------------------------------------------------- */
/* shtrpd_replace_fingers_switch                                             */
/* ------------------------------------------------------------------------- */
static void shtrpd_replace_switch(u8 a, u8 b)
{
	u8 tmp;
	
	SHTRPD_LOG_DBG_DRUMMING_SPLIT_PRINT("switch finger [%d](%d) <-> [%d](%d)\n", a, replace_fingers.order[a], b, replace_fingers.order[b]);
	
	tmp = replace_fingers.order[a];
	replace_fingers.order[a] = replace_fingers.order[b];
	replace_fingers.order[b] = tmp;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_replace_fingers_clear_check                                        */
/* ------------------------------------------------------------------------- */
static void shtrpd_replace_fingers_clear_check(u8 finger_state)
{
	int i;
	
	for(i = 0;i < MAX_TOUCHES;i++){
		if(i != replace_fingers.order[i] && i < replace_fingers.order[i]){
			if((replace_fingers.fw_finger_state & (1 << i)) == 0 &&
				(replace_fingers.fw_finger_state & (1 << replace_fingers.order[i])) == 0)
			{
				shtrpd_replace_switch(i, replace_fingers.order[i]);
			}
		}
	}
}
#endif /* SHTRPD_DRUMMING_SPLIT_SWITCH_FINGER_ENABLE) */

/* ------------------------------------------------------------------------- */
/* shtrpd_drumming_split_notify_no_touch                                     */
/* ------------------------------------------------------------------------- */
static void shtrpd_drumming_split_init(void)
{
	memset(drumming_split, 0, sizeof(drumming_split));
}

/* ------------------------------------------------------------------------- */
/* shtrpd_drumming_split_notify_no_touch                                     */
/* ------------------------------------------------------------------------- */
static void shtrpd_drumming_split_notify_no_touch(struct shtrpd_ts_data *data, u8 finger)
{
	input_mt_slot(data->input_dev, finger);
    input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, -1);
    input_sync(data->input_dev);

#if defined( SHTRPD_DRAG_SMOOTH_ENABLE )
    shtrpd_init_drag_hist(SHTRPD_POSTYPE_X, finger, shtrpd_touches_old[finger].XPos);
    shtrpd_init_drag_hist(SHTRPD_POSTYPE_Y, finger, shtrpd_touches_old[finger].YPos);
#endif /* SHTRPD_DRAG_SMOOTH_ENABLE */

#if defined( SHTRPD_TAP_JITTER_FILTER_ENABLE )
    tap_jitter[finger].enable = 0;
#endif /* SHTRPD_TAP_JITTER_FILTER_ENABLE */
    
    SHTRPD_INFO("ABS_MT_TRACKING_ID ID=%d ; STATE=0 ; X=0 ; Y=0 ; Z=0 (drumming split)\n", finger);
}

/* ------------------------------------------------------------------------- */
/* shtrpd_drumming_split_notify_hold_touch                                   */
/* ------------------------------------------------------------------------- */
static void shtrpd_drumming_split_notify_hold_touch(struct shtrpd_ts_data *data, u8 finger)
{
    if(shtrpd_touchcruiser_state){
	    input_mt_slot(data->input_dev, finger);
	    input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, finger+1);
	    input_report_abs(data->input_dev, ABS_MT_POSITION_X, SHTRPD_NOTIFY_POS_SCALE_X(drumming_split[finger].hold_touch.XPos));
	    input_report_abs(data->input_dev, ABS_MT_POSITION_Y, SHTRPD_NOTIFY_POS_SCALE_Y(drumming_split[finger].hold_touch.YPos));
	    input_report_abs(data->input_dev, ABS_MT_PRESSURE, drumming_split[finger].hold_touch.touchStrength);
	    input_sync(data->input_dev);
    
	    SHTRPD_INFO("ABS_MT_TRACKING_ID ID=%d ; STATE=1 ; X=%d ; Y=%d ; Z=%d (drumming split)\n" ,
                 finger, drumming_split[finger].hold_touch.XPos, drumming_split[finger].hold_touch.YPos,
                 drumming_split[finger].hold_touch.touchStrength);
	}
}

/* ------------------------------------------------------------------------- */
/* shtrpd_drumming_split_add_diff                                            */
/* ------------------------------------------------------------------------- */
static void shtrpd_drumming_split_add_diff(u8 finger, u8 finger_state)
{
	int i;
	int diff_sample_max = SHTRPD_DRUMMING_SPLIT_DIFF_SAMPLE_NUM;
	
	if(diff_sample_max > SHTRPD_DRUMMING_SPLIT_DIFF_SAMPLE_MAX){
		diff_sample_max = SHTRPD_DRUMMING_SPLIT_DIFF_SAMPLE_MAX;
	}

	if((finger_state & shtrpd_finger_state_old & (1 << finger)) != 0){
		int xdiff = shtrpd_touches[finger].XPos - drumming_split[finger].hold_touch.XPos;
		int ydiff = shtrpd_touches[finger].YPos - drumming_split[finger].hold_touch.YPos;
		
		SHTRPD_LOG_DBG_DRUMMING_SPLIT_PRINT("[%d] add diff(%d, %d)\n", finger, xdiff, ydiff);
		
		if(drumming_split[finger].diff_num < diff_sample_max){
			drumming_split[finger].diff_hist_x[drumming_split[finger].diff_num] = xdiff;
			drumming_split[finger].diff_hist_y[drumming_split[finger].diff_num] = ydiff;
			drumming_split[finger].diff_num++;
		}else{
			for(i= 1;i < diff_sample_max;i++){
				drumming_split[finger].diff_hist_x[i-1] = drumming_split[finger].diff_hist_x[i];
				drumming_split[finger].diff_hist_y[i-1] = drumming_split[finger].diff_hist_y[i];
			}
			drumming_split[finger].diff_hist_x[diff_sample_max-1] = xdiff;
			drumming_split[finger].diff_hist_y[diff_sample_max-1] = ydiff;
		}
	}else{
		drumming_split[finger].diff_num = 0;
	}
}

/* ------------------------------------------------------------------------- */
/* shtrpd_drumming_split_get_diff                                            */
/* ------------------------------------------------------------------------- */
static void shtrpd_drumming_split_get_diff(u8 finger, int *xdiff_avg, int *ydiff_avg,
	int *xdiff_pre, int *ydiff_pre)
{
	int i;
	
	*xdiff_avg = 0;
	*ydiff_avg = 0;
	*xdiff_pre = 0;
	*ydiff_pre = 0;
	
	if((drumming_split[finger].diff_num - 1) > 0){
		for(i = 0;i < drumming_split[finger].diff_num - 1;i++){
			*xdiff_avg += drumming_split[finger].diff_hist_x[i];
			*ydiff_avg += drumming_split[finger].diff_hist_y[i];
		}
		
		*xdiff_avg /= (drumming_split[finger].diff_num - 1);
		*ydiff_avg /= (drumming_split[finger].diff_num - 1);

		*xdiff_pre = drumming_split[finger].diff_hist_x[drumming_split[finger].diff_num - 2];
		*ydiff_pre = drumming_split[finger].diff_hist_y[drumming_split[finger].diff_num - 2];
	}
	
	SHTRPD_LOG_DBG_DRUMMING_SPLIT_PRINT("[%d] diff avg = (%d, %d) pre = (%d, %d)\n", 
			finger, *xdiff_avg, *ydiff_avg, *xdiff_pre, *ydiff_pre);
}

/* ------------------------------------------------------------------------- */
/* shtrpd_drumming_split_check_mtap                                          */
/* ------------------------------------------------------------------------- */
static void shtrpd_drumming_split_check_mtap(struct shtrpd_ts_data *data, u8 finger_state)
{
	int i, j;
	
	for(i = 0;i < MAX_TOUCHES;i++){
		if((finger_state & (1 << i)) != 0 && (shtrpd_finger_state_old & (1 << i)) != 0){
			if(abs(shtrpd_touches[i].XPos - shtrpd_touches_old[i].XPos) >= SHTRPD_DRUMMING_SPLIT_MOVE_TH ||
			   abs(shtrpd_touches[i].YPos - shtrpd_touches_old[i].YPos) >= SHTRPD_DRUMMING_SPLIT_MOVE_TH)
			{
				if(abs(shtrpd_touches[i].XPos - SHTRPD_DRUMMING_SPLIT_MTAP_DEAD_AREA_X) <= SHTRPD_DRUMMING_SPLIT_MTAP_DEAD_AREA_SIZE &&
				   abs(shtrpd_touches[i].YPos - SHTRPD_DRUMMING_SPLIT_MTAP_DEAD_AREA_Y) <= SHTRPD_DRUMMING_SPLIT_MTAP_DEAD_AREA_SIZE)
				{
					SHTRPD_LOG_DBG_DRUMMING_SPLIT_PRINT("[%d] split finger in multi (dead area).\n", i);
					shtrpd_drumming_split_notify_no_touch(data, i);
				}else{
					for(j = 0;j < MAX_TOUCHES;j++){
						if(j == i){
							continue;
						}
						
						if((finger_state & (1 << j)) != 0){
							if(abs(shtrpd_touches[i].XPos - shtrpd_touches[j].XPos) <= SHTRPD_DRUMMING_SPLIT_MTAP_POS_TH &&
							   abs(shtrpd_touches[i].YPos - shtrpd_touches[j].YPos) <= SHTRPD_DRUMMING_SPLIT_MTAP_POS_TH)
							{
								SHTRPD_LOG_DBG_DRUMMING_SPLIT_PRINT("[%d] split finger in multi.\n", i);
								shtrpd_drumming_split_notify_no_touch(data, i);
								
							}else if((shtrpd_finger_state_old & (1 << j)) != 0){
								if(abs(shtrpd_touches[i].XPos - shtrpd_touches_old[j].XPos) <= SHTRPD_DRUMMING_SPLIT_MTAP_POS_TH &&
								   abs(shtrpd_touches[i].YPos - shtrpd_touches_old[j].YPos) <= SHTRPD_DRUMMING_SPLIT_MTAP_POS_TH)
								{
									SHTRPD_LOG_DBG_DRUMMING_SPLIT_PRINT("[%d] split finger in multi.\n", i);
									shtrpd_drumming_split_notify_no_touch(data, i);
								}
							}
						}
					}
				}
			}
		}
	}
}

/* ------------------------------------------------------------------------- */
/* shtrpd_drumming_split_check_rjump                                         */
/* ------------------------------------------------------------------------- */
static int shtrpd_drumming_split_check_rjump(int xdiff, int ydiff, int xdiff_avg, int ydiff_avg, 
	int diff_sample_num)
{
	if(abs(xdiff) >= SHTRPD_DRUMMING_SPLIT_RJUMP_MOVE_TH &&
		(diff_sample_num <= 1 || ((xdiff_avg >= 0 && xdiff < 0) || (xdiff_avg <= 0 && xdiff > 0))))
	{
		SHTRPD_LOG_DBG_DRUMMING_SPLIT_PRINT("rjump detect (x diff avg (num=%d) = %d , x diff = %d).\n", 
			diff_sample_num, xdiff_avg, xdiff);
		return 1;
	}
	
	if(abs(ydiff) >= SHTRPD_DRUMMING_SPLIT_RJUMP_MOVE_TH &&
		(diff_sample_num <= 1 || ((ydiff_avg >= 0 && ydiff < 0) || (ydiff_avg <= 0 && ydiff > 0))))
	{
		SHTRPD_LOG_DBG_DRUMMING_SPLIT_PRINT("rjump detect (y diff avg (num=%d) = %d , y diff = %d).\n", 
			diff_sample_num, ydiff_avg, ydiff);
		return 1;
	}
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_drumming_split_check_tap                                           */
/* ------------------------------------------------------------------------- */
static int shtrpd_drumming_split_check_tap(int xdiff, int ydiff, int xdiff_pre, int ydiff_pre, 
	int diff_sample_num, int tap_move_thresh)
{
	if(diff_sample_num <= 1){
		return 0;
	}
	
	if((abs(xdiff_pre) <= SHTRPD_DRUMMING_SPLIT_TAP_PRE_MOVE_TH && abs(ydiff_pre) <= SHTRPD_DRUMMING_SPLIT_TAP_PRE_MOVE_TH) &&
	   (abs(xdiff) >= tap_move_thresh || abs(ydiff) >= tap_move_thresh))
	{
		return 1;
	}
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_drumming_split_check                                               */
/* ------------------------------------------------------------------------- */
static void shtrpd_drumming_split_check(struct shtrpd_ts_data *data, u8 *finger_state_p, u16 *x, u16 *y)
{
	int i;
	int fw_finger_num = 0, finger_num = 0;
	int tap_move_thresh;
	u16 tmpX, tmpY;
	
	if(!SHTRPD_DRUMMING_SPLIT){
		return;
	}
	
	for(i = 0;i < MAX_TOUCHES;i++){
		if((shtrpd_finger_fw_state & (1 << i)) != 0){
			fw_finger_num++;
		}
		if((*finger_state_p & (1 << i)) != 0){
			finger_num++;
		}
	}

	if(fw_finger_num > 1){
		shtrpd_drumming_split_check_mtap(data, *finger_state_p);
		tap_move_thresh = SHTRPD_DRUMMING_SPLIT_MULTI_MOVE_TH;
	}else{
		tap_move_thresh = SHTRPD_DRUMMING_SPLIT_MOVE_TH;
	}
	
	if(finger_num > 1){
		drumming_split_latest_mt_time = jiffies;
	}
	
	for(i = 0;i < MAX_TOUCHES;i++){
	
		if(drumming_split[i].is_hold != 0){
			if((*finger_state_p & (1 << i)) != 0){
				SHTRPD_LOG_DBG_DRUMMING_SPLIT_PRINT("[%d] check tap split (%d, %d) - (%d, %d) = abs(%lu, %lu)\n",
					i, drumming_split[i].hold_touch.XPos, drumming_split[i].hold_touch.YPos,
					shtrpd_touches[i].XPos, shtrpd_touches[i].YPos,
					abs(drumming_split[i].hold_touch.XPos - shtrpd_touches[i].XPos),
					abs(drumming_split[i].hold_touch.YPos - shtrpd_touches[i].YPos));
				
				if(abs(drumming_split[i].hold_touch.XPos - shtrpd_touches[i].XPos) <= SHTRPD_DRUMMING_SPLIT_TAP_POST_MOVE_TH &&
			       abs(drumming_split[i].hold_touch.YPos - shtrpd_touches[i].YPos) <= SHTRPD_DRUMMING_SPLIT_TAP_POST_MOVE_TH)
				{
					shtrpd_drumming_split_notify_no_touch(data, i);
				}
			}else{
				if(drumming_split[i].is_flick == 0){
					shtrpd_drumming_split_notify_no_touch(data, i);
				}
			}
			
			shtrpd_drumming_split_notify_hold_touch(data, i);
			
			drumming_split[i].is_hold  = 0;
			drumming_split[i].diff_num = 0;
		}

		shtrpd_drumming_split_add_diff(i, *finger_state_p);

		if((*finger_state_p & (1 << i)) != 0 && 
		   (shtrpd_finger_state_old & (1 << i)) != 0)
		{
			int xdiff = shtrpd_touches[i].XPos - drumming_split[i].hold_touch.XPos;
			int ydiff = shtrpd_touches[i].YPos - drumming_split[i].hold_touch.YPos;
			int xdiff_avg, ydiff_avg;
			int xdiff_pre, ydiff_pre;
			
			shtrpd_drumming_split_get_diff(i, &xdiff_avg, &ydiff_avg, &xdiff_pre, &ydiff_pre);
			
			if(time_after(jiffies, drumming_split_latest_mt_time + 
					msecs_to_jiffies(SHTRPD_DRUMMING_SPLIT_RJUMP_CHECK_TIME)) == 0 &&
				shtrpd_drumming_split_check_rjump(xdiff, ydiff, 
					xdiff_avg, ydiff_avg, drumming_split[i].diff_num) != 0)
			{
				int j;
				
				SHTRPD_LOG_DBG_DRUMMING_SPLIT_PRINT("[%d] split finger (revers jump)\n", i);
				
				#if defined(SHTRPD_DRUMMING_SPLIT_SWITCH_FINGER_ENABLE)
				for(j = 0;j < MAX_TOUCHES;j++){
					if(i != j && (*finger_state_p & (1 << j)) != 0 && (shtrpd_finger_fw_state & (1 << j)) == 0){
						shtrpd_replace_switch(i, j);
						*finger_state_p &= ~(1 << i);
						memcpy(&shtrpd_touches[j], &shtrpd_touches[i], sizeof(shtrpd_touches[j]));
						drumming_split[j].hold_touch.XPos = shtrpd_touches[j].XPos;
						drumming_split[j].hold_touch.YPos = shtrpd_touches[j].YPos;
						x[j] = shtrpd_touches[j].XPos;
						y[j] = shtrpd_touches[j].YPos;
						#if defined(SHTRPD_TOUCH_UP_DELAY_ENABLE)
							td_history[j].x_bk = shtrpd_touches[j].XPos;
							td_history[j].y_bk = shtrpd_touches[j].YPos;
							td_history[j].z_bk = shtrpd_touches[j].touchStrength;
							td_history[j].a_bk = shtrpd_touches[j].area;
						#endif /* SHTRPD_TOUCH_UP_DELAY_ENABLE */
						
						memcpy(&shtrpd_touches[i], &shtrpd_touches_old[i], sizeof(shtrpd_touches[i]));
						drumming_split[i].hold_touch.XPos = shtrpd_touches[i].XPos;
						drumming_split[i].hold_touch.YPos = shtrpd_touches[i].YPos;
						x[i] = shtrpd_touches[i].XPos;
						y[i] = shtrpd_touches[j].YPos;
						#if defined(SHTRPD_TOUCH_UP_DELAY_ENABLE)
							td_history[i].x_bk = shtrpd_touches[i].XPos;
							td_history[i].y_bk = shtrpd_touches[i].YPos;
							td_history[i].z_bk = shtrpd_touches[i].touchStrength;
							td_history[i].a_bk = shtrpd_touches[i].area;
						#endif /* SHTRPD_TOUCH_UP_DELAY_ENABLE */
						break;
					}
				}
				if(j >= MAX_TOUCHES){
					shtrpd_drumming_split_notify_no_touch(data, i);
				}
				#else
				shtrpd_drumming_split_notify_no_touch(data, i);
				#endif /* SHTRPD_DRUMMING_SPLIT_SWITCH_FINGER_ENABLE */
			}else if(shtrpd_drumming_split_check_tap(xdiff, ydiff, 
						xdiff_pre, ydiff_pre, drumming_split[i].diff_num, tap_move_thresh) != 0)
			{
				SHTRPD_LOG_DBG_DRUMMING_SPLIT_PRINT("[%d] hold touch. diff=(%lu, %lu)\n", i,
					abs(drumming_split[i].hold_touch.XPos - shtrpd_touches[i].XPos),
					abs(drumming_split[i].hold_touch.YPos - shtrpd_touches[i].YPos));
				
				drumming_split[i].is_hold = 1;
				if(abs(drumming_split[i].hold_touch.XPos - shtrpd_touches[i].XPos) < SHTRPD_DRUMMING_SPLIT_FLICK_TH &&
				   abs(drumming_split[i].hold_touch.YPos - shtrpd_touches[i].YPos) < SHTRPD_DRUMMING_SPLIT_FLICK_TH)
				{
					drumming_split[i].is_flick = 1;
				}else{
					drumming_split[i].is_flick = 0;
				}

				tmpX = drumming_split[i].hold_touch.XPos;
				tmpY = drumming_split[i].hold_touch.YPos;

				memcpy(&drumming_split[i].hold_touch, &shtrpd_touches[i], sizeof(drumming_split[i].hold_touch));
				
				shtrpd_touches[i].XPos = tmpX;
				shtrpd_touches[i].YPos = tmpY;
				x[i] = tmpX;
				y[i] = tmpY;

				SHTRPD_LOG_DBG_DRUMMING_SPLIT_PRINT("[%d] hold touch = (%d, %d)\n", i,
					drumming_split[i].hold_touch.XPos,
					drumming_split[i].hold_touch.YPos);
			}
		}else{
			drumming_split[i].is_hold  = 0;
			drumming_split[i].diff_num = 0;
		}
		
		if(drumming_split[i].is_hold == 0 && (*finger_state_p & (1 << i)) != 0){
			drumming_split[i].hold_touch.XPos = shtrpd_touches[i].XPos;
			drumming_split[i].hold_touch.YPos = shtrpd_touches[i].YPos;
		}
	}
}
#endif /* SHTRPD_DRUMMING_SPLIT_ENABLE */

#if defined(SHTRPD_TAP_JITTER_FILTER_ENABLE)
/* ------------------------------------------------------------------------- */
/* shtrpd_tap_jitter_filter_init                                             */
/* ------------------------------------------------------------------------- */
static void shtrpd_tap_jitter_filter_init(void)
{
	memset(tap_jitter, 0, sizeof(tap_jitter));
}

/* ------------------------------------------------------------------------- */
/* shtrpd_tap_jitter_filter_report_touch                                     */
/* ------------------------------------------------------------------------- */
static void shtrpd_tap_jitter_filter_report_touch(void *dev_id, int id)
{
    struct shtrpd_ts_data *data = dev_id;

	if(shtrpd_touchcruiser_state){
	    input_mt_slot(data->input_dev, id);
	    input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, id+1);
	    input_report_abs(data->input_dev, ABS_MT_POSITION_X, SHTRPD_NOTIFY_POS_SCALE_X(tap_jitter[id].hold_touch.XPos));
	    input_report_abs(data->input_dev, ABS_MT_POSITION_Y, SHTRPD_NOTIFY_POS_SCALE_Y(tap_jitter[id].hold_touch.YPos));
	    input_report_abs(data->input_dev, ABS_MT_PRESSURE, tap_jitter[id].hold_touch.touchStrength);
	    input_sync(data->input_dev);
	}
    
    shtrpd_finger_state_old |= (1 << id);

#if defined(SHTRPD_DRUMMING_SPLIT_ENABLE)
	drumming_split[id].hold_touch.XPos = tap_jitter[id].hold_touch.XPos;
	drumming_split[id].hold_touch.YPos = tap_jitter[id].hold_touch.YPos;
#endif /* SHTRPD_DRUMMING_SPLIT_ENABLE */

    SHTRPD_INFO("ABS_MT_TRACKING_ID ID=%d ; STATE=1 ; X=%d ; Y=%d ; Z=%d (tap jitter hold event)\n" ,
                 id, tap_jitter[id].hold_touch.XPos, tap_jitter[id].hold_touch.YPos,
                 tap_jitter[id].hold_touch.touchStrength);
}

/* ------------------------------------------------------------------------- */
/* shtrpd_tap_jitter_filter_check                                            */
/* ------------------------------------------------------------------------- */
void shtrpd_tap_jitter_filter_check(void *dev_id, u8 *finger_state_p)
{
	int i;
	
	if(!SHTRPD_TAP_JITTER_FILTER){
		return;
	}
	
    for(i = 0;i < MAX_TOUCHES ;i++){
		if((*finger_state_p & (1 << i)) != 0){
			if(tap_jitter[i].enable == 0 && (shtrpd_finger_state_old & (1 << i)) == 0 &&
				((shtrpd_finger_fw_state_old & (1 << i)) == 0 && (shtrpd_finger_fw_state & (1 << i)) != 0))
			{
				tap_jitter[i].enable = 1;
				tap_jitter[i].td_time= jiffies;
				tap_jitter[i].jump_check_enable = 1;
				memcpy(&tap_jitter[i].hold_touch, &shtrpd_touches[i], sizeof(tap_jitter[i].hold_touch));

			}else if(tap_jitter[i].enable != 0){
				int ts = (shtrpd_touches[i].touchStrength == 0)? 1: shtrpd_touches[i].touchStrength;
				int pre_z = (tap_jitter[i].hold_touch.area == 0)?
					tap_jitter[i].hold_touch.touchStrength : (tap_jitter[i].hold_touch.touchStrength / tap_jitter[i].hold_touch.area);
				int cur_z = (shtrpd_touches[i].area == 0)?
					shtrpd_touches[i].touchStrength : (shtrpd_touches[i].touchStrength / shtrpd_touches[i].area);
				unsigned long xdiff    = abs(shtrpd_touches[i].XPos - tap_jitter[i].hold_touch.XPos);
				unsigned long ydiff    = abs(shtrpd_touches[i].YPos - tap_jitter[i].hold_touch.YPos);
				unsigned long tsdiff   = abs(shtrpd_touches[i].touchStrength - tap_jitter[i].hold_touch.touchStrength);
				unsigned long areadiff = abs(shtrpd_touches[i].area - tap_jitter[i].hold_touch.area);
				unsigned long zdiff    = abs(pre_z - cur_z);
				
				if(tap_jitter[i].jump_check_enable != 0){
					/* jump-up reject */
					if((shtrpd_touches[i].YPos < tap_jitter[i].hold_touch.YPos && ydiff >= SHTRPD_TAP_JITTER_FILTER_JUMPUP_Y_THRESH) &&
					   (shtrpd_touches[i].area > tap_jitter[i].hold_touch.area) &&
					   (cur_z < pre_z && (pre_z == 0 || ((zdiff * 100) / pre_z) > SHTRPD_TAP_JITTER_FILTER_JUMPUP_Z_THRESH)))
					{
						SHTRPD_LOG_DBG_SHTRPD_TAP_JITTER_FILTER_PRINT("[%d] detect jump-up.\n", i);
						tap_jitter[i].enable = 0;
					}
					/* jump-down reject */
					else if((shtrpd_touches[i].YPos > tap_jitter[i].hold_touch.YPos && ydiff >= SHTRPD_TAP_JITTER_FILTER_JUMPDOWN_Y_THRESH) &&
					        (shtrpd_touches[i].area < tap_jitter[i].hold_touch.area) &&
					        (areadiff >= SHTRPD_TAP_JITTER_FILTER_JUMPDOWN_AREA_THRESH) &&
					        (pre_z > 0 && ((zdiff * 100) / pre_z) < SHTRPD_TAP_JITTER_FILTER_JUMPDOWN_Z_THRESH))
					{
						SHTRPD_LOG_DBG_SHTRPD_TAP_JITTER_FILTER_PRINT("[%d] detect jump-down.\n", i);
						tap_jitter[i].enable = 0;
					}
					tap_jitter[i].jump_check_enable = 0;
				}
				
				/* jitter reject */
				if(tap_jitter[i].enable){
					if(time_after(jiffies, tap_jitter[i].td_time + msecs_to_jiffies(SHTRPD_TAP_JITTER_FILTER_TIME_THRESH)) == 0 &&
					   ((tsdiff * 100) / ts) >= SHTRPD_TAP_JITTER_FILTER_TSDIFF_THRESH &&
					   xdiff <= SHTRPD_TAP_JITTER_FILTER_MOVE_OVER_THRESH && ydiff <= SHTRPD_TAP_JITTER_FILTER_MOVE_OVER_THRESH)
					{
						memcpy(&tap_jitter[i].hold_touch, &shtrpd_touches[i], sizeof(tap_jitter[i].hold_touch));
					}else{
						tap_jitter[i].enable = 0;
						shtrpd_tap_jitter_filter_report_touch(dev_id, i);
					}
				}
			}else{
				tap_jitter[i].enable = 0;
			}
		}else{
			if(tap_jitter[i].enable){
				tap_jitter[i].enable = 0;
				shtrpd_tap_jitter_filter_report_touch(dev_id, i);
			}
		}
		
		if(tap_jitter[i].enable){
			*finger_state_p &= ~(1 << i);
            SHTRPD_LOG_DBG_SHTRPD_TAP_JITTER_FILTER_PRINT("[%d] is inhibited\n", i);
		}
	}
}
#endif /* SHTRPD_TAP_JITTER_FILTER_ENABLE */

#if defined(SHTRPD_SPLIT_FINGER_REJECTION_ENABLE)
/* ------------------------------------------------------------------------- */
/* shtrpd_split_finger_reject_init                                           */
/* ------------------------------------------------------------------------- */
static void shtrpd_split_finger_reject_init(void)
{
	memset(split_finger_reject, 0, sizeof(split_finger_reject));
}

/* ------------------------------------------------------------------------- */
/* shtrpd_split_finger_reject_getz                                           */
/* ------------------------------------------------------------------------- */
static inline int shtrpd_split_finger_reject_getz(int ts, int area)
{
	return (area > 0)? (ts / area) : (ts);
}

/* ------------------------------------------------------------------------- */
/* shtrpd_split_finger_reject_check_ghost                                    */
/* ------------------------------------------------------------------------- */
static void shtrpd_split_finger_reject_check_ghost(u8 finger_state, u8 finger, u8 zcheck)
{
	int i;
	unsigned long dist_th;
	unsigned long xdiff, ydiff;
	
	if(SHTRPD_SPLIT_FINGER_REJECT_MULTI_RANGE >= SHTRPD_SPLIT_FINGER_REJECT_SINGLE_RANGE){
		dist_th = SHTRPD_SPLIT_FINGER_REJECT_MULTI_RANGE * SHTRPD_SPLIT_FINGER_REJECT_MULTI_RANGE;
	}else{
		dist_th = SHTRPD_SPLIT_FINGER_REJECT_SINGLE_RANGE * SHTRPD_SPLIT_FINGER_REJECT_SINGLE_RANGE;
	}

	for(i = 0;i < MAX_TOUCHES;i++){
		if(i == finger){
			continue;
		}
		
		if(split_finger_reject[i].is_ghost == 0){
			xdiff = abs(shtrpd_touches[finger].XPos - shtrpd_touches[i].XPos);
			ydiff = abs(shtrpd_touches[finger].YPos - shtrpd_touches[i].YPos);
			
			if(((xdiff * xdiff) + (ydiff * ydiff)) <= dist_th){
				if((shtrpd_finger_state_old & (1 << i)) != 0 && (shtrpd_finger_state_old & (1 << finger)) == 0){
					if(split_finger_reject[finger].is_ghost == 0){
						SHTRPD_LOG_DBG_SPLIT_FINGER_REJECT_PRINT("[%d] detect ghost\n", finger);
						split_finger_reject[finger].is_ghost = 1;
					}
				}else if((shtrpd_finger_state_old & (1 << i)) == 0 && (shtrpd_finger_state_old & (1 << finger)) != 0){
					SHTRPD_LOG_DBG_SPLIT_FINGER_REJECT_PRINT("[%d] detect ghost\n", i);
					split_finger_reject[i].is_ghost = 1;
				}else{
					if(shtrpd_split_finger_reject_getz(shtrpd_touches[i].touchStrength, shtrpd_touches[i].area) <= 
						shtrpd_split_finger_reject_getz(shtrpd_touches[finger].touchStrength, shtrpd_touches[finger].area))
					{
						SHTRPD_LOG_DBG_SPLIT_FINGER_REJECT_PRINT("[%d] detect ghost\n", i);
						split_finger_reject[i].is_ghost = 1;
					}else{
						SHTRPD_LOG_DBG_SPLIT_FINGER_REJECT_PRINT("[%d] detect ghost\n", finger);
						split_finger_reject[finger].is_ghost = 1;
					}
				}
			}
		}
	}
}

/* ------------------------------------------------------------------------- */
/* shtrpd_split_finger_reject_check_release                                  */
/* ------------------------------------------------------------------------- */
static void shtrpd_split_finger_reject_check_release(u8 finger_state, 
	u8 finger, unsigned long dist_th, unsigned long hysteresis)
{
	int i;
	u8 is_release = 0;
	unsigned long xdiff, ydiff;
	
	dist_th = (dist_th + hysteresis) * (dist_th + hysteresis);

	SHTRPD_LOG_DBG_SPLIT_FINGER_REJECT_PRINT("[%d] release check\n", finger);

	for(i = 0;i < MAX_TOUCHES;i++){
		if(i == finger){
			continue;
		}
		
		SHTRPD_LOG_DBG_SPLIT_FINGER_REJECT_PRINT("[%d] release check with %d (state=%d)\n", finger, i, (finger_state & (1 << i)) != 0);
		if((finger_state & (1 << i)) != 0){
			xdiff = abs(shtrpd_touches[finger].XPos - shtrpd_touches[i].XPos);
			ydiff = abs(shtrpd_touches[finger].YPos - shtrpd_touches[i].YPos);
			
			SHTRPD_LOG_DBG_SPLIT_FINGER_REJECT_PRINT("[%d][%d] (xdiff=%lu, ydiff=%lu)=%lu, dist_th=%lu\n", 
				finger, i, xdiff, ydiff, (xdiff * xdiff) + (ydiff * ydiff), dist_th);

			if(((xdiff * xdiff) + (ydiff * ydiff)) <= dist_th){
				is_release = 1;
				split_finger_reject[finger].pair_fingers |= (1 << i);
			}else{
				split_finger_reject[finger].pair_fingers &= ~(1 << i);
			}
		}
	}
	
	if(is_release == 0){
		if(split_finger_reject[finger].is_hold == 0){
			u8 pair_fingers_cur = (finger_state & split_finger_reject[finger].pair_fingers);
			u8 pair_fingers_old = (shtrpd_finger_state_old & split_finger_reject[finger].pair_fingers);
			
			if(((pair_fingers_cur ^ pair_fingers_old) & pair_fingers_old) != 0){
				SHTRPD_LOG_DBG_SPLIT_FINGER_REJECT_PRINT("[%d] release hold\n", finger);
				split_finger_reject[finger].is_hold = 1;
				split_finger_reject[finger].hold_time = jiffies;
			}else{
				SHTRPD_LOG_DBG_SPLIT_FINGER_REJECT_PRINT("[%d] release ghost\n", finger);
				split_finger_reject[finger].is_ghost = 0;
			}
			split_finger_reject[finger].pair_fingers = 0;

		}else{
			if(time_after(jiffies, split_finger_reject[finger].hold_time + 
							msecs_to_jiffies(SHTRPD_SPLIT_FINGER_REJECT_TU_HOLD_TIME)) != 0)
			{
				SHTRPD_LOG_DBG_SPLIT_FINGER_REJECT_PRINT("[%d] release ghost\n", finger);
				split_finger_reject[finger].is_ghost = 0;
				split_finger_reject[finger].is_hold  = 0;
			}
		}
	}
	
	split_finger_reject[finger].pair_fingers &= finger_state;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_split_finger_reject_get_rej_state                                  */
/* ------------------------------------------------------------------------- */
static int shtrpd_split_finger_reject_get_rej_state(u8 finger_state, u8 *rej_finger_state_p)
{
	int i;
	int finger_num = 0;
	
	*rej_finger_state_p = finger_state;
	for(i = 0;i < MAX_TOUCHES;i++){
		if(split_finger_reject[i].is_ghost){
			*rej_finger_state_p &= ~(1 << i);
		}
		
		if((*rej_finger_state_p & (1 << i)) != 0){
			finger_num++;
		}
	}
	
	return finger_num;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_split_finger_reject_check                                          */
/* ------------------------------------------------------------------------- */
static void shtrpd_split_finger_reject_check(u8 *finger_state_p)
{
	u8  tmp_finger_state, tmp_pre_finger_state;
	int i;
	int finger_num = 0;
	unsigned long dist_th;
	
	if(!SHTRPD_SPLIT_FINGER_REJECT_ENABLE){
		return;
	}
	
	shtrpd_split_finger_reject_get_rej_state(*finger_state_p, &tmp_pre_finger_state);
	
	if(shtrpd_finger_state_old == 0 && *finger_state_p != 0){
		for(i = 0;i < MAX_TOUCHES;i++){
			if(((*finger_state_p) & (1 << i)) != 0){
				shtrpd_split_finger_reject_check_ghost(*finger_state_p, i, 1);
			}
		}
	}else{
		for(i = 0;i < MAX_TOUCHES;i++){
			if(((*finger_state_p) & (1 << i)) != 0){
				if((shtrpd_finger_state_old & (1 << i)) != 0){
					shtrpd_split_finger_reject_check_ghost(*finger_state_p, i, 1);
				}else{
					shtrpd_split_finger_reject_check_ghost(*finger_state_p, i, 0);
				}
			}else{
				if(split_finger_reject[i].is_ghost){
					SHTRPD_LOG_DBG_SPLIT_FINGER_REJECT_PRINT("[%d] release ghost\n", i);
				}
				split_finger_reject[i].is_ghost = 0;
				split_finger_reject[i].is_hold = 0;
				split_finger_reject[i].pair_fingers = 0;
			}
		}
	}

	
	finger_num = shtrpd_split_finger_reject_get_rej_state(*finger_state_p, &tmp_finger_state);
	
	if(finger_num == 1){
		dist_th = SHTRPD_SPLIT_FINGER_REJECT_SINGLE_RANGE;
	}else{
		dist_th = SHTRPD_SPLIT_FINGER_REJECT_MULTI_RANGE;
	}
	
	for(i = 0;i < MAX_TOUCHES;i++){
		if(split_finger_reject[i].is_ghost){
			shtrpd_split_finger_reject_check_release(*finger_state_p, i, dist_th,
				((tmp_pre_finger_state & (1 << i)) != 0)? 0 : SHTRPD_SPLIT_FINGER_REJECT_MULTI_HYSTERESIS);
			
			if(split_finger_reject[i].is_ghost){
				*finger_state_p &= ~(1 << i);
				SHTRPD_LOG_DBG_SPLIT_FINGER_REJECT_PRINT("[%d] reject ghost\n", i);
			}
		}
	}
}
#endif /* SHTRPD_SPLIT_FINGER_REJECTION_ENABLE */

#if defined(SHTRPD_FAIL_TOUCH_CLEAR_ENABLE)
/* ------------------------------------------------------------------------- */
/* shtrpd_fail_touch_clear_init                                              */
/* ------------------------------------------------------------------------- */
static void shtrpd_fail_touch_clear_init(void)
{
	fail_touch_clear_reati_req = 0;
	memset(fail_touch_clear, 0, sizeof(fail_touch_clear));
}

/* ------------------------------------------------------------------------- */
/* shtrpd_fail_touch_clear_count_init                                        */
/* ------------------------------------------------------------------------- */
static void shtrpd_fail_touch_clear_count_init(void)
{
	memset(fail_touch_clear, 0, sizeof(fail_touch_clear));
}

/* ------------------------------------------------------------------------- */
/* shtrpd_fail_touch_clear_reati                                             */
/* ------------------------------------------------------------------------- */
static void shtrpd_fail_touch_clear_reati(struct shtrpd_ts_data *data)
{
	SHTRPD_LOG_DBG_FAIL_TOUCH_CLEAR_PRINT("Re-ATI execute\n");
	shtrpd_int_timing_check_reati();
	shtrpd_switch_event_mode(data, true);
}

/* ------------------------------------------------------------------------- */
/* shtrpd_fail_touch_clear_check                                             */
/* ------------------------------------------------------------------------- */
static void shtrpd_fail_touch_clear_check(struct shtrpd_ts_data *data, u8 *finger_state_p)
{
	int i;
	int x_min, x_max;
	
	if(!SHTRPD_FAIL_TOUCH_CLEAR){
		return;
	}
	
	x_max = (SHTRPD_FAIL_TOUCH_CLEAR_XPOS_BASE + SHTRPD_FAIL_TOUCH_CLEAR_XPOS_RANGE);
	x_min = ((SHTRPD_FAIL_TOUCH_CLEAR_XPOS_BASE - SHTRPD_FAIL_TOUCH_CLEAR_XPOS_RANGE) >= 0)? 
				(SHTRPD_FAIL_TOUCH_CLEAR_XPOS_BASE - SHTRPD_FAIL_TOUCH_CLEAR_XPOS_RANGE) : 0;
	
	for(i = 0;i < MAX_TOUCHES;i++){
		if(fail_touch_clear[i].count > 0){
			if(time_after(jiffies, fail_touch_clear[i].start_time + msecs_to_jiffies(SHTRPD_FAIL_TOUCH_CLEAR_REATI_TIME_TH)) != 0){
				fail_touch_clear[i].count = 0;
				SHTRPD_LOG_DBG_FAIL_TOUCH_CLEAR_PRINT("[%d] clear count by timeout\n", i);
			}
		}
		
		if((shtrpd_finger_fw_state & (1 << i)) != 0){
			if(fail_touch_clear[i].invalid_count > 0 || ((shtrpd_finger_fw_state_old) & (1 << i)) == 0){
				if((x_min <= shtrpd_touches[i].XPos && shtrpd_touches[i].XPos <= x_max) &&
					shtrpd_touches[i].touchStrength <= SHTRPD_FAIL_TOUCH_CLEAR_TS_TH &&
					shtrpd_touches[i].area <= SHTRPD_FAIL_TOUCH_CLEAR_AREA_TH)
				{
					fail_touch_clear[i].invalid_count++;
					SHTRPD_LOG_DBG_FAIL_TOUCH_CLEAR_PRINT("[%d] inc invalid_count(%d)\n", i, fail_touch_clear[i].invalid_count);
					
					if(fail_touch_clear[i].invalid_count > SHTRPD_FAIL_TOUCH_CLEAR_EVENTNUM_TH){
						fail_touch_clear[i].invalid_count = 0;
						SHTRPD_LOG_DBG_FAIL_TOUCH_CLEAR_PRINT("[%d] clear invalid_count by touch continue\n", i);
					}
				}else{
					fail_touch_clear[i].invalid_count = 0;
					SHTRPD_LOG_DBG_FAIL_TOUCH_CLEAR_PRINT("[%d] clear invalid_count by touch condition\n", i);
				}
			}
		}else{
			if(fail_touch_clear[i].invalid_count > 0 && 
				fail_touch_clear[i].count < SHTRPD_FAIL_TOUCH_CLEAR_REATI_COUNT_TH)
			{
				if(fail_touch_clear[i].count == 0){
					fail_touch_clear[i].start_time = jiffies;
					SHTRPD_LOG_DBG_FAIL_TOUCH_CLEAR_PRINT("[%d] start time check\n", i);
				}
				
				fail_touch_clear[i].count++;
				SHTRPD_LOG_DBG_FAIL_TOUCH_CLEAR_PRINT("[%d] inc fail touch detect count(%d)\n", 
					i, fail_touch_clear[i].count);

				if(fail_touch_clear[i].count >= SHTRPD_FAIL_TOUCH_CLEAR_REATI_COUNT_TH){
					fail_touch_clear_reati_req = 1;
					SHTRPD_LOG_DBG_FAIL_TOUCH_CLEAR_PRINT("[%d] set re-ati flag\n", i);
				}
			}
			fail_touch_clear[i].invalid_count = 0;
			SHTRPD_LOG_DBG_FAIL_TOUCH_CLEAR_PRINT("[%d] clear invalid_count by touch up\n", i);
		}
	
		if(fail_touch_clear[i].invalid_count){
			*finger_state_p &= ~(1 << i);
			SHTRPD_LOG_DBG_FAIL_TOUCH_CLEAR_PRINT("[%d] detect fail touch\n", i);
		}
	}
}
#endif /* SHTRPD_FAIL_TOUCH_CLEAR_ENABLE */

#ifdef SHTRPD_INTOUCH_GHOST_REJECTION_ENABLE
/* ------------------------------------------------------------------------- */
/* shtrpd_intouch_ghost_reject_init                                          */
/* ------------------------------------------------------------------------- */
static void shtrpd_intouch_ghost_reject_init(void)
{
    memset(intouch_ghost_reject, 0, sizeof(intouch_ghost_reject));
}

/* ------------------------------------------------------------------------- */
/* shtrpd_intouch_ghost_reject_get_z                                         */
/* ------------------------------------------------------------------------- */
static inline int shtrpd_intouch_ghost_reject_get_z(int ts, int area)
{
    ts   = (ts <= 0)? 1 : ts;
    area = (area <= 0)? 1 : 
           (area >= SHTRPD_INTOUCH_GHOST_REJECT_AREA_MAX)? SHTRPD_INTOUCH_GHOST_REJECT_AREA_MAX : area;
    
    return (ts / area);
}

/* ------------------------------------------------------------------------- */
/* shtrpd_intouch_ghost_reject_get_z                                         */
/* ------------------------------------------------------------------------- */
static inline int shtrpd_intouch_ghost_reject_check_zratio(int ghost_finger, int ghost_z)
{
    int i, z;
    
    for(i = 0;i < MAX_TOUCHES;i++){
        if(ghost_finger != i && (shtrpd_finger_fw_state & (1 << i)) != 0){
            z = (shtrpd_touches[i].touchStrength <= 0)? 1 : shtrpd_touches[i].touchStrength;
            SHTRPD_LOG_INTOUCH_GHOST_REJECT("[%d]/[%d] z=%d / %d, zratio=%d\n", ghost_finger, i, ghost_z, z, 
                                               ((ghost_z * 1000) / z) / 10);
            if(z > ghost_z){
                if(((ghost_z * 1000) / z) < SHTRPD_INTOUCH_GHOST_REJECT_ZRATIO_THRESH * 10){
                    return 1;
                }
            }
        }
    }
    
    return 0;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_intouch_ghost_reject_check                                         */
/* ------------------------------------------------------------------------- */
static void shtrpd_intouch_ghost_reject_check(u8 *finger_state_p)
{
    int i, ghost_z;
    
    if(!SHTRPD_INTOUCH_GHOST_REJECT_ENABLE){
        return;
    }
    
    for(i = 0;i < MAX_TOUCHES;i++){
        if(intouch_ghost_reject[i].is_ghost == 0){
            if((shtrpd_finger_fw_state & (1 << i)) != 0 && (shtrpd_finger_fw_state_old & (1 << i)) == 0){
                ghost_z = shtrpd_intouch_ghost_reject_get_z(shtrpd_touches[i].touchStrength, shtrpd_touches[i].area);

                if(ghost_z < SHTRPD_INTOUCH_GHOST_REJECT_Z_THRESH){
                    if(shtrpd_intouch_ghost_reject_check_zratio(i, ghost_z) != 0){
                        SHTRPD_LOG_INTOUCH_GHOST_REJECT("[%d] detect ghost\n", i);
                        intouch_ghost_reject[i].is_ghost = 1;
                        intouch_ghost_reject[i].td_x     = shtrpd_touches[i].XPos;
                        intouch_ghost_reject[i].td_y     = shtrpd_touches[i].YPos;
                    }
                }
            }
        }else{
            if((shtrpd_finger_fw_state & (1 << i)) == 0 ||
               shtrpd_touches[i].touchStrength >= SHTRPD_INTOUCH_GHOST_REJECT_Z_THRESH ||
               abs(intouch_ghost_reject[i].td_x - shtrpd_touches[i].XPos) >= SHTRPD_INTOUCH_GHOST_REJECT_MOVE_THRESH ||
               abs(intouch_ghost_reject[i].td_y - shtrpd_touches[i].YPos) >= SHTRPD_INTOUCH_GHOST_REJECT_MOVE_THRESH ||
               shtrpd_intouch_ghost_reject_check_zratio(i, shtrpd_intouch_ghost_reject_get_z(shtrpd_touches[i].touchStrength, shtrpd_touches[i].area)) == 0)
            {
                intouch_ghost_reject[i].is_ghost = 0;
                SHTRPD_LOG_INTOUCH_GHOST_REJECT("[%d] release ghost\n", i);
            }
        }
    }
    
    for(i = 0;i < MAX_TOUCHES;i++){
        if(intouch_ghost_reject[i].is_ghost != 0){
            *finger_state_p &= ~(1 << i);
            SHTRPD_LOG_INTOUCH_GHOST_REJECT("[%d] reject ghost\n", i);
        }
    }
}

#endif /* SHTRPD_INTOUCH_GHOST_REJECTION_ENABLE */

/* ------------------------------------------------------------------------- */
/* shtrpd_sys_enable_irq                                                     */
/* ------------------------------------------------------------------------- */
static void shtrpd_sys_enable_irq(void)
{
	unsigned long flags;

	spin_lock_irqsave(&shtrpd_spinlock, flags);

	if (shtrpd_irq_enable == SHTRPD_IRQ_DISABLED) {
		enable_irq(shtrpd_client->irq);
		shtrpd_irq_enable = SHTRPD_IRQ_ENABLED;
		SHTRPD_DBG("enable_irq(shtrpd_client->irq) \n");
	}

	spin_unlock_irqrestore(&shtrpd_spinlock, flags);

	return;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_sys_disable_irq                                                    */
/* ------------------------------------------------------------------------- */
static void shtrpd_sys_disable_irq(void)
{
	unsigned long flags;

	spin_lock_irqsave(&shtrpd_spinlock, flags);

	if (shtrpd_irq_enable == SHTRPD_IRQ_ENABLED) {
        disable_irq_nosync(shtrpd_client->irq);
		shtrpd_irq_enable = SHTRPD_IRQ_DISABLED;
		SHTRPD_DBG("disable_irq(shtrpd_client->irq) \n");
	}

	spin_unlock_irqrestore(&shtrpd_spinlock, flags);

	return;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_sys_enable_irq_wake                                                */
/* ------------------------------------------------------------------------- */
static void shtrpd_sys_enable_irq_wake(void)
{
	unsigned long flags;

	spin_lock_irqsave(&shtrpd_spinlock, flags);

	if (shtrpd_irq_enable_wake == SHTRPD_IRQ_DISABLED) {
		enable_irq_wake(shtrpd_client->irq);
		shtrpd_irq_enable_wake = SHTRPD_IRQ_ENABLED;
		SHTRPD_DBG("enable_irq_wake(shtrpd_client->irq) \n");
	}

	spin_unlock_irqrestore(&shtrpd_spinlock, flags);

	return;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_sys_disable_irq_wake                                               */
/* ------------------------------------------------------------------------- */
static void shtrpd_sys_disable_irq_wake(void)
{
	unsigned long flags;

	spin_lock_irqsave(&shtrpd_spinlock, flags);

	if (shtrpd_irq_enable_wake == SHTRPD_IRQ_ENABLED) {
		disable_irq_wake(shtrpd_client->irq);
		shtrpd_irq_enable_wake = SHTRPD_IRQ_DISABLED;
		SHTRPD_DBG("disable_irq_wake(shtrpd_client->irq) \n");
	}

	spin_unlock_irqrestore(&shtrpd_spinlock, flags);

	return;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_ts_phys_init                                                       */
/* ------------------------------------------------------------------------- */
/*
 *  The physical init is for first communication with the Trackpad Device
 *  The flags are reset here so as to indicate that the Trackpad Device
 *  needs to be setup again
 */
static void shtrpd_ts_phys_init(void)
{
    /*  Values used for the initial setup */
    initialSetup = true;
    setupCounter = 0;
    initialATI = false;
    initReseed = false;
    reset = 0;
    firstXYInfoRead = false;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_reset_setting                                                       */
/* ------------------------------------------------------------------------- */
static void shtrpd_reset_setting(void)
{
    setupCounter = 0;
    initialSetup = true;
    SHTRPD_WARN("shtrpd was Reset - Setting up...\n");
}

/* ------------------------------------------------------------------------- */
/* shtrpd_i2c_write_byte_data                                                */
/* ------------------------------------------------------------------------- */
/* write I2C  1 byte data  */
static int shtrpd_i2c_write_byte_data(const struct i2c_client *client, 
                    u8 command, u8 values)
{
    int err = 0;
    u32 i;

    SHTRPD_WARN("addr:0x%02x, data:0x%02x\n", command, values);

    for(i=0; i<SHTRPD_I2C_RTETRY_MAX; i++){
        mutex_lock(&shtrpd_i2c_lock);
        err = i2c_smbus_write_byte_data(client, command, values );
        mutex_unlock(&shtrpd_i2c_lock);
        if( err < 0){
            SHTRPD_ERR("i2c write error[%d](retry=%d) \n", err, i);
            SHTRPD_ERR("i2c write data=%02Xh %02Xh \n", command, values);
            usleep(SHTRPD_I2C_RETRY_DELAY);
            continue;
        }
        else{
            break;
        }
    }
    return err;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_i2c_write_block_data                                               */
/* ------------------------------------------------------------------------- */
/* write I2C  block data  */
static int shtrpd_i2c_write_block_data(const struct i2c_client *client, 
                    u8 command, int length, u8 *values)
{
    int err = 0;
    u32 i;

    SHTRPD_WARN("addr:0x%02x\n", command);
    for(i=0; i<length; i++){
        SHTRPD_WARN("data[%d]:0x%02x\n", i, values[i]);
    }

    for(i=0; i<SHTRPD_I2C_RTETRY_MAX; i++){
        mutex_lock(&shtrpd_i2c_lock);
        err = i2c_smbus_write_i2c_block_data(client, command, length, values );
        mutex_unlock(&shtrpd_i2c_lock);
        if( err < 0){
            SHTRPD_ERR("i2c write error[%d](retry=%d) \n", err, i);
            SHTRPD_ERR("i2c write cmd=%02Xh len=%d data=%02Xh \n", 
                                         command, length, *values);
            usleep(SHTRPD_I2C_RETRY_DELAY);
            continue;
        }
        else{
            break;
        }
    }
    return err;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_i2c_read_byte_data                                                */
/* ------------------------------------------------------------------------- */
/* read I2C data  */
static int shtrpd_i2c_read_byte_data(const struct i2c_client *client, 
                    u8 command, u8 *values)
{
    int err = 0;
    u32 i;


    for(i=0; i<SHTRPD_I2C_RTETRY_MAX; i++){
        mutex_lock(&shtrpd_i2c_lock);
        err = i2c_smbus_read_byte_data(client, command);
        mutex_unlock(&shtrpd_i2c_lock);
        if (err < 0) {
            /* Only for debugging purposes  */
            SHTRPD_ERR("i2c read err[%d] \n", err);
            SHTRPD_ERR("i2c read cmd=%02Xh \n", command);
            usleep(SHTRPD_I2C_RETRY_DELAY);
            continue;
        }
        else{
            *values = err;
            break;
        }
    }
    SHTRPD_WARN("addr:0x%02x, data:0x%02x\n", command, *values);
    return err;

}

/* ------------------------------------------------------------------------- */
/* shtrpd_i2c_read_block_data                                                */
/* ------------------------------------------------------------------------- */
/* read I2C data  */
static int shtrpd_i2c_read_block_data(const struct i2c_client *client, 
                    u8 command, int length, u8 *values)
{
    s32 i2c_ret = 0;
    u32 i;
    u8 data[256];
    u8 cmd = command;
    struct i2c_msg mesgs[] = {
        {
            .addr   = SHTRPD_ADDR,
            .flags  = 0,
            .len    = 1,
            .buf    = &cmd,
        },
        {
            .addr   = SHTRPD_ADDR,
            .flags  = I2C_M_RD,
            .len    = length,
            .buf    = &data[0],
        },
    };

    for(i=0; i<SHTRPD_I2C_RTETRY_MAX; i++){
        mutex_lock(&shtrpd_i2c_lock);
        i2c_ret = i2c_transfer(client->adapter, mesgs, 2);
        mutex_unlock(&shtrpd_i2c_lock);

        /* Only for debugging purposes  */
        if (i2c_ret <= 0){
            SHTRPD_ERR("read_block_data i2c_transfer err =%d \n", i2c_ret);
            SHTRPD_ERR("i2c read cmd=%02Xh len=%d \n", command, length);
            usleep(SHTRPD_I2C_RETRY_DELAY);
            continue;
        }
        else{
            memcpy(values, &data[0], length);
            break;
        }
    }
    if(i2c_ret == 0){
        i2c_ret = -1; 
    }else if(i2c_ret > 0){
        i2c_ret = 0;
    }

    SHTRPD_WARN("addr:0x%02x\n", command);
    for(i=0; i<length; i++){
        SHTRPD_WARN("data[%d]:0x%02x\n", i, values[i]);
    }
    return i2c_ret;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_i2c_user_write_byte_data                                           */
/* ------------------------------------------------------------------------- */
/* write I2C  1 byte data  */
#if 0
static int shtrpd_i2c_user_write_byte_data(const struct i2c_client *client, 
                    u8 command, u8 values)
{
    u32 i;

    for(i=0; i<SHTRPD_USER_I2C_RTETRY_MAX; i++){
        if(1 == userRequest.req_flag){
            SHTRPD_ERR("UserRequest duplicate error(write byte) retry=%d\n", i);
            usleep(SHTRPD_I2C_RETRY_DELAY);
            continue;
        }
        else{
            break;
        }
    }
    if(1 == userRequest.req_flag){
        SHTRPD_ERR("UserRequest duplicate error(write byte) retry over\n");
        return -1;
    }
    
    memset(&userRequest, 0, sizeof(userRequest));
    userRequest.read_write = SHTRPD_I2C_REQ_WRITE_BYTE;
    userRequest.client     = client;
    userRequest.cmd        = command;
    userRequest.values     = values;
    userRequest.req_flag   = 1;
    userRequest.ret        = -1;
    shtrpd_user_request();
    userRequest.req_flag = 0;
    return userRequest.ret;
}
#endif

/* ------------------------------------------------------------------------- */
/* shtrpd_i2c_user_write_block_data                                          */
/* ------------------------------------------------------------------------- */
/* write I2C  1 block data  */
static int shtrpd_i2c_user_write_block_data(const struct i2c_client *client, 
                    u8 command, int length, u8 *values)
{
    u32 i;

    for(i=0; i<SHTRPD_USER_I2C_RTETRY_MAX; i++){
        if(1 == userRequest.req_flag){
            SHTRPD_ERR("UserRequest duplicate error(write block) retry=%d\n", i);
            usleep(SHTRPD_I2C_RETRY_DELAY);
            continue;
        }
        else{
            break;
        }
    }
    if(1 == userRequest.req_flag){
        SHTRPD_ERR("UserRequest duplicate error(write block) retry over\n");
        return -1;
    }

    memset(&userRequest, 0, sizeof(userRequest));
    userRequest.read_write = SHTRPD_I2C_REQ_WRITE_BLOCK;
    userRequest.client     = client;
    userRequest.cmd        = command;
    userRequest.length     = length;
    userRequest.pbuf       = values;
    userRequest.req_flag   = 1;
    userRequest.ret        = -1;
    shtrpd_user_request();
    userRequest.req_flag = 0;
    return userRequest.ret;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_i2c_user_read_byte_data                                          */
/* ------------------------------------------------------------------------- */
/* read I2C  1 byte data  */
#if 0
static int shtrpd_i2c_user_read_byte_data(const struct i2c_client *client, 
                    u8 command, int length, u8 *values)
{
    u32 i;

    for(i=0; i<SHTRPD_USER_I2C_RTETRY_MAX; i++){
        if(1 == userRequest.req_flag){
            SHTRPD_ERR("UserRequest duplicate error(read byte) retry=%d\n", i);
            usleep(SHTRPD_I2C_RETRY_DELAY);
            continue;
        }
        else{
            break;
        }
    }
    if(1 == userRequest.req_flag){
        SHTRPD_ERR("UserRequest duplicate error(read byte) retry over\n");
        return -1;
    }

    memset(&userRequest, 0, sizeof(userRequest));
    userRequest.read_write = SHTRPD_I2C_REQ_READ_BYTE;
    userRequest.client     = client;
    userRequest.cmd        = command;
    userRequest.pbuf       = values;
    userRequest.req_flag   = 1;
    userRequest.ret        = -1;
    shtrpd_user_request();
    userRequest.req_flag = 0;
    return userRequest.ret;
}
#endif

/* ------------------------------------------------------------------------- */
/* shtrpd_i2c_user_read_block_data                                           */
/* ------------------------------------------------------------------------- */
/* read I2C  1 block data  */
static int shtrpd_i2c_user_read_block_data(const struct i2c_client *client, 
                    u8 command, int length, u8 *values)
{
    u32 i;

    for(i=0; i<SHTRPD_USER_I2C_RTETRY_MAX; i++){
        if(1 == userRequest.req_flag){
            SHTRPD_ERR("UserRequest duplicate error(read block) retry=%d\n", i);
            usleep(SHTRPD_I2C_RETRY_DELAY);
            continue;
        }
        else{
            break;
        }
    }
    if(1 == userRequest.req_flag){
        SHTRPD_ERR("UserRequest duplicate error(read block) retry over\n");
        return -1;
    }

    memset(&userRequest, 0, sizeof(userRequest));
    userRequest.read_write = SHTRPD_I2C_REQ_READ_BLOCK;
    userRequest.client     = client;
    userRequest.cmd        = command;
    userRequest.length     = length;
    userRequest.pbuf       = values;
    userRequest.req_flag   = 1;
    userRequest.ret        = -1;
    shtrpd_user_request();
    userRequest.req_flag = 0;
    return userRequest.ret;
}

/********   Trackpad Device specific setup is done here ********/

/* ------------------------------------------------------------------------- */
/* shtrpd_event_mode_handshake                                               */
/* ------------------------------------------------------------------------- */
/**
 *  A function to do a 'handshake' with the Trackpad device in order to
 *  initiate comms when the device is in event mode. The Trackpad device
 *  will return '0xA3' to indicate that it received a handshake
 *  and will now give a normal RDY window
 *  A handshake is performed by reading any byte from the
 *  Trackpad device
 */
static void shtrpd_event_mode_handshake(void)
{
#if 1
    u8 buffer[10];
    shtrpd_i2c_user_read_block_data(shtrpd_client, VERSION_INFO, 10, &buffer[0]);
#else
    u8 buffer;
    shtrpd_i2c_user_read_block_data(shtrpd_client, VERSION_INFO, 1, &buffer);
#endif
}

/* ------------------------------------------------------------------------- */
/* shtrpd_reati                                                              */
/* ------------------------------------------------------------------------- */
/*  Command for redoing ATI on the Trackpad Device */
static int shtrpd_reati(struct shtrpd_ts_data *data)
{
    int ret;
#if 0
    /*  Re-ATI  */
    ret = shtrpd_i2c_write_byte_data(data->client, CONTROL_SETTINGS,
        CONTROLSETTINGS0_VAL|AUTO_ATI);
    /*  Re-ATI  */
    ret = shtrpd_i2c_write_byte_data(data->client, CONTROL_SETTINGS,
        shtrpd_properties.controlsetting_val[0] | AUTO_ATI);
#else
    u8 data_buffer[11];

    data_buffer[0] = shtrpd_properties.controlsetting_val[0] | AUTO_ATI;
    /*  ControlSettings1    */
    if(shtrpd_touchcruiser_state){
        data_buffer[1] = shtrpd_properties.controlsetting_val[1];
    }else{
        data_buffer[1] = shtrpd_properties.controlsetting_val[1] | DIS_TOUCH_EVENT;
    }
    /*  ControlSettings2(add)    */
    if(shtrpd_atic_adj_disable)
        data_buffer[2] = shtrpd_properties.controlsetting_val[2] | ATIC_ADJ_DISABLE;
    else
        data_buffer[2] = shtrpd_properties.controlsetting_val[2];
    
    if(shtrpd_temp_drift)
        data_buffer[2] |= TEMP_DRIFT_MODE;
    
    /*  ControlSettings3(add)    */
    data_buffer[3] = shtrpd_properties.controlsetting_val[3];
    
    /*  ATI Settings Data   */
    data_buffer[4] = (unsigned char)(shtrpd_properties.atitarget_val>>8);
    /*  ATI Target  */
    data_buffer[5] = (unsigned char)shtrpd_properties.atitarget_val;
    /*  ATI C   */
    data_buffer[6] = shtrpd_properties.atic_val;
    data_buffer[7] = (unsigned char)(shtrpd_properties.atitarget2_val>>8);
    /*  Non-trackpad channels ATI Target    */
    data_buffer[8] = (unsigned char)shtrpd_properties.atitarget2_val;
    /*  Non-trackpad channels ATI C */
    data_buffer[9] = shtrpd_properties.atic2_val;
    data_buffer[10] = shtrpd_properties.re_ati_range;

    ret = shtrpd_i2c_write_block_data(data->client, CONTROL_SETTINGS,
        11, data_buffer);

#endif
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_reati_atisetting_data_clear                                        */
/* ------------------------------------------------------------------------- */
/*  Command for redoing ATI and data clear at ATI C Adjust[Rx0]-[Rx8] on the Trackpad Device */
static int shtrpd_reati_atisetting_data_clear(struct shtrpd_ts_data *data)
{
    int ret;
    int i = 0;
    u8 data_buffer[20];

    data_buffer[0] = shtrpd_properties.controlsetting_val[0] | AUTO_ATI;
    /*  ControlSettings1    */
    if(shtrpd_touchcruiser_state){
        data_buffer[1] = shtrpd_properties.controlsetting_val[1];
    }else{
        data_buffer[1] = shtrpd_properties.controlsetting_val[1] | DIS_TOUCH_EVENT;
    }
    /*  ControlSettings2(add)    */
    data_buffer[2] = shtrpd_properties.controlsetting_val[2];

    if(shtrpd_temp_drift)
        data_buffer[2] |= TEMP_DRIFT_MODE;
    
    /*  ControlSettings3(add)    */
    data_buffer[3] = shtrpd_properties.controlsetting_val[3];
    
    /*  ATI Settings Data   */
    data_buffer[4] = (unsigned char)(shtrpd_properties.atitarget_val>>8);
    /*  ATI Target  */
    data_buffer[5] = (unsigned char)shtrpd_properties.atitarget_val;
    /*  ATI C   */
    data_buffer[6] = shtrpd_properties.atic_val;
    data_buffer[7] = (unsigned char)(shtrpd_properties.atitarget2_val>>8);
    /*  Non-trackpad channels ATI Target    */
    data_buffer[8] = (unsigned char)shtrpd_properties.atitarget2_val;
    /*  Non-trackpad channels ATI C */
    data_buffer[9] = shtrpd_properties.atic2_val;
    data_buffer[10] = shtrpd_properties.re_ati_range;
    /*  ATI C Adjust[Rx0]-[Rx8] */
    for( i = 11; i < 20; i++){
        data_buffer[i] = 0;
    }

    ret = shtrpd_i2c_write_block_data(data->client, CONTROL_SETTINGS,
        20, data_buffer);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_control_settings                                                   */
/* ------------------------------------------------------------------------- */
/*  Send control settings to the Trackpad Device - Power modes, etc */
static int shtrpd_control_settings(struct shtrpd_ts_data *data)
{
    u8 data_buffer[4];
    int ret;

#if 0
    /* Control Settings Data Data   */
    /*  ControlSettings0 and clear ACK_RESET Bit    */
    data_buffer[0] = CONTROLSETTINGS0_VAL|SHOW_RESET;
    /*  ControlSettings1    */
    data_buffer[1] = CONTROLSETTINGS1_VAL;
#else
    /* Control Settings Data Data   */
    /*  ControlSettings0 and clear ACK_RESET Bit    */
    data_buffer[0] = shtrpd_properties.controlsetting_val[0] | SHOW_RESET;
    /*  ControlSettings1    */
    if(shtrpd_touchcruiser_state){
        data_buffer[1] = shtrpd_properties.controlsetting_val[1];
    }else{
        data_buffer[1] = shtrpd_properties.controlsetting_val[1] | DIS_TOUCH_EVENT;
    }
    /*  ControlSettings2(add)    */
    if(shtrpd_atic_adj_disable)
        data_buffer[2] = shtrpd_properties.controlsetting_val[2] | ATIC_ADJ_DISABLE;
    else
        data_buffer[2] = shtrpd_properties.controlsetting_val[2];

    if(shtrpd_temp_drift)
        data_buffer[2] |= TEMP_DRIFT_MODE;
    
    /*  ControlSettings3(add)    */
    data_buffer[3] = shtrpd_properties.controlsetting_val[3];

#endif
    /*  Write the 2 control settings bytes  */
    ret = shtrpd_i2c_write_block_data(data->client, CONTROL_SETTINGS,
        4, data_buffer);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_switch_event_mode                                                  */
/* ------------------------------------------------------------------------- */
/*  Set the Trackpad Device to Event Mode or Streaming mode */
static int shtrpd_switch_event_mode(struct shtrpd_ts_data *data, bool event)
{
    u8 data_buffer[4];
    int ret;

#if 0
    /*  If we need to switch to event mode, OR the bit with existing
     *  register data
     */
    if (event)
        data_byte = CONTROLSETTINGS0_VAL|EVENT_MODE;
    /*  If we need to go to streaming data,
     *  just send register data
     */
    else
        data_byte = CONTROLSETTINGS0_VAL;
#else
    /*  If we need to switch to event mode, OR the bit with existing
     *  register data
     */
    if (event) {
        /*  ControlSettings0 */
        data_buffer[0] = shtrpd_properties.controlsetting_val[0] | EVENT_MODE | AUTO_MODES;
        /*  ControlSettings1    */
        if(shtrpd_touchcruiser_state){
            data_buffer[1] = shtrpd_properties.controlsetting_val[1] | LOW_POWER;
        }else{
            data_buffer[1] = shtrpd_properties.controlsetting_val[1] | DIS_TOUCH_EVENT | LOW_POWER;
        }
        /*  ControlSettings2(add)    */
        if(shtrpd_atic_adj_disable)
            data_buffer[2] = shtrpd_properties.controlsetting_val[2] | ATIC_ADJ_DISABLE | PM_RE_ATI | NM_RE_ATI;
        else
            data_buffer[2] = shtrpd_properties.controlsetting_val[2] | PM_RE_ATI | NM_RE_ATI;

        if(shtrpd_temp_drift)
            data_buffer[2] |= TEMP_DRIFT_MODE;
        
        /*  ControlSettings3(add)    */
        data_buffer[3] = shtrpd_properties.controlsetting_val[3];

    /*  If we need to go to streaming data,
     *  just send register data
     */
    } else {
        /*  ControlSettings0 */
        data_buffer[0] = shtrpd_properties.controlsetting_val[0] | AUTO_MODES;
        /*  ControlSettings1    */
        if(shtrpd_touchcruiser_state){
            data_buffer[1] = shtrpd_properties.controlsetting_val[1] | LOW_POWER;
        }else{
            data_buffer[1] = shtrpd_properties.controlsetting_val[1] | DIS_TOUCH_EVENT | LOW_POWER;
        }
        /*  ControlSettings2(add)    */
        if(shtrpd_atic_adj_disable)
            data_buffer[2] = shtrpd_properties.controlsetting_val[2] | ATIC_ADJ_DISABLE | PM_RE_ATI | NM_RE_ATI;
        else
            data_buffer[2] = shtrpd_properties.controlsetting_val[2] | PM_RE_ATI | NM_RE_ATI;

        if(shtrpd_temp_drift)
            data_buffer[2] |= TEMP_DRIFT_MODE;
        
        /*  ControlSettings3(add)    */
        data_buffer[3] = shtrpd_properties.controlsetting_val[3];
    }
#endif
    /*  Switch Event/Streaming Mode */
    ret = shtrpd_i2c_write_block_data(data->client, CONTROL_SETTINGS,
        4, data_buffer);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_channel_setup                                                      */
/* ------------------------------------------------------------------------- */
/*  Setup the total Rx and Tx Trackpad device */
static int shtrpd_channel_setup(struct shtrpd_ts_data *data)
{
    u8 data_buffer[7];
    int ret;

#if 0
    /* ChannelSetup Data    */
    data_buffer[0] = TOTALRXS_VAL;      /*  TotalRx */
    data_buffer[1] = TOTALTXS_VAL;      /*  TotalTx */
    data_buffer[2] = TRACKPADRXS_VAL;   /*  TrackPadRx  */
    data_buffer[3] = TRACKPADTXS_VAL;   /*  TrackPadTx  */
    /*  PMSetup register    */
    data_buffer[4] = PMSETUP0_VAL;
    data_buffer[5] = TXHIGH_VAL;
    /*
     *  Which Tx's are used for the
     *  ProxMode channel (projected only)
     */
    data_buffer[6] = TXLOW_VAL;
#else
    /* ChannelSetup Data    */
    data_buffer[0] = shtrpd_properties.totalrxs_val;      /*  TotalRx */
    data_buffer[1] = shtrpd_properties.totaltxs_val;      /*  TotalTx */
    data_buffer[2] = shtrpd_properties.trackpadrxs_val;   /*  TrackPadRx  */
    data_buffer[3] = shtrpd_properties.trackpadtxs_val;   /*  TrackPadTx  */
    /*  PMSetup register    */
    data_buffer[4] = shtrpd_properties.pmsetup0_val;
    data_buffer[5] = shtrpd_properties.txhigh_val;
    /*
     *  Which Tx's are used for the
     *  ProxMode channel (projected only)
     */
    data_buffer[6] = shtrpd_properties.txlow_val;
#endif

    ret = shtrpd_i2c_write_block_data(data->client, CHANNEL_SETUP,
        7, data_buffer);
    return ret;
}

#ifndef SHTRPD_CHECK_CPU_TEMP_ENABLE
/* ------------------------------------------------------------------------- */
/* shtrpd_thresholds                                                         */
/* ------------------------------------------------------------------------- */
/*  Setup the touch Thresholds for each channel */
static int shtrpd_thresholds(struct shtrpd_ts_data *data)
{
    u8 data_buffer[10];
    int ret;

#if 0
    /*  Prox Threshold  */
    data_buffer[0] = PROXTHRESHOLD_VAL;
    /*  Touch Multiplier    */
    data_buffer[1] = TOUCHMULTIPLIER_VAL;
    /*  Touch Shifter   */
    data_buffer[2] = TOUCHSHIFTER_VAL;
    /*  PM Prox Threshold   */
    data_buffer[3] = PMPROXTHRESHOLD_VAL;
    /*  Snap threshold  */
    data_buffer[4] = (unsigned char)(SNAPTHRESHOLD_VAL>>8);
    /*  Snap threshold  */
    data_buffer[5] = (unsigned char)SNAPTHRESHOLD_VAL;
    /*  Non-trackpad channels prox threshold    */
    data_buffer[6] = PROXTHRESHOLD2_VAL;
    /*  Non-trackpad channels Touch Multiplier  */
    data_buffer[7] = TOUCHMULTIPLIER2_VAL;
    /*  Non-trackpad channels Touch Shifter */
    data_buffer[8] = TOUCHSHIFTER2_VAL;
    /*  (in-touch) Touch Threshold Multiplier */
    data_buffer[9] = INTOUCHMULTIPLIER_VAL;
#else
    /*  Prox Threshold  */
    data_buffer[0] = shtrpd_properties.proxthreshold_val;
    /*  Touch Multiplier    */
    data_buffer[1] = shtrpd_touchmultiplier_val;
    /*  Touch Shifter   */
    data_buffer[2] = shtrpd_touchshifter_val;
    /*  PM Prox Threshold   */
    data_buffer[3] = shtrpd_properties.pmproxthreshold_val;
    /*  Snap threshold  */
    data_buffer[4] = (unsigned char)(shtrpd_snapthreshold_val>>8);
    /*  Snap threshold  */
    data_buffer[5] = (unsigned char)shtrpd_snapthreshold_val;
    /*  Non-trackpad channels prox threshold    */
    data_buffer[6] = shtrpd_properties.proxthreshold2_val;
    /*  Non-trackpad channels Touch Multiplier  */
    data_buffer[7] = shtrpd_touchmultiplier2_val;
    /*  Non-trackpad channels Touch Shifter */
    data_buffer[8] = shtrpd_touchshifter2_val;
    /*  (in-touch) Touch Threshold Multiplier */
	data_buffer[9] = shtrpd_intouchmultiplier_val;
#endif

    ret = shtrpd_i2c_write_block_data(data->client, THRESHOLD_SETTINGS,
        10, data_buffer);
    return ret;
}
#endif /* SHTRPD_CHECK_CPU_TEMP_ENABLE */

/* ------------------------------------------------------------------------- */
/* shtrpd_ati_settings                                                       */
/* ------------------------------------------------------------------------- */
/*  Setup the ATI settings for each channel */
static int shtrpd_ati_settings(struct shtrpd_ts_data *data)
{
    u8 data_buffer[7];
    int ret;

#if 0
    /*  ATI Settings Data   */
    data_buffer[0] = (unsigned char)(ATITARGET_VAL>>8);
    /*  ATI Target  */
    data_buffer[1] = (unsigned char)ATITARGET_VAL;
    /*  ATI C   */
    data_buffer[2] = ATIC_VAL;
    data_buffer[3] = (unsigned char)(ATITARGET2_VAL>>8);
    /*  Non-trackpad channels ATI Target    */
    data_buffer[4] = (unsigned char)ATITARGET2_VAL;
    /*  Non-trackpad channels ATI C */
    data_buffer[5] = ATIC2_VAL;
    data_buffer[6] = RE_ATI_RANGE;
#else
    /*  ATI Settings Data   */
    data_buffer[0] = (unsigned char)(shtrpd_properties.atitarget_val>>8);
    /*  ATI Target  */
    data_buffer[1] = (unsigned char)shtrpd_properties.atitarget_val;
    /*  ATI C   */
    data_buffer[2] = shtrpd_properties.atic_val;
    data_buffer[3] = (unsigned char)(shtrpd_properties.atitarget2_val>>8);
    /*  Non-trackpad channels ATI Target    */
    data_buffer[4] = (unsigned char)shtrpd_properties.atitarget2_val;
    /*  Non-trackpad channels ATI C */
    data_buffer[5] = shtrpd_properties.atic2_val;
    data_buffer[6] = shtrpd_properties.re_ati_range;
#endif

    /*  NOTE: after the setup we need to do the ATI */
    ret = shtrpd_i2c_write_block_data(data->client, ATI_SETTINGS,
        7, data_buffer);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_filter_settings                                                    */
/* ------------------------------------------------------------------------- */
/*  Setup the Filter settings for each channel  */
static int shtrpd_filter_settings(struct shtrpd_ts_data *data)
{
    u8 data_buffer[4];
    int ret;

#if 0
    /*  Filter Settings Data    */
    /*  Numerous filter settings    */
    data_buffer[0] = FILTERSETTINGS0_VAL;
    /*  XY touch point filtering parameter  */
    data_buffer[1] = TOUCHDAMPING_VAL;
    /*  ProxMode count value filter parameter (full-speed)  */
    data_buffer[2] = PMCOUNTDAMPING_VAL;
    /*  ProxMode count value filter parameter (Low Power)   */
    data_buffer[3] = LPPMCOUNTDAMPING_VAL;
#else
    /*  Filter Settings Data    */
    /*  Numerous filter settings    */
    data_buffer[0] = shtrpd_properties.filtersettings0_val;
    /*  XY touch point filtering parameter  */
    data_buffer[1] = shtrpd_properties.touchdamping_val;
    /*  ProxMode count value filter parameter (full-speed)  */
    data_buffer[2] = shtrpd_properties.pmcountdamping_val;
    /*  ProxMode count value filter parameter (Low Power)   */
    data_buffer[3] = shtrpd_properties.lppmcountdamping_val;
#endif
    ret = shtrpd_i2c_write_block_data(data->client, FILTER_SETTINGS,
        4, data_buffer);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_timing_settings                                                    */
/* ------------------------------------------------------------------------- */
/**
 *  Setup the Timing settings for each channel
 *  Low Power Time helps with power consumption when no Prox or Touch
 *  is detected.
 *  At the moment a LP time of 160ms is implemented
 */
static int shtrpd_timing_settings(struct shtrpd_ts_data *data)
{
    u8 data_buffer[6];
    int ret;

#if 0
    /*  Timing Settings Data    */
    /*  LTA reseed timer    */
    data_buffer[0] = RESEEDTIME_VAL;
    /*  Inactive i2c timeout value  */
    data_buffer[1] = COMMSTIMEOUT_VAL;
    /*  Mode timer value (Switching between modes time) */
    data_buffer[2] = MODETIME_VAL;
    /*  Low power time added in low-power state */
    data_buffer[3] = LPTIME_VAL;
    /*  Sleep time permanently added    */
    data_buffer[4] = SLEEPTIME_VAL;
    data_buffer[5] = PMRESEED_TIME;
#else
    /*  Timing Settings Data    */
    /*  LTA reseed timer    */
    data_buffer[0] = shtrpd_properties.reseedtime_val;
    /*  Inactive i2c timeout value  */
    data_buffer[1] = shtrpd_properties.commstimeout_val;
    /*  Mode timer value (Switching between modes time) */
    data_buffer[2] = shtrpd_properties.modetime_val;
    /*  Low power time added in low-power state */
    data_buffer[3] = shtrpd_properties.lptime_val;
    /*  Sleep time permanently added    */
    data_buffer[4] = shtrpd_properties.sleeptime_val;
    data_buffer[5] = shtrpd_properties.pmreseedtime;
#endif

    /*  Send Timing settings to Trackpad Device */
    ret = shtrpd_i2c_write_block_data(data->client, TIMING_SETTINGS,
        6, data_buffer);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_hardware_settings                                                  */
/* ------------------------------------------------------------------------- */
/*  Setup the Hardware settings for each channel    */
static int shtrpd_hardware_settings(struct shtrpd_ts_data *data)
{
    u8 data_buffer[4];
    int ret;

#if 0
    /*  Hardware Config Settings Data   */
    data_buffer[0] = PROXSETTINGS0_VAL;
    data_buffer[1] = PROXSETTINGS1_VAL;
    data_buffer[2] = PROXSETTINGS2_VAL;
    data_buffer[3] = PROXSETTINGS3_VAL;
#else
    /*  Hardware Config Settings Data   */
    data_buffer[0] = shtrpd_properties.proxsettings_val[0];
    data_buffer[1] = shtrpd_properties.proxsettings_val[1];
    data_buffer[2] = shtrpd_properties.proxsettings_val[2];
    data_buffer[3] = shtrpd_properties.proxsettings_val[3];
#endif

    /*  Send Hardware Settings to Trackpad Device   */
    ret = shtrpd_i2c_write_block_data(data->client, HW_CONFIG_SETTINGS,
        4, data_buffer);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_active_channel_settings                                            */
/* ------------------------------------------------------------------------- */
/*  Setup the Active channel settings   */
static int shtrpd_active_channel_settings(struct shtrpd_ts_data *data)
{
    u8 data_buffer[30];
    u8 i;
    int ret;

#if 0
    /*  Active Channels Data    */
    data_buffer[0] = (unsigned char)(ACTIVECHANNELS0_VAL>>8);
    data_buffer[1] = (unsigned char)ACTIVECHANNELS0_VAL;
    data_buffer[2] = (unsigned char)(ACTIVECHANNELS1_VAL>>8);
    data_buffer[3] = (unsigned char)ACTIVECHANNELS1_VAL;
    data_buffer[4] = (unsigned char)(ACTIVECHANNELS2_VAL>>8);
    data_buffer[5] = (unsigned char)ACTIVECHANNELS2_VAL;
    data_buffer[6] = (unsigned char)(ACTIVECHANNELS3_VAL>>8);
    data_buffer[7] = (unsigned char)ACTIVECHANNELS3_VAL;
    data_buffer[8] = (unsigned char)(ACTIVECHANNELS4_VAL>>8);
    data_buffer[9] = (unsigned char)ACTIVECHANNELS4_VAL;
    data_buffer[10] = (unsigned char)(ACTIVECHANNELS5_VAL>>8);
    data_buffer[11] = (unsigned char)ACTIVECHANNELS5_VAL;
    data_buffer[12] = (unsigned char)(ACTIVECHANNELS6_VAL>>8);
    data_buffer[13] = (unsigned char)ACTIVECHANNELS6_VAL;
    data_buffer[14] = (unsigned char)(ACTIVECHANNELS7_VAL>>8);
    data_buffer[15] = (unsigned char)ACTIVECHANNELS7_VAL;
    data_buffer[16] = (unsigned char)(ACTIVECHANNELS8_VAL>>8);
    data_buffer[17] = (unsigned char)ACTIVECHANNELS8_VAL;
    data_buffer[18] = (unsigned char)(ACTIVECHANNELS9_VAL>>8);
    data_buffer[19] = (unsigned char)ACTIVECHANNELS9_VAL;
    data_buffer[20] = (unsigned char)(ACTIVECHANNELS10_VAL>>8);
    data_buffer[21] = (unsigned char)ACTIVECHANNELS10_VAL;
    data_buffer[22] = (unsigned char)(ACTIVECHANNELS11_VAL>>8);
    data_buffer[23] = (unsigned char)ACTIVECHANNELS11_VAL;
    data_buffer[24] = (unsigned char)(ACTIVECHANNELS12_VAL>>8);
    data_buffer[25] = (unsigned char)ACTIVECHANNELS12_VAL;
    data_buffer[26] = (unsigned char)(ACTIVECHANNELS13_VAL>>8);
    data_buffer[27] = (unsigned char)ACTIVECHANNELS13_VAL;
    data_buffer[28] = (unsigned char)(ACTIVECHANNELS14_VAL>>8);
    data_buffer[29] = (unsigned char)ACTIVECHANNELS14_VAL;
#else
    for(i=0;i<15;i++){
        data_buffer[i*2]     =(u8)(shtrpd_properties.activechannels_val[i]>>8);
        data_buffer[(i*2)+1] =(u8) shtrpd_properties.activechannels_val[i];
    }

#endif
    /*  Send the Active channels data to the TrackPad Device    */
    ret = shtrpd_i2c_write_block_data(data->client, ACTIVE_CHANNELS,
        30, data_buffer);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_debounce_settings                                                  */
/* ------------------------------------------------------------------------- */
/*  Setup the Debounce settings for each channel    */
static int shtrpd_debounce_settings(struct shtrpd_ts_data *data)
{
    u8 data_buffer[3];
    int ret;

#if 0
    /*  DebounceSettings Data   */
    data_buffer[0] = PROXDB_VAL;    /*  Prox Debounce values    */
    data_buffer[1] = TOUCHSNAPDB_VAL;
#else
    /*  DebounceSettings Data   */
    data_buffer[0] = shtrpd_properties.proxdb_val;   /* Prox Debounce values*/
    data_buffer[1] = shtrpd_properties.touchsnapdb_val;
    data_buffer[2] = shtrpd_properties.touchsnapdb2_val;
#endif

    /*  Send Debounce Settings  */
    ret = shtrpd_i2c_write_block_data(data->client, DB_SETTINGS,
        3, data_buffer);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_pm_ati_settings                                                    */
/* ------------------------------------------------------------------------- */
static int shtrpd_pm_ati_settings(struct shtrpd_ts_data *data)
{
    u8 data_buffer[3];
    int ret;

    /*  PM ATI Target High */
    data_buffer[0] = (unsigned char)(shtrpd_properties.pmatitarget_val>>8);
    /*  PM ATI Target Low  */
    data_buffer[1] = (unsigned char)shtrpd_properties.pmatitarget_val;
    /*  PM ATIC   */
    data_buffer[2] = shtrpd_properties.pmatic_val;

    /*  Send Debounce Settings  */
    ret = shtrpd_i2c_write_block_data(data->client, PM_ATI_SETTINGS,
        3, data_buffer);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_change_prox_mode                                                   */
/* ------------------------------------------------------------------------- */
static int shtrpd_change_prox_mode(struct shtrpd_ts_data *data)
{
    int ret;

    // Change mode to PM (set ControlSettings to PM)
    ret = shtrpd_i2c_write_byte_data(data->client, CONTROL_SETTINGS,
                shtrpd_properties.controlsetting_val[0] | MODE_SELECT);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_change_normal_mode                                                 */
/* ------------------------------------------------------------------------- */
static int shtrpd_change_normal_mode(struct shtrpd_ts_data *data)
{
    int ret;

    // change back to NM (optional, maybe not necessary)
    ret = shtrpd_i2c_write_byte_data(data->client, CONTROL_SETTINGS,
                shtrpd_properties.controlsetting_val[0]);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_reati_temp_drift_settings                                          */
/* ------------------------------------------------------------------------- */
static int shtrpd_reati_temp_drift_settings(struct shtrpd_ts_data *data)
{
    u8 data_buffer[3];
    int ret;

    if(shtrpd_properties.temp_non_detect_avg   == 0 &&
       shtrpd_properties.temp_min_delta        == 0 &&
       shtrpd_properties.temp_lta_adj_interval == 0)
    {
        return 0;
    }
    
    /*  Number of channels showing drift that triggers Re-ATI counter */
    data_buffer[0] = shtrpd_properties.temp_non_detect_avg;
    /*  Length of time before Re-ATI */
    data_buffer[1] = shtrpd_properties.temp_min_delta;
    /*  Delta of variation allowed */
    data_buffer[2] = shtrpd_properties.temp_lta_adj_interval;

    /* Set Parameters for Re-ATI from temperature drift */
    ret = shtrpd_i2c_write_block_data(data->client, REATI_TEMP_DRIFT,
        3, data_buffer);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_initialSetup                                                       */
/* ------------------------------------------------------------------------- */
static void shtrpd_properties_init(void)
{
    memset(&shtrpd_properties, 0, sizeof(shtrpd_properties));

    shtrpd_properties.controlsetting_val[0] = CONTROLSETTINGS0_VAL;
    shtrpd_properties.controlsetting_val[1] = CONTROLSETTINGS1_VAL;
    shtrpd_properties.controlsetting_val[2] = CONTROLSETTINGS2_VAL;
    shtrpd_properties.controlsetting_val[3] = CONTROLSETTINGS3_VAL;

    shtrpd_properties.proxthreshold_val    = PROXTHRESHOLD_VAL;
    shtrpd_properties.touchmultiplier_val  = TOUCHMULTIPLIER_VAL;
    shtrpd_properties.touchshifter_val     = TOUCHSHIFTER_VAL;
    shtrpd_properties.pmproxthreshold_val  = PMPROXTHRESHOLD_VAL;
    shtrpd_properties.snapthreshold_val    = SNAPTHRESHOLD_VAL;
    shtrpd_properties.proxthreshold2_val   = PROXTHRESHOLD2_VAL;
    shtrpd_properties.touchmultiplier2_val = TOUCHMULTIPLIER2_VAL;
    shtrpd_properties.touchshifter2_val    = TOUCHSHIFTER2_VAL;
    shtrpd_properties.intouchmultiplier_val= INTOUCHMULTIPLIER_VAL;

    shtrpd_properties.atitarget_val  = ATITARGET_VAL;
    shtrpd_properties.atic_val       = ATIC_VAL;
    shtrpd_properties.atitarget2_val = ATITARGET2_VAL;
    shtrpd_properties.atic2_val      = ATIC2_VAL;
    shtrpd_properties.re_ati_range   = RE_ATI_RANGE;

    shtrpd_properties.filtersettings0_val  = FILTERSETTINGS0_VAL;
    shtrpd_properties.touchdamping_val     = TOUCHDAMPING_VAL;
    shtrpd_properties.pmcountdamping_val   = PMCOUNTDAMPING_VAL;
    shtrpd_properties.lppmcountdamping_val = LPPMCOUNTDAMPING_VAL;

    shtrpd_properties.reseedtime_val   = RESEEDTIME_VAL;
    shtrpd_properties.commstimeout_val = COMMSTIMEOUT_VAL;
    shtrpd_properties.modetime_val     = MODETIME_VAL;
    shtrpd_properties.lptime_val       = LPTIME_VAL;
    shtrpd_properties.sleeptime_val    = SLEEPTIME_VAL;
    shtrpd_properties.pmreseedtime     = PMRESEED_TIME;

    shtrpd_properties.totalrxs_val     = TOTALRXS_VAL;
    shtrpd_properties.totaltxs_val     = TOTALTXS_VAL;
    shtrpd_properties.trackpadrxs_val  = TRACKPADRXS_VAL;
    shtrpd_properties.trackpadtxs_val  = TRACKPADTXS_VAL;
    shtrpd_properties.pmsetup0_val     = PMSETUP0_VAL;
    shtrpd_properties.txhigh_val       = TXHIGH_VAL;
    shtrpd_properties.txlow_val        = TXLOW_VAL;

    shtrpd_properties.proxsettings_val[0] = PROXSETTINGS0_VAL;
    shtrpd_properties.proxsettings_val[1] = PROXSETTINGS1_VAL;
    shtrpd_properties.proxsettings_val[2] = PROXSETTINGS2_VAL;
    shtrpd_properties.proxsettings_val[3] = PROXSETTINGS3_VAL;

    shtrpd_properties.activechannels_val[0] = ACTIVECHANNELS0_VAL;
    shtrpd_properties.activechannels_val[1] = ACTIVECHANNELS1_VAL;
    shtrpd_properties.activechannels_val[2] = ACTIVECHANNELS2_VAL;
    shtrpd_properties.activechannels_val[3] = ACTIVECHANNELS3_VAL;
    shtrpd_properties.activechannels_val[4] = ACTIVECHANNELS4_VAL;
    shtrpd_properties.activechannels_val[5] = ACTIVECHANNELS5_VAL;
    shtrpd_properties.activechannels_val[6] = ACTIVECHANNELS6_VAL;
    shtrpd_properties.activechannels_val[7] = ACTIVECHANNELS7_VAL;
    shtrpd_properties.activechannels_val[8] = ACTIVECHANNELS8_VAL;
    shtrpd_properties.activechannels_val[9] = ACTIVECHANNELS9_VAL;
    shtrpd_properties.activechannels_val[10] = ACTIVECHANNELS10_VAL;
    shtrpd_properties.activechannels_val[11] = ACTIVECHANNELS11_VAL;
    shtrpd_properties.activechannels_val[12] = ACTIVECHANNELS12_VAL;
    shtrpd_properties.activechannels_val[13] = ACTIVECHANNELS13_VAL;
    shtrpd_properties.activechannels_val[14] = ACTIVECHANNELS14_VAL;

    shtrpd_properties.proxdb_val       = PROXDB_VAL;
    shtrpd_properties.touchsnapdb_val  = TOUCHSNAPDB_VAL;
    shtrpd_properties.touchsnapdb2_val = TOUCHSNAPDB2_VAL;

    shtrpd_properties.pmatitarget_val = PMATITARGET_VAL;
    shtrpd_properties.pmatic_val      = PMATIC_VAL;
    shtrpd_properties.pm_re_ati_range = PM_RE_ATI_RANGE;

    shtrpd_properties.temp_non_detect_avg   = TEMP_NON_DETECT_AVG;
    shtrpd_properties.temp_min_delta        = TEMP_MIN_DELTA;
    shtrpd_properties.temp_lta_adj_interval = TEMP_LTA_ADJ_INTERVAL;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_initialSetup                                                       */
/* ------------------------------------------------------------------------- */
/* Do the initial setup for the Trackpad device
 * Because we have to wait for the RDY from the Trackpad device
 * the setup will take a few comms cycles
 */
static void shtrpd_initialSetup( void *dev_id )
{
    struct shtrpd_ts_data *data = dev_id;
    int ret;

    SHTRPD_INFO("setupCounter=%d\n", setupCounter);

    switch (setupCounter) {
    /*  Setup ControlSettings   */
    case 0:
        ret = shtrpd_control_settings(data);
        if(ret >= 0){
            setupCounter++;
        }else{
            goto fail;
        }
        break;
    case 1:
        ret = shtrpd_channel_setup(data);
        if(ret >= 0){
            setupCounter++;
        }else{
            goto fail;
        }
        break;
    /*  Setup Thresholds    */
    case 2:
#if defined( SHTRPD_CHECK_CPU_TEMP_ENABLE )
        if(SHTRPD_CHECK_CPU_TEMP){
	        SHTRPD_LOG_CHECK_CPU_TEMP("cpu temp check timer start\n");
            cancel_delayed_work(&shtrpd_work_cpu_temp);
            queue_delayed_work(shtrpd_work_queue, &shtrpd_work_cpu_temp, msecs_to_jiffies(SHTRPD_CHECK_CPU_TEMP_INTERVAL));
        }
        ret = shtrpd_thresholds_check_cpu_temp(1);
        if(ret >= 0){
            setupCounter++;
        }else{
            goto fail;
        }
        break;
#else /* SHTRPD_CHECK_CPU_TEMP_ENABLE */
        ret = shtrpd_thresholds(data);
        if(ret >= 0){
            setupCounter++;
        }else{
            goto fail;
        }
        break;
#endif /* SHTRPD_CHECK_CPU_TEMP_ENABLE */
    /*  Setup ATI Settings  */
    case 3:
        ret = shtrpd_ati_settings(data);
        if(ret >= 0){
            setupCounter++;
        }else{
            goto fail;
        }
        break;
    /*  Redo ATI    */
    case 4:
        ret = shtrpd_reati(data);
        if(ret >= 0){
            setupCounter++;
        }else{
            goto fail;
        }
        break;
    /*  PM ATI    */
    case 5:
        ret = shtrpd_pm_ati_settings(data);
        if(ret >= 0){
            setupCounter++;
        }else{
            goto fail;
        }
        break;
    /* Prox Mode */
    case 6: 
        ret = shtrpd_change_prox_mode(data);
        if(ret >= 0){
            setupCounter++;
        }else{
            goto fail;
        }
        break;
    /*  Redo ATI    */
    case 7:
        ret = shtrpd_reati(data);
        if(ret >= 0){
            setupCounter++;
        }else{
            goto fail;
        }
        break;
    /* Normal Mode */
    case 8: 
        ret = shtrpd_change_normal_mode(data);
        if(ret >= 0){
            setupCounter++;
        }else{
            goto fail;
        }
        break;
    /*  Filter Settings */
    case 9:
        ret = shtrpd_filter_settings(data);
        if(ret >= 0){
            setupCounter++;
        }else{
            goto fail;
        }
        break;
    /*  Setup Timings   */
    case 10:
        ret = shtrpd_timing_settings(data);
        if(ret >= 0){
            setupCounter++;
        }else{
            goto fail;
        }
        break;
    /*  Setup Hardware Settings */
    case 11:
        ret = shtrpd_hardware_settings(data);
        if(ret >= 0){
            setupCounter++;
        }else{
            goto fail;
        }
        break;
    /*  Setup Active Channels   */
    case 12:
        ret = shtrpd_active_channel_settings(data);
        if(ret >= 0){
            setupCounter++;
        }else{
            goto fail;
        }
        break;
    /*  Debounce Settings   */
    case 13:
        ret = shtrpd_debounce_settings(data);
        if(ret >= 0){
            setupCounter++;
        }else{
            goto fail;
        }
        break;
    /*  Do an initial ATI before switching
     *  to Event mode
     */
    case 14:
        //ret = shtrpd_reati(data);
        ret = shtrpd_reati_atisetting_data_clear(data);
        if(ret >= 0){
            usleep(shtrpd_atic_adj_wait);
            /* Re-ATI   */
            initialATI = true;
            setupCounter++;
        }else{
            goto fail;
        }
        break;
    /*  Now setup is done, switch to Event Mode */
    case 15:
        ret = shtrpd_switch_event_mode(data, true);
        if(ret >= 0){

            usleep(SHTRPD_INITIAL_DOWN_DELAY);

            ret = shtrpd_reati_temp_drift_settings(data);
            if(ret < 0){
                goto fail;
            }

            /*  Setup is now done   */
            setupCounter = 0;
            initialSetup = false;

            noOfTouches_old = 0;
            finger_enable = 0;
            old_output_slot_no = -1;

            SHTRPD_INFO("shtrpd Driver Setup Done!\n");
        }else{
            goto fail;
        }
        break;
    default:
        setupCounter = 0;
        initialSetup = false;
        break;
    }

    return;

fail:
    SHTRPD_ERR("case%d failed!, shtrpd_driver close\n", setupCounter);
    
    mutex_lock(&shtrpd_ioctl_lock);
    if(shtrpd_flip_state) {
        gpio_set_value(NRST_LINE_PIN,0);
        usleep(100000);
        gpio_set_value(NRST_LINE_PIN,1);
        usleep(SHTRPD_NRST_DELAY);
        shtrpd_ts_phys_init();
    }
    mutex_unlock(&shtrpd_ioctl_lock);
    
    return;
}

#if (defined(SHTRPD_INTERRUPT_TIMING_CHECK_ENABLE) || defined(SHTRPD_FAIL_TOUCH_CLEAR_ENABLE))
/* ------------------------------------------------------------------------- */
/* shtrpd_int_timing_delay_func                                              */
/* ------------------------------------------------------------------------- */
static long shtrpd_int_timing_check_reati(void)
{
    int32_t ret = -1;
    u8 data_buffer[11];

    SHTRPD_DBG("refresh re-ati start  \n");

    /*  ControlSettings0 and clear ACK_RESET Bit    */
    data_buffer[0] = shtrpd_properties.controlsetting_val[0] | AUTO_ATI | EVENT_MODE | AUTO_MODES;
    /*  ControlSettings1    */
    if(shtrpd_touchcruiser_state){
        data_buffer[1] = shtrpd_properties.controlsetting_val[1] | LOW_POWER;
    }else{
        data_buffer[1] = shtrpd_properties.controlsetting_val[1] | DIS_TOUCH_EVENT | LOW_POWER;
    }
    /*  ControlSettings2(add)    */
    if(shtrpd_atic_adj_disable)
        data_buffer[2] = shtrpd_properties.controlsetting_val[2] | ATIC_ADJ_DISABLE | PM_RE_ATI | NM_RE_ATI;
    else
        data_buffer[2] = shtrpd_properties.controlsetting_val[2] | PM_RE_ATI | NM_RE_ATI;

    if(shtrpd_temp_drift)
        data_buffer[2] |= TEMP_DRIFT_MODE;
    
    /*  ControlSettings3(add)    */
    data_buffer[3] = shtrpd_properties.controlsetting_val[3];

    /*  ATI Settings Data   */
    data_buffer[4] = (unsigned char)(shtrpd_properties.atitarget_val>>8);
    /*  ATI Target  */
    data_buffer[5] = (unsigned char)shtrpd_properties.atitarget_val;
    /*  ATI C   */
    data_buffer[6] = shtrpd_properties.atic_val;
    data_buffer[7] = (unsigned char)(shtrpd_properties.atitarget2_val>>8);
    /*  Non-trackpad channels ATI Target    */
    data_buffer[8] = (unsigned char)shtrpd_properties.atitarget2_val;
    /*  Non-trackpad channels ATI C */
    data_buffer[9] = shtrpd_properties.atic2_val;
    data_buffer[10] = shtrpd_properties.re_ati_range;
    
    ret = shtrpd_i2c_user_write_block_data(shtrpd_client, CONTROL_SETTINGS, 
                                       11, data_buffer);

    SHTRPD_DBG("refresh re-ati end\n");

    return ret;
}
#endif /* SHTRPD_INTERRUPT_TIMING_CHECK_ENABLE */

#if defined(SHTRPD_INTERRUPT_TIMING_CHECK_ENABLE)
/* ------------------------------------------------------------------------- */
/* shtrpd_int_timing_delay_func                                              */
/* ------------------------------------------------------------------------- */
static void shtrpd_int_timing_delay_func(struct work_struct *work)
{
    long ret = 0;

    if(1 == shtrpd_flip_state){
        if((1 == force_reati_req) || 
          ((0 == force_reati_req) && (0 == int_timing_check_state))){
            SHTRPD_DBG("refresh re-ati start!!! \n");
            ret = shtrpd_int_timing_check_reati();
            if( ret < 0){
                SHTRPD_DBG("re-ati error, ret=%d \n", (int)ret);
            }
        }
    }
    if(1 == force_reati_req){
        force_reati_req = 0;
    }
}

/* ------------------------------------------------------------------------- */
/* shtrpd_int_timing_check                                                   */
/* ------------------------------------------------------------------------- */
static void shtrpd_int_timing_check_delay_cancel(void)
{
    cancel_delayed_work(&shtrpd_work_int_timing_delay);
}

/* ------------------------------------------------------------------------- */
/* shtrpd_int_timing_check_touches_get                                       */
/* ------------------------------------------------------------------------- */
static void shtrpd_int_timing_check_touches_get(u8 *buffer)
{
    u8  i;
    u16 wk_X,wk_Y;
    struct shtrpd_fingers_structure wk_tch[MAX_TOUCHES];

    for (i=0; i<MAX_TOUCHES; i++) {
        /*  XPos    */
        wk_X = (u16)(buffer[(i * 7) + 5] << 8) | (u16)(buffer[(i * 7) + 6]);
        /*  YPos    */
        wk_Y = (u16)(buffer[(i * 7) + 7] << 8) | (u16)(buffer[(i * 7) + 8]);

        if(((SHTRPD_MAX_X > wk_X) && (SHTRPD_MAX_Y > wk_Y)) ||
            (SHTRPD_XY_POS_DISABLE == wk_X)){
            /*  XPos    */
            wk_tch[i].XPos = (u16)(buffer[(i * 7) + 5] << 8) | 
                                                    (u16)(buffer[(i * 7) + 6]);
            /*  YPos    */
            wk_tch[i].YPos = (u16)(buffer[(i * 7) + 7] << 8) | 
                                                    (u16)(buffer[(i * 7) + 8]);
            /*  Touch Strength  */
            wk_tch[i].touchStrength = (u16)(buffer[(i * 7) + 9] << 8) | 
                                                    (u16)(buffer[(i * 7) + 10]);
            /* area count */
            wk_tch[i].area = (u8)(buffer[(i*7)+11]);

        }
    }
    memcpy(shtrpd_int_check_touches_old, wk_tch, sizeof(wk_tch));
    memset(shtrpd_int_check_touches_time_count, 0, 
                            sizeof(shtrpd_int_check_touches_time_count));
}

/* ------------------------------------------------------------------------- */
/* shtrpd_int_timing_check_touches_keep_check                                */
/* ------------------------------------------------------------------------- */
static void shtrpd_int_timing_check_touches_keep_check(u8 *buffer)
{
    u8  i;
    u16 wk_X,wk_Y;
    u16 wk_x_p,wk_x_n,wk_y_p,wk_y_n;
    struct shtrpd_fingers_structure wk_tch[MAX_TOUCHES];
    int clr_flg = 0;

    if(0 == shtrpd_int_check_touches_state){
        SHTRPD_DBG("touches_keep_check state off  \n");
        return;
    }

    for (i=0; i<MAX_TOUCHES; i++) {
        /*  XPos    */
        wk_X = (u16)(buffer[(i * 7) + 5] << 8) | (u16)(buffer[(i * 7) + 6]);
        /*  YPos    */
        wk_Y = (u16)(buffer[(i * 7) + 7] << 8) | (u16)(buffer[(i * 7) + 8]);

        if(((SHTRPD_MAX_X > wk_X) && (SHTRPD_MAX_Y > wk_Y)) ||
           ((SHTRPD_XY_POS_DISABLE == wk_X) || 
            (SHTRPD_XY_POS_DISABLE == wk_Y))){
            /*  XPos    */
            wk_tch[i].XPos = (u16)(buffer[(i * 7) + 5] << 8) | 
                                                    (u16)(buffer[(i * 7) + 6]);
            /*  YPos    */
            wk_tch[i].YPos = (u16)(buffer[(i * 7) + 7] << 8) | 
                                                    (u16)(buffer[(i * 7) + 8]);
            /*  Touch Strength  */
            wk_tch[i].touchStrength = (u16)(buffer[(i * 7) + 9] << 8) | 
                                                    (u16)(buffer[(i * 7) + 10]);
            /* area count */
            wk_tch[i].area = (u8)(buffer[(i*7)+11]);

            if((SHTRPD_XY_POS_DISABLE == wk_X) || (SHTRPD_XY_POS_DISABLE == wk_Y)){
                shtrpd_int_check_touches_old[i].XPos = wk_X;
                shtrpd_int_check_touches_old[i].YPos = wk_Y;
                shtrpd_int_check_touches_time_count[i] = 0;
                continue;
            }

            wk_x_p = shtrpd_int_check_touches_old[i].XPos + SHTRPD_TCH_KEEP_CHECK_POS_DELTA;
            wk_x_n = shtrpd_int_check_touches_old[i].XPos - SHTRPD_TCH_KEEP_CHECK_POS_DELTA;
            wk_y_p = shtrpd_int_check_touches_old[i].YPos + SHTRPD_TCH_KEEP_CHECK_POS_DELTA;
            wk_y_n = shtrpd_int_check_touches_old[i].YPos - SHTRPD_TCH_KEEP_CHECK_POS_DELTA;

            if((wk_tch[i].XPos <= wk_x_p) && 
               (wk_tch[i].XPos >= wk_x_n) &&
               (wk_tch[i].YPos <= wk_y_p) &&
               (wk_tch[i].YPos >= wk_y_n) &&
               (wk_tch[i].area <= SHTRPD_TCH_KEEP_CHECK_AREA_MAX)){

                shtrpd_int_check_touches_time_count[i]++;
                if(SHTRPD_TCH_KEEP_CHECK_TIME_CNT < shtrpd_int_check_touches_time_count[i]){
                    SHTRPD_DBG("request re-ati  \n");
                    SHTRPD_DBG("touches_no=%d, x=%04x, y=%04x, area=%d cnt=%d\n",i, 
                                       wk_tch[i].XPos, wk_tch[i].YPos, wk_tch[i].area,
                                       shtrpd_int_check_touches_time_count[i] );
                    force_reati_req = 1;
                    queue_delayed_work(shtrpd_work_queue, &shtrpd_work_int_timing_delay, 
                            msecs_to_jiffies(5));
                    shtrpd_int_check_touches_time_count[i] = 0;
                    return;
                }
                SHTRPD_DBG("touches_no=%d, touches keep cnt=%d  \n", i, shtrpd_int_check_touches_time_count[i]);
            }
            else{
                shtrpd_int_check_touches_old[i].XPos = wk_X;
                shtrpd_int_check_touches_old[i].YPos = wk_Y;
                shtrpd_int_check_touches_time_count[i] = 0;
            }
        }
    }

    for (i=0; i<MAX_TOUCHES; i++) {
        SHTRPD_DBG("touches_old[%d] X=%04x Y=%04x AREA=%04x \n", i,
                    shtrpd_int_check_touches_old[i].XPos,
                    shtrpd_int_check_touches_old[i].YPos,
                    shtrpd_int_check_touches_old[i].area );
    }

    clr_flg = 1;

    for (i=0; i<MAX_TOUCHES; i++) {
        if((SHTRPD_XY_POS_DISABLE != wk_tch[i].XPos) && 
           (SHTRPD_XY_POS_DISABLE != wk_tch[i].YPos)){
            clr_flg = 0;
        }
    }
    if(clr_flg){
        SHTRPD_DBG("touches keeping check off \n");
       shtrpd_int_check_touches_state = 0;
    }
}

/* ------------------------------------------------------------------------- */
/* shtrpd_int_timing_check_touches_clr                                       */
/* ------------------------------------------------------------------------- */
static void shtrpd_int_timing_check_touches_ini(u8 *buf)
{
    memset(shtrpd_int_check_touches_old, 0, sizeof(shtrpd_int_check_touches_old));
    shtrpd_int_check_touches_state = 0;
    shtrpd_int_timing_check_touches_get(buf);
    shtrpd_int_check_touches_state = 1;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_int_timing_check                                                   */
/* ------------------------------------------------------------------------- */
static void shtrpd_int_timing_check(u8 *buf)
{
    unsigned long curTime = jiffies;
    int timeCondition = 0;

    if(buf[0] & SHOW_RESET){
        SHTRPD_DBG("int timing check SHOW_RESET \n");
        int_timing_check_state = 0;
        int_timing_check_reati = 0;
        return;
    }

    if((0 == int_timing_check_state) && (buf[0] & NO_OF_FINGERS)){
        SHTRPD_DBG("TD int timing check start \n");
        int_timing_check_state = 1;
        int_timing_check_old_time = curTime;
        shtrpd_int_timing_check_delay_cancel();
        shtrpd_int_timing_check_touches_ini(buf);

    }
    else if((0 != int_timing_check_state) && (buf[0] & NO_OF_FINGERS) && 
            (0 == int_timing_check_reati)){
        SHTRPD_DBG("DRAG int timing checking  \n");
        timeCondition = time_after(curTime, int_timing_check_old_time + 
                         msecs_to_jiffies(SHTRPD_INTERRUPT_TIMING_CHECK_TIME));
        SHTRPD_DBG("interrupt timing check, time_after condition=%d\n", timeCondition);

        int_timing_check_old_time = curTime;

        if(timeCondition || SHTRPD_INTERRUPT_TIMING_CHECK_TIME_DBG){
            SHTRPD_DBG("DRAG int timing timeover !!!\n");
            int_timing_check_reati = 1;
//            if(SHTRPD_INTERRUPT_TIMING_CHECK_TIME_DBG){
//               SHTRPD_INTERRUPT_TIMING_CHECK_TIME_DBG = 0;
//            }
              return;
        }
        shtrpd_int_timing_check_touches_keep_check(buf);
    }
    else if((buf[0] & SNAP_OUTPUT) && !(buf[0] & NO_OF_FINGERS) &&
            (0 == int_timing_check_reati) && (buf[1] || buf[2] || buf[3])){
        if(SHTRPD_SNAP_TCH_UNMATCH_CHECK){
            SHTRPD_DBG("SNAP_OUTPUT and nothing NO_OF_FINGER  !!!\n");
            int_timing_check_state = 1;
            int_timing_check_old_time = curTime;
            int_timing_check_reati = 1;
        }
    }
    else if((0 != int_timing_check_state) && !(buf[0] & NO_OF_FINGERS) &&
            !(buf[0] & SNAP_OUTPUT) && !(buf[1] || buf[2] || buf[3] || buf[4])){
        SHTRPD_DBG("TU int timing check end \n");
        if(1 == int_timing_check_reati){
            SHTRPD_DBG("request re-ati  \n");
            queue_delayed_work(shtrpd_work_queue, &shtrpd_work_int_timing_delay, 
                            msecs_to_jiffies(25));
            int_timing_check_reati = 0;
        }
        else{
            if(SHTRPD_TU_DELAY_REATI_ENABLE){
                SHTRPD_DBG("request TU delay re-ati  \n");
                queue_delayed_work(shtrpd_work_queue, &shtrpd_work_int_timing_delay, 
                            msecs_to_jiffies(SHTRPD_TU_DELAY_REATI_TIME));
            }
        }
        int_timing_check_state = 0;
        shtrpd_int_check_touches_state = 0;
    }
}
#endif /* SHTRPD_INTERRUPT_TIMING_CHECK_ENABLE */

/* ------------------------------------------------------------------------- */
/* shtrpd_multiple_touches                                                   */
/* ------------------------------------------------------------------------- */
/*  Report multiple touches with the Trackpad Device    */
static void shtrpd_multiple_touches(void *dev_id, u8 *buffer, u8 noOfTouches)
{
#if defined( SHTRPD_DRAG_SMOOTH_ENABLE )
    struct shtrpd_ts_data *data = dev_id;
    u8 i;
    u8 finger_state = 0;

    u16 wk_Xpos[MAX_TOUCHES];
    u16 wk_Ypos[MAX_TOUCHES];
    u16 wk_Z[MAX_TOUCHES];
    u8 wk_A[MAX_TOUCHES];
    u16 wk_X,wk_Y;

    SHTRPD_DBG(KERN_ERR "Multi Touch start \n");

#if defined(SHTRPD_DRUMMING_SPLIT_ENABLE) && defined(SHTRPD_DRUMMING_SPLIT_SWITCH_FINGER_ENABLE)
    for (i=0; i<MAX_TOUCHES; i++) {
        /*  XPos    */
        wk_X = (u16)(buffer[(i * 7) + 5] << 8) | (u16)(buffer[(i * 7) + 6]);
        /*  YPos    */
        wk_Y = (u16)(buffer[(i * 7) + 7] << 8) | (u16)(buffer[(i * 7) + 8]);

        if(((SHTRPD_MAX_X > wk_X) && (SHTRPD_MAX_Y > wk_Y)) ||
            (SHTRPD_XY_POS_DISABLE == wk_X)){
            /*  XPos    */
            shtrpd_touches[i].XPos = (u16)(buffer[(i * 7) + 5] << 8) | 
                                                    (u16)(buffer[(i * 7) + 6]);
            /*  YPos    */
            shtrpd_touches[i].YPos = (u16)(buffer[(i * 7) + 7] << 8) | 
                                                    (u16)(buffer[(i * 7) + 8]);
            /*  Touch Strength  */
            shtrpd_touches[i].touchStrength = (u16)(buffer[(i * 7) + 9] << 8) | 
                                                    (u16)(buffer[(i * 7) + 10]);
            /* area count */
            shtrpd_touches[i].area = (u8)(buffer[(i*7)+11]);

        }

        if(SHTRPD_XY_POS_DISABLE == wk_X){
            /* Touche off */
            finger_state &= ~(1<<i);
        } else {
            if ((SHTRPD_MAX_X > wk_X) && (SHTRPD_MAX_Y > wk_Y)) {
                finger_state |= (1<<i);
            } else {
                finger_state &= ~(1<<i);
            }
        }
    }
    
    shtrpd_replace_fingers(&finger_state);
    
    for (i=0; i<MAX_TOUCHES; i++) {
        wk_Xpos[i] = shtrpd_touches[i].XPos;
        wk_Ypos[i] = shtrpd_touches[i].YPos;
        wk_Z[i] = shtrpd_touches[i].touchStrength;
    }
#else
    for (i=0; i<MAX_TOUCHES; i++) {
        /*  XPos    */
        wk_X = (u16)(buffer[(i * 7) + 5] << 8) | (u16)(buffer[(i * 7) + 6]);
        /*  YPos    */
        wk_Y = (u16)(buffer[(i * 7) + 7] << 8) | (u16)(buffer[(i * 7) + 8]);

        if(((SHTRPD_MAX_X > wk_X) && (SHTRPD_MAX_Y > wk_Y)) ||
            (SHTRPD_XY_POS_DISABLE == wk_X)){
            /*  XPos    */
            wk_Xpos[i] = (u16)(buffer[(i * 7) + 5] << 8) | 
                                                    (u16)(buffer[(i * 7) + 6]);
            shtrpd_touches[i].XPos = (u16)(buffer[(i * 7) + 5] << 8) | 
                                                    (u16)(buffer[(i * 7) + 6]);
            /*  YPos    */
            wk_Ypos[i] = (u16)(buffer[(i * 7) + 7] << 8) | 
                                                    (u16)(buffer[(i * 7) + 8]);
            shtrpd_touches[i].YPos = (u16)(buffer[(i * 7) + 7] << 8) | 
                                                    (u16)(buffer[(i * 7) + 8]);
            /*  Touch Strength  */
            wk_Z[i]    = (u16)(buffer[(i * 7) + 9] << 8) | 
                                                    (u16)(buffer[(i * 7) + 10]);

            shtrpd_touches[i].touchStrength = (u16)(buffer[(i * 7) + 9] << 8) | 
                                                    (u16)(buffer[(i * 7) + 10]);
            /* area count */
            shtrpd_touches[i].area = (u8)(buffer[(i*7)+11]);

        }

        if(SHTRPD_XY_POS_DISABLE == wk_Xpos[i] ){
            /* Touche off */
            finger_state &= ~(1<<i);
        } else {
            if ((SHTRPD_MAX_X > wk_X) && (SHTRPD_MAX_Y > wk_Y)) {
                finger_state |= (1<<i);
            } else {
                finger_state &= ~(1<<i);
            }
        }
    }
#endif /* SHTRPD_DRUMMING_SPLIT_ENABLE && SHTRPD_DRUMMING_SPLIT_SWITCH_FINGER_ENABLE */

    shtrpd_finger_fw_state = finger_state;

#if defined(SHTRPD_EDGE_FAIL_TOUCH_REJECTION_ENABLE)
    shtrpd_filter_edge_fail_touch_check(dev_id, &finger_state);
    #if defined(SHTRPD_GRIP_FAIL_FLICK_REJECTION_ENABLE)
        shtps_filter_grip_fail_flick_check(&finger_state);
    #endif /* SHTRPD_GRIP_FAIL_FLICK_REJECTION_ENABLE */
#endif /* SHTRPD_EDGE_FAIL_TOUCH_REJECTION_ENABLE */

#if defined(SHTRPD_EDGE_TD_MASK_ENABLE)
	shtrpd_edge_td_mask_check(&finger_state);
#endif /* SHTRPD_EDGE_TD_MASK_ENABLE */

#if defined(SHTRPD_SPLIT_FINGER_REJECTION_ENABLE)
    shtrpd_split_finger_reject_check(&finger_state);
#endif /* SHTRPD_SPLIT_FINGER_REJECTION_ENABLE */

#if defined(SHTRPD_TOP_OF_SCREEN_GHOST_REJECTION_ENABLE)
    shtrpd_filter_topofscreen_ghost_reject_check(&finger_state);
#endif /* SHTRPD_TOP_OF_SCREEN_GHOST_REJECTION_ENABLE */

#ifdef SHTRPD_INTOUCH_GHOST_REJECTION_ENABLE
    shtrpd_intouch_ghost_reject_check(&finger_state);
#endif /* SHTRPD_INTOUCH_GHOST_REJECTION_ENALBE */

#if defined(SHTRPD_TOUCH_DOWN_THRESHOLD_ENABLE)
    shtrpd_change_touch_down_threshold(data, finger_state, wk_Xpos, wk_Ypos);
#endif /* SHTRPD_TOUCH_DOWN_THRESHOLD_ENABLE */

#if defined(SHTRPD_TOUCH_UP_DELAY_ENABLE)
    mutex_lock(&shtrpd_tu_delay_lock);
    shtrpd_touch_up_check_func(&finger_state, wk_Xpos, wk_Ypos, wk_Z, wk_A);
#endif /* SHTRPD_TOUCH_UP_DELAY_ENABLE */

#if defined(SHTRPD_FAIL_TOUCH_CLEAR_ENABLE)
    shtrpd_fail_touch_clear_check(data, &finger_state);
#endif /* SHTRPD_FAIL_TOUCH_CLEAR_ENABLE */
    
#if defined(SHTRPD_TAP_JITTER_FILTER_ENABLE)
    shtrpd_tap_jitter_filter_check(dev_id, &finger_state);
#endif /* SHTRPD_TAP_JITTER_FILTER_ENABLE */

#if defined(SHTRPD_DRUMMING_SPLIT_ENABLE)
    shtrpd_drumming_split_check(data, &finger_state, wk_Xpos, wk_Ypos);
#endif /* SHTRPD_DRUMMING_SPLIT_ENABLE */

#if defined(SHTRPD_TU_JITTER_FILTER_ENABLE)
    shtrpd_tu_jitter_filter_check(finger_state, wk_Xpos, wk_Ypos);
#endif /* SHTRPD_TU_JITTER_FILTER_ENABLE */

#if defined(SHTRPD_FAIL_FLICK_RECOVER_ENABLE)
    shtrpd_fail_flick_recover_check(finger_state);
#endif /* SHTRPD_FAIL_FLICK_RECOVER_ENABLE */

#if defined(SHTRPD_CPU_CLOCK_CONTROL_ENABLE)
	shtrpd_perf_lock_check(finger_state);
#endif /* SHTRPD_CPU_CLOCK_CONTROL_ENABLE */

    for(i = 0;i < MAX_TOUCHES; i++){
        if((finger_state & (1<<i))){
            if((shtrpd_finger_state_old & (1<<i))){
                shtrpd_add_drag_hist(SHTRPD_POSTYPE_X, i, wk_Xpos[i]);
                shtrpd_add_drag_hist(SHTRPD_POSTYPE_Y, i, wk_Ypos[i]);
            }
        }else{
            shtrpd_init_drag_hist(SHTRPD_POSTYPE_X, i, wk_Xpos[i]);
            shtrpd_init_drag_hist(SHTRPD_POSTYPE_Y, i, wk_Ypos[i]);
        }
    }

    shtrpd_filter_pos_compensation(SHTRPD_POSTYPE_X ,finger_state);
    shtrpd_filter_pos_compensation(SHTRPD_POSTYPE_Y ,finger_state);

    for(i = 0;i < MAX_TOUCHES; i++){
        SHTRPD_INFO("ABS_MT_TRACKING_ID ID=%d ; STATE=%d ; X=%d ; Y=%d ; Z=%d\n" ,
                     i,
                     (finger_state & (1<<i)),
                     shtrpd_touches[i].XPos,
                     shtrpd_touches[i].YPos,
                     shtrpd_touches[i].touchStrength);

        if((finger_state & (1<<i))){ 
            if(shtrpd_touchcruiser_state){
                SHTRPD_DBG("input_report: ID=%d\n", i);
                input_mt_slot(data->input_dev, i);
                input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, i+1);
                input_report_abs(data->input_dev, ABS_MT_POSITION_X, SHTRPD_NOTIFY_POS_SCALE_X(shtrpd_touches[i].XPos));
                input_report_abs(data->input_dev, ABS_MT_POSITION_Y, SHTRPD_NOTIFY_POS_SCALE_Y(shtrpd_touches[i].YPos));
                if (shtrpd_touches[i].touchStrength == 0) {
                    shtrpd_touches[i].touchStrength = 1;
                }
                input_report_abs(data->input_dev, ABS_MT_PRESSURE, shtrpd_touches[i].touchStrength);
                input_sync(data->input_dev);
            }
        } else {
            if((shtrpd_finger_state_old & (1<<i))){
                SHTRPD_DBG("input_report -1: ID=%d\n", i);
                input_mt_slot(data->input_dev, i);
                input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, -1);
                input_sync(data->input_dev);
            }
        }
    }
    
    shtrpd_finger_state_old = finger_state;
    shtrpd_finger_fw_state_old = shtrpd_finger_fw_state;

#if defined(SHTRPD_TOUCH_UP_DELAY_ENABLE)
    mutex_unlock(&shtrpd_tu_delay_lock);
#endif /* SHTRPD_TOUCH_UP_DELAY_ENABLE */

    /* old the info of the reported touches update */
    memcpy(shtrpd_touches_old, shtrpd_touches, sizeof(shtrpd_touches_old));
    
    #if defined(SHTRPD_DRUMMING_SPLIT_ENABLE) && defined(SHTRPD_DRUMMING_SPLIT_SWITCH_FINGER_ENABLE)
        shtrpd_replace_fingers_clear_check(finger_state);
    #endif /* SHTRPD_DRUMMING_SPLIT_ENABLE && SHTRPD_DRUMMING_SPLIT_SWITCH_FINGER_ENABLE */
#else
    struct shtrpd_ts_data *data = dev_id;
    u8 i;
    u16 wk_X,wk_Y;

    if (noOfTouches > 0) {
        /* If only one touch is present, handle it as a single touch
         * Multi touches are reported in a different manner
         */
        /*  Multi Touch - More than 1 touch is present  */

        SHTRPD_DBG(KERN_ERR "Multi Touch start \n");

        for (i=0; i < MAX_TOUCHES; i++) {
            /*  XPos    */
            wk_X = (u16)(buffer[(i * 7) + 5] << 8) | (u16)(buffer[(i * 7) + 6]);
            /*  YPos    */
            wk_Y = (u16)(buffer[(i * 7) + 7] << 8) | (u16)(buffer[(i * 7) + 8]);

            if(!(((SHTRPD_MAX_X > wk_X) && (SHTRPD_MAX_Y > wk_Y)) ||
                (SHTRPD_XY_POS_DISABLE == wk_X))){
                continue;
            }

            /*  XPos    */
            shtrpd_touches[i].XPos = (u16)(buffer[(i * 7) + 5] << 8) | 
                                                        (u16)(buffer[(i * 7) + 6]);
            /*  YPos    */
            shtrpd_touches[i].YPos = (u16)(buffer[(i * 7) + 7] << 8) | 
                                                        (u16)(buffer[(i * 7) + 8]);
            /*  Touch Strength  */
            shtrpd_touches[i].touchStrength = 
                                     (u16)(buffer[(i * 7) + 9] << 8) | 
                                                        (u16)(buffer[(i * 7) + 10]);

            /* area count */
            shtrpd_touches[i].area = (u8)(buffer[(i*7)+11]);

            if(SHTRPD_XY_POS_DISABLE == shtrpd_touches[i].XPos ){
                /* Touche off */
                if(finger_enable & (1<<i)){
                    SHTRPD_DBG("Touche off \n");
                    /* Check ABS_MT_SLOT */
                    if( old_output_slot_no != i){
                        input_mt_slot(data->input_dev, i);
                        old_output_slot_no = -1;
                        SHTRPD_DBG("ABS_MT_SLOT slot=%d\n",i);
                    }
                    finger_enable &= ~(1<<i);
                    input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, -1);
                    input_sync(data->input_dev);
                    SHTRPD_DBG("ABS_MT_TRACKING_ID ID=-1\n");
                }
                continue;
            }

            if(finger_enable & (1<<i)){
                /*  Implement small filter  */
                if ((abs(shtrpd_touches[i].prev_x - 
                         shtrpd_touches[i].XPos) < 15) && 
                    (abs(shtrpd_touches[i].prev_y - 
                         shtrpd_touches[i].YPos) < 15)) {

                    shtrpd_touches[i].XPos = shtrpd_touches[i].prev_x;
                    shtrpd_touches[i].YPos = shtrpd_touches[i].prev_y;
                }

                if( (shtrpd_touches[i].XPos != shtrpd_touches[i].prev_x) ||
                    (shtrpd_touches[i].YPos != shtrpd_touches[i].prev_y)){

                    /* Check ABS_MT_SLOT */
                    if( old_output_slot_no != i){
                        input_mt_slot(data->input_dev, i);
                        old_output_slot_no = i;
                    }

                    /*  Report key as pressed   */
                    input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, i+1);
                    input_report_abs(data->input_dev, ABS_MT_POSITION_X, 
                                    shtrpd_touches[i].XPos);
                    input_report_abs(data->input_dev, ABS_MT_POSITION_Y, 
                                    shtrpd_touches[i].YPos);
                    input_report_abs(data->input_dev, ABS_MT_PRESSURE,  
                                    shtrpd_touches[i].touchStrength);
                    input_sync(data->input_dev);
                    SHTRPD_INFO("ABS_MT_TRACKING_ID ID=%d ; X=%d ; Y=%d\n",
                            i+1,
                            shtrpd_touches[i].XPos,
                            SHTRPD_MAX_Y - shtrpd_touches[i].YPos);
                }
            }
            else{
                finger_enable |= (1<<i);
                SHTRPD_DBG("ABS_MT_SLOT slot=%d finger_enable=%d\n", 
                                                        i, finger_enable);
                input_mt_slot(data->input_dev, i);
                old_output_slot_no = i;

                /*  Report key as pressed   */
                input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, i+1);
                input_report_abs(data->input_dev, ABS_MT_POSITION_X, 
                                shtrpd_touches[i].XPos);
                input_report_abs(data->input_dev, ABS_MT_POSITION_Y, 
                                shtrpd_touches[i].YPos);
                input_report_abs(data->input_dev, ABS_MT_PRESSURE,  
                                shtrpd_touches[i].touchStrength);
                input_sync(data->input_dev);
                SHTRPD_INFO("ABS_MT_TRACKING_ID ID=%d ; X=%d ; Y=%d\n",
                            i+1,
                            shtrpd_touches[i].XPos,
                            SHTRPD_MAX_Y - shtrpd_touches[i].YPos);
                
            }
            /*  Save the current X Y positions for use
            *  in the next communication window
            */
            shtrpd_touches[i].prev_x = shtrpd_touches[i].XPos;
            shtrpd_touches[i].prev_y = shtrpd_touches[i].YPos;
        }
    }
    /*  Clear Touches   */
    else {
        SHTRPD_DBG("Clear all Touches\n");

        for( i=0; i < MAX_TOUCHES; i++){
            if( finger_enable & (1<<i)){
                input_mt_slot(data->input_dev, i);
                input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, -1);
                input_sync(data->input_dev);
                SHTRPD_DBG("ABS_MT_SLOT slot=%d\n",i);
            }
        }
        finger_enable = 0;
        old_output_slot_no = -1;
    }
    /* old NO_OF_FINGERS update */
    noOfTouches_old = noOfTouches;

    /* old the info of the reported touches update */
    memcpy(shtrpd_touches_old, shtrpd_touches, sizeof(shtrpd_touches_old));
#endif /* SHTRPD_DRAG_SMOOTH_ENABLE */
#if 0 /* FW0001 */
    struct shtrpd_ts_data *data = dev_id;
    u8 i;
    u8 request_finger = 0;
    u8 wk_id;
    u16 wk_Xpos;
    u16 wk_Ypos;

    if (noOfTouches > 0) {
        /* If only one touch is present, handle it as a single touch
         * Multi touches are reported in a different manner
         */
        /*  Only one touch is present   */
        if (noOfTouches == 0x01) {
            SHTRPD_DBG("Single Touch start \n");
            
            wk_id = buffer[1];
            wk_Xpos = (u16)((buffer[2] << 8) + (buffer[3]));
            wk_Ypos = (u16)((buffer[4] << 8) + (buffer[5]));
            
            if((0 < wk_id) && (MAX_TOUCHES >= wk_id) &&
               (SHTRPD_MAX_X > wk_Xpos) && 
               (SHTRPD_MAX_Y > wk_Ypos)) {
            
                /*  ID of this touch    */
                shtrpd_touches[0].ID = buffer[1];
                /*  XPos    */
                shtrpd_touches[0].XPos = (u16)((buffer[2] << 8) + (buffer[3]));
                /*  YPos    */
                shtrpd_touches[0].YPos = (u16)((buffer[4] << 8) + (buffer[5]));
                /*  Touch Strength  */
                shtrpd_touches[0].touchStrength = (u16)((buffer[6] << 8)
                                                            + (buffer[7]));

                /*  Implement small filter  */
                if ((abs(shtrpd_touches[0].prev_x 
                    - shtrpd_touches[0].XPos) < 20) && 
                    (abs(shtrpd_touches[0].prev_y 
                    - shtrpd_touches[0].YPos) < 20)) {
                    shtrpd_touches[0].XPos = shtrpd_touches[0].prev_x;
                    shtrpd_touches[0].YPos = shtrpd_touches[0].prev_y;
                }

                /*  Save the current X Y positions for use in
                 *  the next communication window
                 */
                shtrpd_touches[0].prev_x = shtrpd_touches[0].XPos;
                shtrpd_touches[0].prev_y = shtrpd_touches[0].YPos;

                /*  report the key as pressed,
                 *  if it is not a proximity that is detected
                 */
                if (shtrpd_touches[0].ID < 50) {
                    if( old_output_slot_no != (shtrpd_touches[0].ID-1)){
                        input_mt_slot(data->input_dev, shtrpd_touches[0].ID-1);
                        old_output_slot_no = shtrpd_touches[0].ID - 1;
                        finger_enable |= (1<<old_output_slot_no);
                        SHTRPD_DBG("ABS_MT_SLOT slot=%d finger_enable=%d\n",
                                        shtrpd_touches[0].ID-1, finger_enable);
                    }

                    input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, 
                                        shtrpd_touches[0].ID);
                    input_report_abs(data->input_dev, ABS_MT_POSITION_X,  
                                        SHTRPD_MAX_X - shtrpd_touches[0].XPos);
                    input_report_abs(data->input_dev, ABS_MT_POSITION_Y,  
                                        shtrpd_touches[0].YPos);
                    input_report_abs(data->input_dev, ABS_MT_PRESSURE,  
                                        shtrpd_touches[0].touchStrength);
                    input_sync(data->input_dev);

                    SHTRPD_INFO("ABS_MT_TRACKING_ID ID=%d ; X=%d ; Y=%d ; Z=%d\n",
                            shtrpd_touches[0].ID,
                            shtrpd_touches[0].XPos,
                            SHTRPD_MAX_Y - shtrpd_touches[0].YPos,
                            shtrpd_touches[0].touchStrength);
                }
            }
        }
        /*  Multi Touch - More than 1 touch is present  */
        else {

            SHTRPD_DBG(KERN_ERR "Multi Touch start \n");

            for (i = 0; i < noOfTouches; i++) {
                wk_id = buffer[(i * 7) + 1];
                wk_Xpos = (u16)((buffer[(i * 7) + 2] << 8) +
                                (buffer[(i * 7) + 3]));
                wk_Ypos = (u16)((buffer[(i * 7) + 4] << 8) +
                                (buffer[(i * 7) + 5]));

                if(!((0 < wk_id) && (MAX_TOUCHES >= wk_id) &&
                   (SHTRPD_MAX_X > wk_Xpos) && 
                   (SHTRPD_MAX_Y > wk_Ypos))) {
                    continue;
                }
                /*  ID of this touch    */
                shtrpd_touches[i].ID = buffer[(i * 7) + 1];
                /*  XPos    */
                shtrpd_touches[i].XPos = (u16)((buffer[(i * 7) + 2] << 8) + 
                                                        (buffer[(i * 7) + 3]));
                /*  YPos    */
                shtrpd_touches[i].YPos = (u16)((buffer[(i * 7) + 4] << 8) + 
                                                        (buffer[(i * 7) + 5]));
                /*  Touch Strength  */
                shtrpd_touches[i].touchStrength = 
                                         (u16)((buffer[(i * 7) + 6] << 8) + 
                                                        (buffer[(i * 7) + 7]));

                /*  Implement small filter  */
                if ((abs(shtrpd_touches[i].prev_x - 
                         shtrpd_touches[i].XPos) < 15) && 
                    (abs(shtrpd_touches[i].prev_y - 
                         shtrpd_touches[i].YPos) < 15)) {

                    shtrpd_touches[i].XPos = shtrpd_touches[i].prev_x;
                    shtrpd_touches[i].YPos = shtrpd_touches[i].prev_y;
                }

                /*  Save the current X Y positions for use
                 *  in the next communication window
                 */
                shtrpd_touches[i].prev_x = shtrpd_touches[i].XPos;
                shtrpd_touches[i].prev_y = shtrpd_touches[i].YPos;

                if (shtrpd_touches[i].ID < 50) {

                    /* Check ABS_MT_SLOT */
                    if(( 1 <= shtrpd_touches[i].ID ) && 
                        (5 >= shtrpd_touches[i].ID)){
                        if( old_output_slot_no != (shtrpd_touches[i].ID-1)){
                            input_mt_slot(data->input_dev, 
                                                    shtrpd_touches[i].ID-1);
                            old_output_slot_no = shtrpd_touches[i].ID - 1;
                            finger_enable |= (1<<old_output_slot_no);
                            SHTRPD_DBG("ABS_MT_SLOT slot=%d finger_enable=%d\n",
                                        shtrpd_touches[i].ID-1, finger_enable);
                        }
                    }

                    /*  Report key as pressed   */
                    input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, 
                                    shtrpd_touches[i].ID);
                    input_report_abs(data->input_dev, ABS_MT_POSITION_X, 
                                    SHTRPD_MAX_X - shtrpd_touches[i].XPos);
                    input_report_abs(data->input_dev, ABS_MT_POSITION_Y, 
                                    shtrpd_touches[i].YPos);
                    input_report_abs(data->input_dev, ABS_MT_PRESSURE,  
                                    shtrpd_touches[i].touchStrength);
                    input_sync(data->input_dev);

                    SHTRPD_INFO("ABS_MT_TRACKING_ID ID=%d ; X=%d ; Y=%d ; Z=%d \n",
                            shtrpd_touches[i].ID,
                            shtrpd_touches[i].XPos,
                            SHTRPD_MAX_Y - shtrpd_touches[i].YPos,
                            shtrpd_touches[i].touchStrength);
                }
            }
            /*  Sync Touches    */
            input_sync(data->input_dev);
        }

        /* check Touches off */

        SHTRPD_DBG("noOfTouches_old=%d ; noOfTouches=%d\n",
                            noOfTouches_old, noOfTouches);

        if( noOfTouches_old > noOfTouches ){

            SHTRPD_DBG("Clear Touche start \n");
            request_finger = 0;
            for (i = 0; i < noOfTouches; i++) {
                if(( 1 <= shtrpd_touches[i].ID ) && 
                    (5 >= shtrpd_touches[i].ID)){
                    request_finger |= (1<<(shtrpd_touches[i].ID-1));
                }
            }

            request_finger = ~request_finger & finger_enable;
            SHTRPD_DBG("request_finger=%d ; finger_enable=%d\n",
                                        request_finger,finger_enable);
                    
            for( i=0; i < MAX_TOUCHES; i++){
                if( request_finger & (1<<i)){
                    /* Check ABS_MT_SLOT */
                    if( old_output_slot_no != i){
                        input_mt_slot(data->input_dev, i);
                        old_output_slot_no = -1;
                        finger_enable &= ~(1<<i);
                        SHTRPD_DBG("ABS_MT_SLOT slot=%d\n",i);
                    }
                    input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, -1);
                    input_sync(data->input_dev);
                    SHTRPD_DBG("ABS_MT_TRACKING_ID ID=-1\n");
                }
            }
        }
        /*  Sync Touches    */
        input_sync(data->input_dev);
    }
    /*  Clear Touches   */
    else {
        SHTRPD_DBG("Clear all Touches\n");

        for( i=0; i < MAX_TOUCHES; i++){
            if( finger_enable & (1<<i)){
                input_mt_slot(data->input_dev, i);
                input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, -1);
                input_sync(data->input_dev);
                SHTRPD_DBG("ABS_MT_SLOT slot=%d\n",i);
            }
        }
        finger_enable = 0;
        old_output_slot_no = -1;
    }
    /* old NO_OF_FINGERS update */
    noOfTouches_old = noOfTouches;

    /* old the info of the reported touches update */
    memcpy(shtrpd_touches_old, shtrpd_touches, sizeof(shtrpd_touches_old));
#endif
}

/* ------------------------------------------------------------------------- */
/* shtrpd_touch_force_off                                                    */
/* ------------------------------------------------------------------------- */
static void shtrpd_touch_force_off(void)
{
    struct shtrpd_ts_data *data = i2c_get_clientdata(shtrpd_client);
    u8 i;

    SHTRPD_DBG("Clear all Touches(force)\n");

    for( i=0; i < MAX_TOUCHES; i++){
#if defined( SHTRPD_DRAG_SMOOTH_ENABLE )
        if( shtrpd_finger_state_old & (1<<i)){
#else
        if( finger_enable & (1<<i)){
#endif /* SHTRPD_DRAG_SMOOTH_ENABLE */
            input_mt_slot(data->input_dev, i);
            input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, -1);
            input_sync(data->input_dev);
            SHTRPD_DBG("ABS_MT_SLOT slot=%d\n",i);
        }
    }
    finger_enable = 0;
#if defined( SHTRPD_DRAG_SMOOTH_ENABLE )
    shtrpd_finger_state_old = 0;
#endif /* SHTRPD_DRAG_SMOOTH_ENABLE */
#if defined( SHTRPD_TOUCH_UP_DELAY_ENABLE )
    memset(td_history, 0, sizeof(td_history));
#endif /* SHTRPD_TOUCH_UP_DELAY_ENABLE */
#if defined(SHTRPD_EDGE_FAIL_TOUCH_REJECTION_ENABLE)
    shtrpd_filter_edge_fail_touch_init();
    #if defined(SHTRPD_GRIP_FAIL_FLICK_REJECTION_ENABLE)
        shtps_filter_grip_fail_flick_init();
    #endif /* SHTRPD_GRIP_FAIL_FLICK_REJECTION_ENABLE */
#endif /* SHTRPD_EDGE_FAIL_TOUCH_REJECTION_ENABLE */
#if defined(SHTRPD_TOP_OF_SCREEN_GHOST_REJECTION_ENABLE)
    shtrpd_filter_topofscreen_ghost_reject_init();
#endif /* SHTRPD_TOP_OF_SCREEN_GHOST_REJECTION_ENABLE */
#ifdef SHTRPD_INTOUCH_GHOST_REJECTION_ENABLE
    shtrpd_intouch_ghost_reject_init();
#endif /* SHTRPD_INTOUCH_GHOST_REJECTION_ENABLE */
#if defined(SHTRPD_TU_JITTER_FILTER_ENABLE)
    shtrpd_tu_jitter_filter_init();
#endif /* SHTRPD_TU_JITTER_FILTER_ENABLE */
#if defined(SHTRPD_CPU_CLOCK_CONTROL_ENABLE)
	shtrpd_perf_lock_sleep();
#endif /* SHTRPD_CPU_CLOCK_CONTROL_ENABLE */
#if defined(SHTRPD_DRUMMING_SPLIT_ENABLE)
    shtrpd_drumming_split_init();
    #if defined(SHTRPD_DRUMMING_SPLIT_SWITCH_FINGER_ENABLE)
        shtrpd_replace_fingers_init();
    #endif /* SHTRPD_DRUMMING_SPLIT_SWITCH_FINGER_ENABLE */
#endif /* SHTRPD_DRUMMING_SPLIT_ENABLE */
#if defined(SHTRPD_TAP_JITTER_FILTER_ENABLE)
	shtrpd_tap_jitter_filter_init();
#endif /* SHTRPD_TAP_JITTER_FILTER_ENABLE */
#if defined(SHTRPD_SPLIT_FINGER_REJECTION_ENABLE)
    shtrpd_split_finger_reject_init();
#endif /* SHTRPD_SPLIT_FINGER_REJECTION_ENABLE */
#if defined(SHTRPD_FAIL_TOUCH_CLEAR_ENABLE)
    shtrpd_fail_touch_clear_count_init();
#endif /* SHTRPD_FAIL_TOUCH_CLEAR_ENABLE */
#if defined(SHTRPD_FAIL_FLICK_RECOVER_ENABLE)
    shtrpd_fail_flick_recover_init();
#endif /* SHTRPD_FAIL_FLICK_RECOVER_ENABLE */

    shtrpd_finger_fw_state = 0;
    shtrpd_finger_fw_state_old = 0;

    old_output_slot_no = -1;
    /* old NO_OF_FINGERS update */
    noOfTouches_old = 0;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_snap_set_keymode                                                   */
/* ------------------------------------------------------------------------- */
static void shtrpd_snap_set_keymode(int mode)
{
#if defined(SHTRPD_SNAP_USER_MASK)
    mutex_lock(&shtrpd_snap_keymode_lock);
    shtrpd_snap_keymode = mode;
    mutex_unlock(&shtrpd_snap_keymode_lock);
#endif /* SHTRPD_SNAP_USER_MASK */
}

/* ------------------------------------------------------------------------- */
/* shtrpd_snap_get_keymode                                                   */
/* ------------------------------------------------------------------------- */
static int shtrpd_snap_get_keymode(void)
{
#if defined(SHTRPD_SNAP_USER_MASK)
    int ret;
    
    mutex_lock(&shtrpd_snap_keymode_lock);
    ret = shtrpd_snap_keymode;
    mutex_unlock(&shtrpd_snap_keymode_lock);
    
    return ret;
#else
    return 0;
#endif /* SHTRPD_SNAP_USER_MASK */
}

#if defined(SHTRPD_SNAP_USER_MASK)
/* ------------------------------------------------------------------------- */
/* shtrpd_snap_user_mask_check                                               */
/* ------------------------------------------------------------------------- */
static void shtrpd_snap_user_mask_check(u16 *keycode)
{
    if(!SHTRPD_SNAP_USER_MASK_DIAGONAL_KEY){
        return;
    }

    if(shtrpd_snap_get_keymode() == TRPD_KEYMODE_DISALBE_DIAGONAL_KEY){
        if(keycode[MASK_KEY_UPLEFT]  == WK_UPLEFT) {
            keycode[MASK_KEY_UPLEFT]  = KEY_LEFT;
        }
        if(keycode[MASK_KEY_UPRIGHT] == WK_UPRIGHT) {
            keycode[MASK_KEY_UPRIGHT] = KEY_RIGHT;
        }
        if(keycode[MASK_KEY_DNLEFT]  == WK_DNLEFT) {
            keycode[MASK_KEY_DNLEFT]  = KEY_LEFT;
        }
        if(keycode[MASK_KEY_DNRIGHT] == WK_DNRIGHT) {
            keycode[MASK_KEY_DNRIGHT] = KEY_RIGHT;
        }
    }
}
#endif /* SHTRPD_SNAP_USER_MASK */

#if defined(SHTRPD_SNAP_MASK)
/* ------------------------------------------------------------------------- */
/* shtrpd_snap_chattering_removal                                            */
/* ------------------------------------------------------------------------- */
static void shtrpd_snap_chattering_removal(u16 *keycode, u8 *keycode_onoff)
{
    int i;

    if (SHTRPD_SNAP_CHATTERING_REMOVAL_ENABLE) {
        for(i=0; i<MAX_SNAPS; i++) {
            if(keycode[i]) {
                if(keycode_onoff[i] == SHTRPD_SNAP_OFF_ON) {
                    if((keycode[i] == KEY_S5) || (keycode[i] == KEY_S6)
                    || (keycode[i] == KEY_STAR) || (keycode[i] == KEY_0) || (keycode[i] == KEY_POUND)) {
                        continue;
                    } else {
                        if(shtrpd_key_chattering_removal_count[i] < SHTRPD_SNAP_CHATTERING_REMOVAL_COUNT) {
                            keycode[i] = 0;
                            keycode_onoff[i] = 0;
                            shtrpd_key_chattering_removal_count[i]++;
                        }
                    }
                }
            } else {
                shtrpd_key_chattering_removal_count[i] = 0;
            }
        }
    }
}

/* ------------------------------------------------------------------------- */
/* shtrpd_snap_mask_diagonal_key                                             */
/* ------------------------------------------------------------------------- */
static void shtrpd_snap_mask_diagonal_key(u16 *keycode)
{
    if (SHTRPD_SNAP_MASK_DIAGONAL_KEY) {
        if(keycode[MASK_KEY_UPLEFT]  == WK_UPLEFT) {
            keycode[MASK_KEY_UPLEFT]  = 0;
        }
        if(keycode[MASK_KEY_UPRIGHT] == WK_UPRIGHT) {
            keycode[MASK_KEY_UPRIGHT] = 0;
        }
        if(keycode[MASK_KEY_DNLEFT]  == WK_DNLEFT) {
            keycode[MASK_KEY_DNLEFT]  = 0;
        }
        if(keycode[MASK_KEY_DNRIGHT] == WK_DNRIGHT) {
            keycode[MASK_KEY_DNRIGHT] = 0;
        }
    }
}

/* ------------------------------------------------------------------------- */
/* shtrpd_snap_mask_check                                                    */
/* ------------------------------------------------------------------------- */
static void shtrpd_snap_mask_check(u16 *keycode, u8 *keycode_onoff)
{
    if (SHTRPD_SNAP_MASK_ENABLE) {

        if (keycode_onoff[MASK_KEY_UPLEFT] == SHTRPD_SNAP_ON_OFF) {
            shtrpd_key_mask_count[MASK_KEY_UPLEFT] = 0;
        }
        if (keycode_onoff[MASK_KEY_UP] == SHTRPD_SNAP_ON_OFF) {
            shtrpd_key_mask_count[MASK_KEY_UP] = 0;
        }
        if (keycode_onoff[MASK_KEY_UPRIGHT] == SHTRPD_SNAP_ON_OFF) {
            shtrpd_key_mask_count[MASK_KEY_UPRIGHT] = 0;
        }
        if (keycode_onoff[MASK_KEY_LEFT] == SHTRPD_SNAP_ON_OFF) {
            shtrpd_key_mask_count[MASK_KEY_LEFT] = 0;
        }
        if (keycode_onoff[MASK_KEY_ENTER] == SHTRPD_SNAP_ON_OFF) {
            shtrpd_key_mask_count[MASK_KEY_ENTER] = 0;
        }
        if (keycode_onoff[MASK_KEY_RIGHT] == SHTRPD_SNAP_ON_OFF) {
            shtrpd_key_mask_count[MASK_KEY_RIGHT] = 0;
        }
        if (keycode_onoff[MASK_KEY_DNLEFT] == SHTRPD_SNAP_ON_OFF) {
            shtrpd_key_mask_count[MASK_KEY_DNLEFT] = 0;
        }
        if (keycode_onoff[MASK_KEY_DOWN] == SHTRPD_SNAP_ON_OFF) {
            shtrpd_key_mask_count[MASK_KEY_DOWN] = 0;
        }
        if (keycode_onoff[MASK_KEY_DNRIGHT] == SHTRPD_SNAP_ON_OFF) {
            shtrpd_key_mask_count[MASK_KEY_DNRIGHT]  = 0;
        }

        if(SHTRPD_SNAP_MASK_KEY_TO_ENTER_MASK_ENABLE) {
            if((keycode[MASK_KEY_ENTER] == KEY_ENTER) && (keycode_onoff[MASK_KEY_ENTER] == SHTRPD_SNAP_OFF_ON)){
                if(((keycode[MASK_KEY_UPLEFT]  == WK_UPLEFT)  && (keycode_onoff[MASK_KEY_UPLEFT]  == SHTRPD_SNAP_ON_ON))
                || ((keycode[MASK_KEY_UP]      == KEY_UP)     && (keycode_onoff[MASK_KEY_UP]      == SHTRPD_SNAP_ON_ON))
                || ((keycode[MASK_KEY_UPRIGHT] == WK_UPRIGHT) && (keycode_onoff[MASK_KEY_UPRIGHT] == SHTRPD_SNAP_ON_ON))
                || ((keycode[MASK_KEY_LEFT]    == KEY_LEFT)   && (keycode_onoff[MASK_KEY_LEFT]    == SHTRPD_SNAP_ON_ON))
                || ((keycode[MASK_KEY_RIGHT]   == KEY_RIGHT)  && (keycode_onoff[MASK_KEY_RIGHT]   == SHTRPD_SNAP_ON_ON))
                || ((keycode[MASK_KEY_DNLEFT]  == WK_DNLEFT)  && (keycode_onoff[MASK_KEY_DNLEFT]  == SHTRPD_SNAP_ON_ON))
                || ((keycode[MASK_KEY_DOWN]    == KEY_DOWN)   && (keycode_onoff[MASK_KEY_DOWN]    == SHTRPD_SNAP_ON_ON))
                || ((keycode[MASK_KEY_DNRIGHT] == WK_DNRIGHT) && (keycode_onoff[MASK_KEY_DNRIGHT] == SHTRPD_SNAP_ON_ON))) {
                    keycode[MASK_KEY_ENTER] = 0;
                    keycode_onoff[MASK_KEY_ENTER] = 0;
                    SHTRPD_LOG_SNAP_MASK("KEY_ENTER Masked\n");
                    return;
                }
            }
        }

        if(SHTRPD_SNAP_MASK_ENTER_TO_KEY_MASK_ENABLE) {
            if ((keycode[MASK_KEY_UPLEFT] == WK_UPLEFT) && (keycode_onoff[MASK_KEY_UPLEFT] == SHTRPD_SNAP_OFF_ON)) {
                if((keycode[MASK_KEY_ENTER] == KEY_ENTER) && (keycode_onoff[MASK_KEY_ENTER] == SHTRPD_SNAP_ON_ON)) {
                    if(shtrpd_key_mask_count[MASK_KEY_UPLEFT] < SHTRPD_SNAP_MASK_COUNT) {
                        keycode[MASK_KEY_UPLEFT] = 0;
                        keycode_onoff[MASK_KEY_UPLEFT] = 0;
                        shtrpd_key_mask_count[MASK_KEY_UPLEFT]++;
                        SHTRPD_LOG_SNAP_MASK("KEY:WK_UPLEFT Masked\n");
                    }
                }
            }
            if ((keycode[MASK_KEY_UP] == KEY_UP) && (keycode_onoff[MASK_KEY_UP] == SHTRPD_SNAP_OFF_ON)) {
                if((keycode[MASK_KEY_ENTER] == KEY_ENTER) && (keycode_onoff[MASK_KEY_ENTER] == SHTRPD_SNAP_ON_ON)) {
                    if(shtrpd_key_mask_count[MASK_KEY_UP] < SHTRPD_SNAP_MASK_COUNT) {
                        keycode[MASK_KEY_UP] = 0;
                        keycode_onoff[MASK_KEY_UP] = 0;
                        shtrpd_key_mask_count[MASK_KEY_UP]++;
                        SHTRPD_LOG_SNAP_MASK("KEY:KEY_UP Masked\n");
                    }
                }
            }
            if ((keycode[MASK_KEY_UPRIGHT] == WK_UPRIGHT) && (keycode_onoff[MASK_KEY_UPRIGHT] == SHTRPD_SNAP_OFF_ON)) {
                if((keycode[MASK_KEY_ENTER] == KEY_ENTER) && (keycode_onoff[MASK_KEY_ENTER] == SHTRPD_SNAP_ON_ON)) {
                    if(shtrpd_key_mask_count[MASK_KEY_UPRIGHT] < SHTRPD_SNAP_MASK_COUNT) {
                        keycode[MASK_KEY_UPRIGHT] = 0;
                        keycode_onoff[MASK_KEY_UPRIGHT] = 0;
                        shtrpd_key_mask_count[MASK_KEY_UPRIGHT]++;
                        SHTRPD_LOG_SNAP_MASK("KEY:WK_UPRIGHT Masked\n");
                    }
                }
            }
            if ((keycode[MASK_KEY_LEFT] == KEY_LEFT) && (keycode_onoff[MASK_KEY_LEFT] == SHTRPD_SNAP_OFF_ON)) {
                if((keycode[MASK_KEY_ENTER] == KEY_ENTER) && (keycode_onoff[MASK_KEY_ENTER] == SHTRPD_SNAP_ON_ON)) {
                    if(shtrpd_key_mask_count[MASK_KEY_LEFT] < SHTRPD_SNAP_MASK_COUNT) {
                        keycode[MASK_KEY_LEFT] = 0;
                        keycode_onoff[MASK_KEY_LEFT] = 0;
                        shtrpd_key_mask_count[MASK_KEY_LEFT]++;
                        SHTRPD_LOG_SNAP_MASK("KEY:KEY_LEFT Masked\n");
                    }
                }
            }
            if ((keycode[MASK_KEY_RIGHT] == KEY_RIGHT) && (keycode_onoff[MASK_KEY_RIGHT] == SHTRPD_SNAP_OFF_ON)) {
                if((keycode[MASK_KEY_ENTER] == KEY_ENTER) && (keycode_onoff[MASK_KEY_ENTER] == SHTRPD_SNAP_ON_ON)) {
                    if(shtrpd_key_mask_count[MASK_KEY_RIGHT] < SHTRPD_SNAP_MASK_COUNT) {
                        keycode[MASK_KEY_RIGHT] = 0;
                        keycode_onoff[MASK_KEY_RIGHT] = 0;
                        shtrpd_key_mask_count[MASK_KEY_RIGHT]++;
                        SHTRPD_LOG_SNAP_MASK("KEY:KEY_RIGHT Masked\n");
                    }
                }
            }
            if ((keycode[MASK_KEY_DNLEFT] == WK_DNLEFT) && (keycode_onoff[MASK_KEY_DNLEFT] == SHTRPD_SNAP_OFF_ON)) {
                if((keycode[MASK_KEY_ENTER] == KEY_ENTER) && (keycode_onoff[MASK_KEY_ENTER] == SHTRPD_SNAP_ON_ON)) {
                    if(shtrpd_key_mask_count[MASK_KEY_DNLEFT] < SHTRPD_SNAP_MASK_COUNT) {
                        keycode[MASK_KEY_DNLEFT] = 0;
                        keycode_onoff[MASK_KEY_DNLEFT] = 0;
                        shtrpd_key_mask_count[MASK_KEY_DNLEFT]++;
                        SHTRPD_LOG_SNAP_MASK("KEY:WK_DNLEFT Masked\n");
                    }
                }
            }
            if ((keycode[MASK_KEY_DOWN] == KEY_DOWN) && (keycode_onoff[MASK_KEY_DOWN] == SHTRPD_SNAP_OFF_ON)) {
                if((keycode[MASK_KEY_ENTER] == KEY_ENTER) && (keycode_onoff[MASK_KEY_ENTER] == SHTRPD_SNAP_ON_ON)) {
                    if(shtrpd_key_mask_count[MASK_KEY_DOWN] < SHTRPD_SNAP_MASK_COUNT) {
                        keycode[MASK_KEY_DOWN] = 0;
                        keycode_onoff[MASK_KEY_DOWN] = 0;
                        shtrpd_key_mask_count[MASK_KEY_DOWN]++;
                        SHTRPD_LOG_SNAP_MASK("KEY:KEY_DOWN Masked\n");
                    }
                }
            }
            if ((keycode[MASK_KEY_DNRIGHT] == WK_DNRIGHT) && (keycode_onoff[MASK_KEY_DNRIGHT] == SHTRPD_SNAP_OFF_ON)) {
                if((keycode[MASK_KEY_ENTER] == KEY_ENTER) && (keycode_onoff[MASK_KEY_ENTER] == SHTRPD_SNAP_ON_ON)) {
                    if(shtrpd_key_mask_count[MASK_KEY_DNRIGHT] < SHTRPD_SNAP_MASK_COUNT) {
                        keycode[MASK_KEY_DNRIGHT] = 0;
                        keycode_onoff[MASK_KEY_DNRIGHT] = 0;
                        shtrpd_key_mask_count[MASK_KEY_DNRIGHT]++;
                        SHTRPD_LOG_SNAP_MASK("KEY:WK_DNRIGHT Masked\n");
                    }
                }
            }
        }
    }
}
#endif /* SHTRPD_SNAP_MASK */

/* ------------------------------------------------------------------------- */
/* shtrpd_snap_input_event                                                   */
/* ------------------------------------------------------------------------- */
static void shtrpd_snap_input_event(void *dev_id, u16 keycode, u8 onoff)
{
    struct shtrpd_ts_data *data = dev_id;

    if(WK_UPRIGHT == keycode){
        input_event(data->input_dev, EV_KEY, KEY_UP,    onoff);
        input_event(data->input_dev, EV_KEY, KEY_RIGHT, onoff);
        SHTRPD_SNAP("Snap %d KEY_UP + KEY_RIGHT\n",  onoff);
    }
    else if(WK_UPLEFT  == keycode){
        input_event(data->input_dev, EV_KEY, KEY_UP,    onoff);
        input_event(data->input_dev, EV_KEY, KEY_LEFT,  onoff);
        SHTRPD_SNAP("Snap %d KEY_UP + KEY_LEFT\n",   onoff);
    }
    else if(WK_DNRIGHT == keycode){
        input_event(data->input_dev, EV_KEY, KEY_DOWN,  onoff);
        input_event(data->input_dev, EV_KEY, KEY_RIGHT, onoff);
        SHTRPD_SNAP("Snap %d KEY_DOWN + KEY_RIGHT\n", onoff);
    }
    else if(WK_DNLEFT  == keycode){
        input_event(data->input_dev, EV_KEY, KEY_DOWN,  onoff);
        input_event(data->input_dev, EV_KEY, KEY_LEFT,  onoff);
        SHTRPD_SNAP("Snap %d KEY_DOWN + KEY_LEFT\n", onoff);
    }
    else{
        input_event(data->input_dev, EV_KEY, (int)keycode, onoff);
        SHTRPD_SNAP("Snap %d key=%d\n", onoff, keycode);
    }
    input_sync(data->input_dev);
}

#if 1
/* ------------------------------------------------------------------------- */
/* shtrpd_snap                                                               */
/* ------------------------------------------------------------------------- */
static u8 shtrpd_snap(void *dev_id, u8 snap_output, u8 *psnap)
{
    u8 wk_snap;
    u8 i,j;
    u16 keycode[MAX_SNAPS];
    u8 keycode_onoff[MAX_SNAPS];
    u8 keycnt=0;
    u8 ret = 0;

    /*  Snap output on  */
    if (snap_output){
        SHTRPD_SNAP("Snap output start\n");

        memset(keycode, 0, sizeof(keycode));
        memset(keycode_onoff, 0, sizeof(keycode_onoff));

        for(i=0; i<SHTRPD_XY_SNAP_NUM ; i++){
            wk_snap = psnap[i];
            for(j=0; j<8; j++){
                if(wk_snap & (1<<j)) {
                    if(shtrpd_snap_to_keycode[i][j] != -1) {
                        keycode[keycnt] = (u16)shtrpd_snap_to_keycode[i][j];
                        SHTRPD_SNAP("Snap %c%d \n", 'A'+i, j);
                    }
                }
                keycnt++;
            }
        }

#if defined(SHTRPD_SNAP_MASK)
        shtrpd_snap_mask_diagonal_key(keycode);
#endif /* SHTRPD_SNAP_MASK */

#if defined(SHTRPD_SNAP_USER_MASK)
        shtrpd_snap_user_mask_check(keycode);
#endif /* SHTRPD_SNAP_USER_MASK */

        for(i=0; i<MAX_SNAPS; i++) {
            if(keycode[i]){
                if(old_keycode[i] == 0) {
                    keycode_onoff[i] = SHTRPD_SNAP_OFF_ON;
                } else {
                    keycode_onoff[i] = SHTRPD_SNAP_ON_ON;
                }
            } else {
                if(old_keycode[i] == 0) {
                    keycode_onoff[i] = 0;
                } else {
                    keycode_onoff[i] = SHTRPD_SNAP_ON_OFF;
                }
            }
        }

#if defined(SHTRPD_SNAP_MASK)
        shtrpd_snap_chattering_removal(keycode, keycode_onoff);
        shtrpd_snap_mask_check(keycode, keycode_onoff);
#endif /* SHTRPD_SNAP_MASK */

        SHTRPD_SNAP("Snap keycode0 =%d,%d,%d,%d,%d,%d,%d,%d : keycode_onoff[i]=%d,%d,%d,%d,%d,%d,%d,%d\n",
                    keycode[0],keycode[1],keycode[2],keycode[3],keycode[4],keycode[5],keycode[6],keycode[7],
                    keycode_onoff[0],keycode_onoff[1],keycode_onoff[2],keycode_onoff[3],keycode_onoff[4],
                    keycode_onoff[5],keycode_onoff[6],keycode_onoff[7]);
        SHTRPD_SNAP("Snap keycode1 =%d,%d,%d,%d,%d,%d,%d,%d : keycode_onoff[i]=%d,%d,%d,%d,%d,%d,%d,%d\n",
                    keycode[8],keycode[9],keycode[10],keycode[11],keycode[12],keycode[13],keycode[14],keycode[15],
                    keycode_onoff[8],keycode_onoff[9],keycode_onoff[10],keycode_onoff[11],keycode_onoff[12],
                    keycode_onoff[13],keycode_onoff[14],keycode_onoff[15]);
        SHTRPD_SNAP("Snap keycode2 =%d,%d,%d,%d,%d,%d,%d,%d : keycode_onoff[i]=%d,%d,%d,%d,%d,%d,%d,%d\n",
                    keycode[16],keycode[17],keycode[18],keycode[19],keycode[20],keycode[21],keycode[22],keycode[23],
                    keycode_onoff[16],keycode_onoff[17],keycode_onoff[18],keycode_onoff[19],keycode_onoff[20],
                    keycode_onoff[21],keycode_onoff[22],keycode_onoff[23]);
        SHTRPD_SNAP("Snap keycode3 =%d,%d,%d,%d,%d,%d,%d,%d : keycode_onoff[i]=%d,%d,%d,%d,%d,%d,%d,%d\n",
                    keycode[24],keycode[25],keycode[26],keycode[27],keycode[28],keycode[29],keycode[30],keycode[31],
                    keycode_onoff[24],keycode_onoff[25], keycode_onoff[26],keycode_onoff[27],keycode_onoff[28],
                    keycode_onoff[29],keycode_onoff[30], keycode_onoff[31]);

        SHTRPD_SNAP("Snap old_keycode0=%d,%d,%d,%d,%d,%d,%d,%d : old_keycode_onoff[i]=%d,%d,%d,%d,%d,%d,%d,%d\n",
                    old_keycode[0],old_keycode[1],old_keycode[2],old_keycode[3],old_keycode[4],
                    old_keycode[5],old_keycode[6],old_keycode[7],
                    old_keycode_onoff[0],old_keycode_onoff[1],old_keycode_onoff[2],old_keycode_onoff[3],
                    old_keycode_onoff[4],old_keycode_onoff[5],old_keycode_onoff[6],old_keycode_onoff[7]);
        SHTRPD_SNAP("Snap old_keycode0=%d,%d,%d,%d,%d,%d,%d,%d : old_keycode_onoff[i]=%d,%d,%d,%d,%d,%d,%d,%d\n",
                    old_keycode[8],old_keycode[9],old_keycode[10],old_keycode[11],old_keycode[12],
                    old_keycode[13],old_keycode[14],old_keycode[15],
                    old_keycode_onoff[8],old_keycode_onoff[9],old_keycode_onoff[10],old_keycode_onoff[11],
                    old_keycode_onoff[12],old_keycode_onoff[13],old_keycode_onoff[14],old_keycode_onoff[15]);
        SHTRPD_SNAP("Snap old_keycode0=%d,%d,%d,%d,%d,%d,%d,%d : old_keycode_onoff[i]=%d,%d,%d,%d,%d,%d,%d,%d\n",
                    old_keycode[16],old_keycode[17],old_keycode[18],old_keycode[19],old_keycode[20],
                    old_keycode[21],old_keycode[22],old_keycode[23],
                    old_keycode_onoff[16],old_keycode_onoff[17],old_keycode_onoff[18],old_keycode_onoff[19],
                    old_keycode_onoff[20],old_keycode_onoff[21],old_keycode_onoff[22],old_keycode_onoff[23]);
        SHTRPD_SNAP("Snap old_keycode0=%d,%d,%d,%d,%d,%d,%d,%d : old_keycode_onoff[i]=%d,%d,%d,%d,%d,%d,%d,%d\n",
                    old_keycode[24],old_keycode[25],old_keycode[26],old_keycode[27],old_keycode[28],
                    old_keycode[29],old_keycode[30],old_keycode[31],
                    old_keycode_onoff[24],old_keycode_onoff[25],old_keycode_onoff[26],old_keycode_onoff[27],
                    old_keycode_onoff[28],old_keycode_onoff[29],old_keycode_onoff[30],old_keycode_onoff[31]);

        /* check input event EV_KEY off */
        for(i=0; i<MAX_SNAPS; i++){
            if(keycode[i]) {
                if(keycode_onoff[i] == SHTRPD_SNAP_OFF_ON) {
                    /* input event KEY off->on */
                    shtrpd_snap_input_event(dev_id, keycode[i], 1);
                }
            } else {
                if(keycode_onoff[i] == SHTRPD_SNAP_ON_OFF) {
                    /* input event KEY on->off */
                    shtrpd_snap_input_event(dev_id, old_keycode[i], 0);
                }
            }
        }

        memcpy( old_keycode, keycode, sizeof(old_keycode));
        memcpy( old_keycode_onoff, keycode_onoff, sizeof(old_keycode_onoff));
        SHTRPD_SNAP("Snap output end\n");
    }
    /*  Snap output off */
    else {
        /* check input event EV_KEY off */
        for(i=0; i<MAX_SNAPS; i++){
            if( old_keycode[i] && 
               ((SHTRPD_SNAP_OFF_ON == old_keycode_onoff[i]) || 
                (SHTRPD_SNAP_ON_ON  == old_keycode_onoff[i]))){
                /* on->off */
                old_keycode_onoff[i] = SHTRPD_SNAP_ON_OFF;
                /* input event KEY on->off */
                shtrpd_snap_input_event(dev_id, old_keycode[i], 0);
            }
        }

        memset( old_keycode, 0, sizeof(old_keycode));
        memset( old_keycode_onoff, 0, sizeof(old_keycode_onoff));
#if defined(SHTRPD_SNAP_MASK)
        memset( shtrpd_key_mask_count, 0, sizeof(shtrpd_key_mask_count));
        memset( shtrpd_key_chattering_removal_count, 0, 
                sizeof(shtrpd_key_chattering_removal_count));
#endif /* SHTRPD_SNAP_MASK */
    }
    /* Snap completed */
    ret = 1;
    
    SHTRPD_DBG("Snap end\n");
    return ret;
}
#else
/* ------------------------------------------------------------------------- */
/* shtrpd_snap                                                               */
/* ------------------------------------------------------------------------- */
static u8 shtrpd_snap(void *dev_id, u8 snap_output)
{
    struct shtrpd_ts_data *data = dev_id;
    struct i2c_client *client = data->client;
    u8 buffer[40];
    int err;
    u8 i,j;
    ushort snap_rx = 0; 
    u16 keycode[5];
    u8 keycnt=0;
    u8 keycode_onoff[5];
    u8 ret = 0;
    u8 snap_off_find=0;
#if 0
    int oft=0;
    u8 buf[TOTALTXS_MAX*9*2];
#endif

    /*  Snap output on  */
    if (snap_output){
        SHTRPD_DBG("Snap output start\n");

        err = shtrpd_i2c_user_read_block_data(client, SNAP_STATUS, 
                                                (TOTALTXS_MAX*2), buffer);
        if (err < 0) {
            /* Only for debugging purposes  */
            SHTRPD_ERR("i2c read err[%d]\n", err);
            goto snapexit;
        }

#if 1
        SHTRPD_INFO("SNAP_STATUS Tx0 =%02x%02x Tx1 =%02x%02x Tx2 =%02x%02x Tx3 =%02x%02x \n",
                    buffer[0],buffer[1],buffer[2],buffer[3],
                    buffer[4],buffer[5],buffer[6],buffer[7]);

        SHTRPD_INFO("SNAP_STATUS Tx4 =%02x%02x Tx5 =%02x%02x Tx6 =%02x%02x Tx7 =%02x%02x \n",
                    buffer[8], buffer[ 9],buffer[10],buffer[11],
                    buffer[12],buffer[13],buffer[14],buffer[15]);

        SHTRPD_INFO("SNAP_STATUS Tx8 =%02x%02x Tx9 =%02x%02x Tx10=%02x%02x Tx11=%02x%02x \n",
                    buffer[16],buffer[17],buffer[18],buffer[19],
                    buffer[20],buffer[21],buffer[22],buffer[23]);

        SHTRPD_INFO(" SNAP_STATUS Tx12=%02x%02x Tx13=%02x%02x Tx14=%02x%02x \n",
                    buffer[24],buffer[25],buffer[26],buffer[27],
                    buffer[28],buffer[29]);
#endif

        memset(keycode, 0, sizeof(keycode));
        memset(keycode_onoff, 0, sizeof(keycode_onoff));

        for(i=0; i<TOTALTXS_MAX ; i++){
            snap_rx = ((ushort)(buffer[(i*2)] << 8)) | 
                       (ushort) buffer[(i*2)+1];
            for(j=0; j<TOTALRXS_MAX; j++){
                if(( snap_rx & (1<<j)) && 
                    (shtrpd_keycode_table_addr[i][TOTALRXS_MAX-1-j] != -1)){
                    keycode[keycnt] = (u16)shtrpd_keycode_table_addr[i][TOTALRXS_MAX-1-j];
                    SHTRPD_DBG("Snap Tx=%d Rx=%d\n", i, j);
                    keycnt++;
                }
            }
        }
        SHTRPD_DBG("Snap count=%d\n", keycnt);

        memset(old_keycode_onoff, 0, sizeof(old_keycode_onoff));

        for(i=0; i<MAX_SNAPS; i++){
            for(j=0; j<MAX_TOUCHES; j++){
                if(( keycode[i] == old_keycode[j]) && 
                   (0 != keycode[i]) && 
                   (0 != old_keycode[j])){
                    /* on->on */
                    keycode_onoff[i] = SHTRPD_SNAP_ON_ON;
                    old_keycode_onoff[j] = SHTRPD_SNAP_ON_ON;
                    break;
                }
            }

            if((0 != keycode[i]) && (!keycode_onoff[i])){
                /* off->on */
                keycode_onoff[i] = SHTRPD_SNAP_OFF_ON;
            }
            if((0 != old_keycode[i]) && 
                (SHTRPD_SNAP_ON_ON != old_keycode_onoff[i])){
                /* on->off */
                old_keycode_onoff[i] = SHTRPD_SNAP_ON_OFF;
            }
        }

        SHTRPD_DBG("Snap keycode[i]    =%d,%d,%d,%d,%d : keycode_onoff[i]=%d,%d,%d,%d,%d\n",
                    keycode[0],keycode[1],keycode[2],keycode[3],keycode[4],
                    keycode_onoff[0],keycode_onoff[1],
                    keycode_onoff[2],keycode_onoff[3],
                    keycode_onoff[4]);
        SHTRPD_DBG("Snap old_keycode[i]=%d,%d,%d,%d,%d : old_keycode_onoff[i]=%d,%d,%d,%d,%d\n",
                    old_keycode[0],old_keycode[1],
                    old_keycode[2],old_keycode[3],
                    old_keycode[4],
                    old_keycode_onoff[0],old_keycode_onoff[1],
                    old_keycode_onoff[2],old_keycode_onoff[3],
                    old_keycode_onoff[4]);

        /* check input event EV_KEY off */
        for(i=0; i<MAX_SNAPS; i++){
            if( old_keycode[i] && 
               (SHTRPD_SNAP_ON_OFF == old_keycode_onoff[i])){
                /* input event KEY on->off */
                shtrpd_snap_input_event(dev_id, old_keycode[i], 0);
            }
        }

        /* check input event EV_KEY on */
        for(i=0; i<MAX_SNAPS; i++){
            if( keycode[i] && 
               (1 == keycode_onoff[i])){
                /* input event KEY off->on */
                shtrpd_snap_input_event(dev_id, keycode[i], 1);
            }
        }

        memcpy( old_keycode, keycode, sizeof(old_keycode));
        memcpy( old_keycode_onoff, keycode_onoff, sizeof(old_keycode_onoff));

        SHTRPD_DBG("Snap output end\n");

    }
    /*  Snap output off */
    else {
        /* check input event EV_KEY off */
        for(i=0,snap_off_find=0; i<MAX_SNAPS; i++){
            if( old_keycode[i] && 
               ((SHTRPD_SNAP_OFF_ON == old_keycode_onoff[i]) || 
                (SHTRPD_SNAP_ON_ON  == old_keycode_onoff[i]))){
                /* on->off */
                old_keycode_onoff[i] = SHTRPD_SNAP_ON_OFF;
                /* input event KEY on->off */
                shtrpd_snap_input_event(dev_id, old_keycode[i], 0);
                /* snap off find */
                snap_off_find = 1;
            }
        }
        if( 1 == snap_off_find ){
            memset( old_keycode, 0, sizeof(old_keycode));
            memset( old_keycode_onoff, 0, sizeof(old_keycode_onoff));
        }
    }
    /* Snap completed */
    ret = 1;
    
snapexit:
    SHTRPD_DBG("Snap end\n");
    return ret;
}
#endif

/* ------------------------------------------------------------------------- */
/* shtrpd_user_request                                                       */
/* ------------------------------------------------------------------------- */
static void shtrpd_user_request(void)
{
    switch(userRequest.read_write){
    case SHTRPD_I2C_REQ_READ_BYTE:
        userRequest.ret = shtrpd_i2c_read_byte_data(
                                          userRequest.client,
                                          userRequest.cmd,
                                          userRequest.pbuf);
        break;

    case SHTRPD_I2C_REQ_READ_BLOCK:
        userRequest.ret = shtrpd_i2c_read_block_data(
                                          userRequest.client,
                                          userRequest.cmd,
                                          userRequest.length,
                                          userRequest.pbuf);
        break;

    case SHTRPD_I2C_REQ_WRITE_BYTE:
        userRequest.ret = shtrpd_i2c_write_byte_data(
                                          userRequest.client,
                                          userRequest.cmd,
                                          userRequest.values);
        break;

    case SHTRPD_I2C_REQ_WRITE_BLOCK:
        userRequest.ret = shtrpd_i2c_write_block_data(
                                          userRequest.client,
                                          userRequest.cmd,
                                          userRequest.length,
                                          userRequest.pbuf);
        break;

    default:
        userRequest.ret = -1;
        break;
    }
    userRequest.req_flag = 0;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_snap_err_delay_func                                                */
/* ------------------------------------------------------------------------- */
static void shtrpd_snap_err_delay_func(struct work_struct *work)
{
    mutex_lock(&shtrpd_ioctl_lock);

    SHTRPD_DBG("START, flip_state:%d, initialSetup:%d\n", shtrpd_flip_state, initialSetup);

    if(!shtrpd_flip_state){
        mutex_unlock(&shtrpd_ioctl_lock);
        return;
    }

    if (initialSetup) {
        mutex_unlock(&shtrpd_ioctl_lock);
        return;
    }

    shtrpd_event_mode_handshake();

    mutex_unlock(&shtrpd_ioctl_lock);
}

/* ------------------------------------------------------------------------- */
/* msm_trpd_set_poweron                                                      */
/* ------------------------------------------------------------------------- */
void msm_trpd_set_poweron(void)
{
#if defined(SHTRPD_NOINTERRUPT_IN_TOUCH_CHECK_ENABLE)

    SHTRPD_DBG("msm_trpd_set_poweron start\n");
#ifdef SHTRPD_ENABLE_INIT_FW_DL
    if (shtrpd_fwdl_fail_flg) {
        SHTRPD_ERR("init fwdl failed\n");
        return;
    }
#endif /* SHTRPD_ENABLE_INIT_FW_DL */

    if (!shtrpd_work_queue) {
        SHTRPD_ERR("shtrpd_work_queue NULL failed\n");
        return;
    }
	queue_delayed_work(shtrpd_work_queue, &shtrpd_work_pwkey_delay, 
	                   msecs_to_jiffies(10));

#endif /* SHTRPD_NOINTERRUPT_IN_TOUCH_CHECK_ENABLE */
}

#if defined(SHTRPD_NOINTERRUPT_IN_TOUCH_CHECK_ENABLE)
/* ------------------------------------------------------------------------- */
/* shtrpd_pwkey_delay_func                                                   */
/* ------------------------------------------------------------------------- */
static void shtrpd_pwkey_delay_func(struct work_struct *work)
{
    SHTRPD_DBG("shtrpd_pwkey_delay_func start\n");

#ifdef SHTRPD_ENABLE_INIT_FW_DL
    if (shtrpd_fwdl_fail_flg) {
        SHTRPD_ERR("init fwdl failed\n");
        return;
    }
#endif /* SHTRPD_ENABLE_INIT_FW_DL */

    if (!shtrpd_work_queue) {
        SHTRPD_ERR("shtrpd_work_queue NULL failed\n");
        return;
    }

#if defined( SHTRPD_CHECK_CPU_TEMP_ENABLE )
    mutex_lock(&shtrpd_ioctl_lock);
    if(shtrpd_flip_state) {
        if(SHTRPD_CHECK_CPU_TEMP){
	        SHTRPD_LOG_CHECK_CPU_TEMP("cpu temp check timer start\n");
            cancel_delayed_work(&shtrpd_work_cpu_temp);
            queue_delayed_work(shtrpd_work_queue, &shtrpd_work_cpu_temp, msecs_to_jiffies(SHTRPD_CHECK_CPU_TEMP_INTERVAL));
            shtrpd_thresholds_check_cpu_temp(0);
        }
    }
    mutex_unlock(&shtrpd_ioctl_lock);
#endif /* SHTRPD_CHECK_CPU_TEMP_ENABLE */

#if defined(SHTRPD_INTERRUPT_TIMING_CHECK_ENABLE)
    if(SHTRPD_PWKEY_FORCE_REATI_ENABLE){
        force_reati_req = 1;
        queue_delayed_work(shtrpd_work_queue, &shtrpd_work_int_timing_delay, 
                            msecs_to_jiffies(5));
    }
    else
#endif /* SHTRPD_INTERRUPT_TIMING_CHECK_ENABLE */
    {
	    mutex_lock(&shtrpd_noint_check_delay_lock);
        shtrpd_noint_check_state = 2;
   	    queue_delayed_work(shtrpd_work_queue, &shtrpd_work_noint_check_delay, 
	                   msecs_to_jiffies(20));

	    mutex_unlock(&shtrpd_noint_check_delay_lock);
    }
}

/* ------------------------------------------------------------------------- */
/* shtrpd_nonint_check_clr                                                   */
/* ------------------------------------------------------------------------- */
static void shtrpd_nonint_check_clr(void)
{
	mutex_lock(&shtrpd_noint_check_delay_lock);
    shtrpd_noint_check_state = 0;
    SHTRPD_DBG("Clear no interrupt check mode!!!\n");
	mutex_unlock(&shtrpd_noint_check_delay_lock);
}

/* ------------------------------------------------------------------------- */
/* shtrpd_nonint_check_delay_func                                            */
/* ------------------------------------------------------------------------- */
static void shtrpd_nonint_check_delay_func(struct work_struct *work)
{
    int ret;
    char m_buf[10];

	mutex_lock(&shtrpd_noint_check_delay_lock);
    if(0 != shtrpd_noint_check_state){
        SHTRPD_DBG("no interrupt, version read start\n");
        ret = shtrpd_i2c_user_read_block_data(shtrpd_client, VERSION_INFO, 
                                            SHTRPD_FW_VER_SIZE, &m_buf[0]);

        if(ret < 0){
            SHTRPD_ERR("VERSION_INFO error(ret=%d) \n",ret);
        }
        shtrpd_noint_check_state = 0;
    }
	mutex_unlock(&shtrpd_noint_check_delay_lock);
}
/* ------------------------------------------------------------------------- */
/* shtrpd_nonint_check_delay_start                                           */
/* ------------------------------------------------------------------------- */
static void shtrpd_nonint_check_delay_start(void)
{

	mutex_lock(&shtrpd_noint_check_delay_lock);
    SHTRPD_DBG("shtrpd_nonint_check_delay_start\n");
    shtrpd_noint_check_state = 1;
	queue_delayed_work(shtrpd_work_queue, &shtrpd_work_noint_check_delay, 
	                   msecs_to_jiffies(SHTRPD_NOINTERRUPT_IN_TOUCH_CHECK_DELAY));

	mutex_unlock(&shtrpd_noint_check_delay_lock);
}
/* ------------------------------------------------------------------------- */
/* shtrpd_nonint_check_delay_cancel                                          */
/* ------------------------------------------------------------------------- */
static void shtrpd_nonint_check_delay_cancel(void)
{

	mutex_lock(&shtrpd_noint_check_delay_lock);
    SHTRPD_DBG("shtrpd_nonint_check_delay_cancel\n");
    shtrpd_noint_check_state = 0;
    cancel_delayed_work(&shtrpd_work_noint_check_delay);
	mutex_unlock(&shtrpd_noint_check_delay_lock);
}
#endif /* SHTRPD_NOINTERRUPT_IN_TOUCH_CHECK_ENABLE */

/* ------------------------------------------------------------------------- */
/* shtrpd_ts_interrupt                                                       */
/* ------------------------------------------------------------------------- */
/**
 *  Interrupt event fires on the rising edge of
 *  the RDY signal from the Trackpad Device
 */
static irqreturn_t shtrpd_ts_interrupt(int irq, void *dev_id)
{
    struct shtrpd_ts_data *data = dev_id;
    struct i2c_client *client = data->client;
    u8 buffer[40];
    int err;
    u8 noOfTouches = 0;
    u8 snap_output = 0;
    u8 ret = 0;
    int i;

    if(!shtrpd_flip_state){
        goto out;
    }

    if(bootLoaderMode){
        goto out;
    }

    /* Do the initial setup for the Trackpad device
     * Because we have to wait for the RDY from the Trackpad device
     * the setup will take a few comms cycles
     */
    if (initialSetup) {
        
        /* Do the initial setup for the Trackpad device */
        shtrpd_initialSetup(dev_id);

        /*  Jump out and wait for next window   */
        goto out;
    }

    /* Re-ATI */
    if (!initialATI) {
        shtrpd_reati(data);
        initialATI = true;
        goto out;
    }

    if(firmWritting){
        goto out;
    }

    /* Setup done */

#if defined(SHTRPD_CPU_IDLE_SLEEP_CONTROL_ENABLE)
	shtrpd_wake_lock_idle();
#endif /* SHTRPD_CPU_IDLE_SLEEP_CONTROL_ENABLE */

    SHTRPD_DBG("interrupt start \n");

    /* User request process */
    if(userRequest.req_flag){
        goto out;
    }

#if defined(SHTRPD_NOINTERRUPT_IN_TOUCH_CHECK_ENABLE)
    if(1 == shtrpd_noint_check_state){
        shtrpd_nonint_check_clr();
    }
#endif /* SHTRPD_NOINTERRUPT_IN_TOUCH_CHECK_ENABLE */

    /*************************************************/
    /*  Read the Touches and Handle Accordingly      */
    /*************************************************/

    err = shtrpd_i2c_read_block_data(client, XY_DATA,
        SHTRPD_BLOCK_SIZE, buffer);
    if (err < 0) {
        /* Only for debugging purposes  */
        SHTRPD_ERR("read touches err[%d] goto initialSetup.. \n", err);
        if(1 == shtrpd_i2cerror_reset_enable){
            SHTRPD_ERR("reset setting exec \n");
            shtrpd_reset_setting();
        }
        goto out;
    }

    firstXYInfoRead = true;

#if defined(SHTRPD_INTERRUPT_TIMING_CHECK_ENABLE)
    shtrpd_int_timing_check(buffer);
#endif /* SHTRPD_INTERRUPT_TIMING_CHECK_ENABLE */

    if (buffer[0] & SNAP_OUTPUT) {
        if ((buffer[1] == 0) && (buffer[2] == 0) && (buffer[3] == 0) && (buffer[4] == 0)) {
            SHTRPD_ERR("SNAP_OUTPUT bit Err\n");
            queue_delayed_work(shtrpd_work_queue, &shtrpd_work_snap_err_delay, msecs_to_jiffies(10));
            goto out;
        }
    }

    /*  Report the number of fingers on the screen  */
    noOfTouches = buffer[0] & NO_OF_FINGERS;
    /*  Check whether an unsuspected reset occured - if a reset
     *  occured, redo the initial setup of the Trackpad device for
     *  optimal performance
     */
    reset = buffer[0] & SHOW_RESET;
    /*  Now, redo setup */
    if (reset) {
        SHTRPD_ERR("SHOW_RESET goto initialSetup.. \n");
        gpio_set_value(NRST_LINE_PIN,0);
        usleep(100000);
        gpio_set_value(NRST_LINE_PIN,1);
        usleep(SHTRPD_NRST_DELAY);
        shtrpd_ts_phys_init();
        goto out;
    }

#if 1
    SHTRPD_DBG("noOfTouches=%d\n",noOfTouches);

    SHTRPD_INFO("XY_DATA  XYInfo=0x%02x \n",buffer[0]);
    SHTRPD_INFO("XY_DATA  snap=0x%02x 0x%02x 0x%02x 0x%02x \n",buffer[1],buffer[2],buffer[3],buffer[4]);
    SHTRPD_INFO("XY_DATA  X=0x%02x%02x Y=0x%02x%02x TS=0x%02x%02x AREA=0x%02x \n",
                buffer[5],buffer[6],buffer[7],  buffer[8],buffer[9],buffer[10],buffer[11]);

    SHTRPD_INFO("XY_DATA  X=0x%02x%02x Y=0x%02x%02x TS=0x%02x%02x AREA=0x%02x \n",
                buffer[12],buffer[13],buffer[14],buffer[15],buffer[16],buffer[17],buffer[18]);

    SHTRPD_INFO("XY_DATA  X=0x%02x%02x Y=0x%02x%02x TS=0x%02x%02x AREA=0x%02x \n",
                buffer[19],buffer[20],buffer[21],buffer[22],buffer[23], buffer[24],buffer[25]);

    SHTRPD_INFO("XY_DATA  X=0x%02x%02x Y=0x%02x%02x TS=0x%02x%02x AREA=0x%02x \n",
                buffer[26],buffer[27],buffer[28],buffer[29],buffer[30],buffer[31],buffer[32]);

    SHTRPD_INFO("XY_DATA  X=0x%02x%02x Y=0x%02x%02x TS=0x%02x%02x AREA=0x%02x \n",
                buffer[33],buffer[34],buffer[35],buffer[36],buffer[37],buffer[38],buffer[39]);
#endif

    if(0 != shtrpd_touchevent_sw){
        /*  multiple touches */
        shtrpd_multiple_touches(dev_id, buffer, noOfTouches);
    }

    if(!shtrpd_touchcruiser_state){
        for(i=0; i<MAX_TOUCHES; i++){
            if( shtrpd_finger_state_old & (1<<i)){
                input_mt_slot(data->input_dev, i);
                input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, -1);
                input_sync(data->input_dev);
                SHTRPD_DBG("ABS_MT_SLOT slot=%d\n",i);
            }
        }
    }

    /*  Report Snap output bit */
    snap_output = buffer[0] & SNAP_OUTPUT;

    /* Snap */
    ret = shtrpd_snap(dev_id, snap_output, &buffer[1]);
    if( !ret){
        SHTRPD_ERR("shtrpd_snap error[%d] goto initialSetup.. \n", ret);
        if(1 == shtrpd_i2cerror_reset_enable){
            SHTRPD_ERR("reset setting exec \n");
            shtrpd_reset_setting();
        }
        goto out;
    }

out:
    SHTRPD_DBG("interrupt end\n");

#if defined(SHTRPD_CPU_IDLE_SLEEP_CONTROL_ENABLE)
	shtrpd_wake_unlock_idle();
#endif /* SHTRPD_CPU_IDLE_SLEEP_CONTROL_ENABLE */

    return IRQ_HANDLED;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_initialSetup_complete_check                                        */
/* ------------------------------------------------------------------------- */
static s8 shtrpd_initialSetup_complete_check(void)
{
    u32 i;

    /* wait the initial setup for the Trackpad device */
    for(i=0; i<SHTRPD_INITIALSETUP_RETRY_MAX; i++){
        if(initialSetup){
            usleep(SHTRPD_INITIALSETUP_COMPLETE_DELAY);
        }
        else{
            return 0;
        }
    }
    return -1;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_firstXYInfoRead_complete_check                                    */
/* ------------------------------------------------------------------------- */
#ifdef SHTRPD_DBG_INITIAL_BOOT_CHK
static s8 shtrpd_firstXYInfoRead_complete_check(void)
{
    u32 i;

    /* wait the first XY-Info read for the Trackpad device */
    for(i=0; i<SHTRPD_FIRSTXYINFOREAD_RETRY_MAX; i++){
        if(!firstXYInfoRead){
            usleep(SHTRPD_FIRSTXYINFOREAD_COMPLETE_DELAY);
        }
        else{
            return 0;
        }
    }
    return -1;
}
#endif

/* ------------------------------------------------------------------------- */
/* shtrpd_ioctl_get_fw_version                                               */
/* ------------------------------------------------------------------------- */
static long shtrpd_ioctl_get_fw_version(struct file *filp, unsigned int cmd, 
                                                           unsigned long arg)
{
    void __user *argp = (void __user *)arg;
    int32_t ret = -1;
    struct IoctlDiagCmdReq cmdreq;

    SHTRPD_DBG("SHTRPDIO_GET_FW_VERSION \n");

    cmdreq.m_Cmd = VERSION_INFO;
    cmdreq.m_req_size = SHTRPD_FW_VER_SIZE;

    if(!shtrpd_flip_state) {
        if(shtrpd_flip_reset_state) {
            gpio_set_value(NRST_LINE_PIN,1);
            usleep(SHTRPD_NRST_DELAY);
            shtrpd_ts_phys_init();
        }
        shtrpd_flip_state = 1;
        shtrpd_sys_enable_irq();

        ret = shtrpd_i2c_user_read_block_data(shtrpd_client, cmdreq.m_Cmd, 
                                            cmdreq.m_req_size, &cmdreq.m_buf[0]);

        shtrpd_sys_disable_irq();
        shtrpd_flip_state = 0;

        if(shtrpd_flip_reset_state) {
            gpio_set_value(NRST_LINE_PIN, 0);
        }
    } else {
        ret = shtrpd_i2c_user_read_block_data(shtrpd_client, cmdreq.m_Cmd, 
                                            cmdreq.m_req_size, &cmdreq.m_buf[0]);
    }
    cmdreq.rtn = ret;
    cmdreq.m_res_size = cmdreq.m_req_size;

    if( ret >= 0 ){
        ret = copy_to_user(argp, &cmdreq.m_buf[0], SHTRPD_FW_VER_SIZE);
        if (ret) {
            SHTRPD_ERR("error(copy_to_user) : shtrpd_ioctl(cmd = SHTRPDIO_GET_FW_VERSION)\n" );
            return -EFAULT;
        }
        SHTRPD_DBG("Version:Product=%02x%02x, Project=%02x%02x\n", 
                cmdreq.m_buf[0], cmdreq.m_buf[1],cmdreq.m_buf[2], cmdreq.m_buf[3]);
        SHTRPD_DBG("Version:Major=%02x, Minor=%02x, HWID=%02x%02x, HWRV=%02x%02x\n", 
                cmdreq.m_buf[4], cmdreq.m_buf[5],cmdreq.m_buf[6], cmdreq.m_buf[7],
                cmdreq.m_buf[8], cmdreq.m_buf[9]);
    }
    else{
        SHTRPD_ERR("i2c read error : shtrpd_ioctl(cmd = SHTRPDIO_GET_FW_VERSION)\n" );
        SHTRPD_DBG("return code =%d response=%02x\n", ret, cmdreq.m_buf[0] );

        SHTRPD_DBG("Version:Product=%02x%02x, Project=%02x%02x\n", 
                cmdreq.m_buf[1], cmdreq.m_buf[2],cmdreq.m_buf[3], cmdreq.m_buf[4]);
        SHTRPD_DBG("Version:Major=%02x, Minor=%02x, HWID=%02x%02x, HWRV=%02x%02x\n", 
                cmdreq.m_buf[5], cmdreq.m_buf[6],cmdreq.m_buf[7], cmdreq.m_buf[8],
                cmdreq.m_buf[9], cmdreq.m_buf[10]);
        return -EFAULT;
    }
    return 0;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_resetDeviceWithBootLoadInstruction                                 */
/* ------------------------------------------------------------------------- */
static s32 shtrpd_resetDeviceWithBootLoadInstruction(void)
{
    s32 ret = -1;
    u8 buffer = SHTRPD_BOOT_ENTRY_DAT;

    ret = shtrpd_i2c_write_block_data(shtrpd_client, SHTRPD_BOOT_ENTRY_CMD, 
                                         1, &buffer);
    SHTRPD_DBG("BootLoadInstruction ret=%02x \n", ret);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_bootloader_i2c_ackPolling                                          */
/* ------------------------------------------------------------------------- */
static s32 shtrpd_bootloader_i2c_ackPolling(const struct i2c_client *client)
{
#if 0
    s32 i2c_ret = 0;
    u8 data = 0;
    u32 retry_cnt = 0;
    struct i2c_msg mesgs[] = {
        {
//            .addr   = SHTRPD_BOOT_SADDR,
//            .flags  = I2C_M_RD,
//            .len    = 1,
//            .buf    = &data,

            .addr   = SHTRPD_BOOT_SADDR,
            .flags  = 0,
            .len    = 0,
            .buf    = &data,
        },
    };

    for(retry_cnt=0; retry_cnt<SHTRPD_ACK_POLL_RETRY_MAX; retry_cnt++){
        mutex_lock(&shtrpd_i2c_lock);
        i2c_ret = i2c_transfer(client->adapter, mesgs, 1);
        mutex_unlock(&shtrpd_i2c_lock);

        if (i2c_ret <= 0){
            SHTRPD_ERR("ack poll check response not (cnt=%d) err =%d \n", retry_cnt,i2c_ret);
            usleep(SHTRPD_BOOT_ACK_POLL_DELAY);
            continue;
        }
        else{
            break;
        }
    }

    if(i2c_ret <= 0){
        SHTRPD_ERR("Bootloader ack none  err=%d \n", i2c_ret);
        return SHTRPD_BOOT_ERR_ACK_NONE;
    }
    return SHTRPD_BOOT_SUCCESS;
#else
    s32 i2c_ret = 0;
    u8 data[2];
    u8 cmd = SHTRPD_GET_BOOT_VER;
    struct i2c_msg mesgs[] = {
        {
            .addr   = SHTRPD_BOOT_SADDR,
            .flags  = 0,
            .len    = 1,
            .buf    = &cmd,
        },
    };

    mutex_lock(&shtrpd_i2c_lock);

    i2c_ret = i2c_transfer(client->adapter, mesgs, 1);
    mutex_unlock(&shtrpd_i2c_lock);

    if (i2c_ret <= 0){
        SHTRPD_ERR("ack polling Bootloader version read err =%d \n", i2c_ret);
        return SHTRPD_BOOT_ERR_READ;
    }

    SHTRPD_DBG("ack polling Bootloader ver=%02x,%02x \n", data[0], data[1]);
    return SHTRPD_BOOT_SUCCESS;
#endif
}

/* ------------------------------------------------------------------------- */
/* shtrpd_i2c_readBootloaderVersion                                          */
/* ------------------------------------------------------------------------- */
static s32 shtrpd_i2c_readBootloaderVersion(const struct i2c_client *client, 
                                                                     u8 *pver)
{
    s32 i2c_ret = 0;
    u8 data[2];
    u8 cmd = SHTRPD_GET_BOOT_VER;
    struct i2c_msg mesgs[] = {
        {
            .addr   = SHTRPD_BOOT_SADDR,
            .flags  = 0,
            .len    = 1,
            .buf    = &cmd,
        },
        {
            .addr   = SHTRPD_BOOT_SADDR,
            .flags  = I2C_M_RD,
            .len    = 2,
            .buf    = &data[0],
        },
    };

    mutex_lock(&shtrpd_i2c_lock);
    i2c_ret = i2c_transfer(client->adapter, mesgs, 2);
    mutex_unlock(&shtrpd_i2c_lock);

    if (i2c_ret <= 0){
        SHTRPD_ERR("Bootloader version read err =%d \n", i2c_ret);
        return SHTRPD_BOOT_ERR_READ;
    }

    SHTRPD_DBG("Bootloader ver=%02x,%02x \n", data[0], data[1]);

    memcpy(pver, &data[0], 2);
    usleep(SHTRPD_BOOT_VERSION_READ_DELAY);
    return SHTRPD_BOOT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_i2c_write_Bootloader_data                                          */
/* ------------------------------------------------------------------------- */
s32 shtrpd_i2c_write_Bootloader_data(const struct i2c_client *client, 
                                                u8 addr1, u8 addr2, u8 *values)
{
    s32 i2c_ret = 0;
    u8 cmd[SHTRPD_BOOT_BLOCK_SIZE+2];
    struct i2c_msg mesgs[] = {
        {
            .addr   = SHTRPD_BOOT_SADDR,
            .flags  = 0,
            .len    = SHTRPD_BOOT_BLOCK_SIZE+2,
            .buf    = &cmd[0],
        },
    };

    cmd[0] = addr1;
    cmd[1] = addr2;
    memcpy(&cmd[2], values, SHTRPD_BOOT_BLOCK_SIZE);
    
    mutex_lock(&shtrpd_i2c_lock);
    i2c_ret = i2c_transfer(client->adapter, mesgs, 1);
    mutex_unlock(&shtrpd_i2c_lock);

    if (i2c_ret <= 0){
        SHTRPD_ERR("Bootloader write err =%d \n", i2c_ret);
        return SHTRPD_BOOT_ERR_READ;
    }

    return SHTRPD_BOOT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_i2c_read_Bootloader_data                                           */
/* ------------------------------------------------------------------------- */
s32 shtrpd_i2c_read_Bootloader_data(const struct i2c_client *client, 
                                                u8 addr1, u8 addr2, u8 *values)
{
    s32 i2c_ret = 0;
    u8 data[SHTRPD_BOOT_BLOCK_SIZE];
    u8 cmd[3] = {SHTRPD_GET_BOOT_DATA, 0, 0 };
    struct i2c_msg mesgs[] = {
        {
            .addr   = SHTRPD_BOOT_SADDR,
            .flags  = 0,
            .len    = 3,
            .buf    = &cmd[0],
        },
        {
            .addr   = SHTRPD_BOOT_SADDR,
            .flags  = I2C_M_RD,
            .len    = SHTRPD_BOOT_BLOCK_SIZE,
            .buf    = &data[0],
        },
    };

    cmd[1] = addr1;
    cmd[2] = addr2;
    
    mutex_lock(&shtrpd_i2c_lock);
    i2c_ret = i2c_transfer(client->adapter, mesgs, 2);
    mutex_unlock(&shtrpd_i2c_lock);

    if (i2c_ret <= 0){
        SHTRPD_ERR("Bootloader read err =%d \n", i2c_ret);
        return SHTRPD_BOOT_ERR_READ;
    }

    memcpy(values, &data[0], SHTRPD_BOOT_BLOCK_SIZE);
    return SHTRPD_BOOT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_i2c_exec_app                                                       */
/* ------------------------------------------------------------------------- */
s32 shtrpd_i2c_exec_app(const struct i2c_client *client)
{
    s32 i2c_ret = 0;
    u8 cmd = SHTRPD_EXEC_APP;
    struct i2c_msg mesgs[] = {
        {
            .addr   = SHTRPD_BOOT_SADDR,
            .flags  = 0,
            .len    = 1,
            .buf    = &cmd,
        },
    };

    mutex_lock(&shtrpd_i2c_lock);
    i2c_ret = i2c_transfer(client->adapter, mesgs, 1);
    mutex_unlock(&shtrpd_i2c_lock);

    if (i2c_ret <= 0){
        SHTRPD_ERR("App exec err =%d \n", i2c_ret);
        return SHTRPD_BOOT_ERR_EXEC_APP;
    }

    return SHTRPD_BOOT_SUCCESS;
}

#ifdef SHTRPD_ENABLE_INIT_FW_DL
/* ------------------------------------------------------------------------- */
/* shtrpd_enter_bootloader_mode                                              */
/* ------------------------------------------------------------------------- */
s32 shtrpd_enter_bootloader_mode(const struct i2c_client *client)
{
    s32 ret = 0;
    s32 ack = 0;
    u8 ver[2];
    int cnt=0;

    SHTRPD_DBG("start\n");

    gpio_set_value(NRST_LINE_PIN,0);
    usleep(100000);
    gpio_set_value(NRST_LINE_PIN,1);

    while(1)
    {
        ack = shtrpd_bootloader_i2c_ackPolling(shtrpd_client);
        if(SHTRPD_BOOT_SUCCESS == ack){
            break;
        }
        if(0 != shtrpd_fw_poll_delay){
            usleep(shtrpd_fw_poll_delay);
        }
        cnt++;
        if( cnt > shtrpd_fw_poll_retry_cnt){
            break;
        }
    }

    if(SHTRPD_BOOT_SUCCESS == ack){
        ret = shtrpd_i2c_readBootloaderVersion(shtrpd_client, ver);
    }

    if((SHTRPD_BOOT_SUCCESS != ack) || (SHTRPD_BOOT_SUCCESS != ret)) {
        SHTRPD_DBG("bootloader ver read error, restart bootloader mode \n");

        /* send the instruction to restart the Trackpad device in bootloader */
        shtrpd_resetDeviceWithBootLoadInstruction();

        /* create a delay to give the Trackpad device time to reset in bootloader */
        usleep(SHTRPD_BOOTLOADER_RESET_DELAY);   

        /* now the bootloader version can be read */
        ret = shtrpd_i2c_readBootloaderVersion(shtrpd_client, ver);
        if(ret){
            SHTRPD_DBG("Cannot read Bootloader version !!!\n");
        }
    }
    return ret;
}
#endif /* SHTRPD_ENABLE_INIT_FW_DL */

#ifdef SHTRPD_ENABLE_INIT_FW_DL_CRC_CHECK
/* ------------------------------------------------------------------------- */
/* shtrpd_i2c_firm_crc_check                                                 */
/* ------------------------------------------------------------------------- */
s32 shtrpd_i2c_firm_crc_check(const struct i2c_client *client)
{
    s32 i2c_ret = 0;
//    u32 retry_cnt=0;
    u8 data[256];
    u8 cmd = SHTRPD_CRC_CECK;
    struct i2c_msg mesgs[] = {
        {
            .addr   = SHTRPD_BOOT_SADDR,
            .flags  = 0,
            .len    = 1,
            .buf    = &cmd,
        },
        {
            .addr   = SHTRPD_BOOT_SADDR,
            .flags  = I2C_M_RD,
            .len    = 1,
            .buf    = &data[0],
        },
    };

//    i2c_ret = i2c_transfer(client->adapter, &mesgs[0], 1);

//    if (i2c_ret <= 0){
//        SHTRPD_ERR("CRC check error err =%d \n", i2c_ret);
//        return SHTRPD_BOOT_ERR_CRC;
//    }

//    usleep(SHTRPD_CRC_CHECK_DELAY);

//    for(retry_cnt=0; retry_cnt<SHTRPD_CRC_RETRY_MAX; retry_cnt++){

//        i2c_ret = i2c_transfer(client->adapter, &mesgs[1], 1);

//        if (i2c_ret <= 0){
//            SHTRPD_ERR("CRC check response not error(%d) err =%d \n", retry_cnt,i2c_ret);
//            continue;
//        }
//        else{
//            break;
//        }
//    }

    mutex_lock(&shtrpd_i2c_lock);
    i2c_ret = i2c_transfer(client->adapter, mesgs, 2);
    mutex_unlock(&shtrpd_i2c_lock);

    if(i2c_ret <= 0){
        SHTRPD_ERR("CRC check response not error(%d) \n", i2c_ret);
        return SHTRPD_BOOT_ERR_CRC;
    }

    if(0 != data[0]){
        SHTRPD_ERR("CRC check response not 0 : crc =%d \n", data[0]);
        return SHTRPD_BOOT_ERR_CRC;
    }

    SHTRPD_DBG("CRC check OK !!!\n");
    return SHTRPD_BOOT_SUCCESS;
}
#endif /* SHTRPD_ENABLE_INIT_FW_DL_CRC_CHECK */

/* ------------------------------------------------------------------------- */
/* shtrpd_verify_data                                                        */
/* ------------------------------------------------------------------------- */
static s32 shtrpd_verify_data(u8 *pdata, u32 len)
{
    u32 i,j;
    u32 index = 0;
    s32 ret = 0;
    int address = SHTRPD_BOOT_LOADER_START_ADR;
    u8 data_buffer[66];

    SHTRPD_DBG("shtrpd_verify_data start \n");

    for(i=0; i<SHTRPD_BOOT_WRITES; i++){
        data_buffer[0] = (address & 0xFF00) >> 8;
        data_buffer[1] = (address & 0x00FF);

        ret = shtrpd_i2c_read_Bootloader_data(shtrpd_client, 
                              data_buffer[0], data_buffer[1], &data_buffer[2]);

        if( SHTRPD_BOOT_SUCCESS != ret ){
            SHTRPD_ERR("i2c read error from Bootloader (ret=%04x) \n", ret);
            return SHTRPD_BOOT_ERR_READ;
        }
/*
        SHTRPD_DBG("read fw[%03d] = %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x\n",
        i, data_buffer[ 0],data_buffer[ 1],data_buffer[ 2],data_buffer[ 3],
           data_buffer[ 4],data_buffer[ 5],data_buffer[ 6],data_buffer[ 7],
           data_buffer[ 8],data_buffer[ 9],data_buffer[10],data_buffer[11],
           data_buffer[12],data_buffer[13],data_buffer[14],data_buffer[15]);
*/
        for(j=0; j<SHTRPD_BOOT_BLOCK_SIZE; j++){
            if(data_buffer[2+j] != pdata[index+j]){
                SHTRPD_ERR("Bootloader verify err : addr=%04x \n", (address+j));
                return SHTRPD_BOOT_ERR_VERIFY;
            }
        }
        index += SHTRPD_BOOT_BLOCK_SIZE;

        address += SHTRPD_BOOT_BLOCK_SIZE;
    }
    SHTRPD_DBG("Bootloader verify OK \n");

    return SHTRPD_BOOT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_programBootloader                                                  */
/* ------------------------------------------------------------------------- */
static s32 shtrpd_programBootloader(u8 *pdata, u32 len)
{
    u32 i,j;
    u32 index = 0;
    int address = SHTRPD_BOOT_LOADER_START_ADR;
    s32 ret = -1;
    u8 data_buffer[66];

    SHTRPD_DBG("shtrpd_programBootloader start \n");

    for(i=0; i<SHTRPD_BOOT_WRITES; i++){
        data_buffer[0] = (address & 0xFF00) >> 8;
        data_buffer[1] = (address & 0x00FF);
        
        for(j=0; j<SHTRPD_BOOT_BLOCK_SIZE; j++){
            data_buffer[2+j] = pdata[index+j];
        }
        index += SHTRPD_BOOT_BLOCK_SIZE;
/*
        SHTRPD_DBG("write fw[%03d] = %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x\n",
        i, data_buffer[ 0],data_buffer[ 1],data_buffer[ 2],data_buffer[ 3],
           data_buffer[ 4],data_buffer[ 5],data_buffer[ 6],data_buffer[ 7],
           data_buffer[ 8],data_buffer[ 9],data_buffer[10],data_buffer[11],
           data_buffer[12],data_buffer[13],data_buffer[14],data_buffer[15]);
*/
        shtrpd_i2c_write_Bootloader_data(shtrpd_client, data_buffer[0], 
                                          data_buffer[1], &data_buffer[2]);

        usleep(SHTRPD_BOOT_BLOCK_WRITE_DELAY);
        address += SHTRPD_BOOT_BLOCK_SIZE;
    }

    ret = shtrpd_verify_data(pdata, len);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_fw_write                                                           */
/* ------------------------------------------------------------------------- */
static s32 shtrpd_fw_write(u8 *pdata, u32 len)
{
    s32 ret = 0;
    s32 ack = 0;
    u8 ver[2];
    int cnt=0;

    SHTRPD_DBG("firmware write start \n");

    while(1)
    {
        ack = shtrpd_bootloader_i2c_ackPolling(shtrpd_client);
        if(SHTRPD_BOOT_SUCCESS == ack){
            break;
        }
        if(0 != shtrpd_fw_poll_delay){
            usleep(shtrpd_fw_poll_delay);
        }
        cnt++;
        if( cnt > shtrpd_fw_poll_retry_cnt){
            break;
        }
    }

    if(SHTRPD_BOOT_SUCCESS == ack){

        ret = shtrpd_i2c_readBootloaderVersion(shtrpd_client, ver);
    }

    if((SHTRPD_BOOT_SUCCESS != ack) || (SHTRPD_BOOT_SUCCESS != ret)) {
        SHTRPD_DBG("bootloader ver read error, restart bootloader mode \n");

        /* send the instruction to restart the Trackpad device in bootloader */
        shtrpd_resetDeviceWithBootLoadInstruction();

        /* create a delay to give the Trackpad device time to reset in bootloader */
        usleep(SHTRPD_BOOTLOADER_RESET_DELAY);   

        /* now the bootloader version can be read */
        ret = shtrpd_i2c_readBootloaderVersion(shtrpd_client, ver);
        if(ret){
            SHTRPD_DBG("Cannot read Bootloader version !!!\n");
        }
    }

    /* program the bootloader */
    ret = shtrpd_programBootloader(pdata, len);

    if (SHTRPD_BOOT_SUCCESS == ret){
        SHTRPD_DBG("firmware write OK\n");
        return 0;
    }
    else{
        SHTRPD_DBG("firmware write NG err=%d\n", ret);
        return -EFAULT;
    }
}

/* ------------------------------------------------------------------------- */
/* shtrpd_fw_check                                                           */
/* ------------------------------------------------------------------------- */
#ifdef SHTRPD_DBG_INITIAL_BOOT_CHK
static s32 shtrpd_fw_check(void)
{
    s32 ack = 0;
//    s32 crc = 0;
    s32 enter_boot = 1;
    u8  i2c_poll = 0;

    SHTRPD_DBG("firmware check start \n");

    ack = shtrpd_bootloader_i2c_ackPolling(shtrpd_client);

    if(SHTRPD_BOOT_SUCCESS == ack){
        i2c_poll = 1;
    }

//    crc = shtrpd_i2c_firm_crc_check(shtrpd_client);

//    if(SHTRPD_BOOT_SUCCESS == crc){
//        enter_boot = 0;
//    }
    
    if((1 == enter_boot) || (SHTRPD_BOOT_SUCCESS == ack)){
        return SHTRPD_FM_CHK_BOOTLOADER_MODE;
    }
    else{
        return SHTRPD_FM_CHK_APP_EXEC;
	}
}
#endif

/* ------------------------------------------------------------------------- */
/* shtrpd_fw_update                                                          */
/* ------------------------------------------------------------------------- */
static long shtrpd_fw_update(u8 *pdata, u32 len)
{
    s32 ret = 0;
    s32 rc = 0;
    u8  i = 0;

    /* bootLoaderMode state on */
    bootLoaderMode = true;
    /* firm write state on */
    firmWritting = true;

    for(i=0; i<3; i++){
        shtrpd_ts_phys_init();
        /* NRST H->L->H */
        gpio_set_value(NRST_LINE_PIN,0);
        usleep(100000);
        gpio_set_value(NRST_LINE_PIN,1);
        SHTRPD_DBG("HW Reset completed\n");

        rc = shtrpd_fw_write(pdata, len);
        if (rc)
        {
            SHTRPD_ERR("shtrpd_fw_write NG retry=%d\n", i);
            ret = -EFAULT;
        }
        else{
            ret = 0;
            break;
        }
    }

    bootLoaderMode = false;

#if 1
    shtrpd_ts_phys_init();
    /* APP exec */
    shtrpd_i2c_exec_app(shtrpd_client);
    SHTRPD_DBG("APP exec start\n");
#else
    shtrpd_ts_phys_init();
    /* NRST H->L->H */
    gpio_set_value(NRST_LINE_PIN,0);
    usleep(1000);
    gpio_set_value(NRST_LINE_PIN,1);
    usleep(1000);
    SHTRPD_DBG("HW Reset completed\n");
#endif

    ret = shtrpd_initialSetup_complete_check();

    /* firm write state off */
    firmWritting = false;

    if(ret < 0){
        SHTRPD_DBG("initialSetup check retry over \n");
        return ret;
    }

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_ioctl_fw_update                                                     */
/* ------------------------------------------------------------------------- */
static s32 shtrpd_ioctl_fw_update(struct file *filp, unsigned int cmd, 
                                                           unsigned long arg)
{
    long rc = -1;
    int32_t ret = -1;

    SHTRPD_DBG("SHTRPDIO_FW_UPDATE \n");

    if(!shtrpd_flip_state) {
        if(shtrpd_flip_reset_state) {
            shtrpd_flip_reset_state = 0;
            gpio_set_value(NRST_LINE_PIN,1);
            usleep(SHTRPD_NRST_DELAY);
            shtrpd_ts_phys_init();
        }
        shtrpd_flip_state = 1;
        shtrpd_sys_enable_irq();
        
        rc = shtrpd_fw_update((u8*)&hex_array[0], SHTRPD_FW_IMAGE_MAX);
        if(rc < 0) {
            shtrpd_flip_state = 0;
            goto reset;
        } else {
            shtrpd_flip_state = 0;
            ret = shtrpd_flip_close_proc();
            if(ret < 0){
                goto reset;
            } else {
                shtrpd_sys_disable_irq();
            }
        }
    } else {
        rc = shtrpd_fw_update((u8*)&hex_array[0], SHTRPD_FW_IMAGE_MAX);
    }
    return rc;

reset:
    SHTRPD_ERR("reset setting exec \n");
    shtrpd_sys_disable_irq();
    gpio_set_value(NRST_LINE_PIN, 0);
    shtrpd_flip_reset_state = 1;
    return rc;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_ioctl_fw_write_image                                               */
/* ------------------------------------------------------------------------- */
static s32 shtrpd_ioctl_fw_write_image(struct file *filp, unsigned int cmd, 
                                                           unsigned long arg)
{
    void __user *argp = (void __user *)arg;
    long rc = -1;
    int32_t ret = -1;
    struct shtrpd_ioctl_fwparam param;
    u8 *pdata = NULL;

    SHTRPD_DBG("SHTRPDIO_WRITE_IMAGE \n");
    ret = copy_from_user(&param, argp, sizeof(struct shtrpd_ioctl_fwparam));
    if (ret) {
        SHTRPD_ERR("error(copy_from_user) : shtrpd_ioctl_fwparam)\n" );
        return -EFAULT;
    }

    if(param.size > SHTRPD_FW_IMAGE_MAX){
        param.size = SHTRPD_FW_IMAGE_MAX;
    }

    pdata = (u8*)kmalloc(param.size, GFP_KERNEL);
    if(pdata == NULL){
        SHTRPD_ERR("kmalloc error\n" );
        return -EINVAL;
    }
    if(0 != copy_from_user(pdata, param.data, param.size)){
        kfree(pdata);
        SHTRPD_ERR("error(copy_from_user) shtrpd_ioctl_fwparam.data \n" );
        return -EINVAL;
    }

    if(!shtrpd_flip_state) {
        if(shtrpd_flip_reset_state) {
            shtrpd_flip_reset_state = 0;
            gpio_set_value(NRST_LINE_PIN,1);
            usleep(SHTRPD_NRST_DELAY);
            shtrpd_ts_phys_init();
        }
        shtrpd_flip_state = 1;
        shtrpd_sys_enable_irq();
        
        rc = shtrpd_fw_update(pdata, SHTRPD_FW_IMAGE_MAX);
        if(rc < 0) {
            shtrpd_flip_state = 0;
            goto reset;
        } else {
            shtrpd_flip_state = 0;
            ret = shtrpd_flip_close_proc();
            if(ret < 0){
                goto reset;
            } else {
                shtrpd_sys_disable_irq();
            }
        }
    } else {
        rc = shtrpd_fw_update(pdata, SHTRPD_FW_IMAGE_MAX);
    }

    kfree(pdata);
    return rc;

reset:
    SHTRPD_ERR("reset setting exec \n");
    shtrpd_sys_disable_irq();
    gpio_set_value(NRST_LINE_PIN, 0);
    shtrpd_flip_reset_state = 1;
    kfree(pdata);
    return rc;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_ioctl_write_sensor                                                 */
/* ------------------------------------------------------------------------- */
static long shtrpd_ioctl_write_sensor(struct file *filp, unsigned int cmd, 
                                                           unsigned long arg)
{
    void __user *argp = (void __user *)arg;
    int32_t ret = -1;
    struct IoctlDiagCmdReq cmdreq;

    SHTRPD_DBG("SHTRPDIO_WRITE_SENSOR \n");
    ret = copy_from_user(&cmdreq, argp, sizeof(struct IoctlDiagCmdReq));
    if (ret) {
        SHTRPD_ERR("error(copy_from_user) : shtrpd_ioctl(cmd = SHTRPDIO_WRITE_SENSOR)\n" );
        return -EFAULT;
    }
    ret = shtrpd_i2c_user_write_block_data(shtrpd_client, cmdreq.m_Cmd, 
                                        cmdreq.m_req_size, &cmdreq.m_buf[0]);
    cmdreq.rtn = ret;
    ret = copy_to_user(argp, &cmdreq, sizeof(struct IoctlDiagCmdReq));
    if (ret) {
        SHTRPD_ERR("error(copy_to_user) : shtrpd_ioctl(cmd = SHTRPDIO_WRITE_SENSOR)\n" );
        return -EFAULT;
    }
    SHTRPD_DBG("ioctl_write ret=%d Cmd=%02X Len=%d Buf=%02X%02X%02X%02X %02X%02X%02X%02X\n", 
                                    cmdreq.rtn,
                                    cmdreq.m_Cmd, cmdreq.m_req_size,
                                    cmdreq.m_buf[0], cmdreq.m_buf[1],
                                    cmdreq.m_buf[2], cmdreq.m_buf[3],
                                    cmdreq.m_buf[4], cmdreq.m_buf[5],
                                    cmdreq.m_buf[6], cmdreq.m_buf[7] );
    return 0;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_ioctl_read_sensor                                                  */
/* ------------------------------------------------------------------------- */
static long shtrpd_ioctl_read_sensor(struct file *filp, unsigned int cmd, 
                                                           unsigned long arg)
{
    void __user *argp = (void __user *)arg;
    int32_t ret = -1;
    struct IoctlDiagCmdReq cmdreq;

    SHTRPD_DBG("SHTRPDIO_READ_SENSOR \n");
    ret = copy_from_user(&cmdreq, argp, sizeof(struct IoctlDiagCmdReq));
    if (ret) {
        SHTRPD_ERR("error(copy_from_user) : shtrpd_ioctl(cmd = SHTRPDIO_READ_SENSOR)\n" );
        return -EFAULT;
    }
    ret = shtrpd_i2c_user_read_block_data(shtrpd_client, cmdreq.m_Cmd,
                                        cmdreq.m_req_size, &cmdreq.m_buf[0]);
    cmdreq.rtn = ret;
    cmdreq.m_res_size = cmdreq.m_req_size;
    ret = copy_to_user(argp, &cmdreq, sizeof(struct IoctlDiagCmdReq));
    if (ret) {
        SHTRPD_ERR("error(copy_to_user) : shtrpd_ioctl(cmd = SHTRPDIO_READ_SENSOR)\n" );
        return -EFAULT;
    }
    SHTRPD_DBG("ioctl_read ret=%d Cmd=%02X Len=%d Buf=%02X%02X%02X%02X %02X%02X%02X%02X\n", 
                                    cmdreq.rtn,
                                    cmdreq.m_Cmd, cmdreq.m_req_size,
                                    cmdreq.m_buf[0], cmdreq.m_buf[1],
                                    cmdreq.m_buf[2], cmdreq.m_buf[3],
                                    cmdreq.m_buf[4], cmdreq.m_buf[5],
                                    cmdreq.m_buf[6], cmdreq.m_buf[7] );
    return 0;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_ioctl_reset                                                        */
/* ------------------------------------------------------------------------- */
static long shtrpd_ioctl_reset(struct file *filp, unsigned int cmd, 
                                                           unsigned long arg)
{
    void __user *argp = (void __user *)arg;
    int32_t ret = -1;
    struct shtrpd_ioctl_resetparam resetparam;
    
    SHTRPD_DBG("SHTRPDIO_RESET \n");
    ret = copy_from_user(&resetparam, argp, sizeof(struct shtrpd_ioctl_resetparam));
    if (ret) {
        SHTRPD_ERR("error(copy_from_user) : shtrpd_ioctl(cmd = SHTRPDIO_RESET)\n" );
        return -EFAULT;
    }

    gpio_request(NRST_LINE_PIN, NULL);
    ret = gpio_tlmm_config(GPIO_CFG(NRST_LINE_PIN, 0,
            GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
    if (ret) {
        SHTRPD_ERR("GPIO(NRST_LINE_PIN) set config NG\n");
    }
    
    ret = gpio_direction_output(NRST_LINE_PIN, 1);
    if (ret) {
        SHTRPD_ERR("GPIO(NRST_LINE_PIN) set direction NG\n");
    }

    if(resetparam.data == RESET_ENABLE) {
        gpio_set_value(NRST_LINE_PIN,1);
        usleep(SHTRPD_NRST_DELAY);
        
        setupCounter = 0;
        initialSetup = true;
//        enable_irq(shtrpd_client->irq);
        shtrpd_sys_enable_irq();
    } else {
//        disable_irq_nosync(shtrpd_client->irq);
        shtrpd_sys_disable_irq();
        gpio_set_value(NRST_LINE_PIN, 0);
    }

    ret = copy_to_user(argp, &resetparam, sizeof(struct shtrpd_ioctl_resetparam));
    if (ret) {
        SHTRPD_ERR("error(copy_to_user) : shtrpd_ioctl(cmd = SHTRPDIO_RESET)\n" );
        return -EFAULT;
    }

    return 0;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_ioctl_get_countval                                                  */
/* ------------------------------------------------------------------------- */
static long shtrpd_ioctl_get_countval(struct file *filp, unsigned int cmd, 
                                                           unsigned long arg)
{
    void __user *argp = (void __user *)arg;
    int32_t ret = -1;
    struct shtrpd_ioctl_countval countval;
    unsigned char addr = 0;
    int i = 0, tx = 0, rx = 0;
    unsigned char data[SHTRPD_CONT_DATA_READ_SIZE];

    SHTRPD_DBG("SHTRPDIO_GET_COUNTVALUE \n");
    ret = copy_from_user(&countval, argp, sizeof(struct shtrpd_ioctl_countval));
    if (ret) {
        SHTRPD_ERR("error(copy_from_user) : shtrpd_ioctl(cmd = SHTRPDIO_GET_COUNTVALUE)\n" );
        return -EFAULT;
    }

    if(countval.select == 0){
        addr = COUNT_VALUES;
    }else if(countval.select == 1){
        addr = LTA_VALUES;
    }else{
        return -EFAULT;
    }

    memset(data, 0, SHTRPD_CONT_DATA_READ_SIZE);

    ret = shtrpd_i2c_user_read_block_data(shtrpd_client, addr, SHTRPD_CONT_DATA_READ_SIZE, data);
    if ( ret < 0 ) {
        SHTRPD_ERR("error(shtrpd_i2c_user_read_block_data) : shtrpd_ioctl(cmd = SHTRPDIO_GET_COUNTVALUE)\n" );
        return -EFAULT;
    }

#if 0
    for(tx = 0; tx < TOTALTXS_VAL; tx++){
        for(rx = 0; rx < TOTALRXS_VAL; rx++){
            if ((rx % 2) == 0) {
                countval.data[tx][rx] = (0x00FF & data[i]);
                SHTRPD_DBG("data[%d]:0x%02X\n", i, data[i]);
                SHTRPD_DBG("countval.data[%d][%d]:0x%04X\n", tx, rx, countval.data[tx][rx]);
                i++;
                countval.data[tx][rx] = (((data[i] & 0xF0) << 4) | countval.data[tx][rx]); 
                SHTRPD_DBG("data[%d]:0x%02X\n", i, data[i]);
                SHTRPD_DBG("countval.data[%d][%d]:0x%04X\n", tx, rx, countval.data[tx][rx]);
            } else {
                countval.data[tx][rx] = (((data[i] & 0x0F) << 8) & 0x0F00);
                SHTRPD_DBG("data[%d]:0x%02X\n", i, data[i]);
                SHTRPD_DBG("countval.data[%d][%d]:0x%04X\n", tx, rx, countval.data[tx][rx]);
                i++;
                countval.data[tx][rx] = (data[i] | countval.data[tx][rx]);
                SHTRPD_DBG("data[%d]:0x%02X\n", i, data[i]);
                SHTRPD_DBG("countval.data[%d][%d]:0x%04X\n", tx, rx, countval.data[tx][rx]);
                i++;
            }
            if (rx == (TOTALRXS_VAL - 1)) {
                i++;
            }
        }
    }
#else
    for(tx = 0; tx < TOTALTXS_VAL; tx++){
        if ((tx % 2) == 0){
            for(rx = 0; rx < TOTALRXS_VAL; rx++){
                if ((rx % 2) == 0) {
                    countval.data[tx][rx] = (0x00FF & data[i]);
                    SHTRPD_DBG("data[%d]:0x%02X\n", i, data[i]);
                    SHTRPD_DBG("countval.data[%d][%d]:0x%04X\n", tx, rx, countval.data[tx][rx]);
                    i++;
                    countval.data[tx][rx] = (((data[i] & 0xF0) << 4) | countval.data[tx][rx]); 
                    SHTRPD_DBG("data[%d]:0x%02X\n", i, data[i]);
                    SHTRPD_DBG("countval.data[%d][%d]:0x%04X\n", tx, rx, countval.data[tx][rx]);
                } else {
                    countval.data[tx][rx] = (((data[i] & 0x0F) << 8) & 0x0F00);
                    SHTRPD_DBG("data[%d]:0x%02X\n", i, data[i]);
                    SHTRPD_DBG("countval.data[%d][%d]:0x%04X\n", tx, rx, countval.data[tx][rx]);
                    i++;
                    countval.data[tx][rx] = (data[i] | countval.data[tx][rx]);
                    SHTRPD_DBG("data[%d]:0x%02X\n", i, data[i]);
                    SHTRPD_DBG("countval.data[%d][%d]:0x%04X\n", tx, rx, countval.data[tx][rx]);
                    i++;
                }
            }
        }else{
            for(rx = 0; rx < TOTALRXS_VAL; rx++){
                if ((rx % 2) == 0) {
                    countval.data[tx][rx] = (((data[i] & 0x0F) << 8) & 0x0F00);
                    SHTRPD_DBG("data[%d]:0x%02X\n", i, data[i]);
                    SHTRPD_DBG("countval.data[%d][%d]:0x%04X\n", tx, rx, countval.data[tx][rx]);
                    i++;
                    countval.data[tx][rx] = (data[i] | countval.data[tx][rx]);
                    SHTRPD_DBG("data[%d]:0x%02X\n", i, data[i]);
                    SHTRPD_DBG("countval.data[%d][%d]:0x%04X\n", tx, rx, countval.data[tx][rx]);
                    i++;
                } else {
                    countval.data[tx][rx] = (0x00FF & data[i]);
                    SHTRPD_DBG("data[%d]:0x%02X\n", i, data[i]);
                    SHTRPD_DBG("countval.data[%d][%d]:0x%04X\n", tx, rx, countval.data[tx][rx]);
                    i++;
                    countval.data[tx][rx] = (((data[i] & 0xF0) << 4) | countval.data[tx][rx]); 
                    SHTRPD_DBG("data[%d]:0x%02X\n", i, data[i]);
                    SHTRPD_DBG("countval.data[%d][%d]:0x%04X\n", tx, rx, countval.data[tx][rx]);
                }
            }
        }
    }
#endif
    
    ret = copy_to_user(argp, &countval, sizeof(struct shtrpd_ioctl_countval));
    if (ret) {
        SHTRPD_ERR("error(copy_to_user) : shtrpd_ioctl(cmd = SHTRPDIO_GET_COUNTVALUE)\n" );
        return -EFAULT;
    }
    return 0;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_ioctl_property                                                     */
/* ------------------------------------------------------------------------- */
static long shtrpd_ioctl_property(struct file *filp, unsigned int cmd, 
                                                           unsigned long arg)
{
    void __user *argp = (void __user *)arg;
    int32_t ret = -1;
    struct shtrpd_ioctl_properties property;
    int i;

    SHTRPD_DBG("SHTRPDIO_PROPERTY \n");
    ret = copy_from_user(&property, argp, sizeof(struct shtrpd_ioctl_properties));
    if (ret) {
        SHTRPD_ERR("error(copy_from_user) : shtrpd_ioctl(cmd = SHTRPDIO_PROPERTY)\n" );
        return -EFAULT;
    }

    switch(property.id){
    case SHTRPD_PROPERTY_INIT:
        SHTRPD_DBG("property initial \n");
        shtrpd_properties_init();
        break;

    case SHTRPD_PROPERTY_ATI_VAL:
        if(SHTRPD_PROPERTY_CMD_READ == property.cmd){
            property.param[0] = (int)shtrpd_properties.atitarget_val;
            property.param[1] = (int)shtrpd_properties.atic_val;
            property.param[2] = (int)shtrpd_properties.atitarget2_val;
            property.param[3] = (int)shtrpd_properties.atic2_val;
            property.param[4] = (int)shtrpd_properties.re_ati_range;
            SHTRPD_DBG("property read ATI_VAL, %d, %d, %d, %d, %d \n",
                                      property.param[0],property.param[1],
                                      property.param[2],property.param[3],
                                      property.param[4]);

        }else if(SHTRPD_PROPERTY_CMD_WRITE == property.cmd){
            shtrpd_properties.atitarget_val = (u16)property.param[0];
            shtrpd_properties.atic_val      = (u8) property.param[1];
            shtrpd_properties.atitarget2_val= (u16)property.param[2];
            shtrpd_properties.atic2_val     = (u8) property.param[3];
            shtrpd_properties.re_ati_range  = (u8) property.param[4];
            SHTRPD_DBG("property write ATI_VAL, %d, %d, %d, %d, %d \n",
                                      property.param[0],property.param[1],
                                      property.param[2],property.param[3],
                                      property.param[4]);

        }else{
            SHTRPD_ERR("property access error, id=%d cmd=%d param=%d\n",
                                 property.id, property.cmd, property.param[0]);
            return -EFAULT;
        }
        break;

    case SHTRPD_PROPERTY_CONTROLSETTINGS:
        if(SHTRPD_PROPERTY_CMD_READ == property.cmd){
            property.param[0] = (int)shtrpd_properties.controlsetting_val[0];
            property.param[1] = (int)shtrpd_properties.controlsetting_val[1];
            property.param[2] = (int)shtrpd_properties.controlsetting_val[2];
            property.param[3] = (int)shtrpd_properties.controlsetting_val[3];
            SHTRPD_DBG("property read CONTROLSETTINGS, %d, %d, %d, %d \n",
                                      property.param[0],property.param[1],
                                      property.param[2],property.param[3]);

        }else if(SHTRPD_PROPERTY_CMD_WRITE == property.cmd){
            shtrpd_properties.controlsetting_val[0] = (u8)property.param[0];
            shtrpd_properties.controlsetting_val[1] = (u8)property.param[1];
            shtrpd_properties.controlsetting_val[2] = (u8)property.param[2];
            shtrpd_properties.controlsetting_val[3] = (u8)property.param[3];
            SHTRPD_DBG("property write CONTROLSETTINGS, %d, %d, %d, %d \n",
                                      property.param[0],property.param[1],
                                      property.param[2],property.param[3]);

        }else{
            SHTRPD_ERR("property access error, id=%d cmd=%d param=%d\n",
                                 property.id, property.cmd, property.param[0]);
            return -EFAULT;
        }
        break;

    case SHTRPD_PROPERTY_THRESHOLD:
        if(SHTRPD_PROPERTY_CMD_READ == property.cmd){
            property.param[0] = (int)shtrpd_properties.proxthreshold_val;
            property.param[1] = shtrpd_touchmultiplier_val;
            property.param[2] = shtrpd_touchshifter_val;
            property.param[3] = (int)shtrpd_properties.pmproxthreshold_val;
            property.param[4] = shtrpd_snapthreshold_val;
            property.param[5] = (int)shtrpd_properties.proxthreshold2_val;
            property.param[6] = shtrpd_touchmultiplier2_val;
            property.param[7] = shtrpd_touchshifter2_val;
            property.param[8] = shtrpd_intouchmultiplier_val;
            SHTRPD_DBG("property read THRESHOLD, %d, %d, %d, %d, %d, %d, %d, %d, %d\n",
                                      property.param[0],property.param[1],
                                      property.param[2],property.param[3],
                                      property.param[4],property.param[5],
                                      property.param[6],property.param[7],
                                      property.param[8]);

        }else if(SHTRPD_PROPERTY_CMD_WRITE == property.cmd){
            shtrpd_properties.proxthreshold_val     = (u8) property.param[0];
            shtrpd_touchmultiplier_val              = (u8) property.param[1];
            shtrpd_touchshifter_val                 = (u8) property.param[2];
            shtrpd_properties.pmproxthreshold_val   = (u8) property.param[3];
            shtrpd_snapthreshold_val                = (u16)property.param[4];
            shtrpd_properties.proxthreshold2_val    = (u8) property.param[5];
            shtrpd_touchmultiplier2_val             = (u8) property.param[6];
            shtrpd_touchshifter2_val                = (u8) property.param[7];
            shtrpd_intouchmultiplier_val            = (u8) property.param[8];
            SHTRPD_DBG("property write THRESHOLD, %d, %d, %d, %d, %d, %d, %d, %d, %d\n",
                                      property.param[0],property.param[1],
                                      property.param[2],property.param[3],
                                      property.param[4],property.param[5],
                                      property.param[6],property.param[7],
                                      property.param[8]);
        }else{
            SHTRPD_ERR("property access error, id=%d cmd=%d param=%d\n",
                                 property.id, property.cmd, property.param[0]);
            return -EFAULT;
        }
        break;

    case SHTRPD_PROPERTY_FILTER:
        if(SHTRPD_PROPERTY_CMD_READ == property.cmd){
            property.param[0] = (int)shtrpd_properties.filtersettings0_val;
            property.param[1] = (int)shtrpd_properties.touchdamping_val;
            property.param[2] = (int)shtrpd_properties.pmcountdamping_val;
            property.param[3] = (int)shtrpd_properties.lppmcountdamping_val;
            SHTRPD_DBG("property read FILTER, %d, %d, %d, %d\n",
                                      property.param[0],property.param[1],
                                      property.param[2],property.param[3]);

        }else if(SHTRPD_PROPERTY_CMD_WRITE == property.cmd){
            shtrpd_properties.filtersettings0_val  = (u8)property.param[0];
            shtrpd_properties.touchdamping_val     = (u8)property.param[1];
            shtrpd_properties.pmcountdamping_val   = (u8)property.param[2];
            shtrpd_properties.lppmcountdamping_val = (u8)property.param[3];
            SHTRPD_DBG("property write FILTER, %d, %d, %d, %d\n",
                                      property.param[0],property.param[1],
                                      property.param[2],property.param[3]);

        }else{
            SHTRPD_ERR("property access error, id=%d cmd=%d param=%d\n",
                                 property.id, property.cmd, property.param[0]);
            return -EFAULT;
        }
        break;

    case SHTRPD_PROPERTY_TIMING:
        if(SHTRPD_PROPERTY_CMD_READ == property.cmd){
            property.param[0] = (int)shtrpd_properties.reseedtime_val;
            property.param[1] = (int)shtrpd_properties.commstimeout_val;
            property.param[2] = (int)shtrpd_properties.modetime_val;
            property.param[3] = (int)shtrpd_properties.lptime_val;
            property.param[4] = (int)shtrpd_properties.sleeptime_val;
            property.param[5] = (int)shtrpd_properties.pmreseedtime;
            SHTRPD_DBG("property read TIMING, %d, %d, %d, %d, %d, %d \n",
                                      property.param[0],property.param[1],
                                      property.param[2],property.param[3],
                                      property.param[4],property.param[5]);

        }else if(SHTRPD_PROPERTY_CMD_WRITE == property.cmd){
            shtrpd_properties.reseedtime_val   = (u8)property.param[0];
            shtrpd_properties.commstimeout_val = (u8)property.param[1];
            shtrpd_properties.modetime_val     = (u8)property.param[2];
            shtrpd_properties.lptime_val       = (u8)property.param[3];
            shtrpd_properties.sleeptime_val    = (u8)property.param[4];
            shtrpd_properties.pmreseedtime     = (u8)property.param[5];
            SHTRPD_DBG("property write TIMING, %d, %d, %d, %d, %d, %d \n",
                                      property.param[0],property.param[1],
                                      property.param[2],property.param[3],
                                      property.param[4],property.param[5]);

        }else{
            SHTRPD_ERR("property access error, id=%d cmd=%d param=%d\n",
                                 property.id, property.cmd, property.param[0]);
            return -EFAULT;
        }
        break;

    case SHTRPD_PROPERTY_CHANNELSETUP:
        if(SHTRPD_PROPERTY_CMD_READ == property.cmd){
            property.param[0] = (int)shtrpd_properties.totalrxs_val;
            property.param[1] = (int)shtrpd_properties.totaltxs_val;
            property.param[2] = (int)shtrpd_properties.trackpadrxs_val;
            property.param[3] = (int)shtrpd_properties.trackpadtxs_val;
            property.param[4] = (int)shtrpd_properties.pmsetup0_val;
            property.param[5] = (int)shtrpd_properties.txhigh_val;
            property.param[6] = (int)shtrpd_properties.txlow_val;
            SHTRPD_DBG("property read CHANNELSETUP, %d, %d, %d, %d, %d, %d, %d \n",
                                      property.param[0],property.param[1],
                                      property.param[2],property.param[3],
                                      property.param[4],property.param[5],
                                      property.param[6]);

        }else if(SHTRPD_PROPERTY_CMD_WRITE == property.cmd){
            shtrpd_properties.totalrxs_val    = (u8)property.param[0];
            shtrpd_properties.totaltxs_val    = (u8)property.param[1];
            shtrpd_properties.trackpadrxs_val = (u8)property.param[2];
            shtrpd_properties.trackpadtxs_val = (u8)property.param[3];
            shtrpd_properties.pmsetup0_val    = (u8)property.param[4];
            shtrpd_properties.txhigh_val      = (u8)property.param[5];
            shtrpd_properties.txlow_val       = (u8)property.param[6];
            SHTRPD_DBG("property write CHANNELSETUP, %d, %d, %d, %d, %d, %d, %d \n",
                                      property.param[0],property.param[1],
                                      property.param[2],property.param[3],
                                      property.param[4],property.param[5],
                                      property.param[6]);

        }else{
            SHTRPD_ERR("property access error, id=%d cmd=%d param=%d\n",
                                 property.id, property.cmd, property.param[0]);
            return -EFAULT;
        }
        break;

    case SHTRPD_PROPERTY_PROXSETTINGS:
        if(SHTRPD_PROPERTY_CMD_READ == property.cmd){
            property.param[0] = (int)shtrpd_properties.proxsettings_val[0];
            property.param[1] = (int)shtrpd_properties.proxsettings_val[1];
            property.param[2] = (int)shtrpd_properties.proxsettings_val[2];
            property.param[3] = (int)shtrpd_properties.proxsettings_val[3];
            SHTRPD_DBG("property read PROXSETTINGS, %d, %d, %d, %d \n",
                                      property.param[0],property.param[1],
                                      property.param[2],property.param[3]);

        }else if(SHTRPD_PROPERTY_CMD_WRITE == property.cmd){
            shtrpd_properties.proxsettings_val[0] = (u8)property.param[0];
            shtrpd_properties.proxsettings_val[1] = (u8)property.param[1];
            shtrpd_properties.proxsettings_val[2] = (u8)property.param[2];
            shtrpd_properties.proxsettings_val[3] = (u8)property.param[3];
            SHTRPD_DBG("property write PROXSETTINGS, %d, %d, %d, %d \n",
                                      property.param[0],property.param[1],
                                      property.param[2],property.param[3]);

        }else{
            SHTRPD_ERR("property access error, id=%d cmd=%d param=%d\n",
                                 property.id, property.cmd, property.param[0]);
            return -EFAULT;
        }
        break;

    case SHTRPD_PROPERTY_ACTIVECHANNELS:
        if(SHTRPD_PROPERTY_CMD_READ == property.cmd){
            for(i=0;i<15;i++){
                property.param[i] = (int)shtrpd_properties.activechannels_val[i];
            }
            SHTRPD_DBG("property read ACTIVECHANNELS, %d \n",
                                      property.param[0]);

        }else if(SHTRPD_PROPERTY_CMD_WRITE == property.cmd){
            for(i=0;i<15;i++){
                shtrpd_properties.activechannels_val[i] = (u16)property.param[i];
            }
            SHTRPD_DBG("property write ACTIVECHANNELS, %d \n",
                                      property.param[0]);

        }else{
            SHTRPD_ERR("property access error, id=%d cmd=%d param=%d\n",
                                 property.id, property.cmd, property.param[0]);
            return -EFAULT;
        }
        break;

    case SHTRPD_PROPERTY_DEBOUNCE:
        if(SHTRPD_PROPERTY_CMD_READ == property.cmd){
            property.param[0] = (int)shtrpd_properties.proxdb_val;
            property.param[1] = (int)shtrpd_properties.touchsnapdb_val;
            property.param[2] = (int)shtrpd_properties.touchsnapdb2_val;
            SHTRPD_DBG("property read DEBOUNCE, %d, %d \n",
                                      property.param[0],property.param[1]);

        }else if(SHTRPD_PROPERTY_CMD_WRITE == property.cmd){
            shtrpd_properties.proxdb_val       = (u8)property.param[0];
            shtrpd_properties.touchsnapdb_val  = (u8)property.param[1];
            shtrpd_properties.touchsnapdb2_val = (u8)property.param[2];
            SHTRPD_DBG("property write DEBOUNCE, %d, %d \n",
                                      property.param[0],property.param[1]);

        }else{
            SHTRPD_ERR("property access error, id=%d cmd=%d param=%d\n",
                                 property.id, property.cmd, property.param[0]);
            return -EFAULT;
        }
        break;

    case SHTRPD_PROPERTY_PMATI:
        if(SHTRPD_PROPERTY_CMD_READ == property.cmd){
            property.param[0] = (int)shtrpd_properties.pmatitarget_val;
            property.param[1] = (int)shtrpd_properties.pmatic_val;
            property.param[2] = (int)shtrpd_properties.pm_re_ati_range;
            SHTRPD_DBG("property read PMATI, %d, %d, %d \n",
                                      property.param[0],property.param[1],
                                      property.param[2]);

        }else if(SHTRPD_PROPERTY_CMD_WRITE == property.cmd){
            shtrpd_properties.pmatitarget_val = (u16)property.param[0];
            shtrpd_properties.pmatic_val      = (u8)property.param[1];
            shtrpd_properties.pm_re_ati_range = (u8)property.param[2];
            SHTRPD_DBG("property write PMATI, %d, %d, %d \n",
                                      property.param[0],property.param[1],
                                      property.param[2]);

        }else{
            SHTRPD_ERR("property access error, id=%d cmd=%d param=%d\n",
                                 property.id, property.cmd, property.param[0]);
            return -EFAULT;
        }
        break;

    case SHTRPD_PROPERTY_TEMPDRIFT:
        if(SHTRPD_PROPERTY_CMD_READ == property.cmd){
            property.param[0] = (int)shtrpd_properties.temp_non_detect_avg;
            property.param[1] = (int)shtrpd_properties.temp_min_delta;
            property.param[2] = (int)shtrpd_properties.temp_lta_adj_interval;
            SHTRPD_DBG("property read TEMP_DRIFT, %d, %d, %d \n",
                                      property.param[0],property.param[1],
                                      property.param[2]);

        }else if(SHTRPD_PROPERTY_CMD_WRITE == property.cmd){
            shtrpd_properties.temp_non_detect_avg   = (u8)property.param[0];
            shtrpd_properties.temp_min_delta        = (u8)property.param[1];
            shtrpd_properties.temp_lta_adj_interval = (u8)property.param[2];
            SHTRPD_DBG("property write TEMP_DRIFT, %d, %d, %d \n",
                                      property.param[0],property.param[1],
                                      property.param[2]);

        }else{
            SHTRPD_ERR("property access error, id=%d cmd=%d param=%d\n",
                                 property.id, property.cmd, property.param[0]);
            return -EFAULT;
        }
        break;

    default:
        SHTRPD_ERR("property id nothing, id=%d\n", property.id);
        return -EFAULT;
        break;
    }

    ret = copy_to_user(argp, &property, sizeof(struct shtrpd_ioctl_properties));
    if (ret) {
        SHTRPD_ERR("error(copy_to_user) : shtrpd_ioctl(cmd = SHTRPDIO_PROPERTY)\n" );
        return -EFAULT;
    }
    return 0;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_ioctl_set_chargerarmor                                             */
/* ------------------------------------------------------------------------- */
static long shtrpd_ioctl_set_chargerarmor(struct file *filp, unsigned int cmd, 
                                                           unsigned long arg)
{
    void __user *argp = (void __user *)arg;
    int32_t ret = -1;
    int mode = 0;
    
    SHTRPD_DBG("SHTRPDIO_SET_CHARGERARMOR \n");
    ret = copy_from_user(&mode, argp, sizeof(int));
    if (ret) {
        SHTRPD_ERR("error(copy_from_user) : shtrpd_ioctl(cmd = SHTRPDIO_SET_CHARGERARMOR)\n" );
        return -EFAULT;
    }

    SHTRPD_DBG("SHTRPDIO_SET_CHARGERARMOR mode:%d\n", mode);

    return 0;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_ioctl_set_touchevent                                               */
/* ------------------------------------------------------------------------- */
static long shtrpd_ioctl_set_touchevent(struct file *filp, unsigned int cmd, 
                                                           unsigned long arg)
{
    void __user *argp = (void __user *)arg;
    int32_t ret = -1;
    int mode = 0;
    
    SHTRPD_DBG("SHTRPDIO_SET_TOUCHEVENT \n");
    ret = copy_from_user(&mode, argp, sizeof(int));
    if (ret) {
        SHTRPD_ERR("error(copy_from_user) : shtrpd_ioctl(cmd = SHTRPDIO_SET_TOUCHEVENT)\n" );
        return -EFAULT;
    }

    shtrpd_touchevent_sw = mode;
    SHTRPD_DBG("SHTRPDIO_SET_TOUCHEVENT mode:%d\n", mode);

    return 0;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_ioctl_set_touchevent                                               */
/* ------------------------------------------------------------------------- */
static long shtrpd_ioctl_set_reati(struct file *filp, unsigned int cmd, 
                                                           unsigned long arg)
{
    int32_t ret = -1;
    u8 data_buffer[11];

    SHTRPD_DBG("SHTRPDIO_SET_REATI \n");

    /*  ControlSettings0 and clear ACK_RESET Bit    */
    data_buffer[0] = shtrpd_properties.controlsetting_val[0] | AUTO_ATI;
    /*  ControlSettings1    */
    if(shtrpd_touchcruiser_state){
        data_buffer[1] = shtrpd_properties.controlsetting_val[1];
    }else{
        data_buffer[1] = shtrpd_properties.controlsetting_val[1] | DIS_TOUCH_EVENT;
    }
    /*  ControlSettings2(add)    */
    if(shtrpd_atic_adj_disable)
        data_buffer[2] = shtrpd_properties.controlsetting_val[2] | ATIC_ADJ_DISABLE;
    else
        data_buffer[2] = shtrpd_properties.controlsetting_val[2];

    if(shtrpd_temp_drift)
        data_buffer[2] |= TEMP_DRIFT_MODE;
    
    /*  ControlSettings3(add)    */
    data_buffer[3] = shtrpd_properties.controlsetting_val[3];

    /*  ATI Settings Data   */
    data_buffer[4] = (unsigned char)(shtrpd_properties.atitarget_val>>8);
    /*  ATI Target  */
    data_buffer[5] = (unsigned char)shtrpd_properties.atitarget_val;
    /*  ATI C   */
    data_buffer[6] = shtrpd_properties.atic_val;
    data_buffer[7] = (unsigned char)(shtrpd_properties.atitarget2_val>>8);
    /*  Non-trackpad channels ATI Target    */
    data_buffer[8] = (unsigned char)shtrpd_properties.atitarget2_val;
    /*  Non-trackpad channels ATI C */
    data_buffer[9] = shtrpd_properties.atic2_val;
    data_buffer[10] = shtrpd_properties.re_ati_range;
    
    ret = shtrpd_i2c_user_write_block_data(shtrpd_client, CONTROL_SETTINGS, 
                                       11, data_buffer);

    SHTRPD_DBG("SHTRPDIO_SET_REATI END\n");

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_ioctl_set_keymode                                                  */
/* ------------------------------------------------------------------------- */
static long shtrpd_ioctl_set_keymode(struct file *filp, unsigned int cmd, 
                                                           unsigned long arg)
{
    SHTRPD_DBG("SHTRPDIO_SET_KEYMODE \n");
    
    shtrpd_snap_set_keymode((int)arg);
    
    SHTRPD_DBG("SHTRPDIO_SET_KEYMODE END\n");
    
    return 0;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_ioctl_set_touchcruiser                                             */
/* ------------------------------------------------------------------------- */
static long shtrpd_ioctl_set_touchcruiser(struct file *filp, unsigned int cmd, 
                                                           unsigned long arg)
{
    int i = 0;
    
    struct shtrpd_ts_data *data = i2c_get_clientdata(shtrpd_client);
    
    SHTRPD_DBG("SHTRPDIO_SET_TOUCHCRUISER(%d) \n", (int)arg);
    
    shtrpd_touchcruiser_state = (int)arg;
    
    if(shtrpd_flip_state) {
        shtrpd_user_switch_event_mode(data, true);
        if(!shtrpd_touchcruiser_state) {
            for(i=0; i<MAX_TOUCHES; i++){
                if( shtrpd_finger_state_old & (1<<i)){
                    input_mt_slot(data->input_dev, i);
                    input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, -1);
                    input_sync(data->input_dev);
                    SHTRPD_DBG("ABS_MT_SLOT slot=%d\n",i);
                }
            }
        }
    }
    
    SHTRPD_DBG("SHTRPDIO_SET_TOUCHCRUISER END\n");
    
    return 0;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_ioctl                                                              */
/* ------------------------------------------------------------------------- */
static long shtrpd_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	long rc = -1;

    SHTRPD_DBG("cmd = 0x%x)\n", cmd);

#ifdef SHTRPD_ENABLE_INIT_FW_DL
    if (shtrpd_fwdl_fail_flg) {
        SHTRPD_ERR("init fwdl failed\n");
        return 0;
    }
#endif /* SHTRPD_ENABLE_INIT_FW_DL */

    shtrpd_initialSetup_complete_check();

    mutex_lock(&shtrpd_ioctl_lock);

    switch (cmd) {
    case SHTRPDIO_GET_FW_VERSION:
        rc = shtrpd_ioctl_get_fw_version(filp, cmd, arg);
        break;

    case SHTRPDIO_FW_UPDATE:
        rc = shtrpd_ioctl_fw_update(filp, cmd, arg);
        break;

    case SHTRPDIO_WRITE_IMAGE:
        rc = shtrpd_ioctl_fw_write_image(filp, cmd, arg);
        break;

    case SHTRPDIO_WRITE_SENSOR:
        rc = shtrpd_ioctl_write_sensor(filp, cmd, arg);
        break;

    case SHTRPDIO_READ_SENSOR:
        rc = shtrpd_ioctl_read_sensor(filp, cmd, arg);
        break;

    case SHTRPDIO_RESET:
        rc = shtrpd_ioctl_reset(filp, cmd, arg);
        break;

    case SHTRPDIO_GET_COUNTVALUE:
        rc = shtrpd_ioctl_get_countval(filp, cmd, arg);
        break;

    case SHTRPDIO_PROPERTY:
        rc = shtrpd_ioctl_property(filp, cmd, arg);
        break;

    case SHTRPDIO_SET_CHARGERARMOR:
        rc = shtrpd_ioctl_set_chargerarmor(filp, cmd, arg);
        break;

    case SHTRPDIO_SET_TOUCHEVENT:
        rc = shtrpd_ioctl_set_touchevent(filp, cmd, arg);
        break;

    case SHTRPDIO_SET_REATI:
        rc = shtrpd_ioctl_set_reati(filp, cmd, arg);
        break;

    case SHTRPDIO_SET_KEYMODE:
        rc = shtrpd_ioctl_set_keymode(filp, cmd, arg);
        break;

    case SHTRPDIO_SET_TOUCHCRUISER:
        rc = shtrpd_ioctl_set_touchcruiser(filp, cmd, arg);
        break;

    default:
        mutex_unlock(&shtrpd_ioctl_lock);
        return -ENOTTY;
    }
    mutex_unlock(&shtrpd_ioctl_lock);
    return rc;
}


#define MISC_DEV_NAME  "shtrpd_io_hw"

static struct file_operations shtrpd_fops = {
    .owner   = THIS_MODULE,
    .unlocked_ioctl = shtrpd_ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl   = shtrpd_ioctl,
#endif
};

static struct miscdevice shtrpd_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name  = MISC_DEV_NAME,
    .fops  = &shtrpd_fops,
};

/* ------------------------------------------------------------------------- */
/* shtrpd_user_timing_settings                                               */
/* ------------------------------------------------------------------------- */
/**
 *  Setup the Timing settings for each channel
 *  Low Power Time helps with power consumption when no Prox or Touch
 *  is detected.
 *  At the moment a LP time of 160ms is implemented
 */
static int shtrpd_user_timing_settings(struct shtrpd_ts_data *data, u8 enable)
{
    u8 data_buffer[6];
    int ret;

    if(enable) {
        /*  Timing Settings Data    */
        /*  LTA reseed timer    */
        data_buffer[0] = shtrpd_properties.reseedtime_val;
        /*  Inactive i2c timeout value  */
        data_buffer[1] = shtrpd_properties.commstimeout_val;
        /*  Mode timer value (Switching between modes time) */
        data_buffer[2] = shtrpd_properties.modetime_val;
        /*  Low power time added in low-power state */
        data_buffer[3] = shtrpd_properties.lptime_val;
        /*  Sleep time permanently added    */
        data_buffer[4] = shtrpd_properties.sleeptime_val;
        data_buffer[5] = shtrpd_properties.pmreseedtime;
    } else {
        /*  Timing Settings Data    */
        /*  LTA reseed timer    */
        data_buffer[0] = shtrpd_properties.reseedtime_val;
        /*  Inactive i2c timeout value  */
        data_buffer[1] = shtrpd_properties.commstimeout_val;
        /*  Mode timer value (Switching between modes time) */
        data_buffer[2] = shtrpd_properties.modetime_val;
        /*  Low power time added in low-power state */
        data_buffer[3] = shtrpd_lptime_flipclose_val;
        /*  Sleep time permanently added    */
        data_buffer[4] = shtrpd_properties.sleeptime_val;
        data_buffer[5] = shtrpd_properties.pmreseedtime;
    }
    
    ret = shtrpd_i2c_user_write_block_data(data->client, TIMING_SETTINGS, 6, data_buffer);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_user_thresholds                                                    */
/* ------------------------------------------------------------------------- */
/*  Setup the touch Thresholds for each channel */
static int shtrpd_user_thresholds(struct shtrpd_ts_data *data, u8 enable)
{
    u8 data_buffer[10];
    int ret;
    
    if (enable) {
        /*  Prox Threshold  */
        data_buffer[0] = shtrpd_properties.proxthreshold_val;
        /*  Touch Multiplier    */
        data_buffer[1] = shtrpd_touchmultiplier_val;
        /*  Touch Shifter   */
        data_buffer[2] = shtrpd_touchshifter_val;
        /*  PM Prox Threshold   */
        data_buffer[3] = shtrpd_properties.pmproxthreshold_val;
        /*  Snap threshold  */
        data_buffer[4] = (unsigned char)(shtrpd_snapthreshold_val>>8);
        /*  Snap threshold  */
        data_buffer[5] = (unsigned char)shtrpd_snapthreshold_val;
        /*  Non-trackpad channels prox threshold    */
        data_buffer[6] = shtrpd_properties.proxthreshold2_val;
        /*  Non-trackpad channels Touch Multiplier  */
        data_buffer[7] = shtrpd_touchmultiplier2_val;
        /*  Non-trackpad channels Touch Shifter */
        data_buffer[8] = shtrpd_touchshifter2_val;
        /*  (in-touch) Touch Threshold Multiplier */
        data_buffer[9] = shtrpd_intouchmultiplier_val;
    } else {
        /*  Prox Threshold  */
        data_buffer[0] = shtrpd_proxthreshold_flipclose_val;
        /*  Touch Multiplier    */
        data_buffer[1] = shtrpd_touchmultiplier_flipclose_val;
        /*  Touch Shifter   */
        data_buffer[2] = shtrpd_touchshifter_val;
        /*  PM Prox Threshold   */
        data_buffer[3] = shtrpd_pmproxthreshold_flipclose_val;
        /*  Snap threshold  */
        data_buffer[4] = (unsigned char)(shtrpd_snapthreshold_val>>8);
        /*  Snap threshold  */
        data_buffer[5] = (unsigned char)shtrpd_snapthreshold_val;
        /*  Non-trackpad channels prox threshold    */
        data_buffer[6] = shtrpd_properties.proxthreshold2_val;
        /*  Non-trackpad channels Touch Multiplier  */
        data_buffer[7] = shtrpd_touchmultiplier2_val;
        /*  Non-trackpad channels Touch Shifter */
        data_buffer[8] = shtrpd_touchshifter2_val;
        /*  (in-touch) Touch Threshold Multiplier */
        data_buffer[9] = shtrpd_intouchmultiplier_flipclose_val;
    }
    
    ret = shtrpd_i2c_user_write_block_data(data->client, THRESHOLD_SETTINGS, 10, data_buffer);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_user_switch_event_mode                                             */
/* ------------------------------------------------------------------------- */
static int shtrpd_user_switch_event_mode(struct shtrpd_ts_data *data, u8 enable)
{
    int ret;
    u8 data_buffer[4];

    if(enable) {
        /*  ControlSettings0 */
        data_buffer[0] = shtrpd_properties.controlsetting_val[0] | EVENT_MODE | AUTO_MODES;
        /*  ControlSettings1    */
        if(shtrpd_touchcruiser_state){
            data_buffer[1] = shtrpd_properties.controlsetting_val[1] | LOW_POWER;
        }else{
            data_buffer[1] = shtrpd_properties.controlsetting_val[1] | DIS_TOUCH_EVENT | LOW_POWER;
        }
        /*  ControlSettings2(add)    */
        if(shtrpd_atic_adj_disable)
            data_buffer[2] = shtrpd_properties.controlsetting_val[2] | ATIC_ADJ_DISABLE | PM_RE_ATI | NM_RE_ATI;
        else
            data_buffer[2] = shtrpd_properties.controlsetting_val[2] | PM_RE_ATI | NM_RE_ATI;

        if(shtrpd_temp_drift)
            data_buffer[2] |= TEMP_DRIFT_MODE;
        
        /*  ControlSettings3(add)    */
        data_buffer[3] = shtrpd_properties.controlsetting_val[3];

    } else {
        /*  ControlSettings0 */
        data_buffer[0] = shtrpd_properties.controlsetting_val[0] | EVENT_MODE | AUTO_MODES;
        /*  ControlSettings1    */
        data_buffer[1] = ((shtrpd_properties.controlsetting_val[1] | DIS_TOUCH_EVENT | DIS_SNAP_EVENT | LOW_POWER ) & ~(SNAP_EN));
        /*  ControlSettings2(add)    */
        if(shtrpd_atic_adj_disable)
            data_buffer[2] = shtrpd_properties.controlsetting_val[2] | ATIC_ADJ_DISABLE;
        else
            data_buffer[2] = shtrpd_properties.controlsetting_val[2];

        if(shtrpd_temp_drift)
            data_buffer[2] |= TEMP_DRIFT_MODE;
        
        /*  ControlSettings3(add)    */
        data_buffer[3] = shtrpd_properties.controlsetting_val[3];
    }

    /*  Switch Event/Streaming Mode */
    ret = shtrpd_i2c_user_write_block_data(data->client, CONTROL_SETTINGS, 4, data_buffer);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_user_change_normal_mode                                            */
/* ------------------------------------------------------------------------- */
static int shtrpd_user_change_normal_mode(struct shtrpd_ts_data *data, u8 enable)
{
    int ret;
    u8 data_buffer[4];

    if(enable) {
        /*  ControlSettings0 */
        data_buffer[0] = shtrpd_properties.controlsetting_val[0] | EVENT_MODE;
        /*  ControlSettings1    */
        if(shtrpd_touchcruiser_state){
            data_buffer[1] = shtrpd_properties.controlsetting_val[1];
        }else{
            data_buffer[1] = shtrpd_properties.controlsetting_val[1] | DIS_TOUCH_EVENT;
        }
        /*  ControlSettings2(add)    */
        if(shtrpd_atic_adj_disable)
            data_buffer[2] = shtrpd_properties.controlsetting_val[2] | ATIC_ADJ_DISABLE | PM_RE_ATI | NM_RE_ATI;
        else
            data_buffer[2] = shtrpd_properties.controlsetting_val[2] | PM_RE_ATI | NM_RE_ATI;

        if(shtrpd_temp_drift)
            data_buffer[2] |= TEMP_DRIFT_MODE;
        
        /*  ControlSettings3(add)    */
        data_buffer[3] = shtrpd_properties.controlsetting_val[3];

    } else {
        /*  ControlSettings0 */
        data_buffer[0] = shtrpd_properties.controlsetting_val[0] | EVENT_MODE;
        /*  ControlSettings1    */
        data_buffer[1] = ((shtrpd_properties.controlsetting_val[1] | DIS_TOUCH_EVENT | DIS_SNAP_EVENT) & ~(SNAP_EN));
        /*  ControlSettings2(add)    */
        if(shtrpd_atic_adj_disable)
            data_buffer[2] = shtrpd_properties.controlsetting_val[2] | ATIC_ADJ_DISABLE;
        else
            data_buffer[2] = shtrpd_properties.controlsetting_val[2];

        if(shtrpd_temp_drift)
            data_buffer[2] |= TEMP_DRIFT_MODE;
        
        /*  ControlSettings3(add)    */
        data_buffer[3] = shtrpd_properties.controlsetting_val[3];
    }

    /*  Switch Event/Streaming Mode */
    ret = shtrpd_i2c_user_write_block_data(data->client, CONTROL_SETTINGS, 4, data_buffer);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_clear_xy_info                                                      */
/* ------------------------------------------------------------------------- */
/*  Set the Trackpad Device to Event Mode or Streaming mode */
static int shtrpd_clear_xy_info(struct shtrpd_ts_data *data, u8 *buffer)
{
    int ret;

    ret = shtrpd_i2c_user_read_block_data(data->client, XY_DATA, SHTRPD_BLOCK_SIZE, buffer);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_flip_open                                                          */
/* ------------------------------------------------------------------------- */
static void shtrpd_flip_open(struct work_struct *work)
{
    int ret = 0;
    struct shtrpd_ts_data *data = i2c_get_clientdata(shtrpd_client);

    mutex_lock(&shtrpd_ioctl_lock);

    SHTRPD_DBG("start\n");

    if (shtrpd_flip_reset_state) {
        SHTRPD_ERR("reset state\n");
        shtrpd_flip_reset_state = 0;
        goto reset_done;
    }

    if (initialSetup) {
        SHTRPD_ERR("initial setup\n");
        goto irq_enable;
    }

    shtrpd_sys_enable_irq();

    ret = shtrpd_user_change_normal_mode(data, 0);
    if(ret < 0){
        SHTRPD_ERR("shtrpd_change_normal_mode err, ret:%d\n", ret)
        goto reset;
    }

    ret = shtrpd_user_timing_settings(data, 1);
    if(ret < 0) {
        SHTRPD_ERR("shtrpd_user_timing_settings err, ret:%d\n", ret);
        goto reset;
    }

#if defined( SHTRPD_CHECK_CPU_TEMP_ENABLE )
    if(SHTRPD_CHECK_CPU_TEMP){
        SHTRPD_LOG_CHECK_CPU_TEMP("cpu temp check timer start\n");
        cancel_delayed_work(&shtrpd_work_cpu_temp);
        queue_delayed_work(shtrpd_work_queue, &shtrpd_work_cpu_temp, msecs_to_jiffies(SHTRPD_CHECK_CPU_TEMP_INTERVAL));
    }
    ret = shtrpd_thresholds_check_cpu_temp(1);
    if(ret < 0) {
        SHTRPD_ERR("shtrpd_thresholds_check_cpu_temp err, ret:%d\n", ret);
        goto reset;
    }
#else /* SHTRPD_CHECK_CPU_TEMP_ENABLE */
    ret = shtrpd_user_thresholds(data, 1);
    if(ret < 0) {
        SHTRPD_ERR("shtrpd_user_thresholds err, ret:%d\n", ret);
        goto reset;
    }
#endif /* SHTRPD_CHECK_CPU_TEMP_ENABLE */

    ret = shtrpd_user_switch_event_mode(data, 1);
    if(ret < 0) {
        SHTRPD_ERR("shtrpd_switch_event_mode err, ret:%d\n", ret);
        goto reset;
    }

#if defined(SHTRPD_NOINTERRUPT_IN_TOUCH_CHECK_ENABLE)
    shtrpd_nonint_check_delay_start();
#endif /* SHTRPD_NOINTERRUPT_IN_TOUCH_CHECK_ENABLE */

#if defined(SHTRPD_FAIL_TOUCH_CLEAR_ENABLE)
   	shtrpd_fail_touch_clear_reati(data);
	shtrpd_fail_touch_clear_init();
#endif /* SHTRPD_FAIL_TOUCH_CLEAR_ENABLE */

    shtrpd_flip_state = 1;
    mutex_unlock(&shtrpd_ioctl_lock);
    return;

reset:
    shtrpd_sys_disable_irq();
    /* NRST H->L->H */
    gpio_set_value(NRST_LINE_PIN,0);
    usleep(100000);
reset_done:
    gpio_set_value(NRST_LINE_PIN,1);
    usleep(SHTRPD_NRST_DELAY);
    shtrpd_ts_phys_init();
irq_enable:
#if defined(SHTRPD_NOINTERRUPT_IN_TOUCH_CHECK_ENABLE)
    shtrpd_nonint_check_delay_start();
#endif /* SHTRPD_NOINTERRUPT_IN_TOUCH_CHECK_ENABLE */
#if defined(SHTRPD_FAIL_TOUCH_CLEAR_ENABLE)
	shtrpd_fail_touch_clear_init();
#endif /* SHTRPD_FAIL_TOUCH_CLEAR_ENABLE */
    shtrpd_flip_state = 1;
    shtrpd_sys_enable_irq();
    mutex_unlock(&shtrpd_ioctl_lock);
    return;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_flip_close_proc                                                    */
/* ------------------------------------------------------------------------- */
static int shtrpd_flip_close_proc(void)
{
    int ret = 0;
    u8 buffer[40];
    struct shtrpd_ts_data *data = i2c_get_clientdata(shtrpd_client);

    ret = shtrpd_user_change_normal_mode(data, 1);
    if(ret < 0){
        SHTRPD_ERR("shtrpd_change_normal_mode err, ret:%d\n", ret)
        return -1;
    }

    ret = shtrpd_user_thresholds(data, 0);
    if(ret < 0){
        SHTRPD_ERR("shtrpd_user_thresholds err, ret:%d\n", ret)
        return -1;
    }

    ret = shtrpd_user_timing_settings(data, 0);
    if(ret < 0){
        SHTRPD_ERR("shtrpd_user_timing_settings err, ret:%d\n", ret)
        return -1;
    }

    ret = shtrpd_user_switch_event_mode(data, 0);
    if(ret < 0){
        SHTRPD_ERR("shtrpd_disable_event_mode err, ret:%d\n", ret)
        return -1;
    }
    
    ret = shtrpd_clear_xy_info(data, buffer);
    if(ret >= 0){
        if(buffer[0] & SHOW_RESET) {
            return -1;
        }
    }
    return 0;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_flip_close                                                         */
/* ------------------------------------------------------------------------- */
static void shtrpd_flip_close(struct work_struct *work)
{
    int ret;

    mutex_lock(&shtrpd_ioctl_lock);

    shtrpd_flip_state = 0;

    SHTRPD_DBG("start\n");

#if defined( SHTRPD_CHECK_CPU_TEMP_ENABLE )
    SHTRPD_LOG_CHECK_CPU_TEMP("cpu temp check timer stop\n");
    cancel_delayed_work(&shtrpd_work_cpu_temp);
#endif /* SHTRPD_CHECK_CPU_TEMP_ENABLE */

#if defined(SHTRPD_SNAP_USER_MASK)
    if(SHTRPD_SNAP_USER_MASK_CLEAR_IN_FLIP){
        shtrpd_snap_set_keymode(TRPD_KEYMODE_NORMAL);
    }
#endif /* SHTRPD_SNAP_USER_MASK */

    if (initialSetup) {
        SHTRPD_ERR("initial setup\n");
        goto reset;
    }

    ret = shtrpd_flip_close_proc();
    if(ret < 0){
        goto reset;
    }

    shtrpd_sys_disable_irq();
    shtrpd_touch_force_off();
#if defined(SHTRPD_NOINTERRUPT_IN_TOUCH_CHECK_ENABLE)
    shtrpd_nonint_check_delay_cancel();
#endif /* SHTRPD_NOINTERRUPT_IN_TOUCH_CHECK_ENABLE */
#if defined(SHTRPD_INTERRUPT_TIMING_CHECK_ENABLE)
    shtrpd_int_timing_check_delay_cancel();
#endif /* SHTRPD_INTERRUPT_TIMING_CHECK_ENABLE */
    mutex_unlock(&shtrpd_ioctl_lock);
    return;
reset:
    SHTRPD_ERR("reset setting exec \n");
    shtrpd_sys_disable_irq();
    shtrpd_touch_force_off();
#if defined(SHTRPD_NOINTERRUPT_IN_TOUCH_CHECK_ENABLE)
    shtrpd_nonint_check_delay_cancel();
#endif /* SHTRPD_NOINTERRUPT_IN_TOUCH_CHECK_ENABLE */
#if defined(SHTRPD_INTERRUPT_TIMING_CHECK_ENABLE)
    shtrpd_int_timing_check_delay_cancel();
#endif /* SHTRPD_INTERRUPT_TIMING_CHECK_ENABLE */
    gpio_set_value(NRST_LINE_PIN, 0);
    usleep(100000);
    shtrpd_flip_reset_state = 1;
    mutex_unlock(&shtrpd_ioctl_lock);
    return;
}

#ifdef SHTRPD_ENABLE_INIT_FW_DL
/* ------------------------------------------------------------------------- */
/* shtrpd_probe_fw_write_err_proc                                            */
/* ------------------------------------------------------------------------- */
static int shtrpd_probe_fw_write_err_proc(const struct i2c_client *client)
{
    int ret;
    u8 ver[2];

    SHTRPD_ERR("start \n");
    
    ret = shtrpd_enter_bootloader_mode(client);
    if (ret) {
        SHTRPD_ERR("shtrpd_enter_bootloader_mode Err, ret:%d\n", ret);
        return ret;
    }
    
    ret = shtrpd_i2c_exec_app(client);
    if (ret) {
        SHTRPD_ERR("shtrpd_i2c_exec_app Err, ret:%d\n", ret);
        return ret;
    }

    usleep(shtrpd_exec_app_wait);

    shtrpd_event_mode_handshake();

    usleep(shtrpd_handshake_wait);

    ret = shtrpd_resetDeviceWithBootLoadInstruction();
    if (ret) {
        SHTRPD_ERR("shtrpd_resetDeviceWithBootLoadInstruction Err, ret:%d\n", ret);
        return ret;
    }

    usleep(shtrpd_bootloader_key_wait);

    ret = shtrpd_i2c_readBootloaderVersion(client, ver);
    if(ret){
        SHTRPD_DBG("Cannot read Bootloader version !!!\n");
        return ret;
    }

    ret = shtrpd_programBootloader((u8*)&hex_array[0], SHTRPD_FW_IMAGE_MAX);
    if (ret) {
        SHTRPD_ERR("shtrpd_fw_write Err, ret:%d\n", ret);
    }

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_probe_fw_write                                                     */
/* ------------------------------------------------------------------------- */
static int shtrpd_probe_fw_write(const struct i2c_client *client)
{
    int i = 0;
    int err = 0;
    
    SHTRPD_ERR("start\n");
    
    for (i = 0; i < 3; i++) {
        err = shtrpd_fw_write((u8*)&hex_array[0], SHTRPD_FW_IMAGE_MAX);
        if (err) {
            SHTRPD_ERR("shtrpd_fw_write NG \n");
            err = shtrpd_probe_fw_write_err_proc(client);
            if (!err) {
                SHTRPD_ERR("shtrpd_probe_fw_write_err_proc OK \n");
                break;
            }
            shtrpd_enter_bootloader_mode(client);
        } else {
            SHTRPD_ERR("shtrpd_probe_fw_write OK \n");
            break;
        }
    }
    
    return err;
}
#endif /* SHTRPD_ENABLE_INIT_FW_DL */

#if defined (CONFIG_ANDROID_ENGINEERING)
struct shtrpd_procfs {
    u8 id;
    u8 par[66];
};
static int shtrpd_proc_write(struct file *file, const char *buffer, unsigned long count, void *data)
{
#define SHTRPD_LEN_ID    (2)
#define SHTRPD_LEN_PARAM (2)
#define SHTRPD_PARAM_MAX (66)

    unsigned long len = count;
    struct shtrpd_procfs shtrpd_pfs;
    char buf[SHTRPD_LEN_PARAM + 1];
    char kbuf[SHTRPD_LEN_ID + SHTRPD_PARAM_MAX * SHTRPD_LEN_PARAM];
    int i, j;
    int ret = 0;
    u8 data_buffer[64];
    
    len--;
    /* Check length */
    if (len < SHTRPD_LEN_ID){
        return count;
    }
    if (len > (SHTRPD_LEN_ID + SHTRPD_PARAM_MAX * SHTRPD_LEN_PARAM)){
       len = SHTRPD_LEN_ID + SHTRPD_PARAM_MAX * SHTRPD_LEN_PARAM;
    }
    if (copy_from_user(kbuf, buffer, len)){
        return -EFAULT;
    }
    
    /* Get FunctionID */
    memcpy(buf, kbuf, SHTRPD_LEN_ID);
    buf[SHTRPD_LEN_ID] = '\0';
    shtrpd_pfs.id = simple_strtol(buf, NULL, 10);
    memset(shtrpd_pfs.par, 0, sizeof(shtrpd_pfs.par));
    
    /* Get Parameters */
    for(i = 0; (i + 1) * SHTRPD_LEN_PARAM <= (len - SHTRPD_LEN_ID); i++){
        memcpy(buf, &(kbuf[SHTRPD_LEN_ID + i * SHTRPD_LEN_PARAM]), SHTRPD_LEN_PARAM);
        buf[SHTRPD_LEN_PARAM] = '\0';
        shtrpd_pfs.par[i] = simple_strtol(buf, NULL, 16);
    }
    
    printk("id:%d, Addr:0x%02x%02x\n", shtrpd_pfs.id, shtrpd_pfs.par[0], shtrpd_pfs.par[1]);
    
    switch (shtrpd_pfs.id){
    case 0:
        for (i = 0; i < 3; i++) {
            ret = shtrpd_enter_bootloader_mode(shtrpd_client);
            if (!ret) {
                break;
            }
        }
        break;
    case 1:
        for (i = 0; i < 3; i++) {
            ret = shtrpd_i2c_exec_app(shtrpd_client);
            if (!ret) {
                break;
            }
        }
        break;
    case 2:
        ret = shtrpd_i2c_read_Bootloader_data(shtrpd_client, 
                              shtrpd_pfs.par[0], shtrpd_pfs.par[1], data_buffer);
        if( SHTRPD_BOOT_SUCCESS != ret ){
            SHTRPD_ERR("i2c read error from Bootloader (ret=%04x) \n", ret);
            return -1;
        }
        if (shtrpd_dump_log) {
            for(j=1; j<SHTRPD_BOOT_BLOCK_SIZE+1; j++){
                if(j % 16)
                    printk("0x%02X, ", data_buffer[j-1]);
                else
                    printk("0x%02X\n", data_buffer[j-1]);
            }
        }
        break;
    case 3:
        memcpy(data_buffer, &shtrpd_pfs.par[2], SHTRPD_BOOT_BLOCK_SIZE);
        if (shtrpd_dump_log) {
            for(j=1; j<SHTRPD_BOOT_BLOCK_SIZE+1; j++){
                if(j % 16)
                    printk("0x%02X, ", data_buffer[j-1]);
                else
                    printk("0x%02X\n", data_buffer[j-1]);
            }
        }
        ret = shtrpd_i2c_write_Bootloader_data(shtrpd_client,
                               shtrpd_pfs.par[0], shtrpd_pfs.par[1], data_buffer);
        if( SHTRPD_BOOT_SUCCESS != ret ){
            SHTRPD_ERR("i2c write error from Bootloader (ret=%04x) \n", ret);
            return -1;
        }
        break;
    case 4:
        shtrpd_sys_disable_irq();
        shtrpd_probe_fw_write_err_proc(shtrpd_client);
        shtrpd_sys_enable_irq();
        break;
    default:
        break;
    }

    return count;
}
#endif /* CONFIG_ANDROID_ENGINEERING */

/* ------------------------------------------------------------------------- */
/* shtrpd_ts_probe                                                             */
/* ------------------------------------------------------------------------- */
/*
 *  The probe function is called when Android is
 *  looking for the Trackpad Device on the I2C bus (I2C-2)
 */
static int shtrpd_ts_probe(struct i2c_client *client,
        const struct i2c_device_id *id)
{
    struct shtrpd_ts_data *data;
    struct input_dev *input_dev;
    int ret;
    u8 buffer[10];
    int err;
#ifdef SHTRPD_DBG_INITIAL_BOOT_CHK
    s32 firm_chk = SHTRPD_FM_CHK_APP_EXEC;
#endif
#ifdef SHTRPD_ENABLE_INIT_FW_DL
    unsigned short fwver = 0;
#endif /* SHTRPD_ENABLE_INIT_FW_DL */
    unsigned char handset = 0;
    unsigned short revision_state = 0;
#ifdef SHTRPD_ENABLE_INIT_FW_DL
    int i;
#ifdef SHTRPD_ENABLE_INIT_FW_DL_CRC_CHECK
    int crc = -1;
#endif /* SHTRPD_ENABLE_INIT_FW_DL_CRC_CHECK */
#endif /* SHTRPD_ENABLE_INIT_FW_DL */
#if defined (CONFIG_ANDROID_ENGINEERING)
    struct proc_dir_entry *entry;
#endif /* CONFIG_ANDROID_ENGINEERING */

    buffer[0] = 0;
    buffer[1] = 0;

    SHTRPD_WARN("Trackpad driver - Starting Probe...\n");

    handset = sh_boot_get_handset();
    if (!handset) {
        shtrpd_snapthreshold_val = SNAPTHRESHOLD_VAL_ES1;
#if defined( SHTRPD_CHECK_CPU_TEMP_ENABLE )
        SHTRPD_CHECK_CPU_TEMP = 0;
#endif /* SHTRPD_CHECK_CPU_TEMP_ENABLE */
    } else {
        revision_state = sh_boot_get_hw_revision();
        switch(revision_state) {
        case SHTRDP_HW_REVISION_ES0:
            shtrpd_snapthreshold_val = SNAPTHRESHOLD_VAL_ES1;
#if defined( SHTRPD_CHECK_CPU_TEMP_ENABLE )
            SHTRPD_CHECK_CPU_TEMP = 0;
#endif /* SHTRPD_CHECK_CPU_TEMP_ENABLE */
            break;
        case SHTRDP_HW_REVISION_ES1:
            shtrpd_snapthreshold_val = SNAPTHRESHOLD_VAL_ES1;
#if defined( SHTRPD_CHECK_CPU_TEMP_ENABLE )
            SHTRPD_CHECK_CPU_TEMP = 0;
#endif /* SHTRPD_CHECK_CPU_TEMP_ENABLE */
            break;
        case SHTRDP_HW_REVISION_PP1:
            shtrpd_snapthreshold_val = SNAPTHRESHOLD_VAL_PP1;
#if defined( SHTRPD_CHECK_CPU_TEMP_ENABLE )
            SHTRPD_CHECK_CPU_TEMP = 0;
#endif /* SHTRPD_CHECK_CPU_TEMP_ENABLE */
            break;
        case SHTRDP_HW_REVISION_PP2:
            shtrpd_snapthreshold_val = SNAPTHRESHOLD_VAL_PP2;
#if defined( SHTRPD_CHECK_CPU_TEMP_ENABLE )
            SHTRPD_CHECK_CPU_TEMP = 1;
#endif /* SHTRPD_CHECK_CPU_TEMP_ENABLE */
            break;
        default:
            shtrpd_snapthreshold_val = SNAPTHRESHOLD_VAL_PP2;
#if defined( SHTRPD_CHECK_CPU_TEMP_ENABLE )
            SHTRPD_CHECK_CPU_TEMP = 1;
#endif /* SHTRPD_CHECK_CPU_TEMP_ENABLE */
            break;
        }
    }

    /*  Allocate memory */
    data = kzalloc(sizeof(struct shtrpd_ts_data), GFP_KERNEL);
    input_dev = input_allocate_device();
    if (!data || !input_dev) {
        SHTRPD_ERR("Failed to allocate memory\n");
        ret = -ENOMEM;
        goto err_free_mem;
    }

    spin_lock_init(&shtrpd_spinlock);

    /*  Save the stuctures to be used in the driver */
    data->client = client;
    data->input_dev = input_dev;
    data->platform_data = client->dev.platform_data;

    input_dev->name = "Trackpad driver";
    input_dev->id.bustype = BUS_I2C;
    input_dev->dev.parent = &client->dev;

    gpio_request(NRST_LINE_PIN, NULL);
    ret = gpio_tlmm_config(GPIO_CFG(NRST_LINE_PIN, 0,
            GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
    if (ret) {
        SHTRPD_ERR("GPIO(NRST_LINE_PIN) set config NG\n");
    }

    ret = gpio_direction_output(NRST_LINE_PIN, 1);
    if (ret) {
        SHTRPD_ERR("GPIO(NRST_LINE_PIN) set direction NG\n");
    }

    gpio_request(RDY_LINE_PIN, NULL);
    ret = gpio_tlmm_config(GPIO_CFG(RDY_LINE_PIN, 0,
            GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
    if (ret) {
        SHTRPD_ERR("GPIO(RDY_LINE_PIN) set config NG\n");
    }

    ret = gpio_direction_input(RDY_LINE_PIN);
    if (ret) {
        SHTRPD_ERR("GPIO(RDY_LINE_PIN) set direction NG\n");
    }

    usleep(SHTRPD_NRST_DELAY);

    /*  Register GPIO to IRQ    */
    client->irq = gpio_to_irq(RDY_LINE_PIN);
    /*  Enable this IRQ */
    /*  enable_irq(client->irq); */

    /*  Give settings to Android such as the minimum and
     *  maximum X and Y positions that will be reported
     */
    __set_bit(EV_ABS, input_dev->evbit);
    __set_bit(EV_KEY, input_dev->evbit);
/*  __set_bit(BTN_TOUCH, input_dev->keybit); */
    __set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

    /*  Setup Input to Android  */
    input_set_abs_params(input_dev, ABS_MT_SLOT,
        0, MAX_TOUCHES-1, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_TRACKING_ID,
        0, MAX_TOUCHES, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0,
        65535, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_POSITION_X,
        0, SHTRPD_NOTIFY_MAX_X, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
        0, SHTRPD_NOTIFY_MAX_Y, 0, 0);

    input_mt_init_slots(input_dev, MAX_TOUCHES);

    /* Device capabilities for key */
    input_set_capability(input_dev, EV_KEY, KEY_UP);
    input_set_capability(input_dev, EV_KEY, KEY_DOWN);
    input_set_capability(input_dev, EV_KEY, KEY_RIGHT);
    input_set_capability(input_dev, EV_KEY, KEY_LEFT);
    input_set_capability(input_dev, EV_KEY, KEY_ENTER);
    input_set_capability(input_dev, EV_KEY, KEY_S1);
    input_set_capability(input_dev, EV_KEY, KEY_S2);
    input_set_capability(input_dev, EV_KEY, KEY_S3);
    input_set_capability(input_dev, EV_KEY, KEY_S4);
    input_set_capability(input_dev, EV_KEY, KEY_S5);
    input_set_capability(input_dev, EV_KEY, KEY_S6);
    input_set_capability(input_dev, EV_KEY, KEY_BACKSPACE);
    input_set_capability(input_dev, EV_KEY, KEY_PHONE);
    input_set_capability(input_dev, EV_KEY, KEY_POUND);
    input_set_capability(input_dev, EV_KEY, KEY_STAR);
    input_set_capability(input_dev, EV_KEY, KEY_1);
    input_set_capability(input_dev, EV_KEY, KEY_2);
    input_set_capability(input_dev, EV_KEY, KEY_3);
    input_set_capability(input_dev, EV_KEY, KEY_4);
    input_set_capability(input_dev, EV_KEY, KEY_5);
    input_set_capability(input_dev, EV_KEY, KEY_6);
    input_set_capability(input_dev, EV_KEY, KEY_7);
    input_set_capability(input_dev, EV_KEY, KEY_8);
    input_set_capability(input_dev, EV_KEY, KEY_9);
    input_set_capability(input_dev, EV_KEY, KEY_0);

    /*  Set driver data */
    input_set_drvdata(input_dev, data);

    memset(&userRequest, 0, sizeof(userRequest));

    shtrpd_properties_init();

#if defined(SHTRPD_CPU_IDLE_SLEEP_CONTROL_ENABLE)
	shtrpd_wake_lock_init();
#endif /* SHTRPD_CPU_IDLE_SLEEP_CONTROL_ENABLE */

#if defined(SHTRPD_CPU_CLOCK_CONTROL_ENABLE)
	shtrpd_perf_lock_init();
#endif /* SHTRPD_CPU_CLOCK_CONTROL_ENABLE */

#if defined(SHTRPD_DRUMMING_SPLIT_SWITCH_FINGER_ENABLE)
    shtrpd_replace_fingers_init();
#endif /* SHTRPD_DRUMMING_SPLIT_SWITCH_FINGER_ENABLE */

    i2c_set_clientdata(client, data);
    shtrpd_client = client;

    /*  Call the pysical init for first communication
     *  to setup Trackpad device
     */
    shtrpd_ts_phys_init();

#ifdef SHTRPD_ENABLE_INIT_FW_DL
#ifdef SHTRPD_ENABLE_INIT_FW_DL_CRC_CHECK
    for (i = 0; i < 3; i++) {
        err = shtrpd_enter_bootloader_mode(shtrpd_client);
        if (!err) {
            crc = shtrpd_i2c_firm_crc_check(shtrpd_client);
            break;
        }
    }
    
    if (crc < 0) {
        err = shtrpd_probe_fw_write(shtrpd_client);
        if (!err) {
            shtrpd_i2c_exec_app(shtrpd_client);
            usleep(shtrpd_exec_app_wait);
        } else {
            SHTRPD_ERR("init fwdl failed[%d]\n", err);
            shtrpd_fwdl_fail_flg = 1;
            gpio_set_value(NRST_LINE_PIN, 0);
            goto err_free_mem;
        }
    } else {
        shtrpd_i2c_exec_app(shtrpd_client);
        usleep(shtrpd_exec_app_wait);
        err = shtrpd_i2c_read_block_data(shtrpd_client, VERSION_INFO, 10, buffer);
        if (err < 0) {
            SHTRPD_ERR("i2c read err[%d]\n", err);
        } else {
            SHTRPD_DBG("Version: - %d, %d, %d, %d, %d, %d, %02x, %02x, %02x, %02x\n", 
                        buffer[0], buffer[1],buffer[2], buffer[3],buffer[4], 
                        buffer[5], buffer[6],buffer[7], buffer[8],buffer[9]);

            fwver = ((buffer[4] << 8) | buffer[5]);

            if (fwver != SHTRPD_FW_VERSION) {
                SHTRPD_ERR("shtrpd_fw_update start\n");
                err = shtrpd_enter_bootloader_mode(shtrpd_client);
                if (!err) {
                    err = shtrpd_probe_fw_write(shtrpd_client);
                    if (!err) {
                        shtrpd_i2c_exec_app(shtrpd_client);
                        usleep(shtrpd_exec_app_wait);
                    } else {
                        SHTRPD_ERR("init fwdl failed[%d]\n", err);
                        shtrpd_fwdl_fail_flg = 1;
                        gpio_set_value(NRST_LINE_PIN, 0);
                        goto err_free_mem;
                    }
                }
            }
        }
    }
#else /* SHTRPD_ENABLE_INIT_FW_DL_CRC_CHECK */
    for (i = 0; i < 3; i++) {
        err = shtrpd_i2c_read_block_data(shtrpd_client, VERSION_INFO, 10, buffer);
        if (err < 0) {
            SHTRPD_ERR("Version Read i2c read err[%d]\n", err);
        } else {
            break;
        }
    }
    
    if (err < 0) {
        SHTRPD_ERR("shtrpd_fw_update start\n");
        err = shtrpd_enter_bootloader_mode(shtrpd_client);
        if (!err) {
            err = shtrpd_probe_fw_write(shtrpd_client);
            if (!err) {
                shtrpd_i2c_exec_app(shtrpd_client);
                usleep(shtrpd_exec_app_wait);
            } else {
                SHTRPD_ERR("init fwdl failed[%d]\n", err);
                shtrpd_fwdl_fail_flg = 1;
                gpio_set_value(NRST_LINE_PIN, 0);
                goto err_free_mem;
            }
        }
    } else {
        SHTRPD_DBG("Version: - %d, %d, %d, %d, %d, %d, %02x, %02x, %02x, %02x\n", 
                        buffer[0], buffer[1],buffer[2], buffer[3],buffer[4], 
                        buffer[5], buffer[6],buffer[7], buffer[8],buffer[9]);

        fwver = ((buffer[4] << 8) | buffer[5]);

        if (fwver != SHTRPD_FW_VERSION) {
            SHTRPD_ERR("shtrpd_fw_update start\n");
            err = shtrpd_enter_bootloader_mode(shtrpd_client);
            if (!err) {
                err = shtrpd_probe_fw_write(shtrpd_client);
                if (!err) {
                    shtrpd_i2c_exec_app(shtrpd_client);
                    usleep(shtrpd_exec_app_wait);
                } else {
                    SHTRPD_ERR("init fwdl failed[%d]\n", err);
                    shtrpd_fwdl_fail_flg = 1;
                    gpio_set_value(NRST_LINE_PIN, 0);
                    goto err_free_mem;
                }
            }
        }
    }
#endif /* SHTRPD_ENABLE_INIT_FW_DL_CRC_CHECK */
#else /* SHTRPD_ENABLE_INIT_FW_DL */
    /*  Trackpad device Version Info    */
    err = shtrpd_i2c_read_block_data(shtrpd_client, VERSION_INFO, 10, buffer);
    if (err < 0) {
        SHTRPD_ERR("i2c read err[%d]\n", err);
    }
    SHTRPD_INFO("Version: - %d, %d, %d, %d, %d, %d, %02x, %02x, %02x, %02x\n", 
                buffer[0], buffer[1],buffer[2], buffer[3],buffer[4], 
                buffer[5], buffer[6],buffer[7], buffer[8],buffer[9]);
#endif /* SHTRPD_ENABLE_INIT_FW_DL */

    ret = msm_flip_get_state();
    if(!ret) {
        gpio_set_value(NRST_LINE_PIN, 0);
        shtrpd_flip_state = 0;
        shtrpd_flip_old_state = 0;
        shtrpd_flip_reset_state = 1;
    } else {
        shtrpd_flip_state = 1;
        shtrpd_flip_old_state = 1;
    }

    /*  Request the interrupt on a rising trigger
     *  and only one trigger per rising edge
     */
    ret = request_threaded_irq(client->irq, NULL, shtrpd_ts_interrupt,
            IRQF_TRIGGER_RISING | IRQF_ONESHOT, DEVICE_NAME, data);

    /*  Could not register interrupt    */
    if (ret < 0) {
        SHTRPD_ERR("Failed to register interrupt\n");
        goto err_free_mem;
    }

    shtrpd_irq_enable   = SHTRPD_IRQ_ENABLED;

    /*  Register the device */
    ret = input_register_device(data->input_dev);
    if (ret < 0)
        goto err_free_irq;

    ret = misc_register(&shtrpd_device);
    if (ret) {
        SHTRPD_ERR("misc_register Error!(ret=%d)\n", ret);
        goto err_free_irq;
    }

    shtrpd_work_queue = create_singlethread_workqueue("shtrpd_workqueue");
    if (shtrpd_work_queue) {
        INIT_WORK(&shtrpd_work_flip_open, shtrpd_flip_open);
        INIT_WORK(&shtrpd_work_flip_close, shtrpd_flip_close);
#if defined( SHTRPD_CHECK_CPU_TEMP_ENABLE )
        INIT_DELAYED_WORK(&shtrpd_work_cpu_temp, shtrpd_check_cpu_temp_func);
#endif /* SHTRPD_CHECK_CPU_TEMP_ENABLE */
#if defined( SHTRPD_TOUCH_UP_DELAY_ENABLE )
        INIT_DELAYED_WORK(&shtrpd_work_tu_delay, shtrpd_tu_delay_func);
#endif /* SHTRPD_TOUCH_UP_DELAY_ENABLE */
        INIT_DELAYED_WORK(&shtrpd_work_snap_err_delay, shtrpd_snap_err_delay_func);
#if defined( SHTRPD_NOINTERRUPT_IN_TOUCH_CHECK_ENABLE )
        INIT_DELAYED_WORK(&shtrpd_work_noint_check_delay, shtrpd_nonint_check_delay_func);
        INIT_DELAYED_WORK(&shtrpd_work_pwkey_delay, shtrpd_pwkey_delay_func);
#endif /* SHTRPD_NOINTERRUPT_IN_TOUCH_CHECK_ENABLE */
#if defined(SHTRPD_INTERRUPT_TIMING_CHECK_ENABLE)
        INIT_DELAYED_WORK(&shtrpd_work_int_timing_delay, shtrpd_int_timing_delay_func);
#endif /* SHTRPD_INTERRUPT_TIMING_CHECK_ENABLE */
    }

#ifdef SHTRPD_DBG_INITIAL_BOOT_CHK
    firm_chk = shtrpd_fw_check();
    if( SHTRPD_FM_CHK_BOOTLOADER_MODE == firm_chk){
        bootLoaderMode = true;
        SHTRPD_DBG("bootloader mode state \n");
    }
    else{
        bootLoaderMode = false;

        /* APP exec */
        shtrpd_i2c_exec_app(shtrpd_client);
        SHTRPD_DBG("firm update end, APP exec start\n");

        shtrpd_initialSetup_complete_check();

        /* wait first XY-Info read */
        shtrpd_firstXYInfoRead_complete_check();
    }
#else
    bootLoaderMode = false;
#endif
    /*  Register the Trackpad device TS Driver for Early Suspend
     *  and Late Resume. This use when the 'Home' or
     *  'Power' Button is pressed - Android calls your
     *  driver with this info and you can act accordingly
     */
    #ifdef CONFIG_HAS_EARLYSUSPEND
        data->early_suspend.level =
            EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
        data->early_suspend.suspend = shtrpd_ts_early_suspend;
        data->early_suspend.resume = shtrpd_ts_late_resume;
        register_early_suspend(&data->early_suspend);
    #endif

    memset(old_keycode, 0, sizeof(old_keycode));

#ifdef CONFIG_ANDROID_ENGINEERING
    entry = create_proc_entry("driver/SHTRPD", 0666, NULL);
    if (entry) {
        entry->write_proc = shtrpd_proc_write;
    }
#endif /* #ifdef CONFIG_ANDROID_ENGINEERING */

#if defined(SHTRPD_CHECK_CPU_TEMP_ENABLE) && defined(CONFIG_ANDROID_ENGINEERING)
    shtrpd_snapdebug_init();
#endif /* SHTRPD_CHECK_CPU_TEMP_ENABLE && CONFIG_ANDROID_ENGINEERING */

    return 0;

err_free_irq:
    free_irq(client->irq, data);
err_free_mem:
    input_free_device(input_dev);
    kfree(data);
#ifdef CONFIG_ANDROID_ENGINEERING
    entry = create_proc_entry("driver/SHTRPD", 0666, NULL);
    if (entry) {
        entry->write_proc = shtrpd_proc_write;
    }
#endif /* #ifdef CONFIG_ANDROID_ENGINEERING */
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_ts_remove                                                          */
/* ------------------------------------------------------------------------- */
/*  Removing of the Trackpad driverr    */
static int __devexit shtrpd_ts_remove(struct i2c_client *client)
{
    struct shtrpd_ts_data *data = i2c_get_clientdata(client);

    SHTRPD_WARN("Removing Trackpad driver... ");

#if defined(SHTRPD_CPU_IDLE_SLEEP_CONTROL_ENABLE)
	shtrpd_wake_lock_deinit();
#endif /* SHTRPD_CPU_IDLE_SLEEP_CONTROL_ENABLE */

    if (shtrpd_work_queue) {
        flush_workqueue(shtrpd_work_queue);
        destroy_workqueue(shtrpd_work_queue);
        shtrpd_work_queue = NULL;
    }

    /*  Unregister the Early suspend    */
    unregister_early_suspend(&data->early_suspend);
    free_irq(client->irq, data);
    input_unregister_device(data->input_dev);
    kfree(data);
    i2c_set_clientdata(client, NULL);

    return 0;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_ts_suspend                                                         */
/* ------------------------------------------------------------------------- */
/**
 *  Suspend the Trackpad driver
 *  Usage:
 *  This function gets called when Android shuts down the
 *  screen to save power (either by sleep timer or power button).
 *  When this function gets called, implement ultra low power
 *  functions on the Trackpad Device (we do not need the TS enabled).
 */
static int shtrpd_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
    /*  Client only necessary for I2C   */
    /*  struct shtrpd_ts_data *data =
     *      i2c_get_clientdata(client); */

    /* Touch sleep mode */
    SHTRPD_DBG("Suspending Trackpad driver... ");

    /*  Do a Handshake with the Trackpad Device to initiate comms   */
    /*  shtrpd_event_mode_handshake(data);  */

    /*  Could implement a sleep function    */

    /*  Disable the IRQ for this Driver while suspende  */
    if(shtrpd_flip_state) {
        shtrpd_sys_disable_irq();
        shtrpd_sys_enable_irq_wake();
    }

#if defined( SHTRPD_CHECK_CPU_TEMP_ENABLE )
    SHTRPD_LOG_CHECK_CPU_TEMP("cpu temp check timer stop\n");
    cancel_delayed_work(&shtrpd_work_cpu_temp);
#endif /* SHTRPD_CHECK_CPU_TEMP_ENABLE */

    return 0;
}

/* ------------------------------------------------------------------------- */
/* shtrpd_ts_resume                                                          */
/* ------------------------------------------------------------------------- */
/**
 *  Resume Trackpad driver after a suspend
 *  Usage:
 *  This function gets called when Android powers up again -
 *  this is when the screen switches on (either by home button
 *  or by power button). This is the reverse of suspend function.
 *  Now, we set the Trackpad Device to process events again.
 */
static int shtrpd_ts_resume(struct i2c_client *client)
{
    /*  Client only necessary for I2C   */
    /*  struct shtrpd_ts_data *data =
     *      i2c_get_clientdata(client); */

    SHTRPD_DBG("Resuming Trackpad driver...");

    /*  Do a Handshake with the Trackpad device to initiate comms   */
    /*  shtrpd_event_mode_handshake(data);  */

    /*  Could implement a resume function   */

    /*  Enable the IRQ again    */
    if(shtrpd_flip_state) {
        shtrpd_sys_enable_irq();
#if defined( SHTRPD_CHECK_CPU_TEMP_ENABLE )
        if(SHTRPD_CHECK_CPU_TEMP){
	        SHTRPD_LOG_CHECK_CPU_TEMP("cpu temp check timer start\n");
            cancel_delayed_work(&shtrpd_work_cpu_temp);
            queue_delayed_work(shtrpd_work_queue, &shtrpd_work_cpu_temp, msecs_to_jiffies(0));
        }
#endif /* SHTRPD_CHECK_CPU_TEMP_ENABLE */
    }
    shtrpd_sys_disable_irq_wake();

    return 0;
}

/**
 *  Early Suspend and Late Resume is Android's Application of
 *  the kernel suspend and resume functions. Here we implement
 *  the early_suspend and late_resume functions - these functions
 *  calls the suspend and resume functions of the kernel
 */
#ifdef CONFIG_HAS_EARLYSUSPEND
/* ------------------------------------------------------------------------- */
/* shtrpd_ts_early_suspend                                                   */
/* ------------------------------------------------------------------------- */
static void shtrpd_ts_early_suspend(struct early_suspend *h)
{
    struct shtrpd_ts_data *data;
    data = container_of(h, struct shtrpd_ts_data, early_suspend);
    shtrpd_ts_suspend(data->client, PMSG_SUSPEND);
}

/* ------------------------------------------------------------------------- */
/* shtrpd_ts_late_resume                                                     */
/* ------------------------------------------------------------------------- */
static void shtrpd_ts_late_resume(struct early_suspend *h)
{
    struct shtrpd_ts_data *data;
    data = container_of(h, struct shtrpd_ts_data, early_suspend);
    shtrpd_ts_resume(data->client);
}
#endif

/* ------------------------------------------------------------------------- */
/* shtrpd_ts_id                                                              */
/* ------------------------------------------------------------------------- */
/*  Standard stucture with the device id for identification */
static const struct i2c_device_id shtrpd_ts_id[] = {
    { DEVICE_NAME, 0 },
    { }
};

/*  Register Device Table   */
MODULE_DEVICE_TABLE(i2c, shtrpd_ts_id);

/**
 *  Standard stucture containing the driver
 *  information and procedures
 */
static struct i2c_driver shtrpd_ts_driver = {
    .probe      = shtrpd_ts_probe,
    .remove     = __devexit_p(shtrpd_ts_remove),
//#ifndef CONFIG_HAS_EARLYSUSPEND
    .suspend    = shtrpd_ts_suspend,
    .resume     = shtrpd_ts_resume,
//#endif
    .driver     = { .name = DEVICE_NAME,
                    .owner = THIS_MODULE,},
    .id_table   = shtrpd_ts_id,
};

/* ------------------------------------------------------------------------- */
/* shtrpd_ts_init                                                            */
/* ------------------------------------------------------------------------- */
/*
 *  Gets called from 'board-omap3beagle.c' when the
 *  device I2C bus init is called - install Trackpad device TS Driver
 */
static int __init shtrpd_ts_init(void)
{
    SHTRPD_WARN("Installing Trackpad driver... \n");
    return i2c_add_driver(&shtrpd_ts_driver);
}

/* ------------------------------------------------------------------------- */
/* shtrpd_ts_exit                                                            */
/* ------------------------------------------------------------------------- */
/*  Remove the driver - Uninstall   */
static void __exit shtrpd_ts_exit(void)
{
    /*  Values used for the initial setup - ensure setup is done again
     *  if reinstalled again
     */
    initialSetup = true;
    setupCounter = 0;
    initialATI = false;
    initReseed = false;
    reset = 0;

    i2c_del_driver(&shtrpd_ts_driver);
    SHTRPD_WARN("Trackpad driver Deleted! \n");
}


module_init(shtrpd_ts_init);
module_exit(shtrpd_ts_exit);

/* -----------------------------------------------------------------------------------
 */
void msm_trpd_setsleep(int on)
{
#if 0
    mutex_lock(&shtrpd_ioctl_lock);

    SHTRPD_DBG("on = %d\n", on);

    if(on){
        gpio_set_value(NRST_LINE_PIN, 1);
        usleep(SHTRPD_NRST_DELAY);
        setupCounter = 0;
        initialSetup = true;
//        enable_irq(shtrpd_client->irq);
        shtrpd_sys_enable_irq();
    }else{
//        disable_irq_nosync(shtrpd_client->irq);
        shtrpd_sys_disable_irq();
        gpio_set_value(NRST_LINE_PIN, 0);
    }

    mutex_unlock(&shtrpd_ioctl_lock);
#endif

    return;
}
EXPORT_SYMBOL(msm_trpd_setsleep);

/* -----------------------------------------------------------------------------------
 */
void msm_trpd_set_flip_state(int state)
{
    SHTRPD_DBG("state = %d\n", state);

#ifdef SHTRPD_ENABLE_INIT_FW_DL
    if (shtrpd_fwdl_fail_flg) {
        SHTRPD_ERR("init fwdl failed\n");
        return;
    }
#endif /* SHTRPD_ENABLE_INIT_FW_DL */

    if (!shtrpd_work_queue) {
        SHTRPD_ERR("shtrpd_work_queue NULL failed\n");
        return;
    }

    mutex_lock(&shtrpd_flip_lock);

    if (state == shtrpd_flip_old_state) {
        mutex_unlock(&shtrpd_flip_lock);
        return;
    }

    shtrpd_flip_old_state = state;

    if (state) {
        cancel_work_sync(&shtrpd_work_flip_open);
        queue_work(shtrpd_work_queue, &shtrpd_work_flip_open);
    } else {
        cancel_work_sync(&shtrpd_work_flip_close);
        queue_work(shtrpd_work_queue, &shtrpd_work_flip_close);
    }

    mutex_unlock(&shtrpd_flip_lock);

    return;
}
EXPORT_SYMBOL(msm_trpd_set_flip_state);

/*  Module information  */
MODULE_DESCRIPTION("SHARP TRACKPAD DRIVER MODULE");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.00");
