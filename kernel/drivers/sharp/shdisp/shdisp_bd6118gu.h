/* drivers/sharp/shdisp/shdisp_bd6118gu.h  (Display Driver)
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

#ifndef SHDISP_BD6118GU_H
#define SHDISP_BD6118GU_H

/* ------------------------------------------------------------------------- */
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */
#include <sharp/shdisp_kerl.h>

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */
#define SHDISP_OPT_CHANGE_WAIT_TIME                 (150)
#define SENSOR_REG_COMMAND1                         (0x00)
#define SENSOR_REG_D2_MSB                           (0x11)
#ifndef SHDISP_NOT_SUPPORT_PSALS
#define SHDISP_BDIC_SENSOR_SLAVE_ADDR               (0x39)
#define SHDISP_ALS_SENSOR_ADJUST_STATUS_COMPLETED   (0x90)
#endif
#define BDIC_REG_PSCONT                     (0x01)
#define BDIC_REG_GPOUT                      (0x03)
#define BDIC_REG_LEDDRVCNT                  (0x04)
#define BDIC_REG_MLEDCNT                    (0x05)
#define BDIC_REG_LEDCNTR                    (0x07)
#define BDIC_REG_LEDCNTG                    (0x08)
#define BDIC_REG_LEDCNTB                    (0x06)

#define SHDISP_BDIC_GPIO_COG_RESET                  (0)
#define SHDISP_BDIC_GPIO_SUBDISPLAY_RESET           (1)

#define SHDISP_BDIC_GPIO_LOW                        (0)
#define SHDISP_BDIC_GPIO_HIGH                       (1)

#define SHDISP_BDIC_GPIO_GPOD0                      (0)
#define SHDISP_BDIC_GPIO_GPOD1                      (1)
#define SHDISP_BDIC_GPIO_GPOD2                      (2)
#define SHDISP_BDIC_GPIO_GPOD3                      (3)
#define SHDISP_BDIC_GPIO_GPOD4                      (4)
#define SHDISP_BDIC_CHIPVER_0                       (0)
#define SHDISP_BDIC_CHIPVER_1                       (1)

#define SHDISP_BDIC_INT_GPIO                        (31)

#define SHDISP_BKL_TBL_MODE_NORMAL                  (0)
#define SHDISP_BKL_TBL_MODE_ECO                     (1)
#define SHDISP_BKL_TBL_MODE_EMERGENCY               (2)
#define SHDISP_BKL_TBL_MODE_CHARGE                  (3)
#define NUM_SHDISP_BKL_TBL_MODE                     (SHDISP_BKL_TBL_MODE_CHARGE + 1)

/* ------------------------------------------------------------------------- */
/* TYPES                                                                     */
/* ------------------------------------------------------------------------- */

enum {
    SHDISP_BDIC_REQ_NONE = 1,
    SHDISP_BDIC_REQ_ACTIVE,
    SHDISP_BDIC_REQ_STANDBY,
    SHDISP_BDIC_REQ_STOP,
    SHDISP_BDIC_REQ_START,
    SHDISP_BDIC_REQ_BKL_SET_MODE_OFF,
    SHDISP_BDIC_REQ_BKL_SET_MODE_FIX,
    SHDISP_BDIC_REQ_BKL_SET_MODE_AUTO,
    SHDISP_BDIC_REQ_TRI_LED_SET_MODE_NORMAL,
    SHDISP_BDIC_REQ_TRI_LED_SET_MODE_BLINK,
    SHDISP_BDIC_REQ_TRI_LED_SET_MODE_FIREFLY,
    SHDISP_BDIC_REQ_TRI_LED_SET_MODE_HISPEED,
    SHDISP_BDIC_REQ_TRI_LED_SET_MODE_STANDARD,
    SHDISP_BDIC_REQ_TRI_LED_SET_MODE_BREATH,
    SHDISP_BDIC_REQ_TRI_LED_SET_MODE_LONG_BREATH,
    SHDISP_BDIC_REQ_TRI_LED_SET_MODE_WAVE,
    SHDISP_BDIC_REQ_TRI_LED_SET_MODE_FLASH,
    SHDISP_BDIC_REQ_TRI_LED_SET_MODE_AURORA,
    SHDISP_BDIC_REQ_TRI_LED_SET_MODE_RAINBOW,
    SHDISP_BDIC_REQ_TRI_LED_SET_ONTIME,
    SHDISP_BDIC_REQ_TRI_LED_SET_INTERVAL,
    SHDISP_BDIC_REQ_TRI_LED_SET_COUNT,
#ifndef SHDISP_NOT_SUPPORT_PSALS
    SHDISP_BDIC_REQ_PHOTO_SENSOR_CONFIG,
#endif
    SHDISP_BDIC_REQ_BKL_DTV_OFF,
    SHDISP_BDIC_REQ_BKL_DTV_ON,
    SHDISP_BDIC_REQ_BKL_EMG_OFF,
    SHDISP_BDIC_REQ_BKL_EMG_ON,
    SHDISP_BDIC_REQ_BKL_ECO_OFF,
    SHDISP_BDIC_REQ_BKL_ECO_ON,
    SHDISP_BDIC_REQ_BKL_CHG_OFF,
    SHDISP_BDIC_REQ_BKL_CHG_ON,
    SHDISP_BDIC_REQ_BKL_ON,
    SHDISP_BDIC_REQ_BKL_FIX_START,
    SHDISP_BDIC_REQ_BKL_AUTO_START,
    SHDISP_BDIC_REQ_BKL_SET_LED_VALUE,
    SHDISP_BDIC_REQ_BKL_SET_OPT_VALUE
};

enum {
    SHDISP_BDIC_PWR_STATUS_OFF,
    SHDISP_BDIC_PWR_STATUS_STANDBY,
    SHDISP_BDIC_PWR_STATUS_ACTIVE,
    NUM_SHDISP_BDIC_PWR_STATUS
};

enum {
    SHDISP_BDIC_DEV_TYPE_LCD_BKL,
    SHDISP_BDIC_DEV_TYPE_LCD_PWR,
    SHDISP_BDIC_DEV_TYPE_TRI_LED,
    SHDISP_BDIC_DEV_TYPE_TRI_LED_ANIME,
#ifndef SHDISP_NOT_SUPPORT_PSALS
    SHDISP_BDIC_DEV_TYPE_PHOTO_SENSOR_APP,
    SHDISP_BDIC_DEV_TYPE_PHOTO_SENSOR_LUX,
    SHDISP_BDIC_DEV_TYPE_PHOTO_SENSOR_BKL,
    SHDISP_BDIC_DEV_TYPE_PROX_SENSOR,
#endif
    NUM_SHDISP_BDIC_DEV_TYPE
};

enum {
    SHDISP_BDIC_DEV_PWR_OFF,
    SHDISP_BDIC_DEV_PWR_ON,
    NUM_SHDISP_BDIC_DEV_PWR
};

enum {
    SHDISP_MAIN_BKL_DEV_TYPE_APP,
    SHDISP_MAIN_BKL_DEV_TYPE_APP_AUTO,
    NUM_SHDISP_MAIN_BKL_DEV_TYPE
};

enum {
    SHDISP_BDIC_IRQ_TYPE_NONE,
#ifndef SHDISP_NOT_SUPPORT_PSALS
    SHDISP_BDIC_IRQ_TYPE_ALS,
    SHDISP_BDIC_IRQ_TYPE_PS,
#endif
    SHDISP_BDIC_IRQ_TYPE_DET,
    SHDISP_BDIC_IRQ_TYPE_I2C_ERR,
    NUM_SHDISP_BDIC_IRQ_TYPE
};

enum {
    SHDISP_IRQ_MASK,
    SHDISP_IRQ_NO_MASK,
    NUM_SHDISP_IRQ_SWITCH
};

enum {
    SHDISP_MAIN_BKL_ADJ_RETRY1,
    SHDISP_MAIN_BKL_ADJ_RETRY2,
    SHDISP_MAIN_BKL_ADJ_RETRY3,
    NUM_SHDISP_MAIN_BKL_ADJ
};

enum {
    SHDISP_BDIC_MAIN_BKL_OPT_LOW,
    SHDISP_BDIC_MAIN_BKL_OPT_HIGH,
    NUM_SHDISP_BDIC_MAIN_BKL_OPT_MODE
};

#ifndef SHDISP_NOT_SUPPORT_PSALS
enum {
    SHDISP_BDIC_PHOTO_LUX_TIMER_ON,
    SHDISP_BDIC_PHOTO_LUX_TIMER_OFF,
    NUM_SHDISP_BDIC_PHOTO_LUX_TIMER_SWITCH
};

enum {
    SHDISP_BDIC_LUX_JUDGE_IN,
    SHDISP_BDIC_LUX_JUDGE_IN_CONTI,
    SHDISP_BDIC_LUX_JUDGE_OUT,
    SHDISP_BDIC_LUX_JUDGE_OUT_CONTI,
    SHDISP_BDIC_LUX_JUDGE_ERROR,
    NUM_SHDISP_BDIC_LUX_JUDGE
};
#endif

enum {
    SHDISP_BDIC_BL_PARAM_WRITE = 1,
    SHDISP_BDIC_BL_PARAM_READ,
    SHDISP_BDIC_BL_MODE_SET,
#ifndef SHDISP_NOT_SUPPORT_PSALS
    SHDISP_BDIC_ALS_SET,
    SHDISP_BDIC_ALS_PARAM_WRITE,
    SHDISP_BDIC_ALS_PARAM_READ,
    SHDISP_BDIC_ALS_PARAM_SET,
#endif
    SHDISP_BDIC_CABC_CTL,
    SHDISP_BDIC_CABC_CTL_TIME_SET,
    SHDISP_BDIC_DEVICE_SET
};

enum {
    SHDISP_BDIC_BL_PWM_FIX_PARAM = 1,
    SHDISP_BDIC_BL_PWM_AUTO_PARAM
};

enum {
    SHDISP_BDIC_STR,
    SHDISP_BDIC_SET,
    SHDISP_BDIC_CLR,
    SHDISP_BDIC_RMW,
    SHDISP_BDIC_STRM,
    SHDISP_BDIC_BANK,
    SHDISP_BDIC_WAIT,
#ifndef SHDISP_NOT_SUPPORT_PSALS
    SHDISP_ALS_STR,
    SHDISP_ALS_RMW,
    SHDISP_ALS_STRM,
    SHDISP_ALS_STRMS
#endif
};

#ifndef SHDISP_NOT_SUPPORT_PSALS
enum {
    SHDISP_BDIC_PSALS_RECOVERY_NONE = 1,
    SHDISP_BDIC_PSALS_RECOVERY_DURING,
    SHDISP_BDIC_PSALS_RECOVERY_RETRY_OVER
};
#endif

struct shdisp_bdic_state_str{
    int bdic_is_exist;
    int bdic_chipver;
    int bdic_main_bkl_opt_mode_output;
    int bdic_main_bkl_opt_mode_ado;
#ifndef SHDISP_NOT_SUPPORT_PSALS
    unsigned char shdisp_lux_change_level1;
    unsigned char shdisp_lux_change_level2;
#endif
    int bdic_clrvari_index;
#ifndef SHDISP_NOT_SUPPORT_PSALS
    struct shdisp_photo_sensor_adj photo_sensor_adj;
#endif
};

#ifndef SHDISP_NOT_SUPPORT_PSALS
struct shdisp_bdic_bkl_ado_tbl {
    unsigned long range_low;
    unsigned long range_high;
    unsigned long param_a;
    long param_b;
};
#endif

struct shdisp_bdic_led_color_index {
    unsigned char red;
    unsigned char green;
    unsigned char blue;
    unsigned char color;
};

typedef struct {
    unsigned char   addr;
    unsigned char   flg;
    unsigned char   data;
    unsigned char   mask;
    unsigned long   wait;
} shdisp_bdicRegSetting_t;

/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */
int  shdisp_bdic_API_boot_init( void ) ;
void shdisp_bdic_API_initialize(struct shdisp_bdic_state_str* state_str);
void shdisp_bdic_API_LCD_BKL_off(void) ;
void shdisp_bdic_API_LCD_BKL_fix_on(int param) ;
void shdisp_bdic_API_LCD_BKL_set_request(int type, struct shdisp_main_bkl_ctl *tmp);
void shdisp_bdic_API_LCD_BKL_get_request(int type, struct shdisp_main_bkl_ctl *tmp, struct shdisp_main_bkl_ctl *req) ;
int  shdisp_bdic_API_set_active(int power_status) ;
int  shdisp_bdic_API_set_standby(void) ;
void shdisp_bdic_API_get_bdic_chipver(int* chipver) ;
void shdisp_bdic_API_LCD_release_hw_reset(void) ;
void shdisp_bdic_API_LCD_set_hw_reset(void) ;
void shdisp_bdic_API_LCD_release_deep_standby(void) ;
void shdisp_bdic_API_LCD_power_on(void) ;
void shdisp_bdic_API_LCD_power_off(void) ;
void shdisp_bdic_API_LCD_m_power_on(void) ;
void shdisp_bdic_API_LCD_m_power_off(void) ;
void shdisp_bdic_API_LCD_BKL_auto_on(int param) ;
void shdisp_bdic_API_LCD_BKL_get_param(unsigned long int* param) ;
void shdisp_bdic_API_TRI_LED_set_request(struct shdisp_tri_led *tmp) ;
void shdisp_bdic_API_LCD_BKL_dtv_on(void) ;
void shdisp_bdic_API_LCD_BKL_dtv_off(void) ;
void shdisp_bdic_API_LCD_BKL_emg_on(void) ;
void shdisp_bdic_API_LCD_BKL_emg_off(void) ;
void shdisp_bdic_API_LCD_BKL_eco_on(void) ;
void shdisp_bdic_API_LCD_BKL_eco_off(void) ;
void shdisp_bdic_API_LCD_BKL_chg_on(void) ;
void shdisp_bdic_API_LCD_BKL_chg_off(void) ;

int  shdisp_bdic_API_TRI_LED_off(void);
unsigned char shdisp_bdic_API_TRI_LED_get_color_index_and_reedit(struct shdisp_tri_led *tri_led ) ;
int  shdisp_bdic_API_TRI_LED_normal_on(unsigned char color) ;
void shdisp_bdic_API_TRI_LED_blink_on(unsigned char color, int ontime, int interval, int count) ;
void shdisp_bdic_API_TRI_LED_firefly_on(unsigned char color, int ontime, int interval, int count) ;
int  shdisp_bdic_API_TRI_LED_get_clrvari_index( int clrvari ) ;
#ifndef SHDISP_NOT_SUPPORT_PSALS
int  shdisp_bdic_API_PHOTO_SENSOR_get_lux(unsigned short *value, unsigned long *lux) ;
int  shdisp_bdic_API_PHOTO_SENSOR_get_raw_als(unsigned short *clear, unsigned short *ir) ;
int  shdisp_bdic_API_PHOTO_SENSOR_lux_change_ind(int *mode) ;
#endif
int shdisp_bdic_API_i2c_transfer(struct shdisp_bdic_i2c_msg *msg) ;
unsigned char shdisp_bdic_API_I2C_start_judge(void) ;
void shdisp_bdic_API_I2C_start_ctl(int flg) ;

int  shdisp_bdic_API_DIAG_write_reg(unsigned char reg, unsigned char val) ;
int  shdisp_bdic_API_DIAG_read_reg(unsigned char reg, unsigned char *val) ;
int  shdisp_bdic_API_DIAG_multi_read_reg(unsigned char reg, unsigned char *val, int size) ;
int  shdisp_bdic_API_RECOVERY_check_restoration(void) ;
int  shdisp_bdic_API_RECOVERY_check_bdic_practical(void) ;
#if defined (CONFIG_ANDROID_ENGINEERING)
void shdisp_bdic_API_DBG_INFO_output(void) ;
void shdisp_bdic_API_OPT_INFO_output(void) ;
void shdisp_bdic_API_TRI_LED_INFO_output(void) ;
#ifndef SHDISP_NOT_SUPPORT_PSALS
void shdisp_psals_API_DBG_INFO_output(void) ;
#endif
#endif /* CONFIG_ANDROID_ENGINEERING */

int  shdisp_bdic_API_IRQ_check_type( int irq_type ) ;
void shdisp_bdic_API_IRQ_save_fac(void) ;
int  shdisp_bdic_API_IRQ_check_fac(void) ;
int  shdisp_bdic_API_IRQ_get_fac( int iQueFac ) ;
void shdisp_bdic_API_IRQ_Clear(void) ;
void shdisp_bdic_API_IRQ_i2c_error_Clear(void) ;
void shdisp_bdic_API_IRQ_det_fac_Clear(void) ;
int  shdisp_bdic_API_IRQ_check_DET(void) ;
int  shdisp_bdic_API_IRQ_check_I2C_ERR(void) ;
void shdisp_bdic_API_IRQ_dbg_Clear_All(void) ;
void shdisp_bdic_API_IRQ_dbg_set_fac(unsigned int nGFAC) ;
#ifndef SHDISP_NOT_SUPPORT_PSALS
void shdisp_bdic_API_IRQ_dbg_photo_param( int level1, int level2) ;
int  shdisp_bdic_API_als_sensor_pow_ctl(int dev_type, int power_mode) ;
int  shdisp_bdic_API_psals_power_on(void) ;
int  shdisp_bdic_API_psals_power_off(void) ;
int  shdisp_bdic_API_psals_ps_init_als_off(void) ;
int  shdisp_bdic_API_psals_ps_init_als_on(void) ;
int  shdisp_bdic_API_psals_ps_deinit_als_off(void) ;
int  shdisp_bdic_API_psals_ps_deinit_als_on(void) ;
int  shdisp_bdic_API_psals_als_init_ps_off(void) ;
int  shdisp_bdic_API_psals_als_init_ps_on(void) ;
int  shdisp_bdic_API_psals_als_deinit_ps_off(void) ;
int  shdisp_bdic_API_psals_als_deinit_ps_on(void) ;
int  shdisp_bdic_API_psals_is_recovery_successful(void) ;
#endif
void shdisp_bdic_API_IRQ_det_irq_ctrl(int ctrl) ;
void shdisp_bdic_API_set_bkl_mode(unsigned char bkl_mode, unsigned char data, unsigned char msk) ;
void shdisp_bdic_API_set_device_code(void) ;
#ifndef SHDISP_NOT_SUPPORT_PSALS
void shdisp_bdic_API_set_prox_sensor_param( struct shdisp_prox_params *prox_params) ;
int  shdisp_bdic_API_get_lux_data(void) ;
void shdisp_bdic_API_set_lux_mode(unsigned char lux_mode, unsigned char data, unsigned char msk) ;
void shdisp_bdic_API_set_lux_mode_modify(unsigned char data, unsigned char msk) ;
int  shdisp_bdic_API_get_sensor_state(void) ;
void shdisp_bdic_API_RECOVERY_lux_data_backup(void) ;
void shdisp_bdic_API_RECOVERY_lux_data_restore(void) ;
void shdisp_bdic_API_psals_active(unsigned long dev_type) ;
void shdisp_bdic_API_psals_standby(unsigned long dev_type) ;
void shdisp_bdic_API_ps_background(unsigned long state) ;
void shdisp_bdic_API_check_sensor_param(struct shdisp_photo_sensor_adj *adj_in, struct shdisp_photo_sensor_adj *adj_out) ;
void shdisp_bdic_API_als_sensor_adjust(struct shdisp_photo_sensor_adj *adj) ;
int  shdisp_bdic_API_get_ave_ado(struct shdisp_ave_ado *ave_ado) ;
#endif
void shdisp_bdic_API_subdisplay_power_on(void);
void shdisp_bdic_API_subdisplay_power_off(void);
void shdisp_bdic_API_subdisplay_release_hw_reset(void);
void shdisp_bdic_API_subdisplay_set_hw_reset(void) ;
#endif
/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
