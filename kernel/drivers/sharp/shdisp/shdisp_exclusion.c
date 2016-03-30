/* driver/sharp/shdisp/shdisp_exclusion.c  (Display Driver)
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


#include "shdisp_bd6118gu.h"
#include "shdisp_subdisplay.h"
#include "shdisp_exclusion.h"
#include <sharp/shdisp_kerl.h>
#include "shdisp_dbg.h"
#include "shdisp_kerl_priv.h"
#include "shdisp_type.h"



#define SHDISP_DEVICE_STATE_ALL_OFF     (0)
#define SHDISP_DEVICE_STATE_BKL_ON      (1)
#define SHDISP_DEVICE_STATE_SUBDISP_ON  (2)

#define SHDISP_EXC_REQUEST_BKL_ON       (0x01)
#define SHDISP_EXC_REQUEST_SUBDISP_ON       (0x10)
#define SHDISP_EXC_REQUEST_BKL_SUBDISP_ON   (0x11)
#define SHDISP_EXC_REQUEST_BKL_SUBDISP_OFF  (0x00)

static int shdisp_device_state = 0x00;
static int shdisp_exc_request_state = 0x00;
static int shdisp_param = 0;
extern struct shdisp_kernel_context shdisp_kerl_ctx;

int shdisp_exc_LCD_bkl_on(int param)
{

    SHDISP_TRACE("in\n");

    if(shdisp_device_state == SHDISP_DEVICE_STATE_BKL_ON) {
        SHDISP_TRACE("out1:DS = %d,RS = %d\n",shdisp_device_state,shdisp_exc_request_state);
        return SHDISP_RESULT_SUCCESS;
    }
    if(shdisp_device_state == SHDISP_DEVICE_STATE_ALL_OFF && shdisp_exc_request_state == SHDISP_EXC_REQUEST_BKL_SUBDISP_OFF) {
        shdisp_exc_request_state = SHDISP_EXC_REQUEST_BKL_ON;
    }
    if(shdisp_device_state == SHDISP_DEVICE_STATE_SUBDISP_ON && shdisp_exc_request_state == SHDISP_EXC_REQUEST_SUBDISP_ON) {
        shdisp_exc_request_state = SHDISP_EXC_REQUEST_BKL_SUBDISP_ON;
    }
    if(shdisp_device_state == SHDISP_DEVICE_STATE_SUBDISP_ON && shdisp_exc_request_state == SHDISP_EXC_REQUEST_BKL_SUBDISP_ON) {
        shdisp_exc_request_state = SHDISP_EXC_REQUEST_BKL_SUBDISP_ON;
    }

    switch(shdisp_exc_request_state) {
    case SHDISP_EXC_REQUEST_BKL_ON:
        shdisp_bdic_API_LCD_BKL_fix_on(param);
        break;
    case SHDISP_EXC_REQUEST_BKL_SUBDISP_ON:
        shdisp_subdisplay_API_disp_off();
        shdisp_subdisplay_API_power_off();
        shdisp_bdic_API_LCD_BKL_fix_on(param);
        break;
    default:
        SHDISP_ERR("%s: error state\n",__func__);
        return SHDISP_RESULT_FAILURE;
    }

    shdisp_param = param;
    shdisp_device_state = SHDISP_DEVICE_STATE_BKL_ON;
    SHDISP_TRACE("out2:DS = %d,RS = %d\n",shdisp_device_state,shdisp_exc_request_state);
    return SHDISP_RESULT_SUCCESS;
}


int shdisp_exc_LCD_bkl_off(void)
{

    SHDISP_TRACE("in\n");

    if(shdisp_device_state != SHDISP_DEVICE_STATE_BKL_ON && shdisp_exc_request_state != SHDISP_EXC_REQUEST_BKL_SUBDISP_ON) {
        SHDISP_TRACE("out1:DS = %d,RS = %d\n",shdisp_device_state,shdisp_exc_request_state);
        return SHDISP_RESULT_SUCCESS;
    }
    if(shdisp_device_state == SHDISP_DEVICE_STATE_BKL_ON && shdisp_exc_request_state == SHDISP_EXC_REQUEST_BKL_ON) {
        shdisp_exc_request_state = SHDISP_EXC_REQUEST_BKL_SUBDISP_OFF;
    }
    if(shdisp_device_state == SHDISP_DEVICE_STATE_BKL_ON && shdisp_exc_request_state == SHDISP_EXC_REQUEST_BKL_SUBDISP_ON) {
        shdisp_exc_request_state = SHDISP_EXC_REQUEST_SUBDISP_ON;
    }
    if(shdisp_device_state == SHDISP_DEVICE_STATE_SUBDISP_ON && shdisp_exc_request_state == SHDISP_EXC_REQUEST_BKL_SUBDISP_ON) {
        shdisp_exc_request_state = SHDISP_EXC_REQUEST_SUBDISP_ON;
        SHDISP_TRACE("out2:DS = %d,RS = %d\n",shdisp_device_state,shdisp_exc_request_state);
        return SHDISP_RESULT_SUCCESS;
    }

    switch(shdisp_exc_request_state) {
    case SHDISP_EXC_REQUEST_BKL_SUBDISP_OFF:
        shdisp_bdic_API_LCD_BKL_off();
        shdisp_device_state = SHDISP_DEVICE_STATE_ALL_OFF;
        break;
    case SHDISP_EXC_REQUEST_SUBDISP_ON:
        shdisp_bdic_API_LCD_BKL_off();
        shdisp_subdisplay_API_power_on();
        shdisp_subdisplay_API_disp_on();
        shdisp_device_state = SHDISP_DEVICE_STATE_SUBDISP_ON;
        break;
    default:
        SHDISP_ERR("%s: error state\n",__func__);
        return SHDISP_RESULT_FAILURE;
    }

    SHDISP_TRACE("out3:DS = %d,RS = %d\n",shdisp_device_state,shdisp_exc_request_state);
    return SHDISP_RESULT_SUCCESS;
}


int shdisp_exc_subdisplay_on(void)
{
    SHDISP_TRACE("in\n");

    if(shdisp_device_state == SHDISP_DEVICE_STATE_SUBDISP_ON) {
        SHDISP_TRACE("out1:DS = %d,RS = %d\n",shdisp_device_state,shdisp_exc_request_state);
        return SHDISP_RESULT_SUCCESS;
    }
    if(shdisp_device_state == SHDISP_DEVICE_STATE_ALL_OFF && shdisp_exc_request_state == SHDISP_EXC_REQUEST_BKL_SUBDISP_OFF) {
        shdisp_exc_request_state = SHDISP_EXC_REQUEST_SUBDISP_ON;
    }
    if(shdisp_device_state == SHDISP_DEVICE_STATE_BKL_ON && shdisp_exc_request_state == SHDISP_EXC_REQUEST_BKL_ON) {
        shdisp_exc_request_state = SHDISP_EXC_REQUEST_BKL_SUBDISP_ON;
    }
    if(shdisp_device_state == SHDISP_DEVICE_STATE_BKL_ON && shdisp_exc_request_state == SHDISP_EXC_REQUEST_BKL_SUBDISP_ON) {
        shdisp_exc_request_state = SHDISP_EXC_REQUEST_BKL_SUBDISP_ON;
    }

    switch(shdisp_exc_request_state) {
    case SHDISP_EXC_REQUEST_SUBDISP_ON:
        shdisp_subdisplay_API_power_on();
        shdisp_subdisplay_API_disp_on();
        break;
    case SHDISP_EXC_REQUEST_BKL_SUBDISP_ON:
        shdisp_bdic_API_LCD_BKL_off();
        shdisp_kerl_ctx.main_bkl.mode  = SHDISP_MAIN_BKL_MODE_OFF;
        shdisp_subdisplay_API_power_on();
        shdisp_subdisplay_API_disp_on();
        break;
    default:
        SHDISP_ERR("%s: error state\n",__func__);
        return SHDISP_RESULT_FAILURE;
    }

    shdisp_device_state = SHDISP_DEVICE_STATE_SUBDISP_ON;
    SHDISP_TRACE("out2:DS = %d,RS = %d\n",shdisp_device_state,shdisp_exc_request_state);
    return SHDISP_RESULT_SUCCESS;
}

int shdisp_exc_subdisplay_off(void)
{
    SHDISP_TRACE("in\n");

    if(shdisp_device_state != SHDISP_DEVICE_STATE_SUBDISP_ON && shdisp_exc_request_state != SHDISP_EXC_REQUEST_BKL_SUBDISP_ON) {
        SHDISP_TRACE("out1:DS = %d,RS = %d\n",shdisp_device_state,shdisp_exc_request_state);
        return SHDISP_RESULT_SUCCESS;
    }
    if(shdisp_device_state == SHDISP_DEVICE_STATE_SUBDISP_ON && shdisp_exc_request_state == SHDISP_EXC_REQUEST_SUBDISP_ON) {
        shdisp_exc_request_state = SHDISP_EXC_REQUEST_BKL_SUBDISP_OFF;
    }
    if(shdisp_device_state == SHDISP_DEVICE_STATE_SUBDISP_ON && shdisp_exc_request_state == SHDISP_EXC_REQUEST_BKL_SUBDISP_ON) {
        shdisp_exc_request_state = SHDISP_EXC_REQUEST_BKL_ON;
    }
    if(shdisp_device_state == SHDISP_DEVICE_STATE_BKL_ON && shdisp_exc_request_state == SHDISP_EXC_REQUEST_BKL_SUBDISP_ON) {
        shdisp_exc_request_state = SHDISP_EXC_REQUEST_BKL_ON;
        SHDISP_TRACE("out2:DS = %d,RS = %d\n",shdisp_device_state,shdisp_exc_request_state);
        return SHDISP_RESULT_SUCCESS;
    }

    switch(shdisp_exc_request_state) {
    case SHDISP_EXC_REQUEST_BKL_SUBDISP_OFF:
        shdisp_subdisplay_API_disp_off();
        shdisp_subdisplay_API_power_off();
        shdisp_device_state = SHDISP_DEVICE_STATE_ALL_OFF;
        break;
    case SHDISP_EXC_REQUEST_BKL_ON:
        shdisp_subdisplay_API_disp_off();
        shdisp_subdisplay_API_power_off();
        shdisp_bdic_API_LCD_BKL_fix_on(shdisp_param);
        shdisp_kerl_ctx.main_bkl.mode  = SHDISP_MAIN_BKL_MODE_FIX;
        shdisp_device_state = SHDISP_DEVICE_STATE_BKL_ON;
        break;
    default:
        SHDISP_ERR("%s: error state\n",__func__);
        return SHDISP_RESULT_FAILURE;
    }

    SHDISP_TRACE("out3:DS = %d,RS = %d\n",shdisp_device_state,shdisp_exc_request_state);
    return SHDISP_RESULT_SUCCESS;
}

int shdisp_exc_state_init(void)
{
    SHDISP_TRACE("in\n");

    if(shdisp_api_get_boot_disp_status() == SHDISP_MAIN_DISP_ON) {
        shdisp_device_state = SHDISP_DEVICE_STATE_BKL_ON;
        shdisp_exc_request_state = SHDISP_EXC_REQUEST_BKL_ON;
    }
    else {
        shdisp_device_state = SHDISP_DEVICE_STATE_ALL_OFF;
        shdisp_exc_request_state = SHDISP_EXC_REQUEST_BKL_SUBDISP_OFF;
    }

    SHDISP_TRACE("out:DS = %d,RS = %d\n",shdisp_device_state,shdisp_exc_request_state);
    return SHDISP_RESULT_SUCCESS;
}
