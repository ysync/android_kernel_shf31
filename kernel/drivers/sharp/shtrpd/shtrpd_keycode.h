/* drivers/sharp/shtrpd/shtrpd_keycode.h  (Trackpad driver)
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

#include <linux/input.h>

#define TOTALTXS_MAX            15
#define TOTALRXS_MAX            10

#define SHTRPD_XY_SNAP_NUM              4

#define WK_UPRIGHT              ((KEY_UP   + KEY_RIGHT) | 0x1000)
#define WK_UPLEFT               ((KEY_UP   + KEY_LEFT ) | 0x1000)
#define WK_DNRIGHT              ((KEY_DOWN + KEY_RIGHT) | 0x2000)
#define WK_DNLEFT               ((KEY_DOWN + KEY_LEFT ) | 0x2000)

const short shtrpd_snap_to_keycode[SHTRPD_XY_SNAP_NUM][8]={
/* ES1 */
	{
	    /*  A0              A1              A2              A3
    	    A4              A5              A6              A7     */
	    KEY_S1,				WK_UPLEFT,		KEY_UP,			WK_UPRIGHT,
		KEY_S2,				KEY_LEFT,		KEY_ENTER,		KEY_RIGHT,
	},

	{
		/*  B0              B1              B2              B3
			B4              B5              B6              B7     */
		KEY_S3,				WK_DNLEFT,		KEY_DOWN,		WK_DNRIGHT,
		KEY_S4,				KEY_PHONE,		KEY_BACKSPACE,	KEY_1,
	},

	{
    	/*  C0              C1              C2              C3
        	C4              C5              C6              C7     */
		KEY_2,				KEY_3,			KEY_4,			KEY_5,
		KEY_6,				KEY_7,			KEY_8,			KEY_9
	},

	{
    	/*  D0              D1              D2              D3
        	D4              D5              D6              D7     */
		KEY_STAR,			KEY_0,			KEY_POUND,		KEY_S5,
		KEY_S6,				-1,				-1,				-1,
	},
};

#if 0
/* ------------------------------------------------------------------------- */
/* TABLE                                                                     */
/* ------------------------------------------------------------------------- */
/* The conversion table from Tx,Rx data to a key code */
const short shtrpd_txrx_to_keycode_db[TOTALTXS_MAX][TOTALRXS_MAX] = {
    /*  Rx9         Rx8         Rx7         Rx6         Rx5     
        Rx4         Rx3         Rx2         Rx1         Rx0     */
    {   -1,         -1,         -1,         -1,         -1, 
        KEY_UP,     -1,         -1,         -1,         -1,     },  /* Tx0  */

    {   -1,         KEY_S1,     -1,         -1,         WK_UPLEFT, 
        -1,         WK_UPRIGHT, -1,         -1,         KEY_S2, },  /* Tx1  */

    {   -1,         -1,         -1,         -1,         -1,     
        -1,         -1,         -1,         -1,         -1,     },  /* Tx2  */

    {   -1,         -1,         -1,         KEY_LEFT,   -1,
        KEY_ENTER,  -1,         KEY_RIGHT,  -1,         -1,     },  /* Tx3  */

    {   -1,         KEY_S3,     -1,         -1,         WK_DNLEFT,     
        -1,         WK_DNRIGHT, -1,         -1,         KEY_S4, },  /* Tx4  */

    {   -1,         -1,         -1,         -1,         -1,
        KEY_DOWN,   -1,         -1,         -1,         -1,     },  /* Tx5  */

    {   -1,         -1,         KEY_PHONE,  -1,         -1,     
        -1,         -1,         -1,         -1,         -1,     },  /* Tx6  */

    {   -1,         -1,         -1,         -1,         -1,
        KEY_BACKSPACE, -1,      -1,         -1,         -1,     },  /* Tx7  */

    {   -1,         -1,         KEY_1,      -1,         -1,     
        -1,         -1,         -1,         KEY_3,      -1,     },  /* Tx8  */

    {   -1,         -1,         -1,         -1,         -1,  
        KEY_2,      -1,         -1,         -1,         -1,     },  /* Tx9  */

    {   -1,         -1,         KEY_4,      -1,         -1,     
        -1,         -1,         -1,         KEY_6,      -1,     },  /* Tx10 */

    {   -1,         -1,         -1,         -1,         -1,  
        KEY_5,      -1,         -1,         -1,         -1,     },  /* Tx11 */

    {   -1,         -1,         KEY_7,      -1,         -1,     
        -1,         -1,         -1,         KEY_9,      -1,     },  /* Tx12 */

    {   -1,         -1,         -1,         -1,         -1,  
        KEY_8,      -1,         -1,         -1,         -1,     },  /* Tx13 */

    {   -1,         KEY_S5,     KEY_STAR,   KEY_S6,     -1,  
        KEY_0,      -1,         -1,         KEY_POUND,  -1,     }   /* Tx14 */
};

const short shtrpd_txrx_to_keycode_es0[TOTALTXS_MAX][TOTALRXS_MAX] = {
    /*  Rx9         Rx8         Rx7         Rx6         Rx5     
        Rx4         Rx3         Rx2         Rx1         Rx0     */
    {   -1,         -1,         -1,         -1,         -1, 
        -1,         -1,         -1,         -1,         -1,     },  /* Tx0  */

    {   -1,         KEY_S1,     -1,         WK_UPLEFT,  -1, 
        KEY_UP,     -1,         WK_UPRIGHT, -1,         KEY_S2, },  /* Tx1  */

    {   -1,         -1,         -1,         -1,         -1,     
        -1,         -1,         -1,         -1,         -1,     },  /* Tx2  */

    {   -1,         -1,         -1,         KEY_LEFT,   -1,
        KEY_ENTER,  -1,         KEY_RIGHT,  -1,         -1,     },  /* Tx3  */

    {   -1,         KEY_S3,     -1,         -1,         -1,     
        -1,         -1,         -1,         -1,         KEY_S4, },  /* Tx4  */

    {   -1,         -1,         -1,         WK_DNLEFT,  -1,
        KEY_DOWN,   -1,         WK_DNRIGHT, -1,         -1,     },  /* Tx5  */

    {   -1,         -1,         KEY_PHONE,  -1,         -1,     
        KEY_BACKSPACE, -1,      -1,         -1,         -1,     },  /* Tx6  */

    {   -1,         -1,         -1,         -1,         -1,
        -1,         -1,         -1,         -1,         -1,     },  /* Tx7  */

    {   -1,         -1,         KEY_1,      -1,         -1,     
        KEY_2,      -1,         -1,         KEY_3,      -1,     },  /* Tx8  */

    {   -1,         -1,         -1,         -1,         -1,  
        -1,         -1,         -1,         -1,         -1,     },  /* Tx9  */

    {   -1,         -1,         KEY_4,      -1,         -1,     
        KEY_5,      -1,         -1,         KEY_6,      -1,     },  /* Tx10 */

    {   -1,         -1,         -1,         -1,         -1,  
        -1,         -1,         -1,         -1,         -1,     },  /* Tx11 */

    {   -1,         -1,         KEY_7,      -1,         -1,     
        KEY_8,      -1,         -1,         KEY_9,      -1,     },  /* Tx12 */

    {   -1,         -1,         -1,         -1,         -1,  
        -1,         -1,         -1,         -1,         -1,     },  /* Tx13 */

    {   -1,         -1,         KEY_STAR,   KEY_S5,     -1,  
        KEY_0,      -1,         KEY_S6,     KEY_POUND,  -1,     }   /* Tx14 */
};

const short shtrpd_txrx_to_keycode_es1[TOTALTXS_MAX][TOTALRXS_MAX] = {
    /*  Rx9         Rx8         Rx7         Rx6         Rx5     
        Rx4         Rx3         Rx2         Rx1         Rx0     */
    {   -1,         -1,         -1,         -1,         -1, 
        -1,         -1,         -1,         -1,         -1,     },  /* Tx0  */

    {   -1,         KEY_S1,     -1,         WK_UPLEFT,  -1, 
        KEY_UP,     -1,         WK_UPRIGHT, -1,         KEY_S2, },  /* Tx1  */

    {   -1,         -1,         -1,         -1,         -1,     
        -1,         -1,         -1,         -1,         -1,     },  /* Tx2  */

    {   -1,         -1,         -1,         KEY_LEFT,   -1,
        KEY_ENTER,  -1,         KEY_RIGHT,  -1,         -1,     },  /* Tx3  */

    {   -1,         KEY_S3,     -1,         -1,         -1,     
        -1,         -1,         -1,         -1,         KEY_S4, },  /* Tx4  */

    {   -1,         -1,         -1,         WK_DNLEFT,  -1,
        KEY_DOWN,   -1,         WK_DNRIGHT, -1,         -1,     },  /* Tx5  */

    {   -1,         -1,         KEY_PHONE,  -1,         -1,     
        KEY_BACKSPACE, -1,      -1,         -1,         -1,     },  /* Tx6  */

    {   -1,         -1,         -1,         -1,         -1,
        -1,         -1,         -1,         -1,         -1,     },  /* Tx7  */

    {   -1,         -1,         KEY_1,      -1,         -1,     
        KEY_2,      -1,         -1,         KEY_3,      -1,     },  /* Tx8  */

    {   -1,         -1,         -1,         -1,         -1,  
        -1,         -1,         -1,         -1,         -1,     },  /* Tx9  */

    {   -1,         -1,         KEY_4,      -1,         -1,     
        KEY_5,      -1,         -1,         KEY_6,      -1,     },  /* Tx10 */

    {   -1,         -1,         -1,         -1,         -1,  
        -1,         -1,         -1,         -1,         -1,     },  /* Tx11 */

    {   -1,         -1,         KEY_7,      -1,         -1,     
        KEY_8,      -1,         -1,         KEY_9,      -1,     },  /* Tx12 */

    {   -1,         -1,         -1,         -1,         -1,  
        -1,         -1,         -1,         -1,         -1,     },  /* Tx13 */

    {   -1,         -1,         KEY_STAR,   KEY_S5,     -1,  
        KEY_0,      -1,         KEY_S6,     KEY_POUND,  -1,     }   /* Tx14 */
};

const short shtrpd_txrx_to_keycode_pp1[TOTALTXS_MAX][TOTALRXS_MAX] = {
    /*  Rx9         Rx8         Rx7         Rx6         Rx5     
        Rx4         Rx3         Rx2         Rx1         Rx0     */
    {   -1,         -1,         -1,         -1,         -1, 
        -1,         -1,         -1,         -1,         -1,     },  /* Tx0  */

    {   -1,         KEY_S1,     -1,         WK_UPLEFT,  -1, 
        KEY_UP,     -1,         WK_UPRIGHT, -1,         KEY_S2, },  /* Tx1  */

    {   -1,         -1,         -1,         -1,         -1,     
        -1,         -1,         -1,         -1,         -1,     },  /* Tx2  */

    {   -1,         -1,         -1,         KEY_LEFT,   -1,
        KEY_ENTER,  -1,         KEY_RIGHT,  -1,         -1,     },  /* Tx3  */

    {   -1,         KEY_S3,     -1,         -1,         -1,     
        -1,         -1,         -1,         -1,         KEY_S4, },  /* Tx4  */

    {   -1,         -1,         -1,         WK_DNLEFT,  -1,
        KEY_DOWN,   -1,         WK_DNRIGHT, -1,         -1,     },  /* Tx5  */

    {   -1,         -1,         KEY_PHONE,  -1,         -1,     
        KEY_BACKSPACE, -1,      -1,         -1,         -1,     },  /* Tx6  */

    {   -1,         -1,         -1,         -1,         -1,
        -1,         -1,         -1,         -1,         -1,     },  /* Tx7  */

    {   -1,         -1,         KEY_1,      -1,         -1,     
        KEY_2,      -1,         -1,         KEY_3,      -1,     },  /* Tx8  */

    {   -1,         -1,         -1,         -1,         -1,  
        -1,         -1,         -1,         -1,         -1,     },  /* Tx9  */

    {   -1,         -1,         KEY_4,      -1,         -1,     
        KEY_5,      -1,         -1,         KEY_6,      -1,     },  /* Tx10 */

    {   -1,         -1,         -1,         -1,         -1,  
        -1,         -1,         -1,         -1,         -1,     },  /* Tx11 */

    {   -1,         -1,         KEY_7,      -1,         -1,     
        KEY_8,      -1,         -1,         KEY_9,      -1,     },  /* Tx12 */

    {   -1,         -1,         -1,         -1,         -1,  
        -1,         -1,         -1,         -1,         -1,     },  /* Tx13 */

    {   -1,         KEY_S5,     KEY_STAR,   -1,         -1,  
        KEY_0,      -1,         -1,         KEY_POUND,  KEY_S6, }   /* Tx14 */
};

const short shtrpd_txrx_to_keycode_pp2[TOTALTXS_MAX][TOTALRXS_MAX] = {
    /*  Rx9         Rx8         Rx7         Rx6         Rx5     
        Rx4         Rx3         Rx2         Rx1         Rx0     */
    {   -1,         -1,         -1,         -1,         -1, 
        -1,         -1,         -1,         -1,         -1,     },  /* Tx0  */

    {   -1,         KEY_S1,     -1,         WK_UPLEFT,  -1, 
        KEY_UP,     -1,         WK_UPRIGHT, -1,         KEY_S2, },  /* Tx1  */

    {   -1,         -1,         -1,         -1,         -1,     
        -1,         -1,         -1,         -1,         -1,     },  /* Tx2  */

    {   -1,         -1,         -1,         KEY_LEFT,   -1,
        KEY_ENTER,  -1,         KEY_RIGHT,  -1,         -1,     },  /* Tx3  */

    {   -1,         KEY_S3,     -1,         -1,         -1,     
        -1,         -1,         -1,         -1,         KEY_S4, },  /* Tx4  */

    {   -1,         -1,         -1,         WK_DNLEFT,  -1,
        KEY_DOWN,   -1,         WK_DNRIGHT, -1,         -1,     },  /* Tx5  */

    {   -1,         -1,         KEY_PHONE,  -1,         -1,     
        KEY_BACKSPACE, -1,      -1,         -1,         -1,     },  /* Tx6  */

    {   -1,         -1,         -1,         -1,         -1,
        -1,         -1,         -1,         -1,         -1,     },  /* Tx7  */

    {   -1,         -1,         KEY_1,      -1,         -1,     
        KEY_2,      -1,         -1,         KEY_3,      -1,     },  /* Tx8  */

    {   -1,         -1,         -1,         -1,         -1,  
        -1,         -1,         -1,         -1,         -1,     },  /* Tx9  */

    {   -1,         -1,         KEY_4,      -1,         -1,     
        KEY_5,      -1,         -1,         KEY_6,      -1,     },  /* Tx10 */

    {   -1,         -1,         -1,         -1,         -1,  
        -1,         -1,         -1,         -1,         -1,     },  /* Tx11 */

    {   -1,         -1,         KEY_7,      -1,         -1,     
        KEY_8,      -1,         -1,         KEY_9,      -1,     },  /* Tx12 */

    {   -1,         -1,         -1,         -1,         -1,  
        -1,         -1,         -1,         -1,         -1,     },  /* Tx13 */

    {   -1,         KEY_S5,     KEY_STAR,   -1,         -1,  
        KEY_0,      -1,         -1,         KEY_POUND,  KEY_S6, }   /* Tx14 */
};
#endif
