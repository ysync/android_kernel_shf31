/* drivers/sharp/shdisp/shdisp_subdisplay.h  (Display Driver)
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

#ifndef SHDISP_SUBDISPLAY_H
#define SHDISP_SUBDISPLAY_H

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */

#define    SOFTRES          (0x01)
#define    DDIPON_OFF       (0x02)
#define    DSTBYON          (0x14)
#define    WRITEDIRECTION   (0x1D)
#define    DISPDIRECTION    (0x09)
#define    DISPSIZEX        (0x30)
#define    DISPSIZEY        (0x32)
#define    XDISPSTART       (0x38)
#define    YDISPSTART       (0x39)
#define    XBOXADRSSTART    (0x34)
#define    XBOXADRSEND      (0x35)
#define    YBOXADRSSTART    (0x36)
#define    YBOXADRSEND      (0x37)
#define    READOPERATION    (0x07)
#define    DATARW           (0x08)
#define    READREG          (0x20)
#define    S_STEPTIMER      (0xC3)
#define    S_STEPUNIT       (0xC4)
#define    S_CONDITION      (0xCC)
#define    S_START_STOP     (0xCD)
#define    SCLK             (0xD0)
#define    DDEN             (0xD2)
#define    FDIM             (0xDB)
#define    RESERVED1        (0x10)
#define    RESERVED2        (0x12)
#define    RESERVED3        (0x13)
#define    RESERVED4        (0x16)
#define    RESERVED5        (0x17)
#define    RESERVED6        (0x18)
#define    RESERVED7        (0x1A)
#define    RESERVED8        (0x1C)
#define    RESERVED9        (0x48)
#define    RESERVED10       (0xD9)
#define    RESERVED11       (0xDD)


#define SHDISP_SUBDISPLAY_WIDTH_BUFFER_SIZE  16
#define SHDISP_SUBDISPLAY_HEIGHT_BUFFER_SIZE 24
/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */
void shdisp_subdisplay_API_init(void);
int shdisp_subdisplay_API_power_on(void);
int shdisp_subdisplay_API_power_off(void);
int shdisp_subdisplay_API_disp_on(void);
int shdisp_subdisplay_API_disp_off(void);
int shdisp_subdisplay_API_update_image(unsigned char *data);

#endif /* SHDISP_SUBDISPLAY_H */

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */

