/* drivers/sharp/shdisp/shdisp_system.h  (Display Driver)
 *
 * Copyright (C) 2013 SHARP CORPORATION
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

#ifndef SHDISP_SYSTEM_H
#define SHDISP_SYSTEM_H

/* ------------------------------------------------------------------------- */
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */

#include <sharp/shdisp_kerl.h>

/* ------------------------------------------------------------------------- */
/* DEBUG MACROS                                                              */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */

#define SHDISP_GPIO_CTL_LOW         0
#define SHDISP_GPIO_CTL_HIGH        1

#define SHDISP_BOOT_SW_ENABLE_DISPLAY       (1)
#define SHDISP_BOOT_SW_ENABLE_LED           (1)


#define SHDISP_GPIO_NUM_BL_RST_N    25

#define SHDISP_GPIO_NUM_LCD_EXT_CLK     34
#define SHDISP_GPIO_NUM_LCD_SCS_N   32
#define SHDISP_GPIO_NUM_LCD_RESET   12

#ifdef CONFIG_SHDISP_PANEL_SUBDISPLAY
#define SHDISP_GPIO_NUM_SUBDISPLAY_A0   17
#define SHDISP_SUBDISPLAY_SPI_CLK 4800000
#endif

#define SHDISP_GPIO_NUM_CLK_SEL     107

#define SHDISP_HW_REV_ES0       0x00            /* ES0   */
#define SHDISP_HW_REV_ES05      0x00            /* ES0.5 */
#define SHDISP_HW_REV_ES1       0x02            /* ES1   */
#define SHDISP_HW_REV_ES15      0x02            /* ES1.5 */
#define SHDISP_HW_REV_PP1       0x03            /* PP1   */
#define SHDISP_HW_REV_PP15      0x03            /* PP1.5 */
#define SHDISP_HW_REV_PP2       0x04            /* PP2   */
#define SHDISP_HW_REV_PP25      0x05            /* PP2.5 */
#define SHDISP_HW_REV_MP        0x07            /* MP    */

#define SHDISP_BDIC_I2C_DEVNAME     ("sharp,bdic_i2c")
#ifndef SHDISP_NOT_SUPPORT_PSALS
#define SHDISP_SENSOR_DEVNAME       ("sharp,sensor_i2c")
#endif

#define WAIT_1FRAME_US (16666)

/* ------------------------------------------------------------------------- */
/* TYPES                                                                     */
/* ------------------------------------------------------------------------- */
#define SHDISP_IRQ_MAX_KIND         4


enum {
    SHDISP_HOST_CTL_CMD_LCD_CLK_START,
    SHDISP_HOST_CTL_CMD_LCD_CLK_STOP,
    SHDISP_HOST_CTL_CMD_LCD_CLK_INIT,
    SHDISP_HOST_CTL_CMD_LCD_CLK_EXIT,
    NUM_SHDISP_HOST_CTL_CMD
};

enum {
    SHDISP_IRQ_DISABLE,
    SHDISP_IRQ_ENABLE,
    NUM_SHDISP_IRQ_CMD
};


/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */

int  shdisp_SYS_Host_clock_control(int cmd, unsigned long rate, struct platform_device *pdev);
void shdisp_SYS_delay_us(unsigned long usec);
void shdisp_SYS_Host_gpio_init(void);
void shdisp_SYS_Host_gpio_exit(void);
int  shdisp_SYS_Host_gpio_request(int num);
int  shdisp_SYS_Host_gpio_free(int num);
int  shdisp_SYS_set_Host_gpio(int num, int value);

void shdisp_SYS_set_irq_port(int irq_port, struct platform_device *pdev);
int  shdisp_SYS_request_irq(irqreturn_t (*irq_handler)( int , void * ) );
void shdisp_SYS_free_irq(void);
void shdisp_SYS_set_irq_init(void);
int  shdisp_SYS_set_irq( int enable );
void shdisp_sys_dbg_hw_check_start(void);
void shdisp_sys_dbg_hw_check_end(const char *func);

int  shdisp_SYS_bdic_i2c_init(void);
int  shdisp_SYS_bdic_i2c_exit(void);
int  shdisp_SYS_bdic_i2c_write(unsigned char addr, unsigned char data);
int  shdisp_SYS_bdic_i2c_mask_write(unsigned char addr, unsigned char data, unsigned char mask);
int  shdisp_SYS_bdic_i2c_multi_write(unsigned char addr, unsigned char *wval, unsigned char size);
int  shdisp_SYS_bdic_i2c_read(unsigned char addr, unsigned char *data);
int  shdisp_SYS_bdic_i2c_multi_read(unsigned char addr, unsigned char *data, int size);

#ifndef SHDISP_NOT_SUPPORT_PSALS
int  shdisp_SYS_sensor_i2c_init(void);
int  shdisp_SYS_sensor_i2c_exit(void);

int  shdisp_SYS_Host_i2c_send(unsigned char slaveaddr, unsigned char * sendval, unsigned char size);
int  shdisp_SYS_Host_i2c_recv(unsigned char slaveaddr, unsigned char * sendval, unsigned char sendsize,
                                   unsigned char * recvval, unsigned char recvsize);
#endif
#ifdef CONFIG_SHDISP_PANEL_SUBDISPLAY
int  shdisp_SYS_subdisplay_spi_init(void);
int  shdisp_SYS_subdisplay_spi_command_set(unsigned char reg,unsigned char data);
int  shdisp_SYS_subdisplay_spi_transfer_data(unsigned char *data,int datalen);
int  shdisp_SYS_subdisplay_spi_multi_command_set(unsigned char reg,unsigned char *data,int len);
void shdisp_SYS_subdisplay_spi_exit(void);
#endif


#endif /* SHDISP_SYSTEM_H */

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
