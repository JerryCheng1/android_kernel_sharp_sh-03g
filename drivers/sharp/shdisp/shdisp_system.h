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
#include <linux/platform_device.h>
#include <linux/irq.h>

/* ------------------------------------------------------------------------- */
/* DEBUG MACROS                                                              */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */
#define SHDISP_GPIO_CTL_LOW                 (0)
#define SHDISP_GPIO_CTL_HIGH                (1)



#define SHDISP_GPIO_NUM_BL_RST_N            (28)
#define SHDISP_GPIO_NUM_PANEL_RST_N         (78)
#define SHDISP_GPIO_NUM_CLK_SEL             (107)
#define SHDISP_GPIO_NUM_PANEL_VDD           (107)

#define SHDISP_GPIO_NUM_MIPI_ERROR          (34)
#if defined(CONFIG_SHDISP_PANEL_ANDY)
#define SHDISP_GPIO_NUM_UPPER_UNIT          ("gp-63")
#else  /* CONFIG_SHDISP_PANEL_ANDY */
#define SHDISP_GPIO_NUM_UPPER_UNIT          ("gp-69")
#endif /* CONFIG_SHDISP_PANEL_ANDY */
#define SHDISP_GPIO_PIN_UPPER_UNIT          ("fd510000.pinctrl")

enum {
    SHDISP_HW_REV_ES0,
    SHDISP_HW_REV_ES1,
    SHDISP_HW_REV_ES15,
    SHDISP_HW_REV_PP1,
    SHDISP_HW_REV_PP15,
    SHDISP_HW_REV_PP2,
    SHDISP_HW_REV_PP25,
    SHDISP_HW_REV_MP,
};

#define SHDISP_HW_REV_BIT(bit3, bit2, bit1) (((bit3) << 2) | ((bit2) << 1) | (bit1))

#define SHDISP_HW_REV_BIT_ES0               SHDISP_HW_REV_BIT(0, 0, 0)
#define SHDISP_HW_REV_BIT_ES1               SHDISP_HW_REV_BIT(0, 0, 1)
#define SHDISP_HW_REV_BIT_ES15              SHDISP_HW_REV_BIT(0, 1, 0)
#define SHDISP_HW_REV_BIT_PP1               SHDISP_HW_REV_BIT(0, 1, 1)
#define SHDISP_HW_REV_BIT_PP15              SHDISP_HW_REV_BIT(1, 0, 0)
#define SHDISP_HW_REV_BIT_PP2               SHDISP_HW_REV_BIT(1, 0, 1)
#define SHDISP_HW_REV_BIT_PP25              SHDISP_HW_REV_BIT(1, 1, 0)
#define SHDISP_HW_REV_BIT_MP                SHDISP_HW_REV_BIT(1, 1, 1)

#define SHDISP_HW_REV_TABLE_SIZE            (8)
#define SHDISP_HW_REV_TABLE_ITEMS                     \
        {SHDISP_HW_REV_BIT_ES0,  SHDISP_HW_REV_ES0},  \
        {SHDISP_HW_REV_BIT_ES1,  SHDISP_HW_REV_ES1},  \
        {SHDISP_HW_REV_BIT_ES15, SHDISP_HW_REV_ES15}, \
        {SHDISP_HW_REV_BIT_PP1,  SHDISP_HW_REV_PP1},  \
        {SHDISP_HW_REV_BIT_PP15, SHDISP_HW_REV_PP15}, \
        {SHDISP_HW_REV_BIT_PP2,  SHDISP_HW_REV_PP2},  \
        {SHDISP_HW_REV_BIT_PP25, SHDISP_HW_REV_PP25}, \
        {SHDISP_HW_REV_BIT_MP,   SHDISP_HW_REV_MP},   \

#define SHDISP_BDIC_I2C_DEVNAME             ("sharp,bdic_i2c")
#define SHDISP_SENSOR_DEVNAME               ("sharp,sensor_i2c")

#define WAIT_1FRAME_US                      (16666)

#define SHDISP_DEBUGFLG_BIT_KERNEL_LOG      (0x01)
#define SHDISP_DEBUGFLG_BIT_USER_LOG        (0x02)
#define SHDISP_DEBUGFLG_BIT_APPSBL_LOG      (0x04)
#define SHDISP_DEBUGFLG_BIT_SBL1_LOG        (0x08)
#define SHDISP_DEBUGFLG_BIT_MDP_DUMP        (0x10)

/* ------------------------------------------------------------------------- */
/* TYPES                                                                     */
/* ------------------------------------------------------------------------- */
#define SHDISP_IRQ_MAX_KIND                 (4)

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
int  shdisp_SYS_API_Host_control(int cmd, unsigned long rate);
void shdisp_SYS_API_delay_us(unsigned long usec);
void shdisp_SYS_API_msleep(unsigned int msec);
void shdisp_SYS_API_usleep(unsigned int usec);

void shdisp_SYS_API_Host_gpio_init(void);
void shdisp_SYS_API_Host_gpio_exit(void);
int  shdisp_SYS_API_Host_gpio_request(int num, char *label);
int  shdisp_SYS_API_Host_gpio_free(int num);
int  shdisp_SYS_API_set_Host_gpio(int num, int value);
int  shdisp_SYS_API_get_Host_gpio(int num);
int  shdisp_SYS_API_check_upper_unit(int gpio_no);

void shdisp_SYS_API_set_irq_port(int irq_port, struct platform_device *pdev);
int  shdisp_SYS_API_request_irq(irqreturn_t (*irq_handler)(int , void *));
void shdisp_SYS_API_free_irq(void);
void shdisp_SYS_API_set_irq_init(void);
int  shdisp_SYS_API_set_irq(int enable);

int  shdisp_SYS_API_bdic_i2c_init(void);
int  shdisp_SYS_API_bdic_i2c_exit(void);
int  shdisp_SYS_API_bdic_i2c_write(unsigned char addr, unsigned char data);
int  shdisp_SYS_API_bdic_i2c_mask_write(unsigned char addr, unsigned char data, unsigned char mask);
int  shdisp_SYS_API_bdic_i2c_multi_write(unsigned char addr, unsigned char *wval, unsigned char size);
int  shdisp_SYS_API_bdic_i2c_read(unsigned char addr, unsigned char *data);
int  shdisp_SYS_API_bdic_i2c_multi_read(unsigned char addr, unsigned char *data, int size);
int  shdisp_SYS_API_bdic_i2c_dummy_read(unsigned char addr, unsigned char *data);

int  shdisp_SYS_API_sensor_i2c_init(void);
int  shdisp_SYS_API_sensor_i2c_exit(void);

int  shdisp_SYS_API_panel_external_clk_ctl(int enable);

int  shdisp_SYS_API_Host_i2c_send(unsigned char slaveaddr, unsigned char *sendval, unsigned char size);
int  shdisp_SYS_API_Host_i2c_recv(unsigned char slaveaddr, unsigned char *sendval, unsigned char sendsize,
                                   unsigned char *recvval, unsigned char recvsize);

int shdisp_SYS_API_get_hayabusa_rev(int hw_handset, int hw_revision);
unsigned char shdisp_SYS_API_get_debugflg(void);


int shdisp_SYS_API_check_diag_mode(void);

#endif /* SHDISP_SYSTEM_H */

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
