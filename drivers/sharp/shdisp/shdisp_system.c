/* drivers/sharp/shdisp/shdisp_system.c  (Display Driver)
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
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/hrtimer.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <asm/io.h>
#include <linux/clk.h>
#include <linux/qpnp/qpnp-adc.h>
#include <sharp/shdisp_kerl.h>
#include <sharp/sh_smem.h>
#include "shdisp_system.h"
#include "shdisp_dbg.h"
#include <sharp/sh_boot_manager.h>
#define  SHDISP_BOOT_D      SH_BOOT_D
#define  SHDISP_BOOT_F_F    SH_BOOT_F_F


/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */
#define SHDISP_INT_FLAGS (IRQF_TRIGGER_LOW | IRQF_DISABLED)
#define SHDISP_I2C_SPEED_KHZ (400)

/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */
static int shdisp_host_gpio_request(int num, char *label);
static int shdisp_host_gpio_free(int num);

static int  shdisp_bdic_i2c_probe(struct i2c_client *client, const struct i2c_device_id * devid);
static int  shdisp_bdic_i2c_remove(struct i2c_client *client);

static int  shdisp_sensor_i2c_probe(struct i2c_client *client, const struct i2c_device_id * devid);
static int  shdisp_sensor_i2c_remove(struct i2c_client *client);


/* ------------------------------------------------------------------------- */
/* TYPES                                                                     */
/* ------------------------------------------------------------------------- */
typedef struct bdic_data_tag
{
    struct i2c_client *this_client;
} bdic_i2c_data_t;

typedef struct sensor_data_tag {
    struct i2c_client *this_client;
} sensor_data_t;

#ifdef CONFIG_OF
static const struct of_device_id shdisp_system_bdic_dt_match[] = {
    { .compatible = SHDISP_BDIC_I2C_DEVNAME, },
    {}
};
#endif /* CONFIG_OF */

static const struct i2c_device_id shdisp_bdic_id[] = {
    { SHDISP_BDIC_I2C_DEVNAME, 0 },
    { }
};

static struct i2c_driver bdic_driver =
{
    .driver = {
        .owner   = THIS_MODULE,
        .name    = SHDISP_BDIC_I2C_DEVNAME,
#ifdef CONFIG_OF
        .of_match_table = shdisp_system_bdic_dt_match,
#endif /* CONFIG_OF */
    },
    .class    = I2C_CLASS_HWMON,
    .probe    = shdisp_bdic_i2c_probe,
    .id_table = shdisp_bdic_id,
    .remove   = shdisp_bdic_i2c_remove,
};

#ifdef CONFIG_OF
static const struct of_device_id shdisp_system_sensor_dt_match[] = {
    { .compatible = SHDISP_SENSOR_DEVNAME, },
    {}
};
#endif /* CONFIG_OF */

static const struct i2c_device_id shdisp_sensor_id[] = {
    { SHDISP_SENSOR_DEVNAME, 0 },
    { }
};

static struct i2c_driver sensor_driver =
{
    .driver = {
        .owner   = THIS_MODULE,
        .name    = SHDISP_SENSOR_DEVNAME,
#ifdef CONFIG_OF
        .of_match_table = shdisp_system_sensor_dt_match,
#endif /* CONFIG_OF */
    },
    .class    = I2C_CLASS_HWMON,
    .probe    = shdisp_sensor_i2c_probe,
    .id_table = shdisp_sensor_id,
    .remove   = shdisp_sensor_i2c_remove,
};

/* ------------------------------------------------------------------------- */
/* VARIABLES                                                                 */
/* ------------------------------------------------------------------------- */
static bdic_i2c_data_t *bdic_i2c_p = NULL;

static sensor_data_t   *sensor_data_p = NULL;

static unsigned int shdisp_int_irq_port = 0;
static struct platform_device *shdisp_int_irq_port_pdev = NULL;
static int shdisp_int_irq_port_staus = 0;
static spinlock_t shdisp_set_irq_spinlock;


/* ------------------------------------------------------------------------- */
/* DEBUG MACROS                                                              */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/* FUNCTIONS                                                                 */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* API                                                                       */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_SYS_API_Host_control                                               */
/* ------------------------------------------------------------------------- */
int  shdisp_SYS_API_Host_control(int cmd, unsigned long rate)
{
    int ret = SHDISP_RESULT_SUCCESS;

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SYS_API_delay_us                                                   */
/* ------------------------------------------------------------------------- */
void shdisp_SYS_API_delay_us(unsigned long usec)
{
    struct timespec tu;

    SHDISP_WAIT_LOG("wait start. expect = %ld.%ld(ms). caller = %pS",
        (usec/1000), (usec%1000),__builtin_return_address(0));

    if (usec >= 1000 * 1000) {
        tu.tv_sec  = usec / 1000000;
        tu.tv_nsec = (usec % 1000000) * 1000;
    } else {
        tu.tv_sec  = 0;
        tu.tv_nsec = usec * 1000;
    }

    hrtimer_nanosleep(&tu, NULL, HRTIMER_MODE_REL, CLOCK_MONOTONIC);

    SHDISP_WAIT_LOG("wait end.");
    return;
}

/* ------------------------------------------------------------------------- */
/* void shdisp_SYS_API_msleep                                                */
/* ------------------------------------------------------------------------- */
void shdisp_SYS_API_msleep(unsigned int msec)
{
    SHDISP_WAIT_LOG("wait start. expect = %u. caller = %pS",
        msec,__builtin_return_address(0));
    msleep(msec);

    SHDISP_WAIT_LOG("wait end.");
    return;
}

/* ------------------------------------------------------------------------- */
/* void shdisp_SYS_API_usleep                                                */
/* ------------------------------------------------------------------------- */
void shdisp_SYS_API_usleep(unsigned int usec)
{
    SHDISP_WAIT_LOG("wait start. expect = %u.%u(ms). caller = %pS",
        (usec/1000), (usec%1000),__builtin_return_address(0));

    usleep(usec);

    SHDISP_WAIT_LOG("wait end.");
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SYS_API_Host_gpio_init                                             */
/* ------------------------------------------------------------------------- */
void shdisp_SYS_API_Host_gpio_init(void)
{
}

/* ------------------------------------------------------------------------- */
/* shdisp_SYS_API_Host_gpio_exit                                             */
/* ------------------------------------------------------------------------- */
void shdisp_SYS_API_Host_gpio_exit(void)
{
}

/* ------------------------------------------------------------------------- */
/* shdisp_SYS_API_Host_gpio_request                                          */
/* ------------------------------------------------------------------------- */
int shdisp_SYS_API_Host_gpio_request(int num, char *label)
{
    return shdisp_host_gpio_request(num, label);
}

/* ------------------------------------------------------------------------- */
/* shdisp_SYS_API_Host_gpio_free                                             */
/* ------------------------------------------------------------------------- */
int  shdisp_SYS_API_Host_gpio_free(int num)
{
    return shdisp_host_gpio_free(num);
}

/* ------------------------------------------------------------------------- */
/* shdisp_SYS_API_set_Host_gpio                                              */
/* ------------------------------------------------------------------------- */
int  shdisp_SYS_API_set_Host_gpio(int num, int value)
{
    if (value != SHDISP_GPIO_CTL_LOW &&
        value != SHDISP_GPIO_CTL_HIGH) {
        SHDISP_ERR("<INVALID_VALUE> value(%d).", value);
        return SHDISP_RESULT_FAILURE;
    }

    SHDISP_INFO("num(%d), value(%d)", num, value);
    gpio_set_value(num, value);
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SYS_API_get_Host_gpio                                              */
/* ------------------------------------------------------------------------- */
int shdisp_SYS_API_get_Host_gpio(int num)
{
    int ret;


    ret = gpio_get_value(num);
    SHDISP_INFO("num = %d, ret = %d", num, ret);


    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SYS_API_check_upper_unit                                           */
/* ------------------------------------------------------------------------- */
int shdisp_SYS_API_check_upper_unit(int gpio_no)
{
    int val;
    SHDISP_TRACE("in");
    gpio_request(gpio_no, "upper_unit");
    pin_config_set(SHDISP_GPIO_PIN_UPPER_UNIT, SHDISP_GPIO_NUM_UPPER_UNIT, PIN_CONFIG_BIAS_PULL_UP);

    shdisp_SYS_API_delay_us(5);
    val = shdisp_SYS_API_get_Host_gpio(gpio_no);

    pin_config_set(SHDISP_GPIO_PIN_UPPER_UNIT, SHDISP_GPIO_NUM_UPPER_UNIT, PIN_CONFIG_BIAS_PULL_DOWN);
    gpio_free(gpio_no);
    SHDISP_DEBUG("check_upper_unit val=%d", val);

    if (!val) {
        SHDISP_ERR("<OTHER> Upper unit does not exist.");
        return SHDISP_RESULT_FAILURE;
    }
    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SYS_API_set_irq_port                                               */
/* ------------------------------------------------------------------------- */
void shdisp_SYS_API_set_irq_port(int irq_port, struct platform_device *pdev)
{
    shdisp_int_irq_port = irq_port;
    shdisp_int_irq_port_pdev = pdev;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SYS_API_request_irq                                                */
/* ------------------------------------------------------------------------- */
int shdisp_SYS_API_request_irq(irqreturn_t (*irq_handler)(int , void *))
{
    int ret = SHDISP_RESULT_SUCCESS;
    if ((irq_handler == NULL)
    ||  (shdisp_int_irq_port_pdev == NULL)) {
        return SHDISP_RESULT_FAILURE;
    }

    ret = devm_request_irq(&shdisp_int_irq_port_pdev->dev, shdisp_int_irq_port, *irq_handler,
                        SHDISP_INT_FLAGS,   "shdisp", NULL);

    if (ret == 0) {
        disable_irq(shdisp_int_irq_port);
    }

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SYS_API_free_irq                                                   */
/* ------------------------------------------------------------------------- */
void shdisp_SYS_API_free_irq(void)
{
    free_irq(shdisp_int_irq_port, NULL);
}

/* ------------------------------------------------------------------------- */
/* shdisp_SYS_API_set_irq_init                                               */
/* ------------------------------------------------------------------------- */
void shdisp_SYS_API_set_irq_init(void)
{
    spin_lock_init( &shdisp_set_irq_spinlock );
}

/* ------------------------------------------------------------------------- */
/* shdisp_SYS_API_set_irq                                                    */
/* ------------------------------------------------------------------------- */
int shdisp_SYS_API_set_irq(int enable)
{
    int ret = SHDISP_RESULT_SUCCESS;
    unsigned long flags = 0;

    spin_lock_irqsave( &shdisp_set_irq_spinlock, flags);

    if (enable == shdisp_int_irq_port_staus) {
        spin_unlock_irqrestore( &shdisp_set_irq_spinlock, flags);
        return SHDISP_RESULT_SUCCESS;
    }

    if (enable == SHDISP_IRQ_ENABLE) {
        enable_irq_wake(shdisp_int_irq_port);
        enable_irq(shdisp_int_irq_port);
        shdisp_int_irq_port_staus = enable;
    } else if (enable == SHDISP_IRQ_DISABLE) {
        disable_irq_nosync(shdisp_int_irq_port);
        disable_irq_wake(shdisp_int_irq_port);
        shdisp_int_irq_port_staus = enable;
    } else {
        SHDISP_ERR("<INVALID_VALUE> enable=%d", enable);
        ret = SHDISP_RESULT_FAILURE;
    }
    spin_unlock_irqrestore( &shdisp_set_irq_spinlock, flags);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SYS_API_panel_external_clk_ctl                                     */
/* ------------------------------------------------------------------------- */
int shdisp_SYS_API_panel_external_clk_ctl(int enable)
{
    int ret = 0;

    SHDISP_TRACE("in enable=%d", enable);
    if (ret) {
        SHDISP_ERR("<RESULT_FAILURE> ret=%d", ret);
        ret = SHDISP_RESULT_FAILURE;
    } else {
        ret = SHDISP_RESULT_SUCCESS;
    }
    SHDISP_TRACE("out ret=%04x", ret);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SYS_API_Host_i2c_send                                              */
/* ------------------------------------------------------------------------- */
int  shdisp_SYS_API_Host_i2c_send(unsigned char slaveaddr, unsigned char *sendval, unsigned char size)
{
    struct i2c_adapter *adap;
    struct i2c_msg msg;
    int i2c_ret;
    int result = 1;
    int retry;
#ifdef SHDISP_LOG_ENABLE
    int i;
    char logbuf[512], work[16];
#endif /* SHDISP_LOG_ENABLE */

    if (slaveaddr == sensor_data_p->this_client->addr) {
        adap = sensor_data_p->this_client->adapter;
    } else {
        SHDISP_ERR("<OTHER> slaveaddr(0x%02x) device nothing.", slaveaddr);
        return SHDISP_RESULT_FAILURE;
    }

    if (sendval == NULL) {
        SHDISP_ERR("<NULL_POINTER> data.");
        return SHDISP_RESULT_FAILURE;
    }

    memset(&msg, 0, sizeof(msg));
    msg.addr     = slaveaddr;
    msg.flags    = 0;
    msg.len      = size;
    msg.buf      = sendval;


    for (retry = 0; retry <= 10; retry++) {
        i2c_ret = i2c_transfer(adap, &msg, 1);
        if (i2c_ret > 0) {
            result = 0;
            break;
        }
    }

#ifdef SHDISP_LOG_ENABLE
    memset(logbuf, 0x00, sizeof(logbuf));
    for (i = 0; i < size; i++) {
        sprintf(work, "%02X", msg.buf[i]);
        strcat(logbuf, work);
    }
    SHDISP_I2CLOG("slaveaddr=0x%02X, sendval=0x%s, size=%d", slaveaddr, logbuf, size);
#endif /* SHDISP_LOG_ENABLE */
    if (result == 1) {
        SHDISP_ERR("<OTHER> i2c_transfer time out(slaveaddr = 0x%02x, i2c_ret = %d).", slaveaddr, i2c_ret);
        return SHDISP_RESULT_FAILURE_I2C_TMO;
    }

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SYS_API_Host_i2c_recv                                              */
/* ------------------------------------------------------------------------- */
int  shdisp_SYS_API_Host_i2c_recv(unsigned char slaveaddr, unsigned char *sendval, unsigned char sendsize,
                                   unsigned char *recvval, unsigned char recvsize)
{
    struct i2c_adapter *adap;
    struct i2c_msg msg[2];
    int i2c_ret;
    int result = 1;
    int retry;
#ifdef SHDISP_LOG_ENABLE
    int i;
    char logbuf[512], work[16];
#endif /* SHDISP_LOG_ENABLE */

    if (slaveaddr == sensor_data_p->this_client->addr) {
        adap = sensor_data_p->this_client->adapter;
    } else {
        SHDISP_ERR("<OTHER> slaveaddr(0x%02x) device nothing.", slaveaddr);
        return SHDISP_RESULT_FAILURE;
    }

    if ((sendval == NULL) || (recvval == NULL)) {
        SHDISP_ERR("<NULL_POINTER> data.");
        return SHDISP_RESULT_FAILURE;
    }

    memset(msg, 0, sizeof(*msg) * ARRAY_SIZE(msg));
    msg[0].addr     = slaveaddr;
    msg[0].flags    = 0;
    msg[0].len      = sendsize;
    msg[0].buf      = sendval;

    msg[1].addr  = slaveaddr;
    msg[1].flags = I2C_M_RD;
    msg[1].len   = recvsize;
    msg[1].buf   = recvval;

    for (retry = 0; retry <= 10; retry++) {

        i2c_ret = i2c_transfer(adap, msg, 2);

        if (i2c_ret > 0) {
            result = 0;
            break;
        }
    }

#ifdef SHDISP_LOG_ENABLE
    memset(logbuf, 0x00, sizeof(logbuf));
    for (i = 0; i < sendsize; i++) {
        sprintf(work, "%02X", msg[0].buf[i]);
        strcat(logbuf, work);
    }
    SHDISP_I2CLOG("msg[0]: slaveaddr=0x%02X, sendval=0x%s, size=%d", slaveaddr, logbuf, sendsize);
    memset(logbuf, 0x00, sizeof(logbuf));
    for (i = 0; i < recvsize; i++) {
        sprintf(work, "%02X", msg[1].buf[i]);
        strcat(logbuf, work);
    }
    SHDISP_I2CLOG("msg[1]: slaveaddr=0x%02X, recvval=0x%s, size=%d", slaveaddr, logbuf, recvsize);
#endif /* SHDISP_LOG_ENABLE */
    if (result == 1) {
        SHDISP_ERR("<OTHER> i2c_transfer time out(i2c_ret = %d).", i2c_ret);
        return SHDISP_RESULT_FAILURE_I2C_TMO;
    }
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SYS_API_bdic_i2c_init                                              */
/* ------------------------------------------------------------------------- */
int  shdisp_SYS_API_bdic_i2c_init(void)
{
    int ret;

    ret = i2c_add_driver(&bdic_driver);
    if (ret < 0) {
        SHDISP_ERR("<RESULT_FAILURE> i2c_add_driver.");
        return SHDISP_RESULT_FAILURE;
    }
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SYS_API_bdic_i2c_exit                                              */
/* ------------------------------------------------------------------------- */
int  shdisp_SYS_API_bdic_i2c_exit(void)
{
    i2c_del_driver(&bdic_driver);

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SYS_API_bdic_i2c_write                                             */
/* ------------------------------------------------------------------------- */
int  shdisp_SYS_API_bdic_i2c_write(unsigned char addr, unsigned char data)
{
    struct i2c_msg msg;
    unsigned char write_buf[2];
    int i2c_ret;
    int result = 1;
    int retry;

    SHDISP_I2CLOG("(addr=0x%02X, data=0x%02X)", addr, data);

    msg.addr     = bdic_i2c_p->this_client->addr;
    msg.flags    = 0;
    msg.len      = 2;
    msg.buf      = write_buf;
    write_buf[0] = addr;
    write_buf[1] = data;

    for (retry = 0; retry <= 10; retry++) {
        i2c_ret = i2c_transfer(bdic_i2c_p->this_client->adapter, &msg, 1);
        if (i2c_ret > 0) {
            result = 0;
            break;
        }
    }

    if (result == 1) {
        SHDISP_ERR("<OTHER> i2c_transfer time out(i2c_ret = %d).", i2c_ret);
        return SHDISP_RESULT_FAILURE_I2C_TMO;
    }

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SYS_API_bdic_i2c_mask_write                                        */
/* ------------------------------------------------------------------------- */
int  shdisp_SYS_API_bdic_i2c_mask_write(unsigned char addr, unsigned char data, unsigned char mask)
{
    unsigned char read_data;
    unsigned char write_data;
    int ret;

    SHDISP_I2CLOG("(addr=0x%02X, data=0x%02X, mask=0x%02X)", addr, data, mask);
    ret = shdisp_SYS_API_bdic_i2c_read(addr, &read_data);
    if (ret == SHDISP_RESULT_SUCCESS) {
        write_data = ((read_data & ~mask) | (data & mask));
        if (write_data != read_data) {
            ret =  shdisp_SYS_API_bdic_i2c_write(addr, write_data);
        }
    }

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SYS_API_bdic_i2c_multi_write                                       */
/* ------------------------------------------------------------------------- */
int  shdisp_SYS_API_bdic_i2c_multi_write(unsigned char addr, unsigned char *wval, unsigned char size)
{
    struct i2c_msg msg;
    unsigned char write_buf[21];
    int i2c_ret;
    int result = 1;
    int retry;

    if ((size < 1) || (size > 20)) {
        SHDISP_ERR("<INVALID_VALUE> size(%d).", size);
        return SHDISP_RESULT_FAILURE;
    }

    if (wval == NULL) {
        SHDISP_ERR("<NULL_POINTER> val.");
        return SHDISP_RESULT_FAILURE;
    }

    msg.addr     = bdic_i2c_p->this_client->addr;
    msg.flags    = 0;
    msg.len      = size + 1;
    msg.buf      = write_buf;
    memset(write_buf, 0x00, sizeof(write_buf));
    write_buf[0] = addr;
    memcpy(&write_buf[1], wval, (int)size);
    SHDISP_I2CLOG("(addr=0x%02X, size=0x%02X", addr, size);
    SHDISP_I2CLOG("*wval=%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X)",
                                write_buf[1], write_buf[2], write_buf[3], write_buf[4], write_buf[5],
                                write_buf[6], write_buf[7], write_buf[8], write_buf[9], write_buf[10],
                                write_buf[11], write_buf[12], write_buf[13], write_buf[14], write_buf[15],
                                write_buf[16], write_buf[17], write_buf[18], write_buf[19], write_buf[20]);

    for (retry = 0; retry <= 10; retry++) {
        i2c_ret = i2c_transfer(bdic_i2c_p->this_client->adapter, &msg, 1);
        if (i2c_ret > 0) {
            result = 0;
            break;
        }
    }

    if (result == 1) {
        SHDISP_ERR("<OTHER> i2c_transfer time out(i2c_ret = %d).", i2c_ret);
        return SHDISP_RESULT_FAILURE_I2C_TMO;
    }

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SYS_API_bdic_i2c_read                                              */
/* ------------------------------------------------------------------------- */
int  shdisp_SYS_API_bdic_i2c_read(unsigned char addr, unsigned char *data)
{
    struct i2c_msg msg;
    unsigned char write_buf[2];
    unsigned char read_buf[2];
    int i2c_ret;
    int result = 1;
    int retry;

    if (data == NULL) {
        SHDISP_ERR("<NULL_POINTER> data.");
        return SHDISP_RESULT_FAILURE;
    }

    for (retry = 0; retry <= 10; retry++) {
        msg.addr     = bdic_i2c_p->this_client->addr;
        msg.flags    = 0;
        msg.len      = 1;
        msg.buf      = write_buf;
        write_buf[0] = addr;

        i2c_ret = i2c_transfer(bdic_i2c_p->this_client->adapter, &msg, 1);

        if (i2c_ret > 0) {

            msg.addr  = bdic_i2c_p->this_client->addr;
            msg.flags = I2C_M_RD;
            msg.len   = 1;
            msg.buf   = read_buf;

            i2c_ret = i2c_transfer(bdic_i2c_p->this_client->adapter, &msg, 1);
            if (i2c_ret > 0) {
                *data = read_buf[0];
                result = 0;
                break;
            }
        }
    }

    SHDISP_I2CLOG("(addr=0x%02X, data=0x%02X)", addr, *data);

    if (result == 1) {
        SHDISP_ERR("<OTHER> i2c_transfer time out(i2c_ret = %d).", i2c_ret);
        return SHDISP_RESULT_FAILURE_I2C_TMO;
    }

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SYS_API_bdic_i2c_multi_read                                        */
/* ------------------------------------------------------------------------- */
int  shdisp_SYS_API_bdic_i2c_multi_read(unsigned char addr, unsigned char *data, int size)
{
    struct i2c_msg msg;
    unsigned char write_buf[2];
    unsigned char read_buf[1 + 8];
    int i2c_ret;
    int result = 1;
    int retry;

    if ((size < 1) || (size > 8)) {
        SHDISP_ERR("<INVALID_VALUE> size(%d).", size);
        return SHDISP_RESULT_FAILURE;
    }

    if (data == NULL) {
        SHDISP_ERR("<NULL_POINTER> data.");
        return SHDISP_RESULT_FAILURE;
    }

    msg.addr     = bdic_i2c_p->this_client->addr;
    msg.flags    = 0;
    msg.len      = 1;
    msg.buf      = write_buf;
    write_buf[0] = addr;

    for (retry = 0; retry <= 10; retry++) {
        i2c_ret = i2c_transfer(bdic_i2c_p->this_client->adapter, &msg, 1);
        if (i2c_ret > 0) {
            result = 0;
            break;
        }
    }

    if (result == 0) {
        msg.addr  = bdic_i2c_p->this_client->addr;
        msg.flags = I2C_M_RD;
        msg.len   = size;
        msg.buf   = read_buf;
        memset(read_buf, 0x00, sizeof(read_buf));
        for (retry = 0; retry <= 10; retry++) {
            i2c_ret = i2c_transfer(bdic_i2c_p->this_client->adapter, &msg, 1);
            if (i2c_ret > 0) {
                memcpy(data, &read_buf[0], size);
                result = 0;
                break;
            }
        }
    }

    SHDISP_I2CLOG("(addr=0x%02X, size=0x%02X, *data=%02X%02X%02X%02X%02X%02X%02X%02X)",
                        addr, size,
                        read_buf[0], read_buf[1], read_buf[2], read_buf[3],
                        read_buf[4], read_buf[5], read_buf[6], read_buf[7]);

    if (result == 1) {
        SHDISP_ERR("<OTHER> i2c_transfer time out(i2c_ret = %d).", i2c_ret);
        return SHDISP_RESULT_FAILURE_I2C_TMO;
    }

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SYS_API_sensor_i2c_init                                            */
/* ------------------------------------------------------------------------- */
int  shdisp_SYS_API_sensor_i2c_init(void)
{
    int ret;

    ret = i2c_add_driver(&sensor_driver);
    if (ret < 0) {
        SHDISP_ERR("<RESULT_FAILURE> i2c_add_driver.");
        return SHDISP_RESULT_FAILURE;
    }
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SYS_API_sensor_i2c_exit                                            */
/* ------------------------------------------------------------------------- */
int  shdisp_SYS_API_sensor_i2c_exit(void)
{
    i2c_del_driver(&sensor_driver);
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SYS_API_get_debugflg                                               */
/* ------------------------------------------------------------------------- */
unsigned char shdisp_SYS_API_get_debugflg(void)
{
    sharp_smem_common_type *smem = NULL;

    smem = sh_smem_get_common_address();
    if (smem) {
        return smem->shdiag_debugflg;
    } else {
        return 0;
    }
}

/* ------------------------------------------------------------------------- */
/* SUB ROUTINE                                                               */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_host_gpio_request                                                  */
/* ------------------------------------------------------------------------- */
static int shdisp_host_gpio_request(int num, char *label)
{
    int ret = SHDISP_RESULT_SUCCESS;

    gpio_request(num, label);

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_host_gpio_free                                                     */
/* ------------------------------------------------------------------------- */
static int shdisp_host_gpio_free(int num)
{
    int ret = SHDISP_RESULT_SUCCESS;

    gpio_free(num);

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_i2c_probe                                                     */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_i2c_probe(struct i2c_client *client, const struct i2c_device_id *devid)
{
    bdic_i2c_data_t *i2c_p;

    if (bdic_i2c_p != NULL) {
        return -EPERM;
    }

    i2c_p = (bdic_i2c_data_t *)kzalloc(sizeof(bdic_i2c_data_t), GFP_KERNEL);
    if (i2c_p == NULL) {
        return -ENOMEM;
    }

    bdic_i2c_p = i2c_p;

    i2c_set_clientdata(client, i2c_p);
    i2c_p->this_client = client;

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_i2c_remove                                                    */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_i2c_remove(struct i2c_client *client)
{
    bdic_i2c_data_t *i2c_p;

    i2c_p = i2c_get_clientdata(client);

    kfree(i2c_p);

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_sensor_i2c_probe                                                   */
/* ------------------------------------------------------------------------- */
static int shdisp_sensor_i2c_probe(struct i2c_client *client, const struct i2c_device_id *devid)
{
    sensor_data_t *i2c_p;

    if (sensor_data_p != NULL) {
        return -EPERM;
    }

    i2c_p = (sensor_data_t *)kzalloc(sizeof(sensor_data_t), GFP_KERNEL);
    if (i2c_p == NULL) {
        return -ENOMEM;
    }

    sensor_data_p = i2c_p;

    i2c_set_clientdata(client, i2c_p);
    i2c_p->this_client = client;

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_sensor_i2c_remove                                                  */
/* ------------------------------------------------------------------------- */
static int shdisp_sensor_i2c_remove(struct i2c_client *client)
{
    sensor_data_t *i2c_p;

    i2c_p = i2c_get_clientdata(client);

    kfree(i2c_p);
    sensor_data_p = NULL;

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SYS_API_get_hayabusa_rev                                           */
/* ------------------------------------------------------------------------- */
int shdisp_SYS_API_get_hayabusa_rev(int hw_handset, int hw_revision)
{
#if defined(CONFIG_ARCH_LYNX_DL80) || defined(FEATURE_SH_MODEL_DL80)
    static const bool model_dl80 = 1;
#else  /* defined(CONFIG_ARCH_LYNX_DL80) || defined(FEATURE_SH_MODEL_DL80) */
    static const bool model_dl80 = 0;
#endif /* defined(CONFIG_ARCH_LYNX_DL80) || defined(FEATURE_SH_MODEL_DL80) */
#if defined(CONFIG_ARCH_PA30) || defined(FEATURE_SH_MODEL_PA30)
    static const bool model_pa30 = 0;
#else  /* defined(CONFIG_ARCH_PA30) || defined(FEATURE_SH_MODEL_PA30) */
    static const bool model_pa30 = 0;
#endif /* defined(CONFIG_ARCH_PA30) || defined(FEATURE_SH_MODEL_PA30) */


    if (model_dl80 || model_pa30) {
        if (hw_handset && (hw_revision <= SHDISP_HW_REV_ES1)) {
            return SHDISP_HAYABUSA_REV_CUT10;
        } else {
            return SHDISP_HAYABUSA_REV_CUT11;
        }
    } else {
        return SHDISP_HAYABUSA_REV_CUT11;
    }
}

/* ------------------------------------------------------------------------- */
/* shdisp_SYS_API_check_diag_mode                                            */
/* ------------------------------------------------------------------------- */
int shdisp_SYS_API_check_diag_mode(void)
{
    sharp_smem_common_type *p_sharp_smem_common_type;
    unsigned long bootmode;

    p_sharp_smem_common_type  = sh_smem_get_common_address();
    if (p_sharp_smem_common_type != 0) {
        bootmode = p_sharp_smem_common_type->sh_boot_mode;
    } else {
        bootmode = 0;
    }
    if ((bootmode == SHDISP_BOOT_D) || (bootmode == SHDISP_BOOT_F_F)) {
        return 1;
    } else {
        return 0;
    }
}

MODULE_DESCRIPTION("SHARP DISPLAY DRIVER MODULE");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.00");
/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
