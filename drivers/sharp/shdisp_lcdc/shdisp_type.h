/* drivers/sharp/shdisp_lcdc/shdisp_type.h  (Display Driver)
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

/* ------------------------------------------------------------------------- */
/* SHARP DISPLAY DRIVER FOR KERNEL MODE                                      */
/* ------------------------------------------------------------------------- */

#ifndef SHDISP_TYPE_H
#define SHDISP_TYPE_H

/* ------------------------------------------------------------------------- */
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */



/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/* STRUCTURE                                                                 */
/* ------------------------------------------------------------------------- */

struct shdisp_queue_data_t {
    int                 irq_GFAC;
    struct list_head    list;
};


/* ------------------------------------------------------------------------- */
/* TYPES                                                                     */
/* ------------------------------------------------------------------------- */

enum {
    SHLCDC_DISP_OFF,
    SHLCDC_DISP_ON,
    SHLCDC_DISP_FROM_ON_TO_OFF,
    SHLCDC_DISP_FROM_OFF_TO_ON,
    NUM_SHLCDC_DISP_STATUS
};

enum {
    SHDISP_DRIVER_IS_NOT_INITIALIZED,
    SHDISP_DRIVER_IS_INITIALIZED,
    NUM_SHDISP_DRIVER_STATUS
};

enum {
    SHDISP_THREAD_IS_NOT_ALIVE,
    SHDISP_THREAD_IS_ALIVE,
    NUM_SHDISP_THREAD_STATUS
};

enum {
    SHDISP_SUBSCRIBE_TYPE_INT,
    SHDISP_SUBSCRIBE_TYPE_POL,
    NUM_SHDISP_SUBSCRIBE_TYPE
};

enum {
    SHDISP_DEBUG_PROCESS_STATE_OUTPUT,
    SHDISP_DEBUG_TRACE_LOG_SWITCH,
    SHDISP_DEBUG_BDIC_I2C_WRITE,
    SHDISP_DEBUG_BDIC_I2C_READ,
    SHDISP_DEBUG_PROX_SENSOR_CTL,
    SHDISP_DEBUG_BKL_CTL,
    NUM_SHDISP_DEBUG
};

enum {
    SHDISP_DEBUG_INFO_TYPE_BOOT,
    SHDISP_DEBUG_INFO_TYPE_KERNEL,
    SHDISP_DEBUG_INFO_TYPE_BDIC,
    SHDISP_DEBUG_INFO_TYPE_SENSOR,
    SHDISP_DEBUG_INFO_TYPE_POWERON,
    SHDISP_DEBUG_INFO_TYPE_PANEL,
    SHDISP_DEBUG_INFO_TYPE_PM,
    NUM_SHDISP_DEBUG_INFO_TYPE
};

enum {
    SHDISP_LUX_CHANGE_STATE_INIT,
    SHDISP_LUX_CHANGE_STATE_WAIT,
    SHDISP_LUX_CHANGE_STATE_WAKEUP,
    SHDISP_LUX_CHANGE_STATE_EXIT,
    NUM_SHDISP_LUX_CHANGE_STATE
};

enum {
    SHDISP_BKL_MODE_OFF,
    SHDISP_BKL_MODE_ON,
    SHDISP_BKL_MODE_AUTO,
    NUM_SHDISP_BKL_MODE
};

enum {
    SHDISP_ALS_IRQ_SUBSCRIBE_TYPE_BKL_CTRL,
    SHDISP_ALS_IRQ_SUBSCRIBE_TYPE_DBC_IOCTL,
    NUM_SHDISP_ALS_IRQ_SUBSCRIBE_TYPE
};

enum {
    SHDISP_DISP_PIC_ADJ_MODE_00,
    SHDISP_DISP_PIC_ADJ_MODE_01,
    SHDISP_DISP_PIC_ADJ_MODE_02,
    SHDISP_DISP_PIC_ADJ_MODE_03,
    SHDISP_DISP_PIC_ADJ_MODE_04,
    SHDISP_DISP_PIC_ADJ_MODE_05,
    SHDISP_DISP_PIC_ADJ_MODE_06,
    NUM_SHDISP_DISP_PIC_ADJ_MODE
};

#endif /* SHDISP_TYPE_H */

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
