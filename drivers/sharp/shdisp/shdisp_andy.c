/* drivers/sharp/shdisp/shdisp_andy.c  (Display Driver)
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
#include <linux/of_gpio.h>
#include <linux/hrtimer.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/wakelock.h>
#include <linux/msm_mdp.h>
#include "shdisp_panel.h"
#include "shdisp_andy.h"
#include "shdisp_system.h"
#include "shdisp_bdic.h"
#include "shdisp_type.h"
#include "shdisp_pm.h"
#include "shdisp_dbg.h"
#include "shdisp_kerl_priv.h"
#include <sharp/sh_smem.h>
#include <sharp/sh_boot_manager.h>

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */
#define SHDISP_POWER_MODE_CHK

#define SHDISP_PANEL_DISPON_CHK_INIT            (0)
#define SHDISP_PANEL_DISPON_CHK_OK              (1)
#define SHDISP_PANEL_DISPON_CHK_NG              (2)

#define SHDISP_ANDY_VCOM_REG_NUM                (6)

#define SHDISP_ANDY_GMM_SETTING_SIZE            (60)
#define SHDISP_ANDY_GMM_LEVEL_MIN               (1)
#define SHDISP_ANDY_GMM_LEVEL_MAX               (30)
#define SHDISP_ANDY_GMM_NEGATIVE_OFFSET         (30)
#define SHDISP_ANDY_GMM_GROUP_BELONG_LEVEL      (2)
#define SHDISP_ANDY_GMM_GROUP_BELONG_ADDR       (4)
#define SHDISP_ANDY_VGH                         (2)
#define SHDISP_ANDY_VGL                         (3)
#define SHDISP_ANDY_VDD_Reg                     (4)
#define SHDISP_ANDY_GVDDP                       (6)
#define SHDISP_ANDY_GVDDN                       (7)
#define SHDISP_ANDY_GVDDP2                      (8)
#define SHDISP_ANDY_VGHO                        (10)
#define SHDISP_ANDY_VGLO                        (11)
#define SHDISP_ANDY_AVDDR                       (15)
#define SHDISP_ANDY_AVEER                       (16)

#define SHDISP_ANDY_CUT_ID                      (0x80)

/* ------------------------------------------------------------------------- */
/* TYPES                                                                     */
/* ------------------------------------------------------------------------- */
struct shdisp_andy_calc_vcom_in {
    unsigned short vcom;
    unsigned short vcom_low;
};

struct shdisp_andy_calc_vcom_out {
    char vcom1_l;
    char vcom2_l;
    char vcom12_h;
    char lpvcom1;
    char lpvcom2;
    char vcomoff;
};

/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */
static int shdisp_andy_calc_vcom_param(struct shdisp_andy_calc_vcom_in in, struct shdisp_andy_calc_vcom_out *out, unsigned short vcomoffset);
static int shdisp_andy_init_flicker_param(unsigned short vcom, unsigned short vcom_low);
static int shdisp_andy_API_init_io(struct shdisp_panel_context *panel_ctx);
static int shdisp_andy_API_exit_io(void);
static int shdisp_andy_API_power_on(int mode);
static int shdisp_andy_API_power_off(int mode);
static int shdisp_andy_API_disp_on(void);
static int shdisp_andy_API_disp_off(void);
static int shdisp_andy_API_start_display(void);
static int shdisp_andy_API_post_video_start(void);
static int shdisp_andy_API_diag_write_reg(unsigned char addr, unsigned char *write_data, unsigned char size);
static int shdisp_andy_API_diag_read_reg(unsigned char addr, unsigned char *read_data, unsigned char size);
static int shdisp_andy_API_diag_set_flicker_param(struct shdisp_diag_flicker_param vcom);
static int shdisp_andy_API_diag_get_flicker_param(struct shdisp_diag_flicker_param *vcom);
static int shdisp_andy_API_diag_get_flicker_low_param(struct shdisp_diag_flicker_param *vcom);
static int shdisp_andy_API_check_recovery(void);
static int shdisp_andy_API_diag_set_gmmtable_and_voltage(struct shdisp_diag_gamma_info *gmm_info);
static int shdisp_andy_API_diag_get_gmmtable_and_voltage(struct shdisp_diag_gamma_info *gmm_info);
static int shdisp_andy_API_diag_set_gmm(struct shdisp_diag_gamma *gmm);
static int shdisp_andy_API_shutdown(void);
static void shdisp_andy_API_dump(int type);
static int shdisp_andy_API_set_irq(int enable);
static int shdisp_andy_API_set_freq_param(struct shdisp_freq_params *freq);
static void shdisp_andy_hw_reset(bool);
static int shdisp_andy_mipi_cmd_lcd_on(void);
static int shdisp_andy_mipi_cmd_lcd_off(void);
static int shdisp_andy_mipi_cmd_display_on(void);
static int shdisp_andy_set_switchcommand(char val);

#ifndef SHDISP_NOT_SUPPORT_FLICKER
static int shdisp_andy_diag_set_flicker_param_internal(struct shdisp_diag_flicker_param flicker_param);
static int shdisp_andy_diag_set_flicker_param_ctx(struct shdisp_diag_flicker_param flicker_param);
#endif /* SHDISP_NOT_SUPPORT_FLICKER */

#ifdef SHDISP_POWER_MODE_CHK
static int shdisp_andy_power_mode_chk(unsigned char addr);
#endif /* SHDISP_POWER_MODE_CHK */

static int shdisp_andy_sleepout_wait_proc(void);

#ifndef SHDISP_NOT_SUPPORT_NO_OS
static int shdisp_andy_init_phy_gmm(struct shdisp_lcddr_phy_gmm_reg *phy_gmm);
#endif /* SHDISP_NOT_SUPPORT_NO_OS */
extern void mdss_shdisp_video_transfer_ctrl(int onoff, int commit);
extern int mdss_shdisp_dsi_bus_clk_ctrl(bool enable);
static irqreturn_t shdisp_andy_int_isr(int irq_num, void *data);
static int shdisp_andy_register_driver(void);
static void shdisp_andy_workqueue_handler(struct work_struct *work);

#ifdef SHDISP_ANDY_VDD
static void shdisp_andy_vdd_on(void);
static void shdisp_andy_vdd_off(void);
#endif /* SHDISP_ANDY_VDD */

/* ------------------------------------------------------------------------- */
/* VARIABLES                                                                 */
/* ------------------------------------------------------------------------- */
#ifndef SHDISP_NOT_SUPPORT_FLICKER
static unsigned char andy_wdata[8];
static unsigned char andy_rdata[8];
static char Andy_VCOM_Reg[6] = {0x13, 0x14, 0x15, 0x5C, 0x5D, 0x5E};
#endif /* SHDISP_NOT_SUPPORT_FLICKER */

static struct shdisp_diag_gamma_info diag_tmp_gmm_info;
static int diag_tmp_gmm_info_set = 0;

#ifdef SHDISP_POWER_MODE_CHK
static int shdisp_andy_dispon_chk = SHDISP_PANEL_DISPON_CHK_INIT;
#endif /* SHDISP_POWER_MODE_CHK */

static struct shdisp_panel_context shdisp_panel_ctx;
static struct workqueue_struct    *shdisp_wq_andy = NULL;
static struct work_struct         shdisp_wq_andy_wk;
static int shdisp_andy_irq_gpio = 0;
static int shdisp_andy_rst_gpio = 0;
static int shdisp_andy_vdd_gpio = 0;
static int shdisp_andy_mipi_err_gpio = 0;
static struct platform_device *pshdisp_andy_irq_port_dev = NULL;
static struct wake_lock shdisp_andy_wakelock;

/* ------------------------------------------------------------------------- */
/*      packet header                                                        */
/* ------------------------------------------------------------------------- */
/*      LCD ON                                                              */
/*      Initial Setting                                                     */

#include "./data/shdisp_andy_data_default.h"

#ifndef SHDISP_NOT_SUPPORT_FLICKER
static unsigned char mipi_sh_shdisp_andy_cmd_vcom_tracking[6][2] = {
    { 0x13, 0x00 },
    { 0x14, 0x00 },
    { 0x15, 0x00 },
    { 0x5C, 0x00 },
    { 0x5D, 0x00 },
    { 0x5E, 0x00 },
};

static struct shdisp_dsi_cmd_desc mipi_sh_shdisp_andy_cmds_vcom_tracking[8] = {
    { SHDISP_DTYPE_DCS_WRITE1, 2, mipi_sh_andy_cmd_SwitchCommand[1],        0},
    { SHDISP_DTYPE_DCS_WRITE1, 2, mipi_sh_shdisp_andy_cmd_vcom_tracking[0], 0},
    { SHDISP_DTYPE_DCS_WRITE1, 2, mipi_sh_shdisp_andy_cmd_vcom_tracking[1], 0},
    { SHDISP_DTYPE_DCS_WRITE1, 2, mipi_sh_shdisp_andy_cmd_vcom_tracking[2], 0},
    { SHDISP_DTYPE_DCS_WRITE1, 2, mipi_sh_shdisp_andy_cmd_vcom_tracking[3], 0},
    { SHDISP_DTYPE_DCS_WRITE1, 2, mipi_sh_shdisp_andy_cmd_vcom_tracking[4], 0},
    { SHDISP_DTYPE_DCS_WRITE1, 2, mipi_sh_shdisp_andy_cmd_vcom_tracking[5], 0},
    { SHDISP_DTYPE_DCS_WRITE1, 2, mipi_sh_andy_cmd_SwitchCommand[0],        0},
};
#endif /* SHDISP_NOT_SUPPORT_FLICKER */

#ifdef SHDISP_POWER_MODE_CHK
static struct shdisp_dsi_cmd_desc mipi_sh_andy_cmds_dispon_check[] = {
    {SHDISP_DTYPE_DCS_WRITE1, 2, mipi_sh_andy_cmd_SwitchCommand[0], 0, 0}
};
#endif /* SHDISP_POWER_MODE_CHK */

static struct shdisp_panel_operations shdisp_andy_fops = {
    shdisp_andy_API_init_io,
    shdisp_andy_API_exit_io,
    NULL,
    shdisp_andy_API_power_on,
    shdisp_andy_API_power_off,
    shdisp_andy_API_disp_on,
    shdisp_andy_API_disp_off,
    shdisp_andy_API_start_display,
    shdisp_andy_API_post_video_start,
    NULL,
    shdisp_andy_API_diag_write_reg,
    shdisp_andy_API_diag_read_reg,
    shdisp_andy_API_diag_set_flicker_param,
    shdisp_andy_API_diag_get_flicker_param,
    shdisp_andy_API_diag_get_flicker_low_param,
    shdisp_andy_API_check_recovery,
    shdisp_andy_API_diag_set_gmmtable_and_voltage,
    shdisp_andy_API_diag_get_gmmtable_and_voltage,
    shdisp_andy_API_diag_set_gmm,
    shdisp_andy_API_shutdown,
    shdisp_andy_API_dump,
    shdisp_andy_API_set_irq,
    NULL,
    NULL,
    shdisp_andy_API_set_freq_param,
};

/* ------------------------------------------------------------------------- */
/* DEBUG MACROS                                                              */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */
#define MIPI_DSI_COMMAND_TX(x)              (shdisp_panel_API_mipi_dsi_cmds_tx(0, x, ARRAY_SIZE(x)))
#define MIPI_DSI_COMMAND_TX_COMMIT(x)       (shdisp_panel_API_mipi_dsi_cmds_tx(1, x, ARRAY_SIZE(x)))
#define IS_FLICKER_ADJUSTED(param)          (((param & 0xF000) == 0x9000) ? 1 : 0)
#define TRACKING_MAX_VAL                    (-1)
#define TRACKING_MIN_VAL                    (-3)

/* ------------------------------------------------------------------------- */
/* FUNCTIONS                                                                 */
/* ------------------------------------------------------------------------- */
#if defined(CONFIG_ANDROID_ENGINEERING)
static int shdisp_andy_dump_reg(void);
#endif /* CONFIG_ANDROID_ENGINEERING */

/* ------------------------------------------------------------------------- */
/* API                                                                       */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_andy_API_create                                                    */
/* ------------------------------------------------------------------------- */
struct shdisp_panel_operations *shdisp_andy_API_create(void)
{
    return &shdisp_andy_fops;
}
/* ------------------------------------------------------------------------- */
/* shdisp_andy_API_vcom_tracking                                             */
/* ------------------------------------------------------------------------- */
int shdisp_andy_API_vcom_tracking(int tracking)
{
#ifndef SHDISP_NOT_SUPPORT_FLICKER
    int ret;
    struct shdisp_andy_calc_vcom_in in;
    struct shdisp_andy_calc_vcom_out out;
    unsigned short vcomoffset;
    int is_tracking_panel = 0;

    SHDISP_TRACE("in tracking=%d", tracking);


    if (!is_tracking_panel) {
        SHDISP_DEBUG("not tracking panel.");
        return SHDISP_RESULT_SUCCESS;
    }

    if ((tracking < TRACKING_MIN_VAL) || (tracking > TRACKING_MAX_VAL)) {
        SHDISP_ERR("tracking value error.");
        return SHDISP_RESULT_FAILURE;
    }

    vcomoffset = VCOM_OFFSET + tracking;

    in.vcom     = shdisp_panel_ctx.vcom;
    in.vcom_low = shdisp_panel_ctx.vcom_low;

    shdisp_andy_calc_vcom_param(in, &out, vcomoffset);

    mipi_sh_shdisp_andy_cmd_vcom_tracking[0][1] = out.vcom1_l;
    mipi_sh_shdisp_andy_cmd_vcom_tracking[1][1] = out.vcom2_l;
    mipi_sh_shdisp_andy_cmd_vcom_tracking[2][1] = out.vcom12_h;
    mipi_sh_shdisp_andy_cmd_vcom_tracking[3][1] = out.lpvcom1;
    mipi_sh_shdisp_andy_cmd_vcom_tracking[4][1] = out.lpvcom2;
    mipi_sh_shdisp_andy_cmd_vcom_tracking[5][1] = out.vcomoff;


    ret = MIPI_DSI_COMMAND_TX_COMMIT(mipi_sh_shdisp_andy_cmds_vcom_tracking);

    if(ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("vcom tracking write error.");
        return SHDISP_RESULT_FAILURE;
    }

    SHDISP_TRACE("out");

#endif /* SHDISP_NOT_SUPPORT_FLICKER */
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_andy_API_vcom_is_adjusted                                             */
/* ------------------------------------------------------------------------- */
int shdisp_andy_API_vcom_is_adjusted(void)
{
    return (IS_FLICKER_ADJUSTED(shdisp_panel_ctx.vcom_nvram));
}


/* ------------------------------------------------------------------------- */
/* shdisp_andy_calc_flicker_param                                            */
/* ------------------------------------------------------------------------- */
static int shdisp_andy_calc_vcom_param(struct shdisp_andy_calc_vcom_in in, struct shdisp_andy_calc_vcom_out *out, unsigned short vcomoffset)
{
    unsigned short tmp;
    unsigned short vcomadj;
    signed short vcomadj_tmp;
    unsigned short vcomdcoff;

    SHDISP_TRACE("in");
    SHDISP_DEBUG("vcom=0x%04x vcom_low=0x%04x vcom_offset=0x%04x", in.vcom, in.vcom_low, vcomoffset);

    if (out == NULL) {
        SHDISP_ERR("<NULL_POINTER> out.");
        return SHDISP_RESULT_FAILURE;
    }

    vcomadj_tmp = (signed short)in.vcom + vcomoffset;
    if (vcomadj_tmp > VCOM_MAX) {
        vcomadj_tmp = VCOM_MAX;
    }
    if (vcomadj_tmp < VCOM_MIN) {
        vcomadj_tmp = VCOM_MIN;
    }
    vcomadj = (unsigned short)vcomadj_tmp;
    SHDISP_DEBUG("vcomadj=0x%04x", vcomadj);

    out->vcom1_l = vcomadj & 0xFF;
    out->vcom2_l = out->vcom1_l;
    out->vcom12_h = 0x60;
    if ((vcomadj >> 8) & 0x01) {
        out->vcom12_h |= 0x03;
    }

    SHDISP_DEBUG("VCOM1_L=0x%02x VCOM2_L=0x%02x VCOM12_H=0x%02x",
                        out->vcom1_l,
                        out->vcom2_l,
                        out->vcom12_h);

    vcomdcoff = (vcomadj + 1) / 2;

    if (in.vcom_low - in.vcom >= 0) {
        tmp = in.vcom_low - in.vcom;
        out->lpvcom1 = (tmp & 0x0F);
    } else {
        tmp = in.vcom - in.vcom_low - 1;
        out->lpvcom1 = ((tmp & 0x0F) | 0x10);
    }
    out->lpvcom2 = out->lpvcom1;
    if (vcomdcoff & 0x100) {
        out->lpvcom2 |= 0x80;
    }
    out->vcomoff = (unsigned char) (vcomdcoff & 0xFF);

    SHDISP_DEBUG("LPVCOM1=0x%02x LPVCOM2=0x%02x VCOMOFF=0x%02x",
                        out->lpvcom1,
                        out->lpvcom2,
                        out->vcomoff);

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_andy_init_flicker_param                                            */
/* ------------------------------------------------------------------------- */
static int shdisp_andy_init_flicker_param(unsigned short vcom, unsigned short vcom_low)
{
#ifndef SHDISP_NOT_SUPPORT_FLICKER
    int ret;
    struct shdisp_andy_calc_vcom_in in;
    struct shdisp_andy_calc_vcom_out out;
    unsigned short vcomoffset;
    sharp_smem_common_type *p_sharp_smem_common_type;
    unsigned long bootmode;

    SHDISP_TRACE("in");
    SHDISP_DEBUG("vcom=0x%04x vcom_low=0x%04x", vcom, vcom_low);

    in.vcom = vcom;
    in.vcom_low = vcom_low;

    p_sharp_smem_common_type  = sh_smem_get_common_address();
    if( p_sharp_smem_common_type != 0 )
    {
        bootmode = p_sharp_smem_common_type->sh_boot_mode;
    } else {
        bootmode = 0;
    }
    if ((bootmode == SH_BOOT_D) || (bootmode == SH_BOOT_F_F)) {
        vcomoffset = 0;
    } else {
        vcomoffset = VCOM_OFFSET_UP;
    }

    ret = shdisp_andy_calc_vcom_param(in, &out, vcomoffset);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_andy_calc_vcom_param.");
        return SHDISP_RESULT_FAILURE;
    }

    mipi_sh_andy_cmd_RegulatorPumpSetting[NO_VCOM1_L][1] = out.vcom1_l;
    mipi_sh_andy_cmd_RegulatorPumpSetting[NO_VCOM2_L][1] = out.vcom2_l;
    mipi_sh_andy_cmd_RegulatorPumpSetting[NO_VCOM12_H][1] = out.vcom12_h;
    mipi_sh_andy_cmd_RegulatorPumpSetting[NO_LPVCOM1][1] = out.lpvcom1;
    mipi_sh_andy_cmd_RegulatorPumpSetting[NO_LPVCOM2][1] = out.lpvcom2;
    mipi_sh_andy_cmd_RegulatorPumpSetting[NO_VCOMOFF][1] = out.vcomoff;

    SHDISP_TRACE("out");

#endif
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_andy_API_init_io                                                   */
/* ------------------------------------------------------------------------- */
static int shdisp_andy_API_init_io(struct shdisp_panel_context *panel_ctx)
{
    int funcret = SHDISP_RESULT_SUCCESS;

    SHDISP_TRACE("in");

    memcpy(&(shdisp_panel_ctx), panel_ctx, sizeof(struct shdisp_panel_context));

#ifndef SHDISP_NOT_SUPPORT_NO_OS
    if (shdisp_andy_init_flicker_param(shdisp_panel_ctx.vcom, shdisp_panel_ctx.vcom_low)) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_andy_init_flicker_param.");
    }

    if (shdisp_andy_init_phy_gmm(&shdisp_panel_ctx.lcddr_phy_gmm)) {
        SHDISP_DEBUG("<RESULT_FAILURE> shdisp_andy_init_phy_gmm.");
    }

    shdisp_andy_dispon_chk = shdisp_panel_ctx.disp_on_status;
#endif /* SHDISP_NOT_SUPPORT_NO_OS */

    shdisp_wq_andy = create_singlethread_workqueue("shdisp_andy_queue");
    if (!shdisp_wq_andy) {
        SHDISP_ERR("failed to create_singlethread_workqueue().");
        funcret = SHDISP_RESULT_FAILURE;
        goto exit_with_error;
    }

    INIT_WORK(&shdisp_wq_andy_wk, shdisp_andy_workqueue_handler);

    wake_lock_init(&shdisp_andy_wakelock, WAKE_LOCK_SUSPEND, "andy_wake_lock");

    shdisp_andy_register_driver();

    if (shdisp_api_get_boot_disp_status() == SHDISP_MAIN_DISP_ON) {
        shdisp_andy_hw_reset(false);
    }

    goto exit;
exit_with_error:
    destroy_workqueue(shdisp_wq_andy);
    shdisp_wq_andy = NULL;

exit:

    SHDISP_TRACE("out");
    return funcret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_andy_API_exit_io                                                   */
/* ------------------------------------------------------------------------- */
static int shdisp_andy_API_exit_io(void)
{
    SHDISP_TRACE("in");

    if (shdisp_wq_andy) {
        flush_workqueue(shdisp_wq_andy);
        destroy_workqueue(shdisp_wq_andy);
        shdisp_wq_andy = NULL;
    }

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_andy_API_power_on                                                  */
/* ------------------------------------------------------------------------- */
static int shdisp_andy_API_power_on(int mode)
{
    SHDISP_TRACE("in mode=%d", mode);

    switch (mode) {
    case SHDISP_PANEL_POWER_FIRST_ON:
    case SHDISP_PANEL_POWER_RECOVERY_ON:
#ifdef SHDISP_ANDY_VDD
        shdisp_andy_vdd_on();
#else  /* SHDISP_ANDY_VDD */
        shdisp_SYS_API_delay_us(10 * 1000);
#endif /* SHDISP_ANDY_VDD */
        break;
    case SHDISP_PANEL_POWER_NORMAL_ON:
    default:
        shdisp_andy_hw_reset(true);
        shdisp_SYS_API_delay_us(3 * 1000);
        break;
    }

    shdisp_andy_hw_reset(false);
    shdisp_SYS_API_delay_us(10 * 1000);
    shdisp_andy_hw_reset(true);
    shdisp_SYS_API_delay_us(1 * 1000);
    shdisp_andy_hw_reset(false);
    shdisp_SYS_API_delay_us(10 * 1000);
    shdisp_andy_hw_reset(true);
    shdisp_SYS_API_delay_us(1 * 1000);
    shdisp_andy_hw_reset(false);
    shdisp_SYS_API_delay_us(10 * 1000);

    shdisp_bdic_API_LCD_power_on();
    shdisp_bdic_API_LCD_m_power_on();
    shdisp_SYS_API_delay_us(1 * 1000);

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_andy_API_power_off                                                 */
/* ------------------------------------------------------------------------- */
static int shdisp_andy_API_power_off(int mode)
{
    switch (mode) {
    case SHDISP_PANEL_POWER_RECOVERY_OFF:
        SHDISP_TRACE("in RECOVERY_OFF: mode=%d", mode);
        break;
    case SHDISP_PANEL_POWER_SHUTDOWN_OFF:
        SHDISP_TRACE("in SHUTDOWN_OFF: mode=%d", mode);
        break;
    case SHDISP_PANEL_POWER_NORMAL_OFF:
    default:
        SHDISP_TRACE("in NORMAL_OFF: mode=%d", mode);
        break;
    }

    shdisp_bdic_API_LCD_m_power_off();
    shdisp_bdic_API_LCD_power_off();
    switch (mode) {
    case SHDISP_PANEL_POWER_RECOVERY_OFF:
    case SHDISP_PANEL_POWER_SHUTDOWN_OFF:
        SHDISP_DEBUG("excute andy HW reset");
        shdisp_andy_hw_reset(true);
        shdisp_SYS_API_delay_us(80 * 1000);
        break;
    default:
        break;
    }

#ifdef SHDISP_ANDY_VDD
    switch (mode) {
    case SHDISP_PANEL_POWER_RECOVERY_OFF:
    case SHDISP_PANEL_POWER_SHUTDOWN_OFF:
        shdisp_andy_vdd_off();
        break;
    case SHDISP_PANEL_POWER_NORMAL_OFF:
    default:
        break;
    }
#endif /* SHDISP_ANDY_VDD */

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_andy_API_disp_on                                                   */
/* ------------------------------------------------------------------------- */
static int shdisp_andy_API_disp_on(void)
{
    int ret = 0;

    SHDISP_TRACE("in");

    ret = shdisp_andy_mipi_cmd_lcd_on();

    SHDISP_TRACE("out ret=%d", ret);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_andy_API_disp_off                                                  */
/* ------------------------------------------------------------------------- */
static int shdisp_andy_API_disp_off(void)
{
    SHDISP_TRACE("in");

    (void)shdisp_pm_API_als_user_manager(SHDISP_DEV_TYPE_LCD, SHDISP_DEV_REQ_OFF);
    shdisp_andy_mipi_cmd_lcd_off();

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_andy_API_start_display                                             */
/* ------------------------------------------------------------------------- */
static int shdisp_andy_API_start_display(void)
{
    SHDISP_TRACE("in");

    shdisp_andy_mipi_cmd_display_on();

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_andy_API_post_video_start                                          */
/* ------------------------------------------------------------------------- */
static int shdisp_andy_API_post_video_start(void)
{
    SHDISP_TRACE("in");
    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_andy_API_set_irq                                                   */
/* ------------------------------------------------------------------------- */
static int shdisp_andy_API_set_irq(int enable)
{
    int ret = SHDISP_RESULT_SUCCESS;

    SHDISP_TRACE("in (enable=%d)", enable);

    switch (enable) {
    case SHDISP_IRQ_ENABLE:
        ret = devm_request_irq(&pshdisp_andy_irq_port_dev->dev,
                                   shdisp_andy_irq_gpio, shdisp_andy_int_isr,
                                   IRQF_TRIGGER_RISING, "shdisp_andy", NULL);
        if (ret) {
            ret = SHDISP_RESULT_FAILURE;
            SHDISP_ERR("failed to request_irq(). (ret=%d irq=%d)", ret, shdisp_andy_irq_gpio);
        }
        break;
    case SHDISP_IRQ_DISABLE:
        disable_irq(shdisp_andy_irq_gpio);
        free_irq(shdisp_andy_irq_gpio, NULL);
        break;
    default:
        ret = SHDISP_RESULT_FAILURE;
        SHDISP_ERR("invalid argument. (enable=%d)", enable);
    }


    SHDISP_TRACE("out (ret=%d)", ret);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_andy_API_set_freq_param                                            */
/* ------------------------------------------------------------------------- */
static int shdisp_andy_API_set_freq_param(struct shdisp_freq_params *freq)
{
    SHDISP_TRACE("in");
    mipi_sh_andy_cmd_DisplayLineSetting[NO_RTN][1] = freq->rtn;
    mipi_sh_andy_cmd_SuspendTimingSetting[NO_GIP][1] = freq->gip;
    mipi_sh_andy_cmd_PorchLineSetting[NO_VBP][1] = freq->vbp;
    mipi_sh_andy_cmd_PorchLineSetting[NO_VFP][1] = freq->vfp;
    SHDISP_DEBUG("RTN=0x%02x GIP=0x%02x VBP=0x%02x VFP=0x%02x"
                    , freq->rtn, freq->gip, freq->vbp, freq->vfp);
    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_andy_start_video                                                   */
/* ------------------------------------------------------------------------- */
static int shdisp_andy_start_video(void)
{
    struct {
        unsigned char addr;
        unsigned char data;
        unsigned char size;
        unsigned long wait;
    } reg_data[] = {
        { 0xFF, 0x00, 1, 0 },
        { 0x29, 0x00, 0, 0 },
     };
    int i;
    int ret = 0;

    SHDISP_TRACE("in");

    for (i = 0; i < 2; i++) {
        ret = shdisp_andy_API_diag_write_reg(reg_data[i].addr, &reg_data[i].data, reg_data[i].size);
        if (reg_data[i].wait) {
            shdisp_SYS_API_delay_us(reg_data[i].wait);
        }
    }
    enable_irq(shdisp_andy_irq_gpio);
    shdisp_bdic_API_IRQ_det_irq_ctrl(true);

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_andy_stop_video                                                    */
/* ------------------------------------------------------------------------- */
static int shdisp_andy_stop_video(void)
{
    struct {
        unsigned char addr;
        unsigned char data;
        unsigned char size;
        unsigned long wait;
    } reg_data[] = {
        { 0xFF, 0x00, 1, 0 },
        { 0x28, 0x00, 0, WAIT_1FRAME_US * 1 },
    };
    int i;
    int ret = 0;

    SHDISP_TRACE("in");

    shdisp_bdic_API_IRQ_det_irq_ctrl(false);
    disable_irq(shdisp_andy_irq_gpio);
    for (i = 0; i < 2; i++) {
        ret = shdisp_andy_API_diag_write_reg(reg_data[i].addr, &reg_data[i].data, reg_data[i].size);
        if (reg_data[i].wait) {
            shdisp_SYS_API_delay_us(reg_data[i].wait);
        }
    }

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_andy_API_diag_write_reg                                            */
/* ------------------------------------------------------------------------- */
static int shdisp_andy_API_diag_write_reg(unsigned char addr, unsigned char *write_data, unsigned char size)
{
    int ret = 0;
    char dtype;

    SHDISP_TRACE("in");

    if (size == 0) {
        dtype = SHDISP_DTYPE_DCS_WRITE;
    } else if (size == 1) {
        dtype = SHDISP_DTYPE_DCS_WRITE1;
    } else {
        dtype = SHDISP_DTYPE_DCS_LWRITE;
    }

    ret = shdisp_panel_API_mipi_diag_write_reg(dtype, addr, write_data, size);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out dokick err ret=%d", ret);
        return ret;
    }

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_andy_API_diag_read_reg                                             */
/* ------------------------------------------------------------------------- */
static int shdisp_andy_API_diag_read_reg(unsigned char addr, unsigned char *read_data, unsigned char size)
{
    int ret = 0;

    SHDISP_TRACE("in");

    ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ, addr, read_data, size);
    if (ret) {
        SHDISP_DEBUG("out1");
        return SHDISP_RESULT_FAILURE;
    }

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_andy_API_diag_set_flicker_param                                    */
/* ------------------------------------------------------------------------- */
static int shdisp_andy_API_diag_set_flicker_param(struct shdisp_diag_flicker_param flicker_param)
{
#ifndef SHDISP_NOT_SUPPORT_FLICKER
    int ret = 0;

    SHDISP_TRACE("in");

    ret = shdisp_andy_diag_set_flicker_param_internal(flicker_param);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_andy_diag_set_flicker_param_internal.");
        return SHDISP_RESULT_FAILURE;
    } else {
        shdisp_andy_diag_set_flicker_param_ctx(flicker_param);
    }

    SHDISP_TRACE("out");
#endif /* SHDISP_NOT_SUPPORT_FLICKER */
    return SHDISP_RESULT_SUCCESS;
}

#ifndef SHDISP_NOT_SUPPORT_FLICKER
/* ------------------------------------------------------------------------- */
/* shdisp_andy_diag_set_flicker_param_internal                               */
/* ------------------------------------------------------------------------- */
static int shdisp_andy_diag_set_flicker_param_internal(struct shdisp_diag_flicker_param flicker_param)
{
    int vcom = flicker_param.master_alpha;
    int vcom_low = flicker_param.master_alpha;
    int i;
    int ret = 0;
    unsigned char andy_rdata_tmp[8];
    struct shdisp_andy_calc_vcom_in in;
    struct shdisp_andy_calc_vcom_out out;

    SHDISP_TRACE("in");

    if (flicker_param.request & SHDISP_REG_WRITE) {

        for (i = 1; i <= 7; i++) {
            andy_rdata[i] = 0;
            andy_rdata_tmp[i] = 0;
            andy_wdata[i] = 0;
        }

        shdisp_andy_set_switchcommand(1);

        in.vcom = vcom;
        in.vcom_low = vcom_low;

        ret = shdisp_andy_calc_vcom_param(in, &out, 0);
        if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_andy_calc_vcom_param.");
            return SHDISP_RESULT_FAILURE;
        }

        andy_rdata_tmp[0] = out.vcom1_l;
        andy_rdata_tmp[1] = out.vcom2_l;
        andy_rdata_tmp[2] = out.vcom12_h;
        andy_rdata_tmp[3] = out.lpvcom1;
        andy_rdata_tmp[4] = out.lpvcom2;
        andy_rdata_tmp[5] = out.vcomoff;

        for (i = 0; i < SHDISP_ANDY_VCOM_REG_NUM; i++) {
            andy_wdata[0] = andy_rdata_tmp[i];
            ret = shdisp_panel_API_mipi_diag_write_reg(SHDISP_DTYPE_DCS_WRITE1, Andy_VCOM_Reg[i], &andy_wdata[0], 1);
            if (ret) {
                SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_write_reg!!" );
                break;
            }
        }

        SHDISP_DEBUG("VCOM1_L=0x%02x VCOM2_L=0x%02x VCOM12_H=0x%02x",
                              andy_rdata_tmp[0], andy_rdata_tmp[1], andy_rdata_tmp[2]);
        SHDISP_DEBUG("LPVCOM1=0x%02x LPVCOM2=0x%02x VCOMOFF=0x%02x",
                              andy_rdata_tmp[3], andy_rdata_tmp[4], andy_rdata_tmp[5]);
        SHDISP_DEBUG("vcom=0x%04x", vcom);
        if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_DEBUG("out dokick err ret=%d", ret);
            return ret;
        }
    }

    if (flicker_param.request & (SHDISP_SAVE_VALUE | SHDISP_SAVE_VALUE_LOW)) {
        if (!(flicker_param.request & SHDISP_SAVE_VALUE)) {
            vcom = shdisp_panel_ctx.vcom;
        }
        if (!(flicker_param.request & SHDISP_SAVE_VALUE_LOW)) {
            vcom_low = shdisp_panel_ctx.vcom_low;
        }

        if (shdisp_andy_init_flicker_param(vcom, vcom_low)) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_andy_init_flicker_param.");
        }
    }
    if (flicker_param.request & SHDISP_RESET_VALUE) {
        if (shdisp_andy_init_flicker_param(flicker_param.master_alpha, flicker_param.master_alpha)) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_andy_init_flicker_param.");
        }
    }

    SHDISP_TRACE("out");

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_andy_diag_set_flicker_param_ctx                                    */
/* ------------------------------------------------------------------------- */
static int shdisp_andy_diag_set_flicker_param_ctx(struct shdisp_diag_flicker_param flicker_param)
{
    SHDISP_TRACE("in");

    if (flicker_param.request & SHDISP_SAVE_VALUE) {
        shdisp_panel_ctx.vcom = flicker_param.master_alpha;
        shdisp_panel_ctx.vcom_nvram = 0x9000 | flicker_param.master_alpha;
    }
    if (flicker_param.request & SHDISP_SAVE_VALUE_LOW) {
        shdisp_panel_ctx.vcom_low = flicker_param.master_alpha;
    }
    if (flicker_param.request & SHDISP_RESET_VALUE) {
        shdisp_panel_ctx.vcom = 0;
        shdisp_panel_ctx.vcom_low = 0;
        shdisp_panel_ctx.vcom_nvram = 0;
    }

    SHDISP_TRACE("out");

    return SHDISP_RESULT_SUCCESS;
}
#endif /* SHDISP_NOT_SUPPORT_FLICKER */

/* ------------------------------------------------------------------------- */
/* shdisp_andy_API_diag_get_flicker_param                                    */
/* ------------------------------------------------------------------------- */
static int shdisp_andy_API_diag_get_flicker_param(struct shdisp_diag_flicker_param *flicker_param)
{
    int ret = SHDISP_RESULT_SUCCESS;
#ifndef SHDISP_NOT_SUPPORT_FLICKER
    int i;
    unsigned char andy_rdata_tmp[8];

    SHDISP_TRACE("in");

    if (flicker_param == NULL) {
        SHDISP_ERR("<NULL_POINTER> flicker_param.");
        return SHDISP_RESULT_FAILURE;
    }

    for (i = 1; i <= 7; i++) {
        andy_rdata[i] = 0;
        andy_rdata_tmp[i] = 0;
    }

    shdisp_andy_set_switchcommand(1);

    for (i = 0; i < 3; i++) {
        if (i != 1) {
            ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ, Andy_VCOM_Reg[i], andy_rdata, 1);
            if (ret) {
                SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_read_reg addr=0x%02x data=0x%02x", Andy_VCOM_Reg[i],
                                                                                                  andy_rdata[0]);
            }
            andy_rdata_tmp[i] = andy_rdata[0];
        }
    }

    flicker_param->master_alpha = ((andy_rdata_tmp[2] & 0x01) << 8) | andy_rdata_tmp[0];

    SHDISP_TRACE("out master_alpha=0x%04X", flicker_param->master_alpha);
#endif /* SHDISP_NOT_SUPPORT_FLICKER */
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_andy_API_diag_get_flicker_low_param                                */
/* ------------------------------------------------------------------------- */
static int shdisp_andy_API_diag_get_flicker_low_param(struct shdisp_diag_flicker_param *flicker_param)
{
    int ret = SHDISP_RESULT_SUCCESS;
#ifndef SHDISP_NOT_SUPPORT_FLICKER
    int i;
    unsigned char andy_rdata_tmp[8];
    unsigned short tmp_vcom;

    SHDISP_TRACE("in");

    if (flicker_param == NULL) {
        SHDISP_ERR("<NULL_POINTER> flicker_param.");
        return SHDISP_RESULT_FAILURE;
    }

    for (i = 1; i <= 7; i++) {
        andy_rdata[i] = 0;
        andy_rdata_tmp[i] = 0;
    }

    shdisp_andy_set_switchcommand(1);

    for (i = 0; i < 4; i++) {
        if (i != 1) {
            ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ, Andy_VCOM_Reg[i], andy_rdata, 1);
            if (ret) {
                SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_read_reg addr=0x%02x data=0x%02x", Andy_VCOM_Reg[i],
                                                                                                  andy_rdata[0]);
            }
            andy_rdata_tmp[i] = andy_rdata[0];
        }
    }

    tmp_vcom = ((andy_rdata_tmp[2] & 0x01) << 8) | andy_rdata_tmp[0];
    if (andy_rdata_tmp[3] & 0x10) {
        flicker_param->master_alpha = tmp_vcom - (andy_rdata_tmp[3] & 0x0f) - 1;
    } else {
        flicker_param->master_alpha = tmp_vcom + (andy_rdata_tmp[3] & 0x0f);
    }

    SHDISP_TRACE("out master_alpha=0x%04X", flicker_param->master_alpha);
#endif /* SHDISP_NOT_SUPPORT_FLICKER */
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_andy_API_check_recovery                                            */
/* ------------------------------------------------------------------------- */
static int shdisp_andy_API_check_recovery(void)
{
    int ret1 = SHDISP_RESULT_SUCCESS, ret2;
#ifdef SHDISP_RESET_LOG
    struct shdisp_dbg_error_code err_code;
#endif  /* SHDISP_RESET_LOG */


    SHDISP_TRACE("in");

#ifdef SHDISP_POWER_MODE_CHK
    SHDISP_DEBUG("shdisp_andy_dispon_chk=%d", shdisp_andy_dispon_chk);

    switch (shdisp_andy_dispon_chk) {
    case SHDISP_PANEL_DISPON_CHK_INIT:
        ret1 = shdisp_andy_power_mode_chk(0x0A);
        if (ret1 != SHDISP_RESULT_SUCCESS) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_andy_power_mode_chk.");
        }
        break;
    case SHDISP_PANEL_DISPON_CHK_NG:
        SHDISP_ERR("<RESULT_FAILURE> shdisp_andy_power_mode_chk.");
        shdisp_dbg_API_set_subcode(SHDISP_DBG_SUBCODE_DISPON_CHK);
        ret1 = SHDISP_RESULT_FAILURE;
    default:
        break;
    }
#endif /* SHDISP_POWER_MODE_CHK */

    ret2 = shdisp_bdic_API_RECOVERY_check_restoration();

#if defined(CONFIG_ANDROID_ENGINEERING)
    if (shdisp_dbg_API_get_recovery_check_error() == SHDISP_DBG_RECOVERY_ERROR_DETLOW) {
        shdisp_dbg_API_update_recovery_check_error(SHDISP_DBG_RECOVERY_ERROR_NONE);
        SHDISP_DEBUG("force lcd det low.");
        ret2 = SHDISP_RESULT_FAILURE;
    }
#endif /* CONFIG_ANDROID_ENGINEERING */

    if (ret2 != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_bdic_API_RECOVERY_check_restoration.");
#ifdef SHDISP_RESET_LOG
        err_code.mode = SHDISP_DBG_MODE_LINUX;
        err_code.type = SHDISP_DBG_TYPE_PANEL;
        err_code.code = SHDISP_DBG_CODE_ERROR_DETECT;
        err_code.subcode = SHDISP_DBG_SUBCODE_DET_LOW;
        shdisp_dbg_API_err_output(&err_code, 0);
        shdisp_dbg_API_set_subcode(SHDISP_DBG_SUBCODE_DET_LOW);
#endif /* SHDISP_RESET_LOG */
    }

    if ((ret1 != SHDISP_RESULT_SUCCESS) || (ret2 != SHDISP_RESULT_SUCCESS)) {
        return SHDISP_RESULT_FAILURE;
    }
    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_andy_diag_set_gmmtable_and_voltage                                 */
/* ------------------------------------------------------------------------- */
static int shdisp_andy_diag_set_gmmtable_and_voltage(struct shdisp_diag_gamma_info *gmm_info,
                                                       int set_applied_voltage)
{
    int i, j = 0;
    int ret = SHDISP_RESULT_SUCCESS;
    unsigned char andy_gmm_wdata[372];
    unsigned char andy_gmm_addr[372] = {
        mipi_sh_andy_cmd_SwitchCommand[1][0],
        mipi_sh_andy_cmd_RDPSGMM[0][0],
        mipi_sh_andy_cmd_RDPSGMM[1][0],
        mipi_sh_andy_cmd_RDPSGMM[2][0],
        mipi_sh_andy_cmd_RDPSGMM[3][0],
        mipi_sh_andy_cmd_RDPSGMM[4][0],
        mipi_sh_andy_cmd_RDPSGMM[5][0],
        mipi_sh_andy_cmd_RDPSGMM[6][0],
        mipi_sh_andy_cmd_RDPSGMM[7][0],
        mipi_sh_andy_cmd_RDPSGMM[8][0],
        mipi_sh_andy_cmd_RDPSGMM[9][0],
        mipi_sh_andy_cmd_RDPSGMM[10][0],
        mipi_sh_andy_cmd_RDPSGMM[11][0],
        mipi_sh_andy_cmd_RDPSGMM[12][0],
        mipi_sh_andy_cmd_RDPSGMM[13][0],
        mipi_sh_andy_cmd_RDPSGMM[14][0],
        mipi_sh_andy_cmd_RDPSGMM[15][0],
        mipi_sh_andy_cmd_RDPSGMM[16][0],
        mipi_sh_andy_cmd_RDPSGMM[17][0],
        mipi_sh_andy_cmd_RDPSGMM[18][0],
        mipi_sh_andy_cmd_RDPSGMM[19][0],
        mipi_sh_andy_cmd_RDPSGMM[20][0],
        mipi_sh_andy_cmd_RDPSGMM[21][0],
        mipi_sh_andy_cmd_RDPSGMM[22][0],
        mipi_sh_andy_cmd_RDPSGMM[23][0],
        mipi_sh_andy_cmd_RDPSGMM[24][0],
        mipi_sh_andy_cmd_RDPSGMM[25][0],
        mipi_sh_andy_cmd_RDPSGMM[26][0],
        mipi_sh_andy_cmd_RDPSGMM[27][0],
        mipi_sh_andy_cmd_RDPSGMM[28][0],
        mipi_sh_andy_cmd_RDPSGMM[29][0],
        mipi_sh_andy_cmd_RDPSGMM[30][0],
        mipi_sh_andy_cmd_RDPSGMM[31][0],
        mipi_sh_andy_cmd_RDPSGMM[32][0],
        mipi_sh_andy_cmd_RDPSGMM[33][0],
        mipi_sh_andy_cmd_RDPSGMM[34][0],
        mipi_sh_andy_cmd_RDPSGMM[35][0],
        mipi_sh_andy_cmd_RDPSGMM[36][0],
        mipi_sh_andy_cmd_RDPSGMM[37][0],
        mipi_sh_andy_cmd_RDPSGMM[38][0],
        mipi_sh_andy_cmd_RDPSGMM[39][0],
        mipi_sh_andy_cmd_RDPSGMM[40][0],
        mipi_sh_andy_cmd_RDPSGMM[41][0],
        mipi_sh_andy_cmd_RDPSGMM[42][0],
        mipi_sh_andy_cmd_RDPSGMM[43][0],
        mipi_sh_andy_cmd_RDPSGMM[44][0],
        mipi_sh_andy_cmd_RDPSGMM[45][0],
        mipi_sh_andy_cmd_RDPSGMM[46][0],
        mipi_sh_andy_cmd_RDPSGMM[47][0],
        mipi_sh_andy_cmd_RDPSGMM[48][0],
        mipi_sh_andy_cmd_RDPSGMM[49][0],
        mipi_sh_andy_cmd_RDPSGMM[50][0],
        mipi_sh_andy_cmd_RDPSGMM[51][0],
        mipi_sh_andy_cmd_RDPSGMM[52][0],
        mipi_sh_andy_cmd_RDPSGMM[53][0],
        mipi_sh_andy_cmd_RDPSGMM[54][0],
        mipi_sh_andy_cmd_RDPSGMM[55][0],
        mipi_sh_andy_cmd_RDPSGMM[56][0],
        mipi_sh_andy_cmd_RDPSGMM[57][0],
        mipi_sh_andy_cmd_RDPSGMM[58][0],
        mipi_sh_andy_cmd_RDPSGMM[59][0],
        mipi_sh_andy_cmd_RDNGGMM[0][0],
        mipi_sh_andy_cmd_RDNGGMM[1][0],
        mipi_sh_andy_cmd_RDNGGMM[2][0],
        mipi_sh_andy_cmd_RDNGGMM[3][0],
        mipi_sh_andy_cmd_RDNGGMM[4][0],
        mipi_sh_andy_cmd_RDNGGMM[5][0],
        mipi_sh_andy_cmd_RDNGGMM[6][0],
        mipi_sh_andy_cmd_RDNGGMM[7][0],
        mipi_sh_andy_cmd_RDNGGMM[8][0],
        mipi_sh_andy_cmd_RDNGGMM[9][0],
        mipi_sh_andy_cmd_RDNGGMM[10][0],
        mipi_sh_andy_cmd_RDNGGMM[11][0],
        mipi_sh_andy_cmd_RDNGGMM[12][0],
        mipi_sh_andy_cmd_RDNGGMM[13][0],
        mipi_sh_andy_cmd_RDNGGMM[14][0],
        mipi_sh_andy_cmd_RDNGGMM[15][0],
        mipi_sh_andy_cmd_RDNGGMM[16][0],
        mipi_sh_andy_cmd_RDNGGMM[17][0],
        mipi_sh_andy_cmd_RDNGGMM[18][0],
        mipi_sh_andy_cmd_RDNGGMM[19][0],
        mipi_sh_andy_cmd_RDNGGMM[20][0],
        mipi_sh_andy_cmd_RDNGGMM[21][0],
        mipi_sh_andy_cmd_RDNGGMM[22][0],
        mipi_sh_andy_cmd_RDNGGMM[23][0],
        mipi_sh_andy_cmd_RDNGGMM[24][0],
        mipi_sh_andy_cmd_RDNGGMM[25][0],
        mipi_sh_andy_cmd_RDNGGMM[26][0],
        mipi_sh_andy_cmd_RDNGGMM[27][0],
        mipi_sh_andy_cmd_RDNGGMM[28][0],
        mipi_sh_andy_cmd_RDNGGMM[29][0],
        mipi_sh_andy_cmd_RDNGGMM[30][0],
        mipi_sh_andy_cmd_RDNGGMM[31][0],
        mipi_sh_andy_cmd_RDNGGMM[32][0],
        mipi_sh_andy_cmd_RDNGGMM[33][0],
        mipi_sh_andy_cmd_RDNGGMM[34][0],
        mipi_sh_andy_cmd_RDNGGMM[35][0],
        mipi_sh_andy_cmd_RDNGGMM[36][0],
        mipi_sh_andy_cmd_RDNGGMM[37][0],
        mipi_sh_andy_cmd_RDNGGMM[38][0],
        mipi_sh_andy_cmd_RDNGGMM[39][0],
        mipi_sh_andy_cmd_RDNGGMM[40][0],
        mipi_sh_andy_cmd_RDNGGMM[41][0],
        mipi_sh_andy_cmd_RDNGGMM[42][0],
        mipi_sh_andy_cmd_RDNGGMM[43][0],
        mipi_sh_andy_cmd_RDNGGMM[44][0],
        mipi_sh_andy_cmd_RDNGGMM[45][0],
        mipi_sh_andy_cmd_RDNGGMM[46][0],
        mipi_sh_andy_cmd_RDNGGMM[47][0],
        mipi_sh_andy_cmd_RDNGGMM[48][0],
        mipi_sh_andy_cmd_RDNGGMM[49][0],
        mipi_sh_andy_cmd_RDNGGMM[50][0],
        mipi_sh_andy_cmd_RDNGGMM[51][0],
        mipi_sh_andy_cmd_RDNGGMM[52][0],
        mipi_sh_andy_cmd_RDNGGMM[53][0],
        mipi_sh_andy_cmd_RDNGGMM[54][0],
        mipi_sh_andy_cmd_RDNGGMM[55][0],
        mipi_sh_andy_cmd_RDNGGMM[56][0],
        mipi_sh_andy_cmd_RDNGGMM[57][0],
        mipi_sh_andy_cmd_RDNGGMM[58][0],
        mipi_sh_andy_cmd_RDNGGMM[59][0],
        mipi_sh_andy_cmd_GRPSGMM[0][0],
        mipi_sh_andy_cmd_GRPSGMM[1][0],
        mipi_sh_andy_cmd_GRPSGMM[2][0],
        mipi_sh_andy_cmd_GRPSGMM[3][0],
        mipi_sh_andy_cmd_GRPSGMM[4][0],
        mipi_sh_andy_cmd_GRPSGMM[5][0],
        mipi_sh_andy_cmd_GRPSGMM[6][0],
        mipi_sh_andy_cmd_GRPSGMM[7][0],
        mipi_sh_andy_cmd_GRPSGMM[8][0],
        mipi_sh_andy_cmd_GRPSGMM[9][0],
        mipi_sh_andy_cmd_GRPSGMM[10][0],
        mipi_sh_andy_cmd_GRPSGMM[11][0],
        mipi_sh_andy_cmd_SwitchCommand[2][0],
        mipi_sh_andy_cmd_GRPSGMM[12][0],
        mipi_sh_andy_cmd_GRPSGMM[13][0],
        mipi_sh_andy_cmd_GRPSGMM[14][0],
        mipi_sh_andy_cmd_GRPSGMM[15][0],
        mipi_sh_andy_cmd_GRPSGMM[16][0],
        mipi_sh_andy_cmd_GRPSGMM[17][0],
        mipi_sh_andy_cmd_GRPSGMM[18][0],
        mipi_sh_andy_cmd_GRPSGMM[19][0],
        mipi_sh_andy_cmd_GRPSGMM[20][0],
        mipi_sh_andy_cmd_GRPSGMM[21][0],
        mipi_sh_andy_cmd_GRPSGMM[22][0],
        mipi_sh_andy_cmd_GRPSGMM[23][0],
        mipi_sh_andy_cmd_GRPSGMM[24][0],
        mipi_sh_andy_cmd_GRPSGMM[25][0],
        mipi_sh_andy_cmd_GRPSGMM[26][0],
        mipi_sh_andy_cmd_GRPSGMM[27][0],
        mipi_sh_andy_cmd_GRPSGMM[28][0],
        mipi_sh_andy_cmd_GRPSGMM[29][0],
        mipi_sh_andy_cmd_GRPSGMM[30][0],
        mipi_sh_andy_cmd_GRPSGMM[31][0],
        mipi_sh_andy_cmd_GRPSGMM[32][0],
        mipi_sh_andy_cmd_GRPSGMM[33][0],
        mipi_sh_andy_cmd_GRPSGMM[34][0],
        mipi_sh_andy_cmd_GRPSGMM[35][0],
        mipi_sh_andy_cmd_GRPSGMM[36][0],
        mipi_sh_andy_cmd_GRPSGMM[37][0],
        mipi_sh_andy_cmd_GRPSGMM[38][0],
        mipi_sh_andy_cmd_GRPSGMM[39][0],
        mipi_sh_andy_cmd_GRPSGMM[40][0],
        mipi_sh_andy_cmd_GRPSGMM[41][0],
        mipi_sh_andy_cmd_GRPSGMM[42][0],
        mipi_sh_andy_cmd_GRPSGMM[43][0],
        mipi_sh_andy_cmd_GRPSGMM[44][0],
        mipi_sh_andy_cmd_GRPSGMM[45][0],
        mipi_sh_andy_cmd_GRPSGMM[46][0],
        mipi_sh_andy_cmd_GRPSGMM[47][0],
        mipi_sh_andy_cmd_GRPSGMM[48][0],
        mipi_sh_andy_cmd_GRPSGMM[49][0],
        mipi_sh_andy_cmd_GRPSGMM[50][0],
        mipi_sh_andy_cmd_GRPSGMM[51][0],
        mipi_sh_andy_cmd_GRPSGMM[52][0],
        mipi_sh_andy_cmd_GRPSGMM[53][0],
        mipi_sh_andy_cmd_GRPSGMM[54][0],
        mipi_sh_andy_cmd_GRPSGMM[55][0],
        mipi_sh_andy_cmd_GRPSGMM[56][0],
        mipi_sh_andy_cmd_GRPSGMM[57][0],
        mipi_sh_andy_cmd_GRPSGMM[58][0],
        mipi_sh_andy_cmd_GRPSGMM[59][0],
        mipi_sh_andy_cmd_GRNGGMM[0][0],
        mipi_sh_andy_cmd_GRNGGMM[1][0],
        mipi_sh_andy_cmd_GRNGGMM[2][0],
        mipi_sh_andy_cmd_GRNGGMM[3][0],
        mipi_sh_andy_cmd_GRNGGMM[4][0],
        mipi_sh_andy_cmd_GRNGGMM[5][0],
        mipi_sh_andy_cmd_GRNGGMM[6][0],
        mipi_sh_andy_cmd_GRNGGMM[7][0],
        mipi_sh_andy_cmd_GRNGGMM[8][0],
        mipi_sh_andy_cmd_GRNGGMM[9][0],
        mipi_sh_andy_cmd_GRNGGMM[10][0],
        mipi_sh_andy_cmd_GRNGGMM[11][0],
        mipi_sh_andy_cmd_GRNGGMM[12][0],
        mipi_sh_andy_cmd_GRNGGMM[13][0],
        mipi_sh_andy_cmd_GRNGGMM[14][0],
        mipi_sh_andy_cmd_GRNGGMM[15][0],
        mipi_sh_andy_cmd_GRNGGMM[16][0],
        mipi_sh_andy_cmd_GRNGGMM[17][0],
        mipi_sh_andy_cmd_GRNGGMM[18][0],
        mipi_sh_andy_cmd_GRNGGMM[19][0],
        mipi_sh_andy_cmd_GRNGGMM[20][0],
        mipi_sh_andy_cmd_GRNGGMM[21][0],
        mipi_sh_andy_cmd_GRNGGMM[22][0],
        mipi_sh_andy_cmd_GRNGGMM[23][0],
        mipi_sh_andy_cmd_GRNGGMM[24][0],
        mipi_sh_andy_cmd_GRNGGMM[25][0],
        mipi_sh_andy_cmd_GRNGGMM[26][0],
        mipi_sh_andy_cmd_GRNGGMM[27][0],
        mipi_sh_andy_cmd_GRNGGMM[28][0],
        mipi_sh_andy_cmd_GRNGGMM[29][0],
        mipi_sh_andy_cmd_GRNGGMM[30][0],
        mipi_sh_andy_cmd_GRNGGMM[31][0],
        mipi_sh_andy_cmd_GRNGGMM[32][0],
        mipi_sh_andy_cmd_GRNGGMM[33][0],
        mipi_sh_andy_cmd_GRNGGMM[34][0],
        mipi_sh_andy_cmd_GRNGGMM[35][0],
        mipi_sh_andy_cmd_GRNGGMM[36][0],
        mipi_sh_andy_cmd_GRNGGMM[37][0],
        mipi_sh_andy_cmd_GRNGGMM[38][0],
        mipi_sh_andy_cmd_GRNGGMM[39][0],
        mipi_sh_andy_cmd_GRNGGMM[40][0],
        mipi_sh_andy_cmd_GRNGGMM[41][0],
        mipi_sh_andy_cmd_GRNGGMM[42][0],
        mipi_sh_andy_cmd_GRNGGMM[43][0],
        mipi_sh_andy_cmd_GRNGGMM[44][0],
        mipi_sh_andy_cmd_GRNGGMM[45][0],
        mipi_sh_andy_cmd_GRNGGMM[46][0],
        mipi_sh_andy_cmd_GRNGGMM[47][0],
        mipi_sh_andy_cmd_GRNGGMM[48][0],
        mipi_sh_andy_cmd_GRNGGMM[49][0],
        mipi_sh_andy_cmd_GRNGGMM[50][0],
        mipi_sh_andy_cmd_GRNGGMM[51][0],
        mipi_sh_andy_cmd_GRNGGMM[52][0],
        mipi_sh_andy_cmd_GRNGGMM[53][0],
        mipi_sh_andy_cmd_GRNGGMM[54][0],
        mipi_sh_andy_cmd_GRNGGMM[55][0],
        mipi_sh_andy_cmd_GRNGGMM[56][0],
        mipi_sh_andy_cmd_GRNGGMM[57][0],
        mipi_sh_andy_cmd_GRNGGMM[58][0],
        mipi_sh_andy_cmd_GRNGGMM[59][0],
        mipi_sh_andy_cmd_BLPSGMM[0][0],
        mipi_sh_andy_cmd_BLPSGMM[1][0],
        mipi_sh_andy_cmd_BLPSGMM[2][0],
        mipi_sh_andy_cmd_BLPSGMM[3][0],
        mipi_sh_andy_cmd_BLPSGMM[4][0],
        mipi_sh_andy_cmd_BLPSGMM[5][0],
        mipi_sh_andy_cmd_BLPSGMM[6][0],
        mipi_sh_andy_cmd_BLPSGMM[7][0],
        mipi_sh_andy_cmd_BLPSGMM[8][0],
        mipi_sh_andy_cmd_BLPSGMM[9][0],
        mipi_sh_andy_cmd_BLPSGMM[10][0],
        mipi_sh_andy_cmd_BLPSGMM[11][0],
        mipi_sh_andy_cmd_BLPSGMM[12][0],
        mipi_sh_andy_cmd_BLPSGMM[13][0],
        mipi_sh_andy_cmd_BLPSGMM[14][0],
        mipi_sh_andy_cmd_BLPSGMM[15][0],
        mipi_sh_andy_cmd_BLPSGMM[16][0],
        mipi_sh_andy_cmd_BLPSGMM[17][0],
        mipi_sh_andy_cmd_BLPSGMM[18][0],
        mipi_sh_andy_cmd_BLPSGMM[19][0],
        mipi_sh_andy_cmd_BLPSGMM[20][0],
        mipi_sh_andy_cmd_BLPSGMM[21][0],
        mipi_sh_andy_cmd_BLPSGMM[22][0],
        mipi_sh_andy_cmd_BLPSGMM[23][0],
        mipi_sh_andy_cmd_BLPSGMM[24][0],
        mipi_sh_andy_cmd_BLPSGMM[25][0],
        mipi_sh_andy_cmd_BLPSGMM[26][0],
        mipi_sh_andy_cmd_BLPSGMM[27][0],
        mipi_sh_andy_cmd_BLPSGMM[28][0],
        mipi_sh_andy_cmd_BLPSGMM[29][0],
        mipi_sh_andy_cmd_BLPSGMM[30][0],
        mipi_sh_andy_cmd_BLPSGMM[31][0],
        mipi_sh_andy_cmd_BLPSGMM[32][0],
        mipi_sh_andy_cmd_BLPSGMM[33][0],
        mipi_sh_andy_cmd_BLPSGMM[34][0],
        mipi_sh_andy_cmd_BLPSGMM[35][0],
        mipi_sh_andy_cmd_BLPSGMM[36][0],
        mipi_sh_andy_cmd_BLPSGMM[37][0],
        mipi_sh_andy_cmd_BLPSGMM[38][0],
        mipi_sh_andy_cmd_BLPSGMM[39][0],
        mipi_sh_andy_cmd_BLPSGMM[40][0],
        mipi_sh_andy_cmd_BLPSGMM[41][0],
        mipi_sh_andy_cmd_BLPSGMM[42][0],
        mipi_sh_andy_cmd_BLPSGMM[43][0],
        mipi_sh_andy_cmd_BLPSGMM[44][0],
        mipi_sh_andy_cmd_BLPSGMM[45][0],
        mipi_sh_andy_cmd_BLPSGMM[46][0],
        mipi_sh_andy_cmd_BLPSGMM[47][0],
        mipi_sh_andy_cmd_BLPSGMM[48][0],
        mipi_sh_andy_cmd_BLPSGMM[49][0],
        mipi_sh_andy_cmd_BLPSGMM[50][0],
        mipi_sh_andy_cmd_BLPSGMM[51][0],
        mipi_sh_andy_cmd_BLPSGMM[52][0],
        mipi_sh_andy_cmd_BLPSGMM[53][0],
        mipi_sh_andy_cmd_BLPSGMM[54][0],
        mipi_sh_andy_cmd_BLPSGMM[55][0],
        mipi_sh_andy_cmd_BLPSGMM[56][0],
        mipi_sh_andy_cmd_BLPSGMM[57][0],
        mipi_sh_andy_cmd_BLPSGMM[58][0],
        mipi_sh_andy_cmd_BLPSGMM[59][0],
        mipi_sh_andy_cmd_BLNGGMM[0][0],
        mipi_sh_andy_cmd_BLNGGMM[1][0],
        mipi_sh_andy_cmd_BLNGGMM[2][0],
        mipi_sh_andy_cmd_BLNGGMM[3][0],
        mipi_sh_andy_cmd_BLNGGMM[4][0],
        mipi_sh_andy_cmd_BLNGGMM[5][0],
        mipi_sh_andy_cmd_BLNGGMM[6][0],
        mipi_sh_andy_cmd_BLNGGMM[7][0],
        mipi_sh_andy_cmd_BLNGGMM[8][0],
        mipi_sh_andy_cmd_BLNGGMM[9][0],
        mipi_sh_andy_cmd_BLNGGMM[10][0],
        mipi_sh_andy_cmd_BLNGGMM[11][0],
        mipi_sh_andy_cmd_BLNGGMM[12][0],
        mipi_sh_andy_cmd_BLNGGMM[13][0],
        mipi_sh_andy_cmd_BLNGGMM[14][0],
        mipi_sh_andy_cmd_BLNGGMM[15][0],
        mipi_sh_andy_cmd_BLNGGMM[16][0],
        mipi_sh_andy_cmd_BLNGGMM[17][0],
        mipi_sh_andy_cmd_BLNGGMM[18][0],
        mipi_sh_andy_cmd_BLNGGMM[19][0],
        mipi_sh_andy_cmd_BLNGGMM[20][0],
        mipi_sh_andy_cmd_BLNGGMM[21][0],
        mipi_sh_andy_cmd_BLNGGMM[22][0],
        mipi_sh_andy_cmd_BLNGGMM[23][0],
        mipi_sh_andy_cmd_BLNGGMM[24][0],
        mipi_sh_andy_cmd_BLNGGMM[25][0],
        mipi_sh_andy_cmd_BLNGGMM[26][0],
        mipi_sh_andy_cmd_BLNGGMM[27][0],
        mipi_sh_andy_cmd_BLNGGMM[28][0],
        mipi_sh_andy_cmd_BLNGGMM[29][0],
        mipi_sh_andy_cmd_BLNGGMM[30][0],
        mipi_sh_andy_cmd_BLNGGMM[31][0],
        mipi_sh_andy_cmd_BLNGGMM[32][0],
        mipi_sh_andy_cmd_BLNGGMM[33][0],
        mipi_sh_andy_cmd_BLNGGMM[34][0],
        mipi_sh_andy_cmd_BLNGGMM[35][0],
        mipi_sh_andy_cmd_BLNGGMM[36][0],
        mipi_sh_andy_cmd_BLNGGMM[37][0],
        mipi_sh_andy_cmd_BLNGGMM[38][0],
        mipi_sh_andy_cmd_BLNGGMM[39][0],
        mipi_sh_andy_cmd_BLNGGMM[40][0],
        mipi_sh_andy_cmd_BLNGGMM[41][0],
        mipi_sh_andy_cmd_BLNGGMM[42][0],
        mipi_sh_andy_cmd_BLNGGMM[43][0],
        mipi_sh_andy_cmd_BLNGGMM[44][0],
        mipi_sh_andy_cmd_BLNGGMM[45][0],
        mipi_sh_andy_cmd_BLNGGMM[46][0],
        mipi_sh_andy_cmd_BLNGGMM[47][0],
        mipi_sh_andy_cmd_BLNGGMM[48][0],
        mipi_sh_andy_cmd_BLNGGMM[49][0],
        mipi_sh_andy_cmd_BLNGGMM[50][0],
        mipi_sh_andy_cmd_BLNGGMM[51][0],
        mipi_sh_andy_cmd_BLNGGMM[52][0],
        mipi_sh_andy_cmd_BLNGGMM[53][0],
        mipi_sh_andy_cmd_BLNGGMM[54][0],
        mipi_sh_andy_cmd_BLNGGMM[55][0],
        mipi_sh_andy_cmd_BLNGGMM[56][0],
        mipi_sh_andy_cmd_BLNGGMM[57][0],
        mipi_sh_andy_cmd_BLNGGMM[58][0],
        mipi_sh_andy_cmd_BLNGGMM[59][0],
        mipi_sh_andy_cmd_SwitchCommand[1][0],
        mipi_sh_andy_cmd_RegulatorPumpSetting[SHDISP_ANDY_VGH][0],
        mipi_sh_andy_cmd_RegulatorPumpSetting[SHDISP_ANDY_VGL][0],
        mipi_sh_andy_cmd_RegulatorPumpSetting[SHDISP_ANDY_GVDDP][0],
        mipi_sh_andy_cmd_RegulatorPumpSetting[SHDISP_ANDY_GVDDN][0],
        mipi_sh_andy_cmd_RegulatorPumpSetting[SHDISP_ANDY_GVDDP2][0],
        mipi_sh_andy_cmd_RegulatorPumpSetting[SHDISP_ANDY_VGHO][0],
        mipi_sh_andy_cmd_RegulatorPumpSetting[SHDISP_ANDY_VGLO][0],
        mipi_sh_andy_cmd_RegulatorPumpSetting[SHDISP_ANDY_AVDDR][0],
        mipi_sh_andy_cmd_RegulatorPumpSetting[SHDISP_ANDY_AVEER][0],
    };

    SHDISP_TRACE("in");

    for (i = 0; i < SHDISP_ANDY_GMM_SETTING_SIZE; i++) {
        if (i == 0) {
            andy_gmm_wdata[j++] = mipi_sh_andy_cmd_SwitchCommand[1][1];
        }
        andy_gmm_wdata[j++] = ((gmm_info->gammaR[i] >> 8) & 0x0003);
        andy_gmm_wdata[j++] = (gmm_info->gammaR[i] & 0x00FF);
    }

    for (i = 0; i < SHDISP_ANDY_GMM_SETTING_SIZE; i++) {
        if (i == 6) {
            andy_gmm_wdata[j++] = mipi_sh_andy_cmd_SwitchCommand[2][1];
        }
        andy_gmm_wdata[j++] = ((gmm_info->gammaG[i] >> 8) & 0x0003);
        andy_gmm_wdata[j++] = (gmm_info->gammaG[i] & 0x00FF);
    }

    for (i = 0; i < SHDISP_ANDY_GMM_SETTING_SIZE; i++) {
        andy_gmm_wdata[j++] = ((gmm_info->gammaB[i] >> 8) & 0x0003);
        andy_gmm_wdata[j++] = (gmm_info->gammaB[i] & 0x00FF);
    }

    if (!set_applied_voltage) {
        for (i = 0; i < j; i++) {
            ret = shdisp_panel_API_mipi_diag_write_reg(SHDISP_DTYPE_DCS_WRITE1, andy_gmm_addr[i], &andy_gmm_wdata[i], 1);
            if (ret) {
                SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_mipi_diag_write_reg!!" );
                goto shdisp_end;
            }
        }
        goto shdisp_end;
    }

    andy_gmm_wdata[j++] = mipi_sh_andy_cmd_SwitchCommand[1][1];
    andy_gmm_wdata[j++] = gmm_info->vgh;
    andy_gmm_wdata[j++] = gmm_info->vgl;
    andy_gmm_wdata[j++] = gmm_info->gvddp;
    andy_gmm_wdata[j++] = gmm_info->gvddn;
    andy_gmm_wdata[j++] = gmm_info->gvddp2;
    andy_gmm_wdata[j++] = gmm_info->vgho;
    andy_gmm_wdata[j++] = gmm_info->vglo;
    andy_gmm_wdata[j++] = gmm_info->avddr;
    andy_gmm_wdata[j++] = gmm_info->aveer;

    for (i = 0; i < j; i++) {
        ret = shdisp_panel_API_mipi_diag_write_reg(SHDISP_DTYPE_DCS_WRITE1, andy_gmm_addr[i], &andy_gmm_wdata[i], 1);
        if (ret) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_mipi_diag_write_reg!!" );
            goto shdisp_end;
        }
    }
    memcpy(&diag_tmp_gmm_info, gmm_info, sizeof(diag_tmp_gmm_info));
    diag_tmp_gmm_info_set = 1;

shdisp_end:
    ret = shdisp_panel_API_mipi_diag_write_reg(SHDISP_DTYPE_DCS_WRITE1,
                                               mipi_sh_andy_cmd_SwitchCommand[0][0],
                                               &mipi_sh_andy_cmd_SwitchCommand[0][1],
                                               1);

    if (ret) {
        return ret;
    }

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_andy_diag_get_gmmtable_and_voltage                                 */
/* ------------------------------------------------------------------------- */
static int shdisp_andy_diag_get_gmmtable_and_voltage(struct shdisp_diag_gamma_info *gmm_info,
                                                       int set_applied_voltage)
{
    int i, j;
    int ret = 0;
    unsigned char andy_rdata[1];
    unsigned short andy_temp_data[SHDISP_ANDY_GMM_SETTING_SIZE];

    SHDISP_TRACE("in");

    if (gmm_info == NULL) {
        SHDISP_ERR("<NULL_POINTER> gmm_info.");
        return SHDISP_RESULT_FAILURE;
    }

    memset(andy_temp_data, 0, sizeof(andy_temp_data));
    for (i = 0, j = 0; i < (SHDISP_ANDY_GMM_SETTING_SIZE / 2); i++) {
        if (i == 0) {
            ret = shdisp_panel_API_mipi_diag_write_reg(SHDISP_DTYPE_DCS_WRITE1,
                                                       mipi_sh_andy_cmd_SwitchCommand[1][0],
                                                       &mipi_sh_andy_cmd_SwitchCommand[1][1],
                                                       1);
            if (ret) {
                SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_mipi_diag_read_reg.");
                goto shdisp_end;
            }
        }
        memset(andy_rdata, 0, sizeof(andy_rdata));
        ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,
                                                  mipi_sh_andy_cmd_RDPSGMM[j++][0],
                                                  andy_rdata,
                                                  1);
        if (ret) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_mipi_diag_read_reg.");
            goto shdisp_end;
        }
        andy_temp_data[i] = ((andy_rdata[0] << 8) & 0x0300);
        memset(andy_rdata, 0, sizeof(andy_rdata));
        ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,
                                                  mipi_sh_andy_cmd_RDPSGMM[j++][0],
                                                  andy_rdata,
                                                  1);
        if (ret) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_mipi_diag_read_reg.");
            goto shdisp_end;
        }
        andy_temp_data[i] |= (andy_rdata[0] & 0x00FF);
    }

    for (i = SHDISP_ANDY_GMM_NEGATIVE_OFFSET, j = 0; i < SHDISP_ANDY_GMM_SETTING_SIZE; i++) {
        memset(andy_rdata, 0, sizeof(andy_rdata));
        ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,
                                                  mipi_sh_andy_cmd_RDNGGMM[j++][0],
                                                  andy_rdata,
                                                  1);
        if (ret) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_mipi_diag_read_reg.");
            goto shdisp_end;
        }
        andy_temp_data[i] = ((andy_rdata[0] << 8) & 0x0300);
        memset(andy_rdata, 0, sizeof(andy_rdata));
        ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,
                                                  mipi_sh_andy_cmd_RDNGGMM[j++][0],
                                                  andy_rdata,
                                                  1);
        if (ret) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_mipi_diag_read_reg.");
            goto shdisp_end;
        }
        andy_temp_data[i] |= (andy_rdata[0] & 0x00FF);
    }
    memcpy(gmm_info->gammaR, andy_temp_data, sizeof(andy_temp_data));

    memset(andy_temp_data, 0, sizeof(andy_temp_data));
    for (i = 0, j = 0; i < (SHDISP_ANDY_GMM_SETTING_SIZE / 2); i++) {
        if (i == 6) {
            ret = shdisp_panel_API_mipi_diag_write_reg(SHDISP_DTYPE_DCS_WRITE1,
                                                       mipi_sh_andy_cmd_SwitchCommand[2][0],
                                                       &mipi_sh_andy_cmd_SwitchCommand[2][1],
                                                       1);
            if (ret) {
                SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_write_reg.");
                goto shdisp_end;
            }
        }
        memset(andy_rdata, 0, sizeof(andy_rdata));
        ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,
                                                  mipi_sh_andy_cmd_GRPSGMM[j++][0],
                                                  andy_rdata,
                                                  1);
        if (ret) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_mipi_diag_read_reg.");
            goto shdisp_end;
        }
        andy_temp_data[i] = ((andy_rdata[0] << 8) & 0x0300);
        memset(andy_rdata, 0, sizeof(andy_rdata));
        ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,
                                                  mipi_sh_andy_cmd_GRPSGMM[j++][0],
                                                  andy_rdata,
                                                  1);
        if (ret) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_mipi_diag_read_reg.");
            goto shdisp_end;
        }
        andy_temp_data[i] |= (andy_rdata[0] & 0x00FF);
    }

    for (i = SHDISP_ANDY_GMM_NEGATIVE_OFFSET, j = 0; i < SHDISP_ANDY_GMM_SETTING_SIZE; i++) {
        memset(andy_rdata, 0, sizeof(andy_rdata));
        ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,
                                                  mipi_sh_andy_cmd_GRNGGMM[j++][0],
                                                  andy_rdata,
                                                  1);
        if (ret) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_mipi_diag_read_reg.");
            goto shdisp_end;
        }
        andy_temp_data[i] = ((andy_rdata[0] << 8) & 0x0300);
        memset(andy_rdata, 0, sizeof(andy_rdata));
        ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,
                                                  mipi_sh_andy_cmd_GRNGGMM[j++][0],
                                                  andy_rdata,
                                                  1);
        if (ret) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_mipi_diag_read_reg.");
            goto shdisp_end;
        }
        andy_temp_data[i] |= (andy_rdata[0] & 0x00FF);
    }
    memcpy(gmm_info->gammaG, andy_temp_data, sizeof(andy_temp_data));

    memset(andy_temp_data, 0, sizeof(andy_temp_data));
    for (i = 0, j = 0; i < (SHDISP_ANDY_GMM_SETTING_SIZE / 2); i++) {
        memset(andy_rdata, 0, sizeof(andy_rdata));
        ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,
                                                  mipi_sh_andy_cmd_BLPSGMM[j++][0],
                                                  andy_rdata,
                                                  1);
        if (ret) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_mipi_diag_read_reg.");
            goto shdisp_end;
        }
        andy_temp_data[i] = ((andy_rdata[0] << 8) & 0x0300);
        memset(andy_rdata, 0, sizeof(andy_rdata));
        ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,
                                                  mipi_sh_andy_cmd_BLPSGMM[j++][0],
                                                  andy_rdata,
                                                  1);
        if (ret) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_mipi_diag_read_reg.");
            goto shdisp_end;
        }
        andy_temp_data[i] |= (andy_rdata[0] & 0x00FF);
    }

    for (i = SHDISP_ANDY_GMM_NEGATIVE_OFFSET, j = 0; i < SHDISP_ANDY_GMM_SETTING_SIZE; i++) {
        memset(andy_rdata, 0, sizeof(andy_rdata));
        ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,
                                                  mipi_sh_andy_cmd_BLNGGMM[j++][0],
                                                  andy_rdata,
                                                  1);
        if (ret) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_mipi_diag_read_reg.");
            goto shdisp_end;
        }
        andy_temp_data[i] = ((andy_rdata[0] << 8) & 0x0300);
        memset(andy_rdata, 0, sizeof(andy_rdata));
        ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,
                                                  mipi_sh_andy_cmd_BLNGGMM[j++][0],
                                                  andy_rdata,
                                                  1);
        if (ret) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_mipi_diag_read_reg.");
            goto shdisp_end;
        }
        andy_temp_data[i] |= (andy_rdata[0] & 0x00FF);
    }
    memcpy(gmm_info->gammaB, andy_temp_data, sizeof(andy_temp_data));

    if (!set_applied_voltage) {
        goto shdisp_end;
    }

    ret = shdisp_panel_API_mipi_diag_write_reg(SHDISP_DTYPE_DCS_WRITE1,
                                               mipi_sh_andy_cmd_SwitchCommand[1][0],
                                               &mipi_sh_andy_cmd_SwitchCommand[1][1],
                                               1);
    if (ret) {
        SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_write_reg.");
        goto shdisp_end;
    }

    memset(andy_rdata, 0, sizeof(andy_rdata));
    ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,
                                              mipi_sh_andy_cmd_RegulatorPumpSetting[SHDISP_ANDY_VGH][0],
                                              andy_rdata,
                                              1);
    if (ret) {
        SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_read_reg.");
        goto shdisp_end;
    }
    gmm_info->vgh = andy_rdata[0];

    memset(andy_rdata, 0, sizeof(andy_rdata));
    ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,
                                              mipi_sh_andy_cmd_RegulatorPumpSetting[SHDISP_ANDY_VGL][0],
                                              andy_rdata,
                                              1);
    if (ret) {
        SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_read_reg.");
        goto shdisp_end;
    }
    gmm_info->vgl = andy_rdata[0];

    memset(andy_rdata, 0, sizeof(andy_rdata));
    ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,
                                              mipi_sh_andy_cmd_RegulatorPumpSetting[SHDISP_ANDY_GVDDP][0],
                                              andy_rdata,
                                              1);
    if (ret) {
        SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_read_reg.");
        goto shdisp_end;
    }
    gmm_info->gvddp = andy_rdata[0];

    memset(andy_rdata, 0, sizeof(andy_rdata));
    ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,
                                              mipi_sh_andy_cmd_RegulatorPumpSetting[SHDISP_ANDY_GVDDN][0],
                                              andy_rdata,
                                              1);
    if (ret) {
        SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_read_reg.");
        goto shdisp_end;
    }
    gmm_info->gvddn = andy_rdata[0];

    memset(andy_rdata, 0, sizeof(andy_rdata));
    ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,
                                              mipi_sh_andy_cmd_RegulatorPumpSetting[SHDISP_ANDY_GVDDP2][0],
                                              andy_rdata,
                                              1);
    if (ret) {
        SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_read_reg.");
        goto shdisp_end;
    }
    gmm_info->gvddp2 = andy_rdata[0];

    memset(andy_rdata, 0, sizeof(andy_rdata));
    ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,
                                              mipi_sh_andy_cmd_RegulatorPumpSetting[SHDISP_ANDY_VGHO][0],
                                              andy_rdata,
                                              1);
    if (ret) {
        SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_read_reg.");
        goto shdisp_end;
    }
    gmm_info->vgho = andy_rdata[0];

    memset(andy_rdata, 0, sizeof(andy_rdata));
    ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,
                                              mipi_sh_andy_cmd_RegulatorPumpSetting[SHDISP_ANDY_VGLO][0],
                                              andy_rdata,
                                              1);
    if (ret) {
        SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_read_reg.");
        goto shdisp_end;
    }
    gmm_info->vglo = andy_rdata[0];

    memset(andy_rdata, 0, sizeof(andy_rdata));
    ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,
                                              mipi_sh_andy_cmd_RegulatorPumpSetting[SHDISP_ANDY_AVDDR][0],
                                              andy_rdata,
                                              1);
    if (ret) {
        SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_read_reg.");
        goto shdisp_end;
    }
    gmm_info->avddr = andy_rdata[0];

    memset(andy_rdata, 0, sizeof(andy_rdata));
    ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,
                                              mipi_sh_andy_cmd_RegulatorPumpSetting[SHDISP_ANDY_AVEER][0],
                                              andy_rdata,
                                              1);
    if (ret) {
        SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_read_reg.");
        goto shdisp_end;
    }
    gmm_info->aveer = andy_rdata[0];

shdisp_end:
    shdisp_panel_API_mipi_diag_write_reg(SHDISP_DTYPE_DCS_WRITE1,
                                         mipi_sh_andy_cmd_SwitchCommand[0][0],
                                         &mipi_sh_andy_cmd_SwitchCommand[0][1],
                                         1);

    if (ret) {
        return ret;
    }

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_andy_API_diag_set_gmmtable_and_voltage                             */
/* ------------------------------------------------------------------------- */
static int shdisp_andy_API_diag_set_gmmtable_and_voltage(struct shdisp_diag_gamma_info *gmm_info)
{
    int ret = 0;
    int pcnt, ncnt, i;

    SHDISP_TRACE("in");

    mdss_shdisp_dsi_bus_clk_ctrl(true);
    ret = shdisp_andy_diag_set_gmmtable_and_voltage(gmm_info, 1);
    mdss_shdisp_dsi_bus_clk_ctrl(false);
    if (ret) {
        return ret;
    }

    for (pcnt = 0; pcnt < SHDISP_ANDY_GMM_NEGATIVE_OFFSET; pcnt++) {
        ncnt = pcnt + SHDISP_ANDY_GMM_NEGATIVE_OFFSET;
        i = pcnt * 2;
        mipi_sh_andy_cmd_RDPSGMM[i][1]       = ((gmm_info->gammaR[pcnt] >> 8) & 0x0003);
        mipi_sh_andy_cmd_RDPSGMM[i + 1][1]   = ( gmm_info->gammaR[pcnt] & 0x00FF);
        mipi_sh_andy_cmd_RDNGGMM[i][1]       = ((gmm_info->gammaR[ncnt] >> 8) & 0x0003);
        mipi_sh_andy_cmd_RDNGGMM[i + 1][1]   = ( gmm_info->gammaR[ncnt] & 0x00FF);
        mipi_sh_andy_cmd_GRPSGMM[i][1]     = ((gmm_info->gammaG[pcnt] >> 8) & 0x0003);
        mipi_sh_andy_cmd_GRPSGMM[i + 1][1] = ( gmm_info->gammaG[pcnt] & 0x00FF);
        mipi_sh_andy_cmd_GRNGGMM[i][1]     = ((gmm_info->gammaG[ncnt] >> 8) & 0x0003);
        mipi_sh_andy_cmd_GRNGGMM[i + 1][1] = ( gmm_info->gammaG[ncnt] & 0x00FF);
        mipi_sh_andy_cmd_BLPSGMM[i][1]      = ((gmm_info->gammaB[pcnt] >> 8) & 0x0003);
        mipi_sh_andy_cmd_BLPSGMM[i + 1][1]  = ( gmm_info->gammaB[pcnt] & 0x00FF);
        mipi_sh_andy_cmd_BLNGGMM[i][1]      = ((gmm_info->gammaB[ncnt] >> 8) & 0x0003);
        mipi_sh_andy_cmd_BLNGGMM[i + 1][1]  = ( gmm_info->gammaB[ncnt] & 0x00FF);
    }

    mipi_sh_andy_cmd_RegulatorPumpSetting[SHDISP_ANDY_VGH][1]    = gmm_info->vgh;
    mipi_sh_andy_cmd_RegulatorPumpSetting[SHDISP_ANDY_VGL][1]    = gmm_info->vgl;
    mipi_sh_andy_cmd_RegulatorPumpSetting[SHDISP_ANDY_GVDDP][1]  = gmm_info->gvddp;
    mipi_sh_andy_cmd_RegulatorPumpSetting[SHDISP_ANDY_GVDDN][1]  = gmm_info->gvddn;
    mipi_sh_andy_cmd_RegulatorPumpSetting[SHDISP_ANDY_GVDDP2][1] = gmm_info->gvddp2;
    mipi_sh_andy_cmd_RegulatorPumpSetting[SHDISP_ANDY_VGHO][1]   = gmm_info->vgho;
    mipi_sh_andy_cmd_RegulatorPumpSetting[SHDISP_ANDY_VGLO][1]   = gmm_info->vglo;
    mipi_sh_andy_cmd_RegulatorPumpSetting[SHDISP_ANDY_AVDDR][1]  = gmm_info->avddr;
    mipi_sh_andy_cmd_RegulatorPumpSetting[SHDISP_ANDY_AVEER][1]  = gmm_info->aveer;

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_andy_API_diag_get_gmmtable_and_voltage                             */
/* ------------------------------------------------------------------------- */
static int shdisp_andy_API_diag_get_gmmtable_and_voltage(struct shdisp_diag_gamma_info *gmm_info)
{
    int ret = 0;

    SHDISP_TRACE("in");

    shdisp_andy_stop_video();
    mdss_shdisp_dsi_bus_clk_ctrl(true);
    ret = shdisp_andy_diag_get_gmmtable_and_voltage(gmm_info, 1);
    mdss_shdisp_dsi_bus_clk_ctrl(false);
    shdisp_andy_start_video();
    if (ret) {
        return ret;
    }

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_andy_API_diag_set_gmm                                              */
/* ------------------------------------------------------------------------- */
static int shdisp_andy_API_diag_set_gmm(struct shdisp_diag_gamma *gmm)
{
    int ret = 0;
    int i = 0, j = 0, k = 0;
    int group_idx, level_idx, addr_idx;
    unsigned char andy_gmm_wdata[26];
    unsigned char andy_gmm_addr[26];

    SHDISP_TRACE("in");

    if ((gmm->level < SHDISP_ANDY_GMM_LEVEL_MIN) || (gmm->level > SHDISP_ANDY_GMM_LEVEL_MAX)) {
        SHDISP_ERR("<INVALID_VALUE> gmm->level(%d).", gmm->level);
        return SHDISP_RESULT_FAILURE;
    }

    if (!diag_tmp_gmm_info_set) {
        shdisp_andy_stop_video();
        ret = shdisp_andy_diag_get_gmmtable_and_voltage(&diag_tmp_gmm_info, 0);
        shdisp_andy_start_video();
        if (ret) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_andy_diag_get_gmmtable_and_voltage.");
            goto shdisp_end;
        }
        diag_tmp_gmm_info_set = 1;
    }

    diag_tmp_gmm_info.gammaR[gmm->level - 1] = gmm->gammaR_p;
    diag_tmp_gmm_info.gammaR[SHDISP_ANDY_GMM_NEGATIVE_OFFSET + (gmm->level - 1)] = gmm->gammaR_n;
    diag_tmp_gmm_info.gammaG[gmm->level - 1] = gmm->gammaG_p;
    diag_tmp_gmm_info.gammaG[SHDISP_ANDY_GMM_NEGATIVE_OFFSET + (gmm->level - 1)] = gmm->gammaG_n;
    diag_tmp_gmm_info.gammaB[gmm->level - 1] = gmm->gammaB_p;
    diag_tmp_gmm_info.gammaB[SHDISP_ANDY_GMM_NEGATIVE_OFFSET + (gmm->level - 1)] = gmm->gammaB_n;

    group_idx = (gmm->level - 1) / 2;
    level_idx = group_idx * SHDISP_ANDY_GMM_GROUP_BELONG_LEVEL;
    addr_idx = group_idx * SHDISP_ANDY_GMM_GROUP_BELONG_ADDR;

    andy_gmm_wdata[j++] = mipi_sh_andy_cmd_SwitchCommand[1][1];
    andy_gmm_addr[k++] = mipi_sh_andy_cmd_SwitchCommand[1][0];

    for (i = 0; i < SHDISP_ANDY_GMM_GROUP_BELONG_LEVEL; i++) {
        andy_gmm_wdata[j++] = ((diag_tmp_gmm_info.gammaR[level_idx + i] >> 8) & 0x0003);
        andy_gmm_wdata[j++] = (diag_tmp_gmm_info.gammaR[level_idx + i] & 0x00FF);
        andy_gmm_addr[k++] = mipi_sh_andy_cmd_RDPSGMM[addr_idx + i * SHDISP_ANDY_GMM_GROUP_BELONG_LEVEL][0];
        andy_gmm_addr[k++] =
            mipi_sh_andy_cmd_RDPSGMM[addr_idx + i * SHDISP_ANDY_GMM_GROUP_BELONG_LEVEL + 1][0];
    }

    for (i = 0; i < SHDISP_ANDY_GMM_GROUP_BELONG_LEVEL; i++) {
        andy_gmm_wdata[j++] =
            ((diag_tmp_gmm_info.gammaR[level_idx + i + SHDISP_ANDY_GMM_NEGATIVE_OFFSET] >> 8) & 0x0003);
        andy_gmm_wdata[j++] =
             (diag_tmp_gmm_info.gammaR[level_idx + i + SHDISP_ANDY_GMM_NEGATIVE_OFFSET] & 0x00FF);
        andy_gmm_addr[k++] = mipi_sh_andy_cmd_RDNGGMM[addr_idx + i * SHDISP_ANDY_GMM_GROUP_BELONG_LEVEL][0];
        andy_gmm_addr[k++] =
            mipi_sh_andy_cmd_RDNGGMM[addr_idx + i * SHDISP_ANDY_GMM_GROUP_BELONG_LEVEL + 1][0];
    }

    if (gmm->level > 6) {
        andy_gmm_wdata[j++] = mipi_sh_andy_cmd_SwitchCommand[2][1];
        andy_gmm_addr[k++] = mipi_sh_andy_cmd_SwitchCommand[2][0];
    }
    for (i = 0; i < SHDISP_ANDY_GMM_GROUP_BELONG_LEVEL; i++) {
        andy_gmm_wdata[j++] = ((diag_tmp_gmm_info.gammaG[level_idx + i] >> 8) & 0x0003);
        andy_gmm_wdata[j++] = (diag_tmp_gmm_info.gammaG[level_idx + i] & 0x00FF);
        andy_gmm_addr[k++] =
            mipi_sh_andy_cmd_GRPSGMM[addr_idx + i * SHDISP_ANDY_GMM_GROUP_BELONG_LEVEL][0];
        andy_gmm_addr[k++] =
            mipi_sh_andy_cmd_GRPSGMM[addr_idx + i * SHDISP_ANDY_GMM_GROUP_BELONG_LEVEL + 1][0];
    }

    if (gmm->level <= 6) {
        andy_gmm_wdata[j++] = mipi_sh_andy_cmd_SwitchCommand[2][1];
        andy_gmm_addr[k++] = mipi_sh_andy_cmd_SwitchCommand[2][0];
    }
    for (i = 0; i < SHDISP_ANDY_GMM_GROUP_BELONG_LEVEL; i++) {
        andy_gmm_wdata[j++] =
            ((diag_tmp_gmm_info.gammaG[level_idx + i + SHDISP_ANDY_GMM_NEGATIVE_OFFSET] >> 8) & 0x0003);
        andy_gmm_wdata[j++] =
             (diag_tmp_gmm_info.gammaG[level_idx + i + SHDISP_ANDY_GMM_NEGATIVE_OFFSET] & 0x00FF);
        andy_gmm_addr[k++]  =
            mipi_sh_andy_cmd_GRNGGMM[addr_idx + i * SHDISP_ANDY_GMM_GROUP_BELONG_LEVEL][0];
        andy_gmm_addr[k++]  =
            mipi_sh_andy_cmd_GRNGGMM[addr_idx + i * SHDISP_ANDY_GMM_GROUP_BELONG_LEVEL + 1][0];
    }

    for (i = 0; i < SHDISP_ANDY_GMM_GROUP_BELONG_LEVEL; i++) {
        andy_gmm_wdata[j++] = ((diag_tmp_gmm_info.gammaB[level_idx + i] >> 8) & 0x0003);
        andy_gmm_wdata[j++] = (diag_tmp_gmm_info.gammaB[level_idx + i] & 0x00FF);
        andy_gmm_addr[k++] = mipi_sh_andy_cmd_BLPSGMM[addr_idx + i * SHDISP_ANDY_GMM_GROUP_BELONG_LEVEL][0];
        andy_gmm_addr[k++] =
            mipi_sh_andy_cmd_BLPSGMM[addr_idx + i * SHDISP_ANDY_GMM_GROUP_BELONG_LEVEL + 1][0];
    }

    for (i = 0; i < SHDISP_ANDY_GMM_GROUP_BELONG_LEVEL; i++) {
        andy_gmm_wdata[j++] =
            ((diag_tmp_gmm_info.gammaB[level_idx + i + SHDISP_ANDY_GMM_NEGATIVE_OFFSET] >> 8) & 0x0003);
        andy_gmm_wdata[j++] =
             (diag_tmp_gmm_info.gammaB[level_idx + i + SHDISP_ANDY_GMM_NEGATIVE_OFFSET] & 0x00FF);
        andy_gmm_addr[k++] =
            mipi_sh_andy_cmd_BLNGGMM[addr_idx + i * SHDISP_ANDY_GMM_GROUP_BELONG_LEVEL][0];
        andy_gmm_addr[k++] =
            mipi_sh_andy_cmd_BLNGGMM[addr_idx + i * SHDISP_ANDY_GMM_GROUP_BELONG_LEVEL + 1][0];
    }

    for (i = 0; i < (sizeof(andy_gmm_addr) / sizeof(*andy_gmm_addr)); i++) {
        ret = shdisp_panel_API_mipi_diag_write_reg(SHDISP_DTYPE_DCS_WRITE1, andy_gmm_addr[i],
                                                            &andy_gmm_wdata[i], 1);
        if (ret) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_mipi_diag_write_reg!!" );
            goto shdisp_end;
        }
    }

shdisp_end:
    if (ret) {
        return ret;
    }

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_andy_API_shutdown                                                  */
/* ------------------------------------------------------------------------- */
static int shdisp_andy_API_shutdown(void)
{
    shdisp_andy_hw_reset(true);
    shdisp_SYS_API_delay_us(5 * 1000);
#ifdef SHDISP_ANDY_VDD
    shdisp_andy_vdd_off();
#endif /* SHDISP_ANDY_VDD */

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_andy_API_dump                                                      */
/* ------------------------------------------------------------------------- */
static void shdisp_andy_API_dump(int type)
{
#if defined(CONFIG_ANDROID_ENGINEERING)
    shdisp_andy_dump_reg();
#endif /* CONFIG_ANDROID_ENGINEERING */
}

#ifdef SHDISP_ANDY_VDD
/* ------------------------------------------------------------------------- */
/* shdisp_andy_vdd_on                                                        */
/* ------------------------------------------------------------------------- */
static void shdisp_andy_vdd_on(void)
{
    shdisp_SYS_API_Host_gpio_request(shdisp_andy_vdd_gpio, "LCD_VDD");
    shdisp_SYS_API_set_Host_gpio(shdisp_andy_vdd_gpio, SHDISP_GPIO_CTL_HIGH);
    shdisp_SYS_API_delay_us(10 * 1000);
}

/* ------------------------------------------------------------------------- */
/* shdisp_andy_vdd_off                                                       */
/* ------------------------------------------------------------------------- */
static void shdisp_andy_vdd_off(void)
{
    shdisp_SYS_API_set_Host_gpio(shdisp_andy_vdd_gpio, SHDISP_GPIO_CTL_LOW);
    shdisp_SYS_API_Host_gpio_free(shdisp_andy_vdd_gpio);
}
#endif /* SHDISP_ANDY_VDD */

/* ------------------------------------------------------------------------- */
/* shdisp_andy_set_switchcommand                                             */
/* ------------------------------------------------------------------------- */
static int shdisp_andy_set_switchcommand(char page)
{
    struct shdisp_dsi_cmd_desc cmd;
    char payload[2] = {0xff, 00};

    payload[1] = page;
    memset(&cmd, 0, sizeof(cmd));
    cmd.dtype = SHDISP_DTYPE_DCS_WRITE1;
    cmd.dlen = 2;
    cmd.wait = 0;
    cmd.payload = payload;

    shdisp_panel_API_mipi_dsi_cmds_tx(1, &cmd, 1);

    return SHDISP_RESULT_SUCCESS;
}

#ifdef SHDISP_POWER_MODE_CHK
/* ------------------------------------------------------------------------- */
/* shdisp_panel_andy_reg_read                                                */
/* ------------------------------------------------------------------------- */
static int shdisp_panel_andy_reg_read(unsigned char addr, unsigned char *out_data)
{
    struct shdisp_dsi_cmd_desc cmd[1];
    char cmd_buf[1 + 2];

    memset(cmd_buf, 0x00, sizeof(cmd_buf));
    cmd_buf[0] = addr;
    cmd_buf[1] = 0x00;

    memset(cmd, 0x00, sizeof(cmd));
    cmd[0].dtype = SHDISP_DTYPE_DCS_READ;
    cmd[0].wait = 0x00;
    cmd[0].dlen = 1;
    cmd[0].payload = cmd_buf;
    cmd[0].mode = 0;

    if (shdisp_panel_API_mipi_dsi_cmds_rx(out_data, cmd, 1) != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("mipi_dsi_cmds_rx error");
        return SHDISP_RESULT_FAILURE;
    }

    return SHDISP_RESULT_SUCCESS;
}
#endif /* SHDISP_POWER_MODE_CHK */

/* ------------------------------------------------------------------------- */
/* shdisp_andy_hw_reset                                                      */
/* ------------------------------------------------------------------------- */
static void shdisp_andy_hw_reset(bool reset)
{
    SHDISP_TRACE("call reset=%d", reset);
    if (reset) {
        shdisp_SYS_API_set_Host_gpio(shdisp_andy_rst_gpio, SHDISP_GPIO_CTL_LOW);
        shdisp_SYS_API_Host_gpio_free(shdisp_andy_rst_gpio);
    } else {
        shdisp_SYS_API_Host_gpio_request(shdisp_andy_rst_gpio, "PANEL_RST_N");
        shdisp_SYS_API_set_Host_gpio(shdisp_andy_rst_gpio, SHDISP_GPIO_CTL_HIGH);
    }
}

/* ------------------------------------------------------------------------- */
/* shdisp_andy_mipi_cmd_display_on                                           */
/* ------------------------------------------------------------------------- */
static int shdisp_andy_mipi_cmd_display_on(void)
{
    int ret = SHDISP_RESULT_SUCCESS;

    SHDISP_TRACE("in");

    ret = MIPI_DSI_COMMAND_TX(mipi_sh_andy_cmds_display_on1_1);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out1 ret=%d", ret);
        return ret;
    }

    ret = MIPI_DSI_COMMAND_TX_COMMIT(mipi_sh_andy_cmds_display_on1_2);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out2 ret=%d", ret);
        return ret;
    }

    ret = shdisp_andy_sleepout_wait_proc();
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("Failed to shdisp_andy_sleepout_wait_proc(). (ret=%d)", ret);
    }

    SHDISP_TRACE("out");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_andy_mipi_cmd_lcd_off                                              */
/* ------------------------------------------------------------------------- */
static int shdisp_andy_mipi_cmd_lcd_off(void)
{
    int ret = SHDISP_RESULT_SUCCESS;
    unsigned short tmp_vcom1;

    SHDISP_TRACE("in");

    tmp_vcom1 = (unsigned short)mipi_sh_andy_cmd_RegulatorPumpSetting[NO_VCOM1_L][1];
    if (mipi_sh_andy_cmd_RegulatorPumpSetting[NO_VCOM12_H][1] & 0x03) {
        tmp_vcom1 |= 0x100;
    }

    mipi_sh_andy_cmd_VCOM1_OFF_Setting[1] = (char)((tmp_vcom1 / 2) & 0xFF);
    mipi_sh_andy_cmd_VCOM2_OFF_Setting[1] = (char)((tmp_vcom1 / 2) & 0xFF);
    ret = MIPI_DSI_COMMAND_TX_COMMIT(mipi_sh_andy_cmds_display_off);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("mipi_dsi_cmds_tx error ret=%d", ret);
    }

    diag_tmp_gmm_info_set = 0;

    SHDISP_TRACE("out");
    return ret;
}

#ifdef SHDISP_POWER_MODE_CHK
/* ------------------------------------------------------------------------- */
/* shdisp_andy_power_mode_chk                                                */
/* ------------------------------------------------------------------------- */
static int shdisp_andy_power_mode_chk(unsigned char addr)
{
    int ret;
    unsigned char read_data = 0x00;
#ifdef SHDISP_RESET_LOG
    struct shdisp_dbg_error_code err_code;
#endif /* SHDISP_RESET_LOG */
    int dsi_ret;

    dsi_ret = MIPI_DSI_COMMAND_TX_COMMIT(mipi_sh_andy_cmds_dispon_check);
    if (dsi_ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("mipi_dsi_cmds_tx error ret=%d", dsi_ret);
    }

    ret = shdisp_panel_andy_reg_read(addr, &read_data);

#if defined(CONFIG_ANDROID_ENGINEERING)
    if (shdisp_dbg_API_get_recovery_check_error() == SHDISP_DBG_RECOVERY_ERROR_DISPON_READ) {
        shdisp_dbg_API_update_recovery_check_error(SHDISP_DBG_RECOVERY_ERROR_NONE);
        if (ret == SHDISP_RESULT_SUCCESS) {
            SHDISP_DEBUG("force disp on read error.");
            ret = SHDISP_RESULT_FAILURE;
        } else {
            SHDISP_DEBUG("disp on read error. (ret=%d addr=0x%02x)", ret, addr);
        }
    }
#endif /* CONFIG_ANDROID_ENGINEERING */

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("mipi_dsi_cmds_rx error");
#ifdef SHDISP_RESET_LOG
        err_code.mode = SHDISP_DBG_MODE_LINUX;
        err_code.type = SHDISP_DBG_TYPE_PANEL;
        err_code.code = SHDISP_DBG_CODE_READ_ERROR;
        err_code.subcode = SHDISP_DBG_SUBCODE_DISPON_CHK;
        shdisp_dbg_API_err_output(&err_code, 0);
#endif /* SHDISP_RESET_LOG */
        return SHDISP_RESULT_FAILURE;
    }

#if defined(CONFIG_ANDROID_ENGINEERING)
        if (shdisp_dbg_API_get_recovery_check_error() == SHDISP_DBG_RECOVERY_ERROR_DISPON) {
            shdisp_dbg_API_update_recovery_check_error(SHDISP_DBG_RECOVERY_ERROR_NONE);
            SHDISP_DEBUG("force disp on error.");
            read_data = 0xFF;
        }
#endif /* CONFIG_ANDROID_ENGINEERING */

    SHDISP_DEBUG("addr = 0x%02x.read_data = 0x%02x", addr, read_data);

    if (read_data != 0x9C) {
        SHDISP_ERR("POWER_MODE error.addr = 0x%02x.read_data = 0x%02x", addr, read_data);
#ifdef SHDISP_RESET_LOG
        err_code.mode = SHDISP_DBG_MODE_LINUX;
        err_code.type = SHDISP_DBG_TYPE_PANEL;
        err_code.code = SHDISP_DBG_CODE_ERROR_DETECT;
        err_code.subcode = SHDISP_DBG_SUBCODE_DISPON_CHK;
        shdisp_dbg_API_err_output(&err_code, 0);
#endif /* SHDISP_RESET_LOG */
        return SHDISP_RESULT_FAILURE;
    }

    return SHDISP_RESULT_SUCCESS;
}
#endif /* SHDISP_POWER_MODE_CHK */

/* ------------------------------------------------------------------------- */
/* shdisp_andy_sleepout_wait_proc                                            */
/* ------------------------------------------------------------------------- */
static int shdisp_andy_sleepout_wait_proc()
{
    struct timespec ts_start, ts_blk_start, ts_blk_end;
#ifdef SHDISP_POWER_MODE_CHK
    int ret;
#endif /* SHDISP_POWER_MODE_CHK */
    unsigned long long wtime = 0, wtime_all = 0;
    int dsi_ret;

    SHDISP_TRACE("in");

    getnstimeofday(&ts_start);

    (void)shdisp_pm_API_als_user_manager(SHDISP_DEV_TYPE_LCD, SHDISP_DEV_REQ_INIT);
    shdisp_SYS_API_delay_us(10*1000);
    shdisp_bdic_API_update_led_value();

    getnstimeofday(&ts_blk_start);

    dsi_ret = MIPI_DSI_COMMAND_TX_COMMIT(mipi_sh_andy_cmds_gmm);
    if (dsi_ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("mipi_dsi_cmds_tx error ret=%d", dsi_ret);
    }

#ifdef SHDISP_POWER_MODE_CHK
    ret = shdisp_andy_power_mode_chk(0x0A);
    if (ret == SHDISP_RESULT_SUCCESS) {
        shdisp_andy_dispon_chk = SHDISP_PANEL_DISPON_CHK_OK;
    } else {
        shdisp_andy_dispon_chk = SHDISP_PANEL_DISPON_CHK_NG;
    }

#endif /* SHDISP_POWER_MODE_CHK */

    getnstimeofday(&ts_blk_end);
    wtime = (ts_blk_end.tv_sec - ts_blk_start.tv_sec) * 1000000;
    wtime += (ts_blk_end.tv_nsec - ts_blk_start.tv_nsec) / 1000;
    SHDISP_PERFORMANCE("rest of gmm wait=%lld, wtime=%llu", (10000 - wtime), wtime);

    if (wtime < 10000) {
        shdisp_SYS_API_delay_us(10000 - wtime);
    }

    getnstimeofday(&ts_blk_start);

    (void)shdisp_pm_API_als_user_manager(SHDISP_DEV_TYPE_LCD, SHDISP_DEV_REQ_ON);

    getnstimeofday(&ts_blk_end);
    wtime = (ts_blk_end.tv_sec - ts_blk_start.tv_sec) * 1000000;
    wtime += (ts_blk_end.tv_nsec - ts_blk_start.tv_nsec) / 1000;
    wtime_all = (ts_blk_end.tv_sec - ts_start.tv_sec) * 1000000;
    wtime_all += (ts_blk_end.tv_nsec - ts_start.tv_nsec) / 1000;
    SHDISP_PERFORMANCE("rest of als_mode_on wait=%lld, wtime=%llu", (80000 - wtime_all), wtime);

    if (wtime_all < 80000) {
        shdisp_SYS_API_delay_us(80000 - wtime_all);
    }

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_andy_mipi_cmd_lcd_on                                               */
/* ------------------------------------------------------------------------- */
static int shdisp_andy_mipi_cmd_lcd_on(void)
{
    int ret = SHDISP_RESULT_SUCCESS;

    SHDISP_TRACE("in");

    ret = MIPI_DSI_COMMAND_TX(mipi_sh_andy_cmds_initial1);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out1 ret=%d", ret);
        return ret;
    }

    if (IS_FLICKER_ADJUSTED(shdisp_panel_ctx.vcom_nvram)) {
        ret = MIPI_DSI_COMMAND_TX(mipi_sh_andy_cmds_initial1_regulator_ts2_0);
        if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_ERR("out2-1 ret=%d", ret);
            return ret;
        }
    } else {
        ret = MIPI_DSI_COMMAND_TX(mipi_sh_andy_cmds_initial1_regulator_ts2_0_flicker_unadjusted);
        if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_ERR("out2-2 ret=%d", ret);
            return ret;
        }
    }

    ret = MIPI_DSI_COMMAND_TX(mipi_sh_andy_cmds_terminal);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out3 ret=%d", ret);
        return ret;
    }

    ret = MIPI_DSI_COMMAND_TX(mipi_sh_andy_cmds_timing);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out4 ret=%d", ret);
        return ret;
    }

    ret = MIPI_DSI_COMMAND_TX(mipi_sh_andy_cmds_initial2);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out5 ret=%d", ret);
        return ret;
    }

    ret = MIPI_DSI_COMMAND_TX(mipi_sh_andy_cmds_power);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out6 ret=%d", ret);
        return ret;
    }

    ret = MIPI_DSI_COMMAND_TX_COMMIT(mipi_sh_andy_cmds_sync_ts2_0);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out7 ret=%d", ret);
        return ret;
    }

    SHDISP_TRACE("out");
    return ret;
}

#ifndef SHDISP_NOT_SUPPORT_NO_OS
/* ------------------------------------------------------------------------- */
/* shdisp_andy_init_phy_gmm                                                  */
/* ------------------------------------------------------------------------- */
static int shdisp_andy_init_phy_gmm(struct shdisp_lcddr_phy_gmm_reg *phy_gmm)
{
    int ret = SHDISP_RESULT_SUCCESS;
    int i;
    unsigned int checksum;

    SHDISP_TRACE("in");

    if (phy_gmm == NULL) {
        SHDISP_ERR("phy_gmm is NULL.");
        return SHDISP_RESULT_FAILURE;
    }

    if (phy_gmm->status != SHDISP_LCDDR_GMM_STATUS_OK) {
        SHDISP_DEBUG("gammg status invalid. status=%02x", phy_gmm->status);
        ret = SHDISP_RESULT_FAILURE;
    } else {
        checksum = phy_gmm->status;
        for (i = 0; i < SHDISP_LCDDR_PHY_GMM_BUF_MAX; i++) {
            checksum = checksum + phy_gmm->buf[i];
        }
        for (i = 0; i < SHDISP_LCDDR_APPLIED_VOLTAGE_SIZE; i++) {
            checksum = checksum + phy_gmm->applied_voltage[i];
        }
        if ((checksum & 0x00FFFFFF) != phy_gmm->chksum) {
            SHDISP_DEBUG("%s: gammg chksum NG. chksum=%06x calc_chksum=%06x",
                         __func__, phy_gmm->chksum, (checksum & 0x00FFFFFF));
            ret = SHDISP_RESULT_FAILURE;
        }
    }

    if (ret == SHDISP_RESULT_FAILURE) {
        SHDISP_ERR("phy_gmm error");
        return SHDISP_RESULT_FAILURE;
    }

    for (i = 0; i < SHDISP_ANDY_GMM_NEGATIVE_OFFSET; i++) {
        mipi_sh_andy_cmd_RDPSGMM[i * 2][1] = ((phy_gmm->buf[i] >> 8) & 0x0003);
        mipi_sh_andy_cmd_RDPSGMM[i * 2 + 1][1] = (phy_gmm->buf[i] & 0x00FF);
        mipi_sh_andy_cmd_RDNGGMM[i * 2][1] =
            ((phy_gmm->buf[i + SHDISP_ANDY_GMM_NEGATIVE_OFFSET] >> 8) & 0x0003);
        mipi_sh_andy_cmd_RDNGGMM[i * 2 + 1][1] =
            (phy_gmm->buf[i + SHDISP_ANDY_GMM_NEGATIVE_OFFSET] & 0x00FF);
        mipi_sh_andy_cmd_GRPSGMM[i * 2][1] =
            ((phy_gmm->buf[i + SHDISP_ANDY_GMM_NEGATIVE_OFFSET * 2] >> 8) & 0x0003);
        mipi_sh_andy_cmd_GRPSGMM[i * 2 + 1][1] =
            (phy_gmm->buf[i + SHDISP_ANDY_GMM_NEGATIVE_OFFSET * 2] & 0x00FF);
        mipi_sh_andy_cmd_GRNGGMM[i * 2][1] =
            ((phy_gmm->buf[i + SHDISP_ANDY_GMM_NEGATIVE_OFFSET * 3] >> 8) & 0x0003);
        mipi_sh_andy_cmd_GRNGGMM[i * 2 + 1][1] =
            (phy_gmm->buf[i + SHDISP_ANDY_GMM_NEGATIVE_OFFSET * 3] & 0x00FF);
        mipi_sh_andy_cmd_BLPSGMM[i * 2][1] =
            ((phy_gmm->buf[i + SHDISP_ANDY_GMM_NEGATIVE_OFFSET * 4] >> 8) & 0x0003);
        mipi_sh_andy_cmd_BLPSGMM[i * 2 + 1][1] =
            (phy_gmm->buf[i + SHDISP_ANDY_GMM_NEGATIVE_OFFSET * 4] & 0x00FF);
        mipi_sh_andy_cmd_BLNGGMM[i * 2][1] =
            ((phy_gmm->buf[i + SHDISP_ANDY_GMM_NEGATIVE_OFFSET * 5] >> 8) & 0x0003);
        mipi_sh_andy_cmd_BLNGGMM[i * 2 + 1][1] =
            (phy_gmm->buf[i + SHDISP_ANDY_GMM_NEGATIVE_OFFSET * 5] & 0x00FF);
    }

    mipi_sh_andy_cmd_RegulatorPumpSetting[SHDISP_ANDY_VGH][1] = phy_gmm->applied_voltage[0];
    mipi_sh_andy_cmd_RegulatorPumpSetting[SHDISP_ANDY_VGL][1] = phy_gmm->applied_voltage[1];
    mipi_sh_andy_cmd_RegulatorPumpSetting[SHDISP_ANDY_GVDDP][1] = phy_gmm->applied_voltage[2];
    mipi_sh_andy_cmd_RegulatorPumpSetting[SHDISP_ANDY_GVDDN][1] = phy_gmm->applied_voltage[3];
    mipi_sh_andy_cmd_RegulatorPumpSetting[SHDISP_ANDY_GVDDP2][1] = phy_gmm->applied_voltage[4];
    mipi_sh_andy_cmd_RegulatorPumpSetting[SHDISP_ANDY_VGHO][1] = phy_gmm->applied_voltage[5];
    mipi_sh_andy_cmd_RegulatorPumpSetting[SHDISP_ANDY_VGLO][1] = phy_gmm->applied_voltage[6];
    mipi_sh_andy_cmd_RegulatorPumpSetting[SHDISP_ANDY_AVDDR][1] = phy_gmm->applied_voltage[7];
    mipi_sh_andy_cmd_RegulatorPumpSetting[SHDISP_ANDY_AVEER][1] = phy_gmm->applied_voltage[8];

    SHDISP_TRACE("out ret=%04x", ret);
    return ret;
}
#endif /* SHDISP_NOT_SUPPORT_NO_OS */

/*---------------------------------------------------------------------------*/
/* shdisp_andy_workqueue_handler                                             */
/*---------------------------------------------------------------------------*/
static void shdisp_andy_workqueue_handler(struct work_struct *work)
{
    int ret;
#ifdef SHDISP_RESET_LOG
    struct shdisp_dbg_error_code err_code;
#endif /* SHDISP_RESET_LOG */

    SHDISP_TRACE("in");

    if (shdisp_SYS_API_get_Host_gpio(shdisp_andy_mipi_err_gpio)) {
        SHDISP_ERR("MIPI Error");

#ifdef SHDISP_RESET_LOG
        err_code.mode = SHDISP_DBG_MODE_LINUX;
        err_code.type = SHDISP_DBG_TYPE_PANEL;
        err_code.code = SHDISP_DBG_CODE_ERROR_DETECT;
        err_code.subcode = SHDISP_DBG_SUBCODE_ESD_MIPI;
        shdisp_dbg_API_err_output(&err_code, 0);
        shdisp_dbg_API_set_subcode(SHDISP_DBG_SUBCODE_ESD_MIPI);
#endif /* SHDISP_RESET_LOG */

        ret = shdisp_API_do_lcd_det_recovery();
        if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_ERR("recovery request error!! ret=%d", ret);
        }
    } else {
        SHDISP_DEBUG("ED: GPIO34 Low");
        enable_irq(shdisp_andy_irq_gpio);
    }

    wake_unlock(&shdisp_andy_wakelock);

    SHDISP_TRACE("out");
    return;
}

/*---------------------------------------------------------------------------*/
/* shdisp_andy_int_isr                                                       */
/*---------------------------------------------------------------------------*/
static irqreturn_t shdisp_andy_int_isr(int irq_num, void *data)
{
    int ret;

    SHDISP_TRACE("in irq=%d", irq_num);

    if (shdisp_API_is_lcd_det_recovering()) {
        SHDISP_WARN("now recovering...");
        goto exit;
    }

    disable_irq_nosync(shdisp_andy_irq_gpio);

    if (!shdisp_wq_andy) {
        SHDISP_ERR("invalid work queue. wq=%p", shdisp_wq_andy);
        goto exit;
    }

    ret = shdisp_api_get_main_disp_status();
    if (ret == SHDISP_MAIN_DISP_OFF) {
        SHDISP_DEBUG("display OFF, will be exited.");
        goto exit;
    }

    wake_lock(&shdisp_andy_wakelock);

    ret = queue_work(shdisp_wq_andy, &shdisp_wq_andy_wk);
    if (ret == 0) {
        wake_unlock(&shdisp_andy_wakelock);
        SHDISP_DEBUG("failed to queue_work(). ret=%d", ret);
    }

exit:
    SHDISP_TRACE("out");
    return IRQ_HANDLED;
}

/* ------------------------------------------------------------------------- */
/*      shdisp_andy_probe                                                    */
/* ------------------------------------------------------------------------- */
static int shdisp_andy_probe(struct platform_device *pdev)
{
    int ret = SHDISP_RESULT_SUCCESS;
#ifdef CONFIG_OF
    struct resource *res;

    SHDISP_TRACE("in pdev=0x%p.", pdev);

    if (pdev) {
        res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
        if (!res) {
            SHDISP_ERR("irq resouce err!!");
        } else {
            shdisp_andy_irq_gpio = res->start;
            pshdisp_andy_irq_port_dev = pdev;
        }
        if (&(pdev->dev) != NULL) {
            shdisp_andy_rst_gpio = of_get_named_gpio(pdev->dev.of_node, "andy_rst_gpio", 0);
            if (!gpio_is_valid(shdisp_andy_rst_gpio)) {
                SHDISP_ERR("rst gpio not specified");
            } else {
                SHDISP_DEBUG("rst gpio succusess!");
            }
            shdisp_andy_vdd_gpio = of_get_named_gpio(pdev->dev.of_node, "andy_vdd_gpio", 0);
            if (!gpio_is_valid(shdisp_andy_vdd_gpio)) {
                SHDISP_ERR("rst gpio not specified");
            } else {
                SHDISP_DEBUG("rst gpio succusess!");
            }
            shdisp_andy_mipi_err_gpio = of_get_named_gpio(pdev->dev.of_node, "andy_mipi_err_gpio", 0);
            if (!gpio_is_valid(shdisp_andy_mipi_err_gpio)) {
                SHDISP_ERR("mipi_err gpio not specified");
            } else {
                SHDISP_DEBUG("mipi_err gpio succusess!");
            }
        } else {
            SHDISP_ERR("pdev->dev is NULL");
        }
    }

    SHDISP_TRACE("out ret=%d irq=%d", ret, shdisp_andy_irq_gpio);
#endif /* CONFIG_OF */
    return ret;
}


/* ------------------------------------------------------------------------- */
/*      shdisp_andy_remove                                                   */
/* ------------------------------------------------------------------------- */
static int shdisp_andy_remove(struct platform_device *pdev)
{
    SHDISP_TRACE("in.");
    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

#ifdef CONFIG_OF
static const struct of_device_id shdisp_andy_dt_match[] = {
    { .compatible = "sharp,shdisp_andy", },
    {}
};
#else
#define shdisp_andy_dt_match NULL;
#endif /* CONFIG_OF */

static struct platform_driver shdisp_andy_driver = {
    .probe = shdisp_andy_probe,
    .remove = shdisp_andy_remove,
    .shutdown = NULL,
    .driver = {
        /*
         * Driver name must match the device name added in
         * platform.c.
         */
        .name = "shdisp_andy",
        .of_match_table = shdisp_andy_dt_match,
    },
};

/* ------------------------------------------------------------------------- */
/*      shdisp_andy_register_driver                                          */
/* ------------------------------------------------------------------------- */
static int shdisp_andy_register_driver(void)
{
    SHDISP_TRACE("in.");
    SHDISP_TRACE("out");
    return platform_driver_register(&shdisp_andy_driver);
}

#if defined(CONFIG_ANDROID_ENGINEERING)
/* ------------------------------------------------------------------------- */
/* shdisp_andy_dump_reg                                                      */
/* ------------------------------------------------------------------------- */
static int shdisp_andy_dump_reg(void)
{
    int i, arraysize;
    struct shdisp_dsi_cmd_desc *dumpptr;
    unsigned char addr, page, read_data;

    printk("[SHDISP] PANEL PARAMETER INFO ->>\n");

    printk("[SHDISP] shdisp_panel_ctx.device_code = %d\n", shdisp_panel_ctx.device_code);
    printk("[SHDISP] shdisp_panel_ctx.vcom       = 0x%04X\n", shdisp_panel_ctx.vcom);
    printk("[SHDISP] shdisp_panel_ctx.vcom_low   = 0x%04X\n", shdisp_panel_ctx.vcom_low);
    printk("[SHDISP] shdisp_panel_ctx.vcom_nvram = 0x%04X\n", shdisp_panel_ctx.vcom_nvram);
    printk("[SHDISP] shdisp_panel_ctx.lcddr_phy_gmm.status = %d\n", shdisp_panel_ctx.lcddr_phy_gmm.status);
    printk("[SHDISP] shdisp_panel_ctx.lcddr_phy_gmm.buf = ");
    for (i = 0; i < SHDISP_LCDDR_PHY_GMM_BUF_MAX; i++) {
        printk("%02X,", shdisp_panel_ctx.lcddr_phy_gmm.buf[i]);
    }
    printk("\n");
    printk("[SHDISP] shdisp_panel_ctx.lcddr_phy_gmm.applied_voltage = ");
    for (i = 0; i < SHDISP_LCDDR_APPLIED_VOLTAGE_SIZE; i++) {
        printk("%02X,", shdisp_panel_ctx.lcddr_phy_gmm.applied_voltage[i]);
    }
    printk("\n");
    printk("[SHDISP] shdisp_panel_ctx.lcddr_phy_gmm.chksum = %d\n", shdisp_panel_ctx.lcddr_phy_gmm.chksum);

    arraysize = ARRAY_SIZE(mipi_sh_andy_cmds_initial1);
    dumpptr = mipi_sh_andy_cmds_initial1;
    for (i = 0; i < arraysize; i++) {
        addr = *(dumpptr->payload);
        if (addr == 0xFF) {
            page = *(dumpptr->payload + 1);
            shdisp_andy_API_diag_write_reg(addr, &page, 1);
            printk("[SHDISP] PANEL_PARA_DUMP PAGE: %02X\n", page);
        } else {
            if (dumpptr->dtype == SHDISP_DTYPE_DCS_WRITE1) {
                shdisp_andy_API_diag_read_reg(addr, &read_data, 1);
                printk("[SHDISP] PANEL_PARA_DUMP 0x%02X: %02X\n", addr, read_data);
            }
        }
        dumpptr++;
    }

    arraysize = ARRAY_SIZE(mipi_sh_andy_cmds_initial1_regulator_ts2_0);
    dumpptr = mipi_sh_andy_cmds_initial1_regulator_ts2_0;

    for (i = 0; i < arraysize; i++) {
        addr = *(dumpptr->payload);
        if (addr == 0xFF) {
            page = *(dumpptr->payload + 1);
            shdisp_andy_API_diag_write_reg(addr, &page, 1);
            printk("[SHDISP] PANEL_PARA_DUMP PAGE: %02X\n", page);
        } else {
            if (dumpptr->dtype == SHDISP_DTYPE_DCS_WRITE1) {
                shdisp_andy_API_diag_read_reg(addr, &read_data, 1);
                printk("[SHDISP] PANEL_PARA_DUMP 0x%02X: %02X\n", addr, read_data);
            }
        }
        dumpptr++;
    }

    if (IS_FLICKER_ADJUSTED(shdisp_panel_ctx.vcom_nvram)) {
        arraysize = ARRAY_SIZE(mipi_sh_andy_cmds_initial1_regulator_ts2_0);
        dumpptr = mipi_sh_andy_cmds_initial1_regulator_ts2_0;
    } else {
        arraysize = ARRAY_SIZE(mipi_sh_andy_cmds_initial1_regulator_ts2_0_flicker_unadjusted);
        dumpptr = mipi_sh_andy_cmds_initial1_regulator_ts2_0_flicker_unadjusted;
    }

    for (i = 0; i < arraysize; i++) {
        addr = *(dumpptr->payload);
        if (addr == 0xFF) {
            page = *(dumpptr->payload + 1);
            shdisp_andy_API_diag_write_reg(addr, &page, 1);
            printk("[SHDISP] PANEL_PARA_DUMP PAGE: %02X\n", page);
        } else {
            if (dumpptr->dtype == SHDISP_DTYPE_DCS_WRITE1) {
                shdisp_andy_API_diag_read_reg(addr, &read_data, 1);
                printk("[SHDISP] PANEL_PARA_DUMP 0x%02X: %02X\n", addr, read_data);
            }
        }
        dumpptr++;
    }

    arraysize = ARRAY_SIZE(mipi_sh_andy_cmds_gmm);
    dumpptr = mipi_sh_andy_cmds_gmm;
    for (i = 0; i < arraysize; i++) {
        addr = *(dumpptr->payload);
        if (addr == 0xFF) {
            page = *(dumpptr->payload + 1);
            shdisp_andy_API_diag_write_reg(addr, &page, 1);
            printk("[SHDISP] PANEL_PARA_DUMP PAGE: %02X\n", page);
        } else {
            if (dumpptr->dtype == SHDISP_DTYPE_DCS_WRITE1) {
                shdisp_andy_API_diag_read_reg(addr, &read_data, 1);
                printk("[SHDISP] PANEL_PARA_DUMP 0x%02X: %02X\n", addr, read_data);
            }
        }
        dumpptr++;
    }

    arraysize = ARRAY_SIZE(mipi_sh_andy_cmds_terminal);
    dumpptr = mipi_sh_andy_cmds_terminal;
    for (i = 0; i < arraysize; i++) {
        addr = *(dumpptr->payload);
        if (addr == 0xFF) {
            page = *(dumpptr->payload + 1);
            shdisp_andy_API_diag_write_reg(addr, &page, 1);
            printk("[SHDISP] PANEL_PARA_DUMP PAGE: %02X\n", page);
        } else {
            if (dumpptr->dtype == SHDISP_DTYPE_DCS_WRITE1) {
                shdisp_andy_API_diag_read_reg(addr, &read_data, 1);
                printk("[SHDISP] PANEL_PARA_DUMP 0x%02X: %02X\n", addr, read_data);
            }
        }
        dumpptr++;
    }

    arraysize = ARRAY_SIZE(mipi_sh_andy_cmds_timing);
    dumpptr = mipi_sh_andy_cmds_timing;
    for (i = 0; i < arraysize; i++) {
        addr = *(dumpptr->payload);
        if (addr == 0xFF) {
            page = *(dumpptr->payload + 1);
            shdisp_andy_API_diag_write_reg(addr, &page, 1);
            printk("[SHDISP] PANEL_PARA_DUMP PAGE: %02X\n", page);
        } else {
            if (dumpptr->dtype == SHDISP_DTYPE_DCS_WRITE1) {
                shdisp_andy_API_diag_read_reg(addr, &read_data, 1);
                printk("[SHDISP] PANEL_PARA_DUMP 0x%02X: %02X\n", addr, read_data);
            }
        }
        dumpptr++;
    }

    arraysize = ARRAY_SIZE(mipi_sh_andy_cmds_initial2);
    dumpptr = mipi_sh_andy_cmds_initial2;
    for (i = 0; i < arraysize; i++) {
        addr = *(dumpptr->payload);
        if (addr == 0xFF) {
            page = *(dumpptr->payload + 1);
            shdisp_andy_API_diag_write_reg(addr, &page, 1);
            printk("[SHDISP] PANEL_PARA_DUMP PAGE: %02X\n", page);
        } else {
            if (dumpptr->dtype == SHDISP_DTYPE_DCS_WRITE1) {
                shdisp_andy_API_diag_read_reg(addr, &read_data, 1);
                printk("[SHDISP] PANEL_PARA_DUMP 0x%02X: %02X\n", addr, read_data);
            }
        }
        dumpptr++;
    }

    arraysize = ARRAY_SIZE(mipi_sh_andy_cmds_power);
    dumpptr = mipi_sh_andy_cmds_power;
    for (i = 0; i < arraysize; i++) {
        addr = *(dumpptr->payload);
        if (addr == 0xFF) {
            page = *(dumpptr->payload + 1);
            shdisp_andy_API_diag_write_reg(addr, &page, 1);
            printk("[SHDISP] PANEL_PARA_DUMP PAGE: %02X\n", page);
        } else {
            if (dumpptr->dtype == SHDISP_DTYPE_DCS_WRITE1) {
                shdisp_andy_API_diag_read_reg(addr, &read_data, 1);
                printk("[SHDISP] PANEL_PARA_DUMP 0x%02X: %02X\n", addr, read_data);
            }
        }
        dumpptr++;
    }

    arraysize = ARRAY_SIZE(mipi_sh_andy_cmds_sync_ts2_0);
    dumpptr = mipi_sh_andy_cmds_sync_ts2_0;

    for (i = 0; i < arraysize; i++) {
        addr = *(dumpptr->payload);
        if (addr == 0xFF) {
            page = *(dumpptr->payload + 1);
            shdisp_andy_API_diag_write_reg(addr, &page, 1);
            printk("[SHDISP] PANEL_PARA_DUMP PAGE: %02X\n", page);
        } else {
            if (dumpptr->dtype == SHDISP_DTYPE_DCS_WRITE1) {
                shdisp_andy_API_diag_read_reg(addr, &read_data, 1);
                printk("[SHDISP] PANEL_PARA_DUMP 0x%02X: %02X\n", addr, read_data);
            }
        }
        dumpptr++;
    }

    printk("[SHDISP] PANEL PARAMETER INFO <<-\n");
    return SHDISP_RESULT_SUCCESS;
}
#endif /* CONFIG_ANDROID_ENGINEERING */

MODULE_DESCRIPTION("SHARP DISPLAY DRIVER MODULE");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.00");

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
