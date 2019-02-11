/* include/sharp/sh_smem.h
 *
 * Copyright (C) 2015 Sharp Corporation
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

#ifndef __SH_SMEM_H__
#define __SH_SMEM_H__

/* 
 * AMSS/LINUX common SH_SMEM structure definition
 * 
 * [The file path which must synchronize]
 *  adsp_proc/core/sharp/shsys/inc/sh_smem.h
 *  boot_images/core/sharp/shsys/inc/sh_smem.h
 *  LINUX/android/kernel/include/sharp/sh_smem.h
 *  modem_proc/core/sharp/shsys/inc/sh_smem.h
 */
typedef struct 
{
    uint32_t            shdisp_data_buf[548];        /* Buffer for shdisp */
    unsigned char       shusb_softupdate_mode_flag;  /* softupdate mode flag */
    uint32_t            sh_filesystem_init;          /* file system innitialize flag */
    int                 sh_sleep_test_mode;          /* sleep test mode flag */
    unsigned char       shusb_qxdm_ena_flag;         /* QXDM enable flag */
    unsigned char       shusb_usb_charge_ena_flag;   /* USB charge enable flag */
    uint32_t            fota_boot_mode;              /* FOTA mode information */
    unsigned char       conf_clrvari[4];             /* Color Variations information */
    uint32_t            shdiag_FlagData;             /* shdiag flag information */
    unsigned short      shdiag_BootMode;             /* shdiag Powerup mode */
    unsigned char       shdiag_FirstBoot;            /* shdiag FirstBoot information */
    unsigned char       shdiag_AdjChashe[16];        /* shdiag Adj chashe information */
    unsigned short      shdiag_TpsBaseLineTbl[1000]; /* Touch adjustment */
#if defined(FEATURE_SH_MODEL_DL70) || defined(FEATURE_SH_MODEL_AL20) || defined(CONFIG_ARCH_LYNX_DL70) || defined(CONFIG_ARCH_DECKARD_AL20)
    unsigned char       sh_100hflg;                  /* 100 hours test flag */
#endif /* defined(FEATURE_SH_MODEL_DL70) || defined(FEATURE_SH_MODEL_AL20) || defined(CONFIG_ARCH_LYNX_DL70) || defined(CONFIG_ARCH_DECKARD_AL20) */
    unsigned short      shdiag_proxadj[2];           /* Proximity sensor adjust */
    unsigned char       shdiag_fullchgflg;           /* Full charge FLG(F Only) */
#if defined(FEATURE_SH_MODEL_DL70) || defined(FEATURE_SH_MODEL_AL20) || defined(CONFIG_ARCH_LYNX_DL70) || defined(CONFIG_ARCH_DECKARD_AL20)
    char                shdiag_debugflg;             /* Debug FLG */
    char                shdiag_factoryflg;           /* Factory FLG */
#else /* defined(FEATURE_SH_MODEL_DL70) || defined(FEATURE_SH_MODEL_AL20) || defined(CONFIG_ARCH_LYNX_DL70) || defined(CONFIG_ARCH_DECKARD_AL20) */
    unsigned char       shdiag_debugflg;             /* Debug FLG */
    unsigned char       shdiag_factoryflg;           /* Factory FLG */
#endif /* defined(FEATURE_SH_MODEL_DL70) || defined(FEATURE_SH_MODEL_AL20) || defined(CONFIG_ARCH_LYNX_DL70) || defined(CONFIG_ARCH_DECKARD_AL20) */
    uint64_t            shsys_timestamp[32];         /* System Timestamp */
    uint32_t            sh_hw_revision;              /* hardware revision number */
    unsigned char       sh_hw_handset;               /* Handset FLG */
    uint32_t            sh_boot_mode;                /* power up mode information */
    uint32_t            sh_boot_key;                 /* key(s) ditected OSBL */
    uint32_t            sh_pwr_on_status;            /* power on status information from pmic */
    char                sh_sbl_version[8];           /* sbl Version */
    unsigned char       pImeiData[16];               /* W-CDMA Imei data */
    unsigned char       shdiag_tspdrv_acal_data[3];  /* Haptics LRA control IC AutoCalibration result */
    unsigned char       sh_pvs_flg;                  /* PVS flag */
    int                 shpwr_battery_present;       /* PWR:battery present */
    int                 shpwr_battery_voltage;       /* PWR:battery voltage */
    int                 shpwr_battery_temperature;   /* PWR:battery temperature */
    int                 shpwr_cable_status;          /* PWR:cable status */
    int                 shpwr_fuel_data[4];          /* PWR:fuel gauge correction value */
    int                 shpwr_vbat_data[4];          /* PWR:battery A/D converter  correction value */
    unsigned char       shpwr_batauthflg;            /* PWR:battery ID Authentication plug infomation */
    unsigned char       sh_camOtpData[16384];        /* Camera Production adjustment Data */
    unsigned char       shdiag_charge_th_high[8];    /* ChageLimitMax */
    unsigned char       shdiag_charge_th_low[8];     /* ChageLimitMin */
    unsigned char       shdarea_WlanMacAddress[6];   /* WLAN Mac Address */
    unsigned char       shtps_fwup_flag;             /* Touch panel firmware update flag */
    unsigned short      shdiag_cprx_offset_hl[12][2];/* CDC Proximity Sensor High/Low Offset Data */
    unsigned short      shdiag_cprx_cal_offset[12];  /* CDC Proximity Sensor Calibration Offset Data */
    unsigned char       shpwr_misc_param[21];        /* PWR:charge parameter */
    unsigned char       shpwr_traceability[22];      /* PWR:battery traceability */
    unsigned char       conf_mvno[2];                /* MVcode */
    unsigned char       shdiag_vib_param[9];         /* LINEAR VIBRATOR */
    unsigned short      sh_boot_hookmode;            /* kmsg hook mode */
#if !defined(FEATURE_SH_MODEL_DL70) && !defined(FEATURE_SH_MODEL_AL20) && !defined(CONFIG_ARCH_LYNX_DL70) && !defined(CONFIG_ARCH_DECKARD_AL20)
    unsigned short      sh_proxgrip_lth[4];          /* Grip Sensor LOW threshold */
    unsigned short      sh_proxgrip_hth[4];          /* Grip Sensor HIGH threshold */
    unsigned short      sh_proxgrip_pomax[4];        /* Grip Sensor Count Value Open MAX*/
    unsigned short      sh_proxgrip_poave[4];        /* Grip Sensor Count Value Open AVE*/
    unsigned short      sh_proxgrip_pomin[4];        /* Grip Sensor Count Value Open MIN*/
    unsigned short      sh_proxgrip_ptmax[4];        /* Grip Sensor Count Value Touch MAX*/
    unsigned short      sh_proxgrip_ptave[4];        /* Grip Sensor Count Value Touch AVE*/
    unsigned short      sh_proxgrip_ptmin[4];        /* Grip Sensor Count Value Touch MIN*/
    unsigned short      sh_proxgrip_a[4];            /* Grip Sensor Alpha */
    unsigned short      sh_proxgrip_b[4];            /* Grip Sensor Beta */
    unsigned short      sh_proxgrip_c[4];            /* Grip Sensor Gamma */
    unsigned char       sh_proxgrip_secid[10];       /* Grip Sensor Security ID */
    unsigned short      sh_proxgrip_p3max[2];        /* Grip Sensor Count Value 3mm MAX*/
    unsigned short      sh_proxgrip_p3ave[2];        /* Grip Sensor Count Value 3mm AVE*/
    unsigned short      sh_proxgrip_p3min[2];        /* Grip Sensor Count Value 3mm MIN*/
#endif /* !defined(FEATURE_SH_MODEL_DL70) && !defined(FEATURE_SH_MODEL_AL20) && !defined(CONFIG_ARCH_LYNX_DL70) && !defined(CONFIG_ARCH_DECKARD_AL20) */
} sharp_smem_common_type;

#define SH_SMEM_COMMON_SIZE 256000


/*=============================================================================

FUNCTION sh_smem_get_common_address

=============================================================================*/
sharp_smem_common_type *sh_smem_get_common_address( void );

/*=============================================================================

FUNCTION sh_smem_get_sleep_power_collapse_disabled_address

=============================================================================*/
unsigned long *sh_smem_get_sleep_power_collapse_disabled_address( void );

/*=============================================================================

FUNCTION sh_smem_get_softupdate_flg

=============================================================================*/
unsigned char sh_smem_get_softupdate_flg( void );

/*=============================================================================

FUNCTION sh_smem_get_fota_boot_mode

=============================================================================*/
unsigned long sh_smem_get_fota_boot_mode( void );

/*=============================================================================

FUNCTION sh_smem_get_pvs_flg

=============================================================================*/
unsigned char sh_smem_get_pvs_flg( void );

#endif // __SH_SMEM_H__
