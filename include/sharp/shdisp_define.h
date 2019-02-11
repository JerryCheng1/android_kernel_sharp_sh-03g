/* include/sharp/shdisp_define.h  (Display Driver)
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

#ifndef SHDISP_DEFINE_H
#define SHDISP_DEFINE_H

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
#define SHDISP_LOWBKL

#define SHDISP_SYSFS_LED
/* ------------------------------------------------------------------------- */
#if defined(CONFIG_ARCH_LYNX_DL80) || defined(FEATURE_SH_MODEL_DL80)
#define SHDISP_COLOR_LED_TWIN
#define SHDISP_DL
#define SHDISP_MODEL_FS
#define SHDISP_BDIC_PROHIBIT

#define SHDISP_ALS_INT
#define SHDISP_LED_INT
#define SHDISP_ANIME_COLOR_LED
#define SHDISP_ILLUMI_COLOR_LED

#define USER_CONFIG_SHDISP_PANEL_HAYABUSA
#define SHDISP_MAIN_WIDTH  1080
#define SHDISP_MAIN_HEIGHT 1920
/* ------------------------------------------------------------------------- */
#elif defined(CONFIG_ARCH_LYNX_DL85) || defined(FEATURE_SH_MODEL_DL85)
#if defined(CONFIG_ARCH_LYNX_DL83) || defined(FEATURE_SH_MODEL_DL83)
#define SHDISP_EXTEND_COLOR_LED
#define SHDISP_ILLUMI_TRIPLE_COLOR_LED
#endif /* CONFIG_ARCH_LYNX_DL83 */
#define SHDISP_COLOR_LED_TWIN
#define SHDISP_DL
#define SHDISP_MODEL_MID
#define SHDISP_BDIC_PROHIBIT

#define SHDISP_ALS_INT
#define SHDISP_LED_INT
#define SHDISP_ANIME_COLOR_LED
#define SHDISP_ILLUMI_COLOR_LED

#define USER_CONFIG_SHDISP_PANEL_HAYABUSA
#define SHDISP_MAIN_WIDTH  1080
#define SHDISP_MAIN_HEIGHT 1920
/* ------------------------------------------------------------------------- */
#elif defined(CONFIG_ARCH_DECKARD_AL25) || defined(FEATURE_SH_MODEL_AL25)
#define SHDISP_COLOR_LED_TWIN
#define SHDISP_AL
#define SHDISP_MODEL_MID

#define SHDISP_ALS_INT
#define SHDISP_LED_INT
#define SHDISP_ANIME_COLOR_LED
#define SHDISP_ILLUMI_COLOR_LED

#define USER_CONFIG_SHDISP_PANEL_HAYABUSA
#define SHDISP_MAIN_WIDTH  1080
#define SHDISP_MAIN_HEIGHT 1920
/* ------------------------------------------------------------------------- */
#elif defined(CONFIG_ARCH_PA30) || defined(FEATURE_SH_MODEL_PA30)
#define SHDISP_COLOR_LED_TWIN
#define SHDISP_PA
#define SHDISP_MODEL_FS

#define SHDISP_ALS_INT
#define SHDISP_LED_INT
#define SHDISP_ANIME_COLOR_LED
#define SHDISP_ILLUMI_COLOR_LED

#define USER_CONFIG_SHDISP_PANEL_HAYABUSA
#define SHDISP_MAIN_WIDTH  1080
#define SHDISP_MAIN_HEIGHT 1920
/* ------------------------------------------------------------------------- */
#elif defined(CONFIG_ARCH_PA31) || defined(FEATURE_SH_MODEL_PA31)
#define SHDISP_COLOR_LED_TWIN
#define SHDISP_PA
#define SHDISP_MODEL_MID

#define SHDISP_ALS_INT
#define SHDISP_LED_INT
#define SHDISP_ANIME_COLOR_LED
#define SHDISP_ILLUMI_COLOR_LED

#define USER_CONFIG_SHDISP_PANEL_HAYABUSA
#define SHDISP_MAIN_WIDTH  1080
#define SHDISP_MAIN_HEIGHT 1920
/* ------------------------------------------------------------------------- */
#elif defined(CONFIG_SHDISP_PANEL_ANDY) || defined(FEATURE_SHDISP_PANEL_ANDY)
#if defined(CONFIG_ARCH_LYNX_DL70) || defined(FEATURE_SH_MODEL_DL70)
#define SHDISP_COLOR_LED_TWIN
#define SHDISP_DL
#elif defined(CONFIG_ARCH_DECKARD_AL20) || defined(FEATURE_SH_MODEL_AL20)
#define SHDISP_AL
#endif /* CONFIG_ARCH_LYNX_DL70 || FEATURE_SH_MODEL_DL70 */

#define SHDISP_ALS_INT
#define SHDISP_LED_INT
#define SHDISP_ANIME_COLOR_LED
#if defined(CONFIG_ARCH_LYNX_DL70) || defined(FEATURE_SH_MODEL_DL70)
#define SHDISP_ILLUMI_COLOR_LED
#endif /* CONFIG_ARCH_LYNX_DL70 || FEATURE_SH_MODEL_DL70 */

#define USER_CONFIG_SHDISP_PANEL_ANDY
#define USER_CONFIG_SHDISP_USE_FALCON
#define SHDISP_DISPLAY_INFO
#define SHDISP_MAIN_WIDTH  1080
#define SHDISP_MAIN_HEIGHT 1920
/* ------------------------------------------------------------------------- */
#else /* CONFIG_ARCH_XXX || FEATURE_SH_MODEL_XXX */
#define SHDISP_COLOR_LED_TWIN
#define SHDISP_DL
#define SHDISP_MODEL_FS

#define SHDISP_ALS_INT
#define SHDISP_LED_INT
#define SHDISP_ANIME_COLOR_LED
#define SHDISP_ILLUMI_COLOR_LED

#define USER_CONFIG_SHDISP_PANEL_HAYABUSA
#define SHDISP_MAIN_WIDTH  1080
#define SHDISP_MAIN_HEIGHT 1920
/* ------------------------------------------------------------------------- */
#endif /* CONFIG_ARCH_XXX || FEATURE_SH_MODEL_XXX */

#if defined(USER_CONFIG_SHDISP_PANEL_HAYABUSA) || defined(CONFIG_SHDISP_PANEL_HAYABUSA)
#define SHDISP_TRV_NM2
#endif  /* defined(USER_CONFIG_SHDISP_PANEL_HAYABUSA) || defined(CONFIG_SHDISP_PANEL_HAYABUSA) */

#ifdef USER_CONFIG_SHDISP_PANEL_HAYABUSA
#define SHDISP_FPS_HIGH_ENABLE
#endif  /* USER_CONFIG_SHDISP_PANEL_HAYABUSA */


#ifndef CONFIG_SHDISP_PANEL_GEMINI
#if defined(SHDISP_FACTORY_MODE_ENABLE)
#if !defined(SHDISP_NOT_SUPPORT_DET)
#define SHDISP_NOT_SUPPORT_DET
#endif  /* !defined(SHDISP_NOT_SUPPORT_DET) */
#endif  /* defined(SHDISP_FACTORY_MODE_ENABLE) */
#endif  /* CONFIG_SHDISP_PANEL_GEMINI */

#endif /* SHDISP_DEFINE_H */
/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
