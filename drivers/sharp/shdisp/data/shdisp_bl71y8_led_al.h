/* drivers/sharp/shdisp/data/shdisp_bl71y8_led_al.h  (Display Driver)
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
#ifndef SHDISP_BL71Y8_LED_AL_H
#define SHDISP_BL71Y8_LED_AL_H

/* ------------------------------------------------------------------------- */
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/* MACROS(Register Value)                                                    */
/* ------------------------------------------------------------------------- */
#define BDIC_REG_CH3_A_VAL_FIX                      (0x1E)
#define BDIC_REG_CH3_B_VAL_FIX                      (0x00)
#define BDIC_REG_CH3_A_VAL_ANI                      (0x00)
#define BDIC_REG_CH3_B_VAL_ANI                      (0x1E)
#define BDIC_REG_CH3_C_VAL                          (0x00)
/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */
#define SHDISP_RGB                                  (3)
#define SHDISP_TRI_LED_COLOR_TBL_NUM                (10)
#define SHDISP_COL_VARI_KIND                        (3)
#define SHDISP_HANDSET_COLOR_WHITE                  (0x01)
#define SHDISP_HANDSET_COLOR_PINK                   (0x02)
#define SHDISP_HANDSET_COLOR_GREEN                  (0x05)
#define SHDISP_TRI_LED_ANIME_2PAGE                  (2)
#define SHDISP_TRI_LED_ANIME_3PAGE                  (3)

#ifdef SHDISP_ANIME_COLOR_LED
#define SHDISP_BDIC_TRI_LED_INTERVAL_EMOPATTERN     (0)
#define SHDISP_BDIC_TRI_LED_COUNT_EMOPATTERN        (0)
#define SHDISP_BDIC_TRI_LED_COLOR_WHITE             (7)
#define SHDISP_BDIC_TRI_LED_COLOR_MAGENTA           (5)
#endif /* SHDISP_ANIME_COLOR_LED */

/* ------------------------------------------------------------------------- */
/* MACROS(Register Value)                                                    */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/* VARIABLES                                                                 */
/* ------------------------------------------------------------------------- */
static const struct shdisp_bdic_led_color_index shdisp_triple_led_color_index_tbl[SHDISP_TRI_LED_COLOR_TBL_NUM] = {
    {0, 0, 0,  0},
    {1, 0, 0,  1},
    {0, 1, 0,  2},
    {1, 1, 0,  3},
    {0, 0, 1,  4},
    {1, 0, 1,  5},
    {0, 1, 1,  6},
    {1, 1, 1,  7},
    {2, 0, 0,  8},
    {0, 2, 0,  9}
};

static const unsigned char shdisp_clrvari_index[SHDISP_COL_VARI_KIND] = {
    SHDISP_HANDSET_COLOR_WHITE,
    SHDISP_HANDSET_COLOR_PINK,
    SHDISP_HANDSET_COLOR_GREEN
};

static const unsigned char shdisp_triple_led_tbl[SHDISP_COL_VARI_KIND][SHDISP_TRI_LED_COLOR_TBL_NUM][3] = {
  {
    { 0x00, 0x00, 0x00 },
    { 0x50, 0x00, 0x00 },
    { 0x00, 0x60, 0x00 },
    { 0x30, 0x30, 0x00 },
    { 0x00, 0x00, 0x7F },
    { 0x50, 0x00, 0x50 },
    { 0x00, 0x50, 0x50 },
    { 0x40, 0x40, 0x40 },
    { 0x50, 0x00, 0x00 },
    { 0x00, 0x60, 0x00 }
  },
  {
    { 0x00, 0x00, 0x00 },
    { 0x50, 0x00, 0x00 },
    { 0x00, 0x60, 0x00 },
    { 0x30, 0x50, 0x00 },
    { 0x00, 0x00, 0x7F },
    { 0x50, 0x00, 0x50 },
    { 0x00, 0x50, 0x50 },
    { 0x30, 0x50, 0x50 },
    { 0x50, 0x00, 0x00 },
    { 0x00, 0x60, 0x00 }
  },
  {
    { 0x00, 0x00, 0x00 },
    { 0x10, 0x00, 0x00 },
    { 0x00, 0x08, 0x00 },
    { 0x0C, 0x08, 0x00 },
    { 0x00, 0x00, 0x10 },
    { 0x10, 0x00, 0x08 },
    { 0x00, 0x08, 0x08 },
    { 0x08, 0x08, 0x08 },
    { 0x10, 0x00, 0x00 },
    { 0x00, 0x08, 0x00 }
  }
};

static const unsigned char shdisp_triple_led_anime_tbl[SHDISP_COL_VARI_KIND][SHDISP_TRI_LED_ANIME_2PAGE][SHDISP_TRI_LED_COLOR_TBL_NUM][3] = {
  {
    {
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 }
    },
    {
        { 0x00, 0x00, 0x00 },
        { 0x50, 0x00, 0x00 },
        { 0x00, 0x60, 0x00 },
        { 0x30, 0x30, 0x00 },
        { 0x00, 0x00, 0x7F },
        { 0x50, 0x00, 0x50 },
        { 0x00, 0x50, 0x50 },
        { 0x40, 0x40, 0x40 },
        { 0x50, 0x00, 0x00 },
        { 0x00, 0x60, 0x00 }
    }
  },
  {
    {
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 }
    },
    {
        { 0x00, 0x00, 0x00 },
        { 0x50, 0x00, 0x00 },
        { 0x00, 0x60, 0x00 },
        { 0x30, 0x50, 0x00 },
        { 0x00, 0x00, 0x7F },
        { 0x50, 0x00, 0x50 },
        { 0x00, 0x50, 0x50 },
        { 0x30, 0x50, 0x50 },
        { 0x50, 0x00, 0x00 },
        { 0x00, 0x60, 0x00 }
    }
  },
  {
    {
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 },
        { 0x00, 0x00, 0x00 }
    },
    {
        { 0x00, 0x00, 0x00 },
        { 0x10, 0x00, 0x00 },
        { 0x00, 0x08, 0x00 },
        { 0x0C, 0x08, 0x00 },
        { 0x00, 0x00, 0x10 },
        { 0x10, 0x00, 0x08 },
        { 0x00, 0x08, 0x08 },
        { 0x08, 0x08, 0x08 },
        { 0x10, 0x00, 0x00 },
        { 0x00, 0x08, 0x00 }
    }
  }
};

#ifdef SHDISP_ANIME_COLOR_LED
static const unsigned char shdisp_triple_led_anime_emopattern_tbl[SHDISP_COL_VARI_KIND][3][3] = {
    {
        { 0x14, 0x00, 0x14 },
        { 0x35, 0x00, 0x35 },
        { 0x50, 0x00, 0x50 }
    },
    {
        { 0x14, 0x00, 0x14 },
        { 0x35, 0x00, 0x35 },
        { 0x50, 0x00, 0x50 }
    },
    {
        { 0x04, 0x00, 0x02 },
        { 0x0B, 0x00, 0x05 },
        { 0x10, 0x00, 0x08 }
    }
};
#endif /* SHDISP_ANIME_COLOR_LED */

/* ------------------------------------------------------------------------- */
/* TYPES                                                                     */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */

#endif /* SHDISP_BL71Y8_LED_AL_H */

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
