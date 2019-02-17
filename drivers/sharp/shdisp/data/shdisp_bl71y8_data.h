/* drivers/sharp/shdisp/data/shdisp_bl71y8_data.h  (Display Driver)
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

#ifndef SHDISP_BL71Y8_DATA_H
#define SHDISP_BL71Y8_DATA_H
/* ------------------------------------------------------------------------- */
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */
#if defined(SHDISP_MODEL_FS)
#include "shdisp_bl71y8_data_fs.h"
#elif defined(SHDISP_MODEL_MID)
 #if defined(SHDISP_AL)
#include "shdisp_bl71y8_data_mid_al.h"
 #else
#include "shdisp_bl71y8_data_mid_dl.h"
 #endif /* SHDISP_AL */
#else
 #if defined(SHDISP_DL)
#include "shdisp_bl71y8_data_dl.h"
 #elif defined(SHDISP_AL)
#include "shdisp_bl71y8_data_al.h"
 #else
#include "shdisp_bl71y8_data_default.h"
 #endif /* SHDISP_DL */
#endif /* SHDISP_MODEL_FS SHDISP_MODEL_MID */

#endif /* SHDISP_BL71Y8_DATA_H */

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
