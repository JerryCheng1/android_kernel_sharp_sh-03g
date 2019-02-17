/* drivers/sharp/shtps/sy3000/shtps_cfg.h
 *
 * Copyright (c) 2015, Sharp. All rights reserved.
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
#ifndef __SHTPS_CFG_H__
#define __SHTPS_CFG_H__
/* --------------------------------------------------------------------------- */

#if 0	/** For build test */
	#undef CONFIG_SHTPS_SY3000_PLG541_001_1
	#undef CONFIG_SHTPS_SY3000_PLG541_001_2
	#undef CONFIG_SHTPS_SY3000_PLG543_001_1
	#undef CONFIG_SHTPS_SY3000_PLG543_001_2
	#undef CONFIG_SHTPS_SY3000_PLG543_001_3
	#undef CONFIG_SHTPS_SY3000_PLG543_001_4

	#define CONFIG_SHTPS_SY3000_PLG541_001_1
#endif

#include <sharp/shtps_dev.h>
/* --------------------------------------------------------------------------- */
#if defined(CONFIG_SHTPS_SY3000_PLG541_001_1)
	#include "plg541-001-1/shtps_cfg_plg541-001-1.h"

#elif defined(CONFIG_SHTPS_SY3000_PLG541_001_2)
	#include "plg541-001-2/shtps_cfg_plg541-001-2.h"

#elif defined(CONFIG_SHTPS_SY3000_PLG543_001_1)
	#include "plg543-001-1/shtps_cfg_plg543-001-1.h"

#elif defined(CONFIG_SHTPS_SY3000_PLG543_001_2)
	#include "plg543-001-2/shtps_cfg_plg543-001-2.h"

#elif defined(CONFIG_SHTPS_SY3000_PLG543_001_3)
	#include "plg543-001-3/shtps_cfg_plg543-001-3.h"

#elif defined(CONFIG_SHTPS_SY3000_PLG543_001_4)
	#include "plg543-001-4/shtps_cfg_plg543-001-4.h"

#else
	#include "plg541-001-1/shtps_cfg_plg541-001-1.h"

#endif

/* --------------------------------------------------------------------------- */
#endif	/* __SHTPS_CFG_H__ */

