/* drivers/leds/leds-gpio-ctrl.c  (LED Driver)
 *
 * Copyright (C) 2015 SHARP CORPORATION
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
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/leds.h>

#include "leds-gpio-ctrl.h"
#include "leds.h"

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/* VARIABLES                                                                 */
/* ------------------------------------------------------------------------- */
#ifdef CONFIG_SHDISP /* CUST_ID_00057 */
static int leds_dis_sw_gpio  = 0;
static int leds_sel_pmi_gpio = 0;

static int leds_dis_sw  = 0;
static int leds_sel_pmi = 0;

static char dis_sw_name[]  = "LEDS_DIS_SW";
static char sel_pmi_name[] = "LEDS_SEL_PMI";
#endif /* CONFIG_SHDISP */

/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */
#ifdef CONFIG_SHDISP /* CUST_ID_00057 */
static void led_gpio_ctrl_initialize(void);
static int led_gpio_ctrl_register_driver(void);
#endif /* CONFIG_SHDISP */

/* ------------------------------------------------------------------------- */
/* FUNCTIONS                                                                 */
/* ------------------------------------------------------------------------- */
#ifdef CONFIG_SHDISP /* CUST_ID_00057 */
/* ------------------------------------------------------------------------- */
/* led_gpio_set_value                                                        */
/* ------------------------------------------------------------------------- */
int led_gpio_set_value(int gpio, int value)
{
	int ret = 0;
	int gpio_no;
	char *gpio_name;

	pr_debug("[leds]%s() in: gpio(%d), value(%d)\n", __func__, gpio, value);

	if ((value != LEDS_GPIO_HIGH) && (value != LEDS_GPIO_LOW)) {
		pr_debug("[leds]%s(): <error> value(%d) \n", __func__, value);
		return -EIO;
	}

	switch (gpio) {
	case LEDS_DIS_SW_GPIO:
		if (value == leds_dis_sw) {
			return 0;
		}
		leds_dis_sw = value;
		gpio_no = leds_dis_sw_gpio;
		gpio_name = dis_sw_name;
		break;

	case LEDS_SEL_PMI_GPIO:
		if (value == leds_sel_pmi) {
			return 0;
		}
		leds_sel_pmi = value;
		gpio_no = leds_sel_pmi_gpio;
		gpio_name = sel_pmi_name;
		break;

	default:
		pr_debug("[leds] <error> gpio(%d) \n", gpio);
		return -EIO;
	}

	if (gpio_no == 0) {
		pr_debug("[leds] <error> not get gpio_no \n");
		return -EIO;
	}

	if (value == LEDS_GPIO_HIGH) {
		gpio_request(gpio_no, gpio_name);
	}
	gpio_set_value(gpio_no, value);
	if (value == LEDS_GPIO_LOW) {
		gpio_free(gpio_no);
	}

	pr_debug("[leds]%s() out\n", __func__);
	return ret;
}

/* ------------------------------------------------------------------------- */
/* led_gpio_ctrl_initialize                                                  */
/* ------------------------------------------------------------------------- */
static void led_gpio_ctrl_initialize(void)
{
	pr_debug("[leds]%s() in\n", __func__);

	if (leds_dis_sw_gpio != 0) {
		gpio_request(leds_dis_sw_gpio, dis_sw_name);
		leds_dis_sw = gpio_get_value(leds_dis_sw_gpio);
		pr_debug("[leds]%s(): GPIO_no(%d), name(%s) read leds_dis_sw(%d)\n", __func__, leds_dis_sw_gpio, dis_sw_name, leds_dis_sw);
		if (leds_dis_sw == LEDS_GPIO_LOW) {
			gpio_free(leds_dis_sw_gpio);
		}
	} else {
		leds_dis_sw  = LEDS_GPIO_LOW;
		pr_err("[leds] <error> not get dis_sw GPIO no \n");
	}

	if (leds_sel_pmi_gpio != 0) {
		gpio_request(leds_sel_pmi_gpio, sel_pmi_name);
		leds_sel_pmi = gpio_get_value(leds_sel_pmi_gpio);
		pr_debug("[leds]%s(): GPIO_no(%d), name(%s) read leds_sel_pmi(%d)\n", __func__, leds_sel_pmi_gpio, sel_pmi_name, leds_sel_pmi);
		if (leds_sel_pmi == LEDS_GPIO_LOW) {
			gpio_free(leds_sel_pmi_gpio);
		}
	} else {
		leds_sel_pmi = LEDS_GPIO_HIGH;
		pr_err("[leds] <error> not get sel_pmi GPIO no \n");
	}

	pr_debug("[leds]%s() out : DIS_SW(%d), SEL_PMI(%d)\n", __func__, leds_dis_sw, leds_sel_pmi);
}

/* ------------------------------------------------------------------------- */
/* leds_gpio_probe                                                           */
/* ------------------------------------------------------------------------- */
static int leds_gpio_probe(struct platform_device *pdev)
{
	int rc = 0;

	if (pdev) {
		if (&(pdev->dev) != NULL) {
			leds_sel_pmi_gpio = of_get_named_gpio(pdev->dev.of_node, "pierce_sel_pmi_gpio", 0);
			if (!gpio_is_valid(leds_sel_pmi_gpio)) {
				pr_err("[leds] PIERCE_SEL_PMI gpio not specified \n");
			} else {
				pr_debug("[leds] PIERCE_SEL_PMI gpio succusess! (%d)\n", leds_sel_pmi_gpio);
			}

			leds_dis_sw_gpio = of_get_named_gpio(pdev->dev.of_node, "pierce_dis_sw_gpio", 0);
			if (!gpio_is_valid(leds_dis_sw_gpio)) {
				pr_err("[leds] DIS_SW gpio not specified \n");
			} else {
				pr_debug("[leds] DIS_SW gpio succusess! (%d)\n", leds_dis_sw_gpio);
			}
		} else {
			pr_err("[leds] pdev->dev is NULL \n");
		}
	} else {
		pr_err("[leds] pdev is NULL \n");
	}

	return rc;
}

static const struct of_device_id leds_gpio_match[] = {
	{ .compatible = "sharp,shdisp_leds_gpio", },
	{}
};

static struct platform_driver leds_gpio_driver = {
	.probe = leds_gpio_probe,
	.remove = NULL,
	.shutdown = NULL,
	.driver = {
		/*
		 * Driver name must match the device name added in
		 * platform.c.
		 */
		.name = "shdisp_leds_gpio",
		.of_match_table = leds_gpio_match,
	},
};

/* ------------------------------------------------------------------------- */
/* led_gpio_ctrl_register_driver                                             */
/* ------------------------------------------------------------------------- */
static int led_gpio_ctrl_register_driver(void)
{
	return 	platform_driver_register(&leds_gpio_driver);
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_register_driver                                               */
/* ------------------------------------------------------------------------- */
static int __init leds_gpio_ctrl_init(void)
{
	led_gpio_ctrl_register_driver();

	led_gpio_ctrl_initialize();

	return 0;
}
module_init(leds_gpio_ctrl_init);

/* ------------------------------------------------------------------------- */
/* leds_gpio_ctrl_exit                                                       */
/* ------------------------------------------------------------------------- */
static void leds_gpio_ctrl_exit(void)
{
	platform_driver_unregister(&leds_gpio_driver);
}
module_exit(leds_gpio_ctrl_exit);
#endif /* CONFIG_SHDISP */

MODULE_DESCRIPTION("SHARP LED GPIO MODULE");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.00");

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
