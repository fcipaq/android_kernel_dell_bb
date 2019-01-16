/*
 * Intel Baytrail PWM ACPI driver.
 *
 * Copyright (C) 2013 Intel corporation.
 *
 * ----------------------------------------------------------------------------
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 * ----------------------------------------------------------------------------
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include <linux/pwm.h>
#include <linux/platform_device.h>
#include <linux/acpi.h>
#include "pwm_byt_core.h"

#ifdef CONFIG_ACPI
static const struct acpi_device_id pwm_byt_acpi_ids[] = {
	{ "80860F09", 0 },
	{ "80862288", 0 },
	{ }
};
MODULE_DEVICE_TABLE(acpi, pwm_byt_acpi_ids);
#endif

static int pwm_byt_plat_probe(struct platform_device *pdev)
{
	static int pwm_num;
	struct resource *mem, *ioarea;
	void __iomem *base;
	int r;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_err(&pdev->dev, "no mem resource?\n");
		return -EINVAL;
	}

	ioarea = request_mem_region(mem->start, resource_size(mem),
			pdev->name);
	if (!ioarea) {
		dev_err(&pdev->dev, "pwm region already claimed\n");
		return -EBUSY;
	}

	base = ioremap_nocache(mem->start, resource_size(mem));
	if (!base) {
		dev_err(&pdev->dev, "I/O memory remapping failed\n");
		r = -ENOMEM;
		goto err_release_region;
	}

	r = pwm_byt_init(&pdev->dev, base, pwm_num, PWM_BYT_CLK_KHZ);
	if (r)
		goto err_iounmap;

	pm_runtime_set_autosuspend_delay(&pdev->dev, 5);
	pm_runtime_use_autosuspend(&pdev->dev);
	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);
	pm_runtime_allow(&pdev->dev);

	++pwm_num;
	return 0;

err_iounmap:
	iounmap(base);

err_release_region:
	release_mem_region(mem->start, resource_size(mem));
	dev_info(&pdev->dev, "PWM device probe failed!\n");
	return r;
}

static void pwm_byt_plat_remove(struct platform_device *pdev)
{
	struct resource *mem;
	pm_runtime_forbid(&pdev->dev);
	pwm_byt_remove(&pdev->dev);
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (mem) {
		release_mem_region(mem->start, resource_size(mem));
	}
}

static struct platform_driver pwm_byt_plat_driver = {
	.remove	= pwm_byt_plat_remove,
	.driver	= {
		.name	= "pwm-byt-plat",
		.owner	= THIS_MODULE,
		.pm     = &pwm_byt_pm,
#ifdef CONFIG_ACPI
		.acpi_match_table = ACPI_PTR(pwm_byt_acpi_ids),
#endif
	},
};

static int __init pwm_byt_init_driver(void)
{
	return platform_driver_probe(&pwm_byt_plat_driver, pwm_byt_plat_probe);
}
module_init(pwm_byt_init_driver);

static void __exit pwm_byt_exit_driver(void)
{
	platform_driver_unregister(&pwm_byt_plat_driver);
}
module_exit(pwm_byt_exit_driver);

MODULE_ALIAS("pwm-byt-plat");
MODULE_AUTHOR("Wang, Zhifeng<zhifeng.wang@intel.com>");
MODULE_DESCRIPTION("Intel Baytrail PWM driver");
MODULE_LICENSE("GPL");
