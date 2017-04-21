/*
 * Copyright (c) 2017, National Instruments Corp.
 *
 * Dallas/Maxim DS1374 Watchdog Driver, heavily based on the older
 * drivers/rtc/rtc-ds1374.c implementation
 *
 * SPDX-License-Identifier: GPL-2.0
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/watchdog.h>
#include <linux/slab.h>
#include <linux/regmap.h>
#include <linux/platform_device.h>
#include <linux/mfd/ds1374.h>

#define DS1374_WDT_RATE 4096 /* Hz */
#define DS1374_WDT_MIN_TIMEOUT 1 /* seconds */
#define DS1374_WDT_DEFAULT_TIMEOUT 30 /* seconds */

static bool nowayout = WATCHDOG_NOWAYOUT;
module_param(nowayout, bool, 0444);
MODULE_PARM_DESC(nowayout,
		 "Watchdog cannot be stopped once started (default="
		 __MODULE_STRING(WATCHDOG_NOWAYOUT) ")");

static unsigned int timeout;
module_param(timeout, int, 0444);
MODULE_PARM_DESC(timeout, "Watchdog timeout");

struct ds1374_wdt {
	struct ds1374 *chip;
	struct device *dev;
	struct watchdog_device wdd;
};

static int ds1374_wdt_stop(struct watchdog_device *wdog)
{
	struct ds1374_wdt *ds1374_wdt = watchdog_get_drvdata(wdog);
	int err;

	err = regmap_update_bits(ds1374_wdt->chip->regmap, DS1374_REG_CR,
				 DS1374_REG_CR_WACE, 0);
	if (err)
		return err;

	if (ds1374_wdt->chip->remapped_reset)
		return regmap_update_bits(ds1374_wdt->chip->regmap,
					  DS1374_REG_CR, DS1374_REG_CR_WDSTR,
					  0);

	return 0;
}

static int ds1374_wdt_ping(struct watchdog_device *wdog)
{
	struct ds1374_wdt *ds1374_wdt = watchdog_get_drvdata(wdog);
	u32 val;
	int err;

	err = ds1374_read_bulk(ds1374_wdt->chip, &val, DS1374_REG_WDALM0, 3);
	if (err < 0)
		return err;

	return 0;
}

static int ds1374_wdt_set_timeout(struct watchdog_device *wdog,
				  unsigned int t)
{
	struct ds1374_wdt *ds1374_wdt = watchdog_get_drvdata(wdog);
	struct regmap *regmap = ds1374_wdt->chip->regmap;
	unsigned int timeout = DS1374_WDT_RATE * t;
	u8 remapped = ds1374_wdt->chip->remapped_reset
		? DS1374_REG_CR_WDSTR : 0;
	int err;

	err = regmap_update_bits(regmap, DS1374_REG_CR,
				 DS1374_REG_CR_WACE | DS1374_REG_CR_AIE, 0);

	err = ds1374_write_bulk(ds1374_wdt->chip, timeout,
				DS1374_REG_WDALM0, 3);
	if (err) {
		dev_err(ds1374_wdt->dev, "couldn't set new watchdog time\n");
		return err;
	}

	ds1374_wdt->wdd.timeout = t;

	return regmap_update_bits(regmap, DS1374_REG_CR,
				  (DS1374_REG_CR_WACE | DS1374_REG_CR_WDALM |
				   DS1374_REG_CR_AIE | DS1374_REG_CR_WDSTR),
				  (DS1374_REG_CR_WACE | DS1374_REG_CR_WDALM |
				   DS1374_REG_CR_AIE | remapped));
}

static int ds1374_wdt_start(struct watchdog_device *wdog)
{
	int err;
	struct ds1374_wdt *ds1374_wdt = watchdog_get_drvdata(wdog);

	err = ds1374_wdt_set_timeout(wdog, wdog->timeout);
	if (err) {
		dev_err(ds1374_wdt->dev, "%s: failed to set timeout (%d) %u\n",
			__func__, err, wdog->timeout);
		return err;
	}

	err = ds1374_wdt_ping(wdog);
	if (err) {
		dev_err(ds1374_wdt->dev, "%s: failed to ping (%d)\n", __func__,
			err);
		return err;
	}

	return 0;
}

static const struct watchdog_info ds1374_wdt_info = {
	.identity       = "DS1374 WTD",
	.options        = WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING
			| WDIOF_MAGICCLOSE,
};

static const struct watchdog_ops ds1374_wdt_ops = {
	.owner		= THIS_MODULE,
	.start		= ds1374_wdt_start,
	.stop		= ds1374_wdt_stop,
	.set_timeout	= ds1374_wdt_set_timeout,
	.ping		= ds1374_wdt_ping,
};

static int ds1374_wdt_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ds1374 *ds1374 = dev_get_drvdata(dev->parent);
	struct ds1374_wdt *priv;
	int err;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	priv->chip = ds1374;
	platform_set_drvdata(pdev, priv);

	priv->wdd.info		= &ds1374_wdt_info;
	priv->wdd.ops		= &ds1374_wdt_ops;
	priv->wdd.min_timeout	= DS1374_WDT_MIN_TIMEOUT;
	priv->wdd.timeout	= DS1374_WDT_DEFAULT_TIMEOUT;
	priv->wdd.max_timeout	= 0x1ffffff / DS1374_WDT_RATE;
	priv->wdd.parent	= dev->parent;

	watchdog_init_timeout(&priv->wdd, timeout, dev);
	watchdog_set_nowayout(&priv->wdd, nowayout);
	watchdog_stop_on_reboot(&priv->wdd);
	watchdog_set_drvdata(&priv->wdd, priv);

	err = devm_watchdog_register_device(dev, &priv->wdd);
	if (err) {
		dev_err(dev, "Failed to register watchdog device\n");
		return err;
	}

	dev_info(dev, "Registered DS1374 Watchdog\n");

	return 0;
}

static int ds1374_wdt_remove(struct platform_device *pdev)
{
	struct ds1374_wdt *priv = platform_get_drvdata(pdev);

	if (!nowayout)
		ds1374_wdt_stop(&priv->wdd);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int ds1374_wdt_suspend(struct device *dev)
{
	return 0;
}

static int ds1374_wdt_resume(struct device *dev)
{
	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(ds1374_wdt_pm, ds1374_wdt_suspend, ds1374_wdt_resume);

static struct platform_driver ds1374_wdt_driver = {
	.probe = ds1374_wdt_probe,
	.remove = ds1374_wdt_remove,
	.driver = {
		.name = "ds1374-wdt",
		.pm = &ds1374_wdt_pm,
	},
};
module_platform_driver(ds1374_wdt_driver);

MODULE_AUTHOR("Moritz Fischer <mdf@kernel.org>");
MODULE_DESCRIPTION("Maxim/Dallas DS1374 WDT Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ds1374-wdt");
