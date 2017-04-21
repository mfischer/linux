/*
 * Copyright (c) 2017, National Instruments Corp.
 *
 * Dallas/Maxim DS1374 Multi Function Device Driver
 *
 * The trickle charger code was taken more ore less 1:1 from
 * drivers/rtc/rtc-1390.c
 *
 * SPDX-License-Identifier: GPL-2.0
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/regmap.h>
#include <linux/mfd/core.h>
#include <linux/mfd/ds1374.h>

#define DS1374_TRICKLE_CHARGER_ENABLE	0xa0
#define DS1374_TRICKLE_CHARGER_ENABLE_MASK 0xe0

#define DS1374_TRICKLE_CHARGER_250_OHM	0x01
#define DS1374_TRICKLE_CHARGER_2K_OHM	0x02
#define DS1374_TRICKLE_CHARGER_4K_OHM	0x03
#define DS1374_TRICKLE_CHARGER_ROUT_MASK 0x03

#define DS1374_TRICKLE_CHARGER_NO_DIODE	0x04
#define DS1374_TRICKLE_CHARGER_DIODE	0x08
#define DS1374_TRICKLE_CHARGER_DIODE_MASK 0xc

static const struct regmap_range volatile_ranges[] = {
	regmap_reg_range(DS1374_REG_TOD0, DS1374_REG_WDALM2),
	regmap_reg_range(DS1374_REG_SR, DS1374_REG_SR),
};

static const struct regmap_access_table ds1374_volatile_table = {
	.yes_ranges = volatile_ranges,
	.n_yes_ranges = ARRAY_SIZE(volatile_ranges),
};

static struct regmap_config ds1374_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = DS1374_REG_TCR,
	.volatile_table	= &ds1374_volatile_table,
	.cache_type	= REGCACHE_RBTREE,
};

static struct mfd_cell ds1374_wdt_cell = {
	.name = "ds1374-wdt",
};

static struct mfd_cell ds1374_rtc_cell = {
	.name = "ds1374-rtc",
};

static int ds1374_add_device(struct ds1374 *chip,
			     struct mfd_cell *cell)
{
	cell->platform_data = chip;
	cell->pdata_size = sizeof(*chip);

	return mfd_add_devices(&chip->client->dev, PLATFORM_DEVID_AUTO,
			       cell, 1, NULL, 0, NULL);
}

static int ds1374_trickle_of_init(struct ds1374 *ds1374)
{
	u32 ohms = 0;
	u8 value;
	struct i2c_client *client = ds1374->client;

	if (of_property_read_u32(client->dev.of_node, "trickle-resistor-ohms",
				 &ohms))
		return 0;

	/* Enable charger */
	value = DS1374_TRICKLE_CHARGER_ENABLE;
	if (of_property_read_bool(client->dev.of_node, "trickle-diode-disable"))
		value |= DS1374_TRICKLE_CHARGER_NO_DIODE;
	else
		value |= DS1374_TRICKLE_CHARGER_DIODE;

	/* Resistor select */
	switch (ohms) {
	case 250:
		value |= DS1374_TRICKLE_CHARGER_250_OHM;
		break;
	case 2000:
		value |= DS1374_TRICKLE_CHARGER_2K_OHM;
		break;
	case 4000:
		value |= DS1374_TRICKLE_CHARGER_4K_OHM;
		break;
	default:
		dev_warn(&client->dev,
			 "Unsupported ohm value %02ux in dt\n", ohms);
		return -EINVAL;
	}
	dev_dbg(&client->dev, "Trickle charge value is 0x%02x\n", value);

	return regmap_write(ds1374->regmap, DS1374_REG_TCR, value);
}

int ds1374_read_bulk(struct ds1374 *ds1374, u32 *time, int reg, int nbytes)
{
	u8 buf[4];
	int ret;
	int i;

	if (WARN_ON(nbytes > 4))
		return -EINVAL;

	ret = regmap_bulk_read(ds1374->regmap, reg, buf, nbytes);
	if (ret) {
		dev_err(&ds1374->client->dev,
			"Failed to bulkread n = %d at R%d\n",
			nbytes, reg);
		return ret;
	}

	for (i = nbytes - 1, *time = 0; i >= 0; i--)
		*time = (*time << 8) | buf[i];

	return 0;
}
EXPORT_SYMBOL_GPL(ds1374_read_bulk);

int ds1374_write_bulk(struct ds1374 *ds1374, u32 time, int reg, int nbytes)
{
	u8 buf[4];
	int i;

	if (nbytes > 4) {
		WARN_ON(1);
		return -EINVAL;
	}

	for (i = 0; i < nbytes; i++) {
		buf[i] = time & 0xff;
		time >>= 8;
	}

	return regmap_bulk_write(ds1374->regmap, reg, buf, nbytes);
}
EXPORT_SYMBOL_GPL(ds1374_write_bulk);

static int ds1374_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct ds1374 *ds1374;
	u32 mode;
	int err;

	ds1374 = devm_kzalloc(&client->dev, sizeof(struct ds1374), GFP_KERNEL);
	if (!ds1374)
		return -ENOMEM;

	ds1374->regmap = devm_regmap_init_i2c(client, &ds1374_regmap_config);
	if (IS_ERR(ds1374->regmap))
		return PTR_ERR(ds1374->regmap);

	if (IS_ENABLED(CONFIG_OF) && client->dev.of_node) {
		err = of_property_read_u32(client->dev.of_node,
					   "dallas,mode", &mode);
		if (err < 0) {
			dev_err(&client->dev, "missing dallas,mode property\n");
			return -EINVAL;
		}

		ds1374->remapped_reset
			= of_property_read_bool(client->dev.of_node,
						"dallas,remap-reset");

		ds1374->mode = (enum ds1374_mode)mode;
	} else if (IS_ENABLED(CONFIG_RTC_DRV_DS1374_WDT)) {
		ds1374->mode = DS1374_MODE_RTC_WDT;
	} else {
		ds1374->mode = DS1374_MODE_RTC_ALM;
	}

	ds1374->client = client;
	ds1374->irq = client->irq;
	i2c_set_clientdata(client, ds1374);

	/* check if we're supposed to trickle charge */
	err = ds1374_trickle_of_init(ds1374);
	if (err) {
		dev_err(&client->dev, "Failed to init trickle charger!\n");
		return err;
	}

	/* we always have a rtc */
	err = ds1374_add_device(ds1374, &ds1374_rtc_cell);
	if (err)
		return err;

	/* we might have a watchdog if configured that way */
	if (ds1374->mode == DS1374_MODE_RTC_WDT)
		return ds1374_add_device(ds1374, &ds1374_wdt_cell);

	return err;
}

static const struct i2c_device_id ds1374_id[] = {
	{ "ds1374", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ds1374_id);

#ifdef CONFIG_OF
static const struct of_device_id ds1374_of_match[] = {
	{ .compatible = "dallas,ds1374" },
	{ }
};
MODULE_DEVICE_TABLE(of, ds1374_of_match);
#endif

#ifdef CONFIG_PM_SLEEP
static int ds1374_suspend(struct device *dev)
{
	return 0;
}

static int ds1374_resume(struct device *dev)
{
	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(ds1374_pm, ds1374_suspend, ds1374_resume);

static struct i2c_driver ds1374_driver = {
	.driver = {
		.name = "ds1374",
		.of_match_table = of_match_ptr(ds1374_of_match),
		.pm = &ds1374_pm,
	},
	.probe = ds1374_probe,
	.id_table = ds1374_id,
};

static int __init ds1374_init(void)
{
	return i2c_add_driver(&ds1374_driver);
}
subsys_initcall(ds1374_init);

static void __exit ds1374_exit(void)
{
	i2c_del_driver(&ds1374_driver);
}
module_exit(ds1374_exit);

MODULE_AUTHOR("Moritz Fischer <mdf@kernel.org>");
MODULE_DESCRIPTION("Maxim/Dallas DS1374 MFD Driver");
MODULE_LICENSE("GPL");
