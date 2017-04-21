/*
 * RTC client/driver for the Maxim/Dallas DS1374 Real-Time Clock over I2C
 *
 * Based on code by Randy Vinson <rvinson@mvista.com>,
 * which was based on the m41t00.c by Mark Greer <mgreer@mvista.com>.
 *
 * Copyright (C) 2014 Rose Technology
 * Copyright (C) 2006-2007 Freescale Semiconductor
 *
 * 2005 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */
/*
 * It would be more efficient to use i2c msgs/i2c_transfer directly but, as
 * recommened in .../Documentation/i2c/writing-clients section
 * "Sending and receiving", using SMBus level communication is preferred.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/rtc.h>
#include <linux/bcd.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/pm.h>
#ifdef CONFIG_RTC_DRV_DS1374_WDT
#include <linux/fs.h>
#include <linux/ioctl.h>
#include <linux/miscdevice.h>
#include <linux/reboot.h>
#include <linux/watchdog.h>
#endif

#define DS1374_REG_TOD0		0x00 /* Time of Day */
#define DS1374_REG_TOD1		0x01
#define DS1374_REG_TOD2		0x02
#define DS1374_REG_TOD3		0x03
#define DS1374_REG_WDALM0	0x04 /* Watchdog/Alarm */
#define DS1374_REG_WDALM1	0x05
#define DS1374_REG_WDALM2	0x06
#define DS1374_REG_CR		0x07 /* Control */
#define DS1374_REG_CR_AIE	0x01 /* Alarm Int. Enable */
#define DS1374_REG_CR_WDSTR	0x08 /* 1=Reset on INT, 0=Rreset on RST */
#define DS1374_REG_CR_WDALM	0x20 /* 1=Watchdog, 0=Alarm */
#define DS1374_REG_CR_WACE	0x40 /* WD/Alarm counter enable */
#define DS1374_REG_SR		0x08 /* Status */
#define DS1374_REG_SR_OSF	0x80 /* Oscillator Stop Flag */
#define DS1374_REG_SR_AF	0x01 /* Alarm Flag */
#define DS1374_REG_TCR		0x09 /* Trickle Charge */

#define DS1374_TRICKLE_CHARGER_ENABLE	0xA0
#define DS1374_TRICKLE_CHARGER_250_OHM	0x01
#define DS1374_TRICKLE_CHARGER_2K_OHM	0x02
#define DS1374_TRICKLE_CHARGER_4K_OHM	0x03
#define DS1374_TRICKLE_CHARGER_NO_DIODE	0x04
#define DS1374_TRICKLE_CHARGER_DIODE	0x08

#define WDT_MIN_TIMEOUT		1 /* seconds */
#define WDT_DEFAULT_TIMEOUT	30 /* seconds */


#ifdef CONFIG_RTC_DRV_DS1374_WDT
static bool nowayout = WATCHDOG_NOWAYOUT;
module_param(nowayout, bool, 0);
MODULE_PARM_DESC(nowayout,
		 "Watchdog cannot be stopped once started (default="
		 __MODULE_STRING(WATCHDOG_NOWAYOUT) ")");

static unsigned int timeout;
module_param(timeout, int, 0);
MODULE_PARM_DESC(timeout, "Watchdog timeout");
#endif

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

struct ds1374 {
	struct i2c_client *client;
	struct rtc_device *rtc;
	struct work_struct work;

	/* The mutex protects alarm operations, and prevents a race
	 * between the enable_irq() in the workqueue and the free_irq()
	 * in the remove function.
	 */
	struct mutex mutex;
	int exiting;

#ifdef CONFIG_RTC_DRV_DS1374_WDT
	struct watchdog_device wdd;
	u32 rate;
	bool remapped_reset;
#endif
};

static struct i2c_driver ds1374_driver;

static int ds1374_read_rtc(struct i2c_client *client, u32 *time,
			   int reg, int nbytes)
{
	u8 buf[4];
	int ret;
	int i;

	if (WARN_ON(nbytes > 4))
		return -EINVAL;

	ret = i2c_smbus_read_i2c_block_data(client, reg, nbytes, buf);

	if (ret < 0)
		return ret;
	if (ret < nbytes)
		return -EIO;

	for (i = nbytes - 1, *time = 0; i >= 0; i--)
		*time = (*time << 8) | buf[i];

	return 0;
}

static int ds1374_write_rtc(struct i2c_client *client, u32 time,
			    int reg, int nbytes)
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

	return i2c_smbus_write_i2c_block_data(client, reg, nbytes, buf);
}

static int ds1374_check_rtc_status(struct i2c_client *client)
{
	int ret = 0;
	int control, stat;

	stat = i2c_smbus_read_byte_data(client, DS1374_REG_SR);
	if (stat < 0)
		return stat;

	if (stat & DS1374_REG_SR_OSF)
		dev_warn(&client->dev,
			 "oscillator discontinuity flagged, time unreliable\n");

	stat &= ~(DS1374_REG_SR_OSF | DS1374_REG_SR_AF);

	ret = i2c_smbus_write_byte_data(client, DS1374_REG_SR, stat);
	if (ret < 0)
		return ret;

	/* If the alarm is pending, clear it before requesting
	 * the interrupt, so an interrupt event isn't reported
	 * before everything is initialized.
	 */

	control = i2c_smbus_read_byte_data(client, DS1374_REG_CR);
	if (control < 0)
		return control;

	control &= ~(DS1374_REG_CR_WACE | DS1374_REG_CR_AIE);
	return i2c_smbus_write_byte_data(client, DS1374_REG_CR, control);
}

static int ds1374_read_time(struct device *dev, struct rtc_time *time)
{
	struct i2c_client *client = to_i2c_client(dev);
	u32 itime;
	int ret;

	ret = ds1374_read_rtc(client, &itime, DS1374_REG_TOD0, 4);
	if (!ret)
		rtc_time_to_tm(itime, time);

	return ret;
}

static int ds1374_set_time(struct device *dev, struct rtc_time *time)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned long itime;

	rtc_tm_to_time(time, &itime);
	return ds1374_write_rtc(client, itime, DS1374_REG_TOD0, 4);
}

#ifndef CONFIG_RTC_DRV_DS1374_WDT
/* The ds1374 has a decrementer for an alarm, rather than a comparator.
 * If the time of day is changed, then the alarm will need to be
 * reset.
 */
static int ds1374_read_alarm(struct device *dev, struct rtc_wkalrm *alarm)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ds1374 *ds1374 = i2c_get_clientdata(client);
	u32 now, cur_alarm;
	int cr, sr;
	int ret = 0;

	if (client->irq <= 0)
		return -EINVAL;

	mutex_lock(&ds1374->mutex);

	cr = ret = i2c_smbus_read_byte_data(client, DS1374_REG_CR);
	if (ret < 0)
		goto out;

	sr = ret = i2c_smbus_read_byte_data(client, DS1374_REG_SR);
	if (ret < 0)
		goto out;

	ret = ds1374_read_rtc(client, &now, DS1374_REG_TOD0, 4);
	if (ret)
		goto out;

	ret = ds1374_read_rtc(client, &cur_alarm, DS1374_REG_WDALM0, 3);
	if (ret)
		goto out;

	rtc_time_to_tm(now + cur_alarm, &alarm->time);
	alarm->enabled = !!(cr & DS1374_REG_CR_WACE);
	alarm->pending = !!(sr & DS1374_REG_SR_AF);

out:
	mutex_unlock(&ds1374->mutex);
	return ret;
}

static int ds1374_set_alarm(struct device *dev, struct rtc_wkalrm *alarm)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ds1374 *ds1374 = i2c_get_clientdata(client);
	struct rtc_time now;
	unsigned long new_alarm, itime;
	int cr;
	int ret = 0;

	if (client->irq <= 0)
		return -EINVAL;

	ret = ds1374_read_time(dev, &now);
	if (ret < 0)
		return ret;

	rtc_tm_to_time(&alarm->time, &new_alarm);
	rtc_tm_to_time(&now, &itime);

	/* This can happen due to races, in addition to dates that are
	 * truly in the past.  To avoid requiring the caller to check for
	 * races, dates in the past are assumed to be in the recent past
	 * (i.e. not something that we'd rather the caller know about via
	 * an error), and the alarm is set to go off as soon as possible.
	 */
	if (time_before_eq(new_alarm, itime))
		new_alarm = 1;
	else
		new_alarm -= itime;

	mutex_lock(&ds1374->mutex);

	ret = cr = i2c_smbus_read_byte_data(client, DS1374_REG_CR);
	if (ret < 0)
		goto out;

	/* Disable any existing alarm before setting the new one
	 * (or lack thereof). */
	cr &= ~DS1374_REG_CR_WACE;

	ret = i2c_smbus_write_byte_data(client, DS1374_REG_CR, cr);
	if (ret < 0)
		goto out;

	ret = ds1374_write_rtc(client, new_alarm, DS1374_REG_WDALM0, 3);
	if (ret)
		goto out;

	if (alarm->enabled) {
		cr |= DS1374_REG_CR_WACE | DS1374_REG_CR_AIE;
		cr &= ~DS1374_REG_CR_WDALM;

		ret = i2c_smbus_write_byte_data(client, DS1374_REG_CR, cr);
	}

out:
	mutex_unlock(&ds1374->mutex);
	return ret;
}
#endif

static irqreturn_t ds1374_irq(int irq, void *dev_id)
{
	struct i2c_client *client = dev_id;
	struct ds1374 *ds1374 = i2c_get_clientdata(client);

	disable_irq_nosync(irq);
	schedule_work(&ds1374->work);
	return IRQ_HANDLED;
}

static void ds1374_work(struct work_struct *work)
{
	struct ds1374 *ds1374 = container_of(work, struct ds1374, work);
	struct i2c_client *client = ds1374->client;
	int stat, control;

	mutex_lock(&ds1374->mutex);

	stat = i2c_smbus_read_byte_data(client, DS1374_REG_SR);
	if (stat < 0)
		goto unlock;

	if (stat & DS1374_REG_SR_AF) {
		stat &= ~DS1374_REG_SR_AF;
		i2c_smbus_write_byte_data(client, DS1374_REG_SR, stat);

		control = i2c_smbus_read_byte_data(client, DS1374_REG_CR);
		if (control < 0)
			goto out;

		control &= ~(DS1374_REG_CR_WACE | DS1374_REG_CR_AIE);
		i2c_smbus_write_byte_data(client, DS1374_REG_CR, control);

		rtc_update_irq(ds1374->rtc, 1, RTC_AF | RTC_IRQF);
	}

out:
	if (!ds1374->exiting)
		enable_irq(client->irq);
unlock:
	mutex_unlock(&ds1374->mutex);
}

#ifndef CONFIG_RTC_DRV_DS1374_WDT
static int ds1374_alarm_irq_enable(struct device *dev, unsigned int enabled)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ds1374 *ds1374 = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&ds1374->mutex);

	ret = i2c_smbus_read_byte_data(client, DS1374_REG_CR);
	if (ret < 0)
		goto out;

	if (enabled) {
		ret |= DS1374_REG_CR_WACE | DS1374_REG_CR_AIE;
		ret &= ~DS1374_REG_CR_WDALM;
	} else {
		ret &= ~DS1374_REG_CR_WACE;
	}
	ret = i2c_smbus_write_byte_data(client, DS1374_REG_CR, ret);

out:
	mutex_unlock(&ds1374->mutex);
	return ret;
}
#endif

static const struct rtc_class_ops ds1374_rtc_ops = {
	.read_time = ds1374_read_time,
	.set_time = ds1374_set_time,
#ifndef CONFIG_RTC_DRV_DS1374_WDT
	.read_alarm = ds1374_read_alarm,
	.set_alarm = ds1374_set_alarm,
	.alarm_irq_enable = ds1374_alarm_irq_enable,
#endif
};

#ifdef CONFIG_RTC_DRV_DS1374_WDT
static const struct watchdog_info ds1374_wdt_info = {
	.identity       = "DS1374 WTD",
	.options        = WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING |
						WDIOF_MAGICCLOSE,
};

static int ds1374_wdt_stop(struct watchdog_device *wdog)
{
	struct ds1374 *ds1374 = watchdog_get_drvdata(wdog);
	int cr;

	cr = i2c_smbus_read_byte_data(ds1374->client, DS1374_REG_CR);
	/* Disable watchdog timer */
	cr &= ~(DS1374_REG_CR_WACE | DS1374_REG_CR_WDSTR);

	return i2c_smbus_write_byte_data(ds1374->client, DS1374_REG_CR, cr);
}

static int ds1374_wdt_set_timeout(struct watchdog_device *wdog,
				  unsigned int t)
{
	struct ds1374 *ds1374 = watchdog_get_drvdata(wdog);
	unsigned int timeout = ds1374->rate * t;
	int cr, err;

	cr  = i2c_smbus_read_byte_data(ds1374->client, DS1374_REG_CR);
	if (cr < 0)
		return 0;

	/* Disable any existing watchdog/alarm before setting the new one */
	cr &= ~(DS1374_REG_CR_WACE | DS1374_REG_CR_AIE);

	err = i2c_smbus_write_byte_data(ds1374->client, DS1374_REG_CR, cr);
	if (err < 0)
		return err;

	err = ds1374_write_rtc(ds1374->client, timeout, DS1374_REG_WDALM0, 3);
	if (err) {
		pr_err("couldn't set new watchdog time\n");
		return err;
	}

	/* Enable watchdog timer */
	cr |= DS1374_REG_CR_WACE | DS1374_REG_CR_WDALM | DS1374_REG_CR_AIE;

	if (ds1374->remapped_reset)
		cr |= DS1374_REG_CR_WDSTR;

	err = i2c_smbus_write_byte_data(ds1374->client, DS1374_REG_CR, cr);
	if (err < 0)
		return err;

	/* WHY? */
	ds1374->wdd.timeout = t;

	return err;
}

static int ds1374_wdt_ping(struct watchdog_device *wdog)
{
	struct ds1374 *ds1374 = watchdog_get_drvdata(wdog);
	u32 val;
	int err;

	err = ds1374_read_rtc(ds1374->client, &val, DS1374_REG_WDALM0, 3);
	if (err < 0)
		return err;

	return 0;
}

static int ds1374_wdt_start(struct watchdog_device *wdog)
{
	int err;

	err = ds1374_wdt_set_timeout(wdog, wdog->timeout);
	if (err) {
		pr_err("%s: failed to set timeout (%d) %u\n", __func__, err,
		       wdog->timeout);
		return err;
	}

	err = ds1374_wdt_ping(wdog);
	if (err) {
		pr_err("%s: failed to ping (%d)\n", __func__, err);
		return err;
	}

	return 0;
}


static const struct watchdog_ops ds1374_wdt_ops = {
	.owner		= THIS_MODULE,
	.start		= ds1374_wdt_start,
	.stop		= ds1374_wdt_stop,
	.set_timeout	= ds1374_wdt_set_timeout,
	.ping		= ds1374_wdt_ping,
};

#endif /*CONFIG_RTC_DRV_DS1374_WDT*/

static int ds1374_trickle_of_init(struct i2c_client *client)
{
	u32 ohms = 0;
	u8 value;
	int ret;

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

	ret = i2c_smbus_write_byte_data(client, DS1374_REG_TCR, value);
	if (ret < 0)
		return ret;

	return 0;
}

/*
 *****************************************************************************
 *
 *	Driver Interface
 *
 *****************************************************************************
 */
static int ds1374_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct ds1374 *ds1374;
	int ret;

	ds1374 = devm_kzalloc(&client->dev, sizeof(struct ds1374), GFP_KERNEL);
	if (!ds1374)
		return -ENOMEM;

	ds1374->client = client;
	i2c_set_clientdata(client, ds1374);

	INIT_WORK(&ds1374->work, ds1374_work);
	mutex_init(&ds1374->mutex);

	ret = ds1374_trickle_of_init(client);
	if (ret)
		return ret;

	ret = ds1374_check_rtc_status(client);
	if (ret)
		return ret;

	if (client->irq > 0) {
		ret = devm_request_irq(&client->dev, client->irq, ds1374_irq, 0,
					"ds1374", client);
		if (ret) {
			dev_err(&client->dev, "unable to request IRQ\n");
			return ret;
		}

		device_set_wakeup_capable(&client->dev, 1);
	}

	ds1374->rtc = devm_rtc_device_register(&client->dev, client->name,
						&ds1374_rtc_ops, THIS_MODULE);
	if (IS_ERR(ds1374->rtc)) {
		dev_err(&client->dev, "unable to register the class device\n");
		return PTR_ERR(ds1374->rtc);
	}

#ifdef CONFIG_RTC_DRV_DS1374_WDT
	ds1374->remapped_reset	= true;

	ds1374->rate		= 4096;
	ds1374->wdd.info	= &ds1374_wdt_info;
	ds1374->wdd.ops		= &ds1374_wdt_ops;
	ds1374->wdd.min_timeout	= WDT_MIN_TIMEOUT;
	ds1374->wdd.timeout	= WDT_DEFAULT_TIMEOUT;
	ds1374->wdd.max_timeout	= 0x1ffffff / ds1374->rate;
	ds1374->wdd.parent	= &client->dev;

	watchdog_init_timeout(&ds1374->wdd, timeout, &client->dev);
	watchdog_set_nowayout(&ds1374->wdd, nowayout);
	watchdog_stop_on_reboot(&ds1374->wdd);
	watchdog_set_drvdata(&ds1374->wdd, ds1374);

	ret = watchdog_register_device(&ds1374->wdd);
	if (ret) {
		dev_err(&client->dev, "Failed to register watchdog device\n");
		return ret;
	}

	dev_info(&client->dev, "Registered DS1374 Watchdog\n");
#endif

	return 0;
}

static int ds1374_remove(struct i2c_client *client)
{
	struct ds1374 *ds1374 = i2c_get_clientdata(client);
#ifdef CONFIG_RTC_DRV_DS1374_WDT
	if (!nowayout)
		ds1374_wdt_stop(&ds1374->wdd);
	watchdog_unregister_device(&ds1374->wdd);
#endif

	if (client->irq > 0) {
		mutex_lock(&ds1374->mutex);
		ds1374->exiting = 1;
		mutex_unlock(&ds1374->mutex);

		devm_free_irq(&client->dev, client->irq, client);
		cancel_work_sync(&ds1374->work);
	}

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int ds1374_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	if (client->irq > 0 && device_may_wakeup(&client->dev))
		enable_irq_wake(client->irq);
	return 0;
}

static int ds1374_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	if (client->irq > 0 && device_may_wakeup(&client->dev))
		disable_irq_wake(client->irq);
	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(ds1374_pm, ds1374_suspend, ds1374_resume);

static struct i2c_driver ds1374_driver = {
	.driver = {
		.name = "rtc-ds1374",
		.of_match_table = of_match_ptr(ds1374_of_match),
		.pm = &ds1374_pm,
	},
	.probe = ds1374_probe,
	.remove = ds1374_remove,
	.id_table = ds1374_id,
};

module_i2c_driver(ds1374_driver);

MODULE_AUTHOR("Scott Wood <scottwood@freescale.com>");
MODULE_DESCRIPTION("Maxim/Dallas DS1374 RTC Driver");
MODULE_LICENSE("GPL");
