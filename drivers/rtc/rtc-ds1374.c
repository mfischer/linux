/*
 * RTC driver for the Maxim/Dallas DS1374 Real-Time Clock via MFD
 *
 * Based on code by Randy Vinson <rvinson@mvista.com>,
 * which was based on the m41t00.c by Mark Greer <mgreer@mvista.com>.
 *
 * Copyright (C) 2017 National Instruments Corp
 * Copyright (C) 2014 Rose Technology
 * Copyright (C) 2006-2007 Freescale Semiconductor
 *
 * 2005 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/rtc.h>
#include <linux/bcd.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/regmap.h>
#include <linux/mfd/ds1374.h>
#include <linux/platform_device.h>

struct ds1374_rtc {
	struct rtc_device *rtc;
	struct ds1374 *chip;
	struct work_struct work;

	/* The mutex protects alarm operations, and prevents a race
	 * between the enable_irq() in the workqueue and the free_irq()
	 * in the remove function.
	 */
	struct mutex mutex;
	int exiting;
};

static int ds1374_read_rtc(struct ds1374_rtc *ds1374, u32 *time,
			   int reg, int nbytes)
{
	u8 buf[4];
	int ret;
	int i;

	if (WARN_ON(nbytes > 4))
		return -EINVAL;

	ret = regmap_bulk_read(ds1374->chip->regmap, reg, buf, nbytes);
	if (ret) {
		dev_err(&ds1374->chip->client->dev, "Failed to bulkread n = %d at R%d\n",
			nbytes, reg);
		return ret;
	}

	for (i = nbytes - 1, *time = 0; i >= 0; i--)
		*time = (*time << 8) | buf[i];

	return 0;
}

static int ds1374_write_rtc(struct ds1374_rtc *ds1374, u32 time,
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

	return regmap_bulk_write(ds1374->chip->regmap, reg, buf, nbytes);
}

static int ds1374_check_rtc_status(struct ds1374_rtc *ds1374)
{
	int ret = 0;
	unsigned int control, stat;

	ret = regmap_read(ds1374->chip->regmap, DS1374_REG_SR, &stat);
	if (ret)
		return stat;

	if (stat & DS1374_REG_SR_OSF)
		dev_warn(&ds1374->chip->client->dev,
			 "oscillator discontinuity flagged, time unreliable\n");

	stat &= ~(DS1374_REG_SR_OSF | DS1374_REG_SR_AF);

	ret = regmap_write(ds1374->chip->regmap, DS1374_REG_SR, stat);
	if (ret)
		return ret;

	/* If the alarm is pending, clear it before requesting
	 * the interrupt, so an interrupt event isn't reported
	 * before everything is initialized.
	 */

	ret = regmap_read(ds1374->chip->regmap, DS1374_REG_CR, &control);
	if (ret)
		return ret;

	control &= ~(DS1374_REG_CR_WACE | DS1374_REG_CR_AIE);
	return regmap_write(ds1374->chip->regmap, DS1374_REG_CR, control);
}

static int ds1374_read_time(struct device *dev, struct rtc_time *time)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct ds1374_rtc *ds1374_rtc = platform_get_drvdata(pdev);
	u32 itime;
	int ret;

	ret = ds1374_read_rtc(ds1374_rtc, &itime, DS1374_REG_TOD0, 4);
	if (!ret)
		rtc_time_to_tm(itime, time);

	return ret;
}

static int ds1374_set_time(struct device *dev, struct rtc_time *time)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct ds1374_rtc *ds1374_rtc = platform_get_drvdata(pdev);
	unsigned long itime;

	rtc_tm_to_time(time, &itime);
	return ds1374_write_rtc(ds1374_rtc, itime, DS1374_REG_TOD0, 4);
}

#ifndef CONFIG_RTC_DRV_DS1374_WDT
/* The ds1374 has a decrementer for an alarm, rather than a comparator.
 * If the time of day is changed, then the alarm will need to be
 * reset.
 */
static int ds1374_read_alarm(struct device *dev, struct rtc_wkalrm *alarm)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct ds1374_rtc *ds1374_rtc = platform_get_drvdata(pdev);
	struct ds1374 *ds1374 = ds1374_rtc->chip;

	u32 now, cur_alarm;
	unsigned int cr, sr;
	int ret = 0;

	if (ds1374->irq <= 0)
		return -EINVAL;

	mutex_lock(&ds1374_rtc->mutex);

	ret = regmap_read(ds1374->regmap, DS1374_REG_CR, &cr);
	if (ret < 0)
		goto out;

	ret = regmap_read(ds1374->regmap, DS1374_REG_SR, &cr);
	if (ret < 0)
		goto out;

	ret = ds1374_read_rtc(ds1374_rtc, &now, DS1374_REG_TOD0, 4);
	if (ret)
		goto out;

	ret = ds1374_read_rtc(ds1374_rtc, &cur_alarm, DS1374_REG_WDALM0, 3);
	if (ret)
		goto out;

	rtc_time_to_tm(now + cur_alarm, &alarm->time);
	alarm->enabled = !!(cr & DS1374_REG_CR_WACE);
	alarm->pending = !!(sr & DS1374_REG_SR_AF);

out:
	mutex_unlock(&ds1374_rtc->mutex);
	return ret;
}

static int ds1374_set_alarm(struct device *dev, struct rtc_wkalrm *alarm)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct ds1374_rtc *ds1374_rtc = platform_get_drvdata(pdev);
	struct ds1374 *ds1374 = ds1374_rtc->chip;

	struct rtc_time now;
	unsigned long new_alarm, itime;
	int cr;
	int ret = 0;

	if (ds1374->irq <= 0)
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

	mutex_lock(&ds1374_rtc->mutex);

	ret = regmap_read(ds1374->regmap, DS1374_REG_CR, &cr);
	if (ret < 0)
		goto out;

	/* Disable any existing alarm before setting the new one
	 * (or lack thereof).
	 */
	cr &= ~DS1374_REG_CR_WACE;

	ret = regmap_write(ds1374->regmap, DS1374_REG_CR, cr);
	if (ret < 0)
		goto out;

	ret = ds1374_write_rtc(ds1374_rtc, new_alarm, DS1374_REG_WDALM0, 3);
	if (ret)
		goto out;

	if (alarm->enabled) {
		cr |= DS1374_REG_CR_WACE | DS1374_REG_CR_AIE;
		cr &= ~DS1374_REG_CR_WDALM;

		ret = regmap_write(ds1374->regmap, DS1374_REG_CR, cr);
	}

out:
	mutex_unlock(&ds1374_rtc->mutex);
	return ret;
}
#endif

static irqreturn_t ds1374_irq(int irq, void *dev_id)
{
	struct ds1374_rtc *ds1374_rtc = dev_id;

	disable_irq_nosync(irq);
	schedule_work(&ds1374_rtc->work);
	return IRQ_HANDLED;
}

static void ds1374_work(struct work_struct *work)
{
	struct ds1374_rtc *ds1374_rtc = container_of(work, struct ds1374_rtc,
						     work);
	unsigned int stat, control;
	int ret;

	mutex_lock(&ds1374_rtc->mutex);

	ret = regmap_read(ds1374_rtc->chip->regmap, DS1374_REG_SR, &stat);
	if (ret < 0)
		goto unlock;

	if (stat & DS1374_REG_SR_AF) {
		stat &= ~DS1374_REG_SR_AF;
		regmap_write(ds1374_rtc->chip->regmap, DS1374_REG_SR, stat);

		ret = regmap_read(ds1374_rtc->chip->regmap, DS1374_REG_CR,
				  &control);
		if (ret < 0)
			goto out;

		control &= ~(DS1374_REG_CR_WACE | DS1374_REG_CR_AIE);
		regmap_write(ds1374_rtc->chip->regmap, DS1374_REG_CR, control);

		rtc_update_irq(ds1374_rtc->rtc, 1, RTC_AF | RTC_IRQF);
	}

out:
	if (!ds1374_rtc->exiting)
		enable_irq(ds1374_rtc->chip->irq);
unlock:
	mutex_unlock(&ds1374_rtc->mutex);
}

#ifndef CONFIG_RTC_DRV_DS1374_WDT
static int ds1374_alarm_irq_enable(struct device *dev, unsigned int enabled)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct ds1374_rtc *ds1374 = platform_get_drvdata(pdev);
	unsigned int cr;
	int ret;

	mutex_lock(&ds1374->mutex);

	ret = regmap_read(ds1374->chip->regmap, DS1374_REG_CR, &cr);
	if (ret < 0)
		goto out;

	if (enabled) {
		cr |= DS1374_REG_CR_WACE | DS1374_REG_CR_AIE;
		cr &= ~DS1374_REG_CR_WDALM;
	} else {
		cr &= ~DS1374_REG_CR_WACE;
	}
	ret = regmap_write(ds1374->chip->regmap, DS1374_REG_CR, cr);

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

/*
 *****************************************************************************
 *
 *	Driver Interface
 *
 *****************************************************************************
 */
static int ds1374_rtc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ds1374 *ds1374 = dev_get_drvdata(dev->parent);
	struct ds1374_rtc *ds1374_rtc;
	int ret;

	ds1374_rtc = devm_kzalloc(dev, sizeof(*ds1374_rtc), GFP_KERNEL);
	if (!ds1374_rtc)
		return -ENOMEM;
	ds1374_rtc->chip = ds1374;

	platform_set_drvdata(pdev, ds1374_rtc);

	INIT_WORK(&ds1374_rtc->work, ds1374_work);
	mutex_init(&ds1374_rtc->mutex);

	ret = ds1374_check_rtc_status(ds1374_rtc);
	if (ret) {
		dev_err(dev, "Failed to check rtc status\n");
		return ret;
	}

	if (ds1374->irq > 0) {
		ret = devm_request_irq(dev, ds1374->irq,
				       ds1374_irq, 0, "ds1374", ds1374_rtc);
		if (ret) {
			dev_err(dev, "unable to request IRQ\n");
			return ret;
		}

		device_set_wakeup_capable(dev, 1);
	}

	ds1374_rtc->rtc = devm_rtc_device_register(dev, "ds1374-rtc",
						   &ds1374_rtc_ops,
						   THIS_MODULE);
	if (IS_ERR(ds1374_rtc->rtc)) {
		dev_err(dev, "unable to register the class device\n");
		return PTR_ERR(ds1374_rtc->rtc);
	}
	return 0;
}

static int ds1374_rtc_remove(struct platform_device *pdev)
{
	struct ds1374_rtc *ds1374_rtc = platform_get_drvdata(pdev);

	if (ds1374_rtc->chip->irq > 0) {
		mutex_lock(&ds1374_rtc->mutex);
		ds1374_rtc->exiting = 1;
		mutex_unlock(&ds1374_rtc->mutex);

		devm_free_irq(&pdev->dev, ds1374_rtc->chip->irq,
			      ds1374_rtc);
		cancel_work_sync(&ds1374_rtc->work);
	}

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int ds1374_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct ds1374_rtc *ds1374_rtc = platform_get_drvdata(pdev);

	if (ds1374_rtc->chip->irq > 0 && device_may_wakeup(&pdev->dev))
		enable_irq_wake(ds1374_rtc->chip->irq);
	return 0;
}

static int ds1374_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct ds1374_rtc *ds1374_rtc = platform_get_drvdata(pdev);

	if (ds1374_rtc->chip->irq > 0 && device_may_wakeup(&pdev->dev))
		disable_irq_wake(ds1374_rtc->chip->irq);
	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(ds1374_rtc_pm, ds1374_rtc_suspend, ds1374_rtc_resume);

static struct platform_driver ds1374_rtc_driver = {
	.driver = {
		.name = "ds1374-rtc",
		.pm = &ds1374_rtc_pm,
	},
	.probe = ds1374_rtc_probe,
	.remove = ds1374_rtc_remove,
};
module_platform_driver(ds1374_rtc_driver);

MODULE_AUTHOR("Scott Wood <scottwood@freescale.com>");
MODULE_AUTHOR("Moritz Fischer <mdf@kernel.org>");
MODULE_DESCRIPTION("Maxim/Dallas DS1374 RTC Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ds1374-rtc");
