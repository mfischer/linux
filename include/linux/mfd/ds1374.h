/*
 * Copyright (c) 2017, National Instruments Corp.
 *
 * Multi Function Device for Dallas/Maxim DS1374 RTC/WDT
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef MFD_DS1374_H
#define MFD_DS1374_H

#include <linux/i2c.h>
#include <linux/regmap.h>

enum ds1374_mode {
	DS1374_MODE_RTC_ONLY,
	DS1374_MODE_RTC_ALM,
	DS1374_MODE_RTC_WDT,
};

/* Register definitions to for all subdrivers
 */
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

struct ds1374 {
	struct i2c_client *client;
	struct regmap *regmap;
	int irq;
	enum ds1374_mode mode;
	bool remapped_reset;
};

int ds1374_read_bulk(struct ds1374 *ds1374, u32 *time, int reg, int nbytes);

int ds1374_write_bulk(struct ds1374 *ds1374, u32 time, int reg, int nbytes);

#endif /* MFD_DS1374_H */
