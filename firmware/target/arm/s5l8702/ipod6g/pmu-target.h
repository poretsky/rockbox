/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 * $Id: pmu-target.h 24721 2010-02-17 15:54:48Z theseven $
 *
 * Copyright © 2009 Michael Sparmann
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This software is distributed on an "AS IS" basis, WITHOUT WARRANTY OF ANY
 * KIND, either express or implied.
 *
 ****************************************************************************/

#ifndef __PMU_TARGET_H__
#define __PMU_TARGET_H__

#include <stdint.h>
#include <stdbool.h>
#include "config.h"

#include "pcf5063x.h"

/* undocummented PMU registers */
#define PCF50635_REG_INT6       0x85
#define PCF50635_REG_INT6M      0x86
#define PCF50635_REG_GPIOSTAT   0x87

enum pcf50635_reg_int6 {
        PCF50635_INT6_GPIO1     = 0x01, /* TBC */
        PCF50635_INT6_GPIO2     = 0x02,
        PCF50635_INT6_GPIO3     = 0x04, /* TBC */
};

enum pcf50635_reg_gpiostat {
        PCF50635_GPIOSTAT_GPIO1 = 0x01, /* TBC */
        PCF50635_GPIOSTAT_GPIO2 = 0x02,
        PCF50635_GPIOSTAT_GPIO3 = 0x04, /* TBC */
};


/* GPIO for external PMU interrupt */
#define GPIO_EINT_PMU   0x7b

/* LDOs - enumeration starts at 1 as pcf50635 DS does */
#define LDO_UNK1        1   /* SoC voltage (USB+TBC section) */
#define LDO_UNK2        2   /* SoC voltage (I/O+TBC section) */
#define LDO_LCD         3
#define LDO_CODEC       4
#define LDO_UNK5        5   /* not used */
#define LDO_CWHEEL      6
#define LDO_ACCY        7   /* HCLDO */
/*
 * Other LDOs:
 *  AUTOLDO: Hard Disk
 *  DOWN1:   Vcore
 *  DOWN2:   SDRAM
 *  MEMLDO:  SDRAM self-refresh (TBC)
 *
 * EXTON inputs:
 *  EXTON1: button/holdswitch related (TBC)
 *  EXTON2: USB Vbus (High when present)
 *  EXTON3: ACCESSORY (Low when present)
 *
 * PMU GPIO:
 *  GPIO1: input, Mikey (jack remote ctrl) interrupt
 *  GPIO2: input, hold switch (TBC)
 *  GPIO3: output, OF uses it as a flag to detect hibernation state,
 *         it is unknown where/if this output is connected.
 */


struct pmu_adc_channel
{
    const char *name;
    uint8_t adcc1;
    uint8_t adcc2;
    uint8_t adcc3;
    uint8_t bias_dly; /* RB ticks */
};

void pmu_preinit(void);
void pmu_init(void);
unsigned char pmu_read(int address);
int pmu_write(int address, unsigned char val);
int pmu_read_multiple(int address, int count, unsigned char* buffer);
int pmu_write_multiple(int address, int count, unsigned char* buffer);
#ifdef BOOTLOADER
unsigned char pmu_rd(int address);
int pmu_wr(int address, unsigned char val);
int pmu_rd_multiple(int address, int count, unsigned char* buffer);
int pmu_wr_multiple(int address, int count, unsigned char* buffer);
bool pmu_is_hibernated(void);
#endif

void pmu_ldo_on_in_standby(unsigned int ldo, int onoff);
void pmu_ldo_set_voltage(unsigned int ldo, unsigned char voltage);
void pmu_ldo_power_on(unsigned int ldo);
void pmu_ldo_power_off(unsigned int ldo);
void pmu_set_wake_condition(unsigned char condition);
void pmu_enter_standby(void);
void pmu_hdd_power(bool on);
#ifdef HAVE_ADJUSTABLE_CPU_FREQ
void pmu_set_cpu_voltage(bool high);
#endif 
#if (CONFIG_RTC == RTC_NANO2G)
void pmu_read_rtc(unsigned char* buffer);
void pmu_write_rtc(unsigned char* buffer);
#endif

unsigned short pmu_read_adc(const struct pmu_adc_channel *ch);
unsigned short pmu_adc_raw2mv(
        const struct pmu_adc_channel *ch, unsigned short raw);

int pmu_holdswitch_locked(void);
#if CONFIG_CHARGING
int pmu_firewire_present(void);
#endif
#ifdef IPOD_ACCESSORY_PROTOCOL
int pmu_accessory_present(void);
#endif

#endif /* __PMU_TARGET_H__ */
