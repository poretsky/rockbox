/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 * $Id$
 *
 * Copyright (C) 2016 by Vortex
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
 
#include "config.h"
#include "system.h"
#include "cpu.h"
#include "button.h"
#include "button-target.h"
#include "powermgmt.h"
#include "kernel.h"
#include "backlight.h"
#include "logf.h"
#include "adc.h"

#define PIN_BTN_POWER (32*0+30)
#define PIN_BTN_HOLD  (32*1+15)

#define PIN_KEY_INT   (32*4+13)
#define KEY_INT_IRQ   GPIO141

#define PIN_CHARGE_CON (32*1+7)

static volatile unsigned short bat_val,key_val;

void button_init_device(void)
{
    key_val = 0xfff;
    
    __gpio_as_input(PIN_BTN_POWER);
    __gpio_as_input(PIN_BTN_HOLD);

    __gpio_disable_pull(PIN_BTN_POWER);
    __gpio_disable_pull(PIN_BTN_HOLD);
    
    __gpio_as_irq_fall_edge(PIN_KEY_INT);
    system_enable_irq(GPIO_IRQ(PIN_KEY_INT));

    __gpio_clear_pin(PIN_CHARGE_CON); /* 0.5 A */
    __gpio_as_output(PIN_CHARGE_CON);
}

bool button_hold(void)
{
   return (__gpio_get_pin(PIN_BTN_HOLD) ? true : false);
}

int button_read_device(void)
{
    static bool hold_button = false;
    bool hold_button_old;
    
    hold_button_old = hold_button;
    hold_button = button_hold();

    int btn = BUTTON_NONE;

#ifndef BOOTLOADER
    if (hold_button != hold_button_old) {
        backlight_hold_changed(hold_button);
    }
#endif

    if (hold_button) {
        return 0;
    }

    if (__gpio_get_pin(PIN_BTN_POWER) == 0)
        btn |= BUTTON_POWER;

    if (key_val < 261)
        btn |= BUTTON_VOL_UP;
    else
    if (key_val < 653)
        btn |= BUTTON_VOL_DOWN;
    else
    if (key_val < 1101)
        btn |= BUTTON_PREV;
    else
    if (key_val < 1498)
        btn |= BUTTON_NEXT;
    else
    if (key_val < 1839)
        btn |= BUTTON_PLAY;
    else
    if (key_val < 2213)
        btn |= BUTTON_OPTION;
    else
    if (key_val < 2600)
        btn |= BUTTON_HOME;

    REG_SADC_ADCFG = ADCFG_VBAT_SEL + ADCFG_CMD_AUX(1);
    REG_SADC_ADENA = ADENA_VBATEN + ADENA_AUXEN;
        
    return btn;
}

/* called on button press interrupt */
void KEY_INT_IRQ(void)
{
}

const unsigned short battery_level_dangerous[BATTERY_TYPES_COUNT] =
{
    /* TODO */
    3400
};

const unsigned short battery_level_shutoff[BATTERY_TYPES_COUNT] =
{
    /* TODO */
    3300
};


/* voltages (millivolt) of 0%, 10%, ... 100% when charging disabled */
const unsigned short percent_to_volt_discharge[BATTERY_TYPES_COUNT][11] =
{
    /* TODO */
    { 3300, 3653, 3701, 3735, 3768, 3790, 3833, 3900, 3966, 4056, 4140 }
};

#if CONFIG_CHARGING
/* voltages (millivolt) of 0%, 10%, ... 100% when charging enabled */
const unsigned short percent_to_volt_charge[11] =
{
    3333, 3757, 3815, 3845, 3867, 3900, 3950, 4008, 4078, 4166, 4167
};
#endif /* CONFIG_CHARGING */

/* VBAT = (BDATA/1024) * 2.5V */
#define BATTERY_SCALE_FACTOR 2500

/* Returns battery voltage from ADC [millivolts] */
int _battery_voltage(void)
{
    return (bat_val*BATTERY_SCALE_FACTOR)>>10;
}

void adc_init(void)
{
    bat_val = 0xfff;

    __cpm_start_sadc();
    mdelay(10);
    REG_SADC_ADENA = 0; /* Power Up */
    mdelay(70);
    REG_SADC_ADSTATE = 0;
    REG_SADC_ADCTRL = ADCTRL_MASK_ALL - ADCTRL_ARDYM - ADCTRL_VRDYM;
    REG_SADC_ADCFG = ADCFG_VBAT_SEL + ADCFG_CMD_AUX(1);
    REG_SADC_ADCLK = (4 << 16) | (1 << 8) | 59; /* 200KHz */
    system_enable_irq(IRQ_SADC);
}

void adc_close(void)
{
    REG_SADC_ADENA = ADENA_POWER; /* Power Down */
    __intc_mask_irq(IRQ_SADC);
    mdelay(20);
    __cpm_stop_sadc();
}

/* Interrupt handler */
void SADC(void)
{
    unsigned char state;
    unsigned char sadcstate;

    sadcstate = REG_SADC_ADSTATE;
    state = REG_SADC_ADSTATE & (~REG_SADC_ADCTRL);
    REG_SADC_ADSTATE &= sadcstate;
    
    if(state & ADCTRL_ARDYM)
    {
        key_val = REG_SADC_ADADAT;
    }
    if(state & ADCTRL_VRDYM)
    {
        bat_val = REG_SADC_ADVDAT;
    }
}
