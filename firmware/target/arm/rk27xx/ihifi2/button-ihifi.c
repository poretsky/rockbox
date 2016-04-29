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
#include "button.h"
#include "adc.h"
#include "backlight.h"

void button_init_device(void) {
    GPIO_PCCON &= ~(1<<1);
}

int button_read_device(void) {
    int adc_val = adc_read(ADC_BUTTONS);
    int gpio_btn = GPIO_PCDR & (1<<1);
    
    int button = BUTTON_NONE;

    if (gpio_btn)
        button |= BUTTON_POWER;

    if (adc_val < 46)
        button |= BUTTON_HOME;
    else
    if (adc_val < 170)
        button |= BUTTON_PLAY;
    else
    if (adc_val < 325)
        button |= BUTTON_NEXT;
    else
    if (adc_val < 480)
        button |= BUTTON_VOL_UP;
    else
    if (adc_val < 636)
        button |= BUTTON_VOL_DOWN;
    else
    if (adc_val < 792)
        button |= BUTTON_PREV;
    
    return button;
}
