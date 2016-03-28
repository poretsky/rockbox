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
#include "audio.h"
#include "sound.h"
#include "cpu.h"
#include "system.h"
#include "pcm_sw_volume.h"
#include "cs4398.h"
#include "kernel.h"

#define PIN_CS_RST      (32*1+10)
#define PIN_CODEC_PWRON (32*1+13)
#define PIN_AP_MUTE     (32*1+14)
#define PIN_JD_CON      (32*1+16)

#define PIN_PH_DECT   (32*1+11)
#define IRQ_PH_DECT   GPIO_IRQ(PIN_PH_DECT)
#define GPIO_PH_DECT  GPIO43

#define PIN_LO_DECT   (32*1+12)
#define IRQ_LO_DECT   GPIO_IRQ(PIN_LO_DECT)
#define GPIO_LO_DECT  GPIO44

static inline bool ph_detect(void)
{
    return (__gpio_get_pin(PIN_PH_DECT) == 0);
}

static inline bool lo_detect(void)
{
    return (__gpio_get_pin(PIN_LO_DECT) == 0);
}

static void ph_gpio_setup_irq(void)
{
    if(ph_detect())
        __gpio_as_irq_rise_edge(PIN_PH_DECT);
    else
        __gpio_as_irq_fall_edge(PIN_PH_DECT);
}

static void lo_gpio_setup_irq(void)
{
    if(lo_detect())
        __gpio_as_irq_rise_edge(PIN_LO_DECT);
    else
        __gpio_as_irq_fall_edge(PIN_LO_DECT);
}

static int ph_oneshot_callback(struct timeout *tmo)
{
    (void)tmo;
    
    /* This is called only if the state was stable for 500ms - check state
     * and post appropriate event. */
    if (ph_detect())
    {
        /* TODO */
    }
    else
    {
        //audio_pause();
    }

    ph_gpio_setup_irq();

    return 0;
}

static int lo_oneshot_callback(struct timeout *tmo)
{
    (void)tmo;

    /* This is called only if the state was stable for 500ms - check state
     * and post appropriate event. */
    if (lo_detect())
    {
        /* TODO */
    }
    else
    {
        //audio_pause();
    }

    lo_gpio_setup_irq();

    return 0;
}

/* called on insertion/removal interrupt */
void GPIO_PH_DECT(void)
{
    static struct timeout ph_oneshot;
    timeout_register(&ph_oneshot, ph_oneshot_callback, (HZ/2), 0);
}

void GPIO_LO_DECT(void)
{
    static struct timeout lo_oneshot;
    timeout_register(&lo_oneshot, lo_oneshot_callback, (HZ/2), 0);
}

static void pop_ctrl(const int val)
{
    if(val)
        __gpio_clear_pin(PIN_JD_CON);
    else
        __gpio_set_pin(PIN_JD_CON);
}

static void amp_enable(const int val)
{
    if(val)
        __gpio_set_pin(PIN_CODEC_PWRON);
    else
        __gpio_clear_pin(PIN_CODEC_PWRON);
}

static void dac_enable(const int val)
{
    if(val)
        __gpio_set_pin(PIN_CS_RST);
    else
        __gpio_clear_pin(PIN_CS_RST);
}

static void ap_mute(bool mute)
{
    if(mute)
        __gpio_clear_pin(PIN_AP_MUTE);
    else
        __gpio_set_pin(PIN_AP_MUTE);
}

static void audiohw_mute(bool mute)
{
    if(mute)
        cs4398_write_reg(0x04, cs4398_read_reg(0x04) | CS4398_MUTE_A | CS4398_MUTE_B);
    else
        cs4398_write_reg(0x04, cs4398_read_reg(0x04) & ~(CS4398_MUTE_A | CS4398_MUTE_B));
}

void audiohw_preinit(void)
{
    cs4398_write_reg(0x08, CS4398_CPEN | CS4398_PDN);
    cs4398_write_reg(0x02, CS4398_FM_SINGLE | CS4398_DEM_NONE | CS4398_DIF_LJUST);
    cs4398_write_reg(0x03, CS4398_ATAPI_A_L | CS4398_ATAPI_B_R);
    cs4398_write_reg(0x04, CS4398_MUTEP_LOW);
    cs4398_write_reg(0x05, 0xff);
    cs4398_write_reg(0x06, 0xff);
    cs4398_write_reg(0x07, 0);
    cs4398_write_reg(0x08, CS4398_CPEN);
}

void audiohw_init(void)
{
    __gpio_as_func1(3*32+12); // BCK
    __gpio_as_func0(3*32+13); // LRCK
    __gpio_as_func2(4*32+5);  // MCLK
    __gpio_as_func0(4*32+7);  // DO

    __gpio_as_input(PIN_LO_DECT);
    __gpio_as_input(PIN_PH_DECT);

    __gpio_disable_pull(PIN_LO_DECT);
    __gpio_disable_pull(PIN_PH_DECT);

    ph_gpio_setup_irq();
    lo_gpio_setup_irq();

    system_enable_irq(IRQ_PH_DECT);
    system_enable_irq(IRQ_LO_DECT);

    pop_ctrl(0);
    ap_mute(true);
    amp_enable(0);
    dac_enable(0);

    __gpio_as_output(PIN_JD_CON);
    __gpio_as_output(PIN_AP_MUTE);
    __gpio_as_output(PIN_CODEC_PWRON);
    __gpio_as_output(PIN_CS_RST);

    mdelay(100);
    amp_enable(1);

    /* set AIC clk PLL1 */
    __cpm_select_i2sclk_pll();
    __cpm_select_i2sclk_pll1();

    __cpm_enable_pll_change();
    __cpm_set_i2sdiv(43-1);

    __cpm_start_aic();

    /* Init AIC */
    __i2s_enable_sclk();
    __i2s_external_codec();
    __i2s_select_msbjustified();
    __i2s_as_master();
    __i2s_enable_transmit_dma();
    __i2s_set_transmit_trigger(24);
    __i2s_set_oss_sample_size(16);
    __i2s_enable();

    /* Init DAC */
    dac_enable(1);
    udelay(1);
    audiohw_preinit();
}

static int vol_tenthdb2hw(const int tdb)
{
    if (tdb < CS4398_VOLUME_MIN) {
        return 0xff;
    } else if (tdb > CS4398_VOLUME_MAX) {
        return 0x00;
    } else {
        return (-tdb/5);
    }
}

void audiohw_set_volume(int vol_l, int vol_r)
{
    cs4398_write_reg(0x05, vol_tenthdb2hw(vol_l));
    cs4398_write_reg(0x06, vol_tenthdb2hw(vol_r));
}

void audiohw_set_filter_roll_off(int value)
{
    /* 0 = fast (sharp); 
       1 = slow */
    if (value == 0) {
        cs4398_write_reg(0x07, cs4398_read_reg(0x07) & ~CS4398_FILT_SEL);
    } else {
        cs4398_write_reg(0x07, cs4398_read_reg(0x07) | CS4398_FILT_SEL);
    }
}

void audiohw_set_functional_mode(int value)
{
    /* 0 = Single-Speed Mode;
       1 = Double-Speed Mode;
       2 = Quad-Speed Mode; */
    cs4398_write_reg(0x02, (cs4398_read_reg(0x02) & ~CS4398_FM_MASK) | value);
    if (value == 2)
        cs4398_write_reg(0x08, cs4398_read_reg(0x08) | CS4398_MCLKDIV2);
    else
        cs4398_write_reg(0x08, cs4398_read_reg(0x08) & ~CS4398_MCLKDIV2);
}

void pll1_init(unsigned int freq);
void audiohw_set_frequency(int fsel)
{
    unsigned int  pll1_speed;
    unsigned char mclk_div, bclk_div;;

    switch(fsel)
    {
        case HW_FREQ_8:
            pll1_speed = 426000000;
            mclk_div = 52;
            bclk_div = 16;
            break;
        case HW_FREQ_11:
            pll1_speed = 508000000;
            mclk_div = 45;
            bclk_div = 16;
            break;
        case HW_FREQ_12:
            pll1_speed = 516000000;
            mclk_div = 42;
            bclk_div = 16;
            break;
        case HW_FREQ_16:
            pll1_speed = 426000000;
            mclk_div = 52;
            bclk_div = 8;
            break;
        case HW_FREQ_22:
            pll1_speed = 508000000;
            mclk_div = 45;
            bclk_div = 8;
            break;
        case HW_FREQ_24:
            pll1_speed = 516000000;
            mclk_div = 42;
            bclk_div = 8;
            break;
        case HW_FREQ_32:
            pll1_speed = 426000000;
            mclk_div = 52;
            bclk_div = 4;
            break;
        case HW_FREQ_44:
            pll1_speed = 508000000;
            mclk_div = 45;
            bclk_div = 4;
            break;
        case HW_FREQ_48:
            pll1_speed = 516000000;
            mclk_div = 42;
            bclk_div = 4;
            break;
        case HW_FREQ_64:
            pll1_speed = 426000000;
            mclk_div = 52;
            bclk_div = 2;
            break;
        case HW_FREQ_88:
            pll1_speed = 508000000;
            mclk_div = 45;
            bclk_div = 2;
            break;
        case HW_FREQ_96:
            pll1_speed = 516000000;
            mclk_div = 42;
            bclk_div = 2;
            break;
        default:
            return;
    }
    
    __i2s_stop_bitclk();
    pll1_init(pll1_speed);
    __cpm_enable_pll_change();
    __cpm_set_i2sdiv(mclk_div-1);
    __i2s_set_i2sdiv(bclk_div-1);
    __i2s_start_bitclk();
}

void audiohw_postinit(void)
{
    sleep(HZ);
    audiohw_mute(false);
    ap_mute(false);
    pop_ctrl(1);
}

void audiohw_close(void)
{
    pop_ctrl(0);
    sleep(HZ/10);
    ap_mute(true);
    audiohw_mute(true);
    amp_enable(0);
    dac_enable(0);
    __i2s_disable();
    __cpm_stop_aic();
    sleep(HZ);
    pop_ctrl(1);
}
