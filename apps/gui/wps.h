/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 * $Id$
 *
 * Copyright (C) 2002 Björn Stenberg
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
#ifndef _WPS_H_
#define _WPS_H_

#include <stdbool.h>

struct mp3entry;

/* Please don't add anything else to here... */
struct wps_state
{
    struct mp3entry *id3;
    struct mp3entry *nid3;
    int ff_rewind_count;
    bool paused;
};

long gui_wps_show(void);

/* fade (if enabled) and pause the audio, optionally rewind a little */
void pause_action(bool updatewps);
void unpause_action(bool updatewps);
void wps_do_playpause(bool updatewps);

bool ffwd_rew(int button, bool seek_from_end);

struct wps_state *get_wps_state(void);

/* in milliseconds */
#define DEFAULT_SKIP_THRESH          3000l

#endif /* _WPS_H_ */
