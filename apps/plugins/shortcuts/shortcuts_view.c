/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 * $Id$
 *
 * Copyright (C) 2007 Bryan Childs
 * Copyright (c) 2007 Alexander Levin
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

#include "shortcuts.h"

#define LOOP_EXIT 1

enum sc_list_action_type
{
    SCLA_NONE,
    SCLA_SELECT,
    SCLA_DELETE,
    SCLA_USB,
};

static size_t root_len;
static char *link_filename;
static bool user_file;

enum sc_list_action_type draw_sc_list(struct gui_synclist *gui_sc);

/* Will be passed sc_file* as data */
static const char* build_sc_list(int selected_item, void *data,
                                 char *buffer, size_t buffer_len);

/* Returns LOOP_EXIT iff we should leave the main loop */
int list_sc(int *selected_item);

int goto_entry(char *file_or_dir);
bool ends_with(char *str, char *suffix);


enum sc_list_action_type draw_sc_list(struct gui_synclist *gui_sc)
{
    int button;

    rb->gui_synclist_draw(gui_sc);

    while (true) {
        /* user input */
        button = rb->get_action(CONTEXT_LIST, HZ);
        /* HZ so the status bar redraws corectly */
        if (rb->gui_synclist_do_button(gui_sc, &button))
            continue;
        switch (button) { /* process the user input */
            case ACTION_STD_OK:
                return SCLA_SELECT;
            case ACTION_STD_MENU:
                /* Only allow delete entries in the default file
                 * since entries can be appended (with a plugin)
                 * to the default file only. The behaviour is thus
                 * symmetric in this respect. */
                if (!user_file) {
                    return SCLA_DELETE;
                }
                break;
            case ACTION_STD_CANCEL:
                return SCLA_NONE;
            default:
                if (rb->default_event_handler(button) == SYS_USB_CONNECTED) {
                    return SCLA_USB;
                }
        }
    }
}


static const char* build_sc_list(int selected_item, void *data,
                                 char *buffer, size_t buffer_len)
{
    sc_file_t *file = (sc_file_t*)data;

    if (!is_valid_index(file, selected_item)) {
        return NULL;
    }
    sc_entry_t *entry = file->entries + selected_item;
    rb->snprintf(buffer, buffer_len, "%s", entry->disp);
    return buffer;
}


int list_sc(int *selected_item)
{
    enum sc_list_action_type action = SCLA_NONE;
    struct gui_synclist gui_sc;

    /* Setup the GUI list object, draw it to the screen,
     * and then handle the user input to it */
    rb->gui_synclist_init(&gui_sc, &build_sc_list, &sc_file, false, 1, NULL);
    rb->gui_synclist_set_title(&gui_sc,
        (user_file?"Shortcuts (sealed)":"Shortcuts (editable)"), NOICON);
    rb->gui_synclist_set_nb_items(&gui_sc, sc_file.entry_cnt);
    rb->gui_synclist_select_item(&gui_sc, *selected_item);

    /* Draw the prepared widget to the LCD now */
    action = draw_sc_list(&gui_sc);
    if (action == SCLA_USB) {
        return PLUGIN_USB_CONNECTED;
    }

    /* which item do we action? */
    *selected_item = rb->gui_synclist_get_sel_pos(&gui_sc);

    if (!is_valid_index(&sc_file, *selected_item)) {
        /* This should never happen */
        rb->splash(HZ*2, "Bad entry selected!");
        return PLUGIN_ERROR;
    }

    /* perform the following actions if the user "selected"
     * the item in the list (i.e. they want to go there
     * in the filebrowser tree */
    switch(action) {
        case SCLA_SELECT:
            return goto_entry(sc_file.entries[*selected_item].path);
        case SCLA_DELETE:
            rb->splashf(HZ, "Deleting %s", sc_file.entries[*selected_item].disp);
            remove_entry(&sc_file, *selected_item);
            dump_sc_file(&sc_file, link_filename);
            return (sc_file.entry_cnt == 0)? LOOP_EXIT : PLUGIN_OK;
        default:
            return LOOP_EXIT;
    }
}

#if 0
bool goto_entry(char *file_or_dir)
{
    DEBUGF("Trying to go to '%s'...\n", file_or_dir);

    bool is_dir = ends_with(file_or_dir, PATH_SEPARATOR);
    bool exists;
    char *what;
    if (is_dir) {
        what = "Directory";
        exists = rb->dir_exists(file_or_dir);
    } else {
        what = "File";
        exists = rb->file_exists(file_or_dir);
    }

    if (!exists) {
        rb->splashf(HZ*2, "%s %s no longer exists on disk", what, file_or_dir);
        return false;
    }
    /* Set the browsers dirfilter to the global setting
     * This is required in case the plugin was launched
     * from the plugins browser, in which case the
     * dirfilter is set to only display .rock files */
    rb->set_dirfilter(rb->global_settings->dirfilter);

    /* Change directory to the entry selected by the user */
    rb->set_current_file(file_or_dir);
    return true;
}
#endif

static bool callback_show_item(char *name, int attr, struct tree_context *tc)
{
    (void)name;
    if(attr & ATTR_DIRECTORY)
    {
        if ((tc->browse->flags & BROWSE_SELECTED) == 0)
        {
            size_t curlen = rb->strlen(tc->currdir);
            if (curlen < root_len)
                tc->is_browsing = false; /* exit immediately */
            else if (curlen == root_len)
                tc->browse->title = tc->currdir;
        }
    }

    return true;
}

bool open_browse(char *path, char *buf, size_t bufsz)
{
    struct browse_context browse = {
        .dirfilter = rb->global_settings->dirfilter,
        .flags = BROWSE_DIRFILTER| BROWSE_SELECTONLY | BROWSE_NO_CONTEXT_MENU,
        .title = path,
        .icon = Icon_Plugin,
        .root = path,
        .buf = buf,
        .bufsize = bufsz,
        .callback_show_item = callback_show_item,
    };

    /* if the final slash isn't included return brings you up a level, feature not bug. */
    root_len = 0;
    char *name = rb->strrchr(path, '/');
    if (name)
        root_len = name - path;
    rb->rockbox_browse(&browse);

    return (browse.flags & BROWSE_SELECTED);
}

int goto_entry(char *file_or_dir)
{
    DEBUGF("Trying to go to '%s'...\n", file_or_dir);

    bool is_dir = ends_with(file_or_dir, PATH_SEPARATOR) || rb->dir_exists(file_or_dir);
    bool exists;
    char *what;
    if (is_dir) {
        what = "Directory";
        exists = rb->dir_exists(file_or_dir);
    } else {
        what = "File";
        exists = rb->file_exists(file_or_dir);
    }

    if (!exists) {
        rb->splashf(HZ*2, "%s %s no longer exists on disk", what, file_or_dir);
        return PLUGIN_ERROR;
    }

    int len = rb->strlen(file_or_dir);
    if(!is_dir && len > 5 && rb->strcasecmp(&(file_or_dir[len-5]), ".rock") == 0)
    {
        return rb->plugin_open(file_or_dir, NULL);
    }
    else
    {
        if (!is_dir)
        {
            rb->set_current_file(file_or_dir);
            return LOOP_EXIT;
        }
        char tmp_buf[MAX_PATH];
        if (open_browse(file_or_dir, tmp_buf, sizeof(tmp_buf)))
        {
            DEBUGF("Trying to load '%s'...\n", tmp_buf);
            rb->set_dirfilter(rb->global_settings->dirfilter);
            rb->set_current_file(tmp_buf);
            return LOOP_EXIT;
        }
    }
    return PLUGIN_OK;
}

bool ends_with(char *string, char *suffix)
{
    unsigned int str_len = rb->strlen(string);
    unsigned int sfx_len = rb->strlen(suffix);
    if (str_len < sfx_len)
        return false;
    return (rb->strncmp(string + str_len - sfx_len, suffix, sfx_len) == 0);
}


enum plugin_status plugin_start(const void* void_parameter)
{
    int ret;

    /* This is a viewer, so a parameter must have been specified */
    if (void_parameter == NULL) {
        rb->splash(HZ*2, "No parameter specified!");
        return PLUGIN_ERROR;
    }
    link_filename = (char*)void_parameter;
    user_file = (rb->strcmp(link_filename, SHORTCUTS_FILENAME) != 0);

    allocate_memory(&memory_buf, &memory_bufsize);

    if (!load_sc_file(&sc_file, link_filename, true,
            memory_buf, memory_bufsize)) {
        DEBUGF("Could not load %s\n", link_filename);
        return PLUGIN_ERROR;
    }
    if (sc_file.entry_cnt==0) {
        rb->splash(HZ*2, "No shortcuts in the file!");
        return PLUGIN_OK;
    } else if ((sc_file.entry_cnt==1) && user_file) {
        /* if there's only one entry in the user .link file,
         * go straight to it without displaying the menu
         * thus allowing 'quick links' */
        return goto_entry(sc_file.entries[0].path);
    }

    FOR_NB_SCREENS(i)
        rb->viewportmanager_theme_enable(i, true, NULL);

    int selected_item = 0;
    do {
        /* Display a menu to choose between the entries */
        DEBUGF("list_sc(%d)\n", selected_item);
        ret = list_sc(&selected_item);
    } while (ret == PLUGIN_OK);
    if (ret == LOOP_EXIT)
        ret = PLUGIN_OK;

    FOR_NB_SCREENS(i)
        rb->viewportmanager_theme_undo(i, false);

    return ret;
}
