/**
 * @file   video_capture/switcher.c
 * @author Martin Pulec <pulec@cesnet.cz>
 */
/*
 * Copyright (c) 2014-2026 CESNET, zájmové sdružení právických osob
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, is permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of CESNET nor the names of its contributors may be
 *    used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHORS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESSED OR IMPLIED WARRANTIES, INCLUDING,
 * BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 * EVENT SHALL THE AUTHORS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <assert.h>
#include <errno.h>
#include <inttypes.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "audio/types.h"
#include "debug.h"
#include "lib_common.h"
#include "messaging.h"        // for check_message, free_response
#include "module.h"
#include "utils/color_out.h"
#include "utils/macros.h"
#include "utils/string.h"     // for sprintf_append
#include "utils/text.h"
#include "video_capture.h"

#define MOD_NAME "[switcher] "

/* prototypes of functions defined in this module */
static void show_help(void);
static void vidcap_switcher_done(void *state);

static void show_help()
{
        char desc[] =
            TBOLD("switcher") " capture allow switching between given "
                              "video devices optionally with audio "
                              "(see below or wiki for usage).\n\n"
                              "Switching can be done by using keys 1-N or "
                              "via a control socket.\n\n";
        color_printf("%s", wrap_paragraph(desc));

        color_printf("Usage:\n");
        color_printf("\t" TBOLD(TRED("-t switcher")) "[opts] " TBOLD(
            "-t <dev1> -t "
            "<dev2> [-t ...]") "\n\n");

        color_printf("options:\n");
        color_printf("\t" TBOLD(
            "<devN>") " - a configuration of device to be switched\n");
        color_printf("\t" TBOLD(
            "select=<N|name>") " - specifies initially selected device idx or name\n");
        color_printf("\t" TBOLD(
            "name=<name>") " - stable name for a device, used to address it via\n"
                           "\t\t" TBOLD("capture.data <name>")
                           " (defaults to the raw device config string)\n");
        color_printf("\t" TBOLD(
            "excl_init") " - devices will be initialized after switching to "
                         "and\n\t\tdeinitialized after switching to another\n");
        color_printf("\t" TBOLD(
            "fallback") " - in case that capture doesn't return a frame (in "
                        "time),\n\t\tcapture from next available "
                        "device(s)\n\n");
        color_printf("Control socket: " TBOLD("capture.data <idx|name>")
                     " switches input,\n"
                     "\t" TBOLD("capture.data list")
                     " (or " TBOLD("list-inputs") ") lists available inputs.\n\n");

        color_printf(TBOLD("Audio") " and other " TBOLD(
            "positional") " options should precede its respective device.\n");
        color_printf("Eg. in:\n" TBOLD(
            "   uv -t switcher -s embedded -F flip -t testcard -s "
            "analog -t decklink") "\n");
        color_printf("flip & embedded applies to testcard, analog to decklink.\n");
        color_printf(
            "See "
            "also: <" TUNDERLINE("https://github.com/CESNET/UltraGrid/wiki/"
                                 "Video-Switcher#audio") ">\n\n");
}

struct vidcap_switcher_state {
        struct module       mod;
        struct vidcap     **devices;
        char              (*device_names)[128];
        char              (*names)[128]; ///< stable name for each input (from name= or raw device string)
        unsigned int        devices_cnt;

        unsigned int        selected_device;
        char                initial_select[128]; ///< raw select= value (index or name), resolved after names are built

        struct vidcap_params *params;
        bool                excl_init;
        bool                fallback;
};


static void vidcap_switcher_probe(struct device_info **available_cards, int *count, void (**deleter)(void *))
{
        *deleter = free;
        *available_cards = NULL;
        *count = 0;
}

static void vidcap_switcher_register_keyboard_ctl(struct vidcap_switcher_state *s) {
        for (unsigned int i = 0U; i < MIN(s->devices_cnt, 10); ++i) {
                struct msg_universal *m = (struct msg_universal *) new_message(sizeof(struct msg_universal));
                snprintf(m->text, sizeof m->text, "map %d capture.data %d#switch to video input %d", i + 1, i, i + 1);
                struct response *r = send_message_sync(get_root_module(&s->mod), "keycontrol", (struct message *) m, 100,  SEND_MESSAGE_FLAG_QUIET | SEND_MESSAGE_FLAG_NO_STORE);
                if (response_get_status(r) != RESPONSE_OK) {
                        log_msg(LOG_LEVEL_ERROR, "Cannot register keyboard control for video switcher (error %d)!\n", response_get_status(r));
                }
                free_response(r);
        }
}

static int
parse_fmt(struct vidcap_switcher_state *s, char *cfg)
{
        assert(cfg != NULL);
        char *save_ptr = NULL;
        char *item = NULL;
        while ((item = strtok_r(cfg, ":", &save_ptr))) {
                if (strcmp(item, "help") == 0) {
                        show_help();
                        return VIDCAP_INIT_NOERR;
                }
                if (strcmp(item, "excl_init") == 0) {
                        s->excl_init = true;
                } else if (strcmp(item, "fallback") == 0) {
                        s->fallback = true;
                } else if (strncasecmp(item, "select=", strlen("select=")) ==
                           0) {
                        // Keep the raw token; it may be an index or a name.
                        // Resolved against names[] after they are built.
                        const char *val_s = item + strlen("select=");
                        if (*val_s == '\0') {
                                log_msg(LOG_LEVEL_ERROR,
                                        MOD_NAME "Wrong value: %s\n", val_s);
                                return VIDCAP_INIT_FAIL;
                        }
                        snprintf(s->initial_select, sizeof s->initial_select,
                                 "%s", val_s);
                } else {
                        MSG(ERROR, "Unknown initialization option: %s\n", item);
                        show_help();
                        return VIDCAP_INIT_FAIL;
                }
                cfg = NULL;
        }
        if (s->excl_init && s->fallback) {
                MSG(ERROR, "Options \"excl_init\" and \"fallback\" are "
                           "mutualy incompatible!\n");
                return VIDCAP_INIT_FAIL;
        }

        return 0;
}

/**
 * Extracts an optional name=<x> token from a colon-delimited device fmt string.
 * Returns a freshly allocated fmt with the name= token removed (caller frees),
 * or NULL if @p fmt is NULL/empty. If a name= token was found, its value is
 * copied into @p name_out (NUL-terminated, truncated to @p name_sz).
 */
static char *
extract_name_from_fmt(const char *fmt, char *name_out, size_t name_sz)
{
        if (name_sz > 0) {
                name_out[0] = '\0';
        }
        if (fmt == NULL || fmt[0] == '\0') {
                return fmt == NULL ? NULL : strdup(fmt);
        }

        char *fmt_cpy = strdup(fmt);
        char *cleaned = strdup("");
        char *save_ptr = NULL;
        char *item = NULL;
        while ((item = strtok_r(fmt_cpy, ":", &save_ptr))) {
                if (strncasecmp(item, "name=", strlen("name=")) == 0) {
                        const char *val = item + strlen("name=");
                        if (name_sz > 0) {
                                snprintf(name_out, name_sz, "%s", val);
                        }
                        continue; // drop the name= token from the device config
                }
                cleaned = sprintf_append(cleaned, "%s%s",
                                         cleaned[0] == '\0' ? "" : ":",
                                         item);
        }
        free(fmt_cpy);
        return cleaned;
}

static int
vidcap_switcher_init(const struct vidcap_params *params, void **state)
{
        verbose_msg("vidcap_switcher_init\n");

        struct vidcap_switcher_state *s = calloc(1, sizeof *s);
        if (s == NULL) {
                MSG(ERROR, "Unable to allocate switcher capture state\n");
                return VIDCAP_INIT_FAIL;
        }

        const char *cfg_c = vidcap_params_get_fmt(params);
        if (cfg_c && strcmp(cfg_c, "") != 0) {
                char     *cfg = strdup(cfg_c);
                const int rc  = parse_fmt(s, cfg);
                free(cfg);
                if (rc != 0) {
                        free(s);
                        return rc;
                }
        }

        module_init_default(&s->mod);
        s->mod.cls = MODULE_CLASS_DATA;
        module_register(&s->mod, vidcap_params_get_parent(params));
        s->devices_cnt = vidcap_params_get_count(params);

        s->devices = calloc(s->devices_cnt, sizeof(struct vidcap *));
        s->device_names = calloc(s->devices_cnt, sizeof s->device_names[0]);
        s->names = calloc(s->devices_cnt, sizeof s->names[0]);

        const struct vidcap_params *tmp = params;
        for (unsigned int i = 0; i < s->devices_cnt; ++i) {
                tmp = vidcap_params_get_next(tmp);

                snprintf(s->device_names[i], sizeof s->device_names[i],
                         "%s%s%s", vidcap_params_get_driver(tmp),
                         strlen(vidcap_params_get_fmt(tmp)) > 0 ? ":" : "",
                         vidcap_params_get_fmt(tmp));

                // Stable name: explicit name=<x> in the fmt, else the raw
                // device config string (vidcap_params_get_name()).
                char explicit_name[128] = {0};
                char *clean_fmt = extract_name_from_fmt(
                        vidcap_params_get_fmt(tmp), explicit_name,
                        sizeof explicit_name);
                const char *name = explicit_name[0] != '\0'
                                           ? explicit_name
                                           : vidcap_params_get_name(tmp);
                snprintf(s->names[i], sizeof s->names[i], "%s", name);

                // A purely numeric name would collide with index addressing.
                char *endptr = NULL;
                errno = 0;
                long nval = strtol(s->names[i], &endptr, 0);
                if (explicit_name[0] != '\0' && endptr != NULL &&
                    *endptr == '\0' && errno == 0 && nval >= 0) {
                        MSG(ERROR, "Device #%u name \"%s\" is numeric and "
                                   "would collide with index addressing.\n",
                            i, s->names[i]);
                        free(clean_fmt);
                        goto error;
                }
                // Duplicate names are ambiguous when addressing by name.
                for (unsigned int j = 0; j < i; ++j) {
                        if (strcmp(s->names[i], s->names[j]) == 0) {
                                MSG(ERROR, "Duplicate input name \"%s\" for "
                                           "devices #%u and #%u.\n",
                                    s->names[i], j, i);
                                free(clean_fmt);
                                goto error;
                        }
                }

                if (s->excl_init && i != s->selected_device) {
                        free(clean_fmt);
                        continue;
                }

                struct vidcap_params *copy = vidcap_params_copy(tmp);
                // Strip the name= token from the fmt handed to the driver.
                if (clean_fmt != NULL) {
                        vidcap_params_replace_fmt(copy, clean_fmt);
                }
                if (vidcap_params_get_flags(copy) == 0 && vidcap_params_get_flags(params) != 0) {
                        vidcap_params_set_flags(copy, vidcap_params_get_flags(params));
                }

                int ret = initialize_video_capture(&s->mod, copy, &s->devices[i]);
                vidcap_params_free(copy);
                free(clean_fmt);
                if(ret != 0) {
                        MSG(ERROR,
                            "Unable to initialize device %d (%s:%s).\n",
                            i, vidcap_params_get_driver(tmp),
                            vidcap_params_get_fmt(tmp));
                        goto error;
                }
        }

        // Resolve select=<index|name> now that names[] is populated.
        if (s->initial_select[0] != '\0') {
                char *endptr = NULL;
                errno = 0;
                long val = strtol(s->initial_select, &endptr, 0);
                if (errno == 0 && *endptr == '\0' && val >= 0 &&
                    (uintmax_t) val <= UINT_MAX) {
                        s->selected_device = val;
                } else {
                        s->selected_device = UINT_MAX; // sentinel: unresolved
                        for (unsigned int i = 0; i < s->devices_cnt; ++i) {
                                if (strcmp(s->initial_select, s->names[i]) == 0) {
                                        s->selected_device = i;
                                        break;
                                }
                        }
                        if (s->selected_device == UINT_MAX) {
                                MSG(ERROR, "Error: select= device \"%s\" not "
                                           "available!\n", s->initial_select);
                                goto error;
                        }
                }
        }

        if (s->selected_device >= s->devices_cnt) {
                MSG(ERROR, "Error: device #%d not available!\n",
                    s->selected_device);
                goto error;
        }

        s->params = vidcap_params_copy_all(params);

        vidcap_switcher_register_keyboard_ctl(s);

        *state = s;
	return VIDCAP_INIT_OK;

error:
        vidcap_switcher_done(s);
        return VIDCAP_INIT_FAIL;
}

static void
vidcap_switcher_done(void *state)
{
	struct vidcap_switcher_state *s = (struct vidcap_switcher_state *) state;
        if (s == NULL) {
                return;
        }

	if (s->devices != NULL) {
		for (unsigned int i = 0U; i < s->devices_cnt; ++i) {
                        if (s->devices[i] != NULL) {
                                vidcap_done(s->devices[i]);
                        }
		}
	}
        module_done(&s->mod);
        free(s->devices);
        free(s->device_names);
        free(s->names);
        vidcap_params_free(s->params);
        free(s);
}

static struct video_frame *
vidcap_switcher_grab(void *state, struct audio_frame **audio)
{
	struct vidcap_switcher_state *s = (struct vidcap_switcher_state *) state;
        struct audio_frame *audio_frame = NULL;
        struct video_frame *frame = NULL;

        struct message *msg;
        while ((msg = check_message(&s->mod))) {
                struct msg_universal *msg_univ = (struct msg_universal *) msg;
                const char *text = msg_univ->text;

                // Query command: enumerate configured inputs.
                if (strcasecmp(text, "list") == 0 ||
                    strcasecmp(text, "list-inputs") == 0) {
                        char buffer[2048];
                        char *p = buffer;
                        char *end = buffer + sizeof buffer;
                        p += snprintf(p, end - p,
                                      "\n Idx  %-32s  %s\n", "Name", "Device");
                        for (unsigned int i = 0; i < s->devices_cnt; ++i) {
                                p += snprintf(p, end - p,
                                              " %-3u  %-32s  %s\n",
                                              i, s->names[i],
                                              s->device_names[i]);
                        }
                        log_msg(LOG_LEVEL_NOTICE, "%s", buffer);
                        free_message(msg, new_response(RESPONSE_OK, buffer));
                        continue;
                }

                // Resolve the token to an input index: all-digits => numeric
                // index (backward compatible), otherwise match a stable name.
                int new_selected_device = -1;
                char *endptr = NULL;
                errno = 0;
                long val = strtol(text, &endptr, 0);
                if (errno == 0 && text[0] != '\0' && *endptr == '\0' &&
                    val >= 0 && (uintmax_t) val <= UINT_MAX) {
                        new_selected_device = val;
                } else {
                        for (unsigned int i = 0; i < s->devices_cnt; ++i) {
                                if (strcmp(text, s->names[i]) == 0) {
                                        new_selected_device = i;
                                        break;
                                }
                        }
                }

                struct response *r;
                if (new_selected_device >= 0 &&
                    (unsigned int) new_selected_device < s->devices_cnt) {
                        unsigned int nd = new_selected_device;
                        MSG(NOTICE,
                            "Switched from device %d to device %d (%s).\n",
                            s->selected_device + 1, nd + 1,
                            s->device_names[nd]);
                        if (s->excl_init) {
                                vidcap_done(s->devices[s->selected_device]);
                                s->devices[s->selected_device] = NULL;
                                int ret = initialize_video_capture(&s->mod,
                                                vidcap_params_get_nth((struct vidcap_params *) s->params, nd + 1),
                                                &s->devices[nd]);
                                assert(ret == 0);
                        }

                        s->selected_device = nd;
                        r = new_response(RESPONSE_OK, NULL);
                } else {
                        log_msg(LOG_LEVEL_ERROR,
                                "[switcher] Cannot switch to device %s. "
                                "Unknown device or wrong value.\n", text);
                        r = new_response(RESPONSE_BAD_REQUEST, NULL);
                }
                free_message(msg, r);
        }

        frame = vidcap_grab(s->devices[s->selected_device], &audio_frame);
        *audio = audio_frame;

        if (frame || !s->fallback) {
                return frame;
        }
        // if frame was not returned but we have a fallback behavior, try also other devices
        for (unsigned int i = (s->selected_device + 1U) % s->devices_cnt;
                        i != s->selected_device;
                        i = (s->selected_device + 1U) % s->devices_cnt) {
                frame = vidcap_grab(s->devices[i], &audio_frame);
                *audio = audio_frame;
                if (frame != NULL) {
                        log_msg(LOG_LEVEL_INFO,
                                MOD_NAME
                                "Frame timed out, using fallback device #%d\n",
                                i);
                        break;
                }
        }
        return frame;
}

static const struct video_capture_info vidcap_switcher_info = {
        vidcap_switcher_probe,
        vidcap_switcher_init,
        vidcap_switcher_done,
        vidcap_switcher_grab,
        MOD_NAME,
};

REGISTER_MODULE(switcher, &vidcap_switcher_info, LIBRARY_CLASS_VIDEO_CAPTURE, VIDEO_CAPTURE_ABI_VERSION);

