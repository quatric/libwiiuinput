/* SPDX-FileCopyrightText: 2026 RiftWii contributors */
/* SPDX-FileCopyrightText: FIX94 and the Nintendont contributors, used with permission <https://github.com/FIX94/Nintendont> */
/* SPDX-License-Identifier: GPL-3.0-or-later */
/* USB game controllers described by Nintendont's controller.ini format: a
 * mapping from the bytes of an HID report to a GameCube pad. Taken from
 * RiftWii (github.com/KakarottoCake/Riftwii, runtime/rthid.h and rthid.c,
 * src/data/hidconfig.cpp), which wrote it from Nintendont's HID.c and
 * PADReadGC.c. Plain C with no libogc, so the host tests build it too. */
#ifndef LIBINPUT_HID_H
#define LIBINPUT_HID_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HIDPAD_VERSION 1u
#define HIDPAD_REPORT_MAX 128u
#define HIDPAD_RUMBLE_MAX 128u
#define HIDPAD_CONFIG_BYTES 8192u /* the longest controller.ini read */
#define HIDPAD_ABSENT 0xFFFFu
#define HIDPAD_BUTTONS 18u

/* The keys of controller.ini's buttons, in this order. */
enum hidpad_button {
    HIDPAD_A, HIDPAD_B, HIDPAD_X, HIDPAD_Y, HIDPAD_Z, HIDPAD_L, HIDPAD_R,
    HIDPAD_START, HIDPAD_POWER, HIDPAD_ZL, HIDPAD_LEFT, HIDPAD_DOWN,
    HIDPAD_RIGHT, HIDPAD_UP, HIDPAD_RIGHT_UP, HIDPAD_DOWN_RIGHT,
    HIDPAD_DOWN_LEFT, HIDPAD_UP_LEFT
};

typedef struct hidpad_button_map {
    uint16_t offset; /* byte in the report, HIDPAD_ABSENT when the pad has none */
    uint16_t mask;
} hidpad_button_map;

typedef struct hidpad_axis {
    uint16_t offset;
    uint16_t deadzone;
    uint32_t scale; /* thousandths, with Nintendont's deadzone adjustment */
} hidpad_axis;

typedef struct hidpad_mapping {
    uint16_t vid, pid;
    uint16_t report_bytes; /* the shortest report every offset fits in */
    uint16_t polltype;     /* 0: GET_REPORT (PS3), 1: interrupt IN */
    uint16_t dpad;         /* 0: bit masks, 1: hat values */
    uint16_t dpad_mask;
    uint16_t digital_lr;   /* 0: analog + buttons, 1: digital, 2: thresholds */
    uint16_t multi_in;     /* 0: every report, 1: only those starting with multi_value */
    uint16_t multi_value;
    uint16_t analog_l, analog_r;
    hidpad_button_map buttons[HIDPAD_BUTTONS];
    hidpad_axis axes[4];   /* X, Y, C-X, C-Y; USB's Y grows downwards */
    uint16_t rumble_type;  /* 0: SET_REPORT, 1: interrupt OUT */
    uint16_t rumble_bytes;
    uint16_t rumble_transfer;
    uint16_t rumble_count;
    uint8_t rumble_on[HIDPAD_RUMBLE_MAX];
    uint8_t rumble_off[HIDPAD_RUMBLE_MAX];
} hidpad_mapping;

/* A pad as the SDK's PADStatus carries it (the bits are libogc's
 * PAD_BUTTON_*), sticks from the centre, y up. */
typedef struct hidpad_state {
    uint16_t buttons;
    int8_t stick_x, stick_y, substick_x, substick_y;
    uint8_t trigger_l, trigger_r;
    uint8_t no_rumble; /* the file has no motor for it */
} hidpad_state;

#define HIDPAD_PAD_LEFT 0x0001u
#define HIDPAD_PAD_RIGHT 0x0002u
#define HIDPAD_PAD_DOWN 0x0004u
#define HIDPAD_PAD_UP 0x0008u
#define HIDPAD_PAD_Z 0x0010u
#define HIDPAD_PAD_R 0x0020u
#define HIDPAD_PAD_L 0x0040u
#define HIDPAD_PAD_A 0x0100u
#define HIDPAD_PAD_B 0x0200u
#define HIDPAD_PAD_X 0x0400u
#define HIDPAD_PAD_Y 0x0800u
#define HIDPAD_PAD_START 0x1000u

/* A layout the library carries without a file on the SD card: one of
 * Nintendont's controllerconfigs, as it has it (libinput_hidpads_data.c). */
typedef struct hidpad_builtin {
    uint32_t vid_pid;  /* VID << 16 | PID */
    const char *name;  /* the file's [section] name, for the log */
    const char *file;  /* the file it came from */
    const char *text;
} hidpad_builtin;

extern const hidpad_builtin hidpad_builtins[];
extern const size_t hidpad_builtin_count;

/* The built-in layout for a VID and PID, or NULL. */
const hidpad_builtin *hidpad_find_builtin(uint32_t vid_pid);

/* Reads one controller.ini. 1 and the mapping on success; 0 and a line
 * naming the bad key in `error` (at least 80 bytes) on failure, with
 * `mapping` zeroed. Strict and bounded: nothing partly read escapes. */
int hidpad_parse(const char *text, size_t length, hidpad_mapping *mapping, char *error, size_t error_size);

/* 1 when every field is within what the decoder and the driver can use. */
int hidpad_valid(const hidpad_mapping *mapping);

/* A report as a GameCube pad: 1 when `report` was one the mapping reads. */
int hidpad_decode(const hidpad_mapping *mapping, const uint8_t *report, uint32_t bytes, hidpad_state *out);

#ifdef __cplusplus
}
#endif

#endif
