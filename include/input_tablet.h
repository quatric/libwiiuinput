/* Pen tablets for libwiiuinput: the same calls as libwiiinput's.
 *
 * USB pen tablets through the system's HID library: works from the tablet's
 * own HID report descriptor, so any tablet that describes its pen there is
 * covered (Wacom's current Intuos, One and Intuos Pro lines, and the Huion,
 * XP-Pen and Gaomon tablets that use the standard digitizer page). Older
 * Wacom Bamboo / Intuos (CTL-480 era) tablets are switched into tablet mode
 * and read in their fixed format, with a Bamboo Pen & Touch's touch surface.
 *
 * The uDraw GameTablet for the Wii, plugged into a Wii Remote, comes through
 * the same calls (its samples carry the Wii Remote's number in
 * tb_sample.wiimote). It needs only INPUT_Init, not TB_Init.
 *
 * Built only with HAVE_TABLET (make WITH_TABLET=1, the default). Devices
 * from tablet makers are left to this driver, not the USB mouse driver.
 */

#ifndef INPUT_TABLET_H
#define INPUT_TABLET_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TB_OK       0
#define TB_ENODEV  -1
#define TB_ESTATE  -3
#define TB_ENOMEM  -5

/* tb_sample.buttons */
#define TB_BUTTON_TIP     0x01 /* the nib (or eraser) touching the surface */
#define TB_BUTTON_BARREL  0x02 /* lower side switch */
#define TB_BUTTON_BARREL2 0x04 /* upper side switch */

typedef struct {
    uint16_t vid, pid;
    char     name[48];        /* product string, or "VID:PID" */
    uint8_t  interface;
    uint8_t  endpoint;        /* kept for libwiiinput's layout; 0 here */
    uint16_t max_packet;
    uint8_t  report_id;       /* the pen's report */
    int32_t  max_x, max_y;    /* coordinate range, from 0 */
    uint32_t max_pressure;
    bool     has_tilt;
    bool     has_eraser;
    uint8_t  pad_buttons;     /* buttons on the tablet itself */
    bool     legacy;          /* old fixed-format Wacom protocol */
    bool     has_touch;       /* a Bamboo Pen & Touch's touch surface */
    int32_t  touch_max_x, touch_max_y;
} tb_info;

typedef struct {
    int32_t  x, y;            /* tablet units, 0..max_x / 0..max_y */
    float    nx, ny;          /* the same as 0..1 */
    float    pressure;        /* 0..1 */
    uint32_t raw_pressure;
    int8_t   tilt_x, tilt_y;  /* degrees, when has_tilt */
    uint8_t  buttons;         /* TB_BUTTON_* */
    bool     in_range;        /* the pen is near enough to be tracked */
    bool     eraser;          /* the pen is upside down */
    uint16_t pad;             /* tablet buttons held, bit n = button n */
    uint32_t time;            /* when the report arrived, in milliseconds */
    bool     finger;          /* from a touch surface rather than the pen */
    int8_t   wiimote;         /* the Wii Remote (0-3) whose uDraw sent this, or -1 */
} tb_sample;

#define TB_MAX_FINGERS 4

typedef struct {
    bool    down;
    int32_t x, y;             /* touch units */
    float   nx, ny;           /* 0..1 */
} tb_finger;

/* Where the device scan and errors go, e.g. TB_SetLog(printf). */
void TB_SetLog(int (*print)(const char *fmt, ...));

/* Starts looking for a USB tablet, now and whenever one is plugged in. */
int32_t TB_Init(void);
void    TB_Shutdown(void);

/* A USB tablet is open, or a uDraw is reporting. */
bool TB_Connected(void);
/* The USB tablet's, or else a uDraw's (name "uDraw GameTablet"). */
bool TB_GetInfo(tb_info *info);
/* The pen as last reported, by whichever tablet reported last. */
void TB_GetState(tb_sample *out);
/* Every report since the last call, oldest first; false when none are left. */
bool TB_Pop(tb_sample *out);
/* Maps a sample onto a w x h area, keeping the tablet's proportions. */
void TB_ToScreen(const tb_sample *s, int w, int h, int *x, int *y);

/* A uDraw on this Wii Remote (0-3) has reported in the last quarter second. */
bool TB_UDrawConnected(int wiimote);
/* The uDraw's drawing area in its raw units (default 0x56-0x780 by 0x65-0x5a5). */
void TB_UDrawSetArea(int32_t min_x, int32_t max_x, int32_t min_y, int32_t max_y);

/* The fingers on a touch surface now, up to `max`; returns how many. */
uint32_t TB_GetFingers(tb_finger *out, uint32_t max);

typedef struct {
    uint32_t reports;
    uint32_t pen_reports;
    uint32_t touch_reports;
    uint32_t other;
    uint32_t errors;
    int32_t  last_ret;
    uint8_t  last_id;
    uint8_t  last[64];
    uint8_t  last_len;
    int32_t  proto_ret;
    int32_t  mode_set;
    int32_t  mode_get;
    uint8_t  mode_val;
    int32_t  first_err[4];
    uint8_t  nfirst;
    uint32_t opens;
    uint32_t false_removals;
} tb_stats;

void TB_GetStats(tb_stats *out);

#ifdef __cplusplus
}
#endif

#endif /* INPUT_TABLET_H */
