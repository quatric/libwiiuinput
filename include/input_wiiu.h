#ifndef INPUT_WIIU_H
#define INPUT_WIIU_H

/* libwiiuinput: every Wii U controller through one set of calls, the Wii U
 * port of libwiiinput. Built on wut. */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <padscore/wpad.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------------
 * Buttons
 *
 * Every controller (Wii U GamePad, Wii Remote, Nunchuk, Classic Controller,
 * Wii U Pro Controller, the GameCube controller adapter, USB game
 * controllers) and the keyboard are folded into one mask of WPAD_BUTTON_*
 * bits (<padscore/wpad.h>, the same values as on the Wii), plus two extras
 * for the Nunchuk. The default layout, as in libwiiinput:
 *   A = A, B = B, X = 2, Y = 1, HOME = HOME, D-pad = D-pad,
 *   + / R / ZR = PLUS, - / L / ZL = MINUS.
 * ---------------------------------------------------------------------- */
#define INPUT_BUTTON_Z (1u << 28) /* Nunchuk Z */
#define INPUT_BUTTON_C (1u << 29) /* Nunchuk C */

/* Mouse button bits */
#define INPUT_MOUSE_LEFT   (1 << 0)
#define INPUT_MOUSE_RIGHT  (1 << 1)
#define INPUT_MOUSE_MIDDLE (1 << 2)

/* Analog sticks */
#define INPUT_STICK_LEFT  0
#define INPUT_STICK_RIGHT 1

/* Keyboard LEDs (kept for libwiiinput's API; the Wii U's keyboard library
 * drives the LEDs itself, so INPUT_KeyboardSetLed always fails) */
typedef enum {
    INPUT_LED_NUM = 0,
    INPUT_LED_CAPS,
    INPUT_LED_SCROLL
} input_led;

/* INPUT_InitEx() flags */
#define INPUT_INIT_NO_POINTER   (1u << 0) /* Wii Remote IR off (buttons only) */
#define INPUT_INIT_NO_MOUSE     (1u << 1)
#define INPUT_INIT_NO_KEYBOARD  (1u << 2)
#define INPUT_INIT_NO_GCADAPTER (1u << 3) /* leave the GameCube controller adapter alone */
#define INPUT_INIT_NO_HIDPADS   (1u << 4) /* leave USB game controllers alone */
#define INPUT_INIT_NO_GAMEPAD   (1u << 5) /* don't read the GamePad (VPAD) */
#define INPUT_INIT_NO_REMOTES   (1u << 6) /* don't start KPAD (Wii Remotes, Pro Controllers) */

/* Initialize everything: the GamePad, Wii Remotes (with Nunchuk and
 * Classic Controller), Wii U Pro Controllers, the GameCube controller
 * adapter, USB game controllers, USB keyboards and USB mice. Returns 0 on
 * success. Safe to call twice. The HOME button menu is left as the app set
 * it (OSEnableHomeButtonMenu); disable it to read HOME as a button. */
int32_t INPUT_Init(void);
int32_t INPUT_InitEx(uint32_t flags);

/* Stop rumble, background work and release devices. */
void INPUT_Shutdown(void);

/* Screen size used for the IR pointer, GamePad touch and mouse bounds
 * (default 1280x720, the TV). */
void INPUT_SetScreenSize(int width, int height);

/* Poll every device once. Call once per frame. "Pressed" and "down" report
 * what happened since the previous call. */
void INPUT_Update(void);

/* ---- Controller buttons (WPAD_BUTTON_* | INPUT_BUTTON_*) ---- */
uint32_t INPUT_PadsHeld(void);     /* controllers only */
uint32_t INPUT_PadsDown(void);
uint32_t INPUT_ButtonsHeld(void);  /* controllers + keyboard map */
uint32_t INPUT_ButtonsDown(void);
uint32_t INPUT_ButtonsUp(void);

/* Name of the device that most recently produced input ("GamePad",
 * "Wiimote", "Nunchuk", "Classic", "Wii U Pro", "GameCube", "USB Pad",
 * "Touch", "Keyboard", "Mouse"), or "" if nothing has been used yet. */
const char *INPUT_Source(void);

/* ---- Analog sticks ----
 * x grows right, y grows up, range -1..1 (length clamped to 1, no deadzone
 * applied). The strongest stick across all connected devices wins. Returns
 * the length; *src (optional) receives the device name. */
float INPUT_Stick(int which, float *x, float *y, const char **src);
/* The same stick before its length is clamped to 1. The GamePad, Pro and
 * Classic sticks come from the system already scaled to about -1..1; the
 * GameCube stick is divided by 70 and its C-stick by 60. */
float INPUT_StickRaw(int which, float *x, float *y, const char **src);

/* ---- Wii Remote pointer / motion ---- */
/* IR cursor in screen coordinates, smoothed; false if no remote is
 * pointing at the screen. */
bool  INPUT_Pointer(float *x, float *y);
/* Largest acceleration magnitude (in g) over the Wii Remotes and GamePad. */
float INPUT_Gforce(void);

/* ---- GamePad touch screen ---- */
/* Touch position in screen coordinates; false while nothing touches it. */
bool INPUT_Touch(int *x, int *y);
/* Fold a touch into the button masks as `button` (e.g. WPAD_BUTTON_A, so a
 * tap clicks where a pointer UI expects A). 0, the default, leaves it out. */
void INPUT_SetTouchButton(uint32_t button);

/* ---- Pointer cursor ----
 * One on-screen cursor driven by GamePad touch, the Wii Remote IR pointer,
 * a USB mouse and optionally the left stick. Hand-over between devices is
 * smooth. Draw your own cursor at INPUT_CursorPosition() while
 * INPUT_CursorActive() is true. */
#define INPUT_CURSOR_IR    (1u << 0) /* Wii Remote pointer */
#define INPUT_CURSOR_MOUSE (1u << 1) /* USB mouse */
#define INPUT_CURSOR_STICK (1u << 2) /* left analog stick (off by default) */
#define INPUT_CURSOR_TOUCH (1u << 3) /* GamePad touch screen */

/* Which devices drive the cursor. Default IR | MOUSE | TOUCH; 0 disables it. */
void     INPUT_SetCursorMode(uint32_t modes);
uint32_t INPUT_GetCursorMode(void);
bool     INPUT_CursorActive(void);
void     INPUT_CursorPosition(int *x, int *y);
/* True when the cursor moved more than a few pixels since the previous
 * call to INPUT_Update() (IR jitter is ignored). */
bool     INPUT_CursorMoved(void);

/* ---- Every controller, one by one ----
 * The controllers seen by the last INPUT_Update(), for test screens,
 * player assignment and per-player input. */
typedef enum {
    INPUT_DEV_GAMEPAD = 0,  /* Wii U GamePad (VPAD) */
    INPUT_DEV_WIIMOTE,      /* Wii Remote, alone or with an extension */
    INPUT_DEV_PRO,          /* Wii U Pro Controller */
    INPUT_DEV_GAMECUBE,     /* a console GameCube port (Wii only, never listed here) */
    INPUT_DEV_USBPAD,       /* USB game controller with a Nintendont layout */
    INPUT_DEV_GCADAPTER,    /* a port of the GameCube controller adapter */
    INPUT_DEV_DS3           /* DualShock 3 through libsicksaxis (Wii only; here it is a USB pad) */
} input_device_kind;

typedef struct {
    input_device_kind kind;
    int         port;       /* channel / adapter port / slot, from 0 */
    const char *name;       /* "GamePad", "Wiimote", "Nunchuk", "Classic", "Wii U Pro", "GameCube", "USB Pad" */
    uint32_t    held;       /* folded WPAD_BUTTON_* | INPUT_BUTTON_* */
    uint32_t    down;
    uint32_t    raw;        /* the device's own bits: VPAD_BUTTON_*, WPAD_BUTTON_* (Wii Remote),
                               WPAD_PRO_BUTTON_*, INPUT_GC_* (adapter and USB pads) */
    uint32_t    raw_ext;    /* a Wii Remote's extension: WPAD_NUNCHUK_BUTTON_* or WPAD_CLASSIC_BUTTON_* */
    float       lx, ly, rx, ry; /* sticks, -1..1, y up; 0 where there is none */
    float       lt, rt;     /* analog triggers 0..1 (Classic, GameCube, USB pad), else 0 */
    int         battery;    /* 0..100, -1 when unknown */
    bool        pointer;    /* Wii Remote: IR sees the sensor bar; GamePad: touched */
    float       px, py;     /* where, in screen coordinates */
} input_device;

int  INPUT_DeviceCount(void);
bool INPUT_DeviceGet(int index, input_device *out);
/* Rumble the device at `index` in the current list (GamePad, Wii Remote,
 * Pro Controller, adapter port, or a USB pad whose layout has a motor). It
 * keeps rumbling until turned off; INPUT_Shutdown stops everything. */
void INPUT_DeviceRumble(int index, bool on);

/* ---- Keyboard (USB HID usage codes, e.g. 0x28 = Enter), through nsyskbd ---- */
bool INPUT_KeyboardConnected(void);
bool INPUT_KeyHeld(uint8_t keycode);      /* live state */
bool INPUT_KeyPressed(uint8_t keycode);   /* since the last INPUT_Update() */
bool INPUT_KeyReleased(uint8_t keycode);
int32_t INPUT_KeyboardSetLed(input_led led, bool on);

#define INPUT_MOD_SHIFT (1 << 0)
#define INPUT_MOD_CTRL  (1 << 1)
#define INPUT_MOD_ALT   (1 << 2)
#define INPUT_MOD_GUI   (1 << 3)
uint8_t INPUT_KeyModifiers(void);

typedef struct {
    uint8_t code;   /* USB HID usage code */
    char    ch;     /* printable ASCII for the press (layout aware), else 0 */
    uint8_t mods;   /* INPUT_MOD_* held at the time of the press */
} input_key_event;

/* Pop the next key press (modifier keys alone are not reported). */
bool INPUT_KeyPop(input_key_event *ev);

/* Keyboard-as-controller mapping. The default: arrows = D-pad, Enter = A,
 * Backspace = B, X = 1, Y = 2, Delete = -, Esc/Home = HOME. */
void     INPUT_KeyboardMapClear(void);
void     INPUT_KeyboardMapSet(uint8_t keycode, uint32_t button);
uint32_t INPUT_KeyboardPadHeld(void);
uint32_t INPUT_KeyboardPadDown(void);

/* ---- GameCube pad layout (the adapter and USB pads) ----
 * The bits are libogc's PAD_BUTTON_* (wut has none of its own). */
#define INPUT_GC_LEFT  0x0001
#define INPUT_GC_RIGHT 0x0002
#define INPUT_GC_DOWN  0x0004
#define INPUT_GC_UP    0x0008
#define INPUT_GC_Z     0x0010
#define INPUT_GC_R     0x0020
#define INPUT_GC_L     0x0040
#define INPUT_GC_A     0x0100
#define INPUT_GC_B     0x0200
#define INPUT_GC_X     0x0400
#define INPUT_GC_Y     0x0800
#define INPUT_GC_START 0x1000

/* GameCube-pad-as-controller mapping. The default: A, B, D-pad as named,
 * X = 2, Y = 1, R / Z = +, L = -, Start = HOME. MapSet assigns every
 * INPUT_GC_* bit in pad_buttons; a button of 0 unmaps them. */
void INPUT_GameCubeMapClear(void);
void INPUT_GameCubeMapSet(uint16_t pad_buttons, uint32_t button);

/* Leave some of a Wii Remote's own buttons (WPAD_BUTTON_*) on one channel
 * (0-3) out of the button masks. 0 restores them. */
void INPUT_IgnoreWiimoteButtons(int chan, uint32_t buttons);

/* ---- GameCube controller adapter (WUP-028) ---- */
typedef struct {
    uint16_t buttons;              /* INPUT_GC_* */
    int8_t   stick_x, stick_y;     /* from the centre, y up, about -100..100 */
    int8_t   substick_x, substick_y;
    uint8_t  trigger_l, trigger_r; /* 0..255 */
    bool     wireless;             /* a WaveBird: no rumble */
} input_gc_pad;

bool INPUT_GCAdapterConnected(void);
bool INPUT_GCAdapterPad(int port, input_gc_pad *out);
/* Rumble needs the adapter's power (grey) cable plugged in. */
void INPUT_GCAdapterRumble(int port, bool on);
void INPUT_GCAdapterSetLog(int (*print)(const char *fmt, ...));

/* ---- USB game controllers with a Nintendont layout ----
 * About forty built in, and any other given as fs:/vol/external01/
 * controllers/VVVV_PPPP.ini, fs:/vol/external01/controller.ini (with the SD
 * card mounted, as WHB and the wut devoptab do) or to INPUT_HIDPadAddConfig. */
bool        INPUT_HIDPadConnected(void);
bool        INPUT_HIDPadPad(int port, input_gc_pad *out);
const char *INPUT_HIDPadName(int port);
void        INPUT_HIDPadRumble(int port, bool on);
bool        INPUT_HIDPadAddConfig(const char *text, size_t length);
void        INPUT_HIDPadSetLog(int (*print)(const char *fmt, ...));

/* ---- Mouse (USB, HID boot protocol) ---- */
bool    INPUT_MouseConnected(void);
void    INPUT_MousePosition(int *x, int *y);
void    INPUT_MouseSetPosition(int x, int y);
void    INPUT_MouseSetSpeed(int multiplier);
void    INPUT_MouseDelta(int *dx, int *dy, int *wheel);
uint8_t INPUT_MouseButtonsHeld(void);
uint8_t INPUT_MouseButtonsPressed(void);
uint8_t INPUT_MouseButtonsReleased(void);

#ifdef __cplusplus
}
#endif

#endif /* INPUT_WIIU_H */
