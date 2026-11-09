/* The console's own controllers: the GamePad (VPAD) and whatever is paired
 * through KPAD (Wii Remotes with a Nunchuk or Classic Controller, and Wii U
 * Pro Controllers), plus the GameCube adapter's ports and USB pads, folded
 * into one WPAD_BUTTON_* mask and listed one by one.
 *
 * The reading follows snes9xgx's Wii U driver (WutInputDriver.cpp, Daryl
 * Borth): VPADRead and KPADReadEx only report new packets, so a tick with
 * none (NO_SAMPLES / BUSY) carries the last state forward instead of
 * dropping the controller; the IR pointer is smoothed with a One Euro
 * filter; WPADEnableURCC brings in the Pro Controller. The button layout is
 * libwiiinput's (and GodModeU's). */
#include "libinput_internal.h"
#include <math.h>
#include <string.h>
#include <coreinit/time.h>
#include <padscore/kpad.h>
#include <padscore/wpad.h>
#include <vpad/input.h>

#define STICK_ACTIVE 0.18f
#define MAX_DEVICES 16
#define KPAD_SAMPLES 8
#define CORE_BUTTONS (WPAD_BUTTON_LEFT | WPAD_BUTTON_RIGHT | WPAD_BUTTON_DOWN | WPAD_BUTTON_UP | \
                      WPAD_BUTTON_PLUS | WPAD_BUTTON_2 | WPAD_BUTTON_1 | WPAD_BUTTON_B | WPAD_BUTTON_A | \
                      WPAD_BUTTON_MINUS | WPAD_BUTTON_HOME)

typedef struct {
    float x, y, mag;
    float raw_x, raw_y; /* before the length is clamped to 1 */
    const char *src;
} stick_t;

static uint32_t pads_held, pads_down;
static uint32_t all_held, all_down, all_up, all_prev;
static stick_t sticks[2];
static bool pointer_valid;
static float pointer_x, pointer_y, gforce_max;
static bool touch_valid;
static int touch_x, touch_y;
static uint32_t touch_button;

static bool use_vpad, use_kpad, use_pointer;
static bool initialized;

/* The device list of the last update, and what each entry stands for. */
static input_device devices[MAX_DEVICES];
static int device_count;

/* Rumble wanted per device kind and port. */
static bool rumble_gamepad, rumble_gamepad_on;
static bool rumble_wpad[4], rumble_wpad_on[4];

/* ---- One Euro filter (Casiez, Roussel & Vogel, CHI 2012), as snes9xgx
 * uses it for the IR pointer: heavy smoothing at rest, little in motion. */
typedef struct {
    bool first;
    float x, dx;
} euro_t;

#define IR_MIN_CUTOFF 0.8f
#define IR_BETA 0.015f
#define IR_D_CUTOFF 1.0f

static float euro_alpha(float cutoff, float dt)
{
    float tau = 1.0f / (2.0f * (float)M_PI * cutoff);
    return 1.0f / (1.0f + tau / dt);
}

static float euro_filter(euro_t *f, float value, float dt)
{
    float d, cutoff, a;

    if (dt <= 0.0f)
        dt = 1.0f / 60.0f;
    if (f->first) {
        f->first = false;
        f->x = value;
        f->dx = 0;
        return value;
    }
    d = (value - f->x) / dt;
    a = euro_alpha(IR_D_CUTOFF, dt);
    f->dx = a * d + (1.0f - a) * f->dx;
    cutoff = IR_MIN_CUTOFF + IR_BETA * fabsf(f->dx);
    a = euro_alpha(cutoff, dt);
    f->x = a * value + (1.0f - a) * f->x;
    return f->x;
}

/* ---- Per-source caches: the last fresh state, carried over ticks with
 * no new packet. */
typedef struct {
    bool connected;
    VPADStatus st;
    uint32_t prev_hold;
    bool touched;
    int tx, ty;
} vpad_cache_t;

typedef struct {
    bool connected;
    KPADStatus st;
    bool ir_valid;
    float ir_x, ir_y;
    euro_t fx, fy;
    int battery;
} kpad_cache_t;

static vpad_cache_t vcache;
static kpad_cache_t kcache[4];
static OSTime last_tick;

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

static void stick_feed(int which, const char *src, float x, float y)
{
    float raw_x = x, raw_y = y;
    float m = sqrtf(x * x + y * y);
    if (m > 1.0f) {
        x /= m;
        y /= m;
        m = 1.0f;
    }
    if (m > sticks[which].mag) {
        sticks[which].x = x;
        sticks[which].y = y;
        sticks[which].mag = m;
        sticks[which].raw_x = raw_x;
        sticks[which].raw_y = raw_y;
        sticks[which].src = src;
    }
}

/* ---- Layouts -------------------------------------------------------------- */

static uint32_t map_vpad(uint32_t b)
{
    uint32_t out = 0;
    if (b & (VPAD_BUTTON_R | VPAD_BUTTON_ZR | VPAD_BUTTON_PLUS))  out |= WPAD_BUTTON_PLUS;
    if (b & (VPAD_BUTTON_L | VPAD_BUTTON_ZL | VPAD_BUTTON_MINUS)) out |= WPAD_BUTTON_MINUS;
    if (b & VPAD_BUTTON_A)     out |= WPAD_BUTTON_A;
    if (b & VPAD_BUTTON_B)     out |= WPAD_BUTTON_B;
    if (b & VPAD_BUTTON_X)     out |= WPAD_BUTTON_2;
    if (b & VPAD_BUTTON_Y)     out |= WPAD_BUTTON_1;
    if (b & VPAD_BUTTON_HOME)  out |= WPAD_BUTTON_HOME;
    if (b & VPAD_BUTTON_UP)    out |= WPAD_BUTTON_UP;
    if (b & VPAD_BUTTON_DOWN)  out |= WPAD_BUTTON_DOWN;
    if (b & VPAD_BUTTON_LEFT)  out |= WPAD_BUTTON_LEFT;
    if (b & VPAD_BUTTON_RIGHT) out |= WPAD_BUTTON_RIGHT;
    return out;
}

static uint32_t map_classic(uint32_t b)
{
    uint32_t out = 0;
    if (b & (WPAD_CLASSIC_BUTTON_ZR | WPAD_CLASSIC_BUTTON_R | WPAD_CLASSIC_BUTTON_PLUS))
        out |= WPAD_BUTTON_PLUS;
    if (b & (WPAD_CLASSIC_BUTTON_ZL | WPAD_CLASSIC_BUTTON_L | WPAD_CLASSIC_BUTTON_MINUS))
        out |= WPAD_BUTTON_MINUS;
    if (b & WPAD_CLASSIC_BUTTON_A)     out |= WPAD_BUTTON_A;
    if (b & WPAD_CLASSIC_BUTTON_B)     out |= WPAD_BUTTON_B;
    if (b & WPAD_CLASSIC_BUTTON_X)     out |= WPAD_BUTTON_2;
    if (b & WPAD_CLASSIC_BUTTON_Y)     out |= WPAD_BUTTON_1;
    if (b & WPAD_CLASSIC_BUTTON_HOME)  out |= WPAD_BUTTON_HOME;
    if (b & WPAD_CLASSIC_BUTTON_UP)    out |= WPAD_BUTTON_UP;
    if (b & WPAD_CLASSIC_BUTTON_DOWN)  out |= WPAD_BUTTON_DOWN;
    if (b & WPAD_CLASSIC_BUTTON_LEFT)  out |= WPAD_BUTTON_LEFT;
    if (b & WPAD_CLASSIC_BUTTON_RIGHT) out |= WPAD_BUTTON_RIGHT;
    return out;
}

static uint32_t map_pro(uint32_t b)
{
    uint32_t out = 0;
    if (b & (WPAD_PRO_BUTTON_ZR | WPAD_PRO_BUTTON_R | WPAD_PRO_BUTTON_PLUS))  out |= WPAD_BUTTON_PLUS;
    if (b & (WPAD_PRO_BUTTON_ZL | WPAD_PRO_BUTTON_L | WPAD_PRO_BUTTON_MINUS)) out |= WPAD_BUTTON_MINUS;
    if (b & WPAD_PRO_BUTTON_A)     out |= WPAD_BUTTON_A;
    if (b & WPAD_PRO_BUTTON_B)     out |= WPAD_BUTTON_B;
    if (b & WPAD_PRO_BUTTON_X)     out |= WPAD_BUTTON_2;
    if (b & WPAD_PRO_BUTTON_Y)     out |= WPAD_BUTTON_1;
    if (b & WPAD_PRO_BUTTON_HOME)  out |= WPAD_BUTTON_HOME;
    if (b & WPAD_PRO_BUTTON_UP)    out |= WPAD_BUTTON_UP;
    if (b & WPAD_PRO_BUTTON_DOWN)  out |= WPAD_BUTTON_DOWN;
    if (b & WPAD_PRO_BUTTON_LEFT)  out |= WPAD_BUTTON_LEFT;
    if (b & WPAD_PRO_BUTTON_RIGHT) out |= WPAD_BUTTON_RIGHT;
    return out;
}

static uint32_t map_nunchuk(uint32_t b)
{
    uint32_t out = 0;
    if (b & WPAD_NUNCHUK_BUTTON_Z) out |= INPUT_BUTTON_Z;
    if (b & WPAD_NUNCHUK_BUTTON_C) out |= INPUT_BUTTON_C;
    return out;
}

/* Core Wii Remote buttons left to another layer, per channel. */
static uint32_t wiimote_ignored[4];

void INPUT_IgnoreWiimoteButtons(int chan, uint32_t buttons)
{
    if (chan >= 0 && chan < 4)
        wiimote_ignored[chan] = buttons & 0xFFFF;
}

/* GameCube pad bit -> WPAD_BUTTON_* (remappable). */
static uint32_t gc_map[16];
static bool gc_map_ready;

static void gc_map_set_bits(uint32_t pad_buttons, uint32_t button)
{
    for (int i = 0; i < 16; i++)
        if (pad_buttons & (1u << i))
            gc_map[i] = button;
}

static void gc_map_defaults(void)
{
    gc_map_ready = true;
    memset(gc_map, 0, sizeof(gc_map));
    gc_map_set_bits(INPUT_GC_R | INPUT_GC_Z, WPAD_BUTTON_PLUS);
    gc_map_set_bits(INPUT_GC_L, WPAD_BUTTON_MINUS);
    gc_map_set_bits(INPUT_GC_A, WPAD_BUTTON_A);
    gc_map_set_bits(INPUT_GC_B, WPAD_BUTTON_B);
    gc_map_set_bits(INPUT_GC_X, WPAD_BUTTON_2);
    gc_map_set_bits(INPUT_GC_Y, WPAD_BUTTON_1);
    gc_map_set_bits(INPUT_GC_START, WPAD_BUTTON_HOME);
    gc_map_set_bits(INPUT_GC_UP, WPAD_BUTTON_UP);
    gc_map_set_bits(INPUT_GC_DOWN, WPAD_BUTTON_DOWN);
    gc_map_set_bits(INPUT_GC_LEFT, WPAD_BUTTON_LEFT);
    gc_map_set_bits(INPUT_GC_RIGHT, WPAD_BUTTON_RIGHT);
}

void INPUT_GameCubeMapClear(void)
{
    gc_map_ready = true;
    memset(gc_map, 0, sizeof(gc_map));
}

void INPUT_GameCubeMapSet(uint16_t pad_buttons, uint32_t button)
{
    if (!gc_map_ready)
        gc_map_defaults();
    gc_map_set_bits(pad_buttons, button);
}

static uint32_t map_gc(uint32_t b)
{
    uint32_t out = 0;

    if (!gc_map_ready)
        gc_map_defaults();
    for (int i = 0; i < 16; i++)
        if (b & (1u << i))
            out |= gc_map[i];
    return out;
}

void INPUT_SetTouchButton(uint32_t button) { touch_button = button; }

/* ---- Folding ------------------------------------------------------------- */

static void feed(const char *src, uint32_t held, uint32_t down)
{
    pads_held |= held;
    pads_down |= down;
    if (down)
        inpw_set_source(src);
}

static input_device *dev_add(input_device_kind kind, int port, const char *name, uint32_t held, uint32_t down)
{
    input_device *d;

    if (device_count >= MAX_DEVICES)
        return NULL;
    d = &devices[device_count++];
    memset(d, 0, sizeof(*d));
    d->kind = kind;
    d->port = port;
    d->name = name;
    d->held = held;
    d->down = down;
    d->battery = -1;
    return d;
}

/* GamePad battery: 0 is charging, 1..6 empty..full (snes9xgx's reading). */
static int vpad_battery(uint8_t raw)
{
    if (raw == 0)
        return 100;
    return raw >= 4 ? 100 : raw * 25;
}

static void read_gamepad(void)
{
    VPADStatus st;
    VPADReadError err = VPAD_READ_SUCCESS;
    uint32_t trig = 0;

    memset(&st, 0, sizeof(st));
    VPADRead(VPAD_CHAN_0, &st, 1, &err);
    if (err == VPAD_READ_SUCCESS) {
        vcache.connected = true;
        vcache.st = st;
        trig = st.trigger;
        vcache.touched = st.tpNormal.touched != 0 && st.tpNormal.validity == VPAD_VALID;
        if (vcache.touched) {
            VPADTouchData cal;
            VPADGetTPCalibratedPoint(VPAD_CHAN_0, &cal, &st.tpNormal);
            /* The calibrated point is in the GamePad's 1280x720 space. */
            vcache.tx = (int)clampf(cal.x * (inpw_screen_w / 1280.0f), 0, inpw_screen_w - 1);
            vcache.ty = (int)clampf(cal.y * (inpw_screen_h / 720.0f), 0, inpw_screen_h - 1);
        }
    } else if (err == VPAD_READ_NO_SAMPLES || err == VPAD_READ_BUSY) {
        /* No new packet; the GamePad is still there: keep what it held. */
    } else {
        memset(&vcache, 0, sizeof(vcache));
    }
    if (!vcache.connected)
        return;

    uint32_t hold = vcache.st.hold;
    uint32_t held = map_vpad(hold), down = map_vpad(trig);
    static bool touched_before;
    bool touched = vcache.touched;

    if (touched) {
        touch_valid = true;
        touch_x = vcache.tx;
        touch_y = vcache.ty;
        held |= touch_button;
        if (!touched_before) {
            down |= touch_button;
            inpw_set_source("Touch");
        }
    }
    touched_before = touched;

    feed("GamePad", held, down);
    stick_feed(INPUT_STICK_LEFT, "GamePad", vcache.st.leftStick.x, vcache.st.leftStick.y);
    stick_feed(INPUT_STICK_RIGHT, "GamePad", vcache.st.rightStick.x, vcache.st.rightStick.y);

    const VPADVec3D *a = &vcache.st.accelorometer.acc;
    float g = sqrtf(a->x * a->x + a->y * a->y + a->z * a->z);
    if (g > gforce_max)
        gforce_max = g;

    input_device *d = dev_add(INPUT_DEV_GAMEPAD, 0, "GamePad", held, down);
    if (d) {
        d->raw = hold;
        d->lx = vcache.st.leftStick.x;
        d->ly = vcache.st.leftStick.y;
        d->rx = vcache.st.rightStick.x;
        d->ry = vcache.st.rightStick.y;
        d->lt = (hold & VPAD_BUTTON_ZL) ? 1.0f : 0.0f;
        d->rt = (hold & VPAD_BUTTON_ZR) ? 1.0f : 0.0f;
        d->battery = vpad_battery(vcache.st.battery);
        d->pointer = touched;
        d->px = (float)vcache.tx;
        d->py = (float)vcache.ty;
    }
}

static void read_remote(int ch, float dt)
{
    KPADStatus buf[KPAD_SAMPLES];
    KPADError err = KPAD_ERROR_OK;
    kpad_cache_t *c = &kcache[ch];
    uint32_t trig = 0, ext_trig = 0;
    int n;

    n = (int)KPADReadEx((KPADChan)ch, buf, KPAD_SAMPLES, &err);
    if (n > 0 && err == KPAD_ERROR_OK) {
        /* Newest first; a press seen in any sample since the last frame counts. */
        for (int i = 0; i < n; i++) {
            trig |= buf[i].trigger;
            if (buf[i].extensionType == WPAD_EXT_PRO_CONTROLLER)
                ext_trig |= buf[i].pro.trigger;
            else if (buf[i].extensionType == WPAD_EXT_CLASSIC || buf[i].extensionType == WPAD_EXT_MPLUS_CLASSIC)
                ext_trig |= buf[i].classic.trigger;
            else if (buf[i].extensionType == WPAD_EXT_NUNCHUK || buf[i].extensionType == WPAD_EXT_MPLUS_NUNCHUK)
                ext_trig |= buf[i].nunchuk.trigger;
        }
        if (!c->connected) {
            c->fx.first = c->fy.first = true;
            c->ir_valid = false;
        }
        c->connected = true;
        c->st = buf[0];
        c->battery = WPADGetBatteryLevel((WPADChan)ch) * 25;
        if (c->battery > 100)
            c->battery = 100;

        if (use_pointer && buf[0].extensionType != WPAD_EXT_PRO_CONTROLLER && buf[0].posValid > 0) {
            /* pos is -1..1 across the screen, y down. */
            float rx = clampf((buf[0].pos.x * 0.5f + 0.5f) * inpw_screen_w, 0, inpw_screen_w - 1);
            float ry = clampf((buf[0].pos.y * 0.5f + 0.5f) * inpw_screen_h, 0, inpw_screen_h - 1);
            if (!c->ir_valid)
                c->fx.first = c->fy.first = true; /* snap to a newly found sensor bar */
            c->ir_x = euro_filter(&c->fx, rx, dt);
            c->ir_y = euro_filter(&c->fy, ry, dt);
            c->ir_valid = true;
        } else {
            c->ir_valid = false;
        }
    } else if (err == KPAD_ERROR_NO_SAMPLES || err == KPAD_ERROR_BUSY) {
        /* No new packet: keep what it held. */
    } else {
        memset(c, 0, sizeof(*c));
    }
    if (!c->connected)
        return;

    const KPADStatus *s = &c->st;
    input_device *d;

    if (s->extensionType == WPAD_EXT_PRO_CONTROLLER) {
        uint32_t held = map_pro(s->pro.hold), down = map_pro(ext_trig);
        feed("Wii U Pro", held, down);
        stick_feed(INPUT_STICK_LEFT, "Wii U Pro", s->pro.leftStick.x, s->pro.leftStick.y);
        stick_feed(INPUT_STICK_RIGHT, "Wii U Pro", s->pro.rightStick.x, s->pro.rightStick.y);
        d = dev_add(INPUT_DEV_PRO, ch, "Wii U Pro", held, down);
        if (d) {
            d->raw = s->pro.hold;
            d->lx = s->pro.leftStick.x;
            d->ly = s->pro.leftStick.y;
            d->rx = s->pro.rightStick.x;
            d->ry = s->pro.rightStick.y;
            d->lt = (s->pro.hold & WPAD_PRO_BUTTON_ZL) ? 1.0f : 0.0f;
            d->rt = (s->pro.hold & WPAD_PRO_BUTTON_ZR) ? 1.0f : 0.0f;
            d->battery = c->battery;
        }
        return;
    }

    uint32_t core_h = s->hold & CORE_BUTTONS & ~wiimote_ignored[ch];
    uint32_t core_d = trig & CORE_BUTTONS & ~wiimote_ignored[ch];
    uint32_t xh = 0, xd = 0, ext_raw = 0;
    const char *name = "Wiimote";

    feed("Wiimote", core_h, core_d);
    d = dev_add(INPUT_DEV_WIIMOTE, ch, "Wiimote", 0, 0);

    if (s->extensionType == WPAD_EXT_CLASSIC || s->extensionType == WPAD_EXT_MPLUS_CLASSIC) {
        name = "Classic";
        ext_raw = s->classic.hold;
        xh = map_classic(s->classic.hold);
        xd = map_classic(ext_trig);
        stick_feed(INPUT_STICK_LEFT, "Classic", s->classic.leftStick.x, s->classic.leftStick.y);
        stick_feed(INPUT_STICK_RIGHT, "Classic", s->classic.rightStick.x, s->classic.rightStick.y);
        if (d) {
            d->lx = s->classic.leftStick.x;
            d->ly = s->classic.leftStick.y;
            d->rx = s->classic.rightStick.x;
            d->ry = s->classic.rightStick.y;
            d->lt = clampf(s->classic.leftTrigger, 0, 1);
            d->rt = clampf(s->classic.rightTrigger, 0, 1);
        }
    } else if (s->extensionType == WPAD_EXT_NUNCHUK || s->extensionType == WPAD_EXT_MPLUS_NUNCHUK) {
        name = "Nunchuk";
        ext_raw = s->nunchuk.hold;
        xh = map_nunchuk(s->nunchuk.hold);
        xd = map_nunchuk(ext_trig);
        stick_feed(INPUT_STICK_LEFT, "Nunchuk", s->nunchuk.stick.x, s->nunchuk.stick.y);
        if (d) {
            d->lx = s->nunchuk.stick.x;
            d->ly = s->nunchuk.stick.y;
        }
    }
    feed(name, xh, xd);

    if (c->ir_valid && !pointer_valid) {
        pointer_valid = true;
        pointer_x = c->ir_x;
        pointer_y = c->ir_y;
    }

    float g = sqrtf(s->acc.x * s->acc.x + s->acc.y * s->acc.y + s->acc.z * s->acc.z);
    if (g > gforce_max)
        gforce_max = g;

    if (d) {
        d->name = name;
        d->held = core_h | xh;
        d->down = core_d | xd;
        d->raw = s->hold & 0xFFFF;
        d->raw_ext = ext_raw;
        d->battery = c->battery;
        d->pointer = c->ir_valid;
        d->px = c->ir_x;
        d->py = c->ir_y;
    }
}

/* The adapter's ports and USB pads read like a GameCube pad, through the
 * same mapping, sticks scaled to a GameCube pad's range. */
static void feed_gc_like(input_device_kind kind, int port, const char *src, const input_gc_pad *p, uint16_t *previous)
{
    uint32_t held = map_gc(p->buttons), down = map_gc(p->buttons & ~*previous);
    input_device *d;

    *previous = p->buttons;
    feed(src, held, down);
    stick_feed(INPUT_STICK_LEFT, src, p->stick_x / 70.0f, p->stick_y / 70.0f);
    stick_feed(INPUT_STICK_RIGHT, src, p->substick_x / 60.0f, p->substick_y / 60.0f);
    d = dev_add(kind, port, src, held, down);
    if (d) {
        d->raw = p->buttons;
        d->lx = clampf(p->stick_x / 70.0f, -1, 1);
        d->ly = clampf(p->stick_y / 70.0f, -1, 1);
        d->rx = clampf(p->substick_x / 60.0f, -1, 1);
        d->ry = clampf(p->substick_y / 60.0f, -1, 1);
        d->lt = p->trigger_l / 255.0f;
        d->rt = p->trigger_r / 255.0f;
    }
}

static uint16_t gcad_previous[4], hid_previous[4];

/* ---- Rumble --------------------------------------------------------------- */

static uint8_t vpad_pattern[15] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};

static void pump_rumble(void)
{
    static int vpad_frames;

    if (use_vpad) {
        if (rumble_gamepad && vcache.connected) {
            /* A pattern plays once: renew it before it runs out. */
            if (!rumble_gamepad_on || ++vpad_frames >= 10) {
                VPADControlMotor(VPAD_CHAN_0, vpad_pattern, sizeof(vpad_pattern));
                vpad_frames = 0;
            }
            rumble_gamepad_on = true;
        } else if (rumble_gamepad_on) {
            VPADStopMotor(VPAD_CHAN_0);
            rumble_gamepad_on = false;
        }
    }
    if (use_kpad) {
        for (int ch = 0; ch < 4; ch++) {
            bool want = rumble_wpad[ch] && kcache[ch].connected;
            if (want != rumble_wpad_on[ch]) {
                WPADControlMotor((WPADChan)ch, want ? TRUE : FALSE);
                rumble_wpad_on[ch] = want;
            }
        }
    }
}

int INPUT_DeviceCount(void) { return device_count; }

bool INPUT_DeviceGet(int index, input_device *out)
{
    if (index < 0 || index >= device_count)
        return false;
    if (out)
        *out = devices[index];
    return true;
}

void INPUT_DeviceRumble(int index, bool on)
{
    const input_device *d;

    if (index < 0 || index >= device_count)
        return;
    d = &devices[index];
    switch (d->kind) {
    case INPUT_DEV_GAMEPAD:
        rumble_gamepad = on;
        break;
    case INPUT_DEV_WIIMOTE:
    case INPUT_DEV_PRO:
        rumble_wpad[d->port] = on;
        break;
    case INPUT_DEV_GCADAPTER:
        INPUT_GCAdapterRumble(d->port, on);
        break;
    case INPUT_DEV_USBPAD:
        INPUT_HIDPadRumble(d->port, on);
        break;
    default:
        break;
    }
    pump_rumble();
}

/* ---- Init / update ------------------------------------------------------- */

int32_t inpw_pads_init(uint32_t flags)
{
    use_vpad = !(flags & INPUT_INIT_NO_GAMEPAD);
    use_kpad = !(flags & INPUT_INIT_NO_REMOTES);
    use_pointer = !(flags & INPUT_INIT_NO_POINTER);

    if (use_vpad)
        VPADInit();
    if (use_kpad) {
        KPADInit();
        /* The Pro Controller and the Wii Remote both, as snes9xgx enables them. */
        WPADEnableURCC(TRUE);
        WPADEnableWiiRemote(TRUE);
        for (int ch = 0; ch < 4; ch++) {
            if (use_pointer)
                KPADEnableDPD((KPADChan)ch);
            else
                KPADDisableDPD((KPADChan)ch);
        }
    }
    memset(&vcache, 0, sizeof(vcache));
    memset(kcache, 0, sizeof(kcache));
    last_tick = OSGetSystemTime();
    initialized = true;
    return 0;
}

void inpw_pads_shutdown(void)
{
    if (!initialized)
        return;
    rumble_gamepad = false;
    memset(rumble_wpad, 0, sizeof(rumble_wpad));
    for (int p = 0; p < 4; p++) {
        INPUT_GCAdapterRumble(p, false);
        INPUT_HIDPadRumble(p, false);
    }
    if (use_vpad)
        VPADStopMotor(VPAD_CHAN_0);
    if (use_kpad)
        for (int ch = 0; ch < 4; ch++)
            WPADControlMotor((WPADChan)ch, FALSE);
    rumble_gamepad_on = false;
    memset(rumble_wpad_on, 0, sizeof(rumble_wpad_on));
    /* KPAD stays up: the system's HOME menu and the next app use it. */
    initialized = false;
}

void inpw_pads_update(void)
{
    OSTime now = OSGetSystemTime();
    float dt = (float)OSTicksToMicroseconds(now - last_tick) / 1000000.0f;
    last_tick = now;

    pads_held = pads_down = 0;
    sticks[0] = sticks[1] = (stick_t){ 0, 0, 0, 0, 0, "" };
    pointer_valid = false;
    touch_valid = false;
    gforce_max = 0;
    device_count = 0;

    if (use_vpad)
        read_gamepad();
    if (use_kpad)
        for (int ch = 0; ch < 4; ch++)
            read_remote(ch, dt);

    inpw_gcadapter_update();
    for (int port = 0; port < 4; port++) {
        input_gc_pad p;
        if (!INPUT_GCAdapterPad(port, &p)) {
            gcad_previous[port] = 0;
            continue;
        }
        feed_gc_like(INPUT_DEV_GCADAPTER, port, "GameCube", &p, &gcad_previous[port]);
    }
    for (int port = 0; port < 4; port++) {
        input_gc_pad p;
        if (!INPUT_HIDPadPad(port, &p)) {
            hid_previous[port] = 0;
            continue;
        }
        feed_gc_like(INPUT_DEV_USBPAD, port, "USB Pad", &p, &hid_previous[port]);
    }

    for (int i = 0; i < 2; i++) {
        if (sticks[i].mag > STICK_ACTIVE)
            inpw_set_source(sticks[i].src);
    }

    pump_rumble();

    uint32_t kheld = INPUT_KeyboardPadHeld();
    uint32_t kdown = INPUT_KeyboardPadDown();
    all_held = pads_held | kheld;
    all_down = pads_down | kdown;
    all_up = all_prev & ~all_held;
    all_prev = all_held;
}

uint32_t INPUT_PadsHeld(void)    { return pads_held; }
uint32_t INPUT_PadsDown(void)    { return pads_down; }
uint32_t INPUT_ButtonsHeld(void) { return all_held; }
uint32_t INPUT_ButtonsDown(void) { return all_down; }
uint32_t INPUT_ButtonsUp(void)   { return all_up; }

float INPUT_Stick(int which, float *x, float *y, const char **src)
{
    if (which < 0 || which > 1)
        which = 0;
    if (x) *x = sticks[which].x;
    if (y) *y = sticks[which].y;
    if (src) *src = sticks[which].src ? sticks[which].src : "";
    return sticks[which].mag;
}

float INPUT_StickRaw(int which, float *x, float *y, const char **src)
{
    if (which < 0 || which > 1)
        which = 0;
    if (x) *x = sticks[which].raw_x;
    if (y) *y = sticks[which].raw_y;
    if (src) *src = sticks[which].src ? sticks[which].src : "";
    return sticks[which].mag;
}

bool INPUT_Pointer(float *x, float *y)
{
    if (x) *x = pointer_x;
    if (y) *y = pointer_y;
    return pointer_valid;
}

bool INPUT_Touch(int *x, int *y)
{
    if (x) *x = touch_x;
    if (y) *y = touch_y;
    return touch_valid;
}

float INPUT_Gforce(void) { return gforce_max; }
