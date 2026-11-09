/* SPDX-License-Identifier: GPL-3.0-or-later */
/* USB HID devices through the system's HID library (nsyshid): mice, the
 * GameCube controller adapter for Wii U and USB game controllers with a
 * Nintendont controller.ini layout. One HID client for all three.
 *
 * The system offers every HID interface that is plugged in, now or later,
 * to each client in turn; a client answers 1 to take it and 0 to leave it
 * to the others (the keyboard library among them, so keyboards are never
 * taken). Each device then keeps one read outstanding, re-armed from its
 * own completion. The callbacks run on the system's HID thread; what they
 * share with the main thread sits under an uninterruptible spinlock, and
 * no HID call is made while it is held.
 *
 * The mouse follows snes9xgx's Wii U driver (WutMouseDriver.cpp, Daryl
 * Borth): boot protocol, so the report needs no descriptor. The adapter's
 * report and commands are libwiiinput's (from RiftWii, after Dolphin's
 * GCAdapter.cpp and wup-028-bslug): 0x13 starts it, 0x11 rumbles, each
 * report is 0x21 and nine bytes per port. The pads' layouts and report
 * conversion are libwiiinput's libinput_hid.c (from RiftWii, after
 * Nintendont), unchanged. */

#include <dirent.h>
#include <malloc.h>
#include <stdio.h>
#include <string.h>

#include <coreinit/spinlock.h>
#include <nsyshid/hid.h>

#include "libinput_hid.h"
#include "libinput_hidparse.h"
#include "libinput_internal.h"

#define MAX_SLOTS 8
#define MAX_PADS 4
#define MAX_USER 8
#define BUF_BYTES 128           /* HIDPAD_REPORT_MAX */
#define FAILS_MAX 20            /* reads failing this many times in a row end the device */
#define CLAIMED 1
#define UNCLAIMED 0

#define GCAD_VID 0x057E
#define GCAD_PID 0x0337
#define GCAD_REPORT 37
#define GCAD_TIMEOUT_MS 1500    /* no report for this long: every port reads as unplugged */
#define GCAD_RECAL_MS 3000      /* X+Y+Start held this long takes a new centre */
#define GCAD_ORIGIN_WAIT_MS 500 /* no reading at rest this long: the middle is the centre */

#define HID_REPORT_INPUT 1
#define HID_REPORT_OUTPUT 2
#define HID_REPORT_FEATURE 3

enum { K_NONE, K_MOUSE, K_GCAD, K_PAD, K_PROBE };

#define HID_DT_REPORT 0x22
#define RDESC_BYTES 1024

#ifdef HAVE_HIDPADS
/* A pad with no controller.ini layout, read from its own report
 * descriptor: the fields of a joystick or gamepad collection. */
enum { G_X, G_Y, G_RX, G_RY, G_LT, G_RT, G_AXES };
#define G_BUTTONS 16
typedef struct {
    uint8_t report_id;
    bool uses_ids;
    uint16_t report_bytes;      /* with the ID byte */
    hidp_field axis[G_AXES];
    bool has_axis[G_AXES];
    hidp_field hat;
    bool has_hat;
    hidp_field button[G_BUTTONS];
    uint16_t has_button;        /* bit n: button n + 1 */
} generic_map;
#endif

typedef struct {
    int kind;
    uint32_t gen;               /* bumped whenever the slot is (re)used */
    uint32_t handle;
    uint8_t iface;
    uint16_t vid, pid;
    uint32_t phys;              /* physicalDeviceInst: several interfaces, one device */
    uint32_t rx_size;
    uint8_t *rx, *tx;           /* kept until shutdown: a cancelled transfer may still land in them */
    volatile bool tx_busy, rx_busy;
    int fails;

    /* mouse */
    uint8_t mbuttons;

    /* GameCube adapter */
    uint8_t raw[4][9];
    uint32_t report_ms;
    bool reporting;

#ifdef HAVE_HIDPADS
    /* USB pad */
    int port;
    const char *where;
    hidpad_mapping map;
    bool generic;
    generic_map gmap;
    uint16_t gen_raw;           /* a generic pad's buttons as numbered, bit n = button n + 1 */
    hidpad_state state;
    bool state_valid;
    bool can_rumble;
    uint8_t rumble_sent, rumble_flight, rumble_index;
    bool rumble_active;
#endif
    uint8_t *desc;              /* report descriptor, while probing */
    uint16_t max_rx;
    uint8_t subclass, protocol;
} slot_t;

/* Every HID interface offered, and what was done with it. */
#define MAX_SEEN 16
static input_usb_device seen[MAX_SEEN];
static uint32_t seen_handle[MAX_SEEN];
static int seen_count;

static OSSpinLock lock;
static slot_t slots[MAX_SLOTS];
static HIDClient client;
static bool client_added, hid_setup;
static bool want_mouse, want_gcad, want_pads;

/* Mouse motion since the last frame, under `lock`. */
static int m_dx, m_dy, m_wheel;
static uint8_t m_quick;

/* Wanted rumble: the main thread writes, inpw_hid_update sends. */
static bool gcad_rumble_want[4];
#ifdef HAVE_HIDPADS
static bool pad_rumble_want[MAX_PADS];
static hidpad_mapping user_maps[MAX_USER];
static int user_count;
/* Layouts from the SD card, read once at init: the system's HID thread,
 * which offers devices, is not one to read files on. */
#define MAX_FILES 16
static hidpad_mapping file_maps[MAX_FILES];
static int file_count;
#endif

static int (*gcad_log)(const char *fmt, ...);
static int (*pad_log)(const char *fmt, ...);
#define GLOG(...) do { if (gcad_log) gcad_log(__VA_ARGS__); } while (0)
#define PLOG(...) do { if (pad_log) pad_log(__VA_ARGS__); } while (0)

void INPUT_GCAdapterSetLog(int (*print)(const char *fmt, ...)) { gcad_log = print; }
void INPUT_HIDPadSetLog(int (*print)(const char *fmt, ...))    { pad_log = print; }

static void lock_take(void) { OSUninterruptibleSpinLock_Acquire(&lock); }
static void lock_give(void) { OSUninterruptibleSpinLock_Release(&lock); }

static void *ctx_of(int idx, uint32_t gen) { return (void *)(uintptr_t)((gen << 4) | (uint32_t)idx); }

/* The slot a callback's context stands for, if it is still that device. */
static slot_t *slot_of(void *ctx, uint32_t handle)
{
    uintptr_t v = (uintptr_t)ctx;
    int idx = (int)(v & 0xF);
    slot_t *s;

    if (idx >= MAX_SLOTS)
        return NULL;
    s = &slots[idx];
    if (s->kind == K_NONE || s->gen != (uint32_t)(v >> 4) || s->handle != handle)
        return NULL;
    return s;
}

static void arm_read(int idx);

/* Notes what became of an interface; the caller holds `lock`. */
static void seen_set(HIDDevice *d, uint32_t handle, const char *what)
{
    int i;

    for (i = 0; i < seen_count; i++)
        if (seen_handle[i] == handle)
            break;
    if (i == seen_count) {
        if (!d)
            return;
        if (seen_count == MAX_SEEN) {
            memmove(&seen[0], &seen[1], sizeof(seen[0]) * (MAX_SEEN - 1));
            memmove(&seen_handle[0], &seen_handle[1], sizeof(seen_handle[0]) * (MAX_SEEN - 1));
            i = MAX_SEEN - 1;
        } else {
            seen_count++;
        }
        memset(&seen[i], 0, sizeof(seen[i]));
        seen_handle[i] = handle;
    }
    if (d) {
        seen[i].vid = d->vid;
        seen[i].pid = d->pid;
        seen[i].interface = d->interfaceIndex;
        seen[i].subclass = d->subClass;
        seen[i].protocol = d->protocol;
        seen[i].max_rx = d->maxPacketSizeRx;
    }
    snprintf(seen[i].what, sizeof(seen[i].what), "%s", what);
}

static void seen_note(HIDDevice *d, const char *what)
{
    lock_take();
    seen_set(d, d->handle, what);
    lock_give();
}

static void seen_count_report(uint32_t handle, uint32_t bytes, const uint8_t *data, bool used)
{
    for (int i = 0; i < seen_count; i++) {
        if (seen_handle[i] != handle)
            continue;
        seen[i].reports++;
        if (used)
            seen[i].decoded++;
        seen[i].last_len = (uint8_t)(bytes > 255 ? 255 : bytes);
        memcpy(seen[i].last, data, bytes < sizeof(seen[i].last) ? bytes : sizeof(seen[i].last));
        return;
    }
}

int INPUT_USBDeviceCount(void)
{
    return seen_count;
}

bool INPUT_USBDeviceGet(int index, input_usb_device *out)
{
    bool ok = false;

    if (!client_added || index < 0 || !out)
        return false;
    lock_take();
    if (index < seen_count) {
        *out = seen[index];
        ok = true;
    }
    lock_give();
    return ok;
}

/* ---- Layouts for USB pads ------------------------------------------------ */

#ifdef HAVE_HIDPADS
bool INPUT_HIDPadAddConfig(const char *text, size_t length)
{
    hidpad_mapping m;
    char error[96];

    if (!text || user_count >= MAX_USER)
        return false;
    if (!hidpad_parse(text, length, &m, error, sizeof(error))) {
        PLOG("USB pad: layout refused: %s\n", error);
        return false;
    }
    lock_take();
    user_maps[user_count++] = m;
    lock_give();
    return true;
}

static void load_file(const char *path)
{
    static char text[HIDPAD_CONFIG_BYTES + 1];
    char error[96];
    hidpad_mapping m;
    FILE *file;
    size_t size;
    bool bad;

    if (file_count >= MAX_FILES || !(file = fopen(path, "rb")))
        return;
    size = fread(text, 1, sizeof(text), file);
    bad = ferror(file) != 0;
    fclose(file);
    if (bad || !hidpad_parse(text, size, &m, error, sizeof(error))) {
        PLOG("USB pad: cannot use %s: %s\n", path, bad ? "the file could not be read" : error);
        return;
    }
    file_maps[file_count++] = m;
    PLOG("USB pad: layout %04x:%04x from %s\n", m.vid, m.pid, path);
}

/* Every .ini in sd:/controllers, and sd:/controller.ini, where Nintendont keeps them. */
static void load_files(void)
{
    static const char *const roots[] = { "fs:/vol/external01", "/vol/external01" };
    char path[160];

    file_count = 0;
    for (unsigned r = 0; r < sizeof(roots) / sizeof(roots[0]) && !file_count; r++) {
        DIR *dir;
        snprintf(path, sizeof(path), "%s/controllers", roots[r]);
        if ((dir = opendir(path))) {
            struct dirent *e;
            while ((e = readdir(dir)) && file_count < MAX_FILES) {
                size_t n = strlen(e->d_name);
                if (n > 4 && (!strcmp(e->d_name + n - 4, ".ini") || !strcmp(e->d_name + n - 4, ".INI"))) {
                    snprintf(path, sizeof(path), "%s/controllers/%s", roots[r], e->d_name);
                    load_file(path);
                }
            }
            closedir(dir);
        }
        snprintf(path, sizeof(path), "%s/controller.ini", roots[r]);
        load_file(path);
    }
}

/* A layout the app gave first, then the SD card's, then the library's own.
 * Reads nothing: called on the HID thread. */
static bool find_layout(uint32_t id, hidpad_mapping *out, const char **name)
{
    const hidpad_builtin *builtin;
    bool found = false;

    lock_take();
    for (int i = 0; i < user_count && !found; i++) {
        if ((((uint32_t)user_maps[i].vid << 16) | user_maps[i].pid) == id) {
            *out = user_maps[i];
            *name = "the app's layout";
            found = true;
        }
    }
    lock_give();
    for (int i = 0; i < file_count && !found; i++) {
        if ((((uint32_t)file_maps[i].vid << 16) | file_maps[i].pid) == id) {
            *out = file_maps[i];
            *name = "a file on the SD card";
            found = true;
        }
    }
    if (found)
        return true;
    builtin = hidpad_find_builtin(id);
    if (builtin) {
        char error[96];

        if (!hidpad_parse(builtin->text, strlen(builtin->text), out, error, sizeof(error))) {
            PLOG("USB pad %04x:%04x: the built-in layout (%s) cannot be used: %s\n", (unsigned)(id >> 16),
                 (unsigned)(id & 0xFFFF), builtin->file, error);
            return false;
        }
        *name = builtin->name;
        return true;
    }
    return false;
}
#else
bool INPUT_HIDPadAddConfig(const char *text, size_t length) { (void)text; (void)length; return false; }
#endif

/* ---- Generic pads ------------------------------------------------------- */

#ifdef HAVE_HIDPADS
#define GD(u) (0x00010000u | (u))

static hidp_desc gdesc; /* only the HID thread parses */

static bool is_pad_app(uint32_t app)
{
    return app == GD(0x04) || app == GD(0x05) || app == GD(0x08); /* joystick, gamepad, multi-axis */
}

static const hidp_field *gfind(const hidp_desc *d, uint32_t usage, int report_id)
{
    for (int i = 0; i < d->nfields; i++) {
        const hidp_field *f = &d->fields[i];
        if (f->type == HIDP_INPUT && f->usage == usage && is_pad_app(f->app) &&
            (report_id < 0 || f->report_id == report_id) && f->logical_max > f->logical_min)
            return f;
    }
    return NULL;
}

/* The pad's fields from its descriptor; false when it describes no pad. */
static bool generic_build(const hidp_desc *d, generic_map *g, char *summary, size_t n)
{
    const hidp_field *f;
    int id = -1, nb = 0;
    uint32_t right_x = GD(0x32), right_y = GD(0x35), lt = 0, rt = 0;

    memset(g, 0, sizeof(*g));
    /* the report carrying X, or else the first button */
    f = gfind(d, GD(0x30), -1);
    for (int i = 0; !f && i < d->nfields; i++)
        if (d->fields[i].type == HIDP_INPUT && is_pad_app(d->fields[i].app) && (d->fields[i].usage >> 16) == HIDP_PAGE_BUTTON)
            f = &d->fields[i];
    if (!f)
        return false;
    id = f->report_id;
    g->report_id = (uint8_t)id;
    g->uses_ids = d->uses_ids;
    g->report_bytes = (uint16_t)hidp_report_bytes(d, HIDP_INPUT, (uint8_t)id);

    /* Rx and Ry are the right stick where there are both (then Z and Rz
     * are triggers); otherwise Z and Rz are, as on most DirectInput pads. */
    if (gfind(d, GD(0x33), id) && gfind(d, GD(0x34), id)) {
        right_x = GD(0x33);
        right_y = GD(0x34);
        lt = GD(0x32);
        rt = GD(0x35);
    }
    const uint32_t want[G_AXES] = { GD(0x30), GD(0x31), right_x, right_y, lt, rt };
    for (int a = 0; a < G_AXES; a++) {
        if (want[a] && (f = gfind(d, want[a], id))) {
            g->axis[a] = *f;
            g->has_axis[a] = true;
        }
    }
    if ((f = gfind(d, GD(0x39), id))) {
        g->hat = *f;
        g->has_hat = true;
    }
    for (int b = 0; b < G_BUTTONS; b++) {
        if ((f = gfind(d, ((uint32_t)HIDP_PAGE_BUTTON << 16) | (uint32_t)(b + 1), id))) {
            g->button[b] = *f;
            g->has_button |= (uint16_t)(1u << b);
            nb++;
        }
    }
    if (!g->has_axis[G_X] && !g->has_hat && nb < 4)
        return false;
    snprintf(summary, n, "report %u, %u bytes, %s%s%s%d buttons", (unsigned)id, (unsigned)g->report_bytes,
             g->has_axis[G_X] ? "stick " : "", g->has_axis[G_RX] ? "right stick " : "", g->has_hat ? "hat " : "", nb);
    return true;
}

static int8_t g_axis(const hidp_field *f, const uint8_t *data, uint32_t len, bool flip)
{
    float lo = (float)f->logical_min, hi = (float)f->logical_max;
    float v = ((float)hidp_get(f, data, len) - (lo + hi) * 0.5f) / ((hi - lo) * 0.5f) * 127.0f;
    if (flip)
        v = -v;
    if (v > 127.0f) v = 127.0f;
    if (v < -127.0f) v = -127.0f;
    return (int8_t)v;
}

static uint8_t g_trigger(const hidp_field *f, const uint8_t *data, uint32_t len)
{
    int32_t v = hidp_get(f, data, len) - f->logical_min;
    int32_t r = f->logical_max - f->logical_min;
    v = v * 255 / r;
    return (uint8_t)(v < 0 ? 0 : v > 255 ? 255 : v);
}

/* Buttons by number, as most PC pads and adapters for Nintendo pads number
 * them; the raw numbers are kept too (INPUT_HIDPadRaw). */
static const uint16_t g_button_map[G_BUTTONS] = {
    HIDPAD_PAD_Y, HIDPAD_PAD_B, HIDPAD_PAD_A, HIDPAD_PAD_X,   /* 1-4: the face, left, bottom, right, top */
    HIDPAD_PAD_L, HIDPAD_PAD_R, HIDPAD_PAD_L, HIDPAD_PAD_R,   /* 5-8: shoulders, then ZL / ZR */
    HIDPAD_PAD_Z, HIDPAD_PAD_START, 0, 0,                     /* 9: select / minus, 10: start / plus */
    HIDPAD_PAD_START, 0, 0, 0                                 /* 13: home */
};

static bool generic_decode(const generic_map *g, const uint8_t *rep, uint32_t bytes, hidpad_state *out, uint16_t *raw)
{
    const uint8_t *data = rep;
    uint32_t len = bytes;
    uint16_t buttons = 0, nums = 0;

    if (g->uses_ids) {
        if (!bytes || rep[0] != g->report_id)
            return false;
        data++;
        len--;
    }
    if (!len)
        return false;
    memset(out, 0, sizeof(*out));
    for (int b = 0; b < G_BUTTONS; b++) {
        if ((g->has_button & (1u << b)) && hidp_get(&g->button[b], data, len) > g->button[b].logical_min) {
            nums |= (uint16_t)(1u << b);
            buttons |= g_button_map[b];
        }
    }
    if (g->has_hat) {
        int32_t v = hidp_get(&g->hat, data, len) - g->hat.logical_min;
        static const uint16_t dirs[8] = {
            HIDPAD_PAD_UP, HIDPAD_PAD_UP | HIDPAD_PAD_RIGHT, HIDPAD_PAD_RIGHT, HIDPAD_PAD_DOWN | HIDPAD_PAD_RIGHT,
            HIDPAD_PAD_DOWN, HIDPAD_PAD_DOWN | HIDPAD_PAD_LEFT, HIDPAD_PAD_LEFT, HIDPAD_PAD_UP | HIDPAD_PAD_LEFT
        };
        /* four-way hats count 0-3 */
        if (g->hat.logical_max - g->hat.logical_min == 3 && v >= 0 && v < 4)
            v *= 2;
        if (v >= 0 && v < 8)
            buttons |= dirs[v];
    }
    if (g->has_axis[G_X]) out->stick_x = g_axis(&g->axis[G_X], data, len, false);
    if (g->has_axis[G_Y]) out->stick_y = g_axis(&g->axis[G_Y], data, len, true);
    if (g->has_axis[G_RX]) out->substick_x = g_axis(&g->axis[G_RX], data, len, false);
    if (g->has_axis[G_RY]) out->substick_y = g_axis(&g->axis[G_RY], data, len, true);
    if (g->has_axis[G_LT]) out->trigger_l = g_trigger(&g->axis[G_LT], data, len);
    if (g->has_axis[G_RT]) out->trigger_r = g_trigger(&g->axis[G_RT], data, len);
    if (nums & (1u << 6)) out->trigger_l = 255;
    if (nums & (1u << 7)) out->trigger_r = 255;
    out->buttons = buttons;
    out->no_rumble = 1;
    *raw = nums;
    return true;
}
#endif

/* ---- Callbacks ------------------------------------------------------------ */

static void on_read(uint32_t handle, int32_t error, uint8_t *buffer, uint32_t bytes, void *ctx)
{
    slot_t *s;
    int idx = (int)((uintptr_t)ctx & 0xF);
    bool again = false;

    (void)buffer;
    lock_take();
    s = slot_of(ctx, handle);
    if (s) {
        s->rx_busy = false;
        if (error < 0) {
            again = ++s->fails < FAILS_MAX;
        } else {
            bool used = false;
            s->fails = 0;
            again = true;
            switch (s->kind) {
            case K_MOUSE:
                if (bytes >= 3) {
                    uint8_t b = s->rx[0] & 7;
                    used = true;
                    m_quick |= b & ~s->mbuttons;
                    s->mbuttons = b;
                    m_dx += (int8_t)s->rx[1];
                    m_dy += (int8_t)s->rx[2];
                    /* Most mice still send the wheel after the boot fields. */
                    if (bytes >= 4)
                        m_wheel += (int8_t)s->rx[3];
                }
                break;
            case K_GCAD:
                if (bytes >= GCAD_REPORT && s->rx[0] == 0x21) {
                    memcpy(s->raw, s->rx + 1, sizeof(s->raw));
                    s->report_ms = inpw_now_ms();
                    s->reporting = true;
                    used = true;
                }
                break;
#ifdef HAVE_HIDPADS
            case K_PAD: {
                hidpad_state st;
                bool ok;
                /* A report of another kind, or too short, is skipped. */
                if (s->generic)
                    ok = generic_decode(&s->gmap, s->rx, bytes ? bytes : s->rx_size, &st, &s->gen_raw);
                else
                    ok = hidpad_decode(&s->map, s->rx, bytes ? bytes : s->rx_size, &st);
                used = ok;
                if (ok) {
                    if (!s->can_rumble)
                        st.no_rumble = 1;
                    s->state = st;
                    s->state_valid = true;
                }
                /* GET_REPORT pads are polled from the main thread, once a frame. */
                if (!s->generic && !s->map.polltype)
                    again = false;
                break;
            }
#endif
            }
            seen_count_report(handle, bytes ? bytes : s->rx_size, s->rx, used);
        }
        if (!again && error < 0 && s->kind == K_MOUSE)
            s->mbuttons = 0;
    }
    lock_give();
    if (s && again)
        arm_read(idx);
}

static void on_write(uint32_t handle, int32_t error, uint8_t *buffer, uint32_t bytes, void *ctx)
{
    slot_t *s;

    (void)buffer;
    (void)bytes;
    lock_take();
    s = slot_of(ctx, handle);
    if (s) {
        s->tx_busy = false;
        if (s->kind == K_GCAD && error < 0)
            GLOG("GC adapter: a command failed (%d)\n", (int)error);
    }
    lock_give();
}

/* The adapter's start command went out: reports follow. */
static void on_gcad_started(uint32_t handle, int32_t error, uint8_t *buffer, uint32_t bytes, void *ctx)
{
    int idx = (int)((uintptr_t)ctx & 0xF);
    slot_t *s;

    on_write(handle, error, buffer, bytes, ctx);
    lock_take();
    s = slot_of(ctx, handle);
    lock_give();
    if (!s)
        return;
    GLOG("GC adapter: %s\n", error < 0 ? "the start command failed, reading anyway" : "started");
    arm_read(idx);
}

/* SET_PROTOCOL answered (mice, and pads with a boot interface). */
static void on_protocol(uint32_t handle, int32_t error, uint8_t *buffer, uint32_t bytes, void *ctx)
{
    int idx = (int)((uintptr_t)ctx & 0xF);
    slot_t *s;
    bool go;

    (void)buffer;
    (void)bytes;
    lock_take();
    s = slot_of(ctx, handle);
    /* Without the boot protocol a mouse's report layout is unknown. */
    go = s && (error >= 0 || s->kind != K_MOUSE);
    if (s && !go)
        s->kind = K_NONE;
    lock_give();
    if (go)
        arm_read(idx);
}

static void arm_read(int idx)
{
    slot_t *s = &slots[idx];
    uint32_t handle, size;
    void *ctx;
    int32_t ret;

    lock_take();
    if (s->kind == K_NONE || s->rx_busy) {
        lock_give();
        return;
    }
    s->rx_busy = true;
    handle = s->handle;
    size = s->rx_size;
    ctx = ctx_of(idx, s->gen);
    lock_give();

    ret = HIDRead(handle, s->rx, size, on_read, ctx);
    if (ret < 0) {
        lock_take();
        if (slot_of(ctx, handle))
            s->rx_busy = false;
        lock_give();
    }
}

static int free_slot(void)
{
    for (int i = 0; i < MAX_SLOTS; i++)
        if (slots[i].kind == K_NONE)
            return i;
    return -1;
}

static bool have_kind(int kind)
{
    for (int i = 0; i < MAX_SLOTS; i++)
        if (slots[i].kind == kind)
            return true;
    return false;
}

/* Takes slot `idx` for a device; the caller holds `lock`. */
static slot_t *take_slot(int idx, int kind, HIDDevice *d)
{
    slot_t *s = &slots[idx];
    uint8_t *rx = s->rx, *tx = s->tx, *desc = s->desc;
    uint32_t gen = s->gen + 1;

    memset(s, 0, sizeof(*s));
    s->rx = rx;
    s->tx = tx;
    s->desc = desc;
    s->max_rx = d->maxPacketSizeRx;
    s->subclass = d->subClass;
    s->protocol = d->protocol;
    s->gen = gen & 0x0FFFFFFF;
    s->kind = kind;
    s->handle = d->handle;
    s->iface = d->interfaceIndex;
    s->vid = d->vid;
    s->pid = d->pid;
    s->phys = d->physicalDeviceInst;
    s->rx_size = d->maxPacketSizeRx;
    if (s->rx_size < 4)
        s->rx_size = 4;
    if (s->rx_size > BUF_BYTES)
        s->rx_size = BUF_BYTES;
    return s;
}

static int32_t attach_mouse(HIDDevice *d, int idx)
{
    void *ctx;

    lock_take();
    take_slot(idx, K_MOUSE, d);
    ctx = ctx_of(idx, slots[idx].gen);
    lock_give();
    /* 0: boot protocol, a fixed report of buttons, x, y */
    if (HIDSetProtocol(d->handle, d->interfaceIndex, 0, on_protocol, ctx) < 0) {
        lock_take();
        slots[idx].kind = K_NONE;
        lock_give();
        return UNCLAIMED;
    }
    return CLAIMED;
}

static int32_t attach_gcad(HIDDevice *d, int idx)
{
    slot_t *s;
    void *ctx;

    lock_take();
    s = take_slot(idx, K_GCAD, d);
    s->rx_size = GCAD_REPORT;
    s->tx_busy = true;
    ctx = ctx_of(idx, s->gen);
    lock_give();
    GLOG("GC adapter: found (interface %u, %u byte reports)\n", d->interfaceIndex, d->maxPacketSizeRx);
    s->tx[0] = 0x13;
    if (HIDWrite(d->handle, s->tx, 1, on_gcad_started, ctx) < 0) {
        GLOG("GC adapter: the start command could not be sent\n");
        lock_take();
        s->tx_busy = false;
        lock_give();
        arm_read(idx);
    }
    return CLAIMED;
}

#ifdef HAVE_HIDPADS
static int free_port(void)
{
    bool used[MAX_PADS] = { false };

    for (int i = 0; i < MAX_SLOTS; i++)
        if (slots[i].kind == K_PAD)
            used[slots[i].port] = true;
    for (int i = 0; i < MAX_PADS; i++)
        if (!used[i])
            return i;
    return -1;
}

/* The report descriptor of a device with no layout has come. */
static void on_gen_desc(uint32_t handle, int32_t error, uint8_t *buffer, uint32_t bytes, void *ctx)
{
    static generic_map g;  /* only the HID thread builds */
    int idx = (int)((uintptr_t)ctx & 0xF);
    char summary[96] = "";
    bool pad = false, go = false;
    slot_t *s;
    int port = -1;

    (void)buffer;
    lock_take();
    s = slot_of(ctx, handle);
    lock_give();
    if (!s || s->kind != K_PROBE)
        return;
    if (error >= 0 && bytes > 0 && hidp_parse(s->desc, bytes > RDESC_BYTES ? RDESC_BYTES : bytes, &gdesc))
        pad = generic_build(&gdesc, &g, summary, sizeof(summary));

    lock_take();
    if (slot_of(ctx, handle) == s && s->kind == K_PROBE) {
        port = pad ? free_port() : -1;
        if (port >= 0) {
            s->kind = K_PAD;
            s->generic = true;
            s->gmap = g;
            s->port = port;
            s->where = "generic USB pad";
            s->can_rumble = false;
            s->rumble_sent = 0;
            pad_rumble_want[port] = false;
            if (g.report_bytes > s->rx_size)
                s->rx_size = g.report_bytes <= BUF_BYTES ? g.report_bytes : BUF_BYTES;
            go = true;
            seen_set(NULL, handle, "generic pad");
        } else {
            s->kind = K_NONE;
            seen_set(NULL, handle, error < 0 ? "no report descriptor" : pad ? "pad, but four already" : "not a pad");
        }
    }
    lock_give();
    PLOG("USB %04x:%04x if %u: %s%s%s\n", s->vid, s->pid, s->iface,
         error < 0 ? "no report descriptor" : !pad ? "descriptor describes no pad" : go ? "generic pad: " : "four pads already",
         go ? summary : "", "");
    if (go) {
        PLOG("USB pad %04x:%04x: port %d (from its descriptor)\n", s->vid, s->pid, port + 1);
        arm_read(idx);
    }
}

/* No layout: ask the device for its report descriptor (answered in on_gen_desc). */
static int32_t probe_generic(HIDDevice *d, int idx)
{
    slot_t *s;
    void *ctx;

    if (!slots[idx].desc)
        return UNCLAIMED;
    lock_take();
    s = take_slot(idx, K_PROBE, d);
    ctx = ctx_of(idx, s->gen);
    seen_set(d, d->handle, "asking for its descriptor");
    lock_give();
    memset(s->desc, 0, RDESC_BYTES);
    if (HIDGetDescriptor(d->handle, HID_DT_REPORT, 0, d->interfaceIndex, s->desc, RDESC_BYTES, on_gen_desc, ctx) < 0) {
        lock_take();
        s->kind = K_NONE;
        seen_set(d, d->handle, "descriptor request failed");
        lock_give();
        PLOG("USB %04x:%04x if %u: the descriptor could not be asked for\n", d->vid, d->pid, d->interfaceIndex);
        return UNCLAIMED;
    }
    return CLAIMED;
}

static int32_t attach_pad(HIDDevice *d, int idx)
{
    static hidpad_mapping map;  /* only the HID thread attaches */
    const char *where = NULL;
    uint32_t id = ((uint32_t)d->vid << 16) | d->pid;
    bool used[MAX_PADS] = { false };
    int port = -1;
    slot_t *s;
    void *ctx;

    if (!find_layout(id, &map, &where)) {
        if (d->subClass == 1 && d->protocol == 2) /* a boot mouse: the mouse driver's */
            return UNCLAIMED;
        PLOG("USB %04x:%04x if %u (%u/%u, %u bytes): no layout, reading its descriptor\n", d->vid, d->pid,
             d->interfaceIndex, d->subClass, d->protocol, d->maxPacketSizeRx);
        return probe_generic(d, idx);
    }
    if (map.polltype && d->maxPacketSizeRx < map.report_bytes) {
        PLOG("USB pad %04x:%04x: interface %u sends %u byte reports, the layout needs %u: reading its descriptor\n",
             d->vid, d->pid, d->interfaceIndex, d->maxPacketSizeRx, map.report_bytes);
        return probe_generic(d, idx);
    }

    lock_take();
    for (int i = 0; i < MAX_SLOTS; i++) {
        if (slots[i].kind != K_PAD)
            continue;
        used[slots[i].port] = true;
        /* One interface per pad: the first one offered that reports. */
        if (slots[i].phys == d->physicalDeviceInst && slots[i].vid == d->vid && slots[i].pid == d->pid) {
            lock_give();
            PLOG("USB %04x:%04x if %u: another interface of a pad already read\n", d->vid, d->pid,
                 d->interfaceIndex);
            seen_note(d, "pad's other interface");
            return UNCLAIMED;
        }
    }
    for (int i = 0; i < MAX_PADS && port < 0; i++)
        if (!used[i])
            port = i;
    if (port < 0) {
        lock_give();
        PLOG("USB %04x:%04x: four pads already\n", d->vid, d->pid);
        return UNCLAIMED;
    }
    s = take_slot(idx, K_PAD, d);
    s->port = port;
    s->where = where;
    seen_set(d, d->handle, where);
    s->map = map;
    if (!map.polltype && s->rx_size < map.report_bytes)
        s->rx_size = map.report_bytes <= BUF_BYTES ? map.report_bytes : BUF_BYTES;
    s->can_rumble = map.rumble_bytes && (!map.rumble_type || d->maxPacketSizeTx > 0);
    s->rumble_sent = 0xFF; /* unknown: the first update settles it */
    pad_rumble_want[port] = false;
    ctx = ctx_of(idx, s->gen);
    lock_give();

    PLOG("USB pad %04x:%04x: port %d, interface %u, %u byte reports (%s)\n", d->vid, d->pid, port + 1,
         d->interfaceIndex, (unsigned)s->rx_size, where);

    if (!map.polltype) {
        /* A PlayStation 3 pad sends nothing until told to (Nintendont's
         * "enable" feature report); then it is polled with GET_REPORT. */
        static uint8_t enable[4] = { 0x42, 0x0C, 0x00, 0x00 };
        memcpy(s->tx, enable, sizeof(enable));
        lock_take();
        s->tx_busy = true;
        lock_give();
        if (HIDSetReport(d->handle, HID_REPORT_FEATURE, 0xF4, s->tx, sizeof(enable), on_write, ctx) < 0) {
            lock_take();
            s->tx_busy = false;
            lock_give();
        }
        return CLAIMED;
    }
    /* A boot interface may still be in boot protocol: ask for reports. */
    if (d->subClass == 1 && HIDSetProtocol(d->handle, d->interfaceIndex, 1, on_protocol, ctx) >= 0)
        return CLAIMED;
    arm_read(idx);
    return CLAIMED;
}
#endif

static int32_t on_attach(HIDClient *c, HIDDevice *d, HIDAttachEvent event)
{
    int idx;

    (void)c;
    if (!d)
        return UNCLAIMED;

    if (event == HID_DEVICE_DETACH) {
        int kind = K_NONE;

        lock_take();
        for (int i = 0; i < MAX_SLOTS; i++) {
            slot_t *s = &slots[i];
            if (s->kind == K_NONE || s->handle != d->handle)
                continue;
            kind = s->kind;
            s->kind = K_NONE;
            s->gen = (s->gen + 1) & 0x0FFFFFFF;
        }
        seen_set(NULL, d->handle, "unplugged");
        lock_give();
        if (kind == K_GCAD)
            GLOG("GC adapter: unplugged\n");
        else if (kind == K_PAD)
            PLOG("USB pad %04x:%04x: unplugged\n", d->vid, d->pid);
        return kind != K_NONE ? CLAIMED : UNCLAIMED;
    }

    /* Keyboards belong to the keyboard library, tablets to the tablet driver. */
    if (d->subClass == 1 && d->protocol == 1) {
        PLOG("USB %04x:%04x if %u: keyboard\n", d->vid, d->pid, d->interfaceIndex);
        seen_note(d, "keyboard");
        return UNCLAIMED;
    }
    if (inpw_tablet_vendor(d->vid)) {
        PLOG("USB %04x:%04x if %u: a tablet maker's, left to the tablet driver\n", d->vid, d->pid,
             d->interfaceIndex);
        seen_note(d, "tablet driver's");
        return UNCLAIMED;
    }

    lock_take();
    idx = free_slot();
    lock_give();
    if (idx < 0) {
        seen_note(d, "no free slot");
        return UNCLAIMED;
    }

    if (want_gcad && d->vid == GCAD_VID && d->pid == GCAD_PID) {
        bool taken;
        lock_take();
        taken = have_kind(K_GCAD);
        lock_give();
        seen_note(d, taken ? "second GC adapter" : "GC adapter");
        return taken ? UNCLAIMED : attach_gcad(d, idx);
    }
#ifdef HAVE_HIDPADS
    /* A pad's layout before the mouse: some pads have a boot interface. */
    if (want_pads) {
        int32_t r = attach_pad(d, idx);
        if (r == CLAIMED)
            return r;
    }
#endif
    if (want_mouse && d->subClass == 1 && d->protocol == 2) {
        PLOG("USB %04x:%04x if %u: mouse\n", d->vid, d->pid, d->interfaceIndex);
        seen_note(d, "mouse");
        return attach_mouse(d, idx);
    }
    seen_note(d, want_pads ? "left alone" : "left alone (pads off)");
    return UNCLAIMED;
}

/* ---- Main thread -------------------------------------------------------- */

int32_t inpw_hid_init(bool mouse, bool gcadapter, bool hidpads)
{
    if (client_added)
        return 0;

    want_mouse = mouse;
#ifdef HAVE_GCADAPTER
    want_gcad = gcadapter;
#else
    (void)gcadapter;
    want_gcad = false;
#endif
#ifdef HAVE_HIDPADS
    want_pads = hidpads;
#else
    (void)hidpads;
    want_pads = false;
#endif
    if (!want_mouse && !want_gcad && !want_pads)
        return 0;

    OSInitSpinLock(&lock);
    m_dx = m_dy = m_wheel = 0;
    m_quick = 0;
    for (int i = 0; i < MAX_SLOTS; i++) {
        uint32_t gen = slots[i].gen;
        if (!slots[i].rx)
            slots[i].rx = memalign(0x40, BUF_BYTES);
        if (!slots[i].tx)
            slots[i].tx = memalign(0x40, BUF_BYTES);
        if (!slots[i].desc)
            slots[i].desc = memalign(0x40, RDESC_BYTES); /* without it, no generic pads */
        if (!slots[i].rx || !slots[i].tx)
            return -1;
        slots[i].kind = K_NONE;
        slots[i].gen = gen;
    }
    memset(gcad_rumble_want, 0, sizeof(gcad_rumble_want));
    seen_count = 0;
    memset(&client, 0, sizeof(client));
#ifdef HAVE_HIDPADS
    if (want_pads)
        load_files();
#endif

    /* The keyboard library may have set the HID library up already; only
     * what this side set up is torn down. */
    hid_setup = HIDSetup() >= 0;
    /* Devices already plugged in are offered from inside HIDAddClient. */
    client_added = HIDAddClient(&client, on_attach) >= 0;
    if (!client_added) {
        if (hid_setup)
            HIDTeardown();
        hid_setup = false;
        return -1;
    }
    return 0;
}

/* Rumble off at shutdown, sent synchronously (no callback). */
static void stop_rumble(slot_t *s)
{
    if (s->kind == K_GCAD) {
        uint8_t off[5] = { 0x11, 0, 0, 0, 0 };
        memcpy(s->tx, off, sizeof(off));
        HIDWrite(s->handle, s->tx, sizeof(off), NULL, NULL);
    }
#ifdef HAVE_HIDPADS
    else if (s->kind == K_PAD && s->can_rumble && s->rumble_sent == 1) {
        const hidpad_mapping *m = &s->map;
        for (unsigned i = 0; i < (m->rumble_type ? m->rumble_count : 1); i++) {
            memcpy(s->tx, m->rumble_off + (m->rumble_type ? i * m->rumble_transfer : 0), m->rumble_transfer);
            if (m->rumble_type)
                HIDWrite(s->handle, s->tx, m->rumble_transfer, NULL, NULL);
            else
                HIDSetReport(s->handle, HID_REPORT_OUTPUT, 1, s->tx, m->rumble_transfer, NULL, NULL);
        }
    }
#endif
}

void inpw_hid_shutdown(void)
{
    if (!client_added)
        return;

    for (int i = 0; i < MAX_SLOTS; i++)
        if (slots[i].kind != K_NONE && !slots[i].tx_busy)
            stop_rumble(&slots[i]);

    HIDDelClient(&client);
    if (hid_setup)
        HIDTeardown();
    client_added = hid_setup = false;

    lock_take();
    for (int i = 0; i < MAX_SLOTS; i++) {
        slots[i].kind = K_NONE;
        slots[i].gen = (slots[i].gen + 1) & 0x0FFFFFFF;
    }
    lock_give();
    for (int i = 0; i < MAX_SLOTS; i++) {
        free(slots[i].rx);
        free(slots[i].tx);
        free(slots[i].desc);
        slots[i].rx = slots[i].tx = slots[i].desc = NULL;
    }
}

#ifdef HAVE_HIDPADS
/* One step of a pad's rumble: the next block of the payload, or a new
 * on/off when it changed. Never alongside another transfer. */
static void pump_pad(int idx)
{
    slot_t *s = &slots[idx];
    const hidpad_mapping *m;
    uint32_t handle, offset;
    const uint8_t *data;
    void *ctx;
    bool send = false;

    lock_take();
    m = &s->map;
    if (s->kind != K_PAD || s->tx_busy || !s->can_rumble) {
        lock_give();
        return;
    }
    if (s->rumble_active && s->rumble_index >= (m->rumble_type ? m->rumble_count : 1)) {
        s->rumble_active = false;
        s->rumble_sent = s->rumble_flight;
    }
    if (!s->rumble_active && (uint8_t)pad_rumble_want[s->port] != s->rumble_sent) {
        s->rumble_active = true;
        s->rumble_flight = pad_rumble_want[s->port];
        s->rumble_index = 0;
    }
    if (s->rumble_active) {
        /* Interrupt rumble steps through the payload; SET_REPORT sends the first block. */
        offset = m->rumble_type ? (uint32_t)s->rumble_index * m->rumble_transfer : 0;
        data = s->rumble_flight ? m->rumble_on : m->rumble_off;
        memcpy(s->tx, data + offset, m->rumble_transfer);
        s->rumble_index++;
        s->tx_busy = true;
        send = true;
    }
    handle = s->handle;
    ctx = ctx_of(idx, s->gen);
    lock_give();
    if (!send)
        return;
    if ((m->rumble_type ? HIDWrite(handle, s->tx, m->rumble_transfer, on_write, ctx)
                        : HIDSetReport(handle, HID_REPORT_OUTPUT, 1, s->tx, m->rumble_transfer, on_write, ctx)) < 0) {
        lock_take();
        if (slot_of(ctx, handle)) {
            s->tx_busy = false;
            s->rumble_active = false;
            s->rumble_sent = s->rumble_flight; /* not retried every frame */
        }
        lock_give();
    }
}

/* A GET_REPORT pad (PlayStation 3): one poll a frame. */
static void poll_pad(int idx)
{
    slot_t *s = &slots[idx];
    uint32_t handle, size;
    void *ctx;

    lock_take();
    if (s->kind != K_PAD || s->generic || s->map.polltype || s->rx_busy || s->tx_busy || s->fails >= FAILS_MAX) {
        lock_give();
        return;
    }
    s->rx_busy = true;
    handle = s->handle;
    size = s->rx_size;
    ctx = ctx_of(idx, s->gen);
    lock_give();
    if (HIDGetReport(handle, HID_REPORT_INPUT, 1, s->rx, size, on_read, ctx) < 0) {
        lock_take();
        if (slot_of(ctx, handle)) {
            s->rx_busy = false;
            s->fails++;
        }
        lock_give();
    }
}
#endif

static void pump_gcad(int idx)
{
    slot_t *s = &slots[idx];
    uint8_t cmd[5] = { 0x11, 0, 0, 0, 0 };
    bool send = false;
    uint32_t handle;
    void *ctx;

    lock_take();
    if (s->kind == K_GCAD && !s->tx_busy && s->reporting) {
        for (int p = 0; p < 4; p++) {
            /* No rumble for a WaveBird or an empty port. */
            bool wired = (s->raw[p][0] & 0x30) == 0x10;
            cmd[1 + p] = gcad_rumble_want[p] && wired;
        }
        if (memcmp(cmd, s->tx, sizeof(cmd)) != 0) {
            memcpy(s->tx, cmd, sizeof(cmd));
            s->tx_busy = true;
            send = true;
        }
    }
    handle = s->handle;
    ctx = ctx_of(idx, s->gen);
    lock_give();
    if (send && HIDWrite(handle, s->tx, sizeof(cmd), on_write, ctx) < 0) {
        lock_take();
        if (slot_of(ctx, handle))
            s->tx_busy = false;
        lock_give();
    }
}

void inpw_hid_update(void)
{
    if (!client_added)
        return;
    for (int i = 0; i < MAX_SLOTS; i++) {
        switch (slots[i].kind) {
        case K_GCAD:
            pump_gcad(i);
            break;
#ifdef HAVE_HIDPADS
        case K_PAD:
            pump_pad(i);
            poll_pad(i);
            break;
#endif
        }
    }
}

void inpw_hid_mouse_take(int *dx, int *dy, int *wheel, uint8_t *held, uint8_t *pressed, bool *connected)
{
    uint8_t b = 0;
    bool any = false;

    if (!client_added) {
        *dx = *dy = *wheel = 0;
        *held = *pressed = 0;
        *connected = false;
        return;
    }
    lock_take();
    *dx = m_dx;
    *dy = m_dy;
    *wheel = m_wheel;
    *pressed = m_quick;
    m_dx = m_dy = m_wheel = 0;
    m_quick = 0;
    for (int i = 0; i < MAX_SLOTS; i++) {
        if (slots[i].kind == K_MOUSE) {
            any = true;
            b |= slots[i].mbuttons;
        }
    }
    lock_give();
    *held = b;
    *connected = any;
}

/* ---- GameCube adapter ----------------------------------------------------
 * The adapter sends raw values (sticks near 128); like the console's own
 * ports they are reported against a centre taken when a controller is
 * plugged in, from a reading at rest. Kept on the main thread. */

typedef struct {
    bool present, origin_valid, recal_active;
    uint8_t origin[6];
    uint32_t since, recal_since;
    bool have;
    input_gc_pad pad;
} gc_port;

static gc_port gports[4];
static bool gcad_ok;

static int8_t relative(uint8_t raw, uint8_t origin)
{
    int v = (int)raw - (int)origin;
    return (int8_t)(v < -128 ? -128 : v > 127 ? 127 : v);
}

static uint8_t above(uint8_t raw, uint8_t origin) { return raw > origin ? (uint8_t)(raw - origin) : 0; }

static bool at_rest(const uint8_t a[6])
{
    for (int i = 0; i < 4; i++)
        if (a[i] < 128 - 32 || a[i] > 128 + 32)
            return false;
    return a[4] <= 80 && a[5] <= 80;
}

static void gc_decode(const uint8_t raw[9], const uint8_t origin[6], input_gc_pad *out)
{
    uint8_t b1 = raw[1], b2 = raw[2];
    uint16_t buttons = 0;

    if (b1 & 0x01) buttons |= INPUT_GC_A;
    if (b1 & 0x02) buttons |= INPUT_GC_B;
    if (b1 & 0x04) buttons |= INPUT_GC_X;
    if (b1 & 0x08) buttons |= INPUT_GC_Y;
    if (b1 & 0x10) buttons |= INPUT_GC_LEFT;
    if (b1 & 0x20) buttons |= INPUT_GC_RIGHT;
    if (b1 & 0x40) buttons |= INPUT_GC_DOWN;
    if (b1 & 0x80) buttons |= INPUT_GC_UP;
    if (b2 & 0x01) buttons |= INPUT_GC_START;
    if (b2 & 0x02) buttons |= INPUT_GC_Z;
    if (b2 & 0x04) buttons |= INPUT_GC_R;
    if (b2 & 0x08) buttons |= INPUT_GC_L;
    out->buttons = buttons;
    out->stick_x = relative(raw[3], origin[0]);
    out->stick_y = relative(raw[4], origin[1]);
    out->substick_x = relative(raw[5], origin[2]);
    out->substick_y = relative(raw[6], origin[3]);
    out->trigger_l = above(raw[7], origin[4]);
    out->trigger_r = above(raw[8], origin[5]);
    out->wireless = (raw[0] & 0x10) == 0;
}

/* Called by the pads once a frame, before reading the ports. */
void inpw_gcadapter_update(void)
{
    static const uint8_t middle[6] = { 128, 128, 128, 128, 0, 0 };
    uint8_t raw[4][9];
    uint32_t now = inpw_now_ms(), last = 0;
    bool found = false, reporting = false;

    if (!client_added || !want_gcad) {
        gcad_ok = false;
        return;
    }
    lock_take();
    for (int i = 0; i < MAX_SLOTS; i++) {
        if (slots[i].kind == K_GCAD) {
            found = true;
            reporting = slots[i].reporting;
            last = slots[i].report_ms;
            memcpy(raw, slots[i].raw, sizeof(raw));
        }
    }
    lock_give();

    gcad_ok = found && reporting && now - last < GCAD_TIMEOUT_MS;

    for (int p = 0; p < 4; p++) {
        gc_port *g = &gports[p];
        const uint8_t *r = raw[p];
        uint8_t analog[6];

        if (!gcad_ok || (r[0] & 0x30) == 0) {
            if (g->present)
                GLOG("GC adapter: port %d empty\n", p + 1);
            memset(g, 0, sizeof(*g));
            continue;
        }
        memcpy(analog, r + 3, 6);
        if (!g->present) {
            g->present = true;
            g->since = now;
            GLOG("GC adapter: port %d %s\n", p + 1, (r[0] & 0x10) ? "controller" : "WaveBird");
        }
        if (!g->origin_valid) {
            if (at_rest(analog)) {
                memcpy(g->origin, analog, 6);
                g->origin_valid = true;
            } else if (now - g->since >= GCAD_ORIGIN_WAIT_MS) {
                memcpy(g->origin, middle, 6);
                g->origin_valid = true;
            }
        }
        gc_decode(r, g->origin_valid ? g->origin : middle, &g->pad);

        /* X+Y+Start held for three seconds: a new centre, as on the console. */
        if ((g->pad.buttons & (INPUT_GC_X | INPUT_GC_Y | INPUT_GC_START)) ==
            (INPUT_GC_X | INPUT_GC_Y | INPUT_GC_START)) {
            if (!g->recal_active) {
                g->recal_active = true;
                g->recal_since = now;
            } else if (now - g->recal_since >= GCAD_RECAL_MS) {
                memcpy(g->origin, analog, 6);
                g->origin_valid = true;
                g->recal_since = now;
                GLOG("GC adapter: port %d centred again\n", p + 1);
            }
        } else {
            g->recal_active = false;
        }
        g->have = true;
    }
}

bool INPUT_GCAdapterConnected(void) { return gcad_ok; }

bool INPUT_GCAdapterPad(int port, input_gc_pad *out)
{
    if (port < 0 || port > 3 || !gcad_ok || !gports[port].have)
        return false;
    if (out)
        *out = gports[port].pad;
    return true;
}

void INPUT_GCAdapterRumble(int port, bool on)
{
    if (port >= 0 && port < 4)
        gcad_rumble_want[port] = on;
}

/* ---- USB pads ------------------------------------------------------------ */

#ifdef HAVE_HIDPADS
static slot_t *pad_slot(int port)
{
    for (int i = 0; i < MAX_SLOTS; i++)
        if (slots[i].kind == K_PAD && slots[i].port == port)
            return &slots[i];
    return NULL;
}

bool INPUT_HIDPadConnected(void)
{
    bool any = false;

    if (!client_added)
        return false;
    lock_take();
    for (int i = 0; i < MAX_SLOTS; i++)
        any = any || (slots[i].kind == K_PAD && slots[i].state_valid);
    lock_give();
    return any;
}

bool INPUT_HIDPadPad(int port, input_gc_pad *out)
{
    hidpad_state st;
    slot_t *s;
    bool on = false;

    if (!client_added || port < 0 || port >= MAX_PADS)
        return false;
    lock_take();
    s = pad_slot(port);
    if (s && s->state_valid) {
        on = true;
        st = s->state;
    }
    lock_give();
    if (!on)
        return false;
    if (out) {
        out->buttons = st.buttons;
        out->stick_x = st.stick_x;
        out->stick_y = st.stick_y;
        out->substick_x = st.substick_x;
        out->substick_y = st.substick_y;
        out->trigger_l = st.trigger_l;
        out->trigger_r = st.trigger_r;
        out->wireless = st.no_rumble != 0;
    }
    return true;
}

const char *INPUT_HIDPadName(int port)
{
    const char *name = NULL;
    slot_t *s;

    if (!client_added || port < 0 || port >= MAX_PADS)
        return NULL;
    lock_take();
    s = pad_slot(port);
    if (s)
        name = s->where; /* a string constant: a built-in name or a fixed phrase */
    lock_give();
    return name;
}

uint16_t INPUT_HIDPadRaw(int port)
{
    uint16_t raw = 0;
    slot_t *s;

    if (!client_added || port < 0 || port >= MAX_PADS)
        return 0;
    lock_take();
    s = pad_slot(port);
    if (s && s->generic)
        raw = s->gen_raw;
    lock_give();
    return raw;
}

void INPUT_HIDPadRumble(int port, bool on)
{
    if (port >= 0 && port < MAX_PADS)
        pad_rumble_want[port] = on;
}
#else
bool        INPUT_HIDPadConnected(void) { return false; }
bool        INPUT_HIDPadPad(int port, input_gc_pad *out) { (void)port; (void)out; return false; }
const char *INPUT_HIDPadName(int port) { (void)port; return NULL; }
uint16_t    INPUT_HIDPadRaw(int port) { (void)port; return 0; }
void        INPUT_HIDPadRumble(int port, bool on) { (void)port; (void)on; }
#endif
