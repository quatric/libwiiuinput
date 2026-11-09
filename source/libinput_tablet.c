/* USB pen tablets through the system's HID library.  See input_tablet.h.
 *
 * libwiiinput's tablet driver on nsyshid: the report descriptor parser
 * (libinput_hidparse.c) and the report decoding are its, unchanged; what
 * changed is the transport.  The system offers each HID interface of a
 * tablet maker's device to this client, which takes it, asks for its report
 * descriptor and decides from that what the interface is: the pen, a Bamboo's
 * touch surface, or nothing to read.  Every step is a callback on the
 * system's HID thread; the samples are handed to the main thread under an
 * uninterruptible spinlock. */

#ifdef HAVE_TABLET

#include <malloc.h>
#include <stdio.h>
#include <string.h>

#include <coreinit/spinlock.h>
#include <nsyshid/hid.h>

#include "input_tablet.h"
#include "libinput_hidparse.h"
#include "libinput_internal.h"

#define WACOM_VID     0x056A
#define LEGACY_REPORT 2       /* old Wacom protocol: report 2, mode in feature report 2 */
#define MAX_IFACES    6
#define RDESC_MAX     2048
#define INTR_MAX      64
#define QUEUE         256
#define QUEUE_MASK    (QUEUE - 1)
#define FAILS_MAX     20

#define HID_DT_DEVICE 0x01
#define HID_DT_STRING 0x03
#define HID_DT_REPORT 0x22
#define HID_FEATURE   3

enum { R_NONE, R_PROBING, R_PEN, R_TOUCH, R_CANDIDATE, R_IDLE };

typedef struct {
    int role;
    uint32_t gen;
    uint32_t handle;
    uint8_t iface, subclass, protocol;
    uint16_t vid, pid;
    uint32_t phys;
    uint16_t rx_size;
    int fails;
    uint8_t *buf;        /* report descriptor, then reports; kept until shutdown */
    uint8_t *ctl;        /* feature reports, device and string descriptors */
} tif;

static OSSpinLock lock;
static tif ifs[MAX_IFACES];
static HIDClient client;
static bool running, hid_setup;

static tb_info tb_dev;
static bool tb_opened;
static hidp_desc tb_desc;
static hidp_pen tb_pen;
static hidp_desc probe_desc;   /* only the HID thread probes */

static tb_sample tb_state;
static tb_sample tb_queue[QUEUE];
static uint32_t tb_head, tb_tail;
static tb_finger tb_fingers[TB_MAX_FINGERS];
static tb_stats tb_st;

static int (*tb_logger)(const char *fmt, ...);
#define tb_log(...) (tb_logger ? (void)tb_logger(__VA_ARGS__) : (void)0)

void TB_SetLog(int (*print)(const char *fmt, ...)) { tb_logger = print; }

static void lock_take(void) { OSUninterruptibleSpinLock_Acquire(&lock); }
static void lock_give(void) { OSUninterruptibleSpinLock_Release(&lock); }

static void *ctx_of(int i) { return (void *)(uintptr_t)((ifs[i].gen << 4) | (uint32_t)i); }

static tif *tif_of(void *ctx, uint32_t handle)
{
    uintptr_t v = (uintptr_t)ctx;
    int i = (int)(v & 0xF);

    if (i >= MAX_IFACES || ifs[i].role == R_NONE || ifs[i].gen != (uint32_t)(v >> 4) || ifs[i].handle != handle)
        return NULL;
    return &ifs[i];
}

static int idx_of(void *ctx) { return (int)((uintptr_t)ctx & 0xF); }

static uint16_t le16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static uint16_t be16(const uint8_t *p) { return (uint16_t)((p[0] << 8) | p[1]); }

/* Pen tablet makers: Wacom, Huion and Gaomon, XP-Pen and UGEE, Veikk,
 * UC-Logic, Waltop (libwiiinput's list). */
bool inpw_tablet_vendor(uint16_t vid)
{
    return vid == WACOM_VID || vid == 0x256C || vid == 0x28BD || vid == 0x2FEB || vid == 0x5543 || vid == 0x172F;
}

bool inpw_tablet_running(void) { return running; }

/* ---- reports (under `lock`) --------------------------------------------- */

static void push(const tb_sample *in)
{
    tb_sample s = *in;
    uint32_t next;

    s.wiimote = -1;
    if (s.finger)
        tb_state.pad = s.pad;
    else
        tb_state = s;
    next = (tb_head + 1) & QUEUE_MASK;
    if (next == tb_tail)
        tb_tail = (tb_tail + 1) & QUEUE_MASK; /* drop the oldest */
    tb_queue[tb_head] = s;
    tb_head = next;
}

static int32_t get(const hidp_field *f, const uint8_t *data, uint32_t len)
{
    return f ? hidp_get(f, data, len) : 0;
}

static void finish(tb_sample *s)
{
    s->time = inpw_now_ms();
    if (s->x < 0) s->x = 0;
    if (s->y < 0) s->y = 0;
    if (s->x > tb_dev.max_x) s->x = tb_dev.max_x;
    if (s->y > tb_dev.max_y) s->y = tb_dev.max_y;
    if (s->raw_pressure > tb_dev.max_pressure)
        s->raw_pressure = tb_dev.max_pressure;
    s->nx = (float)s->x / (float)tb_dev.max_x;
    s->ny = (float)s->y / (float)tb_dev.max_y;
    s->pressure = (float)s->raw_pressure / (float)tb_dev.max_pressure;
}

static void legacy_report(const uint8_t *r, uint32_t len)
{
    tb_sample s = tb_state;
    bool prox;

    if (r[0] != LEGACY_REPORT || len < 8) {
        tb_st.other++;
        return;
    }
    tb_st.pen_reports++;
    s.finger = false;
    s.in_range = (r[1] & 0x80) != 0;
    prox = (r[1] & 0x40) != 0;
    s.buttons = 0;
    s.raw_pressure = 0;
    if (r[1] & 0x20) { /* data ready */
        s.raw_pressure = le16(r + 6);
        if (r[1] & 0x01) s.buttons |= TB_BUTTON_TIP;
        if (r[1] & 0x02) s.buttons |= TB_BUTTON_BARREL;
        if (r[1] & 0x04) s.buttons |= TB_BUTTON_BARREL2;
    }
    if (prox) {
        s.x = le16(r + 2);
        s.y = le16(r + 4);
        s.eraser = (r[1] & 0x08) != 0;
        /* The range table is a guess; let the pen correct it. */
        if (s.x > tb_dev.max_x) tb_dev.max_x = s.x;
        if (s.y > tb_dev.max_y) tb_dev.max_y = s.y;
        if (s.raw_pressure > tb_dev.max_pressure)
            tb_dev.max_pressure = s.raw_pressure > 1023 ? 2047 : 1023;
    }
    finish(&s);
    push(&s);
}

static void hid_report(const uint8_t *r, uint32_t len)
{
    uint8_t id = tb_desc.uses_ids ? r[0] : 0;
    const uint8_t *data = tb_desc.uses_ids ? r + 1 : r;
    uint32_t n = tb_desc.uses_ids ? len - 1 : len;
    tb_sample s = tb_state;
    bool used = false;

    if (len == 0)
        return;
    s.finger = false;
    for (int i = 0; i < tb_pen.npad; i++) {
        const hidp_field *f = tb_pen.pad[i];
        if (f->report_id != id)
            continue;
        used = true;
        if (hidp_get(f, data, n))
            s.pad |= (uint16_t)(1u << i);
        else
            s.pad &= (uint16_t)~(1u << i);
    }

    if (id == tb_pen.report_id) {
        used = true;
        if (tb_pen.valid && !hidp_get(tb_pen.valid, data, n))
            return;
        tb_st.pen_reports++;
        s.x = get(tb_pen.x, data, n) - tb_pen.x->logical_min;
        s.y = get(tb_pen.y, data, n) - tb_pen.y->logical_min;
        s.raw_pressure = tb_pen.pressure ? (uint32_t)get(tb_pen.pressure, data, n) : 0;
        s.in_range = tb_pen.in_range ? get(tb_pen.in_range, data, n) != 0 : true;
        s.eraser = get(tb_pen.eraser, data, n) || get(tb_pen.invert, data, n);
        s.buttons = 0;
        if (tb_pen.tip ? get(tb_pen.tip, data, n) || get(tb_pen.eraser, data, n) : s.raw_pressure > 0)
            s.buttons |= TB_BUTTON_TIP;
        if (get(tb_pen.barrel, data, n))
            s.buttons |= TB_BUTTON_BARREL;
        if (get(tb_pen.barrel2, data, n))
            s.buttons |= TB_BUTTON_BARREL2;
        s.tilt_x = (int8_t)get(tb_pen.tilt_x, data, n);
        s.tilt_y = (int8_t)get(tb_pen.tilt_y, data, n);
        if (!s.in_range) {
            s.buttons = 0;
            s.raw_pressure = 0;
        }
    }

    if (!used) {
        tb_st.other++;
        return;
    }
    finish(&s);
    push(&s);
}

/* The tablet keys, in both touch formats. */
static uint16_t bamboo_keys(uint8_t bits)
{
    return (uint16_t)(((bits & 0x08) ? 1 : 0) | ((bits & 0x04) ? 2 : 0) | ((bits & 0x02) ? 4 : 0) |
                      ((bits & 0x01) ? 8 : 0));
}

/* A Bamboo touch packet, report 2: 20 bytes with two finger slots on
 * first-generation tablets, up to seven 8-byte messages on later ones. */
static void touch_report(const uint8_t *r, uint32_t len, uint16_t pkt)
{
    tb_sample s = tb_state;
    bool pen_near = tb_state.in_range && !tb_state.finger;
    int first = -1;

    if (len < 2 || r[0] != LEGACY_REPORT) {
        tb_st.other++;
        return;
    }
    tb_st.touch_reports++;

    if (pkt >= 64 || (pkt == 0 && len > 20)) {
        int count = r[1] & 0x07;
        for (int i = 0; i < count && 2 + 8 * i + 8 <= (int)len; i++) {
            const uint8_t *m = r + 2 + 8 * i;
            if (m[0] >= 2 && m[0] < 2 + TB_MAX_FINGERS) {
                tb_finger *f = &tb_fingers[m[0] - 2];
                f->down = (m[1] & 0x80) != 0;
                if (f->down) {
                    f->x = (m[2] << 4) | (m[4] >> 4);
                    f->y = (m[3] << 4) | (m[4] & 0x0F);
                }
            } else if (m[0] == 128) {
                s.pad = bamboo_keys(m[1]);
            }
        }
    } else {
        for (int i = 0; i < 2; i++) {
            int off = (r[1] & 0x80) ? 8 * i : 9 * i;
            tb_finger *f = &tb_fingers[i];
            if ((uint32_t)off + 7 > len)
                break;
            f->down = (r[off + 3] & 0x80) != 0;
            if (f->down) {
                f->x = be16(r + off + 3) & 0x7FF;
                f->y = be16(r + off + 5) & 0x7FF;
            }
        }
        s.pad = bamboo_keys(r[1]);
    }

    for (int i = 0; i < TB_MAX_FINGERS; i++) {
        tb_finger *f = &tb_fingers[i];
        if (pen_near) /* a hand resting while the pen draws */
            f->down = false;
        if (f->x > tb_dev.touch_max_x) tb_dev.touch_max_x = f->x;
        if (f->y > tb_dev.touch_max_y) tb_dev.touch_max_y = f->y;
        f->nx = (float)f->x / (float)(tb_dev.touch_max_x ? tb_dev.touch_max_x : 1);
        f->ny = (float)f->y / (float)(tb_dev.touch_max_y ? tb_dev.touch_max_y : 1);
        if (f->down && first < 0)
            first = i;
    }

    s.finger = true;
    s.eraser = false;
    s.tilt_x = s.tilt_y = 0;
    s.buttons = first >= 0 ? TB_BUTTON_TIP : 0;
    s.in_range = first >= 0;
    if (first >= 0) {
        s.x = (int32_t)(tb_fingers[first].nx * tb_dev.max_x);
        s.y = (int32_t)(tb_fingers[first].ny * tb_dev.max_y);
    }
    s.raw_pressure = 0;
    finish(&s);
    s.pressure = first >= 0 ? 0.5f : 0.0f;
    push(&s);
}

static void note_error(int32_t ret)
{
    tb_st.errors++;
    tb_st.last_ret = ret;
    for (int i = 0; i < tb_st.nfirst; i++)
        if (tb_st.first_err[i] == ret)
            return;
    if (tb_st.nfirst < 4)
        tb_st.first_err[tb_st.nfirst++] = ret;
}

/* ---- the HID thread's steps ---------------------------------------------- */

static void arm_read(int i);

static void on_read(uint32_t handle, int32_t error, uint8_t *buffer, uint32_t bytes, void *ctx)
{
    tif *t;
    bool again = false;

    (void)buffer;
    lock_take();
    t = tif_of(ctx, handle);
    if (t) {
        tb_st.reports++;
        if (error < 0) {
            note_error(error);
            again = ++t->fails < FAILS_MAX;
        } else {
            uint32_t n = bytes ? bytes : t->rx_size;
            t->fails = 0;
            again = true;
            tb_st.last_len = (uint8_t)(n < sizeof(tb_st.last) ? n : sizeof(tb_st.last));
            memcpy(tb_st.last, t->buf, tb_st.last_len);
            tb_st.last_id = t->buf[0];
            if (t->role == R_TOUCH)
                touch_report(t->buf, n, t->rx_size);
            else if (tb_dev.legacy)
                legacy_report(t->buf, n);
            else
                hid_report(t->buf, n);
        }
    }
    lock_give();
    if (t && again)
        arm_read(idx_of(ctx));
}

static void arm_read(int i)
{
    tif *t = &ifs[i];
    uint32_t handle, size;
    void *ctx;

    lock_take();
    if (t->role != R_PEN && t->role != R_TOUCH) {
        lock_give();
        return;
    }
    handle = t->handle;
    size = t->rx_size;
    ctx = ctx_of(i);
    lock_give();
    if (HIDRead(handle, t->buf, size, on_read, ctx) < 0) {
        lock_take();
        if (tif_of(ctx, handle))
            note_error(-1);
        lock_give();
    }
}

/* The product's name: the device descriptor's string index, then the string. */
static void on_name(uint32_t handle, int32_t error, uint8_t *buffer, uint32_t bytes, void *ctx)
{
    tif *t;
    char name[48];
    uint32_t n = 0;

    (void)buffer;
    lock_take();
    t = tif_of(ctx, handle);
    lock_give();
    if (!t || error < 0 || bytes < 4 || t->ctl[1] != HID_DT_STRING)
        return;
    if (t->ctl[0] < bytes)
        bytes = t->ctl[0];
    for (uint32_t i = 2; i + 1 < bytes && n + 1 < sizeof(name); i += 2) {
        uint16_t c = le16(t->ctl + i);
        if (c == 0)
            break;
        name[n++] = (c >= 0x20 && c < 0x7F) ? (char)c : '?';
    }
    while (n > 0 && name[n - 1] == ' ')
        n--;
    if (!n)
        return;
    name[n] = 0;
    lock_take();
    memcpy(tb_dev.name, name, n + 1);
    lock_give();
    tb_log("  tablet: %s\n", name);
}

static void on_device(uint32_t handle, int32_t error, uint8_t *buffer, uint32_t bytes, void *ctx)
{
    tif *t;

    (void)buffer;
    lock_take();
    t = tif_of(ctx, handle);
    lock_give();
    if (!t || error < 0 || bytes < 18 || t->ctl[15] == 0)
        return;
    HIDGetDescriptor(handle, HID_DT_STRING, t->ctl[15], 0x0409, t->ctl, 64, on_name, ctx);
}

/* Tablet mode set (or not needed): start reading, and look up the name. */
static void on_mode(uint32_t handle, int32_t error, uint8_t *buffer, uint32_t bytes, void *ctx)
{
    tif *t;

    (void)buffer;
    (void)bytes;
    lock_take();
    t = tif_of(ctx, handle);
    if (t && t->role == R_PEN)
        tb_st.mode_set = error;
    lock_give();
    if (!t)
        return;
    if (t->role == R_PEN) {
        tb_log("  tablet mode: %d\n", (int)error);
        HIDGetDescriptor(handle, HID_DT_DEVICE, 0, 0, t->ctl, 18, on_device, ctx);
    }
    arm_read(idx_of(ctx));
}

/* Takes the tablet out of the mouse mode it powers up in, where there is
 * one: old Wacoms take a 2 in feature report 2, descriptor-described
 * tablets say how with an Input Mode feature. */
static void switch_mode(int i)
{
    tif *t = &ifs[i];
    void *ctx = ctx_of(i);

    if (t->role == R_PEN && tb_dev.legacy) {
        t->ctl[0] = LEGACY_REPORT;
        t->ctl[1] = 2;
        if (HIDSetReport(t->handle, HID_FEATURE, LEGACY_REPORT, t->ctl, 2, on_mode, ctx) >= 0)
            return;
    } else if (t->role == R_PEN && tb_pen.input_mode) {
        const hidp_field *f = tb_pen.input_mode;
        uint32_t len = hidp_report_bytes(&tb_desc, HIDP_FEATURE, f->report_id);
        uint32_t skip = tb_desc.uses_ids ? 1 : 0;
        if (len <= 64) {
            memset(t->ctl, 0, 64);
            t->ctl[0] = f->report_id;
            hidp_put(f, t->ctl + skip, len - skip, 2);
            if (HIDSetReport(t->handle, HID_FEATURE, f->report_id, t->ctl, len, on_mode, ctx) >= 0)
                return;
        }
    }
    on_mode(t->handle, 0, NULL, 0, ctx);
}

static void on_protocol(uint32_t handle, int32_t error, uint8_t *buffer, uint32_t bytes, void *ctx)
{
    tif *t;

    (void)buffer;
    (void)bytes;
    lock_take();
    t = tif_of(ctx, handle);
    if (t && t->role == R_PEN)
        tb_st.proto_ret = error;
    lock_give();
    if (t)
        switch_mode(idx_of(ctx));
}

/* Report protocol, in case the interface came up in boot protocol. */
static void start(int i)
{
    tif *t = &ifs[i];
    void *ctx = ctx_of(i);

    if (HIDSetProtocol(t->handle, t->iface, 1, on_protocol, ctx) < 0)
        switch_mode(i);
}

/* A Bamboo's touch interface: its descriptor gives the finger's range,
 * else Linux's defaults for the two packet formats stand in. */
static void become_touch(int i, const hidp_desc *d)
{
    tif *t = &ifs[i];

    lock_take();
    t->role = R_TOUCH;
    tb_dev.has_touch = true;
    tb_dev.touch_max_x = t->rx_size >= 64 ? 4096 : 480;
    tb_dev.touch_max_y = tb_dev.touch_max_x;
    for (int k = 0; d && k < d->nfields; k++) {
        const hidp_field *f = &d->fields[k];
        if (f->type != HIDP_INPUT || f->app == HIDP_GD_MOUSE || f->logical_max <= 0)
            continue;
        if (f->usage == HIDP_GD_X)
            tb_dev.touch_max_x = f->logical_max;
        else if (f->usage == HIDP_GD_Y)
            tb_dev.touch_max_y = f->logical_max;
    }
    memset(tb_fingers, 0, sizeof(tb_fingers));
    lock_give();
    tb_log("  touch: if %u pkt %u, %dx%d\n", t->iface, t->rx_size, (int)tb_dev.touch_max_x, (int)tb_dev.touch_max_y);
    start(i);
}

static void become_pen(int i, bool legacy)
{
    tif *t = &ifs[i];

    lock_take();
    memset(&tb_dev, 0, sizeof(tb_dev));
    tb_dev.vid = t->vid;
    tb_dev.pid = t->pid;
    snprintf(tb_dev.name, sizeof(tb_dev.name), "%04x:%04x", t->vid, t->pid);
    tb_dev.interface = t->iface;
    tb_dev.max_packet = t->rx_size;
    tb_dev.legacy = legacy;
    if (legacy) {
        /* Intuos S/M ranges; grown if the pen goes further. */
        memset(&tb_desc, 0, sizeof(tb_desc));
        memset(&tb_pen, 0, sizeof(tb_pen));
        tb_dev.report_id = LEGACY_REPORT;
        tb_dev.max_x = 15200;
        tb_dev.max_y = 9500;
        tb_dev.max_pressure = 1023;
    } else {
        tb_desc = probe_desc;
        hidp_find_pen(&tb_desc, &tb_pen); /* again: its pointers lead into tb_desc */
        tb_dev.report_id = tb_pen.report_id;
        tb_dev.max_x = tb_pen.x->logical_max;
        tb_dev.max_y = tb_pen.y->logical_max;
        tb_dev.max_pressure = tb_pen.pressure ? (uint32_t)tb_pen.pressure->logical_max : 1;
        tb_dev.has_tilt = tb_pen.tilt_x && tb_pen.tilt_y;
        tb_dev.has_eraser = tb_pen.eraser || tb_pen.invert;
        tb_dev.pad_buttons = (uint8_t)tb_pen.npad;
    }
    if (tb_dev.max_x <= 0) tb_dev.max_x = 1;
    if (tb_dev.max_y <= 0) tb_dev.max_y = 1;
    if (tb_dev.max_pressure == 0) tb_dev.max_pressure = 1;
    memset(&tb_state, 0, sizeof(tb_state));
    tb_state.wiimote = -1;
    t->role = R_PEN;
    tb_opened = true;
    tb_st.opens++;
    lock_give();

    tb_log("  %s: if %u pkt %u, %dx%d, pressure %u%s%s%s, %u buttons\n", tb_dev.name, t->iface, t->rx_size,
           (int)tb_dev.max_x, (int)tb_dev.max_y, (unsigned)tb_dev.max_pressure, tb_dev.has_tilt ? ", tilt" : "",
           tb_dev.has_eraser ? ", eraser" : "", legacy ? ", old Wacom protocol" : "", tb_dev.pad_buttons);
    start(i);

    /* A touch interface of the same tablet that answered first. */
    if (legacy)
        for (int k = 0; k < MAX_IFACES; k++)
            if (k != i && ifs[k].role == R_CANDIDATE && ifs[k].phys == t->phys)
                become_touch(k, NULL);
}

/* The report descriptor: is this the pen? */
static void on_rdesc(uint32_t handle, int32_t error, uint8_t *buffer, uint32_t bytes, void *ctx)
{
    int i = idx_of(ctx);
    tif *t;
    bool parsed, pen = false, have_pen;

    (void)buffer;
    lock_take();
    t = tif_of(ctx, handle);
    have_pen = tb_opened;
    lock_give();
    if (!t)
        return;

    parsed = error >= 0 && bytes > 0 && hidp_parse(t->buf, bytes > RDESC_MAX ? RDESC_MAX : bytes, &probe_desc);
    if (parsed) {
        hidp_pen probe_pen; /* tb_pen stays the open tablet's */
        pen = hidp_find_pen(&probe_desc, &probe_pen);
    }
    tb_log("  tablet %04x:%04x if %u (%u/%u, %u bytes): %s\n", t->vid, t->pid, t->iface, t->subclass, t->protocol,
           t->rx_size, error < 0 ? "no report descriptor" : !parsed ? "descriptor unreadable" : pen ? "pen" : "no pen");

    if (pen && !have_pen) {
        become_pen(i, false);
        return;
    }
    /* A Wacom that describes no pen is one of the old fixed-format ones:
     * its first interface is the pen, a second one the touch surface. */
    if (!pen && t->vid == WACOM_VID && !have_pen && t->iface == 0) {
        become_pen(i, true);
        return;
    }
    if (!pen && t->vid == WACOM_VID && t->iface != 0) {
        if (have_pen && tb_dev.legacy && tb_dev.vid == t->vid) {
            become_touch(i, parsed ? &probe_desc : NULL);
            return;
        }
        lock_take();
        t->role = R_CANDIDATE;
        lock_give();
        return;
    }
    lock_take();
    t->role = R_IDLE; /* the tablet's mouse or key interface: nothing to read */
    lock_give();
}

static int32_t on_attach(HIDClient *c, HIDDevice *d, HIDAttachEvent event)
{
    int i = -1;
    void *ctx;

    (void)c;
    if (!d)
        return 0;
    if (event == HID_DEVICE_DETACH) {
        bool ours = false, pen = false;
        lock_take();
        for (int k = 0; k < MAX_IFACES; k++) {
            if (ifs[k].role == R_NONE || ifs[k].handle != d->handle)
                continue;
            ours = true;
            pen = ifs[k].role == R_PEN;
            if (ifs[k].role == R_TOUCH) {
                tb_dev.has_touch = false;
                memset(tb_fingers, 0, sizeof(tb_fingers));
            }
            ifs[k].role = R_NONE;
            ifs[k].gen = (ifs[k].gen + 1) & 0x0FFFFFFF;
        }
        if (pen) {
            tb_opened = false;
            memset(&tb_state, 0, sizeof(tb_state));
            tb_state.wiimote = -1;
        }
        lock_give();
        if (pen)
            tb_log("  tablet unplugged\n");
        return ours ? 1 : 0;
    }

    /* A tablet maker's device, and not its keyboard (left to the keyboard library). */
    if (!inpw_tablet_vendor(d->vid) || (d->subClass == 1 && d->protocol == 1))
        return 0;

    lock_take();
    for (int k = 0; k < MAX_IFACES && i < 0; k++)
        if (ifs[k].role == R_NONE)
            i = k;
    if (i >= 0) {
        tif *t = &ifs[i];
        uint8_t *buf = t->buf, *ctl = t->ctl;
        uint32_t gen = (t->gen + 1) & 0x0FFFFFFF;
        memset(t, 0, sizeof(*t));
        t->buf = buf;
        t->ctl = ctl;
        t->gen = gen;
        t->role = R_PROBING;
        t->handle = d->handle;
        t->iface = d->interfaceIndex;
        t->subclass = d->subClass;
        t->protocol = d->protocol;
        t->vid = d->vid;
        t->pid = d->pid;
        t->phys = d->physicalDeviceInst;
        t->rx_size = d->maxPacketSizeRx;
        if (t->rx_size == 0 || t->rx_size > INTR_MAX)
            t->rx_size = INTR_MAX;
    }
    lock_give();
    if (i < 0)
        return 0;

    ctx = ctx_of(i);
    /* The report descriptor, asked of the interface (wIndex). */
    if (HIDGetDescriptor(d->handle, HID_DT_REPORT, 0, d->interfaceIndex, ifs[i].buf, RDESC_MAX, on_rdesc, ctx) < 0) {
        tb_log("  tablet %04x:%04x if %u: the descriptor could not be asked for\n", d->vid, d->pid, d->interfaceIndex);
        lock_take();
        ifs[i].role = R_IDLE;
        lock_give();
    }
    return 1;
}

/* ---- API ------------------------------------------------------------------ */

int32_t TB_Init(void)
{
    if (running)
        return TB_OK;
    OSInitSpinLock(&lock);
    for (int i = 0; i < MAX_IFACES; i++) {
        if (!ifs[i].buf)
            ifs[i].buf = memalign(0x40, RDESC_MAX);
        if (!ifs[i].ctl)
            ifs[i].ctl = memalign(0x40, 64);
        if (!ifs[i].buf || !ifs[i].ctl)
            return TB_ENOMEM;
        ifs[i].role = R_NONE;
    }
    memset(&tb_state, 0, sizeof(tb_state));
    tb_state.wiimote = -1;
    memset(&tb_st, 0, sizeof(tb_st));
    tb_head = tb_tail = 0;
    tb_opened = false;
    memset(&client, 0, sizeof(client));

    running = true;
    hid_setup = HIDSetup() >= 0;
    /* Tablets already plugged in are offered from inside HIDAddClient. */
    if (HIDAddClient(&client, on_attach) < 0) {
        running = false;
        if (hid_setup)
            HIDTeardown();
        hid_setup = false;
        return TB_ENODEV;
    }
    return TB_OK;
}

void TB_Shutdown(void)
{
    if (!running)
        return;
    HIDDelClient(&client);
    if (hid_setup)
        HIDTeardown();
    hid_setup = false;
    lock_take();
    running = false;
    tb_opened = false;
    for (int i = 0; i < MAX_IFACES; i++) {
        ifs[i].role = R_NONE;
        ifs[i].gen = (ifs[i].gen + 1) & 0x0FFFFFFF;
    }
    lock_give();
    for (int i = 0; i < MAX_IFACES; i++) {
        free(ifs[i].buf);
        free(ifs[i].ctl);
        ifs[i].buf = ifs[i].ctl = NULL;
    }
}

bool TB_Connected(void)
{
    tb_info info;
    return tb_opened || inpw_udraw_info(&info);
}

bool TB_GetInfo(tb_info *info)
{
    tb_info copy;

    if (!tb_opened)
        return inpw_udraw_info(info ? info : &copy);
    if (info) {
        lock_take();
        *info = tb_dev;
        lock_give();
    }
    return true;
}

void TB_GetState(tb_sample *out)
{
    tb_sample ud;

    if (!out)
        return;
    if (running) {
        lock_take();
        *out = tb_state;
        lock_give();
    } else {
        memset(out, 0, sizeof(*out));
        out->wiimote = -1;
    }
    if (inpw_udraw_state(&ud) && (!tb_opened || (int32_t)(ud.time - out->time) > 0))
        *out = ud;
}

bool TB_Pop(tb_sample *out)
{
    tb_sample s;
    bool got = false;

    if (running) {
        lock_take();
        if (tb_tail != tb_head) {
            s = tb_queue[tb_tail];
            tb_tail = (tb_tail + 1) & QUEUE_MASK;
            got = true;
        }
        lock_give();
    }
    if (!got)
        got = inpw_udraw_pop(&s);
    if (got && out)
        *out = s;
    return got;
}

void TB_ToScreen(const tb_sample *s, int w, int h, int *x, int *y)
{
    int32_t max_x = tb_dev.max_x, max_y = tb_dev.max_y;
    float aspect, aw, ah;

    if (s && s->wiimote >= 0)
        inpw_udraw_extent(&max_x, &max_y);
    if (!s || max_x <= 0 || max_y <= 0) {
        if (x) *x = 0;
        if (y) *y = 0;
        return;
    }
    aspect = (float)max_x / (float)max_y;
    aw = (float)w;
    ah = (float)w / aspect;
    if (ah > (float)h) {
        ah = (float)h;
        aw = ah * aspect;
    }
    if (x) *x = (int)(((float)w - aw) / 2 + s->nx * (aw - 1));
    if (y) *y = (int)(((float)h - ah) / 2 + s->ny * (ah - 1));
}

uint32_t TB_GetFingers(tb_finger *out, uint32_t max)
{
    uint32_t n = 0;

    if (!running)
        return 0;
    lock_take();
    for (int i = 0; i < TB_MAX_FINGERS && n < max; i++)
        if (tb_fingers[i].down && out)
            out[n++] = tb_fingers[i];
    lock_give();
    return n;
}

void TB_GetStats(tb_stats *out)
{
    if (!out)
        return;
    lock_take();
    *out = tb_st;
    lock_give();
}

#else /* !HAVE_TABLET */

#include "libinput_internal.h"
bool inpw_tablet_vendor(uint16_t vid) { (void)vid; return false; }
bool inpw_tablet_running(void) { return false; }

#endif /* HAVE_TABLET */
