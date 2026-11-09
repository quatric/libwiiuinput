/* THQ uDraw GameTablet (the Wii's), plugged into a Wii Remote paired with
 * the Wii U.  See input_tablet.h.
 *
 * The system knows nothing of the uDraw: KPAD sees an extension it cannot
 * read.  The extension's own registers still hold its input, as on every
 * Wii Remote extension (the six bytes at 0x00 are what a report carries),
 * so this driver reads them over Bluetooth with WPADReadExtReg: first the
 * ID at 0xFA, once per extension, and then the data, one read in flight at
 * a time, for as long as the uDraw is there.
 *
 * Report layout and decoding are libwiiinput's (WiiBrew, Dolphin's
 * emulation, rvlpaint's notes):
 *   x = b0 | (b2 & 0x0f) << 8, y = b1 | (b2 >> 4) << 8, 0xfff = no pen
 *   pressure = b3 | (b5 & 0x04) << 6 (nine bits)
 *   b5 bit 0 = upper rocker, bit 1 = lower rocker, both 0 when pressed
 *   b4 is always 0xff and the top five bits of b5 are set
 */

#include <string.h>

#include <coreinit/spinlock.h>
#include <padscore/wpad.h>

#include "libinput_internal.h"

#ifdef HAVE_TABLET
#include "input_tablet.h"

#define CHANNELS     4
#define RING         64
#define RING_MASK    (RING - 1)
#define STALE_MS     250   /* no report for this long: the uDraw has gone */
#define RETRY_MS     1000  /* a failed ID read is tried again after this */
#define PRESS_ON     24    /* the nib touches above this and lifts below PRESS_OFF */
#define PRESS_OFF    16
#define MAX_PRESSURE 511

enum { U_NONE, U_ID, U_DATA };

typedef struct {
    uint8_t b[6];
    uint32_t time;
} ud_report;

typedef struct {
    bool touching;
    int32_t x, y;
} ud_track;

static const uint8_t udraw_id[6] = { 0xff, 0x00, 0xa4, 0x20, 0x01, 0x12 };

static OSSpinLock lock;
static bool lock_ready;
static struct {
    volatile int pending;      /* U_ID or U_DATA in flight */
    volatile bool attached;    /* the ID said uDraw */
    volatile bool id_done;     /* the ID has been read for this extension */
    uint32_t id_failed_at;
    uint8_t ext;               /* the extension type it was read for */
    uint8_t id[6];
    uint8_t buf[16] __attribute__((aligned(0x40)));
    ud_report ring[RING];
    uint32_t head, tail;
    ud_report latest;
    uint32_t last;
} ud[CHANNELS];

static ud_track ud_popped[CHANNELS], ud_current[CHANNELS];
static int32_t ud_min_x = 0x56, ud_max_x = 0x780, ud_min_y = 0x65, ud_max_y = 0x5a5;

static void lock_take(void) { OSUninterruptibleSpinLock_Acquire(&lock); }
static void lock_give(void) { OSUninterruptibleSpinLock_Release(&lock); }

/* One read finished. */
static void on_read(WPADChan chan, WPADError status)
{
    if ((int)chan < 0 || chan >= CHANNELS)
        return;
    lock_take();
    if (ud[chan].pending == U_ID) {
        if (status == WPAD_ERROR_NONE) {
            memcpy(ud[chan].id, ud[chan].buf, 6);
            ud[chan].id_done = true;
            ud[chan].attached = memcmp(ud[chan].buf, udraw_id, 6) == 0;
            if (ud[chan].attached) {
                ud[chan].head = ud[chan].tail = 0;
                ud[chan].last = inpw_now_ms();
            }
        } else {
            ud[chan].id_failed_at = inpw_now_ms();
        }
    } else if (ud[chan].pending == U_DATA && status == WPAD_ERROR_NONE && ud[chan].attached) {
        uint32_t next = (ud[chan].head + 1) & RING_MASK;
        uint32_t now = inpw_now_ms();
        if (next == ud[chan].tail)
            ud[chan].tail = (ud[chan].tail + 1) & RING_MASK; /* drop the oldest */
        memcpy(ud[chan].ring[ud[chan].head].b, ud[chan].buf, 6);
        ud[chan].ring[ud[chan].head].time = now;
        ud[chan].head = next;
        memcpy(ud[chan].latest.b, ud[chan].buf, 6);
        ud[chan].latest.time = ud[chan].last = now;
    }
    ud[chan].pending = U_NONE;
    lock_give();
}

/* Extensions the system reads itself; anything else may be a uDraw (the
 * system may also take one for a Classic Controller). */
static bool known_ext(uint8_t ext)
{
    return ext == WPAD_EXT_CORE || ext == WPAD_EXT_NUNCHUK || ext == WPAD_EXT_MPLUS ||
           ext == WPAD_EXT_MPLUS_NUNCHUK || ext == WPAD_EXT_PRO_CONTROLLER || ext == WPAD_EXT_BALANCE_BOARD ||
           ext == WPAD_EXT_DEV_NOT_FOUND;
}

void inpw_udraw_update(void)
{
    if (!lock_ready) {
        OSInitSpinLock(&lock);
        lock_ready = true;
    }
    for (int ch = 0; ch < CHANNELS; ch++) {
        WPADExtensionType ext = WPAD_EXT_CORE;
        int want = U_NONE;

        if (WPADProbe((WPADChan)ch, &ext) != WPAD_ERROR_NONE)
            ext = WPAD_EXT_CORE;

        lock_take();
        if (ext != ud[ch].ext) {
            /* plugged, unplugged or changed: read the ID again */
            ud[ch].ext = (uint8_t)ext;
            ud[ch].id_done = false;
            ud[ch].attached = false;
            ud[ch].id_failed_at = 0;
        }
        if (ud[ch].pending == U_NONE) {
            if (ud[ch].attached)
                want = U_DATA;
            else if (!ud[ch].id_done && !known_ext((uint8_t)ext) &&
                     (!ud[ch].id_failed_at || inpw_now_ms() - ud[ch].id_failed_at >= RETRY_MS))
                want = U_ID;
            ud[ch].pending = want;
        }
        lock_give();

        if (want == U_NONE)
            continue;
        if (WPADReadExtReg((WPADChan)ch, ud[ch].buf, 6, WPAD_PERIPHERAL_SPACE_EXTENSION, want == U_ID ? 0xFA : 0x00,
                           on_read) != WPAD_ERROR_NONE) {
            lock_take();
            ud[ch].pending = U_NONE;
            if (want == U_ID)
                ud[ch].id_failed_at = inpw_now_ms();
            lock_give();
        }
    }
}

static bool fresh(int ch)
{
    return ud[ch].attached && inpw_now_ms() - ud[ch].last <= STALE_MS;
}

bool inpw_udraw_on(int ch)
{
    return ch >= 0 && ch < CHANNELS && ud[ch].attached;
}

void inpw_udraw_debug(int ch, uint8_t *ext, uint8_t id[6], bool *id_read)
{
    if (ch < 0 || ch >= CHANNELS)
        return;
    *ext = ud[ch].ext;
    *id_read = ud[ch].id_done;
    memcpy(id, ud[ch].id, 6);
}

static bool decode(int chan, const ud_report *r, ud_track *t, tb_sample *s)
{
    const uint8_t *b = r->b;
    int32_t rx, ry, w = ud_max_x - ud_min_x, h = ud_max_y - ud_min_y;
    uint32_t p;

    if (b[4] != 0xff || (b[5] & 0xf8) != 0xf8)
        return false;
    memset(s, 0, sizeof(*s));
    s->wiimote = (int8_t)chan;
    s->time = r->time;
    rx = b[0] | (b[2] & 0x0f) << 8;
    ry = b[1] | (b[2] >> 4) << 8;
    p = b[3] | (b[5] & 0x04) << 6;
    s->in_range = rx != 0xfff && ry != 0xfff;
    if (!(b[5] & 0x01))
        s->buttons |= TB_BUTTON_BARREL2;
    if (!(b[5] & 0x02))
        s->buttons |= TB_BUTTON_BARREL;

    if (!s->in_range) {
        t->touching = false;
        s->x = t->x; /* keep the last place, as a USB tablet does */
        s->y = t->y;
    } else {
        t->touching = p >= (uint32_t)(t->touching ? PRESS_OFF : PRESS_ON);
        s->x = rx - ud_min_x;
        s->y = ud_max_y - ry; /* the tablet counts y from the bottom */
        if (s->x < 0) s->x = 0;
        if (s->y < 0) s->y = 0;
        if (s->x > w) s->x = w;
        if (s->y > h) s->y = h;
    }
    if (t->touching)
        s->buttons |= TB_BUTTON_TIP;
    s->raw_pressure = p;
    s->pressure = (float)p / MAX_PRESSURE;
    s->nx = (float)s->x / (float)w;
    s->ny = (float)s->y / (float)h;
    t->x = s->x;
    t->y = s->y;
    return true;
}

bool inpw_udraw_pop(tb_sample *out)
{
    if (!lock_ready)
        return false;
    for (int ch = 0; ch < CHANNELS; ch++) {
        for (;;) {
            ud_report r;
            bool got = false;
            lock_take();
            if (ud[ch].tail != ud[ch].head) {
                r = ud[ch].ring[ud[ch].tail];
                ud[ch].tail = (ud[ch].tail + 1) & RING_MASK;
                got = true;
            }
            lock_give();
            if (!got)
                break;
            if (decode(ch, &r, &ud_popped[ch], out))
                return true;
        }
    }
    return false;
}

bool inpw_udraw_state(tb_sample *out)
{
    int best = -1;
    ud_report r;

    if (!lock_ready)
        return false;
    for (int ch = 0; ch < CHANNELS; ch++)
        if (fresh(ch) && (best < 0 || (int32_t)(ud[ch].last - ud[best].last) > 0))
            best = ch;
    if (best < 0)
        return false;
    lock_take();
    r = ud[best].latest;
    lock_give();
    return decode(best, &r, &ud_current[best], out);
}

bool inpw_udraw_info(tb_info *info)
{
    int ch;

    for (ch = 0; ch < CHANNELS; ch++)
        if (fresh(ch))
            break;
    if (ch == CHANNELS)
        return false;
    memset(info, 0, sizeof(*info));
    strcpy(info->name, "uDraw GameTablet");
    info->max_x = ud_max_x - ud_min_x;
    info->max_y = ud_max_y - ud_min_y;
    info->max_pressure = MAX_PRESSURE;
    return true;
}

void inpw_udraw_extent(int32_t *w, int32_t *h)
{
    *w = ud_max_x - ud_min_x;
    *h = ud_max_y - ud_min_y;
}

bool TB_UDrawConnected(int wiimote)
{
    return wiimote >= 0 && wiimote < CHANNELS && fresh(wiimote);
}

void TB_UDrawSetArea(int32_t min_x, int32_t max_x, int32_t min_y, int32_t max_y)
{
    if (min_x < 0 || min_y < 0 || max_x > 0xffe || max_y > 0xffe || max_x - min_x < 100 || max_y - min_y < 100)
        return;
    ud_min_x = min_x;
    ud_max_x = max_x;
    ud_min_y = min_y;
    ud_max_y = max_y;
}

#else /* !HAVE_TABLET */

void inpw_udraw_update(void) {}
bool inpw_udraw_on(int ch) { (void)ch; return false; }
void inpw_udraw_debug(int ch, uint8_t *ext, uint8_t id[6], bool *id_read)
{
    (void)ch;
    *ext = 0;
    memset(id, 0, 6);
    *id_read = false;
}

#endif /* HAVE_TABLET */
