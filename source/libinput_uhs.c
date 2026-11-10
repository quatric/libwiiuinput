/* Raw USB devices through nsysuhs. See libinput_uhs.h. */

#ifdef HAVE_MIC

#include <malloc.h>
#include <stdio.h>
#include <string.h>

#include <coreinit/cache.h>
#include <coreinit/memdefaultheap.h>
#include <coreinit/mutex.h>
#include <coreinit/thread.h>
#include <coreinit/time.h>

#include "libinput_uhs.h"

/* usb_mic.rpl passes 50000 for its control transfers: microseconds, it
 * would seem. Generous here, since the Wii Speak goes quiet for a moment
 * after taking its firmware. */
#define CTRL_TIMEOUT 500000
#define BULK_TIMEOUT 1000000
#define EP_PENDING 8
#define EP_MAX_BYTES 6400
#define MAX_PROFILES 16

static UhsHandle uhs __attribute__((aligned(0x40)));
static UhsConfig uhs_cfg __attribute__((aligned(0x40)));
static void *uhs_buf;
static int uhs_users;
static OSMutex uhs_lock;      /* control bounce buffer, frame bookkeeping */
static uint8_t *bounce;       /* control and bulk data, 0x40 aligned */
static UhsInterfaceProfile *profiles;
static int (*ulog)(const char *fmt, ...);

#define LOG(...) do { if (ulog) ulog(__VA_ARGS__); } while (0)

void *uhsx_alloc(uint32_t size)
{
    return MEMAllocFromDefaultHeapEx((size + 0x3F) & ~0x3Fu, 0x40);
}

void uhsx_free(void *p)
{
    if (p)
        MEMFreeToDefaultHeap(p);
}

uint32_t uhsx_ms(void)
{
    return (uint32_t)OSTicksToMilliseconds(OSGetSystemTime());
}

void uhsx_sleep_ms(uint32_t ms)
{
    OSSleepTicks(OSMillisecondsToTicks(ms));
}

int32_t uhsx_init(int (*log)(const char *fmt, ...))
{
    if (log)
        ulog = log;
    if (uhs_users++)
        return 0;
    OSInitMutex(&uhs_lock);
    uhs_buf = uhsx_alloc(UHS_CONFIG_BUFFER_SIZE);
    bounce = uhsx_alloc(UHSX_CFG_MAX + 64);
    profiles = uhsx_alloc(sizeof(UhsInterfaceProfile) * MAX_PROFILES);
    if (!uhs_buf || !bounce || !profiles) {
        uhsx_shutdown();
        return -1;
    }
    memset(&uhs, 0, sizeof(uhs));
    memset(&uhs_cfg, 0, sizeof(uhs_cfg));
    uhs_cfg.controller_num = 0; /* the console's own ports */
    uhs_cfg.buffer = uhs_buf;
    uhs_cfg.buffer_size = UHS_CONFIG_BUFFER_SIZE;
    UHSStatus st = UhsClientOpen(&uhs, &uhs_cfg);
    if (st != UHS_STATUS_OK) {
        LOG("usb: the host stack would not open (%08x)\n", (unsigned)st);
        uhs_users = 1;
        uhsx_shutdown();
        return -1;
    }
    return 0;
}

void uhsx_shutdown(void)
{
    if (uhs_users <= 0)
        return;
    if (--uhs_users)
        return;
    if (uhs.state == UHS_HANDLE_STATE_OPENED)
        UhsClientClose(&uhs);
    uhsx_free(uhs_buf);
    uhsx_free(bounce);
    uhsx_free(profiles);
    uhs_buf = NULL;
    bounce = NULL;
    profiles = NULL;
}

static int query(UhsInterfaceFilter *f)
{
    int n;

    memset(profiles, 0, sizeof(UhsInterfaceProfile) * MAX_PROFILES);
    n = (int)UhsQueryInterfaces(&uhs, f, profiles, MAX_PROFILES);
    return n < 0 ? (int)n : n > MAX_PROFILES ? MAX_PROFILES : n;
}

void uhsx_list(int (*log)(const char *fmt, ...))
{
    UhsInterfaceFilter f;
    int n;

    if (!log || uhs_users <= 0)
        return;
    memset(&f, 0, sizeof(f));
    f.match_params = MATCH_ANY;
    n = query(&f);
    log("usb: %d interface(s) offered%s\n", n, n < 0 ? " (query failed)" : "");
    for (int i = 0; i < n; i++) {
        const UhsInterfaceProfile *p = &profiles[i];
        log("  %04x:%04x if %u class %02x/%02x/%02x, %u endpoint(s)\n", p->dev_desc.idVendor,
            p->dev_desc.idProduct, p->if_desc.bInterfaceNumber, p->if_desc.bInterfaceClass,
            p->if_desc.bInterfaceSubClass, p->if_desc.bInterfaceProtocol, p->if_desc.bNumEndpoints);
    }
}

static void lost(void *ctx, int32_t a, int32_t b)
{
    (void)ctx;
    (void)a;
    (void)b;
}

static uint16_t le16(const uint8_t *p)
{
    return (uint16_t)(p[0] | p[1] << 8);
}

int32_t uhsx_open(uhsx_dev *d, uint16_t vid, uint16_t pid, uint8_t if_class)
{
    UhsInterfaceFilter f;
    int n;
    uint16_t dv, dp;

    memset(d, 0, sizeof(*d));
    if (uhs_users <= 0)
        return -1;
    memset(&f, 0, sizeof(f));
    if (vid) {
        f.match_params = MATCH_DEV_VID | MATCH_DEV_PID;
        f.vid = vid;
        f.pid = pid;
    } else if (if_class != 0xFF) {
        f.match_params = MATCH_IF_CLASS;
        f.if_class = if_class;
    } else {
        f.match_params = MATCH_ANY;
    }
    n = query(&f);
    if (n <= 0)
        return -1;
    /* The interfaces of the first device found. */
    dv = profiles[0].dev_desc.idVendor;
    dp = profiles[0].dev_desc.idProduct;
    for (int i = 0; i < n && d->nif < UHSX_MAX_IFACES; i++) {
        const UhsInterfaceProfile *p = &profiles[i];
        if (p->dev_desc.idVendor != dv || p->dev_desc.idProduct != dp)
            continue;
        UHSStatus st = UhsAcquireInterface(&uhs, p->if_handle, NULL, lost);
        if (st != UHS_STATUS_OK) {
            LOG("usb: interface %u could not be acquired (%08x)\n", p->if_desc.bInterfaceNumber, (unsigned)st);
            continue;
        }
        d->if_handle[d->nif] = p->if_handle;
        d->if_number[d->nif] = p->if_desc.bInterfaceNumber;
        d->if_class[d->nif] = p->if_desc.bInterfaceClass;
        d->if_subclass[d->nif] = p->if_desc.bInterfaceSubClass;
        d->nif++;
    }
    if (!d->nif)
        return -1;
    d->open = true;

    /* The descriptors as the device sends them, little-endian. */
    {
        uint8_t dev[18];
        if (uhsx_ctrl(d, 0x80, 6, 0x0100, 0, 18, dev) >= 0 && dev[1] == 1) {
            d->vid = le16(dev + 8);
            d->pid = le16(dev + 10);
            d->iproduct = dev[15];
        } else {
            d->vid = vid ? vid : dv;
            d->pid = vid ? pid : dp;
        }
    }
    d->cfg = uhsx_alloc(UHSX_CFG_MAX);
    if (d->cfg) {
        memset(d->cfg, 0, UHSX_CFG_MAX);
        DCFlushRange(d->cfg, UHSX_CFG_MAX);
        UHSStatus st = UhsGetFullConfigDescriptor(&uhs, d->if_handle[0], d->cfg, UHSX_CFG_MAX);
        DCInvalidateRange(d->cfg, UHSX_CFG_MAX);
        if ((int32_t)st >= 0 && d->cfg[1] == 2)
            d->cfg_len = le16(d->cfg + 2) > UHSX_CFG_MAX ? UHSX_CFG_MAX : le16(d->cfg + 2);
        if (!d->cfg_len && uhsx_ctrl(d, 0x80, 6, 0x0200, 0, 9, d->cfg) >= 9) {
            /* by hand: the header, then the whole */
            uint16_t total = le16(d->cfg + 2);
            if (total > UHSX_CFG_MAX)
                total = UHSX_CFG_MAX;
            if (uhsx_ctrl(d, 0x80, 6, 0x0200, 0, total, d->cfg) > 0)
                d->cfg_len = total;
        }
    }
    LOG("usb: %04x:%04x open, %d interface(s), configuration %u bytes\n", d->vid, d->pid, d->nif,
        (unsigned)d->cfg_len);
    return 0;
}

void uhsx_close(uhsx_dev *d)
{
    if (!d->open)
        return;
    for (int i = 0; i < d->nif; i++) {
        if (d->ep_enabled[i])
            UhsAdministerEndpoint(&uhs, d->if_handle[i], UHS_ADMIN_EP_DISABLE, d->ep_enabled[i], EP_PENDING,
                                  EP_MAX_BYTES);
        UhsReleaseInterface(&uhs, d->if_handle[i], false);
    }
    uhsx_free(d->cfg);
    memset(d, 0, sizeof(*d));
}

static int iface_index(uhsx_dev *d, uint8_t iface)
{
    for (int i = 0; i < d->nif; i++)
        if (d->if_number[i] == iface)
            return i;
    return -1;
}

/* The interface whose descriptors carry endpoint `ep`. */
static int ep_index(uhsx_dev *d, uint8_t ep)
{
    int cur = -1;

    for (uint32_t off = 0; d->cfg && off + 2 <= d->cfg_len && d->cfg[off] >= 2; off += d->cfg[off]) {
        const uint8_t *p = d->cfg + off;
        if (p[1] == 4 && p[0] >= 9)
            cur = p[2];
        else if (p[1] == 5 && p[0] >= 7 && p[2] == ep && cur >= 0)
            return iface_index(d, (uint8_t)cur);
    }
    return d->nif ? 0 : -1;
}

int32_t uhsx_ctrl(uhsx_dev *d, uint8_t type, uint8_t req, uint16_t val, uint16_t idx, uint16_t len, void *data)
{
    int32_t ret;

    if (!d->nif || len > UHSX_CFG_MAX)
        return -1;
    OSLockMutex(&uhs_lock);
    memset(bounce, 0, (len + 0x3F) & ~0x3Fu);
    if (!(type & 0x80) && len && data)
        memcpy(bounce, data, len);
    DCFlushRange(bounce, (len + 0x3F) & ~0x3Fu);
    ret = (int32_t)UhsSubmitControlRequest(&uhs, d->if_handle[0], bounce, req, type, val, idx, len, CTRL_TIMEOUT);
    if ((type & 0x80) && len && data) {
        DCInvalidateRange(bounce, (len + 0x3F) & ~0x3Fu);
        if (ret >= 0)
            memcpy(data, bounce, len);
    }
    OSUnlockMutex(&uhs_lock);
    return ret;
}

int32_t uhsx_endpoint(uhsx_dev *d, uint8_t iface, uint8_t ep, bool on)
{
    int i = iface_index(d, iface);
    uint32_t mask = (ep & 0x80) ? 1u << ((ep & 0x0F) + 16) : 1u << (ep & 0x0F);
    int32_t ret;

    if (i < 0)
        return -1;
    if (on == !!(d->ep_enabled[i] & mask))
        return 0;
    ret = (int32_t)UhsAdministerEndpoint(&uhs, d->if_handle[i], on ? UHS_ADMIN_EP_ENABLE : UHS_ADMIN_EP_DISABLE,
                                         mask, EP_PENDING, EP_MAX_BYTES);
    if (ret >= 0) {
        if (on)
            d->ep_enabled[i] |= mask;
        else
            d->ep_enabled[i] &= ~mask;
    }
    return ret;
}

int32_t uhsx_bulk_out(uhsx_dev *d, uint8_t ep, const void *data, uint32_t len)
{
    int i = ep_index(d, ep);
    int32_t ret;

    if (i < 0 || len > UHSX_CFG_MAX)
        return -1;
    uhsx_endpoint(d, d->if_number[i], ep, true);
    OSLockMutex(&uhs_lock);
    memcpy(bounce, data, len);
    DCFlushRange(bounce, (len + 0x3F) & ~0x3Fu);
    ret = (int32_t)UhsSubmitBulkRequest(&uhs, d->if_handle[i], ep & 0x0F, ENDPOINT_TRANSFER_OUT, bounce, (int32_t)len,
                                        BULK_TIMEOUT);
    OSUnlockMutex(&uhs_lock);
    return ret;
}

int32_t uhsx_set_alt(uhsx_dev *d, uint8_t iface, uint8_t alt)
{
    int i = iface_index(d, iface);
    int32_t ret;

    if (i < 0)
        return -1;
    if (d->ep_enabled[i]) {
        UhsAdministerEndpoint(&uhs, d->if_handle[i], UHS_ADMIN_EP_DISABLE, d->ep_enabled[i], EP_PENDING,
                              EP_MAX_BYTES);
        d->ep_enabled[i] = 0;
    }
    ret = (int32_t)UhsSetInterface(&uhs, d->if_handle[i], alt);
    if (ret < 0) /* the standard request, by hand */
        ret = uhsx_ctrl(d, 0x01, 11, alt, iface, 0, NULL);
    for (int k = 0; k < 32; k++)
        d->frame_set[k] = false;
    return ret;
}

/* The frame a transfer of `count` packets starts at: following on from the
 * last one on this endpoint, or eight frames ahead when that has passed. */
static uint32_t next_frame(uhsx_dev *d, int i, uint8_t ep, uint32_t count)
{
    int key = (ep & 0x0F) + ((ep & 0x80) ? 16 : 0);
    uint32_t cur = 0, f;
    bool have = UhsGetCurrentFrame(&uhs, d->if_handle[i], &cur) == UHS_STATUS_OK;

    OSLockMutex(&uhs_lock);
    if (have) {
        int16_t ahead = (int16_t)(uint16_t)(d->next_frame[key] - cur);
        if (!d->frame_set[key] || ahead < 1 || ahead > 128) {
            d->next_frame[key] = (cur + 8) & 0xFFFF;
            d->frame_set[key] = true;
        }
    } else if (!d->frame_set[key]) {
        d->next_frame[key] = 0;
        d->frame_set[key] = true;
    }
    f = d->next_frame[key];
    d->next_frame[key] = (f + count) & 0xFFFF;
    OSUnlockMutex(&uhs_lock);
    return f;
}

static int32_t isoc_at(uhsx_dev *d, int i, uint8_t ep, uint32_t frame, uint32_t count, uint32_t size, void *buf,
                       uint16_t *got)
{
    UhsxIsocRequest rq __attribute__((aligned(0x40)));
    bool in = (ep & 0x80) != 0;
    int32_t ret, bytes = 0;

    memset(&rq, 0, sizeof(rq));
    rq.count = count;
    rq.buffer = buf;
    rq.length = count * size;
    for (uint32_t k = 0; k < count; k++)
        rq.packet[k].length = size;
    DCFlushRange(buf, rq.length);
    ret = (int32_t)UhsSubmitIsocRequest(&uhs, d->if_handle[i], ep & 0x0F,
                                        in ? ENDPOINT_TRANSFER_IN : ENDPOINT_TRANSFER_OUT,
                                        frame, 0, &rq, TIMEOUT_NONE);
    if (in)
        DCInvalidateRange(buf, rq.length);
    for (uint32_t k = 0; k < count; k++) {
        bool ok = rq.packet[k].status == 0 || rq.packet[k].status == UHSX_ISOC_SHORT;
        uint32_t n = ok ? rq.packet[k].actual : 0;
        if (n > size)
            n = size;
        if (got)
            got[k] = (uint16_t)n;
        bytes += (int32_t)n;
    }
    return ret < 0 ? ret : bytes;
}

int32_t uhsx_isoc(uhsx_dev *d, uint8_t ep, uint32_t count, uint32_t size, void *buf, uint16_t *got)
{
    int i = ep_index(d, ep);

    if (i < 0 || !count || count > UHSX_ISOC_MAX)
        return -1;
    uhsx_endpoint(d, d->if_number[i], ep, true);
    return isoc_at(d, i, ep, next_frame(d, i, ep, count), count, size, buf, got);
}

/* ---- streams ------------------------------------------------------------ */

#define STREAM_STACK 0x4000

struct uhsx_stream {
    uhsx_dev *d;
    int iface;
    uint8_t ep;
    uint32_t count, size;
    uhsx_stream_cb cb;
    void *ctx;
    volatile bool running;
    OSMutex lock;
    uint32_t tickets;
    volatile uint32_t serving;
    struct {
        OSThread thread __attribute__((aligned(16)));
        uint8_t *stack;
        uint8_t *buf;
        uint16_t got[UHSX_ISOC_MAX];
        bool started;
    } w[UHSX_STREAM_SLOTS];
};

static int stream_worker(int argc, const char **argv)
{
    uhsx_stream *st = (uhsx_stream *)argv;
    int k = argc;
    bool in = (st->ep & 0x80) != 0;

    while (st->running) {
        uint32_t frame, ticket;
        int32_t ret;

        OSLockMutex(&st->lock);
        ticket = st->tickets++;
        frame = next_frame(st->d, st->iface, st->ep, st->count);
        OSUnlockMutex(&st->lock);
        if (!in) {
            /* Filled in frame order, as it is sent. */
            while (st->serving != ticket && st->running)
                OSSleepTicks(OSMicrosecondsToTicks(200));
            st->cb(st->w[k].buf, NULL, st->count, st->size, 0, st->ctx);
            st->serving = ticket + 1;
        }
        ret = isoc_at(st->d, st->iface, st->ep, frame, st->count, st->size, st->w[k].buf, st->w[k].got);
        if (in) {
            while (st->serving != ticket && st->running)
                OSSleepTicks(OSMicrosecondsToTicks(200));
            st->cb(st->w[k].buf, st->w[k].got, st->count, st->size, ret, st->ctx);
            st->serving = ticket + 1;
        }
        if (ret < 0) /* failures can come back at once: give the rest a turn */
            OSSleepTicks(OSMillisecondsToTicks(2));
    }
    return 0;
}

uhsx_stream *uhsx_stream_start(uhsx_dev *d, uint8_t ep, uint32_t count, uint32_t size, uhsx_stream_cb cb, void *ctx)
{
    uhsx_stream *st;
    int i = ep_index(d, ep);

    if (i < 0 || !count || count > UHSX_ISOC_MAX || !cb)
        return NULL;
    st = uhsx_alloc(sizeof(*st));
    if (!st)
        return NULL;
    memset(st, 0, sizeof(*st));
    st->d = d;
    st->iface = i;
    st->ep = ep;
    st->count = count;
    st->size = size;
    st->cb = cb;
    st->ctx = ctx;
    st->running = true;
    OSInitMutex(&st->lock);
    uhsx_endpoint(d, d->if_number[i], ep, true);
    for (int k = 0; k < UHSX_STREAM_SLOTS; k++) {
        st->w[k].stack = uhsx_alloc(STREAM_STACK);
        st->w[k].buf = uhsx_alloc(count * size);
        if (!st->w[k].stack || !st->w[k].buf) {
            uhsx_stream_stop(st);
            return NULL;
        }
        memset(st->w[k].buf, 0, count * size);
        /* Above the main thread (16): a late transfer is a lost one. */
        if (!OSCreateThread(&st->w[k].thread, (OSThreadEntryPointFn)stream_worker, k, (char *)st, st->w[k].stack + STREAM_STACK,
                            STREAM_STACK, 12, OS_THREAD_ATTRIB_AFFINITY_ANY)) {
            uhsx_stream_stop(st);
            return NULL;
        }
        OSSetThreadName(&st->w[k].thread, "libwiiuinput usb stream");
        st->w[k].started = true;
        OSResumeThread(&st->w[k].thread);
    }
    return st;
}

void uhsx_stream_stop(uhsx_stream *st)
{
    if (!st)
        return;
    st->running = false;
    for (int k = 0; k < UHSX_STREAM_SLOTS; k++) {
        if (st->w[k].started) {
            int r;
            OSJoinThread(&st->w[k].thread, &r);
        }
        uhsx_free(st->w[k].stack);
        uhsx_free(st->w[k].buf);
    }
    uhsx_free(st);
}

void uhsx_string(uhsx_dev *d, uint8_t index, char *out, uint32_t size)
{
    uint8_t s[64];
    uint32_t n = 0;

    out[0] = 0;
    if (!index || !size || uhsx_ctrl(d, 0x80, 6, (uint16_t)(0x0300 | index), 0x0409, sizeof(s), s) < 2 || s[1] != 3)
        return;
    for (uint32_t i = 2; i + 1 < s[0] && i + 1 < sizeof(s) && n + 1 < size; i += 2)
        out[n++] = (s[i + 1] || s[i] < 0x20 || s[i] > 0x7E) ? '?' : (char)s[i];
    out[n] = 0;
}

#endif /* HAVE_MIC */
