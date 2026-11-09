/* Wii Speak (P-Mic) for the Wii U. See input_wiispeak.h.
 *
 * libwiiinput's driver, with the transport moved from IOS's /dev/usb/oh0 to
 * the Wii U's USB host stack (libinput_uhs.c). The protocol, the boot order
 * and the settings are unchanged; what IOS made hard (one-byte control reads,
 * transfers without deadlines, guessing packet lengths) the host stack does
 * not, so those workarounds are gone. */

#ifdef HAVE_MIC

#include <stdio.h>
#include <string.h>

#include <coreinit/mutex.h>

#include "input_wiispeak.h"
#include "libinput_uhs.h"

/* ---- vendor protocol (from the RVL SDK's pmic.a) --------------------------
 *
 * Every setting is a 10-byte command written to the device and then read
 * back through the same buffer:
 *
 *     [0..1] opcode  0x9A00 | reg   (bit 0 of reg set means "read")
 *     [2..3] arg1
 *     [4..5] arg2
 *     [6..7] arg3
 *     [8..9] sequence number
 *
 * Reads return their result in arg1, except SP_STATE which answers in arg2.
 */
#define REQTYPE_VENDOR_OUT 0x41
#define REQTYPE_VENDOR_IN  0xC1

#define REQ_RESET   0 /* out, no data: puts the mic back in "not booted" */
#define REQ_CMD_PUT 1 /* out, 10 bytes: submit a command */
#define REQ_CMD_GET 2 /* in,  10 bytes: fetch the response */
#define REQ_STATUS  6 /* in,  1 byte:   0 = awaiting firmware, 1 = booted */

#define CMD_MAGIC 0x9A00

#define REG_SAMPLER_STATE 0x00
#define REG_SAMPLER_FREQ  0x02
#define REG_SAMPLER_GAIN  0x04
#define REG_SAMPLER_MUTE  0x0C
#define REG_EC_STATE      0x14
#define REG_SP_STATE      0x38
#define REG_READ          0x01

#define SP_ENABLE 0x1010
#define SP_SOUT   0x2004

#define CMD_LEN 10

/* Interface 0, alternate setting 1, has all three endpoints. The SDK runs
 * 8 x 32 bytes in (one 8 ms frame of audio) and 8 x 64 bytes out (the echo
 * canceller's reference). */
#define EP_AUDIO_IN  0x81
#define EP_DATA_OUT  0x02
#define EP_AUDIO_OUT 0x03

#define ISO_IN_PKTSIZE  32
#define ISO_IN_MAXPKTS  16
#define ISO_OUT_PACKETS 8
#define ISO_OUT_PKTSIZE 64

#define FIRMWARE_CHUNK 1024

#define RING_SAMPLES (1u << 17) /* ~8 s at 16 kHz */
#define RING_MASK    (RING_SAMPLES - 1)

static uhsx_dev ws_dev;
static bool ws_opened;
static uhsx_stream *ws_in, *ws_out;
static int16_t *ws_ring;
static volatile uint32_t ws_ring_head, ws_ring_tail, ws_drops;
static OSMutex ws_ring_lock, ws_cmd_lock;
static bool ws_locks;
static uint16_t ws_seq;
static bool (*ws_cancel)(void);
static int (*ws_logger)(const char *fmt, ...);

static ws_iso_tap ws_tap;
static ws_stats ws_st;
static volatile int ws_layout = WS_LAYOUT_AUTO;
static volatile bool ws_swap;
static bool ws_out_on = true;
static int ws_in_pkts = 8;
static int ws_boot_wait_ms = 300;
static bool ws_skip_reset;
static volatile int ws_out_signal;

#define ws_log(...) (ws_logger ? (void)ws_logger(__VA_ARGS__) : (void)0)

void WS_SetLog(int (*print)(const char *fmt, ...)) { ws_logger = print; }
void WS_SetCancel(bool (*cancel)(void)) { ws_cancel = cancel; }
void WS_SetStreams(int in_slots, bool out_stream) { (void)in_slots; ws_out_on = out_stream; }
void WS_SetOutSignal(int mode) { ws_out_signal = mode; }
void WS_SetBoot(int wait_ms, bool skip_reset) { ws_boot_wait_ms = wait_ms; ws_skip_reset = skip_reset; }
void WS_SetCallbackMode(bool on) { (void)on; }
void WS_SetIsoTap(ws_iso_tap tap) { ws_tap = tap; }
void WS_SetLayout(int layout) { ws_layout = layout; }
int  WS_GetLayout(void) { return ws_layout; }
void WS_SetSwap(bool swap) { ws_swap = swap; }
bool WS_GetSwap(void) { return ws_swap; }
void WS_GetStats(ws_stats *out) { if (out) *out = ws_st; }

void WS_SetInPackets(int n)
{
    if (n >= 1 && n <= ISO_IN_MAXPKTS)
        ws_in_pkts = n;
}

uint32_t WS_FreqToHz(uint16_t freq)
{
    switch (freq) {
    case WS_FREQ_8KHZ:  return 8000;
    case WS_FREQ_11KHZ: return 11025;
    default:            return 16000;
    }
}

/* ---- command layer ------------------------------------------------------ */

static void put16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)v;
}

static uint16_t get16(const uint8_t *p)
{
    return (uint16_t)((p[0] << 8) | p[1]);
}

static int32_t ws_ctrl(uint8_t type, uint8_t req, uint16_t len, void *data)
{
    return uhsx_ctrl(&ws_dev, type, req, 0, 0, len, data);
}

static int32_t ws_command(uint8_t reg, uint16_t arg1, uint16_t arg2, uint16_t *out1, uint16_t *out2)
{
    uint8_t cmd[CMD_LEN];
    int32_t ret;

    if (!ws_opened)
        return WS_ESTATE;
    OSLockMutex(&ws_cmd_lock);
    memset(cmd, 0, sizeof(cmd));
    put16(cmd + 0, (uint16_t)(CMD_MAGIC | reg));
    put16(cmd + 2, arg1);
    put16(cmd + 4, arg2);
    put16(cmd + 8, ws_seq++);
    ret = ws_ctrl(REQTYPE_VENDOR_OUT, REQ_CMD_PUT, CMD_LEN, cmd);
    /* The SDK fetches the answer to every command, sets included: the mic
     * may not take the next one until it has been. */
    if (ret >= 0)
        ret = ws_ctrl(REQTYPE_VENDOR_IN, REQ_CMD_GET, CMD_LEN, cmd);
    if (ret >= 0) {
        if (out1)
            *out1 = get16(cmd + 2);
        if (out2)
            *out2 = get16(cmd + 4);
        ret = WS_OK;
    }
    OSUnlockMutex(&ws_cmd_lock);
    return ret;
}

static int32_t ws_set(uint8_t reg, uint16_t arg1, uint16_t arg2)
{
    return ws_command(reg, arg1, arg2, NULL, NULL);
}

int32_t WS_RawCommand(uint8_t reg, uint16_t arg1, uint16_t arg2, uint16_t *out1, uint16_t *out2)
{
    return ws_command(reg, arg1, arg2, out1, out2);
}

static int32_t ws_status(void)
{
    uint8_t b[2] = { 0xA5, 0xA5 };
    int32_t ret = ws_ctrl(REQTYPE_VENDOR_IN, REQ_STATUS, 1, b);
    return ret < 0 ? ret : b[0];
}

/* ---- streams -------------------------------------------------------------- */

static void ws_push(const int16_t *samples, uint32_t count)
{
    OSLockMutex(&ws_ring_lock);
    for (uint32_t i = 0; i < count; i++) {
        uint32_t next = (ws_ring_head + 1) & RING_MASK;
        if (next == ws_ring_tail) {
            ws_drops += count - i;
            break;
        }
        ws_ring[ws_ring_head] = samples[i];
        ws_ring_head = next;
    }
    OSUnlockMutex(&ws_ring_lock);
}

/* One IN transfer: each packet's audio sits at the front of its 32-byte
 * slot (a real Wii Speak sends 16 of the 32 bytes asked for), big-endian. */
static void ws_on_in(uint8_t *buf, const uint16_t *got, uint32_t count, uint32_t size, int32_t ret, void *ctx)
{
    static int16_t samples[ISO_IN_MAXPKTS * ISO_IN_PKTSIZE / 2];
    uint8_t *dst = (uint8_t *)samples;
    uint32_t bytes = 0;

    (void)ctx;
    ws_st.in_calls++;
    ws_st.last_in_ret = ret;
    ws_st.last_layout = WS_LAYOUT_SLOTS;
    for (uint32_t i = 0; i < count && i < WS_ISO_PACKETS; i++)
        ws_st.last_sizes[i] = got[i];
    if (ws_tap)
        ws_tap(ret, got, buf);
    if (ret < 0) {
        ws_st.in_errors++;
        return;
    }
    if (ws_layout == WS_LAYOUT_PACKED) {
        bytes = (uint32_t)ret;
        memcpy(dst, buf, bytes);
    } else {
        for (uint32_t i = 0; i < count; i++) {
            memcpy(dst + bytes, buf + i * size, got[i]);
            bytes += got[i];
        }
    }
    bytes &= ~1u;
    if (ws_swap)
        for (uint32_t i = 0; i < bytes; i += 2) {
            uint8_t t = dst[i];
            dst[i] = dst[i + 1];
            dst[i + 1] = t;
        }
    ws_st.in_samples += bytes / 2;
    if (bytes)
        ws_push(samples, bytes / 2);
}

/* The reference stream: silence, or a 1 kHz test tone at 32 kHz. */
static void ws_on_out(uint8_t *buf, const uint16_t *got, uint32_t count, uint32_t size, int32_t ret, void *ctx)
{
    static const int16_t tone[32] = {
        0, 1560, 3060, 4440, 5650, 6640, 7370, 7820, 7980, 7820, 7370, 6640, 5650, 4440, 3060, 1560,
        0, -1560, -3060, -4440, -5650, -6640, -7370, -7820, -7980, -7820, -7370, -6640, -5650, -4440, -3060, -1560
    };
    int16_t *s = (int16_t *)buf;

    (void)got;
    (void)ret;
    (void)ctx;
    ws_st.out_calls++;
    for (uint32_t i = 0; i < count && i < WS_ISO_PACKETS; i++)
        ws_st.last_out_sizes[i] = (uint16_t)size;
    for (uint32_t i = 0; i < count * size / 2; i++)
        s[i] = ws_out_signal ? tone[i & 31] : 0;
}

static int32_t ws_start_streams(void)
{
    if (!ws_in)
        ws_in = uhsx_stream_start(&ws_dev, EP_AUDIO_IN, (uint32_t)ws_in_pkts, ISO_IN_PKTSIZE, ws_on_in, NULL);
    if (!ws_in)
        return WS_ENOMEM;
    /* The device expects the reference to keep coming while it samples. */
    if (ws_out_on && !ws_out)
        ws_out = uhsx_stream_start(&ws_dev, EP_AUDIO_OUT, ISO_OUT_PACKETS, ISO_OUT_PKTSIZE, ws_on_out, NULL);
    return WS_OK;
}

static void ws_stop_streams(void)
{
    uhsx_stream_stop(ws_in);
    uhsx_stream_stop(ws_out);
    ws_in = ws_out = NULL;
}

/* ---- boot -------------------------------------------------------------- */

static int32_t ws_select_alt1(const char *when)
{
    int32_t ret = uhsx_set_alt(&ws_dev, 0, 1);
    ws_log("  set alt 1 (%s): %d\n", when, (int)ret);
    return ret;
}

/* The mic boots from RAM: it comes up bare and stays mute until the host
 * pushes PMIC_FIRM_DATA at it in 1 KB bulk chunks. */
static int32_t ws_upload_firmware(const uint8_t *firmware, uint32_t size)
{
    uint32_t sent = 0;
    int32_t ret;

    if (!ws_skip_reset) {
        ret = ws_ctrl(REQTYPE_VENDOR_OUT, REQ_RESET, 0, NULL);
        ws_log("  reset: %d\n", (int)ret);
    }
    uhsx_sleep_ms(50);
    ws_log("  status before upload: %d\n", (int)ws_status());

    ws_log("  uploading %u bytes...\n", (unsigned)size);
    while (sent < size) {
        uint32_t n = size - sent > FIRMWARE_CHUNK ? FIRMWARE_CHUNK : size - sent;
        ret = uhsx_bulk_out(&ws_dev, EP_DATA_OUT, firmware + sent, n);
        if (ret < 0) {
            ws_log("  bulk write failed at %u/%u (%d)\n", (unsigned)sent, (unsigned)size, (int)ret);
            return ret;
        }
        sent += n;
    }
    ws_log("  uploaded; waiting %d ms\n", ws_boot_wait_ms);
    uhsx_sleep_ms((uint32_t)ws_boot_wait_ms);
    ws_log("  status after upload: %d\n", (int)ws_status());

    /* As the SDK: the streams first, and only then commands. */
    ret = ws_start_streams();
    if (ret < 0)
        return ret;
    for (int ms = 0; ms < 15000; ms += 100) {
        ret = ws_set(REG_SAMPLER_FREQ, WS_FREQ_16KHZ, 0);
        if (ret >= 0) {
            ws_log("  ready after %d ms\n", ms);
            return WS_OK;
        }
        if (ws_cancel && ws_cancel())
            return WS_ECANCEL;
        if (ms == 3000)
            ws_select_alt1("retry");
        if (ms % 2000 == 1900)
            ws_log("  waiting for the mic... (%d)\n", (int)ret);
        uhsx_sleep_ms(100);
    }
    ws_log("  the mic never took a command (%d)\n", (int)ret);
    return WS_OK; /* carried on regardless, so it can still be poked at */
}

void WS_Diagnose(void)
{
    if (uhsx_init(ws_logger) < 0) {
        ws_log("Wii Speak: the USB host stack would not open\n");
        return;
    }
    uhsx_list(ws_logger);
    uhsx_shutdown();
}

int32_t WS_Open(const void *firmware, uint32_t firmware_size)
{
    int32_t ret;

    if (ws_opened)
        return WS_ESTATE;
    if (!firmware || firmware_size == 0)
        return WS_EINVAL;
    if (!ws_locks) {
        OSInitMutex(&ws_ring_lock);
        OSInitMutex(&ws_cmd_lock);
        ws_locks = true;
    }
    if (uhsx_init(ws_logger) < 0)
        return WS_ENODEV;
    if (uhsx_open(&ws_dev, WS_VID, WS_PID, 0xFF) < 0) {
        ws_log("Wii Speak: not found\n");
        uhsx_shutdown();
        return WS_ENODEV;
    }
    ws_ring = uhsx_alloc(RING_SAMPLES * sizeof(int16_t));
    if (!ws_ring) {
        uhsx_close(&ws_dev);
        uhsx_shutdown();
        return WS_ENOMEM;
    }
    memset(&ws_st, 0, sizeof(ws_st));
    ws_ring_head = ws_ring_tail = ws_drops = 0;
    ws_opened = true;

    /* Alt 1 before anything else: alt 0 has no endpoints, so neither the
     * firmware nor the vendor requests would go anywhere. */
    ret = ws_select_alt1("open");
    if (ret < 0)
        goto fail;
    ret = ws_upload_firmware(firmware, firmware_size);
    if (ret < 0)
        goto fail;
    return WS_OK;

fail:
    WS_Close();
    return ret;
}

int32_t WS_Close(void)
{
    if (!ws_opened)
        return WS_ESTATE;
    WS_Stop();
    ws_stop_streams();
    ws_log("  ws: reset %d\n", (int)ws_ctrl(REQTYPE_VENDOR_OUT, REQ_RESET, 0, NULL));
    uhsx_set_alt(&ws_dev, 0, 0);
    uhsx_close(&ws_dev);
    uhsx_shutdown();
    uhsx_free(ws_ring);
    ws_ring = NULL;
    ws_opened = false;
    return WS_OK;
}

void WS_Quiesce(void)
{
    ws_stop_streams();
}

/* ---- settings ---------------------------------------------------------- */

int32_t WS_SetFreq(uint16_t freq)
{
    if (freq != WS_FREQ_8KHZ && freq != WS_FREQ_11KHZ && freq != WS_FREQ_16KHZ)
        return WS_EINVAL;
    return ws_set(REG_SAMPLER_FREQ, freq, 0);
}

int32_t WS_GetFreq(uint16_t *freq)
{
    if (!freq)
        return WS_EINVAL;
    return ws_command(REG_SAMPLER_FREQ | REG_READ, 0, 0, freq, NULL);
}

int32_t WS_SetGain(uint16_t gain)
{
    if (gain > WS_GAIN_36dB)
        return WS_EINVAL;
    return ws_set(REG_SAMPLER_GAIN, (uint16_t)(0x0300 | gain), 0); /* as PMICSetGain */
}

int32_t WS_SetMute(bool mute) { return ws_set(REG_SAMPLER_MUTE, mute ? 1 : 0, 0); }
int32_t WS_SetEcReset(bool reset) { return ws_set(REG_EC_STATE, reset ? 1 : 0, 0); }

int32_t WS_SetSignalProcessing(bool enable)
{
    return ws_set(REG_SP_STATE, SP_ENABLE, enable ? 0 : 1); /* arg2 0 turns it on */
}

int32_t WS_GetSendLevel(uint16_t *level)
{
    if (!level)
        return WS_EINVAL;
    return ws_command(REG_SP_STATE | REG_READ, SP_SOUT, 0, NULL, level);
}

/* ---- streaming ---------------------------------------------------------- */

int32_t WS_Start(void)
{
    int32_t ret = WS_ESTATE;

    if (!ws_opened)
        return WS_ESTATE;
    if (ws_start_streams() < 0)
        return WS_ENOMEM;
    /* Sampler on, then unmute: PMICStart's two commands, in its order. */
    for (int tries = 0; tries < 20; tries++) {
        ret = ws_set(REG_SAMPLER_STATE, 1, 0);
        if (ret >= 0)
            break;
        uhsx_sleep_ms(100);
    }
    ws_log("  sampler on: %d\n", (int)ret);
    ws_log("  mute off: %d\n", (int)ws_set(REG_SAMPLER_MUTE, 0, 0));
    return WS_OK;
}

int32_t WS_Stop(void)
{
    if (!ws_opened)
        return WS_ESTATE;
    /* While the streams still run: commands fail without them. The streams
     * themselves run until WS_Close, as the SDK keeps them. */
    ws_log("  ws: sampler off %d\n", (int)ws_set(REG_SAMPLER_STATE, 0, 0));
    return WS_OK;
}

uint32_t WS_Read(int16_t *dst, uint32_t max_samples)
{
    uint32_t taken = 0;

    if (!dst || !ws_ring)
        return 0;
    OSLockMutex(&ws_ring_lock);
    while (taken < max_samples && ws_ring_tail != ws_ring_head) {
        dst[taken++] = ws_ring[ws_ring_tail];
        ws_ring_tail = (ws_ring_tail + 1) & RING_MASK;
    }
    OSUnlockMutex(&ws_ring_lock);
    return taken;
}

uint32_t WS_Available(void)
{
    return (ws_ring_head - ws_ring_tail) & RING_MASK;
}

uint32_t WS_Dropped(void)
{
    return ws_drops;
}

#endif /* HAVE_MIC */
