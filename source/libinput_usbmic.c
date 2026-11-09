/* USB Audio Class microphones for the Wii U. See input_usbmic.h.
 *
 * libwiiinput's driver: its descriptor parsing (which format to stream,
 * which feature unit holds volume and mute) unchanged, the transport moved
 * from IOS's /dev/usb/oh0 to the Wii U's USB host stack (libinput_uhs.c),
 * which gives every packet's length, so no guessing them from poison. */

#ifdef HAVE_MIC

#include <stdio.h>
#include <string.h>

#include <coreinit/mutex.h>

#include "input_usbmic.h"
#include "libinput_uhs.h"

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef int16_t s16;
typedef int32_t s32;

#define USB_DT_INTERFACE                0x04
#define USB_DT_ENDPOINT                 0x05
#define USB_DT_CLASS_SPECIFIC_INTERFACE 0x24
#define USB_DT_CLASS_SPECIFIC_ENDPOINT  0x25

/* ---- USB Audio Class 1.0 --------------------------------------------------
 *
 * A class-compliant mic has an Audio Control interface (subclass 1) that
 * describes the signal path as terminals and units, and an Audio Streaming
 * interface (subclass 2) whose non-zero alternate settings each offer one
 * format on an isochronous IN endpoint.  Alt 0 has no endpoint at all, so
 * the device uses no bandwidth until the host picks a format.
 */
#define USB_CLASS_AUDIO      0x01
#define AUDIO_SUBCLASS_CTRL  0x01
#define AUDIO_SUBCLASS_STRM  0x02

/* Class-specific descriptor subtypes */
#define AC_INPUT_TERMINAL    0x02
#define AC_OUTPUT_TERMINAL   0x03
#define AC_MIXER_UNIT        0x04
#define AC_SELECTOR_UNIT     0x05
#define AC_FEATURE_UNIT      0x06
#define AC_PROCESSING_UNIT   0x07
#define AC_EXTENSION_UNIT    0x08
#define AS_GENERAL           0x01
#define AS_FORMAT_TYPE       0x02
#define EP_GENERAL           0x01

#define FORMAT_PCM           0x0001
#define FORMAT_TYPE_I        0x01

/* Class requests */
#define REQ_SET_CUR          0x01
#define REQ_GET_CUR          0x81
#define REQ_GET_MIN          0x82
#define REQ_GET_MAX          0x83

#define REQTYPE_CLASS_IF_OUT 0x21
#define REQTYPE_CLASS_IF_IN  0xA1
#define REQTYPE_CLASS_EP_OUT 0x22

/* Control selectors */
#define FU_MUTE              0x01
#define FU_VOLUME            0x02
#define EP_SAMPLING_FREQ     0x01

#define RING_SAMPLES (1u << 18) /* ~5 s at 48 kHz */
#define RING_MASK    (RING_SAMPLES - 1)
#define MAX_UNITS    32
#define ISO_PACKETS  UM_ISO_PACKETS

static uhsx_dev um_usb;
static bool um_opened;
static uhsx_stream *um_stream;
static um_info um_dev;
static u8 um_ac_if, um_fu;
static u16 um_vol_chans, um_mute_chans;
static u8 um_subframe;
static bool um_freq_ctl;
static s16 *um_ring;
static volatile u32 um_ring_head, um_ring_tail, um_drops;
static OSMutex um_ring_lock;
static bool um_lock_ready;
static um_stats um_st;
static int (*um_logger)(const char *fmt, ...);

#define um_log(...) (um_logger ? (void)um_logger(__VA_ARGS__) : (void)0)

void UM_SetLog(int (*print)(const char *fmt, ...)) { um_logger = print; }

static s32 um_ctrl(u8 type, u8 req, u16 val, u16 idx, u16 len, void *data)
{
    return uhsx_ctrl(&um_usb, type, req, val, idx, len, data);
}

static u16 le16(const u8 *p)
{
	return (u16)(p[0] | (p[1] << 8));
}

static u32 le24(const u8 *p)
{
	return (u32)p[0] | ((u32)p[1] << 8) | ((u32)p[2] << 16);
}

/* ---- descriptor parsing -------------------------------------------------- */

typedef struct {
	u8 id, subtype, source;
	u16 type;
	/* feature units only */
	u16 vol_chans, mute_chans;
} um_unit;

/* One streaming alternate setting, and the format it would run at. */
typedef struct {
	u8 iface, alt;
	u8 term_link;
	u16 format;
	u8 channels, subframe, bits;
	u8 ep;
	u16 max_packet;
	bool iso_in;
	bool freq_ctl;
	u32 rate;
	int rate_score;
} um_alt;

/* How much a rate is wanted: 48 kHz first, as most mics run natively there,
 * then CD rate, then anything the Wii can play back. */
static int rate_score(u32 hz)
{
	if (hz == 48000) return 4;
	if (hz == 44100) return 3;
	if (hz == 32000) return 2;
	if (hz >= 8000 && hz <= 48000) return 1;
	return 0;
}

static void pick_rate(um_alt *a, const u8 *d)
{
	u8 n = d[7];
	u32 i;

	a->rate = 0;
	a->rate_score = 0;

	if (n == 0) {
		/* Continuous range: lower and upper bound. */
		u32 lo, hi;

		if (d[0] < 14)
			return;
		lo = le24(d + 8);
		hi = le24(d + 11);
		if (lo <= 48000 && hi >= 48000)
			a->rate = 48000;
		else if (lo <= 44100 && hi >= 44100)
			a->rate = 44100;
		else
			a->rate = hi > 48000 ? 48000 : hi;
		if (a->rate < lo)
			a->rate = 0;
		a->rate_score = rate_score(a->rate);
		return;
	}

	for (i = 0; i < n && 8 + 3 * i + 3 <= d[0]; i++) {
		u32 hz = le24(d + 8 + 3 * i);
		int s = rate_score(hz);

		if (s > a->rate_score || (s == a->rate_score && s == 1 && hz > a->rate)) {
			a->rate = hz;
			a->rate_score = s;
		}
	}
}

static bool alt_usable(const um_alt *a)
{
	u32 frame_bytes;

	if (!a->iso_in || a->format != FORMAT_PCM || a->rate_score == 0)
		return false;
	if (a->channels < 1 || a->channels > 2)
		return false;
	if (a->subframe < 2 || a->subframe > 4)
		return false;

	/* One frame's worth of audio has to fit in a packet. */
	frame_bytes = (a->rate + 999) / 1000 * a->channels * a->subframe;
	return a->max_packet >= frame_bytes;
}

static int alt_score(const um_alt *a)
{
	return a->rate_score * 100 + (a->subframe == 2 ? 20 : 0) +
	       (a->channels == 1 ? 10 : 0);
}

static um_unit *find_unit(um_unit *units, int n, u8 id)
{
	int i;

	for (i = 0; i < n; i++)
		if (units[i].id == id)
			return &units[i];
	return NULL;
}

/* Walks from the streaming terminal back towards the mic until it meets a
 * feature unit -- the one whose volume and mute govern what we record. */
static um_unit *capture_fu(um_unit *units, int n, u8 term_link)
{
	um_unit *u = find_unit(units, n, term_link);
	int hops;

	for (hops = 0; u && hops < MAX_UNITS; hops++) {
		if (u->subtype == AC_FEATURE_UNIT)
			return u;
		if (u->subtype == AC_INPUT_TERMINAL)
			break;
		u = find_unit(units, n, u->source);
	}

	/* No path to follow: fall back to the first feature unit fed by a
	 * microphone-type input terminal (0x02xx), then to any at all. */
	for (hops = 0; hops < n; hops++) {
		um_unit *src;

		if (units[hops].subtype != AC_FEATURE_UNIT)
			continue;
		src = find_unit(units, n, units[hops].source);
		if (src && src->subtype == AC_INPUT_TERMINAL &&
		    (src->type & 0xFF00) == 0x0200)
			return &units[hops];
	}
	for (hops = 0; hops < n; hops++)
		if (units[hops].subtype == AC_FEATURE_UNIT)
			return &units[hops];
	return NULL;
}

static void parse_unit(um_unit *u, const u8 *d)
{
	u8 len = d[0];

	memset(u, 0, sizeof(*u));
	u->subtype = d[2];
	u->id = len > 3 ? d[3] : 0;

	switch (u->subtype) {
	case AC_INPUT_TERMINAL:
		if (len >= 6)
			u->type = le16(d + 4);
		break;
	case AC_OUTPUT_TERMINAL:
		if (len >= 8) {
			u->type = le16(d + 4);
			u->source = d[7];
		}
		break;
	case AC_MIXER_UNIT:
	case AC_SELECTOR_UNIT:
		if (len >= 6 && d[4] > 0)
			u->source = d[5];
		break;
	case AC_FEATURE_UNIT: {
		u8 size = len > 5 ? d[5] : 0;
		int ch, nch;

		if (len < 7 || size == 0)
			break;
		u->source = d[4];
		nch = (len - 7) / size; /* the master plus each channel */
		if (nch > 16)
			nch = 16;
		for (ch = 0; ch < nch; ch++) {
			u8 ctl = d[6 + ch * size];

			if (ctl & 0x01)
				u->mute_chans |= 1u << ch;
			if (ctl & 0x02)
				u->vol_chans |= 1u << ch;
		}
		break;
	}
	case AC_PROCESSING_UNIT:
	case AC_EXTENSION_UNIT:
		if (len >= 8 && d[6] > 0)
			u->source = d[7];
		break;
	}
}

/* Reads through a configuration descriptor for the best streaming format.
 * Returns false when the device has no usable capture interface. */
static bool parse_config(const u8 *cfg, u32 total, um_alt *best, u8 *ac_if,
                         um_unit *fu_out)
{
	static um_unit units[MAX_UNITS];
	int nunits = 0;
	um_alt cur;
	bool in_as = false, in_ac = false, have = false;
	u32 off = 0;

	memset(&cur, 0, sizeof(cur));
	memset(best, 0, sizeof(*best));
	*ac_if = 0;

	while (off + 2 <= total) {
		const u8 *d = cfg + off;
		u8 len = d[0], type = d[1];

		if (len < 2 || off + len > total)
			break;

		if (type == USB_DT_INTERFACE && len >= 9) {
			if (in_as && alt_usable(&cur) &&
			    (!have || alt_score(&cur) > alt_score(best))) {
				*best = cur;
				have = true;
			}

			in_ac = d[5] == USB_CLASS_AUDIO && d[6] == AUDIO_SUBCLASS_CTRL;
			in_as = d[5] == USB_CLASS_AUDIO && d[6] == AUDIO_SUBCLASS_STRM &&
			        d[3] != 0;
			if (in_ac)
				*ac_if = d[2];

			memset(&cur, 0, sizeof(cur));
			cur.iface = d[2];
			cur.alt = d[3];
		} else if (type == USB_DT_CLASS_SPECIFIC_INTERFACE && len >= 3) {
			if (in_ac && nunits < MAX_UNITS &&
			    d[2] >= AC_INPUT_TERMINAL && d[2] <= AC_EXTENSION_UNIT)
				parse_unit(&units[nunits++], d);
			else if (in_as && d[2] == AS_GENERAL && len >= 7) {
				cur.term_link = d[3];
				cur.format = le16(d + 5);
			} else if (in_as && d[2] == AS_FORMAT_TYPE && len >= 8 &&
			           d[3] == FORMAT_TYPE_I) {
				cur.channels = d[4];
				cur.subframe = d[5];
				cur.bits = d[6];
				pick_rate(&cur, d);
			}
		} else if (type == USB_DT_ENDPOINT && len >= 7 && in_as) {
			/* Isochronous IN; a second (feedback) endpoint is not ours. */
			if ((d[2] & 0x80) && (d[3] & 0x03) == 0x01 && !cur.iso_in) {
				cur.iso_in = true;
				cur.ep = d[2];
				cur.max_packet = le16(d + 4) & 0x7FF;
			}
		} else if (type == USB_DT_CLASS_SPECIFIC_ENDPOINT && len >= 4 &&
		           in_as && d[2] == EP_GENERAL) {
			cur.freq_ctl = (d[3] & 0x01) != 0;
		}

		off += len;
	}

	if (in_as && alt_usable(&cur) && (!have || alt_score(&cur) > alt_score(best))) {
		*best = cur;
		have = true;
	}

	if (have) {
		um_unit *fu = capture_fu(units, nunits, best->term_link);

		if (fu)
			*fu_out = *fu;
		else
			memset(fu_out, 0, sizeof(*fu_out));
	}

	return have;
}

/* ---- class controls ------------------------------------------------------ */

static s32 fu_set(u8 cs, u8 ch, const void *data, u16 len)
{
	return um_ctrl(REQTYPE_CLASS_IF_OUT, REQ_SET_CUR, (u16)((cs << 8) | ch),
	               (u16)((um_fu << 8) | um_ac_if), len, (void *)data);
}

static s32 fu_get16(u8 req, u8 cs, u8 ch, s16 *out)
{
	u8 v[2] = { 0, 0 };
	s32 ret = um_ctrl(REQTYPE_CLASS_IF_IN, req, (u16)((cs << 8) | ch),
	                  (u16)((um_fu << 8) | um_ac_if), 2, v);

	if (ret >= 2)
		*out = (s16)le16(v);
	return ret;
}

/* The lowest channel that has the control: the master if it does. */
static u8 first_chan(u16 mask)
{
	u8 ch;

	for (ch = 0; ch < 16; ch++)
		if (mask & (1u << ch))
			return ch;
	return 0;
}

static void read_volume_range(void)
{
	u8 ch = first_chan(um_vol_chans);
	s16 lo = 0, hi = 0, cur = 0;
	s32 r1, r2;

	if (!um_dev.has_volume)
		return;

	r1 = fu_get16(REQ_GET_MIN, FU_VOLUME, ch, &lo);
	r2 = fu_get16(REQ_GET_MAX, FU_VOLUME, ch, &hi);
	if (r1 < 2 || r2 < 2 || hi <= lo) {
		um_log("  volume range unreadable (unit %u ch %u: %d %d, %d..%d);"
		       " volume control off\n", um_fu, ch, (int)r1, (int)r2, lo, hi);
		um_dev.has_volume = false;
		return;
	}
	fu_get16(REQ_GET_CUR, FU_VOLUME, ch, &cur);

	um_dev.vol_min = lo;
	um_dev.vol_max = hi;
	um_dev.vol_cur = cur;
	um_log("  volume %d..%d dB, now %d dB\n", lo / 256, hi / 256, cur / 256);
}

s32 UM_SetMute(bool mute)
{
	u8 v = mute ? 1 : 0;
	s32 ret = UM_ENOTSUP;
	u8 ch;

	if (!um_opened)
		return UM_ESTATE;
	if (!um_dev.has_mute)
		return UM_ENOTSUP;

	for (ch = 0; ch < 16; ch++)
		if (um_mute_chans & (1u << ch))
			ret = fu_set(FU_MUTE, ch, &v, 1);
	return ret < 0 ? ret : UM_OK;
}

s32 UM_SetVolume(u32 percent)
{
	s32 range, vol;
	s32 ret = UM_ENOTSUP;
	u8 v[2];
	u8 ch;

	if (!um_opened)
		return UM_ESTATE;
	if (!um_dev.has_volume)
		return UM_ENOTSUP;
	if (percent > 100)
		percent = 100;

	range = (s32)um_dev.vol_max - um_dev.vol_min;
	vol = um_dev.vol_min + range * (s32)percent / 100;
	v[0] = (u8)vol;
	v[1] = (u8)(vol >> 8);

	for (ch = 0; ch < 16; ch++)
		if (um_vol_chans & (1u << ch))
			ret = fu_set(FU_VOLUME, ch, v, 2);
	if (ret >= 0)
		um_dev.vol_cur = (s16)vol;
	return ret < 0 ? ret : UM_OK;
}

/* One sample of `bytes` little-endian bytes, as 16-bit. */
static s16 sample16(const u8 *p, u8 bytes)
{
	return (s16)(p[bytes - 2] | (p[bytes - 1] << 8));
}

/* Folds `len` bytes of interleaved frames to mono, returning the count. */
static u32 convert(const u8 *src, u32 len, s16 *out)
{
	u32 frame = um_dev.channels * um_subframe;
	u32 n = 0, off;

	for (off = 0; off + frame <= len; off += frame) {
		if (um_dev.channels == 1)
			out[n++] = sample16(src + off, um_subframe);
		else
			out[n++] = (s16)(((s32)sample16(src + off, um_subframe) +
			                  sample16(src + off + um_subframe, um_subframe)) / 2);
	}
	return n;
}

static void um_push(const s16 *samples, u32 count)
{
	OSLockMutex(&um_ring_lock);
	for (u32 i = 0; i < count; i++) {
		u32 next = (um_ring_head + 1) & RING_MASK;
		if (next == um_ring_tail) {
			um_drops += count - i;
			break;
		}
		um_ring[um_ring_head] = samples[i];
		um_ring_head = next;
	}
	OSUnlockMutex(&um_ring_lock);
}

/* One IN transfer: each packet's audio at the front of its slot, as long
 * as the system says it is. */
static void um_on_in(uint8_t *buf, const uint16_t *got, uint32_t count, uint32_t size, int32_t ret, void *ctx)
{
	static s16 samples[ISO_PACKETS * 1024 / 2];
	u32 frame = um_dev.channels * um_subframe, n = 0;

	(void)ctx;
	um_st.calls++;
	um_st.last_ret = ret;
	um_st.sizes_rewritten = true;
	for (u32 i = 0; i < count && i < UM_ISO_PACKETS; i++)
		um_st.last_sizes[i] = got[i];
	if (ret < 0) {
		um_st.errors++;
		return;
	}
	for (u32 i = 0; i < count; i++) {
		u32 len = got[i] < size ? got[i] : size;
		len -= len % frame;
		if (!len) {
			um_st.empty_packets++;
			continue;
		}
		n += convert(buf + i * size, len, samples + n);
	}
	um_st.samples += n;
	if (n)
		um_push(samples, n);
}

/* ---- opening -------------------------------------------------------------- */

static s32 select_alt(u8 alt)
{
	return uhsx_set_alt(&um_usb, um_dev.interface, alt);
}

/* The open device: is it a mic this can stream from? */
static bool probe(void)
{
	um_alt best;
	um_unit fu;
	u8 ac_if;

	if (!um_usb.cfg_len || !parse_config(um_usb.cfg, um_usb.cfg_len, &best, &ac_if, &fu)) {
		um_log("  %04x:%04x: no usable capture format\n", um_usb.vid, um_usb.pid);
		return false;
	}
	memset(&um_dev, 0, sizeof(um_dev));
	um_dev.vid = um_usb.vid;
	um_dev.pid = um_usb.pid;
	um_dev.rate = best.rate;
	um_dev.channels = best.channels;
	um_dev.bits = best.bits ? best.bits : best.subframe * 8;
	um_dev.interface = best.iface;
	um_dev.alt = best.alt;
	um_dev.endpoint = best.ep;
	um_dev.max_packet = best.max_packet;
	uhsx_string(&um_usb, um_usb.iproduct, um_dev.name, sizeof(um_dev.name));
	{
		u32 n = strlen(um_dev.name);
		while (n && um_dev.name[n - 1] == ' ')
			um_dev.name[--n] = 0;
		if (!n)
			snprintf(um_dev.name, sizeof(um_dev.name), "%04x:%04x", um_usb.vid, um_usb.pid);
	}
	um_ac_if = ac_if;
	um_fu = fu.id;
	um_vol_chans = fu.vol_chans;
	um_mute_chans = fu.mute_chans;
	um_dev.has_volume = um_fu && um_vol_chans;
	um_dev.has_mute = um_fu && um_mute_chans;
	um_subframe = best.subframe;
	um_freq_ctl = best.freq_ctl;
	um_log("  %s: if %u alt %u ep %02x, %u ch %u-bit %u Hz, pkt %u\n", um_dev.name, best.iface, best.alt, best.ep,
	       best.channels, um_dev.bits, (unsigned)best.rate, best.max_packet);
	return true;
}

s32 UM_Open(um_info *info)
{
	s32 ret;

	if (um_opened)
		return UM_ESTATE;
	if (!um_lock_ready) {
		OSInitMutex(&um_ring_lock);
		um_lock_ready = true;
	}
	if (uhsx_init(um_logger) < 0)
		return UM_ENODEV;
	/* The first device with an audio interface: the system leaves those
	 * alone (a headset's buttons are HID, and the HID library's). */
	if (uhsx_open(&um_usb, 0, 0, 0x01) < 0) {
		um_log("USB mic: none found\n");
		uhsx_shutdown();
		return UM_ENODEV;
	}
	if (!probe()) {
		uhsx_close(&um_usb);
		uhsx_shutdown();
		return UM_ENODEV;
	}
	um_ring = uhsx_alloc(RING_SAMPLES * sizeof(s16));
	if (!um_ring || um_dev.max_packet > 1024) {
		ret = UM_ENOMEM;
		goto fail;
	}
	memset(&um_st, 0, sizeof(um_st));
	um_opened = true;

	ret = select_alt(um_dev.alt);
	um_log("  set alt %u: %d\n", um_dev.alt, (int)ret);
	if (ret < 0)
		goto fail;
	if (um_freq_ctl) {
		u8 hz[3] = { (u8)um_dev.rate, (u8)(um_dev.rate >> 8), (u8)(um_dev.rate >> 16) };
		/* Not fatal: plenty of mics run one rate and refuse the request. */
		um_log("  set rate %u: %d\n", (unsigned)um_dev.rate,
		       (int)um_ctrl(REQTYPE_CLASS_EP_OUT, REQ_SET_CUR, EP_SAMPLING_FREQ << 8, um_dev.endpoint, 3, hz));
	}
	read_volume_range();
	if (um_dev.has_mute)
		um_log("  unmute: %d\n", (int)UM_SetMute(false));
	um_ring_head = um_ring_tail = um_drops = 0;
	if (info)
		*info = um_dev;
	return UM_OK;

fail:
	um_opened = false;
	uhsx_free(um_ring);
	um_ring = NULL;
	uhsx_close(&um_usb);
	uhsx_shutdown();
	return ret;
}

s32 UM_Close(void)
{
	if (!um_opened)
		return UM_ESTATE;
	UM_Stop();
	um_log("  um: alt 0 %d\n", (int)select_alt(0));
	uhsx_close(&um_usb);
	uhsx_shutdown();
	uhsx_free(um_ring);
	um_ring = NULL;
	um_opened = false;
	return UM_OK;
}

bool UM_IsOpen(void)
{
	return um_opened;
}

s32 UM_Start(void)
{
	if (!um_opened)
		return UM_ESTATE;
	if (!um_stream)
		um_stream = uhsx_stream_start(&um_usb, um_dev.endpoint, ISO_PACKETS, um_dev.max_packet, um_on_in, NULL);
	return um_stream ? UM_OK : UM_ENOMEM;
}

s32 UM_Stop(void)
{
	uhsx_stream_stop(um_stream);
	um_stream = NULL;
	return UM_OK;
}

void UM_Quiesce(void)
{
	UM_Stop();
}

u32 UM_Read(s16 *dst, u32 max_samples)
{
	u32 taken = 0;

	if (!dst || !um_ring)
		return 0;
	OSLockMutex(&um_ring_lock);
	while (taken < max_samples && um_ring_tail != um_ring_head) {
		dst[taken++] = um_ring[um_ring_tail];
		um_ring_tail = (um_ring_tail + 1) & RING_MASK;
	}
	OSUnlockMutex(&um_ring_lock);
	return taken;
}

u32 UM_Available(void)
{
	return (um_ring_head - um_ring_tail) & RING_MASK;
}

u32 UM_Dropped(void)
{
	return um_drops;
}

void UM_GetStats(um_stats *out)
{
	if (out)
		*out = um_st;
}

#endif /* HAVE_MIC */
