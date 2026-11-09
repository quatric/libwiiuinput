/* USB Audio Class microphones for libwiiuinput: libwiiinput's driver, on
 * the Wii U's USB host stack (nsysuhs).
 *
 * Captures from any class-compliant (UAC 1.0) USB microphone -- the kind that
 * needs no driver on a PC, headsets' mics included.  The streaming format is
 * read from the device's own descriptors: 16-bit PCM at 48 kHz is preferred,
 * and stereo or 24-bit streams are folded down to 16-bit mono.
 *
 * Built only with HAVE_MIC (make WITH_MIC=1, the default).
 */

#ifndef INPUT_USBMIC_H
#define INPUT_USBMIC_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Errors.  USB-level failures are returned as-is (always negative). */
#define UM_OK        0
#define UM_ENODEV   -1
#define UM_EINVAL   -2
#define UM_ESTATE   -3
#define UM_ENOMEM   -5
#define UM_ETIMEOUT -6
#define UM_ENOTSUP  -8

typedef struct {
	uint16_t vid, pid;
	char name[48];      /* product string, or "VID:PID" without one */
	uint32_t rate;           /* samples per second, as delivered by UM_Read */
	uint8_t channels;        /* channels the device sends (folded to mono) */
	uint8_t bits;            /* bits per sample the device sends */
	uint8_t interface, alt;  /* the streaming interface in use */
	uint8_t endpoint;
	uint16_t max_packet;
	bool has_volume;
	bool has_mute;
	int16_t vol_min, vol_max, vol_cur; /* 1/256 dB, as the device reports */
} um_info;

/* Where the device scan and errors go, e.g. UM_SetLog(printf). */
void UM_SetLog(int (*print)(const char *fmt, ...));

/* Finds the first USB microphone, selects its best capture format, unmutes
 * it and fills `info` (which may be NULL).  UM_ENODEV when there is none. */
int32_t UM_Open(um_info *info);
int32_t UM_Close(void);
bool UM_IsOpen(void);

/* Streaming.  UM_Start() spawns the transfer thread; samples then accumulate
 * in an internal ring buffer until read. */
int32_t UM_Start(void);
int32_t UM_Stop(void);

/* Stops the transfer thread from queueing anything new, without waiting for
 * it -- for an emergency exit. */
void UM_Quiesce(void);

/* Copies up to `max_samples` 16-bit mono samples out of the ring buffer and
 * returns how many were taken. */
uint32_t UM_Read(int16_t *dst, uint32_t max_samples);
uint32_t UM_Available(void);
uint32_t UM_Dropped(void);

/* Volume as a fraction of the device's range, 0-100.  UM_ENOTSUP when the
 * mic has no volume control. */
int32_t UM_SetVolume(uint32_t percent);
int32_t UM_SetMute(bool mute);

/* ---- debugging ---------------------------------------------------------- */

#define UM_ISO_PACKETS 8

typedef struct {
	uint32_t calls, errors, samples, empty_packets;
	int32_t last_ret;
	uint16_t last_sizes[UM_ISO_PACKETS];
	bool sizes_rewritten; /* kept for libwiiinput's callers: always true here */
} um_stats;

void UM_GetStats(um_stats *out);

#ifdef __cplusplus
}
#endif

#endif /* INPUT_USBMIC_H */
