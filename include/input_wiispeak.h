/* Wii Speak (P-Mic) for libwiiuinput: libwiiinput's driver, on the Wii U's
 * USB host stack (nsysuhs) instead of IOS's /dev/usb/oh0.
 *
 * The protocol (register commands, the firmware upload, the streams) is
 * the RVL SDK's PMIC library's, recovered from pmic.a and checked against
 * Dolphin's emulated device. The Wii Speak plugs into either USB port.
 *
 * Built only with HAVE_MIC (make WITH_MIC=1, the default).
 */

#ifndef INPUT_WIISPEAK_H
#define INPUT_WIISPEAK_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define WS_VID 0x057E
#define WS_PID 0x0308

/* Sampling frequency (SAMPLER_FREQ). */
#define WS_FREQ_8KHZ  0x0000
#define WS_FREQ_11KHZ 0x0001
#define WS_FREQ_16KHZ 0x0003 /* default */

/* Microphone ADC gain (SAMPLER_GAIN). */
#define WS_GAIN_00dB 0x0000
#define WS_GAIN_15dB 0x0001
#define WS_GAIN_30dB 0x0002
#define WS_GAIN_36dB 0x0003 /* default */

/* Errors.  USB-level failures are returned as-is (always negative). */
#define WS_OK          0
#define WS_ENODEV     -1
#define WS_EINVAL     -2
#define WS_ESTATE     -3
#define WS_EFIRMWARE  -4
#define WS_ENOMEM     -5
#define WS_ETIMEOUT   -6
#define WS_ECANCEL    -7

/* Returns the sample rate in Hz for a WS_FREQ_* constant. */
uint32_t WS_FreqToHz(uint16_t freq);

/* Where the boot steps and WS_Diagnose() report go, e.g. WS_SetLog(printf).
 * Nothing is printed until this is set. */
void WS_SetLog(int (*print)(const char *fmt, ...));

/* Polled during WS_Open()'s long waits; returning true abandons the open
 * with WS_ECANCEL. */
void WS_SetCancel(bool (*cancel)(void));

/* Opens the device and boots it with `firmware` (PMIC_FIRM_DATA from the
 * SDK's pmic.a -- see tools/extract_pmic_firmware.py).  The mic ignores every
 * other request until this has completed. */
int32_t WS_Open(const void *firmware, uint32_t firmware_size);

/* The boot firmware built into the library (make PMIC_FIRM=<file>, or
 * data/pmic_firm.bin present at build time), for WS_Open().  NULL, with
 * *size 0, when the library was built without one. */
const uint8_t *WS_BuiltinFirmware(uint32_t *size);
int32_t WS_Close(void);

/* Reports (through WS_SetLog) every USB interface the system offers.
 * Call this when WS_Open() reports WS_ENODEV. */
void WS_Diagnose(void);

/* Settings.  Only valid between WS_Open() and WS_Close(). */
int32_t WS_SetFreq(uint16_t freq);
int32_t WS_GetFreq(uint16_t *freq);
int32_t WS_SetGain(uint16_t gain);
int32_t WS_SetMute(bool mute);
int32_t WS_SetEcReset(bool reset);

/* Acoustic signal processing (echo canceller + AGC).  With it enabled the
 * usable band is capped near 11 kHz even when sampling at 16 kHz, so a plain
 * recorder wants it off. */
int32_t WS_SetSignalProcessing(bool enable);

/* Instantaneous send level, as used by the SDK's level meter. */
int32_t WS_GetSendLevel(uint16_t *level);

/* Streaming.  WS_Start() spawns the transfer threads; samples then accumulate
 * in an internal ring buffer until read. */
int32_t WS_Start(void);
int32_t WS_Stop(void);

/* For experiments, before WS_Open(): whether the OUT (echo-canceller
 * reference) stream runs at all. Two transfers are always kept in flight
 * each way here, so `in_slots` is not used. */
void WS_SetStreams(int in_slots, bool out_stream);

/* For experiments: 0 feeds the OUT stream silence, 1 a 1 kHz tone. */
void WS_SetOutSignal(int mode);

/* For experiments, before WS_Open(): how long to wait after the firmware
 * upload (default 300 ms), and whether to skip the reset before it. */
void WS_SetBoot(int wait_ms, bool skip_reset);

/* For experiments: packets per IN transfer, 1 to 16 (default 8). */
void WS_SetInPackets(int n);

/* Kept for libwiiinput's callers; does nothing here (the streams' own
 * threads resubmit). */
void WS_SetCallbackMode(bool on);

/* Stops the transfer threads from queueing anything new, without waiting
 * for them or talking to the mic -- for the emergency exit. */
void WS_Quiesce(void);

/* Copies up to `max_samples` host-endian 16-bit mono samples out of the ring
 * buffer and returns how many were taken. */
uint32_t WS_Read(int16_t *dst, uint32_t max_samples);

/* Samples currently buffered, and the count dropped to overrun since
 * WS_Start().  A non-zero drop count means the consumer is too slow. */
uint32_t WS_Available(void);
uint32_t WS_Dropped(void);

/* ---- debugging ---------------------------------------------------------- */

/* Sends one raw 10-byte register command.  With out1/out2 NULL it is a pure
 * write; otherwise the reply is read back. */
int32_t WS_RawCommand(uint8_t reg, uint16_t arg1, uint16_t arg2, uint16_t *out1, uint16_t *out2);

#define WS_ISO_PACKETS 8
#define WS_ISO_PKTSIZE 32

typedef struct {
	uint32_t in_calls, in_errors, in_samples;
	uint32_t out_calls, out_errors;
	int32_t last_in_ret, last_out_ret;
	uint16_t last_sizes[WS_ISO_PACKETS];
	int last_layout;
	uint16_t last_out_sizes[WS_ISO_PACKETS]; /* as sent */
} ws_stats;

void WS_GetStats(ws_stats *out);

/* Called from the IN stream with every completed transfer, before any
 * interpretation: the result, each packet's length as the system gave it,
 * and the whole landing buffer (WS_ISO_PKTSIZE bytes a packet). */
typedef void (*ws_iso_tap)(int32_t ret, const uint16_t *sizes, const uint8_t *buf);
void WS_SetIsoTap(ws_iso_tap tap);

/* How samples are pulled out of an IN transfer. The Wii U's host stack
 * always gives each packet's length, so AUTO is SLOTS here. */
#define WS_LAYOUT_AUTO   0
#define WS_LAYOUT_PACKED 1 /* `ret` bytes from the start of the buffer */
#define WS_LAYOUT_SLOTS  2 /* sizes[i] bytes from each 32-byte slot */
void WS_SetLayout(int layout);
int WS_GetLayout(void);

/* Byte-swap every sample (for testing a little-endian device). */
void WS_SetSwap(bool swap);
bool WS_GetSwap(void);

#ifdef __cplusplus
}
#endif

#endif /* INPUT_WIISPEAK_H */
