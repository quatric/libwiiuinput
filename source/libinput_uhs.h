/* Raw USB devices through the system's USB host stack (nsysuhs), for the
 * microphone drivers.
 *
 * wut declares only some of nsysuhs; the rest is declared here from the
 * library itself (nsysuhs.rpl and its use in usb_mic.rpl):
 *
 *   UhsSetInterface(handle, if_handle, alt)
 *   UhsGetCurrentFrame(handle, if_handle, &frame)
 *   UhsSubmitIsocRequest(handle, if_handle, endpoint number, direction,
 *                        start frame, flags (0), &request, timeout)
 *
 * An isochronous request is up to 32 packets, laid out one after another
 * in `buffer` at their requested lengths. The system writes each packet's
 * status (0, or UHSX_ISOC_SHORT for a short packet, are good) and the
 * length that came. Every packet takes one 1 ms frame; usb_mic.rpl starts
 * eight frames ahead of the current one and carries on from there.
 */

#ifndef LIBINPUT_UHS_H
#define LIBINPUT_UHS_H

#include <stdbool.h>
#include <stdint.h>

#include <nsysuhs/uhs.h>

#define UHSX_ISOC_MAX 32
#define UHSX_ISOC_SHORT ((int32_t)0xFFDEFFE1)

typedef struct {
    uint32_t length;  /* asked for */
    int32_t  status;
    uint32_t actual;  /* came */
} UhsxIsocPacket;

typedef struct {
    uint32_t count;   /* packets, 1-32 */
    void    *buffer;
    uint32_t length;  /* the sum of the packets' lengths */
    UhsxIsocPacket packet[UHSX_ISOC_MAX];
} UhsxIsocRequest;

UHSStatus UhsSetInterface(UhsHandle *handle, uint32_t if_handle, uint8_t alt);
UHSStatus UhsGetCurrentFrame(UhsHandle *handle, uint32_t if_handle, uint32_t *frame);
UHSStatus UhsSubmitIsocRequest(UhsHandle *handle, uint32_t if_handle, uint8_t endpoint, int32_t direction,
                               uint32_t frame, uint32_t flags, UhsxIsocRequest *request, int32_t timeout);

/* ---- a device, as the drivers use it ---------------------------------- */

#define UHSX_MAX_IFACES 8
#define UHSX_CFG_MAX 1024

typedef struct {
    bool     open;
    uint16_t vid, pid;
    int      nif;
    uint32_t if_handle[UHSX_MAX_IFACES];
    uint8_t  if_number[UHSX_MAX_IFACES];
    uint8_t  if_class[UHSX_MAX_IFACES], if_subclass[UHSX_MAX_IFACES];
    uint8_t  iproduct;
    uint8_t *cfg;              /* the whole configuration descriptor */
    uint32_t cfg_len;
    uint32_t ep_enabled[UHSX_MAX_IFACES]; /* endpoint masks switched on */
    uint32_t next_frame[32];   /* per endpoint (number + 16 for IN) */
    bool     frame_set[32];
} uhsx_dev;

/* Opens the host stack (once; later calls only count). */
int32_t uhsx_init(int (*log)(const char *fmt, ...));
void    uhsx_shutdown(void);

/* Every interface the system offers, for a diagnosis. */
void uhsx_list(int (*log)(const char *fmt, ...));

/* The first device with this VID:PID (vid 0: any), or with an interface of
 * this class (if_class 0xFF: any); every interface of it is acquired. */
int32_t uhsx_open(uhsx_dev *d, uint16_t vid, uint16_t pid, uint8_t if_class);
void    uhsx_close(uhsx_dev *d);

/* Returns the bytes moved, or a negative error. `data` needs no alignment. */
int32_t uhsx_ctrl(uhsx_dev *d, uint8_t type, uint8_t req, uint16_t val, uint16_t idx, uint16_t len, void *data);
int32_t uhsx_bulk_out(uhsx_dev *d, uint8_t ep, const void *data, uint32_t len);
int32_t uhsx_set_alt(uhsx_dev *d, uint8_t iface, uint8_t alt);
/* Switches an endpoint on (or off), as usb_mic.rpl does before streaming. */
int32_t uhsx_endpoint(uhsx_dev *d, uint8_t iface, uint8_t ep, bool on);

/* One isochronous transfer of `count` packets of `size` bytes each, into or
 * out of `buf` (0x40 aligned, count * size bytes). `got[i]` is set to each
 * packet's length (0 for a bad one). Blocks until the frames have passed;
 * returns the bytes moved or a negative error. */
int32_t uhsx_isoc(uhsx_dev *d, uint8_t ep, uint32_t count, uint32_t size, void *buf, uint16_t *got);

/* A running isochronous stream: two transfers of `count` packets of `size`
 * bytes kept in flight, on frames that follow one another, each handed to
 * `cb` in order on the stream's threads. For an IN endpoint `cb` gets the
 * data (with `got` and the result); for an OUT one it fills `buf` before
 * the transfer goes (`got` NULL). */
#define UHSX_STREAM_SLOTS 2
typedef void (*uhsx_stream_cb)(uint8_t *buf, const uint16_t *got, uint32_t count, uint32_t size, int32_t ret,
                               void *ctx);
typedef struct uhsx_stream uhsx_stream;
uhsx_stream *uhsx_stream_start(uhsx_dev *d, uint8_t ep, uint32_t count, uint32_t size, uhsx_stream_cb cb, void *ctx);
void         uhsx_stream_stop(uhsx_stream *st);

/* The string at `index` as ASCII. */
void uhsx_string(uhsx_dev *d, uint8_t index, char *out, uint32_t size);

void *uhsx_alloc(uint32_t size);   /* 0x40 aligned, from the default heap */
void  uhsx_free(void *p);

uint32_t uhsx_ms(void);
void     uhsx_sleep_ms(uint32_t ms);

#endif
