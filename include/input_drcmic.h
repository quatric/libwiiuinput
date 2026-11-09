/* The Wii U GamePad's own microphone, for libwiiuinput.
 *
 * Read through the system's mic library (mic.rpl): the GamePad sends its
 * microphone with its other data, so nothing needs plugging in. Samples
 * come as 16-bit mono at the rate DM_Rate() gives (32 kHz, as the system
 * reports it, or that when it does not say).
 *
 * The same calls as the Wii Speak's and USB microphones' (input_wiispeak.h,
 * input_usbmic.h): open, start, read what has come, stop, close.
 *
 * Built only with HAVE_MIC (make WITH_MIC=1, the default).
 */

#ifndef INPUT_DRCMIC_H
#define INPUT_DRCMIC_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DM_OK       0
#define DM_ENODEV  -1
#define DM_EINVAL  -2
#define DM_ESTATE  -3
#define DM_ENOMEM  -5

/* Where errors go, e.g. DM_SetLog(printf). */
void DM_SetLog(int (*print)(const char *fmt, ...));

/* Opens the microphone of GamePad `gamepad` (0, or 1 for a second one). */
int32_t DM_Open(int gamepad);
int32_t DM_Close(void);
bool    DM_IsOpen(void);

/* Samples per second. */
uint32_t DM_Rate(void);

/* Streaming: samples accumulate in the system's buffer between DM_Start
 * and DM_Stop. Call DM_Read often (every frame): that buffer holds about a
 * third of a second, and what is still unread when it fills is lost. */
int32_t DM_Start(void);
int32_t DM_Stop(void);

/* Copies up to `max_samples` 16-bit mono samples out and returns how many
 * were taken. */
uint32_t DM_Read(int16_t *dst, uint32_t max_samples);
uint32_t DM_Available(void);
uint32_t DM_Dropped(void);

#ifdef __cplusplus
}
#endif

#endif /* INPUT_DRCMIC_H */
