#ifndef LIBINPUT_INTERNAL_H
#define LIBINPUT_INTERNAL_H

#include "input_wiiu.h"

extern int inpw_screen_w, inpw_screen_h;

void inpw_set_source(const char *name);
/* Milliseconds since boot, free running. */
uint32_t inpw_now_ms(void);

int32_t inpw_pads_init(uint32_t flags);
void    inpw_pads_shutdown(void);
void    inpw_pads_update(void);

int32_t inpw_keyboard_init(void);
void    inpw_keyboard_shutdown(void);
void    inpw_keyboard_update(void);

/* The one HID client: mice, the GameCube adapter and USB pads. */
int32_t inpw_hid_init(bool mouse, bool gcadapter, bool hidpads);
void    inpw_hid_shutdown(void);
/* Rumble and polls that go out from the main thread. */
void    inpw_hid_update(void);

/* The adapter's ports as of now, once a frame before they are read. */
void    inpw_gcadapter_update(void);

/* Mouse motion gathered by the HID client since the last call. */
void    inpw_hid_mouse_take(int *dx, int *dy, int *wheel, uint8_t *held, uint8_t *pressed, bool *connected);

void    inpw_mouse_update(void);
void    inpw_mouse_set_bounds(int w, int h);
void    inpw_cursor_update(void);

/* Pen tablets (libinput_tablet.c) and the uDraw (libinput_udraw.c). */
bool    inpw_tablet_vendor(uint16_t vid);
bool    inpw_tablet_running(void);
void    inpw_udraw_update(void);
bool    inpw_udraw_on(int chan);
void    inpw_udraw_debug(int chan, uint8_t *ext, uint8_t id[6], bool *id_read);
/* A Classic Controller read from its registers: WPAD_CLASSIC_BUTTON_* bits. */
bool    inpw_extreg_classic(int chan, uint32_t *buttons, float st[4], float tr[2]);
#ifdef HAVE_TABLET
#include "input_tablet.h"
bool    inpw_udraw_pop(tb_sample *out);
bool    inpw_udraw_state(tb_sample *out);
bool    inpw_udraw_info(tb_info *info);
void    inpw_udraw_extent(int32_t *w, int32_t *h);
#endif

#endif
