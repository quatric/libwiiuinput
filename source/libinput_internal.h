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

#endif
