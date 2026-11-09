/* The USB mouse: what the HID client (libinput_hidclient.c) gathers,
 * turned into a position and per-frame deltas and button edges. */
#include "libinput_internal.h"

static bool connected;
static int bound_w = 1280, bound_h = 720;
static int pos_x = 640, pos_y = 360;
static int delta_x, delta_y, delta_wheel;
static uint8_t buttons_held, buttons_pressed, buttons_released;
static int speed = 1;

static int clamp(int v, int lo, int hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

void inpw_mouse_update(void)
{
    uint8_t prev = buttons_held, quick = 0;

    inpw_hid_mouse_take(&delta_x, &delta_y, &delta_wheel, &buttons_held, &quick, &connected);

    pos_x = clamp(pos_x + delta_x * speed, 0, bound_w - 1);
    pos_y = clamp(pos_y + delta_y * speed, 0, bound_h - 1);

    /* `quick` holds every button that went down since the last frame, so a
     * click shorter than a frame still counts. */
    buttons_pressed = (buttons_held & ~prev) | quick;
    buttons_released = (prev | quick) & ~buttons_held;

    if (delta_x || delta_y || delta_wheel || buttons_pressed)
        inpw_set_source("Mouse");
}

bool INPUT_MouseConnected(void) { return connected; }

void inpw_mouse_set_bounds(int width, int height)
{
    bound_w = width;
    bound_h = height;
    pos_x = clamp(pos_x, 0, bound_w - 1);
    pos_y = clamp(pos_y, 0, bound_h - 1);
}

void INPUT_MousePosition(int *x, int *y)
{
    if (x) *x = pos_x;
    if (y) *y = pos_y;
}

void INPUT_MouseSetSpeed(int multiplier)
{
    speed = multiplier > 0 ? multiplier : 1;
}

void INPUT_MouseSetPosition(int x, int y)
{
    pos_x = clamp(x, 0, bound_w - 1);
    pos_y = clamp(y, 0, bound_h - 1);
}

void INPUT_MouseDelta(int *dx, int *dy, int *wheel)
{
    if (dx) *dx = delta_x;
    if (dy) *dy = delta_y;
    if (wheel) *wheel = delta_wheel;
}

uint8_t INPUT_MouseButtonsHeld(void)     { return buttons_held; }
uint8_t INPUT_MouseButtonsPressed(void)  { return buttons_pressed; }
uint8_t INPUT_MouseButtonsReleased(void) { return buttons_released; }
