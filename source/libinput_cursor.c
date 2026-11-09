#include "libinput_internal.h"
#include <math.h>
#include <stdlib.h>

#define MOVE_THRESHOLD 3 /* pixels; ignores IR jitter */

static uint32_t modes = INPUT_CURSOR_IR | INPUT_CURSOR_MOUSE | INPUT_CURSOR_TOUCH;
static bool active, moved, mouse_active;
static int cur_x = 640, cur_y = 360;
static int last_x = -1000, last_y = -1000;

static int clamp(int v, int lo, int hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

void     INPUT_SetCursorMode(uint32_t m) { modes = m; }
uint32_t INPUT_GetCursorMode(void)       { return modes; }
bool     INPUT_CursorActive(void)        { return active; }
bool     INPUT_CursorMoved(void)         { return moved; }

void INPUT_CursorPosition(int *x, int *y)
{
    if (x) *x = cur_x;
    if (y) *y = cur_y;
}

void inpw_cursor_update(void)
{
    float px, py, sx, sy;
    int tx, ty, dx, dy, wheel;
    bool pointed = false;

    active = false;
    moved = false;
    INPUT_MouseDelta(&dx, &dy, &wheel);

    if (!INPUT_MouseConnected())
        mouse_active = false;
    else if (dx || dy || INPUT_MouseButtonsPressed())
        mouse_active = true;

    /* Touch first: a finger on the GamePad is the most deliberate pointing. */
    if ((modes & INPUT_CURSOR_TOUCH) && INPUT_Touch(&tx, &ty)) {
        pointed = true;
        cur_x = tx;
        cur_y = ty;
    } else if ((modes & INPUT_CURSOR_IR) && INPUT_Pointer(&px, &py)) {
        pointed = true;
        cur_x = (int)px;
        cur_y = (int)py;
    }

    if (pointed) {
        active = true;
        /* Hand over smoothly: the mouse continues from where the pointer is. */
        INPUT_MouseSetPosition(cur_x, cur_y);
        if (dx || dy)
            mouse_active = true;
    } else if ((modes & INPUT_CURSOR_MOUSE) && mouse_active) {
        active = true;
        INPUT_MousePosition(&cur_x, &cur_y);
    }

    if (!pointed && (modes & INPUT_CURSOR_STICK)) {
        float mag = INPUT_Stick(INPUT_STICK_LEFT, &sx, &sy, NULL);
        if (mag > 0.18f) {
            /* scaled for 1280 wide, twice libwiiinput's 640 */
            float speed = (9.0f * powf(mag, 1.6f) + 0.6f) * (inpw_screen_w / 640.0f);
            cur_x += (int)(sx * speed);
            cur_y -= (int)(sy * speed); /* stick y is up, screen y is down */
            active = true;
            INPUT_MouseSetPosition(cur_x, cur_y);
        }
    }

    cur_x = clamp(cur_x, 0, inpw_screen_w - 1);
    cur_y = clamp(cur_y, 0, inpw_screen_h - 1);

    if (active) {
        moved = abs(cur_x - last_x) > MOVE_THRESHOLD || abs(cur_y - last_y) > MOVE_THRESHOLD;
        if (moved) {
            last_x = cur_x;
            last_y = cur_y;
        }
    }
}
