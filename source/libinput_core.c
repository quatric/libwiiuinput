#include "libinput_internal.h"
#include <coreinit/time.h>

int inpw_screen_w = 1280, inpw_screen_h = 720;

static bool initialized;
static const char *source_name = "";

void inpw_set_source(const char *name) { source_name = name; }
const char *INPUT_Source(void)          { return source_name; }

uint32_t inpw_now_ms(void)
{
    return (uint32_t)OSTicksToMilliseconds(OSGetSystemTime());
}

int32_t INPUT_Init(void)
{
    return INPUT_InitEx(0);
}

int32_t INPUT_InitEx(uint32_t flags)
{
    if (initialized)
        return 0;

    inpw_pads_init(flags);

    /* Neither is fatal: a console without a keyboard or USB still has its
     * controllers. The keyboard library goes first, so it sees keyboards
     * before the HID client is offered them (and leaves them alone). */
    if (!(flags & INPUT_INIT_NO_KEYBOARD))
        inpw_keyboard_init();
    inpw_hid_init(!(flags & INPUT_INIT_NO_MOUSE), !(flags & INPUT_INIT_NO_GCADAPTER),
                  !(flags & INPUT_INIT_NO_HIDPADS));

    initialized = true;
    return 0;
}

void INPUT_Shutdown(void)
{
    if (!initialized)
        return;

    inpw_pads_shutdown();
    inpw_hid_shutdown();
    inpw_keyboard_shutdown();
    initialized = false;
}

void INPUT_SetScreenSize(int width, int height)
{
    if (width < 1 || height < 1)
        return;
    inpw_screen_w = width;
    inpw_screen_h = height;
    inpw_mouse_set_bounds(width, height);
}

void INPUT_Update(void)
{
    if (!initialized)
        return;

    /* Keyboard and mouse first so their maps are current for the pads. */
    inpw_keyboard_update();
    inpw_hid_update();
    inpw_mouse_update();
    inpw_pads_update();
    inpw_cursor_update();
}
