/* USB keyboards through the system's keyboard library (nsyskbd). Its
 * callbacks arrive on the system's own thread; everything they touch is
 * kept under an uninterruptible spinlock and copied out once a frame. */
#include <string.h>
#include <coreinit/spinlock.h>
#include <nsyskbd/nsyskbd.h>
#include "libinput_internal.h"

#define KEY_BYTES (256 / 8)
#define QUEUE_LEN 64

#define BIT_GET(a, k) ((a)[(k) >> 3] & (1 << ((k) & 7)))
#define BIT_SET(a, k) ((a)[(k) >> 3] |= (1 << ((k) & 7)))
#define BIT_CLR(a, k) ((a)[(k) >> 3] &= ~(1 << ((k) & 7)))

static OSSpinLock lock;

/* Written by the callbacks under `lock`. */
static uint8_t live[KEY_BYTES];
static uint8_t pressed_acc[KEY_BYTES];
static uint8_t released_acc[KEY_BYTES];
static input_key_event queue[QUEUE_LEN];
static int q_head, q_tail;
static int keyboards;          /* attached channels */
static uint8_t channel_mask;   /* which channels are attached */
static bool caps;
static volatile bool typed;    /* a key went down: the main thread names the source */

/* Snapshot published by inpw_keyboard_update(). */
static uint8_t held[KEY_BYTES];
static uint8_t pressed[KEY_BYTES];
static uint8_t released[KEY_BYTES];

static uint32_t key_map[256];
static bool kbd_initialized;

static const char plain[]   = "abcdefghijklmnopqrstuvwxyz1234567890\n\x1b\b\t -=[]\\#;'`,./";
static const char shifted[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ!@#$%^&*()\n\x1b\b\t _+{}|~:\"~<>?";

static uint8_t mods_of(const uint8_t *keys)
{
    uint8_t m = 0;
    if (BIT_GET(keys, 0xE1) || BIT_GET(keys, 0xE5)) m |= INPUT_MOD_SHIFT;
    if (BIT_GET(keys, 0xE0) || BIT_GET(keys, 0xE4)) m |= INPUT_MOD_CTRL;
    if (BIT_GET(keys, 0xE2) || BIT_GET(keys, 0xE6)) m |= INPUT_MOD_ALT;
    if (BIT_GET(keys, 0xE3) || BIT_GET(keys, 0xE7)) m |= INPUT_MOD_GUI;
    return m;
}

/* US layout, for when the system gives no character. */
static char key_char(uint8_t code, bool shift)
{
    char c = 0;

    if (code >= 0x04 && code <= 0x38) {
        c = (shift ? shifted : plain)[code - 0x04];
        if (code <= 0x1D && caps)
            c = (c >= 'a' && c <= 'z') ? c - 32 : c + 32;
    } else if (code >= 0x59 && code <= 0x61) {
        c = '1' + (code - 0x59);
    } else {
        switch (code) {
        case 0x54: c = '/'; break;
        case 0x55: c = '*'; break;
        case 0x56: c = '-'; break;
        case 0x57: c = '+'; break;
        case 0x62: c = '0'; break;
        case 0x63: c = '.'; break;
        }
    }
    return (c >= 0x20 && c < 0x7F) ? c : 0;
}

static void on_attach(KBDAttachEvent *ev)
{
    OSUninterruptibleSpinLock_Acquire(&lock);
    if (ev->channel < 8 && !(channel_mask & (1u << ev->channel))) {
        channel_mask |= 1u << ev->channel;
        keyboards++;
    }
    OSUninterruptibleSpinLock_Release(&lock);
}

static void on_detach(KBDAttachEvent *ev)
{
    OSUninterruptibleSpinLock_Acquire(&lock);
    if (ev->channel < 8 && (channel_mask & (1u << ev->channel))) {
        channel_mask &= ~(1u << ev->channel);
        keyboards--;
    }
    /* Whatever was held is let go. */
    for (int i = 0; i < 256; i++)
        if (BIT_GET(live, i))
            BIT_SET(released_acc, i);
    memset(live, 0, sizeof(live));
    OSUninterruptibleSpinLock_Release(&lock);
}

static void on_key(KBDKeyEvent *ev)
{
    uint8_t k = ev->hidCode;

    OSUninterruptibleSpinLock_Acquire(&lock);
    if (ev->isPressedDown) {
        bool fresh = !BIT_GET(live, k);
        BIT_SET(live, k);
        if (fresh) {
            BIT_SET(pressed_acc, k);
            typed = true;
            if (k == 0x39) { /* Caps Lock */
                caps = !caps;
            } else if (k < 0xE0 || k > 0xE7) {
                int next = (q_head + 1) % QUEUE_LEN;
                if (next != q_tail) {
                    uint8_t mods = mods_of(live);
                    uint16_t u = ev->asUTF16Character;
                    char c = (u >= 0x20 && u < 0x7F) ? (char)u : key_char(k, mods & INPUT_MOD_SHIFT);
                    queue[q_head].code = k;
                    queue[q_head].mods = mods;
                    queue[q_head].ch = (mods & INPUT_MOD_CTRL) ? 0 : c;
                    q_head = next;
                }
            }
        }
    } else {
        if (BIT_GET(live, k))
            BIT_SET(released_acc, k);
        BIT_CLR(live, k);
    }
    OSUninterruptibleSpinLock_Release(&lock);
}

void INPUT_KeyboardMapClear(void)
{
    memset(key_map, 0, sizeof(key_map));
}

void INPUT_KeyboardMapSet(uint8_t keycode, uint32_t button)
{
    key_map[keycode] = button;
}

static void map_defaults(void)
{
    INPUT_KeyboardMapClear();
    INPUT_KeyboardMapSet(0x52, WPAD_BUTTON_UP);
    INPUT_KeyboardMapSet(0x51, WPAD_BUTTON_DOWN);
    INPUT_KeyboardMapSet(0x50, WPAD_BUTTON_LEFT);
    INPUT_KeyboardMapSet(0x4F, WPAD_BUTTON_RIGHT);
    INPUT_KeyboardMapSet(0x28, WPAD_BUTTON_A);     /* Enter */
    INPUT_KeyboardMapSet(0x58, WPAD_BUTTON_A);     /* Enter (numpad) */
    INPUT_KeyboardMapSet(0x2A, WPAD_BUTTON_B);     /* Backspace */
    INPUT_KeyboardMapSet(0x1B, WPAD_BUTTON_1);     /* X */
    INPUT_KeyboardMapSet(0x1C, WPAD_BUTTON_2);     /* Y */
    INPUT_KeyboardMapSet(0x4C, WPAD_BUTTON_MINUS); /* Delete */
    INPUT_KeyboardMapSet(0x29, WPAD_BUTTON_HOME);  /* Esc */
    INPUT_KeyboardMapSet(0x4A, WPAD_BUTTON_HOME);  /* Home */
}

int32_t inpw_keyboard_init(void)
{
    if (kbd_initialized)
        return 0;

    OSInitSpinLock(&lock);
    memset(live, 0, sizeof(live));
    memset(pressed_acc, 0, sizeof(pressed_acc));
    memset(released_acc, 0, sizeof(released_acc));
    memset(held, 0, sizeof(held));
    memset(pressed, 0, sizeof(pressed));
    memset(released, 0, sizeof(released));
    q_head = q_tail = 0;
    keyboards = 0;
    channel_mask = 0;
    caps = false;
    map_defaults();

    /* Fails when the app set the keyboard library up itself: then the
     * keys go to its callbacks, and this side reports no keyboard. */
    if (KBDSetup(on_attach, on_detach, on_key) != KDB_ERROR_NONE)
        return -1;
    kbd_initialized = true;
    return 0;
}

void inpw_keyboard_shutdown(void)
{
    if (!kbd_initialized)
        return;
    KBDTeardown();
    kbd_initialized = false;
    keyboards = 0;
    channel_mask = 0;
}

void inpw_keyboard_update(void)
{
    if (!kbd_initialized)
        return;

    OSUninterruptibleSpinLock_Acquire(&lock);
    memcpy(held, live, sizeof(held));
    memcpy(pressed, pressed_acc, sizeof(pressed));
    memcpy(released, released_acc, sizeof(released));
    memset(pressed_acc, 0, sizeof(pressed_acc));
    memset(released_acc, 0, sizeof(released_acc));
    OSUninterruptibleSpinLock_Release(&lock);

    if (typed) {
        typed = false;
        inpw_set_source("Keyboard");
    }
}

bool INPUT_KeyboardConnected(void)
{
    return kbd_initialized && keyboards > 0;
}

bool    INPUT_KeyHeld(uint8_t keycode)     { return BIT_GET(held, keycode) != 0; }
bool    INPUT_KeyPressed(uint8_t keycode)  { return BIT_GET(pressed, keycode) != 0; }
bool    INPUT_KeyReleased(uint8_t keycode) { return BIT_GET(released, keycode) != 0; }
uint8_t INPUT_KeyModifiers(void)           { return mods_of(held); }

bool INPUT_KeyPop(input_key_event *ev)
{
    bool ok = false;

    if (!kbd_initialized)
        return false;

    OSUninterruptibleSpinLock_Acquire(&lock);
    if (q_tail != q_head) {
        if (ev)
            *ev = queue[q_tail];
        q_tail = (q_tail + 1) % QUEUE_LEN;
        ok = true;
    }
    OSUninterruptibleSpinLock_Release(&lock);
    return ok;
}

int32_t INPUT_KeyboardSetLed(input_led led, bool on)
{
    (void)led;
    (void)on;
    return -1; /* nsyskbd keeps the LEDs itself */
}

static uint32_t pad_map(const uint8_t *keys)
{
    uint32_t out = 0;

    for (int i = 0; i < 256; i++) {
        if (key_map[i] && BIT_GET(keys, i))
            out |= key_map[i];
    }
    return out;
}

uint32_t INPUT_KeyboardPadHeld(void) { return pad_map(held); }
uint32_t INPUT_KeyboardPadDown(void) { return pad_map(pressed); }
