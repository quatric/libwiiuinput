/*
 * padtest: shows everything libwiiuinput reads, on the TV and the GamePad.
 *
 *   - every connected controller on its own line: buttons, sticks,
 *     triggers, battery and its own raw bits
 *   - the folded buttons, sticks, IR pointer, touch, cursor, mouse and
 *     keyboard, the way an app sees them
 *   - A+B held on a controller rumbles that controller
 *   - HOME (or Esc) held for two seconds exits
 */
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include <coreinit/cache.h>
#include <coreinit/memfrmheap.h>
#include <coreinit/memheap.h>
#include <coreinit/screen.h>
#include <coreinit/thread.h>
#include <coreinit/time.h>
#include <coreinit/title.h>
#include <proc_ui/procui.h>
#include <sysapp/launch.h>
#include <whb/proc.h>

#include <input_wiiu.h>

#define HEAP_TAG 0x50414454 /* 'PADT' */
#define FONT_X0 50          /* OSScreen's TV text cells: 12 x 24 from (50, 32) */
#define FONT_Y0 32
#define CELL_W 12
#define CELL_H 24
#define TV_W 1280
#define TV_H 720
#define DRC_W 854
#define DRC_H 480

#define C_BG     0x10141CFF
#define C_HELD   0x2E8B3AFF
#define C_CURSOR 0xFFD040FF
#define C_IR     0x40C0FFFF

static void *screen_mem;
static uint32_t tv_size, drc_size;
static bool screen_ready;

/* ---- Screen -------------------------------------------------------------- */

/* OSScreen's buffers live in the MEM1 frame heap, which the system takes
 * back whenever the app leaves the foreground (as in GodModeU). */
static uint32_t screen_acquired(void *ctx)
{
    (void)ctx;
    MEMHeapHandle heap = MEMGetBaseHeapHandle(MEM_BASE_HEAP_MEM1);
    MEMRecordStateForFrmHeap(heap, HEAP_TAG);
    tv_size = OSScreenGetBufferSizeEx(SCREEN_TV);
    drc_size = OSScreenGetBufferSizeEx(SCREEN_DRC);
    screen_mem = MEMAllocFromFrmHeapEx(heap, tv_size + drc_size, 0x100);
    if (!screen_mem)
        return 0;
    OSScreenSetBufferEx(SCREEN_TV, screen_mem);
    OSScreenSetBufferEx(SCREEN_DRC, (uint8_t *)screen_mem + tv_size);
    OSScreenEnableEx(SCREEN_TV, TRUE);
    OSScreenEnableEx(SCREEN_DRC, TRUE);
    screen_ready = true;
    return 0;
}

static uint32_t screen_released(void *ctx)
{
    (void)ctx;
    if (!screen_ready)
        return 0;
    screen_ready = false;
    MEMFreeByStateToFrmHeap(MEMGetBaseHeapHandle(MEM_BASE_HEAP_MEM1), HEAP_TAG);
    screen_mem = NULL;
    return 0;
}

static void rect(OSScreenID s, int x0, int y0, int w, int h, uint32_t color)
{
    int sw = s == SCREEN_TV ? TV_W : DRC_W, sh = s == SCREEN_TV ? TV_H : DRC_H;
    for (int y = y0; y < y0 + h; y++)
        for (int x = x0; x < x0 + w; x++)
            if (x >= 0 && y >= 0 && x < sw && y < sh)
                OSScreenPutPixelEx(s, x, y, color);
}

/* A mark at a point given in TV coordinates, on both screens. */
static void mark(int x, int y, int size, uint32_t color)
{
    rect(SCREEN_TV, x - size, y - 1, size * 2 + 1, 3, color);
    rect(SCREEN_TV, x - 1, y - size, 3, size * 2 + 1, color);
    x = x * DRC_W / TV_W;
    y = y * DRC_H / TV_H;
    size = size * 2 / 3;
    rect(SCREEN_DRC, x - size, y - 1, size * 2 + 1, 3, color);
    rect(SCREEN_DRC, x - 1, y - size, 3, size * 2 + 1, color);
}

static void text(int row, int col, const char *fmt, ...)
{
    char line[160];
    va_list ap;

    va_start(ap, fmt);
    vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    OSScreenPutFontEx(SCREEN_TV, col, row, line);
    OSScreenPutFontEx(SCREEN_DRC, col, row, line);
}

/* A cell highlighted behind the text, on the TV where the grid is known. */
static void highlight(int row, int col, int len)
{
    rect(SCREEN_TV, FONT_X0 + col * CELL_W, FONT_Y0 + row * CELL_H, len * CELL_W, CELL_H, C_HELD);
}

/* ---- What is shown -------------------------------------------------------- */

static const struct { uint32_t bit; const char *name; } BUTTONS[] = {
    { WPAD_BUTTON_A, "A" },      { WPAD_BUTTON_B, "B" },       { WPAD_BUTTON_1, "1" },
    { WPAD_BUTTON_2, "2" },      { WPAD_BUTTON_MINUS, "-" },   { WPAD_BUTTON_PLUS, "+" },
    { WPAD_BUTTON_HOME, "HOME" }, { WPAD_BUTTON_UP, "UP" },    { WPAD_BUTTON_DOWN, "DOWN" },
    { WPAD_BUTTON_LEFT, "LEFT" }, { WPAD_BUTTON_RIGHT, "RIGHT" }, { INPUT_BUTTON_Z, "Z" },
    { INPUT_BUTTON_C, "C" },
};
#define NBUTTONS (int)(sizeof(BUTTONS) / sizeof(BUTTONS[0]))

/* The buttons row: every name, the held ones highlighted and bracketed. */
static void buttons_row(int row, int col, uint32_t held)
{
    char line[128];
    int at = 0;

    for (int i = 0; i < NBUTTONS; i++) {
        bool on = (held & BUTTONS[i].bit) != 0;
        int len = (int)strlen(BUTTONS[i].name) + 2;
        if (on)
            highlight(row, col + at, len);
        at += snprintf(line + at, sizeof(line) - at, on ? "[%s]" : " %s ", BUTTONS[i].name);
    }
    text(row, col, "%s", line);
}

/* The held buttons of one device, short names, for its line. */
static void held_names(uint32_t held, char *out, size_t size)
{
    size_t at = 0;

    out[0] = 0;
    for (int i = 0; i < NBUTTONS && at < size; i++)
        if (held & BUTTONS[i].bit)
            at += snprintf(out + at, size - at, "%s ", BUTTONS[i].name);
}

/* What the GameCube adapter and USB pad drivers say, the last few lines. */
static char log_lines[3][96];
static volatile int log_next;

static int log_print(const char *fmt, ...)
{
    char line[96];
    va_list ap;
    int n;

    va_start(ap, fmt);
    n = vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    line[strcspn(line, "\n")] = 0;
    int i = log_next % 3;
    memcpy(log_lines[i], line, sizeof(line));
    log_next = i + 1;
    return n;
}

static char typed[40];

static void draw(void)
{
    float lx, ly, rx, ry, px, py;
    const char *lsrc, *rsrc;
    int tx, ty, cx, cy, mx, my, dx, dy, wheel, row;
    uint32_t held = INPUT_ButtonsHeld();
    uint8_t mods = INPUT_KeyModifiers(), mb = INPUT_MouseButtonsHeld();

    OSScreenClearBufferEx(SCREEN_TV, C_BG);
    OSScreenClearBufferEx(SCREEN_DRC, C_BG);

    text(0, 0, "libwiiuinput controller test");
    text(1, 0, "A+B on a controller: rumble it. Hold HOME or Esc 2 s: exit.");

    text(3, 0, "Buttons");
    buttons_row(3, 9, held);

    INPUT_Stick(INPUT_STICK_LEFT, &lx, &ly, &lsrc);
    INPUT_Stick(INPUT_STICK_RIGHT, &rx, &ry, &rsrc);
    text(4, 0, "Source   %-10s  Gforce %.2f", INPUT_Source(), INPUT_Gforce());
    text(5, 0, "Sticks   L %+.2f %+.2f %-9s  R %+.2f %+.2f %s", lx, ly, lsrc, rx, ry, rsrc);

    bool ir = INPUT_Pointer(&px, &py), touch = INPUT_Touch(&tx, &ty), cur = INPUT_CursorActive();
    INPUT_CursorPosition(&cx, &cy);
    char irs[24] = "-", ts[24] = "-", cs[24] = "-";
    if (ir) snprintf(irs, sizeof(irs), "%d,%d", (int)px, (int)py);
    if (touch) snprintf(ts, sizeof(ts), "%d,%d", tx, ty);
    if (cur) snprintf(cs, sizeof(cs), "%d,%d", cx, cy);
    text(6, 0, "Pointer  IR %-10s Touch %-10s Cursor %s", irs, ts, cs);

    INPUT_MousePosition(&mx, &my);
    INPUT_MouseDelta(&dx, &dy, &wheel);
    text(7, 0, "Mouse    %-3s %4d,%-4d  buttons %c%c%c  wheel %+d", INPUT_MouseConnected() ? "yes" : "no", mx, my,
         (mb & INPUT_MOUSE_LEFT) ? 'L' : '-', (mb & INPUT_MOUSE_MIDDLE) ? 'M' : '-',
         (mb & INPUT_MOUSE_RIGHT) ? 'R' : '-', wheel);
    text(8, 0, "Keyboard %-3s %c%c%c%c  typed: %s_", INPUT_KeyboardConnected() ? "yes" : "no",
         (mods & INPUT_MOD_SHIFT) ? 'S' : '-', (mods & INPUT_MOD_CTRL) ? 'C' : '-', (mods & INPUT_MOD_ALT) ? 'A' : '-',
         (mods & INPUT_MOD_GUI) ? 'G' : '-', typed);
    text(9, 0, "Adapter  %-3s  USB pads %s", INPUT_GCAdapterConnected() ? "yes" : "no",
         INPUT_HIDPadConnected() ? "yes" : "no");

    int n = INPUT_DeviceCount();
    text(11, 0, "Controllers: %d", n);
    row = 12;
    for (int i = 0; i < n && row < 26; i++, row++) {
        input_device d;
        char names[64], bat[16] = " - ";
        INPUT_DeviceGet(i, &d);
        held_names(d.held, names, sizeof(names));
        if (d.battery >= 0)
            snprintf(bat, sizeof(bat), "%3d%%", d.battery);
        text(row, 0, "%d %-9s %d %s raw %08x%s%04x L%+.1f%+.1f R%+.1f%+.1f T%.1f/%.1f %s", i + 1, d.name, d.port + 1,
             bat, (unsigned)d.raw, d.raw_ext ? "+" : " ", (unsigned)(d.raw_ext & 0xFFFF), d.lx, d.ly, d.rx, d.ry,
             d.lt, d.rt, names);
        if (d.held)
            highlight(row, 0, 1);
        if (d.pointer)
            mark((int)d.px, (int)d.py, 10, d.kind == INPUT_DEV_GAMEPAD ? C_HELD : C_IR);
    }
    for (int i = 0; i < 3 && row < 26; i++) {
        const char *l = log_lines[(log_next + i) % 3];
        if (l[0])
            text(row++, 0, "log: %s", l);
    }

    if (cur)
        mark(cx, cy, 16, C_CURSOR);

    DCFlushRange(screen_mem, tv_size + drc_size);
    OSScreenFlipBuffersEx(SCREEN_TV);
    OSScreenFlipBuffersEx(SCREEN_DRC);
}

/* ---- Main ---------------------------------------------------------------- */

/* The same title IDs WHBProcInit treats as the Homebrew Launcher. */
static bool launched_from_hbl(void)
{
    uint64_t id = OSGetTitleID();
    return id == 0x0005000013374842ULL || id == 0x000500101004A000ULL || id == 0x000500101004A100ULL ||
           id == 0x000500101004A200ULL;
}

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    bool leaving = false;
    OSTime home_since = 0;

    /* HOME is read as a button here, so the system's HOME menu is off. */
    OSEnableHomeButtonMenu(FALSE);
    WHBProcInit();
    OSScreenInit();
    screen_acquired(NULL);
    ProcUIRegisterCallback(PROCUI_CALLBACK_ACQUIRE, screen_acquired, NULL, 100);
    ProcUIRegisterCallback(PROCUI_CALLBACK_RELEASE, screen_released, NULL, 100);

    INPUT_GCAdapterSetLog(log_print);
    INPUT_HIDPadSetLog(log_print);
    INPUT_Init();
    INPUT_SetScreenSize(TV_W, TV_H);

    while (WHBProcIsRunning()) {
        INPUT_Update();

        input_key_event ev;
        while (INPUT_KeyPop(&ev)) {
            size_t len = strlen(typed);
            if (ev.code == 0x2A && len) /* Backspace */
                typed[len - 1] = 0;
            else if (ev.ch) {
                if (len >= sizeof(typed) - 1) {
                    memmove(typed, typed + 1, len);
                    len--;
                }
                typed[len] = ev.ch;
                typed[len + 1] = 0;
            }
        }

        for (int i = 0; i < INPUT_DeviceCount(); i++) {
            input_device d;
            INPUT_DeviceGet(i, &d);
            INPUT_DeviceRumble(i, (d.held & (WPAD_BUTTON_A | WPAD_BUTTON_B)) == (WPAD_BUTTON_A | WPAD_BUTTON_B));
        }

        if (INPUT_ButtonsHeld() & WPAD_BUTTON_HOME) {
            if (!home_since)
                home_since = OSGetSystemTime();
            else if (OSTicksToMilliseconds(OSGetSystemTime() - home_since) >= 2000)
                leaving = true;
        } else {
            home_since = 0;
        }

        if (screen_ready)
            draw();
        if (leaving)
            break;
        OSSleepTicks(OSMillisecondsToTicks(16));
    }

    INPUT_Shutdown();

    /* Outside the Homebrew Launcher a title may only leave through the Wii U
     * Menu, with ProcUI running (as GodModeU does). */
    if (leaving && !launched_from_hbl()) {
        SYSLaunchMenu();
        while (WHBProcIsRunning()) {}
    }
    if (screen_ready) {
        OSScreenShutdown();
        screen_released(NULL);
    }
    WHBProcShutdown();
    return 0;
}
