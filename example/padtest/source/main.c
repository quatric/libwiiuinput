/*
 * padtest: shows everything libwiiuinput reads, on the TV and the GamePad.
 *
 *   - every connected controller on its own line: buttons, sticks,
 *     triggers, battery and its own raw bits
 *   - the folded buttons, sticks, IR pointer, touch, cursor, mouse and
 *     keyboard, the way an app sees them
 *   - USB pen tablets and the uDraw: the pen's state, and a trail drawn
 *     where it touches (bigger with more pressure)
 *   - per Wii Remote channel, what KPAD and WPAD report, for finding out
 *     why an extension is not read
 *   - every driver's log, on screen and in sd:/padtest.log
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

#include <input_tablet.h>
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
#define C_PEN    0xF06090FF
#define C_HOVER  0xFFFFFFFF

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

/* What the drivers say: kept in a ring (they call from the system's HID
 * thread), shown on screen and written to sd:/padtest.log by the main loop. */
#define LOG_LINES 64
#define LOG_SHOWN 6
static char log_lines[LOG_LINES][112];
static volatile uint32_t log_count;
static uint32_t log_saved;

static int log_print(const char *fmt, ...)
{
    char line[112];
    va_list ap;
    int n;

    va_start(ap, fmt);
    n = vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    line[strcspn(line, "\n")] = 0;
    uint32_t i = log_count;
    memcpy(log_lines[i % LOG_LINES], line, sizeof(line));
    log_count = i + 1;
    return n;
}

static void save_log(void)
{
    uint32_t end = log_count;
    FILE *f;

    if (log_saved == end)
        return;
    if (end - log_saved > LOG_LINES)
        log_saved = end - LOG_LINES;
    f = fopen("fs:/vol/external01/padtest.log", log_saved ? "a" : "w");
    if (!f) {
        log_saved = end;
        return;
    }
    for (; log_saved != end; log_saved++)
        fprintf(f, "%s\n", log_lines[log_saved % LOG_LINES]);
    fclose(f);
}

/* One USB HID interface and what the library made of it. */
static void usb_line(const input_usb_device *u, char *out, size_t size)
{
    size_t at = (size_t)snprintf(out, size, "USB %04x:%04x if%u %u/%u %2uB %-18.18s rep %lu/%lu", u->vid, u->pid,
                                 u->interface, u->subclass, u->protocol, u->max_rx, u->what,
                                 (unsigned long)u->decoded, (unsigned long)u->reports);
    for (int i = 0; i < u->last_len && i < 12 && at + 4 < size; i++)
        at += (size_t)snprintf(out + at, size - at, "%s%02x", i ? " " : " | ", u->last[i]);
}

/* The USB table into the log whenever an entry appears or changes what it is. */
static void log_usb(void)
{
    static char said[16][32];
    static uint16_t said_id[16][2];
    static bool said_reports[16];
    int n = INPUT_USBDeviceCount();

    for (int i = 0; i < n && i < 16; i++) {
        input_usb_device u;
        char line[112];
        if (!INPUT_USBDeviceGet(i, &u))
            continue;
        if (!strcmp(said[i], u.what) && said_id[i][0] == u.vid && said_id[i][1] == u.pid &&
            said_reports[i] == (u.reports > 0))
            continue;
        snprintf(said[i], sizeof(said[i]), "%s", u.what);
        said_id[i][0] = u.vid;
        said_id[i][1] = u.pid;
        said_reports[i] = u.reports > 0;
        usb_line(&u, line, sizeof(line));
        log_print("%s\n", line);
    }
}

/* Where the pen touched, newest last, in TV coordinates. */
#define TRAIL 1024
static struct { int16_t x, y; uint8_t size; } trail[TRAIL];
static uint32_t trail_count;
static bool pen_hover;
static int pen_x, pen_y;

static void take_pen(void)
{
    tb_sample s;

    while (TB_Pop(&s)) {
        int x, y;
        TB_ToScreen(&s, TV_W, TV_H, &x, &y);
        pen_hover = s.in_range;
        pen_x = x;
        pen_y = y;
        if (s.buttons & TB_BUTTON_TIP) {
            uint32_t i = trail_count++ % TRAIL;
            trail[i].x = (int16_t)x;
            trail[i].y = (int16_t)y;
            trail[i].size = (uint8_t)(1 + s.pressure * 3);
        }
    }
}

static void dot(int x, int y, int size, uint32_t color)
{
    rect(SCREEN_TV, x - size, y - size, size * 2 + 1, size * 2 + 1, color);
    rect(SCREEN_DRC, x * DRC_W / TV_W - size / 2, y * DRC_H / TV_H - size / 2, size + 1, size + 1, color);
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
    text(1, 0, "A+B on a controller: rumble it. Hold HOME or Esc 2 s: exit. Log: sd:/padtest.log");

    text(2, 0, "Buttons");
    buttons_row(2, 9, held);

    INPUT_Stick(INPUT_STICK_LEFT, &lx, &ly, &lsrc);
    INPUT_Stick(INPUT_STICK_RIGHT, &rx, &ry, &rsrc);
    text(3, 0, "Source   %-10s  Gforce %.2f", INPUT_Source(), INPUT_Gforce());
    text(4, 0, "Sticks   L %+.2f %+.2f %-9s  R %+.2f %+.2f %s", lx, ly, lsrc, rx, ry, rsrc);

    bool ir = INPUT_Pointer(&px, &py), touch = INPUT_Touch(&tx, &ty), cur = INPUT_CursorActive();
    INPUT_CursorPosition(&cx, &cy);
    char irs[24] = "-", ts[24] = "-", cs[24] = "-";
    if (ir) snprintf(irs, sizeof(irs), "%d,%d", (int)px, (int)py);
    if (touch) snprintf(ts, sizeof(ts), "%d,%d", tx, ty);
    if (cur) snprintf(cs, sizeof(cs), "%d,%d", cx, cy);
    text(5, 0, "Pointer  IR %-10s Touch %-10s Cursor %s", irs, ts, cs);

    INPUT_MousePosition(&mx, &my);
    INPUT_MouseDelta(&dx, &dy, &wheel);
    text(6, 0, "Mouse    %-3s %4d,%-4d  buttons %c%c%c  wheel %+d", INPUT_MouseConnected() ? "yes" : "no", mx, my,
         (mb & INPUT_MOUSE_LEFT) ? 'L' : '-', (mb & INPUT_MOUSE_MIDDLE) ? 'M' : '-',
         (mb & INPUT_MOUSE_RIGHT) ? 'R' : '-', wheel);
    text(7, 0, "Keyboard %-3s %c%c%c%c  typed: %s_", INPUT_KeyboardConnected() ? "yes" : "no",
         (mods & INPUT_MOD_SHIFT) ? 'S' : '-', (mods & INPUT_MOD_CTRL) ? 'C' : '-', (mods & INPUT_MOD_ALT) ? 'A' : '-',
         (mods & INPUT_MOD_GUI) ? 'G' : '-', typed);
    text(8, 0, "Adapter  %-3s  USB pads %s", INPUT_GCAdapterConnected() ? "yes" : "no",
         INPUT_HIDPadConnected() ? "yes" : "no");

    /* Pen tablets and the uDraw. */
    tb_info ti;
    tb_sample ts_;
    if (TB_GetInfo(&ti)) {
        TB_GetState(&ts_);
        text(9, 0, "Tablet   %-.24s %s%d,%d p%.2f %s%s%s%s pad %04x", ti.name,
             ts_.wiimote >= 0 ? "(Wii Remote) " : "", (int)ts_.x, (int)ts_.y, ts_.pressure,
             ts_.in_range ? "near " : "away ", (ts_.buttons & TB_BUTTON_TIP) ? "TIP " : "",
             (ts_.buttons & TB_BUTTON_BARREL) ? "B1 " : "", (ts_.buttons & TB_BUTTON_BARREL2) ? "B2 " : "", ts_.pad);
    } else {
        text(9, 0, "Tablet   none (USB pen tablet, or a uDraw on a Wii Remote)");
    }

    /* What the system reports per Wii Remote channel. */
    for (int ch = 0; ch < 4; ch++) {
        input_remote_debug r;
        if (!INPUT_RemoteDebug(ch, &r))
            continue;
        if (r.kpad_error == -2 && r.wpad_error != 0) {
            text(10 + ch, 0, "Remote %d -", ch + 1);
            continue;
        }
        char id[24] = "";
        if (r.ext_id_read)
            snprintf(id, sizeof(id), " id %02x%02x%02x%02x%02x%02x", r.ext_id[0], r.ext_id[1], r.ext_id[2],
                     r.ext_id[3], r.ext_id[4], r.ext_id[5]);
        text(10 + ch, 0, "Remote %d KPAD %d n%d ext %02x | WPAD %d ext %02x fmt %02x | cl K%04x W%04x R%04x %c%s%s",
             ch + 1, r.kpad_error, r.kpad_samples, r.kpad_ext & 0xFF, r.wpad_error, r.wpad_ext & 0xFF, r.format & 0xFF,
             (unsigned)r.classic_kpad, (unsigned)r.classic_wpad, (unsigned)r.classic_reg,
             r.classic_from ? r.classic_from : '-', r.udraw ? " uDraw" : "", id);
    }

    int n = INPUT_DeviceCount();
    text(14, 0, "Controllers: %d", n);
    row = 15;
    for (int i = 0; i < n && row < 21; i++, row++) {
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
    /* The last rows take turns: the USB devices, then the log, 4 s each. */
    row = 21;
    int nusb = INPUT_USBDeviceCount();
    if (nusb > 0 && (OSTicksToMilliseconds(OSGetSystemTime()) / 4000) % 2 == 0) {
        char line[112];
        for (int i = 0; i < nusb && row < 27; i++) {
            input_usb_device u;
            if (INPUT_USBDeviceGet(i, &u)) {
                usb_line(&u, line, sizeof(line));
                text(row++, 0, "%s", line);
            }
        }
    } else {
        uint32_t end = log_count, first = end > LOG_SHOWN ? end - LOG_SHOWN : 0;
        for (uint32_t i = first; i < end; i++)
            text(row++, 0, "log: %.90s", log_lines[i % LOG_LINES]);
    }

    uint32_t shown = trail_count < TRAIL ? trail_count : TRAIL;
    for (uint32_t i = 0; i < shown; i++) {
        uint32_t k = (trail_count - shown + i) % TRAIL;
        dot(trail[k].x, trail[k].y, trail[k].size, C_PEN);
    }
    if (pen_hover)
        mark(pen_x, pen_y, 8, C_HOVER);

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
    TB_SetLog(log_print);
    INPUT_Init();
    TB_Init();
    INPUT_SetScreenSize(TV_W, TV_H);

    while (WHBProcIsRunning()) {
        INPUT_Update();
        take_pen();
        log_usb();
        save_log();

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

    TB_Shutdown();
    INPUT_Shutdown();
    save_log();

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
