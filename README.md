# libwiiuinput

Controller input library for the Wii U, built on wut. The Wii U port of
[libwiiinput](https://github.com/quatric/libwiiinput): the same calls, reading the Wii U's own
controllers.

- One call reads every controller and normalises it to `WPAD_BUTTON_*`: the Wii U GamePad, Wii
  Remotes with a Nunchuk or Classic Controller, Wii U Pro Controllers, the GameCube controller
  adapter for Wii U, and USB game controllers with Nintendont's `controller.ini` layouts (forty
  built in, more from files or the app, up to four at once)
- GamePad touch screen as a pointer (and optionally as a button), Wii Remote IR pointer smoothed
  with a One Euro filter, one cursor shared by touch, IR, mouse and stick
- Every controller one by one (`INPUT_DeviceCount/Get`): buttons, sticks, triggers, battery and the
  device's own bits, with rumble per controller (`INPUT_DeviceRumble`)
- USB keyboards through the system's keyboard library (held / pressed / released per key, typed
  characters, keyboard-to-controller mapping)
- USB mice: position (clamped, optional speed multiplier), deltas, wheel and buttons
- USB pen tablets (`input_tablet.h`): Wacom, Huion, XP-Pen and other drawing tablets, with
  pressure, tilt, eraser, tablet keys and Bamboo finger touch, and the Wii's uDraw GameTablet on a
  Wii Remote, through libwiiinput's `TB_*` calls
- `INPUT_RemoteDebug`: what KPAD and WPAD report per Wii Remote channel, for finding out why a
  controller or extension is not read
- Hot-plug safe throughout; nothing blocks the frame loop

## Build

Requires devkitPro with devkitPPC and wut.

```sh
export DEVKITPRO=/opt/devkitpro DEVKITPPC=/opt/devkitpro/devkitPPC
make            # produces lib/libinput-wiiu.a
make install    # copies into $DEVKITPRO/portlibs/wiiu
make padtest    # the controller test, example/padtest/padtest.wuhb
make test       # host test of the controller.ini reader (no devkitPro needed)
```

```sh
make WITH_GCADAPTER=0   # leave out the GameCube adapter
make WITH_HIDPADS=0     # leave out USB game controllers and their forty layouts
make WITH_TABLET=0      # leave out pen tablets and the uDraw
make WITH_MIC=0         # leave out the microphones
```

All three are on by default and need no other library. Link with `-linput-wiiu -lwut -lm`.

## Usage

```c
#include <input_wiiu.h>

OSEnableHomeButtonMenu(FALSE);             /* optional: read HOME as a button */
WHBProcInit();
INPUT_Init();
INPUT_SetScreenSize(1280, 720);

while (WHBProcIsRunning()) {
    INPUT_Update();                        /* once per frame */

    uint32_t down = INPUT_ButtonsDown();   /* WPAD_BUTTON_* from any device */
    if (down & WPAD_BUTTON_A) { }

    float x, y;
    INPUT_Stick(INPUT_STICK_LEFT, &x, &y, NULL);

    int cx, cy;
    if (INPUT_CursorActive()) INPUT_CursorPosition(&cx, &cy);   /* touch, IR or mouse */

    input_key_event ev;
    while (INPUT_KeyPop(&ev)) { /* ev.ch is typed ASCII */ }
}

INPUT_Shutdown();
WHBProcShutdown();
```

The API is libwiiinput's, so code written against one builds against the other with the header
name and integer types changed:

* Buttons are `WPAD_BUTTON_*` bits (`<padscore/wpad.h>`, the same values as libogc's) plus
  `INPUT_BUTTON_Z` / `INPUT_BUTTON_C` for the Nunchuk. The GamePad, Classic Controller, Pro
  Controller, GameCube pads and USB pads are folded onto the Wii Remote: A = A, B = B, X = 2, Y = 1,
  `+` / R / ZR = `+`, `-` / L / ZL = `-`, HOME = HOME (GameCube Start = HOME).
* Sticks are -1..1, y up, no deadzone; the strongest one across devices wins.
* `INPUT_Source()` names the device used last: `GamePad`, `Touch`, `Wiimote`, `Nunchuk`,
  `Classic`, `Wii U Pro`, `GameCube`, `USB Pad`, `Keyboard`, `Mouse`.
* `INPUT_SetCursorMode(INPUT_CURSOR_TOUCH | INPUT_CURSOR_IR | INPUT_CURSOR_MOUSE |
  INPUT_CURSOR_STICK)` picks what drives the cursor; touch wins over IR, IR over the mouse.
* `INPUT_Touch(&x, &y)` is the GamePad touch screen in screen coordinates;
  `INPUT_SetTouchButton(WPAD_BUTTON_A)` makes a touch press A as well.
* `INPUT_DeviceCount()` / `INPUT_DeviceGet(i, &dev)` list the controllers seen by the last update,
  `INPUT_DeviceRumble(i, on)` rumbles one of them.
* `INPUT_KeyboardMapSet/Clear`, `INPUT_GameCubeMapSet/Clear`, `INPUT_IgnoreWiimoteButtons` and
  `INPUT_StickRaw` work as in libwiiinput.
* `INPUT_InitEx(flags)` leaves parts out: `INPUT_INIT_NO_GAMEPAD`, `_NO_REMOTES`, `_NO_POINTER`,
  `_NO_KEYBOARD`, `_NO_MOUSE`, `_NO_GCADAPTER`, `_NO_HIDPADS`.

The library never changes the HOME button menu: with it on (the system default), HOME opens the
system's HOME menu and is never seen as a button.

### The GamePad and paired controllers

The GamePad is read through VPAD, and Wii Remotes, their extensions and Wii U Pro Controllers
through KPAD, with the Pro Controller and the Wii Remote both enabled (`WPADEnableURCC`,
`WPADEnableWiiRemote`). Both only report new packets, so a frame where none arrived keeps the
controller's last state instead of dropping it, and a press seen in any sample since the last frame
counts. This follows the Wii U driver of [snes9xgx](https://github.com/dborth/snes9xgx) (Daryl
Borth), as does the IR pointer's One Euro filter and the GamePad battery reading. The button layout
is libwiiinput's, which [GodModeU](https://github.com/quatric/GodModeU) also uses.

### GameCube controller adapter for Wii U

Nintendo's WUP-028 (and the adapters that copy it) is read through the system's HID library, so it
needs no special mode and can be plugged in at any time. Its four ports go into
`INPUT_ButtonsHeld/Down` and `INPUT_Stick` through `INPUT_GameCubeMapSet`, with the source
`"GameCube"`. `INPUT_GCAdapterPad` gives a port's raw state (buttons, sticks, analogue triggers,
WaveBird or not), `INPUT_GCAdapterRumble` rumbles one (with the grey power cable plugged in),
`INPUT_GCAdapterConnected` says whether it is reporting and `INPUT_GCAdapterSetLog(printf)` shows
what the driver does. Sticks centre on where they rest when a controller is plugged in; X+Y+Start
held for three seconds takes the centre again. The report format and commands are libwiiinput's
(from RiftWii, after Dolphin's adapter code and wup-028-bslug).

### USB game controllers (Nintendont's layouts)

The pads Nintendont reads through its `controller.ini` files: the Logitech F310 and RumblePad 2,
PlayStation 2, 3, 4 and 5 pads, 8BitDo, Nvidia Shield, Switch, PDP, PowerA, Hori and Qanba pads,
Mayflash adapters, Thrustmaster and Saitek and more (see `source/libinput_hidpads_data.c`). Up to
four at once, folded into the button masks through the same GameCube mapping, with the source
`"USB Pad"`. `INPUT_HIDPadPad/Name/Rumble/Connected/SetLog` work as in libwiiinput.

For any other pad a layout in Nintendont's format is looked for, first among those given to
`INPUT_HIDPadAddConfig(text, length)` (call it before `INPUT_Init`: the system offers a device once,
when it is plugged in), then at `/vol/external01/controllers/VVVV_PPPP.ini` (the ids in hex) and
`/vol/external01/controller.ini` on the SD card (every `.ini` in that folder, read once by
`INPUT_Init`, so the system's HID thread never waits on the card). `INPUT_HIDPadSetLog(printf)`
also names every USB device the system offers and why it was or was not taken, which is the place to
look when a pad is not seen. A pad with no layout (or whose reports are shorter than its layout
needs) is read from its own HID report descriptor instead, as `"generic USB pad"`: X/Y, the right
stick (Rx/Ry, or else Z/Rz), the hat and buttons 1-16 (1 Y, 2 B, 3 A, 4 X, 5/7 L, 6/8 R, 9 Z, 10/13
Start; `INPUT_HIDPadRaw(port)` gives them by number). Keyboards, boot mice and tablet makers'
devices are never taken. The layouts and report conversion (`source/libinput_hid.c`) are
libwiiinput's, unchanged. Unlike on the Wii, the PlayStation 3's pad works too: it is switched on
with Nintendont's feature report and polled with GET_REPORT once a frame. Layouts whose reports
carry several pads (`MultiIn` 2 to 4) are not read.

### Pen tablets and the uDraw

```c
#include <input_tablet.h>

INPUT_Init();
TB_Init();                                 /* finds a USB tablet, now or when plugged in */

tb_sample s;
while (TB_Pop(&s)) {                       /* every report */
    int x, y;
    TB_ToScreen(&s, 1280, 720, &x, &y);
    if (s.buttons & TB_BUTTON_TIP) { /* draw, s.pressure is 0..1 */ }
}
```

The same calls as libwiiinput, on the system's HID library: the report descriptor parser and the
report decoding are libwiiinput's, unchanged. A tablet maker's device (Wacom, Huion and Gaomon,
XP-Pen and UGEE, Veikk, UC-Logic, Waltop) is left to this driver rather than the mouse driver,
and each of its interfaces is told apart by its report descriptor: the pen, a Bamboo's touch
surface, or nothing to read. Old fixed-format Wacoms are switched into tablet mode as before.

The uDraw GameTablet (the Wii's) plugs into a Wii Remote, but the system's KPAD cannot read it.
Its input is still in the extension's registers, as on every Wii Remote extension, so the driver
reads its ID once (`WPADReadExtReg`, 0xFA) and then its six data bytes, one read at a time, for as
long as it is plugged in; `TB_*` then give it as on the Wii, with `wiimote` set. It needs only
`INPUT_Init`. Its device entry is named `"uDraw"`.

### When something is not read

`INPUT_RemoteDebug(chan, &r)` gives what the system said about a Wii Remote channel this frame:
KPAD's error, sample count and extension, WPAD's own extension and data format, the Classic
Controller's buttons through KPAD and through WPAD, and the ID bytes of an extension the system
does not know. A Classic Controller (Classic, Classic Pro, and the NES / SNES Classic Mini pads) is
also read straight from its own registers over Bluetooth, as the uDraw is, and that reading wins
while it is fresh; `classic_from` says which was used (`K`PAD, `W`PAD or `R`egisters).

`INPUT_USBDeviceCount()` / `INPUT_USBDeviceGet(i, &u)` list every USB HID interface the system
offered: VID:PID, interface, subclass/protocol, report size, what the library made of it, how many
reports came and were read, and the start of the last one. The test program shows all of this, and writes every driver's log to
`sd:/padtest.log`.

### Microphones

Three drivers with the same calls (open, start, read what has come, stop, close), all 16-bit mono:

```c
#include <input_drcmic.h>   /* DM_*: the GamePad's own microphone, through mic.rpl */
#include <input_usbmic.h>   /* UM_*: any class-compliant (UAC 1.0) USB microphone */
#include <input_wiispeak.h> /* WS_*: the Wii Speak, either USB port */
```

The USB microphone and the Wii Speak are libwiiinput's drivers, moved from IOS's `/dev/usb/oh0`
onto the Wii U's USB host stack (`nsysuhs`), isochronous streams included. The Wii Speak needs its
boot firmware, which is Nintendo's and never in this repository: pass it to `WS_Open`, or build it in
with `PMIC_FIRM=<file>` (or `data/pmic_firm.bin`, which `tools/extract_pmic_firmware.py` takes out
of the RVL SDK's `pmic.a`) and use `WS_BuiltinFirmware`. The test program loads it from
`sd:/pmic_firm.bin` and shows a level meter for each microphone. `make WITH_MIC=0` leaves them out.

### Keyboard and mouse

Keyboards go through the system's keyboard library (`nsyskbd`), which also turns keys into
characters for the console's keyboard layout; `INPUT_KeyPop` gives them as `ev.ch` (US layout
when the system gives none). The system drives the keyboard LEDs itself, so
`INPUT_KeyboardSetLed` always fails. Mice are switched to the HID boot protocol, as snes9xgx does,
so any USB mouse works without its report descriptor; the wheel is read from the byte most mice
send after the boot fields.

### The test program

`example/padtest` shows everything the library reads, on the TV and the GamePad: every controller
on its own line (buttons, sticks, triggers, battery and raw bits), the folded buttons with the held
ones highlighted, the sticks, the IR pointer, touch, cursor (drawn on screen), mouse, keyboard
(with what was typed), the tablet or uDraw (with a trail where the pen touches, thicker with more
pressure), what KPAD and WPAD report per Wii Remote, and every driver's log (also written to
`sd:/padtest.log`). A+B held on a controller rumbles
that controller; HOME (or Esc) held for two seconds exits. `make` there (or `make padtest` here)
gives `padtest.wuhb` for the Homebrew Launcher.

### Using it from an app

Add it as a git submodule and either link `lib/libinput-wiiu.a` or compile `source/` with your own
flags (`-DHAVE_GCADAPTER -DHAVE_HIDPADS` for those parts):

```sh
git submodule add https://github.com/quatric/libwiiuinput.git libwiiuinput
```

## Status

Builds cleanly with wut and the test program links. A Wacom Intuos S and the uDraw have been read on
a Wii U; the Classic Controller register reading and generic USB pads are still to be confirmed.

## License

AGPL-3.0, see `LICENSE`, like libwiiinput. The files taken from RiftWii through libwiiinput
(`libinput_hid.{c,h}`, `libinput_hidpads_data.c`) and the HID client that uses them (`libinput_hidclient.c`) are
GPL-3.0-or-later, which combines with the AGPL-3.0. The controller layouts in
`libinput_hidpads_data.c` are Nintendont's (FIX94 and contributors,
<https://github.com/FIX94/Nintendont>), used with its developers' permission.
