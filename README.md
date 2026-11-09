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
```

Both are on by default and need no other library. Link with `-linput-wiiu -lwut -lm`.

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
`/vol/external01/controller.ini` on the SD card. Only a device with a layout is taken, so keyboards
and mice stay with their drivers. The layouts and report conversion (`source/libinput_hid.c`) are
libwiiinput's, unchanged. Unlike on the Wii, the PlayStation 3's pad works too: it is switched on
with Nintendont's feature report and polled with GET_REPORT once a frame. Layouts whose reports
carry several pads (`MultiIn` 2 to 4) are not read.

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
(with what was typed) and the adapter and USB pad drivers' log. A+B held on a controller rumbles
that controller; HOME (or Esc) held for two seconds exits. `make` there (or `make padtest` here)
gives `padtest.wuhb` for the Homebrew Launcher.

### Using it from an app

Add it as a git submodule and either link `lib/libinput-wiiu.a` or compile `source/` with your own
flags (`-DHAVE_GCADAPTER -DHAVE_HIDPADS` for those parts):

```sh
git submodule add https://github.com/quatric/libwiiuinput.git libwiiuinput
```

## Status

Builds cleanly with wut and the test program links; not yet tested on real hardware or in Cemu.

## License

AGPL-3.0, see `LICENSE`, like libwiiinput. The files taken from RiftWii through libwiiinput
(`libinput_hid*.{c,h}`) and the HID client that uses them (`libinput_hidclient.c`) are
GPL-3.0-or-later, which combines with the AGPL-3.0. The controller layouts in
`libinput_hidpads_data.c` are Nintendont's (FIX94 and contributors,
<https://github.com/FIX94/Nintendont>), used with its developers' permission.
