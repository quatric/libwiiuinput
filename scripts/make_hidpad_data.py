#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 RiftWii contributors
# SPDX-License-Identifier: GPL-3.0-or-later
"""Writes source/libinput_hidpads_data.c from Nintendont's controllerconfigs
(github.com/FIX94/Nintendont, the *.ini files in controllers/): the USB pad
layouts the library knows without a file on the SD card. Modelled on
RiftWii's tools/make_builtin_pads.py.

The layouts are kept as Nintendont has them, one entry per VID and PID.
Left out:
  - Nintendo's Wii U GameCube adapter (057E:0337), which libinput_gcad.c
    drives itself;
  - files with MultiIn=2, 3 or 4, whose reports carry several pads.
Two files with the same IDs: PREFERRED names the one kept.

    scripts/make_hidpad_data.py path/to/nintendont-controllers
"""
import pathlib
import re
import sys

OUT = pathlib.Path(__file__).resolve().parent.parent / "source" / "libinput_hidpads_data.c"

# The 8BitDo SNES30 Pro answers with a DualShock 4's IDs; the DualShock 4
# is the pad most people have with them.
PREFERRED = {0x054C05C4: "controller_ps4.ini"}
SKIPPED_IDS = {0x057E0337}


def value(text, key):
    match = re.search(r"^\s*" + key + r"\s*=\s*([0-9A-Fa-f]+)\s*$", text, re.MULTILINE | re.IGNORECASE)
    return match.group(1) if match else None


def c_string(text):
    out = []
    for line in text.splitlines(keepends=True):
        escaped = line.replace("\\", "\\\\").replace('"', '\\"').replace("\r", "\\r").replace("\n", "\\n")
        out.append('        "' + escaped + '"')
    return "\n".join(out) if out else '        ""'


def main():
    if len(sys.argv) != 2:
        raise SystemExit(__doc__)
    src = pathlib.Path(sys.argv[1])
    entries = {}
    for path in sorted(src.glob("*.ini")):
        # Line ends as LF whatever the checkout made them.
        text = path.read_text(encoding="ascii", errors="replace").replace("\r\n", "\n")
        vid, pid = value(text, "VID"), value(text, "PID")
        if vid is None or pid is None:
            raise SystemExit(f"{path.name}: no VID or PID")
        multi = value(text, "MultiIn")
        if multi is not None and int(multi, 16) >= 2:
            continue
        key = int(vid, 16) << 16 | int(pid, 16)
        if key in SKIPPED_IDS:
            continue
        if key in entries and PREFERRED.get(key) != path.name:
            if PREFERRED.get(key) != entries[key][0]:
                raise SystemExit(f"{path.name}: {key:08X} again, and PREFERRED names neither file")
            continue
        header = re.search(r"^\[(.+?)\]", text, re.MULTILINE)
        name = (header.group(1).strip() if header else path.stem).replace("\\", "\\\\").replace('"', '\\"')
        entries[key] = (path.name, name, text)

    lines = [
        "/* SPDX-FileCopyrightText: 2026 RiftWii contributors */",
        "/* SPDX-FileCopyrightText: FIX94 and the Nintendont contributors, used with permission */",
        "/* SPDX-License-Identifier: GPL-3.0-or-later */",
        "/* Made by scripts/make_hidpad_data.py from Nintendont's controllerconfigs: do not edit. */",
        "#ifdef HAVE_HIDPADS",
        "",
        '#include "libinput_hid.h"',
        "",
        "const hidpad_builtin hidpad_builtins[] = {",
    ]
    for key in sorted(entries):
        file_name, name, text = entries[key]
        lines.append(f'    {{0x{key:08X}u, "{name}", "{file_name}",')
        lines.append(c_string(text) + "},")
    lines += [
        "};",
        "",
        "const size_t hidpad_builtin_count = sizeof(hidpad_builtins) / sizeof(hidpad_builtins[0]);",
        "",
        "#endif /* HAVE_HIDPADS */",
        "",
    ]
    OUT.write_text("\n".join(lines), encoding="ascii", newline="\n")
    print(f"{OUT.name}: {len(entries)} layouts")


main()
