/* SPDX-FileCopyrightText: 2026 RiftWii contributors */
/* SPDX-FileCopyrightText: FIX94 and the Nintendont contributors, used with permission <https://github.com/FIX94/Nintendont> */
/* SPDX-License-Identifier: GPL-3.0-or-later */
/* Nintendont's controller.ini: its reader and its report conversion, from
 * RiftWii's src/data/hidconfig.cpp and runtime/rthid.c. See libinput_hid.h. */

#ifdef HAVE_HIDPADS

#include <stdio.h>
#include <string.h>

#include "libinput_hid.h"

/* ---- Reading a file --------------------------------------------------- */

typedef struct {
    const char *p;
    size_t n;
} sv;

typedef struct {
    sv name, value;
} field;

#define FIELDS_MAX 64u

static sv trim(sv s)
{
    while (s.n && (s.p[0] == ' ' || s.p[0] == '\t' || s.p[0] == '\r')) {
        s.p++;
        s.n--;
    }
    while (s.n && (s.p[s.n - 1] == ' ' || s.p[s.n - 1] == '\t' || s.p[s.n - 1] == '\r'))
        s.n--;
    return s;
}

static int sv_is(sv s, const char *text)
{
    size_t n = strlen(text);
    return s.n == n && memcmp(s.p, text, n) == 0;
}

static size_t sv_find(sv s, char c)
{
    size_t i;
    for (i = 0; i < s.n; i++)
        if (s.p[i] == c)
            return i;
    return (size_t)-1;
}

static int number(sv text, unsigned base, uint32_t limit, uint32_t *value)
{
    size_t i;

    text = trim(text);
    if (base == 16 && text.n > 2 && text.p[0] == '0' && (text.p[1] == 'x' || text.p[1] == 'X')) {
        text.p += 2;
        text.n -= 2;
    }
    if (!text.n)
        return 0;
    *value = 0;
    for (i = 0; i < text.n; i++) {
        char c = text.p[i];
        unsigned digit = 16;
        if (c >= '0' && c <= '9')
            digit = (unsigned)(c - '0');
        if (c >= 'a' && c <= 'f')
            digit = (unsigned)(c - 'a' + 10);
        if (c >= 'A' && c <= 'F')
            digit = (unsigned)(c - 'A' + 10);
        if (digit >= base || digit > limit || *value > (limit - digit) / base)
            return 0;
        *value = *value * base + digit;
    }
    return 1;
}

static const char *const button_keys[HIDPAD_BUTTONS] = {
    "A", "B", "X", "Y", "Z", "L", "R", "S", "Power", "ZL", "Left", "Down",
    "Right", "Up", "RightUp", "DownRight", "DownLeft", "UpLeft"
};
static const char *const axis_keys[4] = { "StickX", "StickY", "CStickX", "CStickY" };
static const char *const other_keys[] = {
    "VID", "PID", "Polltype", "DPAD", "DigitalLR", "MultiIn", "MultiInValue", "LAnalog", "RAnalog",
    "Rumble", "RumbleType", "RumbleDataLen", "RumbleDataOn", "RumbleDataOff", "RumbleTransferLen",
    "RumbleTransfers"
};

static int known(sv key)
{
    size_t i;
    for (i = 0; i < HIDPAD_BUTTONS; i++)
        if (sv_is(key, button_keys[i]))
            return 1;
    for (i = 0; i < 4; i++)
        if (sv_is(key, axis_keys[i]))
            return 1;
    for (i = 0; i < sizeof(other_keys) / sizeof(other_keys[0]); i++)
        if (sv_is(key, other_keys[i]))
            return 1;
    return 0;
}

/* A comma separated list of hex numbers (an axis's third is decimal, up
 * to 1000). Returns how many, 0 when malformed or outside [min, max]. */
static unsigned values(sv text, uint32_t *out, unsigned min, unsigned max, int axis)
{
    unsigned count = 0;

    while (text.n) {
        size_t comma = sv_find(text, ',');
        sv part = { text.p, comma == (size_t)-1 ? text.n : comma };
        unsigned base = axis && count == 2 ? 10 : 16;
        uint32_t limit = axis && count == 2 ? 1000 : 255;

        if (count == max || !number(part, base, limit, &out[count]))
            return 0;
        count++;
        if (comma == (size_t)-1)
            break;
        text.p += comma + 1;
        text.n -= comma + 1;
        if (!trim(text).n)
            return 0;
    }
    return count >= min ? count : 0;
}

static void say(char *error, size_t size, const char *what, sv key)
{
    if (!error || !size)
        return;
    snprintf(error, size, "%s%.*s", what, (int)(key.n > 48 ? 48 : key.n), key.p);
}

typedef struct {
    field fields[FIELDS_MAX];
    unsigned count;
} fieldset;

static sv get(const fieldset *set, const char *name)
{
    unsigned i;
    sv none = { NULL, 0 };
    for (i = 0; i < set->count; i++)
        if (sv_is(set->fields[i].name, name))
            return set->fields[i].value;
    return none;
}

static int scalar(const fieldset *set, const char *key, uint16_t *dest, uint32_t limit, int required,
                  char *error, size_t error_size)
{
    sv value = get(set, key);
    uint32_t parsed = 0;

    if (!value.n && !required)
        return 1;
    if (!number(value, 16, limit, &parsed)) {
        sv name = { key, strlen(key) };
        say(error, error_size, "Invalid controller value: ", name);
        return 0;
    }
    *dest = (uint16_t)parsed;
    return 1;
}

static void include_offset(hidpad_mapping *m, uint16_t offset)
{
    if (offset != HIDPAD_ABSENT && m->report_bytes <= offset)
        m->report_bytes = (uint16_t)(offset + 1);
}

int hidpad_parse(const char *text, size_t length, hidpad_mapping *mapping, char *error, size_t error_size)
{
    static fieldset set; /* off the stack: the thread's is small */
    hidpad_mapping r;
    sv rest = { text, length };
    int section = 0;
    unsigned i;
    uint32_t v[HIDPAD_RUMBLE_MAX];
    uint16_t rumble = 0;

    memset(mapping, 0, sizeof(*mapping));
    if (error && error_size)
        error[0] = 0;
    if (length > HIDPAD_CONFIG_BYTES || memchr(text, 0, length)) {
        say(error, error_size, "The controller file is too large or contains a zero byte.", (sv){ "", 0 });
        return 0;
    }
    set.count = 0;
    while (rest.n) {
        size_t newline = sv_find(rest, '\n');
        sv line = trim((sv){ rest.p, newline == (size_t)-1 ? rest.n : newline });
        size_t cut, equals;
        sv key, value;

        if (newline == (size_t)-1) {
            rest.n = 0;
        } else {
            rest.p += newline + 1;
            rest.n -= newline + 1;
        }
        for (cut = 0; cut < line.n && line.p[cut] != '#' && line.p[cut] != ';'; cut++)
            ;
        line = trim((sv){ line.p, cut });
        if (!line.n)
            continue;
        if (line.p[0] == '[' && line.p[line.n - 1] == ']') {
            if (section || set.count != 0 || line.n > 128) {
                say(error, error_size, "The controller file must describe one device.", (sv){ "", 0 });
                return 0;
            }
            section = 1;
            continue;
        }
        equals = sv_find(line, '=');
        key = trim((sv){ line.p, equals == (size_t)-1 ? line.n : equals });
        if (equals == (size_t)-1 || !known(key) || set.count == FIELDS_MAX) {
            say(error, error_size, "Unknown or malformed controller key: ", key);
            return 0;
        }
        for (i = 0; i < set.count; i++) {
            if (set.fields[i].name.n == key.n && memcmp(set.fields[i].name.p, key.p, key.n) == 0) {
                say(error, error_size, "Repeated controller key: ", key);
                return 0;
            }
        }
        value = trim((sv){ line.p + equals + 1, line.n - equals - 1 });
        if (!value.n) {
            say(error, error_size, "Empty controller value: ", key);
            return 0;
        }
        set.fields[set.count].name = key;
        set.fields[set.count].value = value;
        set.count++;
    }

    memset(&r, 0, sizeof(r));
    r.report_bytes = 1;
    r.analog_l = r.analog_r = HIDPAD_ABSENT;
    for (i = 0; i < HIDPAD_BUTTONS; i++)
        r.buttons[i].offset = HIDPAD_ABSENT;
    for (i = 0; i < 4; i++) {
        r.axes[i].offset = HIDPAD_ABSENT;
        r.axes[i].scale = 800;
    }
    if (!scalar(&set, "VID", &r.vid, 65535, 1, error, error_size) ||
        !scalar(&set, "PID", &r.pid, 65535, 1, error, error_size) ||
        !scalar(&set, "Polltype", &r.polltype, 1, 1, error, error_size) ||
        !scalar(&set, "DPAD", &r.dpad, 1, 0, error, error_size) ||
        !scalar(&set, "DigitalLR", &r.digital_lr, 2, 0, error, error_size) ||
        !scalar(&set, "MultiIn", &r.multi_in, 1, 0, error, error_size) ||
        !scalar(&set, "MultiInValue", &r.multi_value, 255, 0, error, error_size))
        return 0;

    /* Nintendont uses the first value as the analog offset; some shipped
     * files also give a mask, which its analog reader ignores. */
    for (i = 0; i < 2; i++) {
        const char *key = i ? "RAnalog" : "LAnalog";
        sv value = get(&set, key);
        uint32_t parts[2];

        if (!value.n)
            continue;
        if (!values(value, parts, 1, 2, 0) || parts[0] >= HIDPAD_REPORT_MAX) {
            say(error, error_size, "Invalid controller value: ", (sv){ key, strlen(key) });
            return 0;
        }
        *(i ? &r.analog_r : &r.analog_l) = (uint16_t)parts[0];
    }
    include_offset(&r, r.analog_l);
    include_offset(&r, r.analog_r);

    for (i = 0; i < HIDPAD_BUTTONS; i++) {
        sv value = get(&set, button_keys[i]);
        uint32_t pair[2];

        if (!value.n)
            continue;
        if (values(value, pair, 2, 2, 0) != 2 || pair[0] >= HIDPAD_REPORT_MAX) {
            say(error, error_size, "Invalid controller button: ", (sv){ button_keys[i], strlen(button_keys[i]) });
            return 0;
        }
        r.buttons[i].offset = (uint16_t)pair[0];
        r.buttons[i].mask = (uint16_t)pair[1];
        include_offset(&r, r.buttons[i].offset);
    }

    for (i = 0; i < 4; i++) {
        sv value = get(&set, axis_keys[i]);
        uint32_t parts[3] = { 0, 0, 80 }, radius;

        if (!value.n)
            continue;
        if (!values(value, parts, 1, 3, 1) || parts[0] >= HIDPAD_REPORT_MAX || parts[1] >= 128) {
            say(error, error_size, "Invalid controller stick: ", (sv){ axis_keys[i], strlen(axis_keys[i]) });
            return 0;
        }
        radius = parts[2] ? parts[2] : 80;
        r.axes[i].offset = (uint16_t)parts[0];
        r.axes[i].deadzone = (uint16_t)parts[1];
        r.axes[i].scale = radius * 1280u / (128u - parts[1]);
        include_offset(&r, r.axes[i].offset);
    }

    if (r.dpad) {
        uint16_t mask = 0;
        uint16_t left = r.buttons[HIDPAD_LEFT].offset;
        uint16_t offset = left == HIDPAD_ABSENT ? 0 : left;
        int same = 1;

        for (i = HIDPAD_LEFT; i <= HIDPAD_UP_LEFT; i++) {
            /* Missing directions default to offset and mask zero, as in
             * Nintendont, including pads whose D-pad is two axes. */
            if (r.buttons[i].offset == HIDPAD_ABSENT) {
                r.buttons[i].offset = 0;
                r.buttons[i].mask = 0;
            }
            same = same && r.buttons[i].offset == offset;
            mask |= r.buttons[i].mask;
        }
        if (!same)
            mask = 255;
        else if ((mask & 0xF0) == 0)
            mask = 0x0F;
        else if ((mask & 0x0F) == 0)
            mask = 0xF0;
        r.dpad_mask = mask;
    }

    if (!scalar(&set, "Rumble", &rumble, 1, 0, error, error_size) ||
        !scalar(&set, "RumbleType", &r.rumble_type, 1, 0, error, error_size) ||
        !scalar(&set, "RumbleDataLen", &r.rumble_bytes, HIDPAD_RUMBLE_MAX, 0, error, error_size) ||
        !scalar(&set, "RumbleTransferLen", &r.rumble_transfer, HIDPAD_RUMBLE_MAX, 0, error, error_size) ||
        !scalar(&set, "RumbleTransfers", &r.rumble_count, 16, 0, error, error_size))
        return 0;
    if (rumble) {
        for (i = 0; i < 2; i++) {
            const char *key = i ? "RumbleDataOff" : "RumbleDataOn";
            uint8_t *dest = i ? r.rumble_off : r.rumble_on;
            unsigned n, j;

            n = r.rumble_bytes ? values(get(&set, key), v, r.rumble_bytes, r.rumble_bytes, 0) : 0;
            if (!n) {
                say(error, error_size, "Invalid controller rumble data: ", (sv){ key, strlen(key) });
                return 0;
            }
            for (j = 0; j < r.rumble_bytes; j++)
                dest[j] = (uint8_t)v[j];
        }
    } else if (r.rumble_bytes || r.rumble_transfer || r.rumble_count || get(&set, "RumbleDataOn").n ||
               get(&set, "RumbleDataOff").n) {
        say(error, error_size, "Controller rumble data needs Rumble=1.", (sv){ "", 0 });
        return 0;
    }

    if (!hidpad_valid(&r)) {
        say(error, error_size, "The controller mapping has invalid limits.", (sv){ "", 0 });
        return 0;
    }
    *mapping = r;
    return 1;
}

/* ---- Reading a report ------------------------------------------------- */

static int valid_offset(uint16_t offset, uint16_t bytes)
{
    return offset == HIDPAD_ABSENT || offset < bytes;
}

int hidpad_valid(const hidpad_mapping *m)
{
    uint32_t i;

    if (!m || !m->vid || !m->pid || !m->report_bytes || m->report_bytes > HIDPAD_REPORT_MAX ||
        m->polltype > 1 || m->dpad > 1 || m->dpad_mask > 255 || m->digital_lr > 2 || m->multi_in > 1 ||
        m->multi_value > 255 || m->rumble_type > 1)
        return 0;
    if (!m->polltype && m->report_bytes > 49)
        return 0;
    if (!valid_offset(m->analog_l, m->report_bytes) || !valid_offset(m->analog_r, m->report_bytes))
        return 0;
    for (i = 0; i < HIDPAD_BUTTONS; i++)
        if (!valid_offset(m->buttons[i].offset, m->report_bytes) || m->buttons[i].mask > 255)
            return 0;
    for (i = 0; i < 4; i++)
        if (!valid_offset(m->axes[i].offset, m->report_bytes) || m->axes[i].deadzone >= 128 ||
            !m->axes[i].scale || m->axes[i].scale > 1280000u)
            return 0;
    if (m->rumble_bytes > HIDPAD_RUMBLE_MAX)
        return 0;
    if (m->rumble_bytes) {
        if (!m->rumble_transfer || m->rumble_transfer > HIDPAD_RUMBLE_MAX || !m->rumble_count ||
            m->rumble_count > 16 || (uint32_t)m->rumble_transfer * m->rumble_count != m->rumble_bytes)
            return 0;
    } else if (m->rumble_transfer || m->rumble_count) {
        return 0;
    }
    return 1;
}

static int pressed(const hidpad_mapping *m, uint32_t index, const uint8_t *report)
{
    const hidpad_button_map *b = &m->buttons[index];
    return b->offset != HIDPAD_ABSENT && (report[b->offset] & b->mask) != 0;
}

static int direction(const hidpad_mapping *m, uint32_t index, const uint8_t *report)
{
    const hidpad_button_map *b = &m->buttons[index];

    if (b->offset == HIDPAD_ABSENT)
        return 0;
    if (!m->dpad)
        return pressed(m, index, report);
    return (report[b->offset] & m->dpad_mask) == b->mask;
}

static int8_t axis_value(const hidpad_axis *axis, const uint8_t *report, int vertical)
{
    int32_t value, magnitude;

    if (axis->offset == HIDPAD_ABSENT)
        return 0;
    /* The centres and the radius in thousandths follow controller.ini,
     * including its 127-centred inverted Y axis. Clamped before narrowing. */
    value = vertical ? 127 - (int32_t)report[axis->offset] : (int32_t)report[axis->offset] - 128;
    magnitude = value < 0 ? -value : value;
    if (magnitude <= (int32_t)axis->deadzone)
        return 0;
    magnitude = ((magnitude - (int32_t)axis->deadzone) * (int32_t)axis->scale) / 1000;
    value = value < 0 ? -magnitude : magnitude;
    if (value < -128)
        value = -128;
    if (value > 127)
        value = 127;
    return (int8_t)value;
}

static uint8_t trigger_value(uint16_t offset, const uint8_t *report)
{
    uint32_t value;

    if (offset == HIDPAD_ABSENT || report[offset] <= 26)
        return 0;
    value = ((uint32_t)report[offset] - 26u) * 111u / 100u;
    return (uint8_t)(value > 255u ? 255u : value);
}

int hidpad_decode(const hidpad_mapping *m, const uint8_t *report, uint32_t bytes, hidpad_state *out)
{
    uint16_t buttons = 0;
    int left, right;

    if (!report || !out || !hidpad_valid(m) || bytes < m->report_bytes || bytes > HIDPAD_REPORT_MAX)
        return 0;
    if (m->multi_in && report[0] != m->multi_value)
        return 0;
    if (pressed(m, HIDPAD_A, report)) buttons |= HIDPAD_PAD_A;
    if (pressed(m, HIDPAD_B, report)) buttons |= HIDPAD_PAD_B;
    if (pressed(m, HIDPAD_X, report)) buttons |= HIDPAD_PAD_X;
    if (pressed(m, HIDPAD_Y, report)) buttons |= HIDPAD_PAD_Y;
    if (pressed(m, HIDPAD_Z, report)) buttons |= HIDPAD_PAD_Z;
    if (pressed(m, HIDPAD_START, report)) buttons |= HIDPAD_PAD_START;
    if (direction(m, HIDPAD_LEFT, report) ||
        (m->dpad && (direction(m, HIDPAD_DOWN_LEFT, report) || direction(m, HIDPAD_UP_LEFT, report))))
        buttons |= HIDPAD_PAD_LEFT;
    if (direction(m, HIDPAD_RIGHT, report) ||
        (m->dpad && (direction(m, HIDPAD_RIGHT_UP, report) || direction(m, HIDPAD_DOWN_RIGHT, report))))
        buttons |= HIDPAD_PAD_RIGHT;
    if (direction(m, HIDPAD_DOWN, report) ||
        (m->dpad && (direction(m, HIDPAD_DOWN_RIGHT, report) || direction(m, HIDPAD_DOWN_LEFT, report))))
        buttons |= HIDPAD_PAD_DOWN;
    if (direction(m, HIDPAD_UP, report) ||
        (m->dpad && (direction(m, HIDPAD_RIGHT_UP, report) || direction(m, HIDPAD_UP_LEFT, report))))
        buttons |= HIDPAD_PAD_UP;

    left = pressed(m, HIDPAD_L, report);
    right = pressed(m, HIDPAD_R, report);
    if (m->digital_lr == 2) {
        const hidpad_button_map *l = &m->buttons[HIDPAD_L];
        const hidpad_button_map *r = &m->buttons[HIDPAD_R];
        left = l->offset != HIDPAD_ABSENT && report[l->offset] >= l->mask;
        right = r->offset != HIDPAD_ABSENT && report[r->offset] >= r->mask;
    }
    if (m->digital_lr == 1 && pressed(m, HIDPAD_ZL, report)) {
        out->trigger_l = left ? 127 : 0;
        out->trigger_r = right ? 127 : 0;
    } else {
        if (left) buttons |= HIDPAD_PAD_L;
        if (right) buttons |= HIDPAD_PAD_R;
        out->trigger_l = m->digital_lr == 1 ? (left ? 255 : 0) : trigger_value(m->analog_l, report);
        out->trigger_r = m->digital_lr == 1 ? (right ? 255 : 0) : trigger_value(m->analog_r, report);
    }
    out->buttons = buttons;
    out->stick_x = axis_value(&m->axes[0], report, 0);
    out->stick_y = axis_value(&m->axes[1], report, 1);
    out->substick_x = axis_value(&m->axes[2], report, 0);
    out->substick_y = axis_value(&m->axes[3], report, 1);
    out->no_rumble = m->rumble_bytes ? 0 : 1;
    return 1;
}

const hidpad_builtin *hidpad_find_builtin(uint32_t vid_pid)
{
    size_t i;

    for (i = 0; i < hidpad_builtin_count; i++)
        if (hidpad_builtins[i].vid_pid == vid_pid)
            return &hidpad_builtins[i];
    return NULL;
}

#endif /* HAVE_HIDPADS */
