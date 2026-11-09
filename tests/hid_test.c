/* SPDX-License-Identifier: GPL-3.0-or-later */
/* Host test of the controller.ini reader and the report conversion:
 * `make test`. Every built-in layout must read and be valid, and a few
 * reports must come out the way Nintendont's would. */
#include <stdio.h>
#include <string.h>

#include "libinput_hid.h"

static int failures;

#define CHECK(cond) do { if (!(cond)) { printf("FAIL line %d: %s\n", __LINE__, #cond); failures++; } } while (0)

static int parse(const char *text, hidpad_mapping *m, char *error)
{
    return hidpad_parse(text, strlen(text), m, error, 96);
}

int main(void)
{
    hidpad_mapping m;
    hidpad_state s;
    char error[96];
    const hidpad_builtin *b;
    uint8_t r[16];
    size_t i;

    CHECK(hidpad_builtin_count >= 40);
    for (i = 0; i < hidpad_builtin_count; i++) {
        const hidpad_builtin *e = &hidpad_builtins[i];

        if (!parse(e->text, &m, error)) {
            printf("FAIL %s: %s\n", e->file, error);
            failures++;
            continue;
        }
        CHECK((((uint32_t)m.vid << 16) | m.pid) == e->vid_pid);
        CHECK(hidpad_valid(&m));
        CHECK(i == 0 || hidpad_builtins[i - 1].vid_pid < e->vid_pid);
    }
    CHECK(hidpad_find_builtin(0x057E0337u) == NULL); /* the adapter has its own driver */
    CHECK(hidpad_find_builtin(0x12345678u) == NULL);

    /* GTC Ribbon pad: hat D-pad, digital triggers, A is byte 5 bit 6. */
    b = hidpad_find_builtin(0x00790006u);
    CHECK(b && parse(b->text, &m, error));
    memset(r, 0, sizeof(r));
    r[0] = r[1] = r[3] = r[4] = 128;
    r[5] = 0x40 | 0x02; /* A, hat 2: right */
    r[6] = 0x01;        /* L */
    CHECK(hidpad_decode(&m, r, 8, &s));
    CHECK(s.buttons == (HIDPAD_PAD_A | HIDPAD_PAD_RIGHT | HIDPAD_PAD_L));
    CHECK(s.stick_x == 0 && s.trigger_l == 255 && s.trigger_r == 0);
    r[0] = 255;
    r[1] = 0;
    CHECK(hidpad_decode(&m, r, 8, &s) && s.stick_x > 90 && s.stick_y > 90);
    CHECK(!hidpad_decode(&m, r, m.report_bytes - 1, &s)); /* too short */

    /* Rumble data is kept for the driver. */
    CHECK(m.rumble_type == 1 && m.rumble_bytes == 14 && m.rumble_transfer == 7 && m.rumble_count == 2);
    CHECK(m.rumble_on[4] == 0xFF && m.rumble_off[4] == 0x00);

    /* A file that is not a layout is refused, with the key named. */
    CHECK(!parse("[x]\nVID=1\nPID=2\nPolltype=1\nBogus=1\n", &m, error) && strstr(error, "Bogus"));
    CHECK(m.vid == 0);
    CHECK(!parse("VID=1\nPID=2\nPolltype=1\nPolltype=1\n", &m, error) && strstr(error, "Repeated"));
    CHECK(!parse("VID=1\nPID=2\n", &m, error)); /* no Polltype */
    CHECK(!parse("VID=1\nPID=2\nPolltype=1\nA=5\n", &m, error)); /* a button wants offset and mask */
    CHECK(!parse("VID=1\nPID=2\nPolltype=1\nRumbleDataLen=2\n", &m, error)); /* data without Rumble=1 */
    CHECK(!parse("[a]\n[b]\nVID=1\nPID=2\nPolltype=1\n", &m, error)); /* two devices */
    CHECK(parse("# a comment\nVID=0x1\nPID=2 ; here\nPolltype=1\nA=3,01\n", &m, error));
    CHECK(m.vid == 1 && m.pid == 2 && m.report_bytes == 4);

    printf(failures ? "%d failure(s)\n" : "all passed\n", failures);
    return failures != 0;
}
