/* The Wii Speak's boot firmware, built into the library when one is given.
 *
 * The firmware (PMIC_FIRM_DATA, 97,480 bytes) is Nintendo's and is never in
 * this repository.  `make firmware SDK=<pmic.a>` puts it at data/pmic_firm.bin
 * (ignored by git), which make then builds in, or pass PMIC_FIRM=<file>; apps that compile these sources themselves define
 * WS_FIRMWARE_FILE as the file's path in quotes.  Without one,
 * WS_BuiltinFirmware() returns NULL and the app has to load it itself. */

#ifdef HAVE_MIC

#include <stddef.h>
#include <stdint.h>
#include "input_wiispeak.h"

#ifdef WS_FIRMWARE_FILE
__asm__(".section .rodata.ws_firmware,\"a\"\n"
        ".balign 32\n"
        "ws_firmware_start:\n"
        ".incbin \"" WS_FIRMWARE_FILE "\"\n"
        "ws_firmware_end:\n"
        ".balign 4\n"
        ".globl ws_firmware_ptrs\n"
        "ws_firmware_ptrs:\n"
        ".long ws_firmware_start, ws_firmware_end\n"
        ".previous\n");
extern const uint8_t *const ws_firmware_ptrs[2];
#endif

const uint8_t *WS_BuiltinFirmware(uint32_t *size)
{
#ifdef WS_FIRMWARE_FILE
	if (size)
		*size = (uint32_t)(ws_firmware_ptrs[1] - ws_firmware_ptrs[0]);
	return ws_firmware_ptrs[0];
#else
	if (size)
		*size = 0;
	return NULL;
#endif
}

#endif /* HAVE_MIC */
