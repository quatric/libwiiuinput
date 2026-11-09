ifeq ($(strip $(DEVKITPRO)),)
$(error "Please set DEVKITPRO in your environment. export DEVKITPRO=<path to>/devkitPro")
endif

export DEVKITPPC ?= $(DEVKITPRO)/devkitPPC
export WUT_ROOT  ?= $(DEVKITPRO)/wut

include $(WUT_ROOT)/share/wut_rules

# devkitPPC's own archiver: no LTO here, and a gcc-ar elsewhere on the PATH
# may not find its plugin
AR      := $(DEVKITPPC)/bin/powerpc-eabi-ar

CFLAGS  := -O2 -Wall -Wextra -ffunction-sections -fdata-sections $(MACHDEP) \
           -D__WIIU__ -D__WUT__ -Iinclude -I$(WUT_ROOT)/include

# The GameCube controller adapter and USB game controllers (Nintendont's
# controller.ini layouts). Both on by default; neither needs another library.
WITH_GCADAPTER ?= 1
WITH_HIDPADS   ?= 1
# USB pen tablets and the uDraw GameTablet (input_tablet.h)
WITH_TABLET    ?= 1
ifeq ($(WITH_GCADAPTER),1)
 CFLAGS += -DHAVE_GCADAPTER
endif
ifeq ($(WITH_HIDPADS),1)
 CFLAGS += -DHAVE_HIDPADS
endif
ifeq ($(WITH_TABLET),1)
 CFLAGS += -DHAVE_TABLET
endif

SRCS    := $(wildcard source/*.c)
# one folder of objects for each set of options, so switching them rebuilds
BUILD   := build/$(WITH_GCADAPTER)$(WITH_HIDPADS)$(WITH_TABLET)
OBJS    := $(patsubst source/%.c,$(BUILD)/%.o,$(SRCS))
LIB     := lib/libinput-wiiu.a
HEADERS := source/libinput_internal.h source/libinput_hid.h source/libinput_hidparse.h include/input_wiiu.h include/input_tablet.h

all: $(LIB)

# made again every time: it may hold another set of options' objects
$(LIB): $(OBJS) FORCE
	@mkdir -p lib
	@rm -f $@
	$(AR) rcs $@ $(OBJS)

$(BUILD)/%.o: source/%.c $(HEADERS)
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) -c $< -o $@

install: $(LIB)
	@mkdir -p $(DESTDIR)$(DEVKITPRO)/portlibs/wiiu/lib $(DESTDIR)$(DEVKITPRO)/portlibs/wiiu/include
	cp $(LIB) $(DESTDIR)$(DEVKITPRO)/portlibs/wiiu/lib/
	cp include/input_wiiu.h include/input_tablet.h $(DESTDIR)$(DEVKITPRO)/portlibs/wiiu/include/

# the controller test program (example/padtest), a .wuhb for the Homebrew Launcher
padtest: $(LIB)
	@$(MAKE) --no-print-directory -C example/padtest

clean:
	rm -rf build lib build-test
	@$(MAKE) --no-print-directory -C example/padtest clean

# host test of the controller.ini reader (no devkitPro needed)
test:
	@mkdir -p build-test
	cc -DHAVE_HIDPADS -Wall -Wextra -fsanitize=address,undefined -Isource tests/hid_test.c source/libinput_hid.c source/libinput_hidpads_data.c -o build-test/hid_test
	build-test/hid_test

.PHONY: all install clean test padtest FORCE
