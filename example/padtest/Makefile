.SUFFIXES:
ifeq ($(strip $(DEVKITPRO)),)
$(error "Please set DEVKITPRO in your environment. export DEVKITPRO=<path to>/devkitPro")
endif

export DEVKITPPC ?= $(DEVKITPRO)/devkitPPC
export WUT_ROOT ?= $(DEVKITPRO)/wut

include $(WUT_ROOT)/share/wut_rules

# The controller test for libwiiuinput: `make` here (or `make padtest` in the
# library's folder) builds the library first and gives padtest.wuhb for the
# Homebrew Launcher, and padtest.rpx.
TARGET		:=	padtest
BUILD		:=	build
SOURCES		:=	source
DATA		:=
INCLUDES	:=	source

APP_NAME		:=	libwiiuinput padtest
APP_SHORTNAME	:=	padtest
APP_AUTHOR		:=	quatric

CFLAGS	= -g -O2 -Wall -Wextra $(MACHDEP) $(INCLUDE)
CXXFLAGS	=	$(CFLAGS)
LDFLAGS	=	-g $(RPXSPECS) $(MACHDEP)

LIBS	:=	-linput-wiiu -lwut -lm
LIBDIRS	:=	$(WUT_ROOT)

ifneq ($(BUILD),$(notdir $(CURDIR)))

export LIBINPUT	:=	$(abspath $(CURDIR)/../..)
export OUTPUT	:=	$(CURDIR)/$(TARGET)
export VPATH	:=	$(foreach dir,$(SOURCES),$(CURDIR)/$(dir))
export DEPSDIR	:=	$(CURDIR)/$(BUILD)

CFILES		:=	$(foreach dir,$(SOURCES),$(notdir $(wildcard $(dir)/*.c)))

export LD	:=	$(CC)
export OFILES	:=	$(CFILES:.c=.o)
export INCLUDE	:=	$(foreach dir,$(INCLUDES), -iquote $(CURDIR)/$(dir)) \
					-I$(LIBINPUT)/include -I$(WUT_ROOT)/include
export LIBPATHS	:=	-L$(LIBINPUT)/lib -L$(WUT_ROOT)/lib
export LDFLAGS	:=	$(LDFLAGS)

.PHONY: $(BUILD) clean all library
.DEFAULT_GOAL := all

all: $(BUILD)
	@$(MAKE) --no-print-directory $(TARGET).wuhb

library:
	@$(MAKE) --no-print-directory -C $(LIBINPUT) all

$(BUILD): library
	@[ -d $@ ] || mkdir -p $@
	@$(MAKE) --no-print-directory -C $(BUILD) -f $(CURDIR)/Makefile

clean:
	@echo clean ...
	@rm -fr $(BUILD) $(TARGET).elf $(TARGET).rpx $(TARGET).wuhb $(TARGET).rpx.map

else

DEPENDS	:=	$(OFILES:.o=.d)

$(OUTPUT).rpx: $(OUTPUT).elf
$(OUTPUT).elf: $(OFILES) $(LIBINPUT)/lib/libinput-wiiu.a

-include $(DEPENDS)

endif
