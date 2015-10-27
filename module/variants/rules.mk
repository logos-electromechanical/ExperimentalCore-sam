#
#  SAM Arduino IDE variant makefile.
#
#  Copyright (c) 2015 Thibaut VIARD. All right reserved.
#
#  This library is free software; you can redistribute it and/or
#  modify it under the terms of the GNU Lesser General Public
#  License as published by the Free Software Foundation; either
#  version 2.1 of the License, or (at your option) any later version.
#
#  This library is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
#  See the GNU Lesser General Public License for more details.
#
#  You should have received a copy of the GNU Lesser General Public
#  License along with this library; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
#

#-------------------------------------------------------------------------------
#		Tools
#-------------------------------------------------------------------------------

# Set DEBUG variable for once if not coming from command line
ifndef DEBUG
DEBUG = 0
endif

# Tool suffix when cross-compiling
CROSS_COMPILE = arm-none-eabi-

# Compilation tools
CC = $(CROSS_COMPILE)gcc
AR = $(CROSS_COMPILE)ar
SIZE = $(CROSS_COMPILE)size
STRIP = $(CROSS_COMPILE)strip
OBJCOPY = $(CROSS_COMPILE)objcopy
GDB = $(CROSS_COMPILE)gdb
NM = $(CROSS_COMPILE)nm

# change this value if openocd isn't in the user/system PATH
OPENOCD = openocd

ROOT_PATH = ../..
VARIANT_PATH = $(ROOT_PATH)/variants/$(VARIANT_NAME)
RESOURCES_OPENOCD_UPLOAD = $(VARIANT_PATH)/openocd_scripts/variant_upload.cfg
RESOURCES_OPENOCD_START = $(VARIANT_PATH)/openocd_scripts/variant_debug_start.cfg
RESOURCES_GDB = $(VARIANT_PATH)/debug_scripts/variant.gdb
RESOURCES_LINKER = $(VARIANT_PATH)/linker_scripts/gcc/variant_without_bootloader.ld
CORE_ARM_PATH = $(ROOT_PATH)/cores/arduino/arch_arm
CORE_COMMON_PATH = $(ROOT_PATH)/cores/arduino/arch_common
CORE_AVR_PATH = $(ROOT_PATH)/cores/arduino/avr_compat
CORE_SAM_PATH = $(ROOT_PATH)/cores/arduino/port_sam

INCLUDES  = -I$(ROOT_PATH)/tools/CMSIS_API/Include
INCLUDES += -I$(ROOT_PATH)/tools/CMSIS_Devices/ATMEL
INCLUDES += -I$(ROOT_PATH)/cores/arduino

OBJ_PATH = $(VARIANT_PATH)/obj
OUTPUT_NAME = lib$(VARIANT_NAME)
OUTPUT_FILE_PATH = $(VARIANT_PATH)/$(OUTPUT_NAME).a

#|---------------------------------------------------------------------------------------|
#| Source files                                                                          |
#|---------------------------------------------------------------------------------------|
include ../sources.mk

SRC_VARIANT=\
$(VARIANT_PATH)/pins_arduino.h          \
$(VARIANT_PATH)/variant.h               \
$(VARIANT_PATH)/variant.cpp             \
$(VARIANT_PATH)/variant_startup.c       \
$(VARIANT_PATH)/variant_init.cpp
INCLUDES += -I$(VARIANT_PATH)

SOURCES+=$(SRC_VARIANT)

#|---------------------------------------------------------------------------------------|
#| Extract file names and path                                                           |
#|---------------------------------------------------------------------------------------|
PROJ_ASRCS   = $(filter %.s,$(foreach file,$(SOURCES),$(file)))
PROJ_ASRCS  += $(filter %.S,$(foreach file,$(SOURCES),$(file)))
PROJ_CSRCS   = $(filter %.c,$(foreach file,$(SOURCES),$(file)))
PROJ_CPPSRCS = $(filter %.cpp,$(foreach file,$(SOURCES),$(file)))

#|---------------------------------------------------------------------------------------|
#| Set important path variables                                                          |
#|---------------------------------------------------------------------------------------|
VPATH    = $(foreach path,$(sort $(foreach file,$(SOURCES),$(dir $(file)))),$(path) :)
INC_PATH = $(INCLUDES)
LIB_PATH = -L$(dir $(RESOURCES_LINKER))

#|---------------------------------------------------------------------------------------|
#| Options for compiler binaries                                                         |
#|---------------------------------------------------------------------------------------|
COMMON_FLAGS = -Wall -Wchar-subscripts -Wcomment
COMMON_FLAGS += -Werror-implicit-function-declaration -Wmain -Wparentheses
COMMON_FLAGS += -Wsequence-point -Wreturn-type -Wswitch -Wtrigraphs -Wunused
COMMON_FLAGS += -Wuninitialized -Wunknown-pragmas -Wfloat-equal -Wundef
COMMON_FLAGS += -Wshadow -Wpointer-arith -Wwrite-strings
COMMON_FLAGS += -Wsign-compare -Waggregate-return -Wmissing-declarations
COMMON_FLAGS += -Wmissing-format-attribute -Wno-deprecated-declarations
COMMON_FLAGS += -Wpacked -Wredundant-decls -Wlong-long
COMMON_FLAGS += -Wunreachable-code -Wcast-align
# -Wmissing-noreturn -Wconversion
COMMON_FLAGS += --param max-inline-insns-single=500 -mcpu=$(DEVICE_CORE) -mthumb -ffunction-sections -fdata-sections
# COMMON_FLAGS += -D$(DEVICE_PART) -DDONT_USE_CMSIS_INIT -fdiagnostics-color=always
COMMON_FLAGS += -D$(DEVICE_PART) -DDONT_USE_CMSIS_INIT 
COMMON_FLAGS += -Wa,-adhlns="$(subst .o,.lst,$@)"
COMMON_FLAGS += $(INC_PATH) -DF_CPU=$(DEVICE_FREQUENCY)
COMMON_FLAGS += --param max-inline-insns-single=500

ifeq ($(DEBUG),0)
COMMON_FLAGS += -Os
else
COMMON_FLAGS += -ggdb3 -O0
COMMON_FLAGS += -Wformat=2
endif

CFLAGS = $(COMMON_FLAGS) -std=gnu11 -Wimplicit-int -Wbad-function-cast -Wmissing-prototypes -Wnested-externs

ifeq ($(DEBUG),1)
CFLAGS += -Wstrict-prototypes
endif

CPPFLAGS = $(COMMON_FLAGS) -std=gnu++11 -fno-rtti -fno-exceptions
#-fno-optional-diags -fno-threadsafe-statics

#|---------------------------------------------------------------------------------------|
#| Define targets                                                                        |
#|---------------------------------------------------------------------------------------|
#AOBJS += $(patsubst %.S,%.o,$(PROJ_ASRCS))
AOBJS = $(patsubst %.s,%.o,$(addprefix $(OBJ_PATH)/, $(notdir $(PROJ_ASRCS))))
COBJS = $(patsubst %.c,%.o,$(addprefix $(OBJ_PATH)/, $(notdir $(PROJ_CSRCS))))
CPPOBJS = $(patsubst %.cpp,%.o,$(addprefix $(OBJ_PATH)/, $(notdir $(PROJ_CPPSRCS))))

.PHONY: all clean print_info packaging

all: $(OUTPUT_FILE_PATH)

print_info:
	@echo DEFAULT_GOAL ---------------------------------------------------------------------------------
	@echo $(.DEFAULT_GOAL)
	@echo VPATH ---------------------------------------------------------------------------------
	@echo $(VPATH)
	@echo SOURCES -------------------------------------------------------------------------------
	@echo $(SOURCES)
#	@echo PROJ_ASRCS ----------------------------------------------------------------------------
#	@echo $(PROJ_ASRCS)
#	@echo AOBJS ---------------------------------------------------------------------------------
#	@echo $(AOBJS)
	@echo PROJ_CSRCS ----------------------------------------------------------------------------
	@echo $(PROJ_CSRCS)
	@echo COBJS ---------------------------------------------------------------------------------
	@echo $(COBJS)
	@echo PROJ_CPPSRCS --------------------------------------------------------------------------
	@echo $(PROJ_CPPSRCS)
	@echo CPPOBJS -------------------------------------------------------------------------------
	@echo $(CPPOBJS)
	@echo ---------------------------------------------------------------------------------------
	@echo $(CURDIR)
	@echo $(OUTPUT_FILE_PATH)
	@echo ---------------------------------------------------------------------------------------

$(OUTPUT_FILE_PATH): $(OBJ_PATH) ../rules.mk ../sources.mk $(VARIANT_PATH)/Makefile $(AOBJS) $(COBJS) $(CPPOBJS)
	$(AR) -rv $(OUTPUT_FILE_PATH) $(AOBJS)
	$(AR) -rv $(OUTPUT_FILE_PATH) $(COBJS)
	$(AR) -rv $(OUTPUT_FILE_PATH) $(CPPOBJS)
	$(NM) $(OUTPUT_FILE_PATH) > $(VARIANT_PATH)/$(OUTPUT_NAME)_symbols.txt

#|---------------------------------------------------------------------------------------|
#| Compile or assemble                                                                   |
#|---------------------------------------------------------------------------------------|
$(AOBJS): $(OBJ_PATH)/%.o: %.s
	@echo +++ Assembling [$(notdir $<)]
	@$(AS) $(AFLAGS) $< -o $@

$(AOBJS): $(OBJ_PATH)/%.o: %.S
	@echo +++ Assembling [$(notdir $<)]
	@$(AS) $(AFLAGS) $< -o $@

$(COBJS): $(OBJ_PATH)/%.o: %.c
	@echo +++ Compiling [$(notdir $<)]
	@$(CC) $(CFLAGS) -c $< -o $@

$(CPPOBJS): $(OBJ_PATH)/%.o: %.cpp
	@echo +++ Compiling [$(notdir $<)]
	@$(CC) $(CPPFLAGS) -c $< -o $@

#|---------------------------------------------------------------------------------------|
#| Output folder                                                                         |
#|---------------------------------------------------------------------------------------|
$(OBJ_PATH):
	@echo +++ Creation of [$@]
	@-mkdir $(OBJ_PATH)

#|---------------------------------------------------------------------------------------|
#| Cleanup                                                                               |
#|---------------------------------------------------------------------------------------|
clean:
	-rm -f $(OBJ_PATH)/* $(OBJ_PATH)/*.*
	-rmdir $(OBJ_PATH)
	-rm -f $(OUTPUT_FILE_PATH)
#-rm -f $(VARIANT_PATH)/$(OUTPUT_NAME)_symbols.txt

#|---------------------------------------------------------------------------------------|
#| Dependencies                                                                          |
#|---------------------------------------------------------------------------------------|
$(OBJ_PATH)/%.d: %.s $(OBJ_PATH)
	@echo +++ Dependencies of [$(notdir $<)]
	@$(CC) $(AFLAGS) -MM -c $< -MT $(basename $@).o -o $@

$(OBJ_PATH)/%.d: %.S $(OBJ_PATH)
	@echo +++ Dependencies of [$(notdir $<)]
	@$(CC) $(AFLAGS) -MM -c $< -MT $(basename $@).o -o $@

$(OBJ_PATH)/%.d: %.c $(OBJ_PATH)
	@echo +++ Dependencies of [$(notdir $<)]
	@$(CC) $(CFLAGS) -MM -c $< -MT $(basename $@).o -o $@

$(OBJ_PATH)/%.d: %.cpp $(OBJ_PATH)
	@echo +++ Dependencies of [$(notdir $<)]
	@$(CC) $(CPPFLAGS) -MM -c $< -MT $(basename $@).o -o $@

#|---------------------------------------------------------------------------------------|
#| Include dependencies, if existing                                                     |
#| Little trick to avoid dependencies build for some rules when useless                  |
#| CAUTION: this won't work as expected with 'make clean all'                            |
#|---------------------------------------------------------------------------------------|
DEP_EXCLUDE_RULES := clean print_info
ifeq (,$(findstring $(MAKECMDGOALS), $(DEP_EXCLUDE_RULES)))
-include $(AOBJS:%.o=%.d)
-include $(COBJS:%.o=%.d)
-include $(CPPOBJS:%.o=%.d)
endif


#|---------------------------------------------------------------------------------------|
#| Module packaging for Arduino IDE Board Manager                                        |
#|---------------------------------------------------------------------------------------|
packaging: $(OUTPUT_FILE_PATH)

%.d:
