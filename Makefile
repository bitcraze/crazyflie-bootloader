# CrazyLoader's Makefile

#################### Env config ######################
OPENOCD_INTERFACE ?= interface/jtagkey.cfg
OPENOCD_TARGET    ?= target/stm32f1x.cfg
CROSS_COMPILE     ?= arm-none-eabi-
######################################################

############### Location configuration ################

#ST peripheral library
STLIB=stlib
#CRT0.o
VPATH += $(STLIB)/CMSIS/Core/CM3/startup/gcc
CRT0=startup_stm32f10x_md_multiboot.o

include scripts/st_obj.mk

VPATH += src
PROJ_OBJ = main.o version.o led.o radio.o cpuid.o squeue.o

OBJ = $(CRT0) $(FREERTOS_OBJ) $(PORT_OBJ) $(ST_OBJ) $(PROJ_OBJ)

ifdef P
  C_PROFILE = -D P_$(P)
endif

############### Compilation configuration ################
AS = $(CROSS_COMPILE)as
CC = $(CROSS_COMPILE)gcc
LD = $(CROSS_COMPILE)gcc
SIZE = $(CROSS_COMPILE)size
OBJCOPY = $(CROSS_COMPILE)objcopy

INCLUDES = -I$(STLIB)/STM32F10x_StdPeriph_Driver/inc
INCLUDES+= -I$(STLIB)/CMSIS/Core/CM3 -Iinc
PROCESSOR = -mcpu=cortex-m3 -mthumb

#Flags required by the ST library
STFLAGS = -DSTM32F10X_MD -include stm32f10x_conf.h

CFLAGS  = $(PROCESSOR) $(INCLUDES) $(STFLAGS) -Os -g3 -Wall -fno-strict-aliasing $(C_PROFILE)
ASFLAGS  = $(PROCESSOR) $(INCLUDES)
LDFLAGS = $(PROCESSOR) -Wl,-Map=$(PROG).map,--cref,--gc-sections

#Bootloader specific linker script
ifdef CLOAD
  LDFLAGS += -T scripts/STM32F103_32K_20K_FLASH_CLOAD.ld  
else
  LDFLAGS += -T scripts/STM32F103_32K_20K_FLASH.ld 
endif

#Program name
PROG = cloader
#Where to compile the .o
BIN = bin
VPATH += $(BIN)

##################### Misc. ################################
ifeq ($(SHELL),/bin/sh)
  COL_RED=\033[1;31m
  COL_GREEN=\033[1;32m
  COL_RESET=\033[m
endif

#################### Targets ###############################


all: build
build: version compile print_version size
compile: $(BIN)/ $(PROG).hex $(PROG).bin

$(BIN)/:
	@mkdir -p $@
	@echo "  MKDIR $@"

version:
ifeq ($(SHELL),/bin/sh)
	@echo "  VER.  version.c"
	@scripts/update_version.sh
endif

print_version:
ifeq ($(SHELL),/bin/sh)
	@./scripts/clean_version.sh
	@./scripts/print_revision.sh
endif
ifdef CLOAD
	@echo "CrazyLoad Version!"
endif

size:
	@$(SIZE) -B $(PROG).elf

#Flash the stm.
flash:
	openocd -d0 -f $(OPENOCD_INTERFACE) -f $(OPENOCD_TARGET) -c init -c targets -c "reset halt" -c "flash erase_sector 0 0 20" -c "flash write_image erase cloader.elf"\
          -c "verify_image cloader.elf" -c "reset run" -c shutdown

#STM utility targets
halt:
	openocd -d0 -f $(OPENOCD_INTERFACE) -f $(OPENOCD_TARGET) -c init -c targets -c "halt" -c shutdown

reset:
	openocd -d0 -f $(OPENOCD_INTERFACE) -f $(OPENOCD_TARGET) -c init -c targets -c "reset" -c shutdown

openocd:
	openocd -d0 -f $(OPENOCD_INTERFACE) -f $(OPENOCD_TARGET) -c init -c targets

include scripts/targets.mk

