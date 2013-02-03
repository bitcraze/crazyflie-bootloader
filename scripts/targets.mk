# Part of Crazyflie's Makefile
# Common targets with verbose support


ifeq ($(V),)
  VERBOSE=_SILENT
endif

ifeq ($(V),0)
  QUIET=1
endif

target = @$(if $(QUIET), ,echo $($1_COMMAND$(VERBOSE)) ); @$($1_COMMAND)


CC_COMMAND=$(CC) $(CFLAGS) -c $< -o $(BIN)/$@
CC_COMMAND_SILENT="  CC    $@"
.c.o: 
	@$(if $(QUIET), ,echo $(CC_COMMAND$(VERBOSE)) )
	@$(CC_COMMAND)

LD_COMMAND=$(LD) $(LDFLAGS) $(foreach o,$(OBJ),$(BIN)/$(o)) -o $@
LD_COMMAND_SILENT="  LD    $@"
$(PROG).elf: $(OBJ)
	@$(if $(QUIET), ,echo $(LD_COMMAND$(VERBOSE)) )
	@$(LD_COMMAND)

HEX_COMMAND=$(OBJCOPY) $< -O ihex $@
HEX_COMMAND_SILENT="  COPY  $@"
$(PROG).hex: $(PROG).elf
	@$(if $(QUIET), ,echo $(HEX_COMMAND$(VERBOSE)) )
	@$(HEX_COMMAND)

BIN_COMMAND=$(OBJCOPY) $< -O binary --pad-to 0 $@
BIN_COMMAND_SILENT="  COPY  $@"
$(PROG).bin: $(PROG).elf
	@$(if $(QUIET), ,echo $(BIN_COMMAND$(VERBOSE)) )
	@$(BIN_COMMAND)

AS_COMMAND=$(AS) $(ASFLAGS) $< -o $(BIN)/$@
AS_COMMAND_SILENT="  AS    $@"
.s.o:
	@$(if $(QUIET), ,echo $(AS_COMMAND$(VERBOSE)) )
	@$(AS_COMMAND)

CLEAN_O_COMMAND=rm -f $(foreach o,$(OBJ),$(BIN)/$(o))
CLEAN_O_COMMAND_SILENT="  CLEAN_O"
clean_o:
	@$(if $(QUIET), ,echo $(CLEAN_O_COMMAND$(VERBOSE)) )
	@$(CLEAN_O_COMMAND)

CLEAN_COMMAND=rm -f $(PROG).elf $(PROG).hex $(PROG).map $(PROG).bin
CLEAN_COMMAND_SILENT="  CLEAN"
clean: clean_o
	@$(if $(QUIET), ,echo $(CLEAN_COMMAND$(VERBOSE)) )
	@$(CLEAN_COMMAND)

MRPROPER_COMMAND=rm -f *~ src/*~ inc/*~ scripts/*~
MRPROPER_COMMAND_SILENT="  MRPROPER"
mrproper: clean
	@$(if $(QUIET), ,echo $(MRPROPER_COMMAND$(VERBOSE)) )
	@$(MRPROPER_COMMAND)


