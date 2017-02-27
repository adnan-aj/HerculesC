###############################################################################
# Makefile for the project mega485-tb6612
###############################################################################

## General Flags
PROJECT = herc
MCU = atmega168p
TARGET = mega485-tb6612.elf
CC = avr-gcc
CPP = avr-g++

## Options common to compile, link and assembly rules
COMMON = -mmcu=$(MCU)

## Compile options common for all C compilation units.
CFLAGS = $(COMMON)
CFLAGS += -Wall -gdwarf-2 -std=gnu99 -DF_CPU=7372800UL -Os -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums
CFLAGS += -MD -MP -MT $(*F).o -MF dep/$(@F).d 

## Assembly specific flags
ASMFLAGS = $(COMMON)
ASMFLAGS += $(CFLAGS)
ASMFLAGS += -x assembler-with-cpp -Wa,-gdwarf2

## Linker flags
LDFLAGS = $(COMMON)
LDFLAGS +=  -Wl,-Map=mega485-tb6612.map


## Intel Hex file production flags
HEX_FLASH_FLAGS = -R .eeprom -R .fuse -R .lock -R .signature

HEX_EEPROM_FLAGS = -j .eeprom
HEX_EEPROM_FLAGS += --set-section-flags=.eeprom="alloc,load"
HEX_EEPROM_FLAGS += --change-section-lma .eeprom=0 --no-change-warnings


## Objects that must be built in order to link
OBJECTS = mega485-tb6612.o usart-rs485.o dyna485.o 

## Objects explicitly added by the user
LINKONLYOBJECTS = 

## Build
#all: $(TARGET) mega485-tb6612.hex mega485-tb6612.eep mega485-tb6612.lss size
all: $(TARGET) $(PROJECT).hex size hagar

## Compile
hagar: dyna485.h lib-juice.h hagar.c dyna485.c lib-juice.c
	gcc -o hagar hagar.c dyna485.c lib-juice.c 

mega485-tb6612.o: mega485-tb6612.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

usart-rs485.o: usart-rs485.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

dyna485.o: dyna485.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

##Link
$(TARGET): $(OBJECTS)
	 $(CC) $(LDFLAGS) $(OBJECTS) $(LINKONLYOBJECTS) $(LIBDIRS) $(LIBS) -o $(TARGET)

%.hex: $(TARGET)
	avr-objcopy -O ihex $(HEX_FLASH_FLAGS)  $< $@

%.eep: $(TARGET)
	-avr-objcopy $(HEX_EEPROM_FLAGS) -O ihex $< $@ || exit 0

%.lss: $(TARGET)
	avr-objdump -h -S $< > $@

size: ${TARGET}
	@echo
	@avr-size -C --mcu=${MCU} ${TARGET}

## Clean target
.PHONY: clean
clean:
	-rm -rf $(OBJECTS) mega485-tb6612.elf dep/* mega485-tb6612.hex mega485-tb6612.eep mega485-tb6612.lss mega485-tb6612.map hagar


## Other dependencies
-include $(shell mkdir dep 2>/dev/null) $(wildcard dep/*)

flashinstall: $(PROJECT).hex
	stty -F /dev/ttyMFD1 crtscts
	avrdude -pm328p -carduino -P/dev/ttyMFD1 -D -Uflash:w:HerculesC.hex:i
