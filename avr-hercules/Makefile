###############################################################################
# Makefile for the project hercules-avr
###############################################################################

## General Flags
PROJECT = herc-avr
TARGET = herc-avr.elf
MCU = atmega328p
CC = avr-gcc
CPP = avr-g++

## Options common to compile, link and assembly rules
COMMON = -mmcu=$(MCU)

## Compile options common for all C compilation units.
CFLAGS = $(COMMON)
CFLAGS += -Wall -gdwarf-2 -std=gnu99 -DF_CPU=16000000UL -Os -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums
CFLAGS += -MD -MP -MT $(*F).o -MF dep/$(@F).d 

## Assembly specific flags
ASMFLAGS = $(COMMON)
ASMFLAGS += $(CFLAGS)
ASMFLAGS += -x assembler-with-cpp -Wa,-gdwarf2

## Linker flags
LDFLAGS = $(COMMON)
LDFLAGS +=  -Wl,-u,vfprintf,-lprintf_flt,-lm,-Map=herc-avr.map

## Intel Hex file production flags
HEX_FLASH_FLAGS = -R .eeprom -R .fuse -R .lock -R .signature
HEX_EEPROM_FLAGS = -j .eeprom
HEX_EEPROM_FLAGS += --set-section-flags=.eeprom="alloc,load"
HEX_EEPROM_FLAGS += --change-section-lma .eeprom=0 --no-change-warnings

## Objects that must be built in order to link
OBJS = main.o seeed_pwm.o pid.o usart.o wiring_analog.o 
SRCS = main.c seeed_pwm.c pid.c usart.c wiring_analog.c 
HDRS = Arduino.h hercules.h pins_arduino.h usart.h binary.h pid.h seeed_pwm.h wiring_private.h

## Objects explicitly added by the user
LINKONLYOBJECTS = 

## Build
#all: $(TARGET) mega485-tb6612.hex mega485-tb6612.eep mega485-tb6612.lss size
all: $(TARGET) $(PROJECT).hex size

%.o: %.c
	$(CC) $(INCLUDES) $(CFLAGS) -c $<

##Link
$(TARGET): $(OBJS)
	 $(CC) $(LDFLAGS) $(OBJS) $(LINKONLYOBJECTS) $(LIBDIRS) $(LIBS) -o $(TARGET)

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
	-rm -f $(OBJS) dep/* $(TARGET)  $(PROJECT).hex $(PROJECT).map

## Other dependencies
-include $(shell mkdir dep 2>/dev/null) $(wildcard dep/*)

install: $(PROJECT).hex
	stty -F /dev/ttyMFD1 crtscts
	avrdude -pm328p -carduino -P/dev/ttyMFD1 -D -Uflash:w:$(PROJECT).hex:i
