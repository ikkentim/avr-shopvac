all: shopvac

###########################################
# AVR toolchain detection
###########################################

# Use Arduino toolchain if available
ifeq ($(shell uname), Darwin)
ARDUINO_DIR = /Applications/Arduino.app/Contents/Java/
endif

ifneq "$(wildcard $(ARDUINO_DIR) )" ""
AVR_DIR 	= $(ARDUINO_DIR)hardware/tools/avr/
AVR_BIN 	= $(AVR_DIR)bin/
endif

ifneq "$(wildcard $(AVR_DIR) )" ""
AVRDUDE_ARGS = -C "$(AVR_DIR)/etc/avrdude.conf"
endif

AVR_CC 		= $(AVR_BIN)avr-gcc
AVR_OBJCOPY = $(AVR_BIN)avr-objcopy
AVR_OBJDUMP = $(AVR_BIN)avr-objdump
AVR_SIZE 	= $(AVR_BIN)avr-size
AVRDUDE 	= $(AVR_BIN)avrdude

###########################################
# Default ISP options
###########################################

ISP_SPEED 	= 19200
ISP_PORT 	= $(word 1, $(shell ls /dev/tty.usbmodem*))
ISP_MCU 	= $(subst atmega,m,$(subst attiny,t,$(MCU)))
ISP_TOOL 	= stk500v1

AVRDUDE_ARGS += -P $(ISP_PORT) -p $(ISP_MCU) -c $(ISP_TOOL) -b $(ISP_SPEED)

###########################################
# Functions
###########################################

# Replace .c extensions to .o in $(2) and prefix the results by $(1)
TO_OBJECTS = $(addprefix $(1), $(patsubst %.c,%.o,$(2)))

###########################################
# Build configuration
###########################################

SRC				= ./
BIN				= ./bin/
OBJ         	= $(BIN)$(MCU)/

CC				= $(AVR_CC)
INCLUDES		+= -I "$(AVR_DIR)avr/$(MCU)/include"
override CFLAGS	+= -std=gnu99 -c -g -Os -Wall -Wextra -MMD -mmcu=$(MCU) -DF_CPU=$(F_CPU) -fno-exceptions -ffunction-sections -fdata-sections -fdiagnostics-color=auto $(INCLUDES)
LDFLAGS 		+= -mmcu=$(MCU) -fdiagnostics-color=auto -Wl,-static -Wl,--gc- -finline-functions

MCU				= attiny85
F_CPU			= 16000000

SRC_FILES_SHOPVAC	= shopvac.c
OBJECTS_SHOPVAC		= $(call TO_OBJECTS, $(BIN)attiny85/, $(SRC_FILES_SHOPVAC))

.PHONY: tst
tst:
	@echo CFLAGS: $(CFLAGS)

.PHONY: shopvac shopvac-isp shopvac-size
shopvac: INCLUDES	= -I~/Library/Arduino15/packages/attiny/hardware/avr/1.0.2/variants/tiny8
shopvac: $(BIN)shopvac.hex

shopvac-isp: avr-upload-shopvac
shopvac-size: avr-size-shopvac

$(BIN)shopvac.elf: $(OBJECTS_SHOPVAC)
	@mkdir -p $(shell dirname $@)
	$(CC) $< $(LDFLAGS) -o $@

###########################################
# Build rules
###########################################

.PRECIOUS: %.elf

$(BIN)atmega328p/%.o: %.c
	@mkdir -p $(shell dirname $@)
	$(CC) $< $(CFLAGS) -c -o $@

$(BIN)attiny85/%.o: %.c
	@mkdir -p $(shell dirname $@)
	$(CC) $< $(CFLAGS) -c -o $@

$(BIN)%.hex: $(BIN)%.elf
	@mkdir -p $(shell dirname $@)
	$(AVR_OBJCOPY) -O ihex -R .eeprom $< $@

$(BIN)%.eep: $(BIN)%.elf
	@mkdir -p $(shell dirname $@)
	$(AVR_OBJCOPY) -O ihex -j .eeprom --set-section-flags=.eeprom=alloc,load --no-change-warnings --change-section-lma .eeprom=0 $< $@

$(BIN)%.dump: $(BIN)%.hex
	$(AVR_OBJDUMP) -m avr -D $< > $@

###########################################
# AVR tool targets
###########################################

.PHONY: avr-upload-% avr-size-%

avr-upload-%: %
	$(AVRDUDE) $(AVRDUDE_ARGS) $(ISP_FUSES) -U flash:w:$(BIN)$*.hex:i

avr-size-%: %
	$(AVR_SIZE) --mcu=$(MCU) $(BIN)$*.elf

###########################################
# Housekeeping
###########################################

.PHONY: clean

clean:
	rm -rf $(BIN)


# include deps lists build with gcc -MMD flag
ifneq "$(wildcard $(BIN) )" ""
-include $(shell find $(BIN) -name "**/*.d")
endif
