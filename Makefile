# Arduino Library base folder and example structure
DSC_ARDUINO_BASE = dsc_arduino
DSC_ARDUINO ?= Example1_BasicReadings

# Arduino CLI executable name and directory location
ARDUINO_CLI = arduino-cli
ARDUINO_CLI_DIR = .

# Arduino CLI Board type
BOARD_TYPE ?= adafruit:samd:adafruit_feather_m0_express

# Default port to upload to
SERIAL_PORT ?= COM1

# Optional verbose compile/upload trigger
V ?= 0
VERBOSE=

# Build path -- used to store built binary and object files
BUILD_DIR=_build
BUILD_PATH=$(PWD)/$(DSC_ARDUINO_BASE)/$(DSC_ARDUINO)/$(BUILD_DIR)

ifneq ($(V), 0)
    VERBOSE=-v
endif

.PHONY: all example program clean

all: example

example:
    $(ARDUINO_CLI_DIR)/$(ARDUINO_CLI) compile $(VERBOSE) --build-path=$(BUILD_PATH) --build-cache-path=$(BUILD_PATH) -b $(BOARD_TYPE) $(DSC_ARDUINO_BASE)/$(DSC_ARDUINO)

program:
    $(ARDUINO_CLI_DIR)/$(ARDUINO_CLI) upload $(VERBOSE) -p $(SERIAL_PORT) --fqbn $(BOARD_TYPE) $(DSC_ARDUINO_BASE)/$(DSC_ARDUINO)

clean:
    @rm -rf $(BUILD_PATH)
    @rm $(DSC_ARDUINO_BASE)/$(DSC_ARDUINO)/*.elf
    @rm $(DSC_ARDUINO_BASE)/$(DSC_ARDUINO)/*.hex
