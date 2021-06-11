# Arduino Library base folder and example structure
DSC_ARDUINO_BASE = dsc_arduino
# DSC_ARDUINO ?= dsc_arduino

# Arduino CLI executable name and directory location
ARDUINO_CLI = arduino-cli
# ARDUINO_CLI_DIR = .

# Arduino CLI Board type
BOARD_TYPE ?= adafruit:samd:adafruit_feather_m0_express

# Default port to upload to
SERIAL_PORT ?= COM3

# Optional verbose compile/upload trigger
V ?= 0
VERBOSE=

# Build path -- used to store built binary and object files
BUILD_DIR=_build
BUILD_PATH=$(PWD)/$(DSC_ARDUINO_BASE)/$(BUILD_DIR)

ifneq ($(V), 0)
	VERBOSE=-v
endif

.PHONY: all dsc_arduino program clean

all: dsc_arduino

dsc_arduino:
	arduino-cli compile $(VERBOSE) -b $(BOARD_TYPE) $(DSC_ARDUINO_BASE)

program:
	arduino-cli upload $(VERBOSE) -p $(SERIAL_PORT) --fqbn $(BOARD_TYPE) $(DSC_ARDUINO_BASE)

clean:
	@rm -rf $(BUILD_PATH)
	@rm $(DSC_ARDUINO_BASE)/*.elf
	@rm $(DSC_ARDUINO_BASE)/*.hex
