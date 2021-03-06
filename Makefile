# Arduino Library base folder and example structure
DSC_ARDUINO_BASE = dsc_arduino
DSC_ARDUINO ?= dsc_arduino
DSC_DEBUG_BASE = debug/current_sensor_test

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
# BUILD_DIR=_build
# BUILD_PATH=$(PWD)/$(DSC_ARDUINO_BASE)/$(BUILD_DIR)

ifneq ($(V), 0)
	VERBOSE=-v
endif

.PHONY: build build-debug-script program program-debug-script

build:
	arduino-cli compile $(VERBOSE) --clean -b $(BOARD_TYPE) --warnings all $(DSC_ARDUINO_BASE)

build-debug-script:
	arduino-cli compile $(VERBOSE) --clean -b $(BOARD_TYPE) --warnings all $(DSC_DEBUG_BASE)

program: build
	arduino-cli upload $(VERBOSE) -b $(BOARD_TYPE) -p $(SERIAL_PORT) -t $(DSC_ARDUINO_BASE)

program-debug-script: build-debug-script
	arduino-cli upload $(VERBOSE) -b $(BOARD_TYPE) -p $(SERIAL_PORT) -t $(DSC_DEBUG_BASE)

# clean:
# 	@rm -rf $(BUILD_PATH)
# 	@rm $(DSC_ARDUINO_BASE)/*.elf
# 	@rm $(DSC_ARDUINO_BASE)/*.hex
