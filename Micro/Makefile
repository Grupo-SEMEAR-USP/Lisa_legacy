COM ?= /dev/ttyUSB0

FQBN  ?= esp8266:esp8266:generic
BOARD ?= esp8266.esp8266.generic

BUILDD    = ./build
INO_FILES = $(wildcard ./*.ino)

OUT_FILE  = $(shell basename $(PWD)).ino.elf
ELF_FILE  = $(BUILDD)/$(BOARD)/$(OUT_FILE)


.PHONY: all clean flash

all: $(ELF_FILE)

clean:
	@rm -rf $(BUILDD)

flash:
	arduino-cli upload --fqbn $(FQBN) -p $(COM) .


$(ELF_FILE): $(INO_FILES)
	arduino-cli compile --fqbn $(FQBN) . --build-property 'build.extra_flags=$(EXTRA_FLAGS)' --export-binaries
