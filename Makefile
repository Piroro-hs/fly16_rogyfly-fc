TARGET = $(shell sed -En 's/^name\s*=\s*"(.+)"/\1/p' Cargo.toml)

DEBUG = 1

ifeq ($(DEBUG), 0)
	OPT = --release
endif

ifeq ($(DEBUG), 1)
	TARGET_DIR = target/thumbv7em-none-eabihf/debug
else
	TARGET_DIR = target/thumbv7em-none-eabihf/release
endif

PREFIX = rust-
BIN = $(PREFIX)objcopy -O binary -S
SZ = $(PREFIX)size
NM = $(PREFIX)nm

all: $(TARGET_DIR)/$(TARGET).bin size

$(TARGET_DIR)/$(TARGET): FORCE
	cargo build $(OPT)

$(TARGET_DIR)/$(TARGET).bin: $(TARGET_DIR)/$(TARGET)
	$(BIN) $< $@

.PHONY: size
size: $(TARGET_DIR)/$(TARGET)
	$(SZ) $<
	@echo $(shell echo \
		"obase=10; ibase=16;" \
		"$(shell $(NM) $< | grep _stack_start | cut -d ' ' -f 1 | tr '[:lower:]' '[:upper:]') -" \
		"$(shell $(NM) $< | grep __sheap | cut -d ' ' -f 1 | tr '[:lower:]' '[:upper:]')" \
	| bc) byte is left for heap + stack.

.PHONY: clean
clean:
	cargo clean

.PHONY: FORCE
FORCE:
