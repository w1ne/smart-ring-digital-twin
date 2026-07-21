# Build the smart-ring firmware for the nRF54L15.
#
# The sensor manager core is compiled from ../../src UNMODIFIED — the same
# sources the host unit tests build. There is no vendored copy and no #ifdef
# in the core; only the port and the drivers are target-specific. If the core
# had a hidden host dependency, this build would fail to link, which makes the
# portability claim testable rather than aspirational.
#
#   make            build ELF
#   make run        build and run it in the LabWired simulator
#   make clean

CROSS   ?= arm-none-eabi-
CC      := $(CROSS)gcc
SIZE    := $(CROSS)size

TARGET  := smart-ring-nrf54l15
BUILD   := build
CORE    := ../../src

# LabWired checkout providing the nRF54L15 profile and the smart-ring system.
LABWIRED ?= /Users/andrii/Projects/lw-nrf54l15

# Core (portable, shared with the host build) + target-only sources.
CORE_SRCS := $(CORE)/sm_ringbuf.c $(CORE)/sensor_manager.c $(CORE)/ble_batch.c
TGT_SRCS  := src/startup.c src/libc_shim.c src/board.c src/sm_port_nrf54l15.c src/main.c

OBJS := $(patsubst %.c,$(BUILD)/%.o,$(notdir $(CORE_SRCS) $(TGT_SRCS)))
vpath %.c $(CORE):src

ARCHFLAGS := -mcpu=cortex-m33 -mthumb -mfloat-abi=soft

CFLAGS  := $(ARCHFLAGS) -std=c11 -Os -g3 \
           -Wall -Wextra -Wpedantic -Werror \
           -ffreestanding -fno-common \
           -ffunction-sections -fdata-sections \
           -MMD -MP \
           -fno-tree-loop-distribute-patterns \
           -I$(CORE) -Isrc -Ishim \
           -DSM_PORT_BAREMETAL

LDFLAGS := $(ARCHFLAGS) -T nrf54l15.ld -nostdlib -nostartfiles \
           -Wl,--gc-sections -Wl,-Map=$(BUILD)/$(TARGET).map

# libgcc, even with -nostdlib. The sensor manager divides 64-bit microsecond
# timestamps in sm_rate_poll, and a 32-bit core has no 64-bit divide
# instruction, so GCC emits a call to __aeabi_uldivmod. That helper lives in
# libgcc, not libc — this is the compiler runtime, not the C library, and
# linking it does not drag in newlib.
#
# Worth knowing rather than just fixing: each of those divisions is on the
# order of a hundred cycles, and sm_rate_poll runs it on every fire. At 50 Hz
# that is irrelevant. If the acquisition rate ever climbs into the kHz range,
# replacing the divide with repeated subtraction (the backlog is 0 or 1 periods
# in the overwhelmingly common case) is the obvious optimisation.
LDLIBS  := -lgcc

all: $(BUILD)/$(TARGET).elf

$(BUILD):
	@mkdir -p $(BUILD)

$(BUILD)/%.o: %.c | $(BUILD)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD)/$(TARGET).elf: $(OBJS)
	$(CC) $(OBJS) $(LDFLAGS) $(LDLIBS) -o $@
	$(SIZE) $@

run: $(BUILD)/$(TARGET).elf
	$(LABWIRED)/target/release/labwired \
	    --firmware $(abspath $(BUILD)/$(TARGET).elf) \
	    --system $(LABWIRED)/configs/systems/smart-ring.yaml \
	    --max-steps 120000000

clean:
	rm -rf $(BUILD)

-include $(OBJS:.o=.d)

.PHONY: all run clean
