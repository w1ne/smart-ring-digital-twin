#!/usr/bin/env python3
"""Require complete sensor, overflow-recovery and BLE-load evidence."""

import sys
from pathlib import Path

REQUIRED = (
    "[init] imu    BMI270   ok (CHIP_ID=0x24, init_ok)",
    "[init] ppg    MAX30102 ok (PART_ID=0x15, 1600 Hz)",
    "[init] temp   TMP117   ok (DEVICE_ID=0x0117)",
    "[init] haptic DRV2605  ok",
    "overflow handled gracefully: loss counted, latest 100 retained.",
    "PPG FIFO overflows ONLY under BLE load",
    "=== done ===",
)


def main() -> int:
    if len(sys.argv) != 2:
        print("usage: check_uart.py UART_LOG", file=sys.stderr)
        return 2
    output = Path(sys.argv[1]).read_text()
    missing = [marker for marker in REQUIRED if marker not in output]
    failed = "FAIL" in output or "inconclusive:" in output
    if missing or failed:
        for marker in missing:
            print(f"Missing firmware evidence: {marker}", file=sys.stderr)
        if failed:
            print("Firmware reported failed or inconclusive evidence", file=sys.stderr)
        return 1
    print(f"PASS: all {len(REQUIRED)} firmware evidence markers present")
    return 0


if __name__ == "__main__":
    sys.exit(main())
