#!/usr/bin/env python3
"""DualSense Wireless Bluetooth Hardware Calibrator (NVS Flash).

Works directly over Bluetooth (without USB-C cable) by computing Sony's
Bluetooth HID SET_REPORT (0x53) CRC32 trailer.
"""

import os
import time
import hid

from dualsense_calibrator.hid_protocol import (
    PRODUCT_ID,
    VENDOR_ID,
    lock_nvs,
    read_finetune_table,
    send_ds5_feature,
    unlock_nvs,
)

os.environ["SDL_VIDEODRIVER"] = "dummy"
import pygame  # noqa: E402


def read_axes() -> list[float]:
    """Read current LX, LY, RX, RY stick axes via SDL2/pygame."""
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        pygame.quit()
        return []
    joy = pygame.joystick.Joystick(0)
    joy.init()
    for _ in range(8):
        pygame.event.pump()
        time.sleep(0.03)
    axes = [round(joy.get_axis(a), 4) for a in range(4)]
    pygame.quit()
    return axes


def calibrate_dualsense_bt() -> None:
    """Run hardware stick center calibration over Bluetooth and commit to NVS."""
    print("1. Reading current stick axes (LX, LY, RX, RY)...")
    before_axes = read_axes()
    print(f"   Before: {before_axes}")

    dev = hid.device()
    dev.open(VENDOR_ID, PRODUCT_ID)
    try:
        table_before = read_finetune_table(dev)
        print(f"   NVS ADC table (before): {table_before}")

        unlock_nvs(dev)

        # Begin stick center calibration (0x82 [1, 1, 1])
        send_ds5_feature(dev, 0x82, [1, 1, 1])
        time.sleep(0.08)
        dev.get_feature_report(0x83, 64)

        # Sample 5 center frames (0x82 [3, 1, 1])
        for _ in range(5):
            time.sleep(0.1)
            send_ds5_feature(dev, 0x82, [3, 1, 1])
            time.sleep(0.05)
            dev.get_feature_report(0x83, 64)

        # Finalize stick center calibration (0x82 [2, 1, 1])
        time.sleep(0.1)
        send_ds5_feature(dev, 0x82, [2, 1, 1])
        time.sleep(0.08)
        end_resp = list(dev.get_feature_report(0x83, 64)[:8])
        print(f"   Calibration status (0x83): {end_resp}")

        time.sleep(0.1)
        lock_nvs(dev)

        table_after = read_finetune_table(dev)
        print(f"   NVS ADC table (after):  {table_after}")
    finally:
        dev.close()

    after_axes = read_axes()
    print(f"   After:  {after_axes}")


if __name__ == "__main__":
    calibrate_dualsense_bt()
