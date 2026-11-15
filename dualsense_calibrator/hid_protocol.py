"""Sony DualSense Bluetooth HID feature report (0x53 CRC32) & NVS flash protocol."""

import struct
import time
from typing import Any
import zlib

VENDOR_ID = 0x054C
PRODUCT_ID = 0x0CE6
SDL_CENTER = 0.0039  # Raw HID 0x80 (128 / 255) neutral center in SDL mapping.


def build_ds5_feature_packet(report_id: int, data: list[int], total_len: int = 64) -> list[int]:
    """Build a 64-byte DualSense Bluetooth feature report with 0x53 CRC32 trailer."""
    pad_len = total_len - 4 - 1 - len(data)
    if pad_len < 0:
        raise ValueError(f"Payload too long for {total_len}-byte feature report")
    payload = bytes([report_id] + data + [0] * pad_len)
    crc = zlib.crc32(bytes([0x53]) + payload) & 0xFFFFFFFF
    return list(payload + crc.to_bytes(4, "little"))


def parse_finetune_table(raw_report: bytes) -> list[int]:
    """Unpack the 12-entry 12-bit ADC fine-tune calibration table from report 0x81."""
    if len(raw_report) < 28:
        raise ValueError(f"Expected >= 28 bytes in report 0x81, got {len(raw_report)}")
    return [struct.unpack_from("<H", raw_report, 4 + 2 * i)[0] for i in range(12)]


def encode_finetune_payload(new_table: list[int]) -> list[int]:
    """Encode 12 ADC calibration values (clamped 0..4095) for report 0x80 [12, 1]."""
    if len(new_table) != 12:
        raise ValueError(f"Expected 12 entries in fine-tune table, got {len(new_table)}")
    data = [12, 1]
    for val in new_table:
        clamped = max(0, min(4095, int(val)))
        data.extend([clamped & 0xFF, (clamped >> 8) & 0xFF])
    return data


def send_ds5_feature(dev: Any, report_id: int, data: list[int], total_len: int = 64) -> int:
    """Send a CRC32-authenticated DualSense Bluetooth HID feature report."""
    pkt = build_ds5_feature_packet(report_id, data, total_len=total_len)
    return dev.send_feature_report(pkt)


def read_finetune_table(dev: Any) -> list[int]:
    """Query the current 12-entry stick ADC calibration table from the controller."""
    send_ds5_feature(dev, 0x80, [12, 2])
    time.sleep(0.06)
    raw = bytes(dev.get_feature_report(0x81, 64))
    return parse_finetune_table(raw)


def query_nv_status(dev: Any) -> list[int]:
    """Query DualSense NVS lock status bytes."""
    send_ds5_feature(dev, 0x80, [3, 3])
    time.sleep(0.05)
    return list(dev.get_feature_report(0x81, 64)[:8])


def unlock_nvs(dev: Any) -> None:
    """Unlock DualSense NVS flash for calibration writes."""
    send_ds5_feature(dev, 0x80, [3, 2, 101, 50, 64, 12])
    time.sleep(0.08)
    dev.get_feature_report(0x81, 64)


def lock_nvs(dev: Any) -> None:
    """Commit RAM calibration updates to NVS flash and re-lock NVS."""
    send_ds5_feature(dev, 0x80, [3, 1])
    time.sleep(0.08)
    dev.get_feature_report(0x81, 64)


def flash_finetune_table_on_device(dev: Any, new_table: list[int]) -> tuple[list[int], list[int]]:
    """Write a 12-entry fine-tune table, commit to NVS flash, and verify readback."""
    data = encode_finetune_payload(new_table)
    send_ds5_feature(dev, 0x80, data)
    time.sleep(0.06)
    unlock_nvs(dev)
    lock_nvs(dev)
    status = query_nv_status(dev)
    verified = read_finetune_table(dev)
    return verified, status


def flash_finetune_table(dev_path: bytes, new_table: list[int]) -> tuple[list[int], list[int]]:
    """Open HID device by path, flash fine-tune table to NVS, and close."""
    import hid

    dev = hid.device()
    dev.open_path(dev_path)
    try:
        return flash_finetune_table_on_device(dev, new_table)
    finally:
        dev.close()
