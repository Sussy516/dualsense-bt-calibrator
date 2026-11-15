import struct
import unittest
from unittest.mock import patch
import zlib

from dualsense_calibrator.hid_protocol import (
    SDL_CENTER,
    build_ds5_feature_packet,
    encode_finetune_payload,
    flash_finetune_table_on_device,
    parse_finetune_table,
)
from dualsense_calibrator.minimax import compute_minimax_table, validate_resting_axes


class _FakeDualSenseHID:
    def __init__(self, initial_table: list[int]) -> None:
        self.sent: list[list[int]] = []
        self.table = list(initial_table)

    def send_feature_report(self, pkt: list[int]) -> int:
        self.sent.append(pkt)
        if pkt[0] == 0x80 and pkt[1] == 12 and pkt[2] == 1:
            self.table = [
                struct.unpack_from("<H", bytes(pkt[3:27]), 2 * i)[0]
                for i in range(12)
            ]
        return len(pkt)

    def get_feature_report(self, report_id: int, size: int) -> bytes:
        buf = bytearray(size)
        buf[0] = report_id
        for idx, val in enumerate(self.table):
            struct.pack_into("<H", buf, 4 + 2 * idx, val)
        return bytes(buf)


class TestDualSenseCalibrator(unittest.TestCase):
    def test_feature_packet_crc32_and_length(self) -> None:
        pkt = build_ds5_feature_packet(0x80, [12, 2], total_len=64)
        self.assertEqual(len(pkt), 64)
        payload = bytes(pkt[:60])
        expected_crc = zlib.crc32(bytes([0x53]) + payload) & 0xFFFFFFFF
        actual_crc = int.from_bytes(bytes(pkt[60:64]), "little")
        self.assertEqual(actual_crc, expected_crc)

    def test_encode_and_parse_finetune_table_clamping(self) -> None:
        raw_table = [-50, 100, 200, 300, 4096, 3900, 3950, 4000, 2048, 2050, 2030, 2040]
        encoded = encode_finetune_payload(raw_table)
        self.assertEqual(encoded[:2], [12, 1])

        report = bytearray(64)
        report[0] = 0x81
        for i in range(12):
            report[4 + 2 * i] = encoded[2 + 2 * i]
            report[4 + 2 * i + 1] = encoded[2 + 2 * i + 1]

        parsed = parse_finetune_table(bytes(report))
        self.assertEqual(parsed[0], 0)
        self.assertEqual(parsed[4], 4095)
        self.assertEqual(parsed[8], 2048)

    def test_compute_minimax_table_and_validation(self) -> None:
        base = [200, 210, 190, 205, 3800, 3810, 3790, 3805, 2000, 2010, 1990, 2005]
        samples = [
            [SDL_CENTER + 0.02, SDL_CENTER - 0.04, SDL_CENTER, SDL_CENTER],
            [SDL_CENTER + 0.04, SDL_CENTER + 0.00, SDL_CENTER, SDL_CENTER],
            [SDL_CENTER + 0.01, SDL_CENTER - 0.02, SDL_CENTER, SDL_CENTER],
            [SDL_CENTER + 0.03, SDL_CENTER - 0.02, SDL_CENTER, SDL_CENTER],
        ]
        new_table, mids = compute_minimax_table(base, samples)
        self.assertAlmostEqual(mids[0], SDL_CENTER + 0.025, places=4)
        self.assertAlmostEqual(mids[1], SDL_CENTER - 0.020, places=4)
        self.assertGreater(new_table[8], base[8])
        self.assertLess(new_table[9], base[9])
        self.assertEqual(new_table[10], base[10])

        self.assertTrue(validate_resting_axes([0.01, -0.02, 0.0, 0.03]))
        self.assertFalse(validate_resting_axes([0.5, 0.0, 0.0, 0.0]))

    def test_flash_sequence_with_mock_device(self) -> None:
        initial = [200] * 4 + [3800] * 4 + [2000, 2005, 2010, 2015]
        target = [200] * 4 + [3800] * 4 + [2040, 2020, 1995, 2030]
        fake_dev = _FakeDualSenseHID(initial)

        with patch("dualsense_calibrator.hid_protocol.time.sleep", return_value=None):
            verified, status = flash_finetune_table_on_device(fake_dev, target)

        self.assertEqual(verified, target)
        self.assertEqual(len(status), 8)
        # Verify NVS unlock [3, 2, 101, 50, 64, 12] and lock [3, 1] were sent
        commands = [pkt[:4] for pkt in fake_dev.sent]
        self.assertIn([0x80, 3, 2, 101], commands)
        self.assertIn([0x80, 3, 1, 0], commands)
