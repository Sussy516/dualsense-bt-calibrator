# dualsense-bt-calibrator

Sony DualSense (PS5) wireless stick drift calibrator over **Bluetooth** on macOS — no USB-C cable required.

## How It Works
- Computes Sony's Bluetooth HID `SET_REPORT` (`0x53`) **CRC32 trailer** in Python (`hidapi`) so feature reports `0x80` / `0x82` are accepted wirelessly over Bluetooth.
- Supports two calibration modes:
  1. **`dualsense_bt_calibrator.py`** — triggers Sony's built-in hardware center calibration routine (`0x82 [1, 1, 1]` → sample → finalize) and commits to permanent NVS flash.
  2. **`dualsense_4way_calibrator.py`** — interactive SDL2/Pygame **4-way minimax** center calibrator with live **10x zoom** stick plots, resting-state averaging, NVS fine-tune flashing (`0x80 [12, 1]`), and 1-key backup restore (`R`).

## Install
```bash
pip install -r requirements.txt
```

## Usage
```bash
# Quick wireless hardware center calibration
python3 dualsense_bt_calibrator.py

# Interactive 4-way minimax GUI calibrator
python3 dualsense_4way_calibrator.py
```

## Run Unit Tests
```bash
python3 -m unittest discover -s tests -v
```
