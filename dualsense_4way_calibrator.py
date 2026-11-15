#!/usr/bin/env python3
"""Interactive 4-way minimax DualSense center calibrator over Bluetooth (macOS).

Captures resting center drift after releasing both sticks from Up, Down, Left,
and Right, computes the minimax midpoint for each axis, writes the updated 12-bit
ADC center table via HID report 0x80 [12, 1], and commits it to NVS flash.
"""

import math
import os
import subprocess
import sys
import time

import hid
import pygame
from pygame._sdl2 import controller

from dualsense_calibrator.hid_protocol import (
    PRODUCT_ID,
    SDL_CENTER,
    VENDOR_ID,
    flash_finetune_table,
    read_finetune_table,
)
from dualsense_calibrator.minimax import compute_minimax_table, validate_resting_axes

STEP_PROMPTS = [
    ("Step 1/4: UP", "Push BOTH sticks full UP, release to rest, then press Cross (or Space)"),
    ("Step 2/4: DOWN", "Push BOTH sticks full DOWN, release to rest, then press Cross (or Space)"),
    ("Step 3/4: LEFT", "Push BOTH sticks full LEFT, release to rest, then press Cross (or Space)"),
    ("Step 4/4: RIGHT", "Push BOTH sticks full RIGHT, release to rest, then press Cross (or Space)"),
]


def bring_window_to_front() -> None:
    if sys.platform != "darwin":
        return
    pid = os.getpid()
    script = (
        f'tell application "System Events" to set frontmost of the first process '
        f"whose unix id is {pid} to true"
    )
    subprocess.Popen(["osascript", "-e", script], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def draw_stick_plot(
    surface: pygame.Surface,
    cx: int,
    cy: int,
    radius: int,
    x_val: float,
    y_val: float,
    zoom_10x: bool,
    title: str,
    font_md: pygame.font.Font,
    font_sm: pygame.font.Font,
    trail: list[tuple[float, float]],
) -> None:
    pygame.draw.circle(surface, (28, 32, 42), (cx, cy), radius)
    pygame.draw.circle(surface, (70, 80, 100), (cx, cy), radius, 2)

    scale = 10.0 if zoom_10x else 1.0
    dz5_r = min(radius, int(radius * 0.05 * scale))
    dz10_r = min(radius, int(radius * 0.10 * scale))
    if dz10_r > 3:
        pygame.draw.circle(surface, (55, 65, 85), (cx, cy), dz10_r, 1)
    if dz5_r > 3:
        pygame.draw.circle(surface, (45, 95, 70), (cx, cy), dz5_r, 1)

    pygame.draw.line(surface, (55, 60, 75), (cx - radius, cy), (cx + radius, cy), 1)
    pygame.draw.line(surface, (55, 60, 75), (cx, cy - radius), (cx, cy + radius), 1)

    for tx, ty in trail:
        px = cx + max(-1.0, min(1.0, tx * scale)) * radius
        py = cy + max(-1.0, min(1.0, ty * scale)) * radius
        pygame.draw.circle(surface, (90, 140, 220), (int(px), int(py)), 4)

    px = cx + max(-1.0, min(1.0, x_val * scale)) * radius
    py = cy + max(-1.0, min(1.0, y_val * scale)) * radius
    err = math.hypot(x_val - SDL_CENTER, y_val - SDL_CENTER)
    dot_color = (70, 225, 130) if err <= 0.025 else ((250, 200, 70) if err <= 0.05 else (255, 85, 85))
    pygame.draw.line(surface, (140, 150, 175), (cx, cy), (int(px), int(py)), 2)
    pygame.draw.circle(surface, dot_color, (int(px), int(py)), 8)

    label = font_md.render(f"{title} {'(10x zoom)' if zoom_10x else '(1x)'}", True, (235, 240, 250))
    surface.blit(label, (cx - label.get_width() // 2, cy - radius - 34))

    coords = font_sm.render(f"X: {x_val*100:+.2f}%   Y: {y_val*100:+.2f}%", True, dot_color)
    surface.blit(coords, (cx - coords.get_width() // 2, cy + radius + 12))


def main() -> int:
    devs = hid.enumerate(VENDOR_ID, PRODUCT_ID)
    if not devs:
        print("ERROR: DualSense controller not found via HID.", file=sys.stderr)
        return 1

    dev_path = devs[0]["path"]
    serial = devs[0].get("serial_number", "unknown")

    dev = hid.device()
    dev.open_path(dev_path)
    initial_table = read_finetune_table(dev)
    dev.close()
    print(f"Connected {serial}, initial NVS table: {initial_table}", flush=True)

    pygame.init()
    pygame.joystick.init()
    controller.init()
    if pygame.joystick.get_count() == 0:
        print("ERROR: No Pygame joystick detected.", file=sys.stderr)
        return 1

    joy = pygame.joystick.Joystick(0)
    joy.init()
    ctrl = controller.Controller(0) if controller.is_controller(0) else None
    if ctrl:
        ctrl.init()

    screen = pygame.display.set_mode((880, 560))
    pygame.display.set_caption(f"DualSense 4-Way Center Calibrator ({serial})")
    bring_window_to_front()

    font_path = pygame.font.match_font("Arial") or pygame.font.match_font("Helvetica Neue")
    font_lg = pygame.font.Font(font_path, 23)
    font_md = pygame.font.Font(font_path, 17)
    font_sm = pygame.font.Font(font_path, 15)

    current_table = list(initial_table)
    captured: list[list[float]] = []
    phase = "CALIBRATING"
    warn_msg = ""
    warn_until = 0.0
    zoom_10x = True
    clock = pygame.time.Clock()
    running = True

    while running:
        trigger_capture = False
        trigger_reset = False
        trigger_restore = False

        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                running = False
            elif ev.type == pygame.KEYDOWN:
                if ev.key == pygame.K_ESCAPE:
                    running = False
                elif ev.key == pygame.K_SPACE:
                    trigger_capture = True
                elif ev.key == pygame.K_z:
                    zoom_10x = not zoom_10x
                elif ev.key == pygame.K_t:
                    trigger_reset = True
                elif ev.key == pygame.K_r:
                    trigger_restore = True
            elif ev.type == pygame.CONTROLLERBUTTONDOWN:
                if ev.button == pygame.CONTROLLER_BUTTON_A:
                    trigger_capture = True
                elif ev.button == pygame.CONTROLLER_BUTTON_B and phase == "DONE_VIEW":
                    running = False
                elif ev.button == pygame.CONTROLLER_BUTTON_Y:
                    trigger_reset = True
                elif ev.button == pygame.CONTROLLER_BUTTON_X:
                    zoom_10x = not zoom_10x
            elif ev.type == pygame.JOYBUTTONDOWN and not ctrl:
                if ev.button == 0:
                    trigger_capture = True
                elif ev.button == 1 and phase == "DONE_VIEW":
                    running = False
                elif ev.button == 3:
                    trigger_reset = True

        axes = [joy.get_axis(i) for i in range(4)]

        if trigger_reset:
            captured.clear()
            phase = "CALIBRATING"
            warn_msg = "Reset - restart the 4 steps from current table"
            warn_until = time.time() + 2.5

        if trigger_restore and phase == "DONE_VIEW":
            current_table, status = flash_finetune_table(dev_path, initial_table)
            print(f"RESTORED initial table: {current_table}, status={status}", flush=True)
            warn_msg = "Restored initial NVS table from before calibration!"
            warn_until = time.time() + 3.0

        if trigger_capture and phase == "CALIBRATING":
            if not validate_resting_axes(axes):
                warn_msg = "Release BOTH sticks to center before capturing!"
                warn_until = time.time() + 2.5
            else:
                burst = []
                for _ in range(12):
                    pygame.event.pump()
                    burst.append([joy.get_axis(i) for i in range(4)])
                    time.sleep(0.015)
                mean_frame = [sum(r[i] for r in burst) / len(burst) for i in range(4)]
                captured.append(mean_frame)
                print(f"Step {len(captured)}/4 captured resting axes: {[round(x, 4) for x in mean_frame]}", flush=True)

                if len(captured) == 4:
                    new_table, mids = compute_minimax_table(current_table, captured)
                    verified_table, status = flash_finetune_table(dev_path, new_table)
                    current_table = verified_table
                    phase = "DONE_VIEW"
                    print(f"4-way minimax mids (LX,LY,RX,RY): {[round(m, 4) for m in mids]}", flush=True)
                    print(f"NEW NVS TABLE FLASHED: {verified_table}", flush=True)
                    print(f"NVS STATUS: {status}", flush=True)
                    warn_msg = "Flashed to NVS! Test stick movement and press Circle/ESC to exit."
                    warn_until = time.time() + 6.0

        screen.fill((17, 20, 26))

        if phase == "CALIBRATING":
            title_txt, sub_txt = STEP_PROMPTS[len(captured)]
            title_col = (120, 200, 255)
        else:
            title_txt = "NVS FLASHED! Verify stick centering and press Circle (or ESC)"
            sub_txt = "Triangle/T - retry 4 steps  |  Square/Z - toggle zoom (10x/1x)  |  R - restore backup"
            title_col = (80, 235, 145)

        screen.blit(font_lg.render(title_txt, True, title_col), (32, 20))
        screen.blit(font_sm.render(sub_txt, True, (195, 205, 220)), (32, 54))

        if time.time() < warn_until and warn_msg:
            screen.blit(font_md.render(warn_msg, True, (255, 210, 90)), (32, 86))
        else:
            progress = "   |   ".join(
                f"{lbl}: {'[OK]' if idx < len(captured) else '[  ]'}"
                for idx, lbl in enumerate(["1. Up", "2. Down", "3. Left", "4. Right"])
            )
            screen.blit(font_sm.render(progress, True, (160, 175, 195)), (32, 88))

        l_trail = [(c[0], c[1]) for c in captured]
        r_trail = [(c[2], c[3]) for c in captured]
        draw_stick_plot(screen, 235, 305, 155, axes[0], axes[1], zoom_10x, "Left Stick (L)", font_md, font_sm, l_trail)
        draw_stick_plot(screen, 645, 305, 155, axes[2], axes[3], zoom_10x, "Right Stick (R)", font_md, font_sm, r_trail)

        nvs_line = (
            f"NVS Centers [LX, LY, RX, RY]: {current_table[8:12]}   "
            f"(initial: {initial_table[8:12]})"
        )
        screen.blit(font_sm.render(nvs_line, True, (150, 165, 185)), (32, 515))

        pygame.display.flip()
        clock.tick(60)

    pygame.event.pump()
    final_axes = [round(joy.get_axis(i), 4) for i in range(4)]
    print(f"FINAL EXIT AXES (LX,LY,RX,RY): {final_axes}", flush=True)
    pygame.quit()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
