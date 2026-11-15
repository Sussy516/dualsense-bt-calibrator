"""4-way minimax ADC center midpoint calculation."""

from dualsense_calibrator.hid_protocol import SDL_CENTER


def validate_resting_axes(axes: list[float], max_abs: float = 0.25) -> bool:
    """Return True if all four stick axes are near resting center."""
    return len(axes) == 4 and all(abs(val) <= max_abs for val in axes)


def compute_minimax_table(
    base_table: list[int],
    samples: list[list[float]],
    max_delta_adc: int = 300,
) -> tuple[list[int], list[float]]:
    """Compute updated 12-entry NVS center table using 4-way minimax midpoint."""
    if len(base_table) != 12:
        raise ValueError("base_table must contain 12 entries")
    if not samples or any(len(row) != 4 for row in samples):
        raise ValueError("samples must be a non-empty list of 4-axis frames")

    half_spans = [
        (base_table[4] - base_table[0]) / 2.0,  # LX: (LR - LL) / 2
        (base_table[5] - base_table[1]) / 2.0,  # LY: (LB - LT) / 2
        (base_table[6] - base_table[2]) / 2.0,  # RX: (RR - RL) / 2
        (base_table[7] - base_table[3]) / 2.0,  # RY: (RB - RT) / 2
    ]
    new_table = list(base_table)
    mids: list[float] = []
    for k in range(4):
        col = [row[k] for row in samples]
        mid_axis = (min(col) + max(col)) / 2.0
        mids.append(mid_axis)
        delta_adc = round((mid_axis - SDL_CENTER) * half_spans[k])
        delta_adc = max(-max_delta_adc, min(max_delta_adc, delta_adc))
        new_table[8 + k] = base_table[8 + k] + delta_adc
    return new_table, mids
