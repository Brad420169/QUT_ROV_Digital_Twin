"""Battery presentation for the operator."""
import math
import numpy as np
from rov_config import BATTERY_CELL_CURVE, BATTERY_CELLS, BATTERY_MIN_PLAUSIBLE_V

_CURVE_V = [v for v, _ in BATTERY_CELL_CURVE]
_CURVE_PCT = [pct for _, pct in BATTERY_CELL_CURVE]

def percent_from_voltage(voltage: float):
    """
    Estimate charge from resting pack voltage using a per-cell curve.

    Returns None when the voltage is too low to be a healthy 4S LiPo —
    a bench supply or a flat pack — rather than reporting a
    misleading 0%.
    """
    if not math.isfinite(voltage) or voltage < BATTERY_MIN_PLAUSIBLE_V:
        return None

    cell = voltage / BATTERY_CELLS
    return float(np.interp(cell, _CURVE_V, _CURVE_PCT))

def report_battery(node, battery, mode):
    """Print the pack state. B button."""
    if mode == "sim":
        node.get_logger().info("Battery reporting is real-mode only.")
        return

    if battery is None:
        node.get_logger().warn(
            "No battery data yet — is MAVROS connected?"
        )
        return

    voltage = float(battery.voltage)
    current = float(battery.current)
    cell = voltage / BATTERY_CELLS

    percent = percent_from_voltage(voltage)

    if percent is None:
        level = "n/a (not a 4S pack?)"
    else:
        level = f"{percent:.0f}%"

    node.get_logger().info(
        f"Battery: {voltage:.2f} V  ({cell:.2f} V/cell)  "
        f"{level}   draw {abs(current):.1f} A"
    )

    if percent is not None and percent <= 20.0:
        node.get_logger().warn("Battery low — surface soon.")

