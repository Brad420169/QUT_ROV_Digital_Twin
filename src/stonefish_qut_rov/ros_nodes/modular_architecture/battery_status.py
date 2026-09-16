"""Battery presentation for the operator."""
import math
from rov_config import BATTERY_CELL_CURVE, BATTERY_CELLS, BATTERY_MIN_PLAUSIBLE_V

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

    if cell <= BATTERY_CELL_CURVE[0][0]:
        return 0.0

    if cell >= BATTERY_CELL_CURVE[-1][0]:
        return 100.0

    for index in range(1, len(BATTERY_CELL_CURVE)):
        low_v, low_pct = BATTERY_CELL_CURVE[index - 1]
        high_v, high_pct = BATTERY_CELL_CURVE[index]

        if cell <= high_v:
            span = high_v - low_v
            fraction = 0.0 if span == 0 else (cell - low_v) / span
            return low_pct + fraction * (high_pct - low_pct)

    return 100.0

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

