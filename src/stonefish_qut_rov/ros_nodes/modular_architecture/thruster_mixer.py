"""
Stonefish thruster mixer.

Input: normalised surge/heave/yaw
Output order: [TL, TR, BL, BR]

TL/TR = vertical
BL/BR = horizontal

Do not manually invert TR or BR here because the Stonefish XML already
uses inverted_setpoint="true" for those actuators.
"""

from dataclasses import dataclass

from control_utils import clamp


@dataclass
class VehicleCommand:
    surge: float = 0.0
    heave: float = 0.0
    yaw: float = 0.0

    def zero(self):
        self.surge = 0.0
        self.heave = 0.0
        self.yaw = 0.0


def mix_normalised(command: VehicleCommand) -> list[float]:
    surge = clamp(command.surge)
    heave = clamp(command.heave)
    yaw = clamp(command.yaw)

    tl = heave
    tr = heave

    # Preserve the user's old controller sign convention.
    bl = surge - yaw
    br = surge + yaw

    largest = max(
        1.0,
        abs(tl),
        abs(tr),
        abs(bl),
        abs(br),
    )

    return [
        tl / largest,
        tr / largest,
        bl / largest,
        br / largest,
    ]


def mix_to_setpoints(command: VehicleCommand, max_setpoint: float) -> list[float]:
    return [
        value * max_setpoint
        for value in mix_normalised(command)
    ]
