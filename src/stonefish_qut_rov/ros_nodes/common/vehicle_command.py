"""Normalised vehicle-level efforts shared by the real and sim interfaces."""
from dataclasses import dataclass


@dataclass
class VehicleCommand:
    surge: float = 0.0
    heave: float = 0.0
    yaw: float = 0.0

    def zero(self):
        self.surge = 0.0
        self.heave = 0.0
        self.yaw = 0.0
