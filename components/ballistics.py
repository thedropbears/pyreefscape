import math
from dataclasses import dataclass
from typing import ClassVar

import wpiutil.wpistruct
from magicbot import feedback

from components.chassis import ChassisComponent


@wpiutil.wpistruct.make_wpistruct
@dataclass
class BallisticsSolution:
    WPIStruct: ClassVar

    speed: float
    inclination: float


class BallisticsComponent:
    chassis: ChassisComponent

    def __init__(self) -> None:
        pass

    def is_in_range(self) -> bool:
        return False

    def is_aligned(self) -> bool:
        return False

    @feedback
    def current_solution(self) -> BallisticsSolution:
        return BallisticsSolution(0.0, math.radians(-45.0))

    def execute(self) -> None:
        # TODO Update status LEDs when they are merged
        pass
