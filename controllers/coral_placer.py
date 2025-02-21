import math

from magicbot import StateMachine, state, tunable

from components.coral_placer import CoralPlacerComponent
from components.wrist import WristComponent


class CoralPlacer(StateMachine):
    coral_placer_component: CoralPlacerComponent
    wrist: WristComponent

    CORAL_PLACE_ANGLE = tunable(math.radians(-5.0))

    def __init__(self):
        pass

    def place(self) -> None:
        self.engage()

    @state(first=True, must_finish=True)
    def raising(self, initial_call: bool) -> None:
        if initial_call:
            self.wrist.tilt_to(self.CORAL_PLACE_ANGLE)
        if self.wrist.at_setpoint():
            self.done()

    def done(self) -> None:
        self.wrist.go_to_neutral()
        super().done()
