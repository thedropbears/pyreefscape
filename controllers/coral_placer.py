import math

from magicbot import StateMachine, state, tunable

from components.wrist import WristComponent


class CoralPlacer(StateMachine):
    wrist: WristComponent

    CORAL_PLACE_ANGLE = tunable(
        -5.0
    )  # In degrees for tuning, converted to radians in tilt-to call

    def __init__(self):
        pass

    def place(self) -> None:
        self.engage("lowering")

    def lift(self) -> None:
        self.engage()

    @state(first=True, must_finish=True)
    def raising(self, initial_call: bool) -> None:
        if initial_call:
            self.wrist.tilt_to(math.radians(self.CORAL_PLACE_ANGLE))

    @state(must_finish=True)
    def lowering(self, initial_call: bool) -> None:
        if initial_call:
            self.wrist.go_to_neutral()
        if self.wrist.at_setpoint():
            self.done()

    def done(self) -> None:
        super().done()
