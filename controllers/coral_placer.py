import math

from magicbot import StateMachine, state, timed_state

from components.algae_manipulator import AlgaeManipulatorComponent
from components.wrist import WristComponent


class CoralPlacer(StateMachine):
    wrist: WristComponent
    algae_manipulator_component: AlgaeManipulatorComponent

    WRIST_TILT_POINT = math.radians(-10.0)

    def __init__(self):
        pass

    def place(self):
        self.engage()

    @state(first=True, must_finish=True)
    def lifting_wrist(self):
        self.wrist.tilt_to(self.WRIST_TILT_POINT)
        if self.wrist.at_setpoint():
            self.next_state("placing")

    @timed_state(duration=0.5, must_finish=True)
    def placing(self):
        self.algae_manipulator_component.inject()

    def done(self) -> None:
        super().done()
        self.wrist.go_to_neutral()
