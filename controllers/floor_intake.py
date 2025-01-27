import math

from magicbot import StateMachine, state, tunable

from components.algae_manipulator import AlgaeManipulatorComponent
from components.intake import IntakeComponent
from components.wrist import WristComponent


class FloorIntake(StateMachine):
    algae_manipulator_component: AlgaeManipulatorComponent
    wrist: WristComponent
    intake_component: IntakeComponent

    HANDOFF_POSITION = tunable(math.radians(-108.0))

    def __init__(self):
        pass

    def intake(self) -> None:
        self.engage()

    @state(first=True, must_finish=True)
    def intaking(self, initial_call: bool):
        if self.algae_manipulator_component.has_algae():
            self.done()
            return

        self.intake_component.intake()

        if initial_call:
            self.wrist.tilt_to(self.HANDOFF_POSITION)

        self.algae_manipulator_component.intake()

    def done(self) -> None:
        super().done()
        self.wrist.go_to_neutral()
