from magicbot import StateMachine, state, tunable

from components.algae_manipulator import AlgaeManipulatorComponent
from components.wrist import WristComponent


class AlgaeIntake(StateMachine):
    algae_manipulator_component: AlgaeManipulatorComponent
    wrist: WristComponent

    L2_INTAKE_ANGLE = tunable(40.0)
    L3_INTAKE_ANGLE = tunable(40.0)

    def __init__(self):
        pass

    def setup(self) -> None:
        self.preferred_intake_angle = self.L2_INTAKE_ANGLE

    @state(first=True)
    def raising(self):
        if self.algae_manipulator_component.has_algae():
            self.done()
            return

        self.wrist.tilt_to(self.preferred_intake_angle)

        if self.wrist.at_setpoint():
            self.next_state("intaking")

    @state
    def intaking(self):
        if self.algae_manipulator_component.has_algae():
            self.done()
            return

        self.algae_manipulator_component.intake()

    def prefer_l2(self) -> None:
        self.preferred_intake_angle = self.L2_INTAKE_ANGLE

    def prefer_l3(self) -> None:
        self.preferred_intake_angle = self.L3_INTAKE_ANGLE

    def intake(self):
        self.engage()

    def done(self) -> None:
        super().done()
        self.wrist.go_to_neutral()
