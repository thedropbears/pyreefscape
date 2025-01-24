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

    @state(first=True)
    def raising_to_L2(self):
        if self.algae_manipulator_component.has_algae():
            self.done()
            return

        self.wrist.tilt_to(self.L2_INTAKE_ANGLE)

        if self.wrist.at_setpoint():
            self.next_state("intaking")

    @state()
    def raising_to_L3(self):
        if self.algae_manipulator_component.has_algae():
            self.done()
            return

        self.wrist.tilt_to(self.L3_INTAKE_ANGLE)

        if self.wrist.at_setpoint():
            self.next_state("intaking")

    @state
    def intaking(self):
        if self.algae_manipulator_component.has_algae():
            self.done()
            return

        self.algae_manipulator_component.intake()

    def intake_L2(self) -> None:
        self.engage("raising_to_L2")

    def intake_L3(self) -> None:
        self.engage("raising_to_L3")

    def done(self) -> None:
        super().done()
        self.wrist.go_to_neutral()
