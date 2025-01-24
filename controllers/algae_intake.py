from magicbot import StateMachine, state, tunable

from components.algae_manipulator import AlgaeManipulatorComponent
from components.wrist import WristComponent


class AlgaeIntake(StateMachine):
    algae_manipulator_component: AlgaeManipulatorComponent
    wrist: WristComponent

    intake_L2_angle = tunable(30.0)
    intake_L3_angle = tunable(60.0)

    def __init__(self):
        pass

    @state(first=True)
    def tilting_to_L3(self):
        if self.algae_manipulator_component.has_algae():
            self.done()
            return

        self.wrist.tilt_to(self.intake_L3_angle)

        if self.wrist.at_desired_angle():
            self.next_state("intaking")

    @state()
    def tilting_to_L2(self):
        if self.algae_manipulator_component.has_algae():
            self.done()
            return

        self.wrist.tilt_to(self.intake_L2_angle)

        if self.wrist.at_desired_angle():
            self.next_state("intaking")

    @state()
    def intaking(self):
        if self.algae_manipulator_component.has_algae():
            self.done()
            return

        self.algae_manipulator_component.intake()

    def intake_L2(self):
        self.engage(initial_state="tilting_to_L2")

    def intake_L3(self):
        self.engage(initial_state="tilting_to_L3")
