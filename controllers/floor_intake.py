import math

from magicbot import StateMachine, state, tunable

from components.algae_manipulator import AlgaeManipulatorComponent
from components.intake import IntakeComponent
from components.wrist import WristComponent
from controllers.feeler import Feeler


class FloorIntake(StateMachine):
    algae_manipulator_component: AlgaeManipulatorComponent
    wrist: WristComponent
    intake_component: IntakeComponent
    feeler: Feeler

    HANDOFF_POSITION = tunable(math.radians(-112.0))
    deployed = False

    def __init__(self):
        pass

    def intake(self) -> None:
        self.engage()

    @state(first=True, must_finish=True)
    def intaking(self, initial_call: bool):
        if self.algae_manipulator_component.has_algae():
            self.next_state("retracting")
            return

        self.intake_component.intake()

        if (
            self.intake_component.deploy_motor_current() > 100
        ):  # Current value may be inaccurate
            self.deployed = True

        if not self.deployed:
            self.intake_component.deploy()

        if initial_call:
            self.wrist.tilt_to(self.HANDOFF_POSITION)

        self.algae_manipulator_component.intake()

    @state(must_finish=True)
    def retracting(self):
        if self.deployed:
            self.intake_component.retract()
        else:
            self.next_state("feeling")

        if (
            self.intake_component.deploy_motor_current() > 100
        ):  # Current value may be inaccurate
            self.deployed = False

    @state(must_finish=True)
    def feeling(self, initial_call):
        if initial_call:
            self.feeler.feel()
        elif not self.feeler.is_executing:
            self.done()

    def done(self) -> None:
        super().done()
        self.wrist.go_to_neutral()
