import math

from magicbot import StateMachine, feedback, state, tunable

from components.algae_manipulator import AlgaeManipulatorComponent
from components.chassis import ChassisComponent
from components.wrist import WristComponent
from utilities import game


class AlgaeIntake(StateMachine):
    algae_manipulator_component: AlgaeManipulatorComponent
    wrist: WristComponent
    chassis: ChassisComponent

    L2_INTAKE_ANGLE = tunable(math.radians(-50.0))
    L3_INTAKE_ANGLE = tunable(math.radians(-10.0))

    def __init__(self):
        self.last_l3 = False

    def intake(self) -> None:
        self.engage()

    @state(first=True, must_finish=True)
    def intaking(self):
        if self.algae_manipulator_component.has_algae():
            self.done()
            return

        current_is_L3 = self.is_L3()
    
        if self.last_l3 != current_is_L3:
            if current_is_L3:
                self.wrist.tilt_to(self.L3_INTAKE_ANGLE)
            else:
                self.wrist.tilt_to(self.L2_INTAKE_ANGLE)
            self.last_l3 = current_is_L3

        self.algae_manipulator_component.intake()

    @feedback
    def is_L3(self) -> bool:
        return game.is_L3(game.nearest_reef_tag(self.chassis.get_pose()))

    def done(self) -> None:
        super().done()
        self.wrist.go_to_neutral()
