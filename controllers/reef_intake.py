import math

from magicbot import StateMachine, feedback, state, tunable

from components.algae_manipulator import AlgaeManipulatorComponent
from components.chassis import ChassisComponent
from components.wrist import WristComponent
from utilities import game


class ReefIntake(StateMachine):
    algae_manipulator_component: AlgaeManipulatorComponent
    wrist: WristComponent
    chassis: ChassisComponent

    L2_INTAKE_ANGLE = tunable(math.radians(-40.0))
    L3_INTAKE_ANGLE = tunable(math.radians(-10.0))

    UNSAFE_DISTANCE = tunable(2.0)

    def __init__(self):
        self.last_l3 = False

    def intake(self) -> None:
        self.engage()

    @state(first=True, must_finish=True)
    def intaking(self, initial_call: bool):
        if self.algae_manipulator_component.has_algae() and initial_call:
            self.done()
        elif self.algae_manipulator_component.has_algae():
            self.next_state("safing")

        current_is_L3 = self.is_L3()

        if self.last_l3 != current_is_L3 or initial_call:
            if current_is_L3:
                self.wrist.tilt_to(self.L3_INTAKE_ANGLE)
            else:
                self.wrist.tilt_to(self.L2_INTAKE_ANGLE)
            self.last_l3 = current_is_L3

        self.algae_manipulator_component.intake()

    @state(must_finish=True)
    def safing(self, initial_call: bool):
        if initial_call:
            self.origin_robot_pose = self.chassis.get_pose()
        robot_pose = self.chassis.get_pose()

        distance = self.origin_robot_pose.translation() - robot_pose.translation()

        if distance.norm() >= self.UNSAFE_DISTANCE:
            self.done()

    @feedback
    def is_L3(self) -> bool:
        return game.is_L3(game.nearest_reef_tag(self.chassis.get_pose()))

    def done(self) -> None:
        super().done()
        self.wrist.go_to_neutral()
