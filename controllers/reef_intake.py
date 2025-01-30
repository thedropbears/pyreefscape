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

    RETREAT_DISTANCE = tunable(0.6)

    FEELER_START_ANGLE = tunable(90)
    FEELER_START_OFFEST = tunable(17)

    current_feeler_angle = 0.0

    FEELER_START_OFFSET = tunable(
        math.radians(17)
    )  # TODO: change to magic number we found from testing

    def __init__(self):
        self.last_l3 = False

    def intake(self) -> None:
        self.engage()

    @state(first=True, must_finish=True)
    def intaking(self, initial_call: bool):
        if self.algae_manipulator_component.has_algae():
            self.touch_the_algae(initial_call)
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

        if distance.norm() >= self.RETREAT_DISTANCE:
            self.done()

    def touch_the_algae(self, initial_call: bool):
        if initial_call:
            self.current_feeler_algae = self.FEELER_START_ANGLE + self.FEELER_START_OFFEST
            self.algae_manipulator_component.algae_size = 0.0

        if self.current_feeler_angle >= 160:
            self.current_feeler_angle = 0.0
            self.algae_manipulator_component.algae_size = 0.0

        self.algae_manipulator_component.set_feeler(self.current_feeler_angle, False)

        if self.algae_manipulator_component.feeler_touching_algae():
            self.algae_manipulator_component.algae_size = self.current_feeler_angle
            self.done()
            return

        self.current_feeler_angle += 0.69 # degrees per cycle that the feeler will move 

    @feedback
    def is_L3(self) -> bool:
        return game.is_L3(game.nearest_reef_tag(self.chassis.get_pose()))

    def done(self) -> None:
        super().done()
        self.wrist.go_to_neutral()
        self.current_feeler_angle = self.FEELER_START_ANGLE
        self.algae_manipulator_component.set_feeler(self.current_feeler_angle, False)
