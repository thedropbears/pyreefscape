import math

from magicbot import StateMachine, feedback, state, tunable

from components.algae_manipulator import AlgaeManipulatorComponent
from components.chassis import ChassisComponent
from components.led_component import LightStrip
from components.wrist import WristComponent
from controllers.feeler import Feeler
from utilities import game


class ReefIntake(StateMachine):
    algae_manipulator_component: AlgaeManipulatorComponent
    wrist: WristComponent
    chassis: ChassisComponent
    feeler: Feeler
    status_lights: LightStrip

    L2_INTAKE_ANGLE = tunable(math.radians(-40.0))
    L3_INTAKE_ANGLE = tunable(math.radians(-10.0))

    RETREAT_DISTANCE = tunable(0.6)  # metres
    ENGAGE_DISTANCE = tunable(3.0)  # metres

    def __init__(self):
        self.last_l3 = False

    def intake(self) -> None:
        self.engage()

    @feedback
    def is_L3(self) -> bool:
        return game.is_L3(game.nearest_reef_tag(self.chassis.get_pose()))

    @state(first=True, must_finish=True)
    def intaking(self, initial_call: bool):
        if initial_call:
            current_pose = self.chassis.get_pose()

            red_distance = game.RED_REEF_POS - current_pose.translation()
            blue_distance = game.BLUE_REEF_POS - current_pose.translation()

            if (
                red_distance.norm() < self.ENGAGE_DISTANCE
                or blue_distance.norm() < self.ENGAGE_DISTANCE
            ):
                self.status_lights.too_close_to_reef()
                self.done()
                return
            else:
                self.status_lights.team_colour()  # temporary reset to team colour since it doesn't reset in this branch right now

        if self.algae_manipulator_component.has_algae():
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
            self.feeler.engage()

        robot_pose = self.chassis.get_pose()

        distance = self.origin_robot_pose.translation() - robot_pose.translation()

        if distance.norm() >= self.RETREAT_DISTANCE:
            self.next_state("feeling")

    @state(must_finish=True)
    def feeling(self, initial_call):
        if not self.feeler.is_executing:
            self.done()

    def done(self) -> None:
        super().done()
        self.wrist.go_to_neutral()
