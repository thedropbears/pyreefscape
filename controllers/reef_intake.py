import math

import wpilib
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
        nearest_reef_tag_id = (game.nearest_reef_tag(self.chassis.get_pose()))[0]
        return game.is_L3(nearest_reef_tag_id)

    @state(first=True, must_finish=True)
    def intaking(self, initial_call: bool):
        if initial_call:
            current_pose = self.chassis.get_pose()
            current_pos = current_pose.translation()

            red_distance = game.RED_REEF_POS.distance(current_pos)
            blue_distance = game.BLUE_REEF_POS.distance(current_pos)

            if (
                red_distance < self.ENGAGE_DISTANCE
                or blue_distance < self.ENGAGE_DISTANCE
            ):
                self.status_lights.too_close_to_reef()
                self.done()
                return

        if self.algae_manipulator_component.has_algae():
            self.algae_manipulator_component.HOLDING_ALGAE = True
            self.next_state("safing")

        nearest_tag_pose = (game.nearest_reef_tag(self.chassis.get_pose()))[1]
        self.rotation_lock = nearest_tag_pose.rotation()
        if not wpilib.DriverStation.isAutonomous():
            self.chassis.snap_to_heading(self.rotation_lock.radians())

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
            self.chassis.stop_snapping()

        robot_pose = self.chassis.get_pose()

        distance = self.origin_robot_pose.translation().distance(
            robot_pose.translation()
        )

        if distance >= self.RETREAT_DISTANCE:
            self.next_state("feeling")

    @state(must_finish=True)
    def feeling(self, initial_call):
        if not self.feeler.is_executing:
            self.done()

    def done(self) -> None:
        super().done()
        self.wrist.go_to_neutral()
        self.chassis.stop_snapping()
