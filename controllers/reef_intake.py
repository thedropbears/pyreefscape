import math

import wpilib
from magicbot import StateMachine, feedback, state, tunable

from components.chassis import ChassisComponent
from components.injector import InjectorComponent
from components.led_component import LightStrip
from components.shooter import ShooterComponent
from components.wrist import WristComponent
from utilities import game


class ReefIntake(StateMachine):
    shooter_component: ShooterComponent
    injector_component: InjectorComponent
    wrist: WristComponent
    chassis: ChassisComponent
    status_lights: LightStrip

    L2_INTAKE_ANGLE = tunable(-55.0)
    L3_INTAKE_ANGLE = tunable(-7.0)

    RETREAT_DISTANCE = tunable(0.3)  # metres
    ENGAGE_DISTANCE = tunable(1.5)  # metres

    def __init__(self):
        self.last_l3 = False
        self.holding_coral = False

    def intake(self) -> None:
        self.engage()

    @feedback
    def is_L3(self) -> bool:
        nearest_reef_tag_id = (game.nearest_reef_tag(self.chassis.get_pose()))[0]
        return game.is_L3(nearest_reef_tag_id)

    @state(first=True, must_finish=True)
    def intaking(self, initial_call: bool):
        current_pose = self.chassis.get_pose()
        if initial_call:
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

            if self.injector_component.has_algae():
                self.done()
                return

        if self.injector_component.has_algae():
            self.next_state(self.safing)
            return

        nearest_tag_pose = (game.nearest_reef_tag(current_pose))[1]
        self.rotation_lock = nearest_tag_pose.rotation()
        if not wpilib.DriverStation.isAutonomous():
            self.chassis.snap_to_heading(self.rotation_lock.radians())
        tag_to_robot = current_pose.relativeTo(nearest_tag_pose)
        offset = tag_to_robot.translation().Y()
        self.status_lights.reef_offset(offset)

        current_is_L3 = self.is_L3()

        if current_is_L3:
            self.wrist.tilt_to(math.radians(self.L3_INTAKE_ANGLE))
        else:
            self.wrist.tilt_to(math.radians(self.L2_INTAKE_ANGLE))
        self.last_l3 = current_is_L3

        if not self.holding_coral:
            self.shooter_component.intake()
        self.injector_component.intake()

    @state(must_finish=True)
    def safing(self, initial_call: bool):
        if initial_call:
            self.origin_robot_pose = self.chassis.get_pose()
            self.chassis.stop_snapping()

        robot_pose = self.chassis.get_pose()

        distance = self.origin_robot_pose.translation().distance(
            robot_pose.translation()
        )

        if not wpilib.DriverStation.isAutonomous() and distance < self.RETREAT_DISTANCE:
            self.chassis.limit_to_positive_longitudinal_velocity()

        if distance >= self.RETREAT_DISTANCE:
            self.done()

    def done(self) -> None:
        super().done()
        self.wrist.go_to_neutral()
        self.chassis.stop_snapping()
        self.holding_coral = False
