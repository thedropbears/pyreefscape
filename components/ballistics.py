import math
from dataclasses import dataclass
from typing import ClassVar

import numpy
import wpiutil.wpistruct
from magicbot import feedback
from wpimath.geometry import (
    Rotation2d,
    Translation2d,
)

from components.algae_manipulator import AlgaeManipulatorComponent
from components.chassis import ChassisComponent
from components.led_component import LightStrip
from utilities.game import FIELD_LENGTH, FIELD_WIDTH, is_red


@wpiutil.wpistruct.make_wpistruct
@dataclass
class BallisticsSolution:
    WPIStruct: ClassVar

    speed: float
    inclination: float


class BallisticsComponent:
    chassis: ChassisComponent
    status_lights: LightStrip
    algae_manipulator_component: AlgaeManipulatorComponent

    x_max_offset_range = 4.0  # in meters
    x_min_offset_range = 1.0

    barge_red_mid_end_point = Translation2d(FIELD_LENGTH / 2, FIELD_WIDTH / 2)
    barge_blue_mid_end_point = Translation2d(FIELD_LENGTH / 2, FIELD_WIDTH / 2)

    FLYWHEEL_DISTANCE_LOOKUP = (x_min_offset_range, x_max_offset_range)
    FLYWHEEL_SPEED_LOOKUP = (60, 80)
    FLYWHEEL_ANGLE_LOOKUP = (math.radians(-10), math.radians(-40))

    robot_to_shooter = Rotation2d.fromDegrees(180)

    def __init__(self) -> None:
        pass

    @feedback
    def is_in_range(self) -> bool:
        range = self.range()
        return self.x_min_offset_range < range < self.x_max_offset_range

    @feedback
    def range(self) -> float:
        robot_pose = self.chassis.get_pose()
        if is_red():
            if robot_pose.translation().Y() > FIELD_WIDTH / 2:
                return abs(
                    self.barge_red_mid_end_point.X() - robot_pose.translation().X()
                )
            else:
                return (robot_pose.translation() - self.barge_red_mid_end_point).norm()
        else:
            if robot_pose.translation().Y() < FIELD_WIDTH / 2:
                return abs(
                    self.barge_blue_mid_end_point.X() - robot_pose.translation().X()
                )
            else:
                return (robot_pose.translation() - self.barge_blue_mid_end_point).norm()

    @feedback
    def corrected_range(self) -> float:
        distance = math.inf
        if not self.is_aligned():
            return distance
        robot_pose = self.chassis.get_pose()
        barge_X = FIELD_LENGTH / 2

        robot_to_barge_X_offset = barge_X - robot_pose.translation().X()
        distance = (
            robot_to_barge_X_offset
            / (robot_pose.rotation() + self.robot_to_shooter).cos()
        )
        return distance

    @feedback
    def is_aligned(self) -> bool:
        robot_pose = self.chassis.get_pose()

        if is_red():
            robot_to_top_point = self.barge_red_mid_end_point - robot_pose.translation()
            robot_to_bottom_point = (
                Translation2d(FIELD_LENGTH / 2, 0.0) - robot_pose.translation()
            )
        else:
            robot_to_top_point = (
                Translation2d(FIELD_LENGTH / 2, FIELD_WIDTH) - robot_pose.translation()
            )
            robot_to_bottom_point = (
                self.barge_blue_mid_end_point - robot_pose.translation()
            )

        relative_bearing_to_bottom_point = robot_to_bottom_point.angle() - (
            robot_pose.rotation() + self.robot_to_shooter
        )
        relative_bearing_to_top_point = robot_to_top_point.angle() - (
            robot_pose.rotation() + self.robot_to_shooter
        )
        return (
            relative_bearing_to_top_point.radians()
            * relative_bearing_to_bottom_point.radians()
            < 0.0  # to check if they have opposite signs
            and abs(relative_bearing_to_top_point.degrees()) < 90.0
            and abs(relative_bearing_to_bottom_point.degrees()) < 90.0
        )

    @feedback
    def current_solution(self) -> BallisticsSolution:
        return BallisticsSolution(
            float(
                numpy.interp(
                    self.corrected_range(),
                    self.FLYWHEEL_DISTANCE_LOOKUP,
                    self.FLYWHEEL_SPEED_LOOKUP,
                )
            ),
            float(
                numpy.interp(
                    self.corrected_range(),
                    self.FLYWHEEL_DISTANCE_LOOKUP,
                    self.FLYWHEEL_ANGLE_LOOKUP,
                )
            ),
        )

    def execute(self) -> None:
        if self.algae_manipulator_component.has_algae():
            if self.is_in_range():
                if self.is_aligned():
                    self.status_lights.facing_in_range()
                else:
                    self.status_lights.not_facing_in_range()
            else:
                self.status_lights.not_in_range()
