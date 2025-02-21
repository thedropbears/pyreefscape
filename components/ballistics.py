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

from components.chassis import ChassisComponent
from components.injector import InjectorComponent
from components.led_component import LightStrip
from components.shooter import ShooterComponent
from utilities.game import FIELD_LENGTH, FIELD_WIDTH, is_red


@wpiutil.wpistruct.make_wpistruct
@dataclass
class BallisticsSolution:
    WPIStruct: ClassVar

    top_speed: float
    bottom_speed: float
    inclination: float


class BallisticsComponent:
    chassis: ChassisComponent
    status_lights: LightStrip
    shooter_component: ShooterComponent
    injector_component: InjectorComponent

    barge_red_mid_end_point = Translation2d(FIELD_LENGTH / 2, FIELD_WIDTH / 2)
    barge_blue_mid_end_point = Translation2d(FIELD_LENGTH / 2, FIELD_WIDTH / 2)

    # TODO Retune for  >= 4m when the space allows
    FLYWHEEL_DISTANCE_LOOKUP = (1.5, 2.0, 3.0)  # , 4.0
    # Keys of the lookup dictionaries are ball diameters in inches
    # Tuples are values corresponding to the distances above
    # fmt: off
    FLYWHEEL_TOP_SPEED_LOOKUP = {
        16.0: (30, 32.5, 38.0),  # , 90
        # 16.5: (60, 80),
        17.0: (33, 36, 39.0), # , 50
    }
    # Currently we use the same speed top and bottom, but this could be seperate
    FLYWHEEL_BOTTOM_SPEED_LOOKUP = FLYWHEEL_TOP_SPEED_LOOKUP
    FLYWHEEL_ANGLE_LOOKUP = {
        16.0: (math.radians(-10), math.radians(-12), math.radians(-19)), # , math.radians(-25)
        # 16.5: (math.radians(-15), math.radians(-20)),
        17.0: (math.radians(-10), math.radians(-12), math.radians(-19)), # , math.radians(-25)
    }
    # fmt: on
    BALL_SIZES = list(FLYWHEEL_ANGLE_LOOKUP.keys())
    robot_to_shooter = Rotation2d.fromDegrees(180)

    def __init__(self) -> None:
        pass

    @feedback
    def is_in_range(self) -> bool:
        range = self.range()
        return (
            self.FLYWHEEL_DISTANCE_LOOKUP[0] < range < self.FLYWHEEL_DISTANCE_LOOKUP[-1]
        )

    @feedback
    def range(self) -> float:
        robot_pose = self.chassis.get_pose()
        robot_pos = robot_pose.translation()
        if is_red():
            if robot_pos.Y() < FIELD_WIDTH / 2:
                return abs(self.barge_red_mid_end_point.X() - robot_pos.X())
            else:
                return robot_pos.distance(self.barge_red_mid_end_point)
        else:
            if robot_pos.Y() > FIELD_WIDTH / 2:
                return abs(self.barge_blue_mid_end_point.X() - robot_pos.X())
            else:
                return robot_pos.distance(self.barge_blue_mid_end_point)

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
        d = self.corrected_range()
        # Get the interpolated values for this range and all ball sizes
        top_speeds = [
            float(numpy.interp(d, self.FLYWHEEL_DISTANCE_LOOKUP, speeds))
            for speeds in self.FLYWHEEL_TOP_SPEED_LOOKUP.values()
        ]

        bottom_speeds = [
            float(numpy.interp(d, self.FLYWHEEL_DISTANCE_LOOKUP, speeds))
            for speeds in self.FLYWHEEL_BOTTOM_SPEED_LOOKUP.values()
        ]
        inclinations = [
            float(numpy.interp(d, self.FLYWHEEL_DISTANCE_LOOKUP, angles))
            for angles in self.FLYWHEEL_ANGLE_LOOKUP.values()
        ]
        return BallisticsSolution(
            float(
                numpy.interp(
                    self.shooter_component.algae_size,
                    self.BALL_SIZES,
                    top_speeds,
                )
            ),
            float(
                numpy.interp(
                    self.shooter_component.algae_size,
                    self.BALL_SIZES,
                    bottom_speeds,
                )
            ),
            float(
                numpy.interp(
                    self.shooter_component.algae_size,
                    self.BALL_SIZES,
                    inclinations,
                )
            ),
        )

    def execute(self) -> None:
        if self.injector_component.has_algae():
            if self.is_in_range():
                if self.is_aligned():
                    self.status_lights.facing_in_range()
                else:
                    self.status_lights.not_facing_in_range()
            else:
                self.status_lights.not_in_range()
