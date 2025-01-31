from magicbot import tunable
from wpimath.geometry import (
    Translation2d,
)

from components.chassis import ChassisComponent


class BalisticsComponent:
    chassis: ChassisComponent

    def __init__(self):
        self.max_range = tunable(2.0)

    def in_range_of_barge(self) -> bool:
        robot_pose = self.chassis.get_pose()
        barge_pose = Translation2d(
            6, 3.7211
        )  # TODO: change to proper numbers for the center of the barge
        robot_to_barge = robot_pose.translation() - barge_pose
        distance = robot_to_barge.norm()
        return distance < self.max_range
