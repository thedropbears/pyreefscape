import math
import typing

import robotpy_apriltag
import wpilib
from wpimath.geometry import (
    Pose2d,
    Pose3d,
    Rotation2d,
    Translation2d,
    Translation3d,
)

apriltag_layout = robotpy_apriltag.loadAprilTagLayoutField(
    robotpy_apriltag.AprilTagField.k2024Crescendo
)

TagId = typing.Literal[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16]

get_fiducial_pose = typing.cast(
    typing.Callable[[TagId], Pose3d],
    apriltag_layout.getTagPose,
)

RED_SPEAKER_POSE = get_fiducial_pose(4)
BLUE_SPEAKER_POSE = get_fiducial_pose(7)

FIELD_WIDTH = 8.0161
FIELD_LENGTH = RED_SPEAKER_POSE.x + BLUE_SPEAKER_POSE.x

# Minimum height of the overhanging speaker hood as obtained from a field Onshape model.
SPEAKER_HOOD_HEIGHT = 2.104883
SPEAKER_HOOD_WIDTH = 1.05
SPEAKER_HOOD_DEPTH = 0.456499

BLUE_SPEAKER_TARGET_POSITION = BLUE_SPEAKER_POSE.translation() + Translation3d(
    SPEAKER_HOOD_DEPTH, 0, 0
)

RED_SPEAKER_TARGET_POSITION = RED_SPEAKER_POSE.translation() - Translation3d(
    SPEAKER_HOOD_DEPTH, 0, 0
)

NOTE_DIAMETER = 0.0254 * 14


def field_flip_pose2d(p: Pose2d):
    return Pose2d(
        field_flip_translation2d(p.translation()),
        field_flip_rotation2d(p.rotation()),
    )


def field_flip_translation3d(t: Translation3d):
    return Translation3d(FIELD_LENGTH - t.x, t.y, t.z)


def field_flip_rotation2d(r: Rotation2d):
    return Rotation2d(-r.cos(), r.sin())


def field_flip_angle(r: float):
    return math.atan2(math.sin(math.pi - r), math.cos(math.pi - r))


def field_flip_translation2d(t: Translation2d):
    return Translation2d(FIELD_LENGTH - t.x, t.y)


# This will default to the blue alliance if a proper link to the driver station has not yet been established
def is_red() -> bool:
    return wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed


def get_goal_speaker_position() -> Translation3d:
    if is_red():
        return RED_SPEAKER_POSE.translation()

    return BLUE_SPEAKER_POSE.translation()


def translation_to_goal(position: Translation2d) -> Translation2d:
    return get_goal_speaker_position().toTranslation2d() - position
