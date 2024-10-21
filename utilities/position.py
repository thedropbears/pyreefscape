import math

from wpimath.geometry import Rotation2d, Translation2d, Pose2d

from utilities.game import (
    RED_SPEAKER_POSE,
    BLUE_SPEAKER_POSE,
    field_flip_pose2d,
    field_flip_translation2d,
    field_flip_angle,
)


class Path:
    waypoints: list[Translation2d]
    final_heading: float
    face_target: bool

    def __init__(self, waypoints: list[Translation2d], face_target: bool):
        self.waypoints = waypoints
        self.face_target = face_target
        if face_target:
            last_waypoint = waypoints[-1]
            self.final_heading = (
                (
                    BLUE_SPEAKER_POSE.translation().toTranslation2d()
                    - field_flip_translation2d(last_waypoint)
                )
                .angle()
                .radians()
            )
            self.final_heading = field_flip_angle(self.final_heading) + math.pi
        else:
            self.final_heading = 0


stage_tolerance = 0.4


class NotePositions:
    # These are the 3 close notes, named for the nearest element
    amp = Translation2d(13.645, 7.00045)
    speaker = Translation2d(13.645, 5.55265)
    podium = Translation2d(13.645, 4.1057)

    # 1 is always the closest to the amp side
    Centre1 = Translation2d(8.2956, 7.4585)
    Centre2 = Translation2d(8.2956, 5.7821)
    Centre3 = Translation2d(8.2956, 4.1057)
    Centre4 = Translation2d(8.2956, 2.4293)
    Centre5 = Translation2d(8.2956, 0.75286)

    # The podium note is very close to the stage leg so we need different positions to avoid collisions
    # Directions are in the field coordinate system for red side
    podium_N = podium + Translation2d(stage_tolerance, 0)
    podium_NW = podium + Translation2d(
        stage_tolerance / math.sqrt(2), stage_tolerance / math.sqrt(2)
    )
    podium_NE = podium + Translation2d(
        stage_tolerance / math.sqrt(2), -stage_tolerance / math.sqrt(2)
    )


class ShootingPositions:
    close_straight = Translation2d(15, RED_SPEAKER_POSE.y)
    amp_speaker_bounce = Translation2d(
        14.7, (NotePositions.amp.y + NotePositions.speaker.y) / 2
    )
    source_side = Translation2d(15.556, 4.034)


class TeamPoses:
    RED_TEST_POSE = Pose2d(15.1, 5.5, math.pi)
    BLUE_TEST_POSE = field_flip_pose2d(RED_TEST_POSE)
    BLUE_PODIUM = Pose2d(Translation2d(2.992, 4.08455), Rotation2d(math.pi))
    RED_PODIUM = field_flip_pose2d(BLUE_PODIUM)
    RED_AMP_START_POSE = Pose2d(15.9, 6.7, math.pi)


def distance_between(intended_start_pose: Pose2d, current_pose: Pose2d) -> float:
    return (intended_start_pose.translation() - current_pose.translation()).norm()


class PathPositions:
    stage_transition_N = Translation2d(11.4, 4.5)
    stage_transition_S = Translation2d(11.4, 3.74)
    stage_transition_S_entry = Translation2d(13.0, 2.5)
    avoid_wall = Translation2d(10.80, 6.55)
    avoid_stage_S = Translation2d(11.66, 1.40)
    avoid_starting_faults = Translation2d(14.429, 2.946)
    enforce_pickup_angle = Translation2d(9.6, 1.6)
