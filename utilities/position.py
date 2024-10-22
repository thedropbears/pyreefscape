import math
from wpimath.geometry import Pose2d
from utilities.game import field_flip_pose2d


class TeamPoses:
    RED_TEST_POSE = Pose2d(15.1, 5.5, math.pi)
    BLUE_TEST_POSE = field_flip_pose2d(RED_TEST_POSE)
