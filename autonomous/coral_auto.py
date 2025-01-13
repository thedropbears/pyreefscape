import wpilib

from autonomous.coral_auto_base import CoralAutoBase


class CoralAuto(CoralAutoBase):
    MODE_NAME = "Coral Shooter"

    def __init__(self):
        super().__init__(f"{wpilib.getDeployDirectory()}/traj/Coral_blue")
