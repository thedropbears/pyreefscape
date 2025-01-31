from autonomous.base import AutoBase


class Autonomous(AutoBase):
    MODE_NAME = "Algae Shooter"

    def __init__(self):
        super().__init__("StartToAlgaeJI")
