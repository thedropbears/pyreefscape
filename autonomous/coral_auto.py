from autonomous.auto_base import AutoBase


class CoralAuto(AutoBase):
    MODE_NAME = "Coral Shooter"

    def __init__(self):
        super().__init__(["StartToBranchH", "BranchHToAlgaeGH", "AlgaeGHToShoot"])
