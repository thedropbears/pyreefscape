from autonomous.auto_base import AutoBase


class CentreAuto(AutoBase):
    MODE_NAME = "Centre F>GH>IJ>EF"

    def __init__(self):
        super().__init__(
            [
                "StartToBranchG",
                "BranchGToAlgaeGH",
                "AlgaeGHToShoot",
                "ShootToAlgaeIJ",
                "AlgaeIJToShoot",
                "ShootToAlgaeEF",
                "AlgaeEFToShoot",
            ]
        )


class AllianceSideAuto(AutoBase):
    MODE_NAME = "Alliance Start L>KL>AB"

    def __init__(self):
        super().__init__(
            [
                "StartToBranchL",
                "BranchLToAlgaeKL",
                "AlgaeKLToShoot",
                "ShootToAlgaeAB",
                "AlgaeABToShoot",
            ]
        )


class OppositionSideAuto(AutoBase):
    MODE_NAME = "Opposition Side C>CD>KL"

    def __init__(self):
        super().__init__(
            [
                "StartToBranchC",
                "BranchCToAlgaeCD",
                "AlgaeCDToShoot",
                "ShootToAlgaeKL",
                "AlgaeKLToShoot",
            ]
        )
