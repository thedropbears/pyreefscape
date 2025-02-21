from autonomous.auto_base import AutoBase


class CentreAuto(AutoBase):
    MODE_NAME = "Centre F>EF>GH>IJ"

    def __init__(self):
        super().__init__(
            [
                "StartToBranchF",
                "BranchFToAlgaeEF",
                "AlgaeEFToShoot",
                "ShootToAlgaeGH",
                "AlgaeGHToShoot",
                "ShootToAlgaeIJ",
                "AlgaeIJToShoot",
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
