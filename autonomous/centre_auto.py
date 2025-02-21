from autonomous.auto_base import AutoBase


class CentreAuto(AutoBase):
    MODE_NAME = "Centre (F>EF>GH>IJ)"

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
