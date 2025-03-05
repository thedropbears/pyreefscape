from autonomous.auto_base import AutoBase


class CentreAuto(AutoBase):
    MODE_NAME = "Centre G>EF>GH>IJ"

    def __init__(self):
        super().__init__(
            [
                "StartToAlgaeEF",
                "AlgaeEFToShoot",
                "ShootToAlgaeGH",
                "AlgaeGHToShoot",
                "ShootToAlgaeIJ",
                "AlgaeIJToShoot",
            ]
        )


class AllianceSideAuto(AutoBase):
    MODE_NAME = "Alliance Side L>KL>AB"

    def __init__(self):
        super().__init__(
            [
                "StartToAlgaeKL",
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
                "StartToAlgaeCD",
                "AlgaeCDToShoot",
                "ShootToAlgaeKL",
                "AlgaeKLToShoot",
            ]
        )
