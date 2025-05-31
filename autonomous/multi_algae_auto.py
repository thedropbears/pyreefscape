from autonomous.auto_base import AutoBase, AutoBaseNoBalls


class CentreAuto(AutoBase):
    MODE_NAME = "Centre G>GH>IJ>EF"

    def __init__(self):
        super().__init__(
            [
                "StartToAlgaeGH",
                "AlgaeGHToShoot",
                "ShootToAlgaeIJ",
                "AlgaeIJToShoot",
                "ShootToAlgaeEF",
                "AlgaeEFToShoot",
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


class QuickPath(AutoBase):
    MODE_NAME = "NEW Quick Centre Alliance GH>IJ>KL"

    def __init__(self):
        super().__init__(
            [
                "StartToAlgaeGH",
                "AlgaeGHToShootGH",
                "ShootGHToAlgaeIJ",
                "AlgaeIJToShootIJ",
                "ShootIJToAlgaeKL",
                "AlgaeKLToShootKL",
            ]
        )


class LineRotateTest(AutoBaseNoBalls):
    MODE_NAME = "TEST LINE ROTATION"

    def __init__(self) -> None:
        super().__init__(
            [
                "LineRotateTest",
            ]
        )


class LineRotateJiggleReductionTest(AutoBaseNoBalls):
    MODE_NAME = "TEST JIGGLE REDUCTION line rotation"

    def __init__(self) -> None:
        super().__init__(
            [
                "LineLessJiggleTest",
            ]
        )
