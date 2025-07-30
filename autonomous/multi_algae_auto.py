from autonomous.auto_base import AutoBase


class CentreAuto(AutoBase):
    MODE_NAME = "Centre G>GH>IJ>EF"

    def __init__(self):
        super().__init__(
            [
                "StartToAlgaeGH",  # Seems good
                "AlgaeGHToShoot",  # Slight wiggle, rotates while moving but seems not major issue
                "ShootToAlgaeIJ",  # Goes outwards and takes a wide path while spinning
                "AlgaeIJToShoot",  # Spins right at the end of its path and moves fastest there
                "ShootToAlgaeEF",  # Slow long path, spins whole way, could perhaps start slow and spin fast, then speed up, slow down for the sharp corner
                "AlgaeEFToShoot",  # Funny backward spin at beginning, could perhaps spin the whole way while nearly stationary at end of sharp bend, then move to translational goal
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


class RefinedQuickCentre(AutoBase):
    MODE_NAME = "FAST REFINED Quick Centre GH>IJ>KL"

    def __init__(self):
        super().__init__(
            [
                "StartToAlgaeGH",
                "RefinedAlgaeGHToShootGH",
                "RefinedShootGHToAlgaeIJ",
                "RefinedAlgaeIJToShootIJ",
                "RefinedShootIJToAlgaeKL",
                "RefinedAlgaeKLToShootKL",
            ]
        )


class RefinedQuickCentreSlow(AutoBase):
    MODE_NAME = "SLOW(ed) REFINED Quick Centre GH>IJ>KL"

    def __init__(self):
        super().__init__(
            [
                "StartToAlgaeGH",
                "SlowedAlgaeGHToShootGH",
                "SlowedShootGHToAlgaeIJ",
                "RefinedAlgaeIJToShootIJ",
                "RefinedShootIJToAlgaeKL",
                "RefinedAlgaeKLToShootKL",
            ]
        )
