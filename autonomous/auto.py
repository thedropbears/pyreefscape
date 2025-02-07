from autonomous.algae_only_auto_base import AlgaeOnlyAutoBase
from autonomous.base import AutoBase
from autonomous.test_base import TestAutoBase


class Autonomous(AutoBase):
    MODE_NAME = "Algae Shooter"

    def __init__(self):
        super().__init__("StartToAlgaeGH")


class LucienAuto(AlgaeOnlyAutoBase):
    MODE_NAME = "AlgaeOnly"

    def __init__(self):
        super().__init__("AlgaeOnly")


class TestAuto(TestAutoBase):
    MODE_NAME = "Testing"

    def __init__(self):
        super().__init__("TestRoutine")
