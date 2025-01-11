from magicbot import will_reset_to
from rev import SparkMax

from ids import SparkId


class CoralShooterComponent:

    voltage_set_point = will_reset_to(0.0)

    def __init__(self):
        self.motor = SparkMax(SparkId.CORAL_SHOOTER, SparkMax.MotorType.kBrushless)
        self.motor.setInverted(False)  # Change if needed-----------------------------

    def deploy(self):
        self.voltage_set_point = 7.0

    def execute(self):
        # set motor
        self.motor.setVoltage(self.voltage_set_point)
