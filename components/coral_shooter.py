from magicbot import tunable, will_reset_to
from rev import SparkMax

from ids import SparkId


class CoralShooterComponent:
    voltage_set_point = will_reset_to(0.0)
    deposit_voltage = tunable(7.0)

    def __init__(self):
        self.motor = SparkMax(SparkId.CORAL_SHOOTER, SparkMax.MotorType.kBrushless)
        # TODO Change if needed
        self.motor.setInverted(False)

    def deploy(self):
        self.voltage_set_point = self.deposit_voltage

    def execute(self):
        # set motor
        self.motor.setVoltage(self.voltage_set_point)
