from rev import MotorType, SparkMax

from ids import SparkId


class CoralShooter:
    def __init__(self):
        self.motor = SparkMax(SparkId.CORAL_SHOOTER, MotorType.kBrushless)
        self.motor.setInverted(False)  # Change if needed-----------------------------
        self.voltage_set_point = 0.0

    def deploy(self):
        self.voltage_set_point = 7.0

    def execute(self):
        # set motor
        self.motor.setVoltage(self.voltage_set_point)

        # reset setpoint
        self.voltage_set_point = 0.0
