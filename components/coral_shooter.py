from rev import SparkMax

from ids import SparkId


class CoralShooterComponent:
    def __init__(self):
        self.motor = SparkMax(SparkId.CORAL_SHOOTER, SparkMax.MotorType.kBrushless)
        self.motor.setInverted(False)  # Change if needed-----------------------------
        self.voltage_set_point = 0.0

    def deploy(self):
        self.voltage_set_point = 7.0

    def execute(self):
        # set motor
        self.motor.setVoltage(self.voltage_set_point)

        # reset setpoint
        self.voltage_set_point = 0.0
