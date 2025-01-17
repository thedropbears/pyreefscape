from magicbot import tunable, will_reset_to
from rev import SparkMax, SparkMaxConfig

from ids import SparkId


class ClimberComponent:
    voltage_set_point = will_reset_to(0.0)
    winch_voltage = tunable(12.0)

    def __init__(self):
        self.motor = SparkMax(SparkId.CLIMBER, SparkMax.MotorType.kBrushless)

        motor_config = SparkMaxConfig()
        motor_config.inverted(False)  # TODO Change if needed
        motor_config.setIdleMode(SparkMaxConfig.IdleMode.kBrake)

        self.motor.configure(
            motor_config,
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )

    def pull_rope(self):
        self.voltage_set_point = self.winch_voltage

    def loosen_rope(self):
        self.voltage_set_point = -self.winch_voltage

    def execute(self):
        self.motor.setVoltage(self.voltage_set_point)
        self.voltage_set_point = 0.0
