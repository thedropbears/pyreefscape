from magicbot import tunable, will_reset_to
from rev import SparkMax, SparkMaxConfig

from ids import SparkId


class CoralPlacerComponent:
    voltage_set_point = will_reset_to(0.0)
    deposit_voltage = tunable(7.0)

    def __init__(self):
        self.motor = SparkMax(SparkId.CORAL_PLACER, SparkMax.MotorType.kBrushless)

        motor_config = SparkMaxConfig()
        motor_config.inverted(False)  # TODO Change if needed
        motor_config.setIdleMode(SparkMax.IdleMode.kCoast)

        self.motor.configure(
            motor_config,
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )

    def place(self):
        self.voltage_set_point = self.deposit_voltage

    def execute(self):
        # set motor
        self.motor.setVoltage(self.voltage_set_point)
