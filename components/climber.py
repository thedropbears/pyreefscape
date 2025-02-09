from magicbot import tunable
from rev import SparkMax, SparkMaxConfig

from ids import SparkId


class ClimberComponent:
    target_speed = 0.0
    winch_voltage = tunable(12.0)

    def __init__(self) -> None:
        self.motor = SparkMax(SparkId.CLIMBER, SparkMax.MotorType.kBrushless)
        self.deployed = False  # TODO have some way to detect if this happened
        self.retracted = False  # TODO have some way to detect if this happened

        motor_config = SparkMaxConfig()
        motor_config.inverted(True)
        motor_config.setIdleMode(SparkMaxConfig.IdleMode.kBrake)
        motor_config.openLoopRampRate(1.5)

        self.motor.configure(
            motor_config,
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )

    def deploy(self) -> None:
        self.target_speed = self.winch_voltage

    def retract(self) -> None:
        self.target_speed = -self.winch_voltage

    def is_deployed(self) -> bool:
        return self.deployed

    def is_retracted(self) -> bool:
        return self.retracted

    def elevation(self) -> float:
        return 0.0
        # current place holder

    def execute(self) -> None:
        if (
            self.is_deployed()
            and self.target_speed > 0
            or self.is_retracted()
            and self.target_speed < 0
        ):
            self.target_speed = 0.0
            # stop motor if we're already there

        self.motor.setVoltage(self.target_speed)
        self.target_speed = 0.0
