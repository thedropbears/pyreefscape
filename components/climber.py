import math

from magicbot import feedback, tunable
from rev import SparkMax, SparkMaxConfig
from wpilib import DutyCycleEncoder

from ids import DioChannel, SparkId


class ClimberComponent:
    target_speed = 0.0
    winch_voltage = tunable(12.0)
    MIN_ANGLE = 40
    MAX_ANGLE = 90

    def __init__(self) -> None:
        self.encoder = DutyCycleEncoder(DioChannel.CLIMBER_ENCODER, math.tau, 0)
        self.encoder.setInverted(True)
        self.motor = SparkMax(SparkId.CLIMBER, SparkMax.MotorType.kBrushless)

        motor_config = SparkMaxConfig()
        motor_config.inverted(True)
        motor_config.setIdleMode(SparkMaxConfig.IdleMode.kBrake)

        self.motor.configure(
            motor_config,
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )

    def deploy(self) -> None:
        if not self.is_deployed:
            self.target_speed = self.winch_voltage

    def retract(self) -> None:
        if not self.is_retracted:
            self.target_speed = -self.winch_voltage

    @feedback
    def raw_encoder_val(self) -> float:
        return self.encoder.get()

    @feedback
    def encoder_angle(self) -> float:
        return self.encoder.get() * (180 / math.pi)

    @feedback
    def is_deployed(self) -> bool:
        return self.encoder_angle() >= self.MAX_ANGLE

    @feedback
    def is_retracted(self) -> bool:
        return self.encoder_angle() <= self.MIN_ANGLE

    def execute(self) -> None:
        self.motor.setVoltage(self.target_speed)
        self.target_speed = 0.0
