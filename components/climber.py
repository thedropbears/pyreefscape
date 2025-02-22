import math

from magicbot import feedback, tunable
from rev import LimitSwitchConfig, SparkMax, SparkMaxConfig
from wpilib import DutyCycleEncoder

from ids import DioChannel, SparkId


class ClimberComponent:
    target_speed = 0.0
    winch_voltage = tunable(12.0)
    MIN_ANGLE = 40
    MAX_ANGLE = 90
    ENCODER_OFFSET = 0  # TODO measure correct value

    def __init__(self) -> None:
        self.encoder = DutyCycleEncoder(DioChannel.CLIMBER_ENCODER, math.tau, 0)
        self.encoder.setInverted(False)  # TODO change to correct value
        self.motor = SparkMax(SparkId.CLIMBER, SparkMax.MotorType.kBrushless)

        motor_config = SparkMaxConfig()
        motor_config.inverted(True)
        motor_config.setIdleMode(SparkMaxConfig.IdleMode.kBrake)

        motor_config.limitSwitch.reverseLimitSwitchType(
            LimitSwitchConfig.Type.kNormallyOpen
        )
        motor_config.limitSwitch.reverseLimitSwitchEnabled(True)

        self.motor.configure(
            motor_config,
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )

    def deploy(self) -> None:
        if not self.is_deployed():
            self.target_speed = self.winch_voltage

    def retract(self) -> None:
        if not self.is_retracted():
            self.target_speed = -self.winch_voltage

    @feedback
    def raw_encoder_val(self) -> float:
        return self.encoder.get()

    @feedback
    def encoder_angle(self) -> float:
        return (self.encoder.get() - self.ENCODER_OFFSET) * (180 / math.pi)

    @feedback
    def is_deployed(self) -> bool:
        return math.isclose(self.encoder_angle(), self.MAX_ANGLE, abs_tol=2.0)

    @feedback
    def is_retracted(self) -> bool:
        return self.motor.getReverseLimitSwitch().get()

    def execute(self) -> None:
        self.motor.setVoltage(self.target_speed)
        self.target_speed = 0.0
