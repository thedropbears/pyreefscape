import math

from magicbot import feedback, tunable
from rev import LimitSwitchConfig, SparkMax, SparkMaxConfig
from wpilib import DutyCycleEncoder
from wpimath.controller import PIDController

from ids import DioChannel, SparkId
from utilities.functions import constrain_angle


class ClimberComponent:
    target_speed = 0.0
    winch_voltage = tunable(12.0)

    def __init__(self) -> None:
        self.START_ANGLE = -37.5
        self.DEPLOY_ANGLE = 11.0
        self.RETRACT_ANGLE = -49.0

        self.desired_angle = math.radians(self.START_ANGLE)

        self.encoder = DutyCycleEncoder(DioChannel.CLIMBER_ENCODER, math.tau, 5.4)
        self.encoder.setAssumedFrequency(975.6)
        self.encoder.setDutyCycleRange(1 / 1025, 1024 / 1025)
        self.encoder.setInverted(False)  # TODO change to correct value

        self.pid = PIDController(Kp=40, Ki=0, Kd=0)
        self.pid.enableContinuousInput(0.0, math.tau)

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

    def set_angle(self, angle: float) -> None:
        self.desired_angle = angle

    def go_to_start(self) -> None:
        self.set_angle(math.radians(self.START_ANGLE))

    def go_to_deploy(self) -> None:
        self.set_angle(math.radians(self.DEPLOY_ANGLE))

    def go_to_retract(self) -> None:
        self.set_angle(math.radians(self.RETRACT_ANGLE))

    @feedback
    def raw_encoder_val(self) -> float:
        return self.encoder.get()

    @feedback
    def encoder_angle(self) -> float:
        return math.degrees(constrain_angle(self.encoder.get()))

    @feedback
    def is_deployed(self) -> bool:
        return math.isclose(self.encoder_angle(), self.DEPLOY_ANGLE, abs_tol=2.0)

    @feedback
    def is_retracted(self) -> bool:
        return (
            math.isclose(self.encoder_angle(), self.RETRACT_ANGLE, abs_tol=2.0)
            or self.motor.getReverseLimitSwitch().get()
        )

    def execute(self) -> None:
        pid_result = self.pid.calculate(self.raw_encoder_val(), self.desired_angle)
        self.motor.setVoltage(pid_result)
