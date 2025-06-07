import math

from magicbot import feedback, tunable
from rev import LimitSwitchConfig, SparkMax, SparkMaxConfig
from wpilib import DigitalInput, DutyCycleEncoder
from wpimath.controller import PIDController

from components.led_component import LightStrip
from ids import DioChannel, SparkId
from utilities.functions import constrain_angle
from utilities.rev import configure_through_bore_encoder


class ClimberComponent:
    status_lights: LightStrip

    target_speed = 0.0
    winch_voltage = tunable(12.0)

    def __init__(self) -> None:
        self.START_ANGLE = -30.0
        self.DEPLOY_ANGLE = 16.0
        self.RETRACT_ANGLE = -52.0

        self.left_limit_switch = DigitalInput(DioChannel.LEFT_CLIMBER_SWITCH)
        self.right_limit_switch = DigitalInput(DioChannel.RIGHT_CLIMBER_SWITCH)

        self.desired_angle = math.radians(self.START_ANGLE)
        self.update_pid = True

        self.encoder = DutyCycleEncoder(DioChannel.CLIMBER_ENCODER, math.tau, 5.4)
        configure_through_bore_encoder(self.encoder)
        self.encoder.setInverted(False)

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

    def on_disable(self) -> None:
        self.update_pid = True
        self.go_to_neutral()

    def stop(self) -> None:
        self.set_angle(math.radians(self.encoder_angle()))

    @feedback
    def is_left_engaged(self) -> bool:
        return not self.left_limit_switch.get()

    @feedback
    def is_right_engaged(self) -> bool:
        return not self.right_limit_switch.get()

    def stop_pid_update(self) -> None:
        self.update_pid = False

    def start_pid_update(self) -> None:
        self.update_pid = True

    def set_angle(self, angle: float) -> None:
        self.desired_angle = angle

    def go_to_neutral(self) -> None:
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
        if self.is_retracted():
            self.status_lights.climber_retracting()
        if self.update_pid:
            pid_result = self.pid.calculate(self.raw_encoder_val(), self.desired_angle)
            self.motor.setVoltage(pid_result)
        else:
            self.motor.setVoltage(0)
