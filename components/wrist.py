import math
import time

from magicbot import feedback, tunable
from rev import (
    SparkMax,
    SparkMaxConfig,
)
from wpilib import AnalogEncoder
from wpimath.controller import ArmFeedforward, PIDController
from wpimath.trajectory import TrapezoidProfile

from ids import AnalogChannel, SparkId
from utilities.functions import clamp


class WristComponent:
    ENCODER_ZERO_OFFSET = math.tau / 10
    MAXIMUM_DEPRESSION = math.radians(-113.0)
    MAXIMUM_ELEVATION = math.radians(-10.0)
    NEUTRAL_ANGLE = math.radians(-90.0)

    WRIST_MAX_VEL = math.radians(30.0)
    WRIST_MAX_ACC = math.radians(15.0)
    # New gear ratio to be calculated with wrist encoder
    angle_change_rate_while_zeroing = tunable(math.radians(0.1))
    wrist_gear_ratio = 432.0
    TOLERANCE = math.radians(3.0)

    def __init__(self):
        self.wrist_encoder = AnalogEncoder(
            AnalogChannel.WRIST_ENCODER, math.tau, self.ENCODER_ZERO_OFFSET
        )
        self.wrist_encoder.setInverted(True)

        self.motor = SparkMax(SparkId.WRIST, SparkMax.MotorType.kBrushless)

        wrist_config = SparkMaxConfig()
        wrist_config.inverted(False)
        wrist_config.setIdleMode(SparkMaxConfig.IdleMode.kBrake)

        self.wrist_profile = TrapezoidProfile(
            TrapezoidProfile.Constraints(self.WRIST_MAX_VEL, self.WRIST_MAX_ACC)
        )
        # Values need to be modified with new gear ratio
        self.pid = PIDController(Kp=7.6813, Ki=0, Kd=69.887)

        self.wrist_ff = ArmFeedforward(kS=0.42619, kG=0.09, kV=8.42, kA=0.0)

        wrist_config.encoder.positionConversionFactor(
            math.tau * (1 / self.wrist_gear_ratio)
        )
        wrist_config.encoder.velocityConversionFactor(
            (1 / 60) * math.tau * (1 / self.wrist_gear_ratio)
        )

        self.motor.configure(
            wrist_config,
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )

        self.motor_encoder = self.motor.getEncoder()

        self.desired_angle = WristComponent.NEUTRAL_ANGLE

    def on_enable(self):
        self.tilt_to(WristComponent.NEUTRAL_ANGLE)
        wrist_config = SparkMaxConfig()
        wrist_config.setIdleMode(SparkMaxConfig.IdleMode.kBrake)
        self.motor.configure(
            wrist_config,
            SparkMax.ResetMode.kNoResetSafeParameters,
            SparkMax.PersistMode.kNoPersistParameters,
        )

    def on_disable(self):
        wrist_config = SparkMaxConfig()
        wrist_config.setIdleMode(SparkMaxConfig.IdleMode.kCoast)
        self.motor.configure(
            wrist_config,
            SparkMax.ResetMode.kNoResetSafeParameters,
            SparkMax.PersistMode.kNoPersistParameters,
        )

    @feedback
    def inclination(self) -> float:
        return self.wrist_encoder.get()

    @feedback
    def inclination_deg(self) -> float:
        return math.degrees(self.wrist_encoder.get())

    @feedback
    def shoot_angle_deg(self) -> float:
        return self.inclination_deg() + 90

    @feedback
    def current_velocity(self) -> float:
        return self.motor_encoder.getVelocity()

    @feedback
    def at_setpoint(self) -> bool:
        return abs(self.desired_angle - self.inclination()) < WristComponent.TOLERANCE

    def tilt_to(self, pos: float) -> None:
        self.desired_angle = clamp(pos, self.MAXIMUM_DEPRESSION, self.MAXIMUM_ELEVATION)
        self.last_setpoint_update_time = time.monotonic()

    def go_to_neutral(self) -> None:
        self.tilt_to(WristComponent.NEUTRAL_ANGLE)

    def reset_windup(self) -> None:
        self.tilt_to(self.inclination())

    def execute(self) -> None:
        desired_state = self.wrist_profile.calculate(
            time.monotonic() - self.last_setpoint_update_time,
            TrapezoidProfile.State(self.inclination(), self.current_velocity()),
            TrapezoidProfile.State(self.desired_angle, 0.0),
        )
        ff = self.wrist_ff.calculate(desired_state.position, desired_state.velocity)

        self.motor.setVoltage(
            self.pid.calculate(self.inclination(), desired_state.position) + ff
        )
