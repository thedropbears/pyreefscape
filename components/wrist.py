import math
import time

from magicbot import feedback
from rev import (
    SparkMax,
    SparkMaxConfig,
)
from wpilib import AnalogEncoder, AnalogInput
from wpimath.controller import ArmFeedforward, PIDController
from wpimath.trajectory import TrapezoidProfile

from ids import AnalogChannel, SparkId
from utilities.functions import clamp


class WristComponent:
    ENCODER_ZERO_OFFSET = 4.427696
    MAXIMUM_DEPRESSION = math.radians(-113.0)
    MAXIMUM_ELEVATION = math.radians(0)
    NEUTRAL_ANGLE = math.radians(-90.0)

    WRIST_MAX_VEL = math.radians(30.0)
    WRIST_MAX_ACC = math.radians(15.0)
    wrist_gear_ratio = 350.628
    TOLERANCE = math.radians(3.0)

    def __init__(self):
        self.wrist_encoder_raw = AnalogInput(AnalogChannel.WRIST_ENCODER)

        self.wrist_encoder = AnalogEncoder(self.wrist_encoder_raw, math.tau, 0)
        self.wrist_encoder.setInverted(False)
        self.wrist_encoder.setVoltagePercentageRange(0.2 / 5, 4.8 / 5)

        self.motor = SparkMax(SparkId.WRIST, SparkMax.MotorType.kBrushless)

        wrist_config = SparkMaxConfig()
        wrist_config.inverted(False)
        wrist_config.setIdleMode(SparkMaxConfig.IdleMode.kBrake)

        self.wrist_profile = TrapezoidProfile(
            TrapezoidProfile.Constraints(self.WRIST_MAX_VEL, self.WRIST_MAX_ACC)
        )
        # Values need to be modified with new gear ratio
        self.pid = PIDController(Kp=13.387, Ki=0, Kd=0.0078403)

        self.wrist_ff = ArmFeedforward(kS=0.42619, kG=0.13, kV=6.83, kA=0.0)

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
    def encoder_raw_volts(self) -> float:
        return self.wrist_encoder_raw.getVoltage()

    @feedback
    def inclination(self) -> float:
        return self.wrist_encoder.get() - self.ENCODER_ZERO_OFFSET

    @feedback
    def inclination_deg(self) -> float:
        return math.degrees(self.inclination())

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
        clamped_angle = clamp(pos, self.MAXIMUM_DEPRESSION, self.MAXIMUM_ELEVATION)

        # If the new setpoint is within the tolerance we wouldn't move anyway
        if abs(clamped_angle - self.desired_angle) > self.TOLERANCE:
            self.desired_angle = clamped_angle
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
