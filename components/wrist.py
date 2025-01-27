import math
import time

from magicbot import feedback, tunable
from rev import ClosedLoopSlot, SparkMax, SparkMaxConfig
from wpilib import DigitalInput
from wpimath.controller import ArmFeedforward
from wpimath.trajectory import TrapezoidProfile

from ids import DioChannel, SparkId
from utilities.functions import clamp


class WristComponent:
    MAXIMUM_DEPRESSION = math.radians(-113.0)
    MAXIMUM_ELEVATION = math.radians(-10.0)
    NEUTRAL_ANGLE = math.radians(-90.0)

    WRIST_MAX_VEL = math.radians(30.0)
    WRIST_MAX_ACC = math.radians(15.0)

    angle_change_rate_while_zeroing = tunable(math.radians(0.1))
    # wrist_gear_ratio = (150.0 / 15) * 20 * (50 / 26)
    wrist_gear_ratio = 432
    TOLERANCE = math.radians(3.0)

    zeroing_voltage = tunable(-1.0)

    has_indexed = tunable(False)

    def __init__(self):
        self.switch = DigitalInput(DioChannel.WRIST_LIMIT_SWITCH)

        self.wrist = SparkMax(SparkId.WRIST, SparkMax.MotorType.kBrushless)

        self.wrist_controller = self.wrist.getClosedLoopController()

        wrist_config = SparkMaxConfig()
        wrist_config.inverted(False)
        wrist_config.setIdleMode(SparkMaxConfig.IdleMode.kBrake)
        wrist_config.closedLoop.P(
            7.6813,
            ClosedLoopSlot.kSlot0,
        )
        wrist_config.closedLoop.D(69.887, ClosedLoopSlot.kSlot0)
        self.wrist_profile = TrapezoidProfile(
            TrapezoidProfile.Constraints(self.WRIST_MAX_VEL, self.WRIST_MAX_ACC)
        )

        self.wrist_ff = ArmFeedforward(kS=0.42619, kG=0.31303, kV=7.984, kA=0.69063)

        wrist_config.encoder.positionConversionFactor(
            math.tau * (1 / self.wrist_gear_ratio)
        )
        wrist_config.encoder.velocityConversionFactor(
            (1 / 60) * math.tau * (1 / self.wrist_gear_ratio)
        )

        self.wrist.configure(
            wrist_config,
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )

        self.encoder = self.wrist.getEncoder()

        self.desired_angle = self.inclination()

    def on_enable(self):
        self.tilt_to(self.inclination())
        wrist_config = SparkMaxConfig()
        wrist_config.setIdleMode(SparkMaxConfig.IdleMode.kBrake)
        self.wrist.configure(
            wrist_config,
            SparkMax.ResetMode.kNoResetSafeParameters,
            SparkMax.PersistMode.kNoPersistParameters,
        )

    def on_disable(self):
        wrist_config = SparkMaxConfig()
        wrist_config.setIdleMode(SparkMaxConfig.IdleMode.kCoast)
        self.wrist.configure(
            wrist_config,
            SparkMax.ResetMode.kNoResetSafeParameters,
            SparkMax.PersistMode.kNoPersistParameters,
        )

    def zero_wrist(self) -> None:
        self.has_indexed = False

    @feedback
    def wrist_at_bottom_limit(self) -> bool:
        return not self.switch.get()

    @feedback
    def inclination(self) -> float:
        return self.encoder.getPosition()

    @feedback
    def inclination_deg(self) -> float:
        return math.degrees(self.encoder.getPosition())

    @feedback
    def shoot_angle_deg(self) -> float:
        return self.inclination_deg() + 90

    @feedback
    def current_velocity(self) -> float:
        return self.encoder.getVelocity()

    @feedback
    def at_setpoint(self) -> bool:
        return abs(self.desired_angle - self.inclination()) < WristComponent.TOLERANCE

    def tilt_to(self, pos: float) -> None:
        self.desired_angle = clamp(pos, self.MAXIMUM_DEPRESSION, self.MAXIMUM_ELEVATION)
        self.last_setpoint_update_time = time.monotonic()

    def go_to_neutral(self) -> None:
        self.tilt_to(WristComponent.NEUTRAL_ANGLE)

    def execute(self) -> None:
        if self.wrist_at_bottom_limit():
            self.encoder.setPosition(self.MAXIMUM_DEPRESSION)
            self.has_indexed = True

        if not self.has_indexed:
            self.wrist.setVoltage(self.zeroing_voltage)
            return

        desired_state = self.wrist_profile.calculate(
            time.monotonic() - self.last_setpoint_update_time,
            TrapezoidProfile.State(self.inclination(), self.current_velocity()),
            TrapezoidProfile.State(self.desired_angle, 0.0),
        )
        ff = self.wrist_ff.calculate(desired_state.position, desired_state.velocity)
        self.wrist_controller.setReference(
            desired_state.position,
            SparkMax.ControlType.kPosition,
            slot=ClosedLoopSlot.kSlot0,
            arbFeedforward=ff,
        )
