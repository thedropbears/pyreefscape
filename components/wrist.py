import math

import wpilib
from magicbot import feedback
from rev import (
    LimitSwitchConfig,
    SparkMax,
    SparkMaxConfig,
)
from wpilib import DutyCycleEncoder
from wpimath.controller import ArmFeedforward, PIDController
from wpimath.trajectory import TrapezoidProfile

from ids import DioChannel, SparkId
from utilities.functions import clamp
from utilities.rev import (
    configure_spark_ephemeral,
    configure_spark_reset_and_persist,
    configure_through_bore_encoder,
)


class WristComponent:
    ENCODER_ZERO_OFFSET = 3.46463832679
    COM_DIFFERENCE = 0.54722467321
    MAXIMUM_DEPRESSION = math.radians(-112.0) + COM_DIFFERENCE
    MAXIMUM_ELEVATION = math.radians(0) + COM_DIFFERENCE
    NEUTRAL_ANGLE = math.radians(-90.0)

    WRIST_MAX_VEL = math.radians(180.0)
    WRIST_MAX_ACC = math.radians(360.0)
    wrist_gear_ratio = 208.206
    TOLERANCE = math.radians(3.0)
    VEL_TOLERANCE = math.radians(6.0)

    def __init__(self, mech_root: wpilib.MechanismRoot2d):
        self.wrist_ligament = mech_root.appendLigament(
            "wrist", length=0.5, angle=0, color=wpilib.Color8Bit(wpilib.Color.kYellow)
        )

        self.wrist_COM_ligament = mech_root.appendLigament(
            "wrist_COM", length=0.5, angle=0, color=wpilib.Color8Bit(wpilib.Color.kBlue)
        )

        self.wrist_encoder = DutyCycleEncoder(DioChannel.WRIST_ENCODER, math.tau, 0)
        configure_through_bore_encoder(self.wrist_encoder)
        self.wrist_encoder.setInverted(False)

        self.motor = SparkMax(SparkId.WRIST, SparkMax.MotorType.kBrushless)

        self.idle_mode = SparkMaxConfig.IdleMode.kBrake
        wrist_config = SparkMaxConfig()
        wrist_config.inverted(False)
        wrist_config.setIdleMode(self.idle_mode)
        wrist_config.limitSwitch.reverseLimitSwitchType(
            LimitSwitchConfig.Type.kNormallyOpen
        )
        wrist_config.limitSwitch.reverseLimitSwitchEnabled(True)

        self.wrist_profile = TrapezoidProfile(
            TrapezoidProfile.Constraints(self.WRIST_MAX_VEL, self.WRIST_MAX_ACC)
        )
        # theoretical max pos 0.01 max velocity 0.05
        self.pid = PIDController(Kp=19.508, Ki=0, Kd=0.048599)

        # https://www.reca.lc/arm?armMass=%7B%22s%22%3A8%2C%22u%22%3A%22kg%22%7D&comLength=%7B%22s%22%3A0.15%2C%22u%22%3A%22m%22%7D&currentLimit=%7B%22s%22%3A40%2C%22u%22%3A%22A%22%7D&efficiency=90&endAngle=%7B%22s%22%3A-10%2C%22u%22%3A%22deg%22%7D&iterationLimit=10000&motor=%7B%22quantity%22%3A1%2C%22name%22%3A%22NEO%22%7D&ratio=%7B%22magnitude%22%3A432%2C%22ratioType%22%3A%22Reduction%22%7D&startAngle=%7B%22s%22%3A-110%2C%22u%22%3A%22deg%22%7D
        self.wrist_ff = ArmFeedforward(kS=0.42619, kG=0.48, kV=4.10, kA=0.02)

        wrist_config.encoder.positionConversionFactor(
            math.tau * (1 / self.wrist_gear_ratio)
        )
        wrist_config.encoder.velocityConversionFactor(
            (1 / 60) * math.tau * (1 / self.wrist_gear_ratio)
        )

        configure_spark_reset_and_persist(self.motor, wrist_config)

        self.motor_encoder = self.motor.getEncoder()

        self.desired_state = TrapezoidProfile.State(WristComponent.NEUTRAL_ANGLE, 0.0)

        self.last_setpoint_update_time = wpilib.Timer.getFPGATimestamp()
        self.initial_state = TrapezoidProfile.State(
            self.inclination(), self.current_velocity()
        )

    def on_enable(self):
        self._tilt_to(self.inclination())
        self.idle_mode = SparkMaxConfig.IdleMode.kBrake
        wrist_config = SparkMaxConfig()
        wrist_config.setIdleMode(self.idle_mode)

        configure_spark_ephemeral(self.motor, wrist_config)

    def on_disable(self):
        wrist_config = SparkMaxConfig()
        self.idle_mode = SparkMaxConfig.IdleMode.kBrake
        wrist_config.setIdleMode(self.idle_mode)

        configure_spark_ephemeral(self.motor, wrist_config)

    def toggle_neutral_mode(self) -> None:
        if self.idle_mode == SparkMaxConfig.IdleMode.kBrake:
            self.idle_mode = SparkMaxConfig.IdleMode.kCoast
        else:
            self.idle_mode = SparkMaxConfig.IdleMode.kBrake

        wrist_config = SparkMaxConfig()
        wrist_config.setIdleMode(self.idle_mode)

        configure_spark_ephemeral(self.motor, wrist_config)

    @feedback
    def raw_encoder(self) -> float:
        return self.wrist_encoder.get()

    @feedback
    def inclination(self) -> float:
        return self.wrist_encoder.get() - self.ENCODER_ZERO_OFFSET

    @feedback
    def inclination_deg(self) -> float:
        return math.degrees(self.inclination())

    @feedback
    def shooter_FOR_inclination(self) -> float:
        return math.degrees(self.inclination() - self.COM_DIFFERENCE)

    @feedback
    def shoot_angle_deg(self) -> float:
        return self.inclination_deg() + 90

    @feedback
    def current_velocity(self) -> float:
        return self.motor_encoder.getVelocity()

    @feedback
    def at_setpoint(self) -> bool:
        return (
            abs(self.desired_state.position - self.inclination())
            < WristComponent.TOLERANCE
        ) and abs(
            self.desired_state.velocity - self.current_velocity()
        ) < WristComponent.VEL_TOLERANCE

    # Tilts to an angle with regards to the shooter frame of reference.
    # Shooter FOR takes the arm as in-line with the backplate of the shooter
    # SHould be called with angles in the shooter FOR
    def tilt_to_shooter_FOR(self, pos: float) -> None:
        clamped_angle = clamp(
            (pos + self.COM_DIFFERENCE), self.MAXIMUM_DEPRESSION, self.MAXIMUM_ELEVATION
        )
        # If the new setpoint is within the tolerance we wouldn't move anyway
        if abs(clamped_angle - self.desired_state.position) > self.TOLERANCE:
            self._tilt_to(clamped_angle)

    # Tilts to an angle with respect to the COM FOR
    def tilt_to(self, pos: float) -> None:
        clamped_angle = clamp(pos, self.MAXIMUM_DEPRESSION, self.MAXIMUM_ELEVATION)

        # If the new setpoint is within the tolerance we wouldn't move anyway
        if abs(clamped_angle - self.desired_state.position) > self.TOLERANCE:
            self._tilt_to(clamped_angle)

    def _tilt_to(self, pos: float):
        clamped_angle = clamp(pos, self.MAXIMUM_DEPRESSION, self.MAXIMUM_ELEVATION)

        self.desired_state = TrapezoidProfile.State(clamped_angle, 0.0)
        self.last_setpoint_update_time = wpilib.Timer.getFPGATimestamp()
        self.initial_state = TrapezoidProfile.State(
            self.inclination(), self.current_velocity()
        )

    def go_to_neutral(self) -> None:
        self.tilt_to_shooter_FOR(WristComponent.NEUTRAL_ANGLE)

    @feedback
    def at_limit(self) -> bool:
        return self.motor.getReverseLimitSwitch().get()

    def execute(self) -> None:
        tracked_state = self.wrist_profile.calculate(
            wpilib.Timer.getFPGATimestamp() - self.last_setpoint_update_time,
            self.initial_state,
            self.desired_state,
        )
        ff = self.wrist_ff.calculate(tracked_state.position, tracked_state.velocity)

        self.motor.setVoltage(
            self.pid.calculate(self.inclination(), tracked_state.position) + ff
        )

        self.wrist_ligament.setAngle(self.shooter_FOR_inclination())

        self.wrist_COM_ligament.setAngle(self.inclination_deg())
        # self.wrist_ligament.setAngle(math.degrees(desired_state.position))
