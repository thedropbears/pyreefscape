import math

from magicbot import feedback, tunable
from phoenix6.configs import (
    ClosedLoopRampsConfigs,
    FeedbackConfigs,
    MotorOutputConfigs,
    Slot0Configs,
)
from phoenix6.controls import VelocityVoltage
from phoenix6.hardware import TalonFX
from phoenix6.signals import InvertedValue, NeutralModeValue
from rev import SparkMax, SparkMaxConfig
from wpilib import DigitalInput

from ids import DioChannel, SparkId, TalonId


class AlgaeManipulatorComponent:
    FLYWHEEL_INTAKE_SPEED = tunable(-20.0)
    INJECTOR_INJECT_SPEED = tunable(12.0)
    INJECTOR_INTAKE_SPEED = tunable(200)
    INJECTOR_BACKDRIVE_SPEED = tunable(-1.0)
    INJECTOR_MEASURE_SPEED = tunable(75)

    FLYWHEEL_RPS_TOLERANCE = 1.0
    FLYWHEEL_RAMP_TIME = 1
    FLYWHEEL_GEAR_RATIO = 1 / (1.0 / 1.0)
    MEASUREMENT_CURRENT_THRESHOLD = 20.0
    INJECTOR_RPS_TOLERANCE = 0.5
    INJECTOR_MAX_ACCEL = 0.5

    def __init__(self) -> None:
        self.injector_1 = SparkMax(SparkId.INJECTOR_1, SparkMax.MotorType.kBrushless)
        self.injector_2 = SparkMax(SparkId.INJECTOR_2, SparkMax.MotorType.kBrushless)
        self.injector_1_closed_loop = self.injector_1.getClosedLoopController()
        self.injector_2_closed_loop = self.injector_2.getClosedLoopController()
        injector_config = SparkMaxConfig()

        self.algae_limit_switch = DigitalInput(DioChannel.ALGAE_INTAKE_SWITCH)

        injector_config.inverted(True)
        injector_config.closedLoop.maxMotion.maxAcceleration(
            self.INJECTOR_MAX_ACCEL
        ).allowedClosedLoopError(self.INJECTOR_RPS_TOLERANCE)
        injector_config.closedLoop.velocityFF(1 / 917)
        # PID Values Need To Be Implemented
        injector_config.closedLoop.P(0.0001).I(0.0).D(0.0)
        self.injector_1.configure(
            injector_config,
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )
        injector_config.inverted(False)
        self.injector_2.configure(
            injector_config,
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )

        self.injector_1_encoder = self.injector_1.getEncoder()
        self.injector_2_encoder = self.injector_2.getEncoder()

        self.top_flywheel = TalonFX(TalonId.TOP_FLYWHEEL)
        self.bottom_flywheel = TalonFX(TalonId.BOTTOM_FLYWHEEL)
        top_flywheel_config = self.top_flywheel.configurator
        bottom_flywheel_config = self.bottom_flywheel.configurator
        flywheel_config = MotorOutputConfigs()
        flywheel_config.neutral_mode = NeutralModeValue.COAST
        flywheel_config.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE

        flywheel_top_pid = (
            Slot0Configs()
            .with_k_p(0.1586)
            .with_k_i(0.0)
            .with_k_d(0.0)
            .with_k_s(0.058604)
            .with_k_v(0.11164)
            .with_k_a(0.010271)
        )

        flywheel_bottom_pid = (
            Slot0Configs()
            .with_k_p(0.10694)
            .with_k_i(0.0)
            .with_k_d(0.0)
            .with_k_s(0.015251)
            .with_k_v(0.11226)
            .with_k_a(0.0207)
        )

        flywheel_gear_ratio = FeedbackConfigs().with_sensor_to_mechanism_ratio(
            self.FLYWHEEL_GEAR_RATIO
        )

        flywheel_closed_loop_ramp_config = (
            ClosedLoopRampsConfigs().with_voltage_closed_loop_ramp_period(
                self.FLYWHEEL_RAMP_TIME
            )
        )

        top_flywheel_config.apply(flywheel_config)
        top_flywheel_config.apply(flywheel_top_pid)
        top_flywheel_config.apply(flywheel_gear_ratio)
        top_flywheel_config.apply(flywheel_closed_loop_ramp_config)

        bottom_flywheel_config.apply(flywheel_config)
        bottom_flywheel_config.apply(flywheel_bottom_pid)
        bottom_flywheel_config.apply(flywheel_gear_ratio)
        bottom_flywheel_config.apply(flywheel_closed_loop_ramp_config)

        self.top_desired_flywheel_speed = 0.0
        self.bottom_desired_flywheel_speed = 0.0
        self.desired_injector_speed = 0.0

        self.algae_size = 0.0
        self.desired_feeler_angle = 90.0

        self.has_seen_algae: bool = False
        self.measurement_in_process = False
        self.algae_measured = False

    def spin_flywheels(
        self, top_flywheel_shoot_speed: float, bottom_flywheel_shoot_speed: float
    ) -> None:
        self.top_desired_flywheel_speed = top_flywheel_shoot_speed
        self.bottom_desired_flywheel_speed = bottom_flywheel_shoot_speed

    @feedback
    def top_flywheels_up_to_speed(self) -> bool:
        return math.isclose(
            self.top_desired_flywheel_speed,
            self.top_flywheel.get_velocity().value,
            abs_tol=AlgaeManipulatorComponent.FLYWHEEL_RPS_TOLERANCE,
        )

    @feedback
    def bottom_flywheels_up_to_speed(self) -> bool:
        return math.isclose(
            self.bottom_desired_flywheel_speed,
            self.bottom_flywheel.get_velocity().value,
            abs_tol=AlgaeManipulatorComponent.FLYWHEEL_RPS_TOLERANCE,
        )

    @feedback
    def top_flywheel_speed(self) -> float:
        return self.top_flywheel.get_velocity().value

    @feedback
    def bottom_flywheel_speed(self) -> float:
        return self.bottom_flywheel.get_velocity().value

    def inject(self) -> None:
        self.desired_injector_speed = self.INJECTOR_INJECT_SPEED
        self.has_seen_algae = False

    def intake(self) -> None:
        self.top_desired_flywheel_speed = self.FLYWHEEL_INTAKE_SPEED
        self.bottom_desired_flywheel_speed = self.FLYWHEEL_INTAKE_SPEED
        self.desired_injector_speed = self.INJECTOR_INTAKE_SPEED

    def measure_algae(self) -> None:
        self.desired_injector_speed = self.INJECTOR_MEASURE_SPEED

    def get_injector_1_position(self) -> float:
        return self.injector_1_encoder.getPosition()

    def get_injector_2_position(self) -> float:
        return self.injector_2_encoder.getPosition()

    def flywheel_top_position(self) -> float:
        return self.top_flywheel.get_position().value_as_double

    @feedback
    def flywheel_bottom_position(self) -> float:
        return self.bottom_flywheel.get_position().value_as_double

    @feedback
    def algae_size_feedback(self) -> float:
        return self.algae_size

    @feedback
    def _algae_limit_switch_pressed(self) -> bool:
        return not self.algae_limit_switch.get()

    @feedback
    def has_algae(self) -> bool:
        return self.has_seen_algae

    def execute(self) -> None:
        if self._algae_limit_switch_pressed():
            self.has_seen_algae = True

        self.top_flywheel.set_control(VelocityVoltage(self.top_desired_flywheel_speed))

        self.bottom_flywheel.set_control(
            VelocityVoltage(self.bottom_desired_flywheel_speed)
        )

        if math.isclose(self.desired_injector_speed, 0.0) and self.has_algae():
            self.injector_1.setVoltage(self.INJECTOR_BACKDRIVE_SPEED)
            self.injector_2.setVoltage(self.INJECTOR_BACKDRIVE_SPEED)
        else:
            self.injector_1_closed_loop.setReference(
                self.desired_injector_speed,
                SparkMax.ControlType.kVelocity,
            )
            self.injector_2_closed_loop.setReference(
                self.desired_injector_speed,
                SparkMax.ControlType.kVelocity,
            )

        self.top_desired_flywheel_speed = 0.0
        self.bottom_desired_flywheel_speed = 0.0
        self.desired_injector_speed = 0.0
