import math

import wpilib
from magicbot import feedback, tunable, will_reset_to
from phoenix6.configs import (
    ClosedLoopRampsConfigs,
    FeedbackConfigs,
    MotorOutputConfigs,
    Slot0Configs,
)
from phoenix6.controls import VelocityVoltage
from phoenix6.hardware import TalonFX
from phoenix6.signals import InvertedValue, NeutralModeValue

from ids import TalonId
from utilities.game import ALGAE_MAX_DIAMETER, ALGAE_MIN_DIAMETER


class ShooterComponent:
    top_desired_flywheel_speed = will_reset_to(0.0)
    bottom_desired_flywheel_speed = will_reset_to(0.0)

    FLYWHEEL_INTAKE_SPEED = tunable(-20.0)
    FLYWHEEL_RPS_TOLERANCE = 1.0
    FLYWHEEL_RAMP_TIME = 1
    FLYWHEEL_GEAR_RATIO = 1 / (1.0 / 1.0)

    def __init__(self) -> None:
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

        self._algae_size = 16.4
        self.has_measured = True

    @property
    def algae_size(self) -> float:
        return (
            self._algae_size
            if not wpilib.RobotBase.isSimulation()
            else (ALGAE_MIN_DIAMETER + ALGAE_MAX_DIAMETER) / 2.0
        )

    @algae_size.setter
    def algae_size(self, value: float) -> None:
        self._algae_size = value
        self.has_measured = True

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
            abs_tol=ShooterComponent.FLYWHEEL_RPS_TOLERANCE,
        )

    @feedback
    def bottom_flywheels_up_to_speed(self) -> bool:
        return math.isclose(
            self.bottom_desired_flywheel_speed,
            self.bottom_flywheel.get_velocity().value,
            abs_tol=ShooterComponent.FLYWHEEL_RPS_TOLERANCE,
        )

    @feedback
    def flywheel_speeds(self) -> tuple[float, float]:
        return (
            self.top_flywheel.get_velocity().value,
            self.bottom_flywheel.get_velocity().value,
        )

    def intake(self) -> None:
        self.top_desired_flywheel_speed = self.FLYWHEEL_INTAKE_SPEED
        self.bottom_desired_flywheel_speed = self.FLYWHEEL_INTAKE_SPEED

    @feedback
    def algae_size_feedback(self) -> float:
        return self.algae_size

    @feedback
    def flywheel_positions(self) -> tuple[float, float]:
        return (
            self.top_flywheel.get_position().value_as_double,
            self.bottom_flywheel.get_position().value_as_double,
        )

    def execute(self) -> None:
        self.top_flywheel.set_control(
            VelocityVoltage(
                self.top_desired_flywheel_speed, override_brake_dur_neutral=True
            )
        )

        self.bottom_flywheel.set_control(
            VelocityVoltage(
                self.bottom_desired_flywheel_speed, override_brake_dur_neutral=True
            )
        )
