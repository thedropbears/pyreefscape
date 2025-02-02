from magicbot import feedback, tunable
from phoenix6.configs import (
    ClosedLoopRampsConfigs,
    FeedbackConfigs,
    MotorOutputConfigs,
    Slot0Configs,
)
from phoenix6.controls import NeutralOut, VelocityVoltage
from phoenix6.hardware import TalonFX
from phoenix6.signals import InvertedValue, NeutralModeValue
from rev import SparkMax, SparkMaxConfig
from wpilib import DigitalInput

from ids import DioChannel, SparkId, TalonId


class AlgaeManipulatorComponent:
    flywheel_intake_speed = tunable(-10)
    injector_inject_speed = tunable(6.0)
    injector_intake_speed = tunable(-0.5)
    injector_backdrive_speed = tunable(-0.2)

    FLYWHEEL_RPS_TOLERENCE = 1.0
    FLYWHEEL_RAMP_TIME = 1
    FLYWHEEL_GEAR_RATIO = 1 / (1.0 / 1.0)

    def __init__(self) -> None:
        self.injector_1 = SparkMax(SparkId.INJECTOR_1, SparkMax.MotorType.kBrushless)
        self.injector_2 = SparkMax(SparkId.INJECTOR_2, SparkMax.MotorType.kBrushless)
        injector_config = SparkMaxConfig()

        self.algae_limit_switch = DigitalInput(DioChannel.ALGAE_INTAKE_SWITCH)

        injector_config.inverted(True)
        self.injector_1.configure(
            injector_config,
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )
        injector_config.follow(SparkId.INJECTOR_1, True)
        self.injector_2.configure(
            injector_config,
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )

        self.flywheel_1 = TalonFX(TalonId.FLYWHEEL_1)
        self.flywheel_2 = TalonFX(TalonId.FLYWHEEL_2)
        flywheel_1_config = self.flywheel_1.configurator
        flywheel_2_config = self.flywheel_2.configurator
        flywheel_config = MotorOutputConfigs()
        flywheel_config.neutral_mode = NeutralModeValue.COAST
        flywheel_config.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE

        flywheel_pid = (
            Slot0Configs()
            .with_k_p(0.047949)
            .with_k_i(0.0)
            .with_k_d(0.0)
            .with_k_s(0.10383)
            .with_k_v(0.11228)
            .with_k_a(0.0062382)
        )

        flywheel_gear_ratio = FeedbackConfigs().with_sensor_to_mechanism_ratio(
            self.FLYWHEEL_GEAR_RATIO
        )

        flywheel_closed_loop_ramp_config = (
            ClosedLoopRampsConfigs().with_voltage_closed_loop_ramp_period(
                self.FLYWHEEL_RAMP_TIME
            )
        )

        flywheel_1_config.apply(flywheel_config)
        flywheel_1_config.apply(flywheel_pid)
        flywheel_1_config.apply(flywheel_gear_ratio)
        flywheel_1_config.apply(flywheel_closed_loop_ramp_config)

        flywheel_2_config.apply(flywheel_config)
        flywheel_2_config.apply(flywheel_pid)
        flywheel_2_config.apply(flywheel_gear_ratio)
        flywheel_2_config.apply(flywheel_closed_loop_ramp_config)

        self.desired_flywheel_speed = 0.0
        self.desired_injector_speed = 0.25

        self.algae_size = 0.0
        self.desired_feeler_angle = 90.0

    def spin_flywheels(self, flywheel_shoot_speed: float) -> None:
        self.desired_flywheel_speed = flywheel_shoot_speed

    @feedback
    def flywheels_up_to_speed(self) -> bool:
        return (
            abs(self.flywheel_1.get_velocity().value - self.desired_flywheel_speed)
            <= self.FLYWHEEL_RPS_TOLERENCE
            and abs(self.flywheel_2.get_velocity().value - self.desired_flywheel_speed)
            <= self.FLYWHEEL_RPS_TOLERENCE
        )

    @feedback
    def flywheel_speed(self) -> float:
        return self.flywheel_1.get_velocity().value

    @feedback
    def flywheel_speed_2(self) -> float:
        return self.flywheel_2.get_velocity().value

    def inject(self) -> None:
        self.desired_injector_speed = self.injector_inject_speed

    def intake(self) -> None:
        self.desired_flywheel_speed = self.flywheel_intake_speed
        self.desired_injector_speed = self.injector_intake_speed

    @feedback
    def has_algae(self) -> bool:
        return not self.algae_limit_switch.get()

    def execute(self) -> None:
        if self.desired_flywheel_speed == 0:
            self.flywheel_1.set_control(NeutralOut())
            self.flywheel_2.set_control(NeutralOut())

        else:
            self.flywheel_1.set_control(VelocityVoltage(self.desired_flywheel_speed))
            self.flywheel_2.set_control(VelocityVoltage(self.desired_flywheel_speed))

        if self.desired_injector_speed == 0.0 and self.has_algae():
            self.injector_1.setVoltage(self.injector_backdrive_speed)
        else:
            self.injector_1.setVoltage(self.desired_injector_speed)

        self.desired_flywheel_speed = 0.0
        self.desired_injector_speed = 0.0
