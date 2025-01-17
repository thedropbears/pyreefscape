from magicbot import tunable
from phoenix6.configs import MotorOutputConfigs
from phoenix6.controls import Follower, VoltageOut
from phoenix6.hardware import TalonFX
from phoenix6.signals import NeutralModeValue
from rev import SparkMax, SparkMaxConfig


class ManipulatorComponent:
    flywheel_shoot_speed = tunable(-6.0)
    flywheel_intake_speed = tunable(2.0)
    injector_inject_speed = tunable(-6.0)

    def __init__(self) -> None:
        self.injector_1 = SparkMax(5, SparkMax.MotorType.kBrushless)
        self.injector_2 = SparkMax(4, SparkMax.MotorType.kBrushless)
        injector_config = SparkMaxConfig()

        injector_config.inverted(False)
        self.injector_1.configure(
            injector_config,
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )
        injector_config.follow(5, True)
        self.injector_2.configure(
            injector_config,
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )

        self.flywheel_1 = TalonFX(9)
        self.flywheel_2 = TalonFX(10)
        flywheel_1_config = self.flywheel_1.configurator
        flywheel_2_config = self.flywheel_2.configurator
        flywheel_config = MotorOutputConfigs()
        flywheel_config.neutral_mode = NeutralModeValue.COAST

        flywheel_1_config.apply(flywheel_config)
        flywheel_2_config.apply(flywheel_config)

        self.desired_flywheel_speed = 0.0
        self.desired_injector_speed = 0.25

    def spin_flywheels(self) -> None:
        self.desired_flywheel_speed = self.flywheel_shoot_speed

    def inject(self) -> None:
        self.desired_injector_speed = self.injector_inject_speed

    def intake(self) -> None:
        self.desired_flywheel_speed = self.flywheel_intake_speed

    def execute(self) -> None:
        self.injector_1.setVoltage(self.desired_injector_speed)

        self.flywheel_1.set_control(VoltageOut(self.desired_flywheel_speed))
        self.flywheel_2.set_control(Follower(9, True))

        self.desired_flywheel_speed = 0.0
        self.desired_injector_speed = 0.25
