from magicbot import tunable
from phoenix6.configs import MotorOutputConfigs
from phoenix6.controls import Follower, VoltageOut
from phoenix6.hardware import TalonFX
from phoenix6.signals import NeutralModeValue
from rev import SparkMax


class ManipulatorComponent:
    flywheel_setpoint = tunable(6.0)
    injector_setpoint = tunable(6.0)
    flywheel_intake_setpoint = tunable(4.0)
    injector_intake_setpoint = tunable(4.0)
    
    def __init__(self) -> None:
        self.injector_1 = SparkMax(5, SparkMax.MotorType.kBrushless)
        self.injector_2 = SparkMax(4, SparkMax.MotorType.kBrushless)

        self.flywheel_1 = TalonFX(9)
        self.flywheel_2 = TalonFX(10)
        self.flywheel_2.set_control(Follower(9, True))
        flywheel_1_config = self.flywheel_1.configurator
        flywheel_2_config = self.flywheel_2.configurator
        motor_config = MotorOutputConfigs()
        motor_config.neutral_mode = NeutralModeValue.COAST
        flywheel_1_config.apply(motor_config)
        flywheel_2_config.apply(motor_config)

        self.desired_flywheel_speed = 0.0
        self.desired_injector_speed = 0.0

    def spin_flywheels(self) -> None:
        self.desired_flywheel_speed = self.flywheel_setpoint

    def inject(self) -> None:
        self.desired_injector_speed = self.injector_setpoint

    def intake(self) -> None:
        self.desired_flywheel_speed = -self.flywheel_intake_setpoint
        self.desired_injector_speed = -self.injector_intake_setpoint

    def intake_inject_only(self) -> None:
        self.desired_injector_speed = -self.injector_intake_setpoint

    def execute(self) -> None:
        self.injector_1.setVoltage(-self.desired_injector_speed)
        self.injector_2.setVoltage(self.desired_injector_speed)

        self.flywheel_1.set_control(VoltageOut(self.desired_flywheel_speed))

        self.desired_flywheel_speed = 0.0
        self.desired_injector_speed = 0.0
