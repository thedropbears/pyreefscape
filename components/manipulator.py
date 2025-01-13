from magicbot import tunable, will_reset_to
from phoenix6.configs import MotorOutputConfigs
from phoenix6.controls import VoltageOut
from phoenix6.hardware import TalonFX
from phoenix6.signals import NeutralModeValue
from rev import SparkMax


class ManipulatorComponent:
    flywheel_setpoint_1 = tunable(6.0)
    flywheel_setpoint_2 = tunable(6.0)
    injector_setpoint = tunable(6.0)
    flywheel_intake_setpoint = tunable(2.0)
    injector_intake_setpoint = tunable(0.25)
    desired_flywheel_speed_1 = will_reset_to(0.0)
    desired_flywheel_speed_2 = will_reset_to(0.0)
    desired_injector_speed = will_reset_to(0.25)

    def __init__(self) -> None:
        self.injector_1 = SparkMax(5, SparkMax.MotorType.kBrushless)
        self.injector_2 = SparkMax(4, SparkMax.MotorType.kBrushless)

        self.flywheel_1 = TalonFX(9)
        self.flywheel_2 = TalonFX(10)
        flywheel_1_config = self.flywheel_1.configurator
        flywheel_2_config = self.flywheel_2.configurator
        motor_config = MotorOutputConfigs()
        motor_config.neutral_mode = NeutralModeValue.COAST
        flywheel_1_config.apply(motor_config)
        flywheel_2_config.apply(motor_config)

    def spin_flywheels(self) -> None:
        self.desired_flywheel_speed_1 = self.flywheel_setpoint_1
        self.desired_flywheel_speed_2 = -self.flywheel_setpoint_2

    def inject(self) -> None:
        self.desired_injector_speed = self.injector_setpoint

    def intake(self) -> None:
        self.desired_flywheel_speed = self.flywheel_intake_setpoint
        self.desired_injector_speed = -self.injector_intake_setpoint

    def execute(self) -> None:
        self.injector_1.setVoltage(self.desired_injector_speed)
        self.injector_2.setVoltage(self.desired_injector_speed)

        self.flywheel_1.set_control(VoltageOut(self.desired_flywheel_speed_1))
        self.flywheel_2.set_control(VoltageOut(self.desired_flywheel_speed_2))
