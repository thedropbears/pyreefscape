from magicbot import tunable
from phoenix6.configs import MotorOutputConfigs
from phoenix6.controls import Follower, NeutralOut, VoltageOut
from phoenix6.hardware import TalonFX
from phoenix6.signals import NeutralModeValue


class ManipulatorComponent:
    def __init__(self) -> None:
        self.left_flywheel = TalonFX(9)
        self.right_flywheel = TalonFX(10)
        self.right_flywheel.set_control(Follower(9, True))
        left_flywheel_config = self.left_flywheel.configurator
        right_flywheel_config = self.right_flywheel.configurator
        motor_config = MotorOutputConfigs()
        motor_config.neutral_mode = NeutralModeValue.COAST
        left_flywheel_config.apply(motor_config)
        right_flywheel_config.apply(motor_config)

        self.desired_flywheel_speed = tunable(0.0)
        self.left_flywheel.set_control(NeutralOut())

    def spin_flywheels(self) -> None:
        self.left_flywheel.set_control(VoltageOut(self.desired_flywheel_speed))

    def coast_flywheels(self) -> None:
        self.left_flywheel.set_control(NeutralOut())

    def execute(self) -> None:
        pass
