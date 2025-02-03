from magicbot import tunable
from phoenix5 import ControlMode, TalonSRX

from ids import TalonId


class IntakeComponent:
    intake_output = tunable(0.9)
    deploy_output = tunable(0.1)
    retract_output = tunable(-0.1)

    def __init__(self) -> None:
        self.motor = TalonSRX(TalonId.INTAKE)
        self.motor.setInverted(True)

        self.deploy_motor = TalonSRX(TalonId.DEPLOY_INTAKE)

        self.desired_output = 0.0
        self.desired_deploy_output = 0.0

    def intake(self):
        self.desired_output = self.intake_output

    def deploy(self) -> None:
        self.desired_deploy_output = self.deploy_output

    def retract(self) -> None:
        self.desired_deploy_output = self.retract_output

    def deploy_motor_current(self) -> float:
        return TalonSRX.getSupplyCurrent(self.deploy_motor)

    def execute(self) -> None:
        self.motor.set(ControlMode.PercentOutput, self.desired_output)

        self.deploy_motor.set(ControlMode.PercentOutput, self.desired_deploy_output)

        self.desired_output = 0.0
        self.desired_deploy_output = 0.0
