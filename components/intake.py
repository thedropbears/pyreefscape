from magicbot import tunable
from phoenix5 import ControlMode, TalonSRX

from ids import TalonId


class IntakeComponent:
    intake_output = tunable(0.5)

    def __init__(self) -> None:
        self.motor = TalonSRX(TalonId.INTAKE)
        self.motor.setInverted(False)

        self.desired_output = 0.0

    def intake(self):
        self.desired_output = self.intake_output

    def execute(self) -> None:
        self.motor.set(ControlMode.PercentOutput, self.desired_output)

        self.desired_output = 0.0
