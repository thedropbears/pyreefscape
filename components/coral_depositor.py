from wpilib import Servo

from ids import PwmChannel


class CoralDepositorComponent:
    holding_angle = 90
    depositing_angle = 180
    tucked_angle = 0

    def __init__(self) -> None:
        self.servo = Servo(PwmChannel.CORAL_SERVO)
        self.retract()

    def on_disable(self):
        self.retract()

    def deposit(self):
        self.desired_angle = self.depositing_angle

    def tuck(self):
        self.desired_angle = self.tucked_angle

    def retract(self):
        self.desired_angle = self.holding_angle

    def execute(self):
        self.servo.set(self.desired_angle / 180.0)
