from wpilib import Servo

from ids import PwmChannel


class CoralDepositorComponent:
    holding_angle = 0
    depositing_angle = 180

    def __init__(self) -> None:
        self.servo = Servo(PwmChannel.CORAL_SERVO)
        self.retract()

    def on_disable(self):
        self.retract()

    def deposit(self):
        self.desired_angle = self.depositing_angle

    def retract(self):
        self.desired_angle = self.holding_angle

    def execute(self):
        self.servo.set(self.desired_angle / 180.0)
