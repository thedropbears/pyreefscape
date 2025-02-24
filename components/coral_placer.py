from wpilib import Servo

from ids import PwmChannel


class CoralPlacerComponent:
    servo_resting_angle = 178
    servo_active_angle = 2

    def __init__(self) -> None:
        self.servo = Servo(PwmChannel.CORAL_SERVO)
        self.desired_angle = self.servo_resting_angle

    def on_enable(self) -> None:
        self.desired_angle = self.servo_resting_angle

    def coral_latch_open(self) -> None:
        self.desired_angle = self.servo_active_angle

    def coral_latch_closed(self) -> None:
        self.desired_angle = self.servo_resting_angle

    def execute(self):
        # set motor
        self.servo.set(self.desired_angle / 180.0)
