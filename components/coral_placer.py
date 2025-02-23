from magicbot import tunable
from wpilib import Servo

from ids import PwmChannel


class CoralPlacerComponent:
    servo_resting_angle = tunable(70)
    servo_active_angle = tunable(180)

    def __init__(self) -> None:
        self.servo = Servo(PwmChannel.CORAL_SERVO)

    def setup(self) -> None:
        self.desired_angle = self.servo_resting_angle

    def coral_latch_open(self) -> None:
        self.servo.setAngle(self.servo_active_angle)

    def coral_latch_closed(self) -> None:
        self.servo.setAngle(self.servo_resting_angle)

    def execute(self):
        # set motor
        self.servo.setAngle(self.desired_angle)
