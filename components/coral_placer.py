from magicbot import tunable
from wpilib import Servo

from ids import PwmChannel


class CoralPlacerComponent:
    servo_resting_angle = tunable(70)
    servo_active_angle = tunable(180)

    def __init__(self):
        self.servo = Servo(PwmChannel.CORAL_SERVO)
        self.desired_angle = self.servo_resting_angle

    def coral_latch_open(self):
        self.servo.setAngle(self.servo_active_angle)

    def coral_latch_closed(self):
        self.servo.setAngle(self.servo_resting_angle)

    def execute(self):
        # set motor
        self.servo.setAngle(self.desired_angle)
