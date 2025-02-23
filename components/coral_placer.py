from magicbot import tunable
from wpilib import Servo

from ids import PwmChannel


class CoralPlacerComponent:
    servo_resting_angle = tunable(70)
    servo_active_angle = tunable(180)

    def __init__(self):
        self.servo = Servo(PwmChannel.CORAL_SERVO)
        self.desired_angle = self.servo_resting_angle

    def unhook_coral_latch(self):
        self.servo.setAngle(self.servo_active_angle)

    def coral_latch_rest(self):
        self.servo.setAngle(self.servo_resting_angle)

    def execute(self):
        # set motor
        self.servo.setAngle(self.desired_angle)
