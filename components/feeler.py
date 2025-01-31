from magicbot import feedback, tunable
from wpilib import DigitalInput, Servo

from ids import DioChannel, PwmChannel


class FeelerComponent:
    DETECT_SPEED = tunable(0.6)  # degrees per cycle
    START_ANGLE = tunable(90)
    START_OFFEST = tunable(17)

    def __init__(self):
        self.is_inverted = True

        self.current_angle = 0.0
        self.desired_angle = 90.0

        self.algae_size = 0.0

        self.limit_switch = DigitalInput(DioChannel.FEELER_LIMIT_SWITCH)
        self.servo = Servo(PwmChannel.FEELER_SERVO)

    @feedback
    def touching_algae(self) -> bool:
        return self.limit_switch.get()

    def set_angle(self, rot: float = 0.0):
        if not self.is_inverted:
            self.desired_angle = rot
        else:
            self.desired_angle = 180 - rot

    def execute(self):
        self.servo.setAngle(self.desired_angle)
