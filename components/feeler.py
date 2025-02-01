from magicbot import feedback
from wpilib import DigitalInput, Servo

from ids import DioChannel, PwmChannel


class FeelerComponent:
    def __init__(self):
        self.is_inverted = False

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
