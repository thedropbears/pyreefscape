import numpy as np
from magicbot import feedback
from wpilib import DigitalInput, Servo

from ids import DioChannel, PwmChannel


class FeelerComponent:
    # Angles must be in increasing order
    ALGAE_SERVO_ANGLES = [120, 130, 140]  # TODO Measure these
    ALGAE_SIZES = [17.0, 16.5, 16.0]

    def __init__(self):
        self.is_inverted = False

        self.current_angle = 0.0
        self.desired_angle = 90.0

        self.algae_angle = 0.0
        self.algae_size = 0.0

        self.limit_switch = DigitalInput(DioChannel.FEELER_LIMIT_SWITCH)
        self.servo = Servo(PwmChannel.FEELER_SERVO)

    def record_size(self) -> None:
        self.algae_angle = self.current_angle
        # Convert servo angle to algae size in inches
        self.algae_size = float(
            np.interp(
                self.algae_angle,
                self.ALGAE_SERVO_ANGLES,
                self.ALGAE_SIZES,
            )
        )

    @feedback
    def size(self) -> float:
        return self.algae_size

    @feedback
    def touch_angle(self) -> float:
        return self.algae_angle

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
