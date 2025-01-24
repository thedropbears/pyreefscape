import math

from magicbot import feedback, tunable
from rev import ClosedLoopSlot, SparkMax, SparkMaxConfig
from wpilib import DigitalInput

from ids import DioChannel, SparkId
from utilities.functions import clamp


class WristComponent:
    maximum_angle = 80.0
    minimum_angle = -23.0
    angle_tolerance = 3.0

    angle_change_rate_while_zeroing = tunable(1.0)
    wrist_gear_ratio = 20 * 150 / 15  # 200 / 1
    desired_angle = 0.0

    def __init__(self):
        self.switch = DigitalInput(DioChannel.WRIST_LIMIT_SWITCH)

        self.wrist = SparkMax(SparkId.WRIST, SparkMax.MotorType.kBrushless)

        self.wrist_controller = self.wrist.getClosedLoopController()

        wrist_config = SparkMaxConfig()
        wrist_config.inverted(True)
        wrist_config.setIdleMode(SparkMaxConfig.IdleMode.kCoast)
        wrist_config.closedLoop.P(3.0 / self.maximum_angle, ClosedLoopSlot.kSlot0)
        wrist_config.closedLoop.D(0.1, ClosedLoopSlot.kSlot0)

        wrist_config.encoder.positionConversionFactor(360 * (1 / self.wrist_gear_ratio))

        self.wrist.configure(
            wrist_config,
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )

        self.encoder = self.wrist.getEncoder()

        self.encoder.setPosition(0.0)

    def zero_wrist(self) -> None:
        if not self.wrist_at_bottom_limit():
            self.desired_angle -= self.angle_change_rate_while_zeroing
            self.desired_angle = clamp(
                self.desired_angle, self.minimum_angle, self.maximum_angle
            )

        if self.wrist_at_bottom_limit():
            self.encoder.setPosition(self.minimum_angle)
            self.desired_angle = self.minimum_angle

    @feedback
    def wrist_at_bottom_limit(self) -> bool:
        return not self.switch.get()

    @feedback
    def get_encoder(self) -> float:
        return self.encoder.getPosition()

    def tilt_to(self, pos) -> None:
        self.desired_angle = clamp(pos, self.minimum_angle, self.maximum_angle)

    def at_desired_angle(self) -> bool:
        return math.isclose(
            self.desired_angle, self.get_encoder(), abs_tol=self.angle_tolerance
        )

    def execute(self) -> None:
        if self.wrist_at_bottom_limit():
            self.encoder.setPosition(self.minimum_angle)

        self.desired_angle = clamp(
            self.desired_angle, self.minimum_angle, self.maximum_angle
        )
        self.wrist_controller.setReference(
            self.desired_angle, SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0
        )
