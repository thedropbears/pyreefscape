import math

from magicbot import feedback, tunable
from rev import ClosedLoopSlot, SparkMax, SparkMaxConfig
from wpilib import DigitalInput

from ids import DioChannel, SparkId
from utilities.functions import clamp


class WristComponent:
    MAXIMUM_DEPRESSION = -23.0
    MAXIMUM_ELEVATION = 90.0
    NEUTRAL_ANGLE = 0.0

    angle_change_rate_while_zeroing = tunable(0.05)
    wrist_gear_ratio = (150.0 / 15) * 20
    desired_angle = 0.0
    TOLERANCE = 3.0

    def __init__(self):
        self.switch = DigitalInput(DioChannel.WRIST_LIMIT_SWITCH)

        self.wrist = SparkMax(SparkId.WRIST, SparkMax.MotorType.kBrushless)

        self.wrist_controller = self.wrist.getClosedLoopController()

        wrist_config = SparkMaxConfig()
        wrist_config.inverted(True)
        wrist_config.setIdleMode(SparkMaxConfig.IdleMode.kCoast)
        wrist_config.closedLoop.P(
            1.0 / (self.MAXIMUM_ELEVATION - self.MAXIMUM_DEPRESSION),
            ClosedLoopSlot.kSlot0,
        )
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
            self.tilt_to(self.desired_angle - self.angle_change_rate_while_zeroing)
            if math.isclose(
                self.desired_angle, self.MAXIMUM_DEPRESSION, abs_tol=1.0
            ) and math.isclose(
                self.inclination(), self.MAXIMUM_DEPRESSION, abs_tol=1.0
            ):
                self.encoder.setPosition(self.inclination() + 1)

    @feedback
    def wrist_at_bottom_limit(self) -> bool:
        return not self.switch.get()

    @feedback
    def inclination(self) -> float:
        return self.encoder.getPosition()

    @feedback
    def at_setpoint(self) -> bool:
        return abs(self.desired_angle - self.inclination()) < WristComponent.TOLERANCE

    def tilt_to(self, pos: float) -> None:
        self.desired_angle = clamp(pos, self.MAXIMUM_DEPRESSION, self.MAXIMUM_ELEVATION)

    def go_to_neutral(self) -> None:
        self.desired_angle = WristComponent.NEUTRAL_ANGLE

    def execute(self) -> None:
        if self.wrist_at_bottom_limit():
            self.encoder.setPosition(self.MAXIMUM_DEPRESSION)

        self.wrist_controller.setReference(
            self.desired_angle, SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0
        )
