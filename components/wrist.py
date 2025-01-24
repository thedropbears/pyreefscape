from magicbot import feedback, tunable
from rev import ClosedLoopSlot, SparkMax, SparkMaxConfig
from wpilib import DigitalInput

from ids import DioChannel, SparkId


class WristComponent:
    maximum_angle = 73.0
    angle_change_rate_while_zeroing = tunable(3.0)
    wrist_gear_ratio = 20 * 66 / 26
    desired_angle = 0.0

    def __init__(self):
        self.switch = DigitalInput(DioChannel.WRIST_LIMIT_SWITCH)

        self.wrist = SparkMax(SparkId.WRIST, SparkMax.MotorType.kBrushless)

        self.wrist_controller = self.wrist.getClosedLoopController()

        wrist_config = SparkMaxConfig()
        wrist_config.inverted(False)
        wrist_config.setIdleMode(SparkMaxConfig.IdleMode.kBrake)
        wrist_config.closedLoop.P(6.0 / self.maximum_angle, ClosedLoopSlot.kSlot0)
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
        if not self.wrist_at_top_limit():
            self.desired_angle += self.angle_change_rate_while_zeroing
        else:
            self.encoder.setPosition(self.maximum_angle)
            self.desired_angle = self.maximum_angle

    @feedback
    def wrist_at_top_limit(self) -> bool:
        return not self.switch.get()

    @feedback
    def get_encoder(self) -> float:
        return self.encoder.getPosition()

    @feedback
    def at_setpoint(self) -> bool:
        return abs(self.desired_angle - self.get_encoder()) < WristComponent.TOLERANCE

    def tilt_to(self, pos: float) -> None:
        self.desired_angle = pos

    def execute(self) -> None:
        self.wrist_controller.setReference(
            self.desired_angle, SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0
        )
