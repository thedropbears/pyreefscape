from magicbot import feedback
from rev import ClosedLoopSlot, SparkMax, SparkMaxConfig
from wpilib import DigitalInput

from ids import SparkId


class TilterComponent:
    tilter_gear_ratio = 20 * 66 / 26
    desired_angle = 0

    def __init__(self):
        self.switch = DigitalInput(3)

        self.tilter_1 = SparkMax(SparkId.TILTER_LEFT, SparkMax.MotorType.kBrushless)
        self.tilter_2 = SparkMax(SparkId.TILTER_RIGHT, SparkMax.MotorType.kBrushless)

        self.tilter_controller_1 = self.tilter_1.getClosedLoopController()
        self.tilter_controller_2 = self.tilter_2.getClosedLoopController()

        tilter_1_config = SparkMaxConfig()
        tilter_2_config = SparkMaxConfig()

        tilter_1_config.inverted(
            True
        )  # THIS MUST BE OPPOSITE TO tilter_2 OR MOTORS WILL STALL
        tilter_2_config.inverted(
            False
        )  # THIS MUST BE OPPOSITE TO tilter_1 OR MOTORS WILL STALL

        tilter_1_config.setIdleMode(
            SparkMaxConfig.IdleMode.kBrake
        )  # set to coast for encoder testing
        tilter_2_config.setIdleMode(
            SparkMaxConfig.IdleMode.kBrake
        )  # set to coast for encoder testing

        tilter_1_config.closedLoop.P(2.0 / 73 / 100, ClosedLoopSlot.kSlot0)
        tilter_2_config.closedLoop.P(2.0 / 73 / 100, ClosedLoopSlot.kSlot0)

        tilter_1_config.closedLoop.D(0.0, ClosedLoopSlot.kSlot0)
        tilter_2_config.closedLoop.D(0.0, ClosedLoopSlot.kSlot0)

        self.tilter_1.configure(
            tilter_1_config,
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )

        self.tilter_2.configure(
            tilter_2_config,
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )

        self.encoder_1 = self.tilter_1.getEncoder()
        self.encoder_2 = self.tilter_2.getEncoder()

        self.encoder_1.setPosition(0)
        self.encoder_2.setPosition(0)

    def zero_tilter(self) -> None:
        if not self.tilter_at_top_limit():
            self.desired_angle += 1
        else:
            self.encoder_1.setPosition(73 * self.tilter_gear_ratio)
            self.encoder_2.setPosition(73 * self.tilter_gear_ratio)
            self.desired_angle = 73

    @feedback
    def tilter_at_top_limit(self) -> bool:
        return not self.switch.get()

    @feedback
    def get_encoder_1(self) -> float:
        return self.encoder_1.getPosition()

    @feedback
    def get_encoder_2(self) -> float:
        return self.encoder_2.getPosition()

    def tilt_to(self, pos) -> None:
        self.desired_angle = pos

    def execute(self) -> None:
        self.tilter_controller_1.setReference(
            self.desired_angle * self.tilter_gear_ratio,
            SparkMax.ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
        )

        self.tilter_controller_2.setReference(
            self.desired_angle * self.tilter_gear_ratio,
            SparkMax.ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
        )
