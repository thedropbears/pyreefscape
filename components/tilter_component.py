from magicbot import tunable
from rev import SparkMax, SparkMaxConfig
from wpilib import DigitalInput

from ids import SparkId


class TilterComponent:
    switch = DigitalInput(1)
    tilter_gear_ratio = 20 * 66 / 26
    tilt_speed = tunable(2.0)
    desired_angle = 0

    def __init__(self):
        self.tilter_1 = SparkMax(SparkId.TILTER_LEFT, SparkMax.MotorType.kBrushless)
        self.tilter_2 = SparkMax(SparkId.TILTER_RIGHT, SparkMax.MotorType.kBrushless)

        self.tilter_controller_1 = self.tilter_1.getClosedLoopController()
        self.tilter_controller_2 = self.tilter_2.getClosedLoopController()

        tilter_1_config = SparkMaxConfig()
        tilter_1_config.inverted(
            False
        )  # THIS MUST BE OPPOSITE TO tilter_2 OR MOTORS WILL STALL
        tilter_1_config.setIdleMode(SparkMaxConfig.IdleMode.kBrake)

        tilter_1_config.closedLoop.P(0.0)  # TODO write correct values here
        tilter_1_config.closedLoop.I(0.0)  # TODO write correct values here
        tilter_1_config.closedLoop.D(0.0)  # TODO write correct values here
        tilter_1_config.closedLoop.outputRange(
            -1.0, 1.0
        )  # TODO write correct values here

        tilter_2_config = tilter_1_config
        tilter_2_config.inverted(
            True
        )  # THIS MUST BE OPPOSITE TO tilter_1 OR MOTORS WILL STALL

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

    def zero_tilter(self) -> None:
        if not self.switch.get():
            self.desired_angle += 1  # TODO check if this is a good way to do this
        else:
            self.encoder_1.setPosition(
                73 / self.tilter_gear_ratio
            )  # TODO check if correct :)
            self.desired_angle = 73

    def tilt_to(self, pos) -> None:
        self.desired_angle = pos

    def execute(self) -> None:
        self.tilter_controller_1.setReference(
            self.desired_angle / self.tilter_gear_ratio,  # TODO check if correct
            SparkMax.ControlType.kPosition,
        )

        self.tilter_controller_2.setReference(
            self.desired_angle / self.tilter_gear_ratio,  # TODO check if correct
            SparkMax.ControlType.kPosition,
        )
