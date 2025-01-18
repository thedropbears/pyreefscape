from magicbot import tunable
from phoenix6.configs import MotorOutputConfigs
from phoenix6.controls import Follower, VoltageOut
from phoenix6.hardware import TalonFX
from phoenix6.signals import NeutralModeValue
from rev import SparkMax, SparkMaxConfig
from wpilib import DigitalInput

from ids import SparkId


class ManipulatorComponent:
    switch = DigitalInput(1)  # TODO Check if this works
    tilter_gear_ratio = 20 * 66 / 26
    tilt_speed = tunable(2.0)  # TODO change if necessary
    tilter_angle = 0
    flywheel_shoot_speed = tunable(-6.0)
    flywheel_intake_speed = tunable(2.0)
    injector_inject_speed = tunable(-6.0)

    def __init__(self) -> None:
        # Injector motors

        self.injector_1 = SparkMax(5, SparkMax.MotorType.kBrushless)
        self.injector_2 = SparkMax(4, SparkMax.MotorType.kBrushless)
        injector_config = SparkMaxConfig()

        injector_config.inverted(False)
        self.injector_1.configure(
            injector_config,
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )
        injector_config.follow(5, True)
        self.injector_2.configure(
            injector_config,
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )

        # Flywheel motors

        self.flywheel_1 = TalonFX(9)
        self.flywheel_2 = TalonFX(10)
        flywheel_1_config = self.flywheel_1.configurator
        flywheel_2_config = self.flywheel_2.configurator
        flywheel_config = MotorOutputConfigs()
        flywheel_config.neutral_mode = NeutralModeValue.COAST

        flywheel_1_config.apply(flywheel_config)
        flywheel_2_config.apply(flywheel_config)
        self.desired_flywheel_speed = 0.0
        self.desired_injector_speed = 0.25

        # Tilter motors

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

    def zero_tilter(self) -> None:
        if not self.switch.get():
            self.tilter_angle += 1  # TODO check if this is a good way to do this
            pass
        else:
            self.unzeroed_tilter_angle = 73

    def tilt_to(self, pos) -> None:
        self.tilter_angle = pos

    def spin_flywheels(self) -> None:
        self.desired_flywheel_speed = self.flywheel_shoot_speed

    def inject(self) -> None:
        self.desired_injector_speed = self.injector_inject_speed

    def intake(self) -> None:
        self.desired_flywheel_speed = self.flywheel_intake_speed

    def execute(self) -> None:
        self.injector_1.setVoltage(self.desired_injector_speed)

        self.flywheel_1.set_control(VoltageOut(self.desired_flywheel_speed))
        self.flywheel_2.set_control(Follower(9, True))

        self.tilter_controller_1.setReference(
            self.tilter_angle / self.tilter_gear_ratio,  # TODO check if correct
            SparkMax.ControlType.kPosition,
        )

        self.tilter_controller_2.setReference(
            self.tilter_angle / self.tilter_gear_ratio,  # TODO check if correct
            SparkMax.ControlType.kPosition,
        )

        self.desired_flywheel_speed = 0.0
        self.desired_injector_speed = 0.25
