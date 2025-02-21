import math

from magicbot import feedback, tunable
from rev import SparkMax, SparkMaxConfig
from wpilib import DigitalInput

from ids import DioChannel, SparkId


class InjectorComponent:
    INJECTOR_INJECT_SPEED = tunable(12.0)
    INJECTOR_INTAKE_SPEED = tunable(200)
    INJECTOR_BACKDRIVE_SPEED = tunable(-1.0)
    INJECTOR_MEASURE_SPEED = tunable(75)

    INJECTOR_RPS_TOLERANCE = 0.5
    INJECTOR_MAX_ACCEL = 0.5

    def __init__(self) -> None:
        self.algae_limit_switch = DigitalInput(DioChannel.ALGAE_INTAKE_SWITCH)

        self.injector_1 = SparkMax(SparkId.INJECTOR_1, SparkMax.MotorType.kBrushless)
        self.injector_2 = SparkMax(SparkId.INJECTOR_2, SparkMax.MotorType.kBrushless)
        self.injector_1_closed_loop = self.injector_1.getClosedLoopController()
        self.injector_2_closed_loop = self.injector_2.getClosedLoopController()
        injector_config = SparkMaxConfig()

        injector_config.inverted(True)
        injector_config.closedLoop.maxMotion.maxAcceleration(
            self.INJECTOR_MAX_ACCEL
        ).allowedClosedLoopError(self.INJECTOR_RPS_TOLERANCE)
        injector_config.closedLoop.velocityFF(1 / 917)
        # PID Values Need To Be Implemented
        injector_config.closedLoop.P(0.0001).I(0.0).D(0.0)
        self.injector_1.configure(
            injector_config,
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )
        injector_config.inverted(False)
        self.injector_2.configure(
            injector_config,
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )

        self.injector_1_encoder = self.injector_1.getEncoder()
        self.injector_2_encoder = self.injector_2.getEncoder()

        self.has_seen_algae: bool = False

        self.desired_injector_speed = 0.0

    @feedback
    def has_algae(self) -> bool:
        return not self.algae_limit_switch.get()

    @feedback
    def should_be_holding_algae(self) -> bool:
        return self.has_seen_algae

    def inject(self) -> None:
        self.desired_injector_speed = self.INJECTOR_INJECT_SPEED
        self.has_seen_algae = False

    def intake(self) -> None:
        self.desired_injector_speed = self.INJECTOR_INTAKE_SPEED

    def measure_algae(self) -> None:
        self.desired_injector_speed = self.INJECTOR_MEASURE_SPEED

    def get_injector_positions(self) -> tuple[float, float]:
        return (
            self.injector_1_encoder.getPosition(),
            self.injector_2_encoder.getPosition(),
        )

    def execute(self) -> None:
        if self.has_algae():
            self.has_seen_algae = True

        if math.isclose(self.desired_injector_speed, 0.0) and self.has_seen_algae:
            self.injector_1.setVoltage(self.INJECTOR_BACKDRIVE_SPEED)
            self.injector_2.setVoltage(self.INJECTOR_BACKDRIVE_SPEED)
        else:
            self.injector_1_closed_loop.setReference(
                self.desired_injector_speed,
                SparkMax.ControlType.kVelocity,
            )
            self.injector_2_closed_loop.setReference(
                self.desired_injector_speed,
                SparkMax.ControlType.kVelocity,
            )

        self.desired_injector_speed = 0.0
