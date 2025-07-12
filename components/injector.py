import math

from magicbot import feedback, tunable, will_reset_to
from rev import SparkMax, SparkMaxConfig
from wpilib import DigitalInput

from ids import DioChannel, SparkId


class InjectorComponent:
    desired_injector_voltage = will_reset_to(0.0)
    should_measure = will_reset_to(False)

    INJECTOR_INJECT_VOLTAGE = tunable(12.0)
    INJECTOR_INTAKE_VOLTAGE = tunable(-2.0)
    INJECTOR_BACKDRIVE_VOLTAGE = tunable(-0.5)
    INJECTOR_MEASURE_SPEED = tunable(200)

    INJECTOR_RPS_TOLERANCE = 0.5
    INJECTOR_MAX_ACCEL = 0.5

    cycle_counter = tunable(0)
    cycle_segment = tunable("intaking")

    def __init__(self) -> None:
        self.algae_limit_switch = DigitalInput(DioChannel.ALGAE_INTAKE_SWITCH)

        self.injector_1 = SparkMax(SparkId.INJECTOR_1, SparkMax.MotorType.kBrushless)
        self.injector_2 = SparkMax(SparkId.INJECTOR_2, SparkMax.MotorType.kBrushless)
        self.injector_1_closed_loop = self.injector_1.getClosedLoopController()
        self.injector_2_closed_loop = self.injector_2.getClosedLoopController()
        injector_config = SparkMaxConfig()

        injector_config.inverted(True)
        injector_config.smartCurrentLimit(20)
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

    def on_enable(self) -> None:
        self.has_seen_algae = False
        self.cycle_counter = 0

    def increment_segment(self) -> None:
        if self.cycle_segment == "intaking":
            self.cycle_segment = "shooting"
        else:
            self.cycle_segment = "intaking"
            self.cycle_counter += 1

    @feedback
    def _algae_limit_switch_pressed(self) -> bool:
        return not self.algae_limit_switch.get()

    @feedback
    def has_algae(self) -> bool:
        return self.has_seen_algae

    def inject(self) -> None:
        self.desired_injector_voltage = self.INJECTOR_INJECT_VOLTAGE
        self.has_seen_algae = False

    def intake(self) -> None:
        self.desired_injector_voltage = self.INJECTOR_INTAKE_VOLTAGE

    def measure_algae(self) -> None:
        self.should_measure = True

    def get_injector_positions(self) -> tuple[float, float]:
        return (
            self.injector_1_encoder.getPosition(),
            self.injector_2_encoder.getPosition(),
        )

    def get_injector_velocities(self) -> tuple[float, float]:
        return (
            self.injector_1_encoder.getVelocity(),
            self.injector_2_encoder.getVelocity(),
        )

    def execute(self) -> None:
        if self._algae_limit_switch_pressed():
            self.has_seen_algae = True

        if math.isclose(self.desired_injector_voltage, 0.0) and self.has_algae():
            self.desired_injector_voltage = self.INJECTOR_BACKDRIVE_VOLTAGE

        if self.should_measure:
            self.injector_1_closed_loop.setReference(
                self.INJECTOR_MEASURE_SPEED,
                SparkMax.ControlType.kVelocity,
            )
            self.injector_2_closed_loop.setReference(
                self.INJECTOR_MEASURE_SPEED,
                SparkMax.ControlType.kVelocity,
            )
        else:
            self.injector_1.setVoltage(self.desired_injector_voltage)
            self.injector_2.setVoltage(self.desired_injector_voltage)
