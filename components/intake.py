import math

import wpilib
from magicbot import feedback, tunable
from phoenix5 import ControlMode, TalonSRX
from rev import (
    SparkMax,
    SparkMaxConfig,
)
from wpilib import DutyCycleEncoder
from wpimath import estimator
from wpimath.controller import (
    ArmFeedforward,
    LinearQuadraticRegulator_2_1,
    PIDController,
)
from wpimath.system import LinearSystemLoop_2_1_1, plant
from wpimath.trajectory import TrapezoidProfile

from ids import DioChannel, SparkId, TalonId
from utilities.rev import configure_through_bore_encoder
from utilities.state_space import single_jointed_arm_system


class IntakeComponent:
    intake_output = tunable(0.9)

    # Offset is measured in the vertical position
    VERTICAL_ENCODER_VALUE = 4.592024
    ARM_ENCODER_OFFSET = VERTICAL_ENCODER_VALUE - math.pi / 2.0
    DEPLOYED_ANGLE_LOWER = 3.391598 - ARM_ENCODER_OFFSET
    DEPLOYED_ANGLE_UPPER = 3.891598 - ARM_ENCODER_OFFSET
    RETRACTED_ANGLE = 4.592024 - ARM_ENCODER_OFFSET
    ARM_MOI = 0.181717788

    gear_ratio = 4.0 * 5.0 * (48.0 / 40.0)

    def __init__(self, intake_mech_root: wpilib.MechanismRoot2d) -> None:
        self.intake_ligament = intake_mech_root.appendLigament(
            "intake", length=0.25, angle=90, color=wpilib.Color8Bit(wpilib.Color.kGreen)
        )

        self.intake_motor = TalonSRX(TalonId.INTAKE)
        self.intake_motor.setInverted(True)

        self.desired_output = 0.0

        self.arm_motor = SparkMax(SparkId.INTAKE_ARM, SparkMax.MotorType.kBrushless)
        self.encoder = DutyCycleEncoder(DioChannel.INTAKE_ENCODER, math.tau, 0)
        configure_through_bore_encoder(self.encoder)
        self.encoder.setInverted(True)

        spark_config = SparkMaxConfig()
        spark_config.inverted(False)
        spark_config.setIdleMode(SparkMaxConfig.IdleMode.kBrake)

        self.motion_profile = TrapezoidProfile(TrapezoidProfile.Constraints(1.0, 1.0))
        self.pid = PIDController(Kp=5.9679, Ki=0, Kd=0.0)

        # CG is at 220mm, 2.9kg
        # https://www.reca.lc/arm?armMass=%7B%22s%22%3A2.9%2C%22u%22%3A%22kg%22%7D&comLength=%7B%22s%22%3A0.22%2C%22u%22%3A%22m%22%7D&currentLimit=%7B%22s%22%3A40%2C%22u%22%3A%22A%22%7D&efficiency=90&endAngle=%7B%22s%22%3A90%2C%22u%22%3A%22deg%22%7D&iterationLimit=10000&motor=%7B%22quantity%22%3A1%2C%22name%22%3A%22NEO%22%7D&ratio=%7B%22magnitude%22%3A24%2C%22ratioType%22%3A%22Reduction%22%7D&startAngle=%7B%22s%22%3A15%2C%22u%22%3A%22deg%22%7D
        self.arm_ff = ArmFeedforward(kS=0.0, kG=0.86, kV=0.47, kA=0.02)
        spark_config.encoder.positionConversionFactor(math.tau * (1 / self.gear_ratio))
        spark_config.encoder.velocityConversionFactor(
            (1 / 60) * math.tau * (1 / self.gear_ratio)
        )

        self.arm_motor.configure(
            spark_config,
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )

        self.motor_encoder = self.arm_motor.getEncoder()

        self.armPlant = single_jointed_arm_system(
            plant.DCMotor.NEO(1), self.ARM_MOI, self.gear_ratio
        )

        """No idea if these are the correct error/trust values"""
        self.observer = estimator.KalmanFilter_2_1_1(
            self.armPlant,
            (
                0.15,
                0.17,
            ),
            (0.005,),
            0.020,
        )

        self.controller = LinearQuadraticRegulator_2_1(
            self.armPlant,
            (
                0.01,
                0.5,
            ),
            (12.0,),
            0.020,
        )

        self.loop = LinearSystemLoop_2_1_1(
            self.armPlant, self.controller, self.observer, 12.0, 0.020
        )

        self.loop.reset([self.position_observation(), self.velocity_observation()])
        self.loop.setNextR([self.position_observation(), self.velocity_observation()])
        self.innovation = self.loop.xhat()

        self.desired_state = TrapezoidProfile.State(
            IntakeComponent.RETRACTED_ANGLE, 0.0
        )
        self.tracked_state = self.desired_state
        self.last_setpoint_update_time = wpilib.Timer.getFPGATimestamp()
        self.initial_state = TrapezoidProfile.State(
            self.position_observation(), self.velocity_observation()
        )

    def intake(self, upper: bool):
        deployed_angle = (
            IntakeComponent.DEPLOYED_ANGLE_UPPER
            if upper
            else IntakeComponent.DEPLOYED_ANGLE_LOWER
        )

        if not math.isclose(self.desired_state.position, deployed_angle, abs_tol=0.1):
            self.desired_state = TrapezoidProfile.State(deployed_angle, 0.0)
            self.last_setpoint_update_time = wpilib.Timer.getFPGATimestamp()
            self.initial_state = TrapezoidProfile.State(
                self.position(), self.velocity()
            )

        self.desired_output = self.intake_output

    def retract(self):
        if not math.isclose(
            self.desired_state.position, self.RETRACTED_ANGLE, abs_tol=0.1
        ):
            self._force_retract()

    @feedback
    def raw_encoder(self) -> float:
        return self.encoder.get()

    @feedback
    def position_degrees(self) -> float:
        return math.degrees(self.position())

    def position(self):
        return self.loop.xhat(0)

    def position_observation(self) -> float:
        return self.encoder.get() - IntakeComponent.ARM_ENCODER_OFFSET

    @feedback
    def position_observation_degrees(self) -> float:
        return math.degrees(self.position_observation())

    def velocity(self) -> float:
        return self.loop.xhat(1)

    def velocity_observation(self) -> float:
        return self.motor_encoder.getVelocity()

    @feedback
    def current_input(self):
        return self.loop.U()

    def correct_and_predict(self) -> None:
        # this is still predicted from the last loop
        predicted = self.loop.xhat()
        self.loop.correct([self.position_observation()])
        corrected = self.loop.xhat()

        self.innovation = corrected - predicted
        self.loop.predict(0.020)

    @feedback
    def desired(self):
        return (self.desired_state.position, self.desired_state.velocity)

    @feedback
    def tracked(self):
        return (self.tracked_state.position, self.tracked_state.velocity)

    @feedback
    def initial(self):
        return (self.initial_state.position, self.initial_state.velocity)

    @feedback
    def profile_time(self):
        return wpilib.Timer.getFPGATimestamp() - self.last_setpoint_update_time

    @feedback
    def filter_error(self):
        return self.innovation

    def _force_retract(self):
        self.desired_state = TrapezoidProfile.State(
            IntakeComponent.RETRACTED_ANGLE, 0.0
        )
        self.initial_state = TrapezoidProfile.State(self.position(), self.velocity())
        self.last_setpoint_update_time = wpilib.Timer.getFPGATimestamp()

    def on_enable(self) -> None:
        self._force_retract()
        self.desired_output = 0.0

    def execute(self) -> None:
        self.intake_motor.set(ControlMode.PercentOutput, self.desired_output)

        self.desired_output = 0.0

        self.tracked_state = self.motion_profile.calculate(
            self.profile_time(),
            self.initial_state,
            self.desired_state,
        )

        self.loop.setNextR([self.desired_state.position, self.desired_state.velocity])

        self.correct_and_predict()

        if not math.isclose(
            self.desired_state.position, self.position(), abs_tol=math.radians(5)
        ):
            self.arm_motor.setVoltage(self.loop.U(0))
        else:
            self.arm_motor.setVoltage(0.0)

    def periodic(self) -> None:
        self.intake_ligament.setAngle(math.degrees(self.position()))
