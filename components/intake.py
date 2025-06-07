import math

import numpy as np
import wpilib
from magicbot import feedback, tunable
from phoenix5 import ControlMode, TalonSRX
from rev import (
    SparkMax,
    SparkMaxConfig,
)
from wpilib import DutyCycleEncoder
from wpimath.controller import (
    LinearQuadraticRegulator_2_1,
)
from wpimath.estimator import KalmanFilter_2_1_2
from wpimath.system import LinearSystemLoop_2_1_2
from wpimath.system.plant import DCMotor, LinearSystemId
from wpimath.trajectory import TrapezoidProfile

from ids import DioChannel, SparkId, TalonId
from utilities.rev import configure_through_bore_encoder


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

        plant = LinearSystemId.singleJointedArmSystem(
            DCMotor.NEO(1), self.ARM_MOI, self.gear_ratio
        )

        self.observer = KalmanFilter_2_1_2(
            plant,
            (
                0.15,
                0.17,
            ),
            (0.005, 0.009),
            0.020,
        )

        self.controller = LinearQuadraticRegulator_2_1(
            plant,
            (
                0.005,
                0.02,
            ),
            (2.0,),
            0.020,
        )

        self.loop = LinearSystemLoop_2_1_2(
            plant, self.controller, self.observer, 12.0, 0.020
        )

        self.loop.reset([self.position_observation(), self.velocity_observation()])
        self.loop.setNextR([self.position_observation(), self.velocity_observation()])
        self.innovation = np.zeros(self.loop.xhat().shape)

        self.motion_profile = TrapezoidProfile(TrapezoidProfile.Constraints(8.0, 10.0))
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
        if wpilib.DriverStation.isDisabled():
            self.observer.correct(
                [0.0], [self.position_observation(), self.velocity_observation()]
            )

            self.observer.predict([0.0], 0.02)
        else:
            self.loop.correct(
                [self.position_observation(), self.velocity_observation()]
            )

            self.loop.predict(0.020)

        # constrain ourselves if we are going to do damage
        if (
            self.position() > IntakeComponent.RETRACTED_ANGLE
            or self.position() < IntakeComponent.DEPLOYED_ANGLE_LOWER
        ):
            self.loop.reset([self.position_observation(), self.velocity_observation()])

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

        self.loop.setNextR([self.tracked_state.position, self.tracked_state.velocity])

        self.correct_and_predict()

        if not math.isclose(
            self.desired_state.position, self.position(), abs_tol=math.radians(5)
        ):
            self.arm_motor.setVoltage(self.loop.U(0))
        else:
            self.arm_motor.setVoltage(0.0)

    def periodic(self) -> None:
        self.intake_ligament.setAngle(math.degrees(self.position()))
