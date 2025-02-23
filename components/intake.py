import math
import time

import wpilib
from magicbot import feedback, tunable
from phoenix5 import ControlMode, TalonSRX
from rev import (
    SparkMax,
    SparkMaxConfig,
)
from wpilib import DutyCycleEncoder
from wpimath.controller import ArmFeedforward, PIDController
from wpimath.trajectory import TrapezoidProfile

from ids import DioChannel, SparkId, TalonId
from utilities.rev import configure_through_bore_encoder


class IntakeComponent:
    intake_output = tunable(0.9)

    # Offset is measured in the vertical position
    VERTICAL_ENCODER_VALUE = 2.365140
    ARM_ENCODER_OFFSET = VERTICAL_ENCODER_VALUE - math.pi / 2.0
    # magic offset the deployed angle by 4 degrees to limit damage inflicted on mechanism
    DEPLOYED_ANGLE = 1.194348 - ARM_ENCODER_OFFSET + math.radians(4)
    RETRACTED_ANGLE = 2.365140 - ARM_ENCODER_OFFSET

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

        self.motion_profile = TrapezoidProfile(TrapezoidProfile.Constraints(0.7, 0.5))
        self.pid = PIDController(Kp=2.1741, Ki=0, Kd=0.18444 / 8.0)

        # CG is at 220mm, 2.7kg
        # https://www.reca.lc/arm?armMass=%7B%22s%22%3A2.7%2C%22u%22%3A%22kg%22%7D&comLength=%7B%22s%22%3A0.22%2C%22u%22%3A%22m%22%7D&currentLimit=%7B%22s%22%3A40%2C%22u%22%3A%22A%22%7D&efficiency=90&endAngle=%7B%22s%22%3A90%2C%22u%22%3A%22deg%22%7D&iterationLimit=10000&motor=%7B%22quantity%22%3A1%2C%22name%22%3A%22NEO%22%7D&ratio=%7B%22magnitude%22%3A48%2C%22ratioType%22%3A%22Reduction%22%7D&startAngle=%7B%22s%22%3A30%2C%22u%22%3A%22deg%22%7D
        self.arm_ff = ArmFeedforward(kS=0.0, kG=0.4, kV=0.94, kA=0.01)

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

        self.desired_angle = IntakeComponent.RETRACTED_ANGLE

        self.last_setpoint_update_time = time.monotonic()

    def intake(self):
        if not math.isclose(
            self.desired_angle, IntakeComponent.DEPLOYED_ANGLE, abs_tol=0.1
        ):
            self.desired_angle = IntakeComponent.DEPLOYED_ANGLE
            self.last_setpoint_update_time = time.monotonic()

        self.desired_output = self.intake_output

    def retract(self):
        if not math.isclose(self.desired_angle, self.RETRACTED_ANGLE, abs_tol=0.1):
            self.desired_angle = IntakeComponent.RETRACTED_ANGLE
            self.last_setpoint_update_time = time.monotonic()

    @feedback
    def raw_encoder(self) -> float:
        return self.encoder.get()

    def position(self):
        return self.encoder.get() - IntakeComponent.ARM_ENCODER_OFFSET

    def velocity(self) -> float:
        return self.motor_encoder.getVelocity()

    def on_enable(self) -> None:
        self.last_setpoint_update_time = time.monotonic()
        self.desired_angle = self.position()
        self.desired_output = 0.0

    def execute(self) -> None:
        self.intake_motor.set(ControlMode.PercentOutput, self.desired_output)

        self.desired_output = 0.0

        desired_state = self.motion_profile.calculate(
            time.monotonic() - self.last_setpoint_update_time,
            TrapezoidProfile.State(self.position(), self.velocity()),
            TrapezoidProfile.State(self.desired_angle, 0.0),
        )
        ff = self.arm_ff.calculate(desired_state.position, desired_state.velocity)

        if not math.isclose(
            self.desired_angle, self.position(), abs_tol=math.radians(5)
        ):
            self.arm_motor.setVoltage(
                self.pid.calculate(self.position(), desired_state.position) + ff
            )
        else:
            self.arm_motor.setVoltage(0.0)

    def periodic(self) -> None:
        self.intake_ligament.setAngle(math.degrees(self.position()))
