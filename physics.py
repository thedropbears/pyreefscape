from __future__ import annotations

import math
import statistics
import typing
from collections.abc import Callable

import phoenix6
import phoenix6.unmanaged
import rev
import wpilib
from photonlibpy.simulation import PhotonCameraSim, SimCameraProperties, VisionSystemSim
from pyfrc.physics.core import PhysicsInterface
from wpilib.simulation import (
    DCMotorSim,
    DIOSim,
    DutyCycleEncoderSim,
    PWMSim,
    SingleJointedArmSim,
)
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.system.plant import DCMotor, LinearSystemId
from wpimath.units import kilogram_square_meters

from components.chassis import SwerveModule
from components.intake import IntakeComponent
from components.wrist import WristComponent
from utilities import game
from utilities.functions import constrain_angle

if typing.TYPE_CHECKING:
    from robot import MyRobot


class RollingBuffer:
    def __init__(self, max_length: int):
        self.max_length = max_length

        self.buffer_: list[float] = []

    def average(self) -> float:
        return statistics.fmean(self.buffer_) if self.buffer_ else 0.0

    def add_sample(self, sample: float):
        self.buffer_.append(sample)

        if len(self.buffer_) > self.max_length:
            self.buffer_.pop(0)


class SimpleTalonFXMotorSim:
    def __init__(
        self, motor: phoenix6.hardware.TalonFX, units_per_rev: float, kV: float
    ) -> None:
        self.sim_state = motor.sim_state
        self.sim_state.set_supply_voltage(12.0)
        self.kV = kV  # volt seconds per unit
        self.units_per_rev = units_per_rev
        self.voltage_buffer = RollingBuffer(10)

    def update(self, dt: float) -> None:
        voltage = self.sim_state.motor_voltage

        self.voltage_buffer.add_sample(voltage)

        if math.isclose(self.voltage_buffer.average(), 0.0, abs_tol=0.1):
            voltage = 0.0

        velocity = voltage / self.kV  # units per second
        velocity_rps = velocity * self.units_per_rev
        self.sim_state.set_rotor_velocity(velocity_rps)
        self.sim_state.add_rotor_position(velocity_rps * dt)


class TalonFXMotorSim:
    def __init__(
        self,
        # DCMotor gearbox factory, e.g. DCMotor.falcon500
        gearbox_motor: Callable[[int], DCMotor],
        *motors: phoenix6.hardware.TalonFX,
        # Reduction between motor and encoder readings, as output over input.
        # If the mechanism spins slower than the motor, this number should be greater than one.
        gearing: float,
        moi: kilogram_square_meters,
    ):
        gearbox = gearbox_motor(len(motors))
        self.plant = LinearSystemId.DCMotorSystem(gearbox, moi, gearing)
        self.gearing = gearing
        self.sim_states = [motor.sim_state for motor in motors]
        for sim_state in self.sim_states:
            sim_state.set_supply_voltage(12.0)
        self.motor_sim = DCMotorSim(self.plant, gearbox)

    def update(self, dt: float) -> None:
        voltage = self.sim_states[0].motor_voltage
        self.motor_sim.setInputVoltage(voltage)
        self.motor_sim.update(dt)
        motor_rev_per_mechanism_rad = self.gearing / math.tau
        for sim_state in self.sim_states:
            sim_state.set_raw_rotor_position(
                self.motor_sim.getAngularPosition() * motor_rev_per_mechanism_rad
            )
            sim_state.set_rotor_velocity(
                self.motor_sim.getAngularVelocity() * motor_rev_per_mechanism_rad
            )


class SparkMotorSim:
    """Simulates Spark MAX/Flex with brushless motors."""

    def __init__(
        self,
        # DCMotor gearbox factory, e.g. DCMotor.NEO
        gearbox_motor: Callable[[int], DCMotor],
        *motors: rev.SparkBase,
        gearing: float,
        moi: kilogram_square_meters,
        velocity_units_per_rad_per_sec: float = 60 / math.tau,
    ) -> None:
        gearbox = gearbox_motor(len(motors))
        self.plant = LinearSystemId.DCMotorSystem(gearbox, moi, gearing)
        self.motors = [rev.SparkSim(motor, gearbox) for motor in motors]
        self.motor_sim = DCMotorSim(self.plant, gearbox)
        self.velocity_conversion_factor = velocity_units_per_rad_per_sec

    def update(self, dt: float) -> None:
        vbus = self.motors[0].getBusVoltage()
        self.motor_sim.setInputVoltage(self.motors[0].getAppliedOutput() * vbus)
        self.motor_sim.update(dt)
        velocity = self.motor_sim.getAngularVelocity() * self.velocity_conversion_factor
        for motor in self.motors:
            motor.iterate(velocity, vbus, dt)


class SparkArmSim:
    def __init__(self, mech_sim: SingleJointedArmSim, motor_sim: rev.SparkSim) -> None:
        self.mech_sim = mech_sim
        self.motor_sim = motor_sim
        self.motor_encoder_sim = self.motor_sim.getRelativeEncoderSim()

    def update(self, dt: float) -> None:
        vbus = self.motor_sim.getBusVoltage()
        self.mech_sim.setInputVoltage(self.motor_sim.getAppliedOutput() * vbus)
        self.mech_sim.update(dt)
        self.motor_sim.iterate(self.mech_sim.getVelocity(), vbus, dt)
        self.motor_encoder_sim.iterate(self.mech_sim.getVelocity(), dt)


# class ServoEncoderSim:
#     def __init__(self, pwm, encoder):
#         self.pwm_sim = PWMSim(pwm)
#         self.encoder_sim = DutyCycleEncoderSim(encoder)

#     def update(self):
#         command = self.pwm_sim.getPosition()


class PhysicsEngine:
    def __init__(self, physics_controller: PhysicsInterface, robot: MyRobot):
        self.physics_controller = physics_controller

        self.kinematics: SwerveDrive4Kinematics = robot.chassis.kinematics
        self.swerve_modules: tuple[
            SwerveModule, SwerveModule, SwerveModule, SwerveModule
        ] = robot.chassis.modules

        # Motors
        self.wheels = [
            SimpleTalonFXMotorSim(
                module.drive,
                units_per_rev=1 / robot.chassis.drive_motor_rev_to_meters,
                kV=2.7,
            )
            for module in robot.chassis.modules
        ]
        self.steer = [
            TalonFXMotorSim(
                DCMotor.krakenX60,
                module.steer,
                gearing=1 / robot.chassis.swerve_config.steer_ratio,
                # measured from MKCad CAD
                moi=0.0009972,
            )
            for module in robot.chassis.modules
        ]
        self.flywheels = [
            TalonFXMotorSim(
                DCMotor.falcon500,
                robot.shooter_component.top_flywheel,
                gearing=1 / 1,
                moi=0.00105679992,
            ),
            TalonFXMotorSim(
                DCMotor.falcon500,
                robot.shooter_component.bottom_flywheel,
                gearing=1 / 1,
                moi=0.00156014845,
            ),
        ]

        # Intake arm simulation
        intake_arm_gearbox = DCMotor.NEO(1)
        self.intake_arm_motor = rev.SparkMaxSim(
            robot.intake_component.arm_motor, intake_arm_gearbox
        )
        self.intake_arm_encoder_sim = DutyCycleEncoderSim(
            robot.intake_component.encoder
        )
        self.intake_arm = SparkArmSim(
            SingleJointedArmSim(
                intake_arm_gearbox,
                IntakeComponent.gear_ratio,
                moi=IntakeComponent.ARM_MOI,
                armLength=0.22,
                minAngle=IntakeComponent.DEPLOYED_ANGLE_LOWER,
                maxAngle=IntakeComponent.RETRACTED_ANGLE,
                simulateGravity=True,
                startingAngle=IntakeComponent.RETRACTED_ANGLE,
            ),
            self.intake_arm_motor,
        )

        wrist_gearbox = DCMotor.NEO(1)
        wrist_motor = rev.SparkMaxSim(robot.wrist.motor, wrist_gearbox)
        self.wrist_encoder_sim = DutyCycleEncoderSim(robot.wrist.wrist_encoder)

        wrist_sim = SingleJointedArmSim(
            wrist_gearbox,
            WristComponent.wrist_gear_ratio,
            moi=0.295209215,
            armLength=0.5,
            minAngle=WristComponent.MAXIMUM_DEPRESSION,
            maxAngle=WristComponent.MAXIMUM_ELEVATION,
            simulateGravity=True,
            startingAngle=WristComponent.MAXIMUM_DEPRESSION,
        )
        self.wrist = SparkArmSim(wrist_sim, wrist_motor)

        self.imu = robot.chassis.imu.sim_state

        self.injector = SparkMotorSim(
            DCMotor.NEO550,
            robot.injector_component.injector_1,
            robot.injector_component.injector_2,
            gearing=1 / 1,
            moi=0.000105679992,  # TODO: measure
        )

        self.vision_sim = VisionSystemSim("main")
        self.vision_sim.addAprilTags(game.apriltag_layout)
        properties = SimCameraProperties.OV9281_1280_720()
        self.starboard_camera = PhotonCameraSim(
            robot.starboard_vision.camera, properties
        )
        self.port_camera = PhotonCameraSim(robot.port_vision.camera, properties)
        self.starboard_camera.setMaxSightRange(5.0)
        self.starboard_visual_localiser = robot.starboard_vision
        self.vision_sim.addCamera(
            self.starboard_camera,
            self.starboard_visual_localiser.robot_to_camera(
                wpilib.Timer.getFPGATimestamp()
            ),
        )
        self.port_camera.setMaxSightRange(5.0)
        self.port_visual_localiser = robot.port_vision
        self.vision_sim.addCamera(
            self.port_camera,
            self.port_visual_localiser.robot_to_camera(wpilib.Timer.getFPGATimestamp()),
        )
        self.vision_sim_counter = 0

        self.starboard_vision_servo_sim = PWMSim(self.starboard_visual_localiser.servo)
        self.starboard_vision_encoder_sim = DutyCycleEncoderSim(
            self.starboard_visual_localiser.encoder
        )
        self.port_vision_servo_sim = PWMSim(self.port_visual_localiser.servo)
        self.port_vision_encoder_sim = DutyCycleEncoderSim(
            self.port_visual_localiser.encoder
        )

        self.algae_limit_switch_sim = DIOSim(
            robot.injector_component.algae_limit_switch
        )
        self.algae_pickup_counter = 0

    def update_sim(self, now: float, tm_diff: float) -> None:
        # Enable the Phoenix6 simulated devices
        # TODO: delete when phoenix6 integrates with wpilib
        if wpilib.DriverStation.isEnabled():
            phoenix6.unmanaged.feed_enable(0.1)

        for wheel in self.wheels:
            wheel.update(tm_diff)
        for steer in self.steer:
            steer.update(tm_diff)
        for flywheel in self.flywheels:
            flywheel.update(tm_diff)

        speeds = self.kinematics.toChassisSpeeds(
            (
                self.swerve_modules[0].get(),
                self.swerve_modules[1].get(),
                self.swerve_modules[2].get(),
                self.swerve_modules[3].get(),
            )
        )

        self.imu.add_yaw(math.degrees(speeds.omega * tm_diff))

        self.physics_controller.drive(speeds, tm_diff)

        self.starboard_vision_encoder_sim.set(
            constrain_angle(
                (
                    (
                        self.starboard_visual_localiser.servo_offsets.full_range
                        - self.starboard_visual_localiser.servo_offsets.neutral
                    )
                    * (2.0 * self.starboard_visual_localiser.servo.getPosition() - 1.0)
                    + self.starboard_visual_localiser.servo_offsets.neutral
                ).radians()
            )
        )

        self.port_vision_encoder_sim.set(
            constrain_angle(
                (
                    (
                        self.port_visual_localiser.servo_offsets.full_range
                        - self.port_visual_localiser.servo_offsets.neutral
                    )
                    * (2.0 * self.port_visual_localiser.servo.getPosition() - 1.0)
                    + self.port_visual_localiser.servo_offsets.neutral
                ).radians()
            )
        )

        # Simulate slow vision updates.
        self.vision_sim_counter += 1
        if self.vision_sim_counter == 10:
            self.vision_sim.adjustCamera(
                self.starboard_camera,
                self.starboard_visual_localiser.robot_to_camera(
                    wpilib.Timer.getFPGATimestamp()
                ),
            )
            self.vision_sim.adjustCamera(
                self.port_camera,
                self.port_visual_localiser.robot_to_camera(
                    wpilib.Timer.getFPGATimestamp()
                ),
            )
            self.vision_sim.update(self.physics_controller.get_pose())
            self.vision_sim_counter = 0

        # Update intake arm simulation
        self.intake_arm.update(tm_diff)
        intake_arm_angle = self.intake_arm.mech_sim.getAngle()
        self.intake_arm_encoder_sim.set(
            intake_arm_angle + IntakeComponent.ARM_ENCODER_OFFSET
        )

        # Update wrist simulation
        self.wrist.update(tm_diff)
        wrist_angle = self.wrist.mech_sim.getAngle()
        self.wrist_encoder_sim.set(wrist_angle + WristComponent.ENCODER_ZERO_OFFSET)

        self.injector.update(tm_diff)
        injector_output = self.injector.motor_sim.getAngularVelocity()

        # Simulate algae pick up
        if (
            wrist_angle < WristComponent.NEUTRAL_ANGLE
            and intake_arm_angle < IntakeComponent.RETRACTED_ANGLE
            and injector_output < 0
        ):
            # Simulate driving around for a couple of seconds
            self.algae_pickup_counter += 1
            if self.algae_pickup_counter == 100:
                self.algae_limit_switch_sim.setValue(False)
        else:
            self.algae_pickup_counter = 0

        # Reef intake
        if (
            wrist_angle > WristComponent.NEUTRAL_ANGLE + WristComponent.TOLERANCE
            and injector_output < 0
        ):
            # Check near reef
            pose = self.physics_controller.get_pose()
            pos = pose.translation()
            if (
                pos.distance(game.BLUE_REEF_POS) < 2.0
                or pos.distance(game.RED_REEF_POS) < 2.0
            ):
                self.algae_limit_switch_sim.setValue(False)

        # Algae shooting
        if injector_output > 0:
            self.algae_limit_switch_sim.setValue(True)
