from __future__ import annotations

import math
import typing

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
)
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.system.plant import DCMotor, LinearSystemId
from wpimath.units import kilogram_square_meters

from components.chassis import SwerveModule
from utilities import game
from utilities.functions import constrain_angle

if typing.TYPE_CHECKING:
    from robot import MyRobot


class SimpleTalonFXMotorSim:
    def __init__(
        self, motor: phoenix6.hardware.TalonFX, units_per_rev: float, kV: float
    ) -> None:
        self.sim_state = motor.sim_state
        self.sim_state.set_supply_voltage(12.0)
        self.kV = kV  # volt seconds per unit
        self.units_per_rev = units_per_rev

    def update(self, dt: float) -> None:
        voltage = self.sim_state.motor_voltage
        velocity = voltage / self.kV  # units per second
        velocity_rps = velocity * self.units_per_rev
        self.sim_state.set_rotor_velocity(velocity_rps)
        self.sim_state.add_rotor_position(velocity_rps * dt)


class Falcon500MotorSim:
    def __init__(
        self,
        *motors: phoenix6.hardware.TalonFX,
        # Reduction between motor and encoder readings, as output over input.
        # If the mechanism spins slower than the motor, this number should be greater than one.
        gearing: float,
        moi: kilogram_square_meters,
    ):
        self.falcon = DCMotor.falcon500(len(motors))
        self.plant = LinearSystemId.DCMotorSystem(self.falcon, moi, gearing)
        self.gearing = gearing
        self.sim_states = [motor.sim_state for motor in motors]
        for sim_state in self.sim_states:
            sim_state.set_supply_voltage(12.0)
        self.motor_sim = DCMotorSim(self.plant, self.falcon)

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
            Falcon500MotorSim(
                module.steer,
                gearing=1 / robot.chassis.swerve_config.steer_ratio,
                # measured from MKCad CAD
                moi=0.0009972,
            )
            for module in robot.chassis.modules
        ]
        self.flywheels = [
            Falcon500MotorSim(motor, gearing=1 / 1, moi=0.000412752224)
            for motor in (
                robot.algae_manipulator_component.flywheel_1,
                robot.algae_manipulator_component.flywheel_2,
            )
        ]

        self.wrist_motor = rev.SparkMaxSim(
            sparkMax=robot.wrist.motor, motor=DCMotor.NEO(1)
        )

        self.imu = robot.chassis.imu.sim_state

        self.vision_sim = VisionSystemSim("main")
        self.vision_sim.addAprilTags(game.apriltag_layout)
        properties = SimCameraProperties.OV9281_1280_720()
        self.camera = PhotonCameraSim(robot.vision.camera, properties)
        self.camera.setMaxSightRange(5.0)
        self.visual_localiser = robot.vision
        self.vision_sim.addCamera(self.camera, self.visual_localiser.robot_to_camera)
        self.vision_sim_counter = 0

        self.vision_servo_sim = PWMSim(self.visual_localiser.servo)
        self.vision_encoder_sim = DutyCycleEncoderSim(self.visual_localiser.encoder)

        self.algae_limit_switch_sim = DIOSim(
            robot.algae_manipulator_component.algae_limit_switch
        )
        self.algae_finger_switch_sim = DIOSim(robot.feeler_component.limit_switch)
        self.algae_pickup_counter = 0
        self.feeler = robot.feeler
        self.floor_intake = robot.floor_intake
        self.reef_intake = robot.reef_intake
        self.algae_shooter = robot.algae_shooter
        self.wrist_component = robot.wrist

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

        self.vision_encoder_sim.set(
            constrain_angle(
                (
                    (2.0 * self.visual_localiser.servo.getPosition() - 1.0)
                    * self.visual_localiser.SERVO_HALF_ANGLE
                )
                + self.visual_localiser.servo_offset.radians()
            )
        )

        # Simulate slow vision updates.
        self.vision_sim_counter += 1
        if self.vision_sim_counter == 10:
            self.vision_sim.adjustCamera(
                self.camera, self.visual_localiser.robot_to_camera
            )
            self.vision_sim.update(self.physics_controller.get_pose())
            self.vision_sim_counter = 0

        # Trivially simple wrist simulation
        self.wrist_motor.setPosition(self.wrist_component.desired_angle)
        self.wrist_motor.iterate(0.0, 12.0, tm_diff)

        # Simulate algae pick up
        if self.floor_intake.current_state == "intaking":
            # Simulate driving around for a couple of seconds
            self.algae_pickup_counter += 1
            if self.algae_pickup_counter == 100:
                self.algae_limit_switch_sim.setValue(False)
        else:
            self.algae_pickup_counter = 0
        if self.reef_intake.current_state == "intaking":
            # Check near reef
            pose = self.physics_controller.get_pose()
            pos = pose.translation()
            if (
                pos.distance(game.BLUE_REEF_POS) < 2.0
                or pos.distance(game.RED_REEF_POS) < 2.0
            ):
                self.algae_limit_switch_sim.setValue(False)
        # Algae feeler
        if self.feeler.current_state == "searching":
            self.algae_finger_switch_sim.setValue(True)
        else:
            self.algae_finger_switch_sim.setValue(False)
        # Algae shooting
        if self.algae_shooter.current_state == "shooting":
            self.algae_limit_switch_sim.setValue(True)
