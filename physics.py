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
    AnalogEncoderSim,
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


class SparkArmSim:
    def __init__(self, mech_sim: SingleJointedArmSim, motor_sim: rev.SparkSim) -> None:
        self.mech_sim = mech_sim
        self.motor_sim = motor_sim

    def update(self, dt: float) -> None:
        vbus = self.motor_sim.getBusVoltage()
        self.mech_sim.setInputVoltage(self.motor_sim.getAppliedOutput() * vbus)
        self.mech_sim.update(dt)
        self.motor_sim.iterate(self.mech_sim.getVelocity(), vbus, dt)


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
            Falcon500MotorSim(
                robot.shooter_component.top_flywheel,
                gearing=1 / 1,
                moi=0.00105679992,
            ),
            Falcon500MotorSim(
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
                moi=0.035579622,
                armLength=0.22,
                minAngle=IntakeComponent.DEPLOYED_ANGLE,
                maxAngle=IntakeComponent.RETRACTED_ANGLE,
                simulateGravity=True,
                startingAngle=IntakeComponent.RETRACTED_ANGLE,
            ),
            self.intake_arm_motor,
        )

        wrist_gearbox = DCMotor.NEO(1)
        wrist_motor = rev.SparkMaxSim(robot.wrist.motor, wrist_gearbox)
        self.wrist_encoder_sim = AnalogEncoderSim(robot.wrist.wrist_encoder)

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

        self.vision_sim = VisionSystemSim("main")
        self.vision_sim.addAprilTags(game.apriltag_layout)
        properties = SimCameraProperties.OV9281_1280_720()
        self.camera = PhotonCameraSim(robot.vision.camera, properties)
        self.camera.setMaxSightRange(5.0)
        self.visual_localiser = robot.vision
        self.vision_sim.addCamera(
            self.camera,
            self.visual_localiser.robot_to_camera(wpilib.Timer.getFPGATimestamp()),
        )
        self.vision_sim_counter = 0

        self.vision_servo_sim = PWMSim(self.visual_localiser.servo)
        self.vision_encoder_sim = DutyCycleEncoderSim(self.visual_localiser.encoder)

        self.algae_limit_switch_sim = DIOSim(
            robot.injector_component.algae_limit_switch
        )
        self.algae_pickup_counter = 0
        self.floor_intake = robot.floor_intake
        self.reef_intake = robot.reef_intake
        self.algae_shooter = robot.algae_shooter

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
                self.camera,
                self.visual_localiser.robot_to_camera(wpilib.Timer.getFPGATimestamp()),
            )
            self.vision_sim.update(self.physics_controller.get_pose())
            self.vision_sim_counter = 0

        # Update intake arm simulation
        self.intake_arm.update(tm_diff)
        self.intake_arm_encoder_sim.set(self.intake_arm.mech_sim.getAngle())

        # Update wrist simulation
        self.wrist.update(tm_diff)
        self.wrist_encoder_sim.set(
            self.wrist.mech_sim.getAngle() + WristComponent.ENCODER_ZERO_OFFSET
        )

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
        # Algae shooting
        if self.algae_shooter.current_state == "shooting":
            self.algae_limit_switch_sim.setValue(True)
