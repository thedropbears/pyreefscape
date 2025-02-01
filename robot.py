import math

import magicbot
import ntcore
import wpilib
import wpilib.event
from magicbot import tunable
from phoenix6.configs import Slot0Configs
from wpimath.geometry import Rotation2d, Rotation3d, Translation3d

from components.algae_manipulator import AlgaeManipulatorComponent
from components.ballistics import BallisticsComponent
from components.chassis import ChassisComponent, SwerveConfig
from components.coral_placer import CoralPlacerComponent
from components.feeler import FeelerComponent
from components.intake import IntakeComponent
from components.led_component import LightStrip
from components.vision import VisualLocalizer
from components.wrist import WristComponent
from controllers.algae_shooter import AlgaeShooter
from controllers.coral_placer import CoralPlacer
from controllers.feeler import Feeler
from controllers.floor_intake import FloorIntake
from controllers.reef_intake import ReefIntake
from ids import DioChannel, PwmChannel, RioSerialNumber
from utilities.functions import clamp
from utilities.game import is_red
from utilities.scalers import rescale_js


class MyRobot(magicbot.MagicRobot):
    # Controllers
    coral_placer: CoralPlacer
    reef_intake: ReefIntake
    algae_shooter: AlgaeShooter
    floor_intake: FloorIntake
    feeler: Feeler

    # Components
    chassis: ChassisComponent
    coral_placer_component: CoralPlacerComponent
    algae_manipulator_component: AlgaeManipulatorComponent
    vision: VisualLocalizer
    wrist: WristComponent
    intake_component: IntakeComponent
    status_lights: LightStrip
    feeler_component: FeelerComponent
    ballistics_component: BallisticsComponent

    max_speed = tunable(5.0)  # m/s
    lower_max_speed = tunable(2.0)  # m/s
    max_spin_rate = tunable(4.0)  # m/s
    lower_max_spin_rate = tunable(2.0)  # m/s
    inclination_angle = tunable(0.0)

    START_POS_TOLERANCE = 1

    def createObjects(self) -> None:
        self.data_log = wpilib.DataLogManager.getLog()

        # Log deploy info to show in AdvantageScope.
        meta_table = ntcore.NetworkTableInstance.getDefault().getTable("Metadata")
        deploy_info = wpilib.deployinfo.getDeployData()
        if deploy_info is not None:
            for k, v in deploy_info.items():
                meta_table.putString(k, v)

        # Also log roboRIO metadata.
        meta_table.putString("runtime_type", self.getRuntimeType().name[1:])
        meta_table.putString("rio_serial", wpilib.RobotController.getSerialNumber())

        self.gamepad = wpilib.XboxController(0)

        self.field = wpilib.Field2d()
        wpilib.SmartDashboard.putData(self.field)

        self.status_lights_strip_length = 28 * 3

        self.vision_name = "ardu_cam"
        self.vision_encoder_id = DioChannel.VISION_ENCODER
        self.vision_servo_id = PwmChannel.VISION_SERVO

        if wpilib.RobotController.getSerialNumber() == RioSerialNumber.TEST_BOT:
            self.chassis_swerve_config = SwerveConfig(
                drive_ratio=(14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0),
                drive_gains=Slot0Configs()
                .with_k_p(1.0868)
                .with_k_i(0)
                .with_k_d(0)
                .with_k_s(0.15172)
                .with_k_v(2.8305)
                .with_k_a(0.082659),
                steer_ratio=(14 / 50) * (10 / 60),
                steer_gains=Slot0Configs()
                .with_k_p(30.234)
                .with_k_i(0)
                .with_k_d(0.62183)
                .with_k_s(0.1645),
                reverse_drive=False,
            )
            # metres between centre of left and right wheels
            self.chassis_track_width = 0.467
            # metres between centre of front and back wheels
            self.chassis_wheel_base = 0.467

            self.vision_pos = Translation3d(0.22, 0, 0.295)
            self.vision_rot = Rotation3d(0, -math.radians(20), 0)
            self.vision_servo_offset = Rotation2d(3.107)
            self.vision_encoder_offset = Rotation2d(3.052)
        else:
            self.chassis_swerve_config = SwerveConfig(
                drive_ratio=(14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0),
                drive_gains=Slot0Configs()
                .with_k_p(0.15039)
                .with_k_i(0)
                .with_k_d(0)
                .with_k_s(0.21723)
                .with_k_v(2.8697)
                .with_k_a(0.048638),
                steer_ratio=(14 / 50) * (10 / 60),
                steer_gains=Slot0Configs()
                .with_k_p(92.079)
                .with_k_i(0)
                .with_k_d(1.6683)
                .with_k_s(0.086374),
                reverse_drive=True,
            )
            # metres between centre of left and right wheels
            self.chassis_track_width = 0.517
            # metres between centre of front and back wheels
            self.chassis_wheel_base = 0.517

            self.vision_pos = Translation3d(0.290, -0.195, 0.235)
            self.vision_rot = Rotation3d(0, 0, 0)
            self.vision_servo_offset = Rotation2d(0.563)
            self.vision_encoder_offset = Rotation2d(0.975)

    def teleopInit(self) -> None:
        self.field.getObject("Intended start pos").setPoses([])
        self.chassis.set_coast_in_neutral(False)

    def teleopPeriodic(self) -> None:
        # Set max speed
        max_speed = self.lower_max_speed
        max_spin_rate = self.lower_max_spin_rate
        if self.gamepad.getRightBumper():
            max_speed = self.max_speed
            max_spin_rate = self.max_spin_rate

        # Driving
        drive_x = -rescale_js(self.gamepad.getLeftY(), 0.05, 2.5) * max_speed
        drive_y = -rescale_js(self.gamepad.getLeftX(), 0.05, 2.5) * max_speed
        drive_z = (
            -rescale_js(self.gamepad.getRightX(), 0.1, exponential=2) * max_spin_rate
        )
        local_driving = self.gamepad.getXButton()

        if local_driving:
            self.chassis.drive_local(drive_x, drive_y, drive_z)
        else:
            if is_red():
                drive_x = -drive_x
                drive_y = -drive_y
            self.chassis.drive_field(drive_x, drive_y, drive_z)

        # Give rotational access to the driver
        if drive_z != 0:
            self.chassis.stop_snapping()

        dpad = self.gamepad.getPOV()
        # dpad upwards
        # if dpad in (0, 45, 315):
        # self.climber.deploy()
        # elif dpad in (135, 180, 235):
        # self.climber.retract()

        if self.gamepad.getLeftTriggerAxis() > 0.5:
            self.coral_placer.place()

        if self.gamepad.getYButton():
            self.reef_intake.intake()
        if self.gamepad.getAButton():
            self.floor_intake.intake()
        if self.gamepad.getBButton():
            self.reef_intake.done()
            self.floor_intake.done()

        if dpad in (0, 45, 315):
            self.inclination_angle += math.radians(0.05)
            self.inclination_angle = clamp(
                self.inclination_angle,
                self.wrist.MAXIMUM_DEPRESSION,
                self.wrist.MAXIMUM_ELEVATION,
            )
            self.wrist.tilt_to(self.inclination_angle)
        if dpad in (180, 135, 225):
            self.inclination_angle -= math.radians(0.05)
            self.inclination_angle = clamp(
                self.inclination_angle,
                self.wrist.MAXIMUM_DEPRESSION,
                self.wrist.MAXIMUM_ELEVATION,
            )
            self.wrist.tilt_to(self.inclination_angle)

        if self.gamepad.getRightTriggerAxis() > 0.5:
            self.algae_shooter.shoot()

        # Set current robot direction to forward
        if self.gamepad.getBackButton():
            self.chassis.reset_yaw()
        # Reset Odometry
        if self.gamepad.getStartButton():
            self.chassis.reset_odometry()

    def testInit(self) -> None:
        self.chassis.set_coast_in_neutral(True)

    def testPeriodic(self) -> None:
        dpad = self.gamepad.getPOV()
        if dpad != -1:
            if is_red():
                self.chassis.snap_to_heading(-math.radians(dpad) + math.pi)
            else:
                self.chassis.snap_to_heading(-math.radians(dpad))
        else:
            self.chassis.stop_snapping()
            self.chassis.drive_local(0, 0, 0)

        if self.gamepad.getYButton():
            self.coral_placer_component.place()
        self.coral_placer_component.execute()
        self.status_lights.execute()

        if self.gamepad.getBButton():
            self.vision.zero_servo_()
        else:
            self.vision.execute()

        self.chassis.execute()

        self.chassis.update_odometry()

        if self.gamepad.getRightTriggerAxis() > 0.5:
            self.algae_shooter.shoot()
        if self.gamepad.getAButton():
            self.reef_intake.intake()

        self.algae_shooter.execute()

        self.reef_intake.execute()

        self.algae_manipulator_component.execute()

        if self.gamepad.getLeftBumper():
            self.wrist.zero_wrist()
        if self.gamepad.getLeftTriggerAxis() > 0.3:
            self.wrist.tilt_to(self.inclination_angle)
        self.wrist.execute()

    def disabledPeriodic(self) -> None:
        self.chassis.update_alliance()
        self.chassis.update_odometry()

        self.wrist.reset_windup()
        self.wrist.execute()

        self.vision.execute()

        if self.gamepad.getAButtonPressed():
            self.chassis.toggle_coast_in_neutral()
