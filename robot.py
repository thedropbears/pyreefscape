import math

import magicbot
import ntcore
import wpilib
import wpilib.event
from magicbot import tunable
from phoenix6.configs import Slot0Configs
from wpimath.geometry import Rotation2d, Translation3d

from autonomous.auto_base import AutoBase
from components.ballistics import BallisticsComponent
from components.chassis import ChassisComponent, SwerveConfig
from components.climber import ClimberComponent
from components.coral_depositor import CoralDepositorComponent
from components.injector import InjectorComponent
from components.intake import IntakeComponent
from components.led_component import LightStrip
from components.shooter import ShooterComponent
from components.vision import ServoOffsets, VisualLocalizer
from components.wrist import WristComponent
from controllers.algae_measurement import AlgaeMeasurement
from controllers.algae_shooter import AlgaeShooter
from controllers.climber import ClimberStateMachine
from controllers.floor_intake import FloorIntake
from controllers.reef_intake import ReefIntake
from ids import DioChannel, PwmChannel, RioSerialNumber
from utilities.game import is_red
from utilities.scalers import rescale_js


class MyRobot(magicbot.MagicRobot):
    # Controllers
    reef_intake: ReefIntake
    algae_shooter: AlgaeShooter
    floor_intake: FloorIntake
    climber_state_machine: ClimberStateMachine
    algae_measurement: AlgaeMeasurement

    # Components
    chassis: ChassisComponent
    climber: ClimberComponent
    coral_depositor_component: CoralDepositorComponent
    shooter_component: ShooterComponent
    injector_component: InjectorComponent
    vision: VisualLocalizer
    wrist: WristComponent
    intake_component: IntakeComponent
    status_lights: LightStrip
    ballistics_component: BallisticsComponent

    max_speed = tunable(5.0)  # m/s
    lower_max_speed = tunable(2.0)  # m/s
    max_spin_rate = tunable(4.0)  # m/s
    lower_max_spin_rate = tunable(2.0)  # m/s
    inclination_angle = tunable(0.0)
    dpad_max_speed = tunable(0.4)

    START_POS_TOLERANCE = 0.2

    def createObjects(self) -> None:
        self.event_loop = wpilib.event.EventLoop()
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

        self.mech = wpilib.Mechanism2d(2, 2)
        wpilib.SmartDashboard.putData("Mech2d", self.mech)
        self.intake_mech_root = self.mech.getRoot("Intake", 1.5, 0.1)
        self.frame_mech_root = self.mech.getRoot("A-Frame", 1, 0)
        self.frame_member = self.frame_mech_root.appendLigament(
            "upright", length=1, angle=90, lineWidth=3
        )
        self.wrist_mech_root = self.mech.getRoot("Wrist", 1, 1)

        self.status_lights_strip_length = 112 * 4

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

            self.vision_turret_pos = Translation3d(0.300, 0.000, 0.250)
            self.vision_turret_rot = Rotation2d.fromDegrees(0.0)
            self.vision_camera_offset = Translation3d(0.021, 0, 0)
            self.vision_camera_pitch = math.radians(-20.0)
            self.vision_encoder_offset = Rotation2d(0.0)
            self.vision_servo_offsets = ServoOffsets(
                neutral=Rotation2d(0.0), full_range=Rotation2d(1.377)
            )
            self.vision_rotation_range = (Rotation2d(-1.377), Rotation2d(1.377))

        else:
            self.chassis_swerve_config = SwerveConfig(
                drive_ratio=(14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0),
                drive_gains=Slot0Configs()
                .with_k_p(7.8294)
                .with_k_i(0)
                .with_k_d(0)
                .with_k_s(0.11742)
                .with_k_v(2.3941)
                .with_k_a(0.11426),
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

            self.vision_turret_pos = Translation3d(-0.030, -0.300, 0.660)
            self.vision_turret_rot = Rotation2d.fromDegrees(-90.0)
            self.vision_camera_offset = Translation3d(0.021, 0, 0)
            self.vision_camera_pitch = math.radians(10.0)
            self.vision_encoder_offset = Rotation2d(0.942)
            self.vision_servo_offsets = ServoOffsets(
                neutral=Rotation2d(0.006), full_range=Rotation2d(1.377)
            )
            self.vision_rotation_range = (Rotation2d(4.76), Rotation2d(1.87))

        self.coast_button = wpilib.DigitalInput(DioChannel.SWERVE_COAST_SWITCH)
        self.coast_button_pressed_event = wpilib.event.BooleanEvent(
            self.event_loop, self.coast_button.get
        ).falling()

    def teleopInit(self) -> None:
        self.field.getObject("Intended start pos").setPoses([])
        self.chassis.set_coast_in_neutral(False)

    def teleopPeriodic(self) -> None:
        # Set max speed
        # TODO Set max speed to something sensible for comp
        # max_speed = self.max_speed
        # max_spin_rate = self.max_spin_rate
        max_speed = self.lower_max_speed
        max_spin_rate = self.lower_max_spin_rate
        if self.gamepad.getRightBumperButton():
            max_speed = self.lower_max_speed
            max_spin_rate = self.lower_max_spin_rate

        # Driving
        drive_x = -rescale_js(self.gamepad.getLeftY(), 0.05, 15) * max_speed
        drive_y = -rescale_js(self.gamepad.getLeftX(), 0.05, 15) * max_speed
        drive_z = (
            -rescale_js(self.gamepad.getRightX(), 0.1, exponential=20) * max_spin_rate
        )
        local_driving = self.gamepad.getRightBumperButton()

        if local_driving:
            if (
                self.injector_component.has_algae()
                or self.floor_intake.is_executing
                or self.reef_intake.is_executing
            ):
                # Make the shooter behave like the front of the robot
                self.chassis.drive_local(-drive_x, -drive_y, drive_z)
            else:
                # Climber is front as defined in the chassis
                self.chassis.drive_local(drive_x, drive_y, drive_z)
        else:
            if is_red():
                drive_x = -drive_x
                drive_y = -drive_y
            self.chassis.drive_field(drive_x, drive_y, drive_z)

        # Give rotational access to the driver
        if drive_z != 0:
            self.chassis.stop_snapping()

        if self.gamepad.getLeftTriggerAxis() > 0.5:
            self.floor_intake.intake()

        if self.gamepad.getLeftBumperButton():
            self.reef_intake.intake()

        if self.gamepad.getYButton():
            self.climber_state_machine.deploy()
        if self.gamepad.getAButton():
            self.climber_state_machine.retract()

        if self.gamepad.getBButton():
            self.reef_intake.done()
            self.floor_intake.done()

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
        if self.gamepad.getBButton():
            self.reef_intake.done()
            self.floor_intake.done()

        self.chassis.update_odometry()

        if self.gamepad.getRightTriggerAxis() > 0.5:
            self.algae_shooter.shoot()

        if self.gamepad.getYButtonPressed():
            if self.climber_state_machine.current_state == "deploying":
                self.climber_state_machine.retract()
            else:
                self.climber_state_machine.deploy()

        if self.gamepad.getXButton():
            self.coral_depositor_component.deposit()
        elif self.gamepad.getAButton():
            self.coral_depositor_component.retract()
        elif self.gamepad.getBackButton():
            self.coral_depositor_component.tuck()

        if self.gamepad.getLeftBumperButton():
            self.reef_intake.intake()

        if self.gamepad.getLeftTriggerAxis() > 0.5:
            self.floor_intake.intake()

        if self.gamepad.getRightBumper():
            self.algae_measurement.engage()

        # Controllers
        self.reef_intake.execute()
        self.algae_shooter.execute()
        self.floor_intake.execute()
        self.climber_state_machine.execute()
        self.algae_measurement.execute()

        # Components
        self.chassis.execute()
        self.climber.execute()
        self.coral_depositor_component.execute()
        self.shooter_component.execute()
        self.injector_component.execute()
        if self.gamepad.getLeftStickButton():
            self.vision.zero_servo_()
        elif self.gamepad.getRightStickButton():
            self.vision.full_range_servo_()
        else:
            self.vision.execute()
        self.wrist.execute()
        self.intake_component.execute()
        self.status_lights.execute()
        # self.ballistics_component.execute()

    def disabledPeriodic(self) -> None:
        self.event_loop.poll()

        self.chassis.update_alliance()
        self.chassis.update_odometry()

        self.wrist.reset_windup()
        self.wrist.execute()

        self.vision.execute()

        if self.vision.sees_multi_tag_target():
            selected_auto = self._automodes.chooser.getSelected()
            if isinstance(selected_auto, AutoBase):
                intended_start_pose = selected_auto.get_starting_pose()
                current_pose = self.chassis.get_pose()
                if intended_start_pose is not None:
                    self.field.getObject("Intended start pos").setPose(
                        intended_start_pose
                    )
                    relative_translation = intended_start_pose.relativeTo(
                        current_pose
                    ).translation()
                    if relative_translation.norm() > self.START_POS_TOLERANCE:
                        self.status_lights.invalid_start(
                            relative_translation, self.START_POS_TOLERANCE
                        )
                    else:
                        self.status_lights.rainbow()
            else:
                self.status_lights.no_auto()
        else:
            self.status_lights.vision_timeout()

        self.status_lights.execute()

        if self.coast_button_pressed_event.getAsBoolean():
            self.chassis.toggle_coast_in_neutral()
            self.wrist.toggle_neutral_mode()

    def robotPeriodic(self) -> None:
        super().robotPeriodic()
        self.intake_component.periodic()
        # Clear component per-loop caches.
        self.vision._per_loop_cache.clear()
