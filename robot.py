import math

import magicbot
import ntcore
import wpilib
import wpilib.event
from magicbot import tunable
from phoenix6.configs import Slot0Configs
from wpimath.filter import Debouncer
from wpimath.geometry import Rotation2d, Rotation3d, Translation3d

from autonomous.auto_base import AutoBase
from components.algae_manipulator import AlgaeManipulatorComponent
from components.ballistics import BallisticsComponent
from components.chassis import ChassisComponent, SwerveConfig
from components.climber import ClimberComponent
from components.coral_placer import CoralPlacerComponent
from components.feeler import FeelerComponent
from components.intake import IntakeComponent
from components.led_component import LightStrip
from components.vision import VisualLocalizer
from components.wrist import WristComponent
from controllers.algae_shooter import AlgaeShooter
from controllers.climber import ClimberStateMachine
from controllers.coral_placer import CoralPlacer
from controllers.feeler import Feeler
from controllers.floor_intake import FloorIntake
from controllers.reef_intake import ReefIntake
from ids import DioChannel, PwmChannel, RioSerialNumber
from utilities.game import is_red
from utilities.scalers import rescale_js


class MyRobot(magicbot.MagicRobot):
    # Controllers
    coral_placer: CoralPlacer
    reef_intake: ReefIntake
    algae_shooter: AlgaeShooter
    floor_intake: FloorIntake
    feeler: Feeler
    climber_state_machine: ClimberStateMachine

    # Components
    chassis: ChassisComponent
    climber: ClimberComponent
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
    dpad_max_speed = tunable(0.4)

    START_POS_TOLERANCE = 0.5

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

        self.mech = wpilib.Mechanism2d(2, 2)
        wpilib.SmartDashboard.putData("Mech2d", self.mech)
        self.frame_mech_root = self.mech.getRoot("A-Frame", 1, 0)
        self.frame_member = self.frame_mech_root.appendLigament(
            "upright", length=1, angle=90, lineWidth=3
        )
        self.wrist_mech_root = self.mech.getRoot("Wrist", 1, 1)

        self.status_lights_strip_length = 28 * 3 * 3 + 144 + 3

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

            self.vision_pos = Translation3d(-0.050, 0.305, 0.675)
            self.vision_rot = Rotation3d(0, 0, math.radians(-90.0))
            self.vision_servo_offset = Rotation2d(0.839)
            self.vision_encoder_offset = Rotation2d(0.136)

        self.coast_button = wpilib.DigitalInput(DioChannel.SWERVE_COAST_SWITCH)
        self.coast_button_debouncer = Debouncer(0.3, Debouncer.DebounceType.kBoth)

    def teleopInit(self) -> None:
        self.field.getObject("Intended start pos").setPoses([])
        self.chassis.set_coast_in_neutral(False)

    def teleopPeriodic(self) -> None:
        # Set max speed
        max_speed = self.lower_max_speed
        dpad_max_speed = self.dpad_max_speed
        max_spin_rate = self.lower_max_spin_rate
        if self.gamepad.getRightBumperButton():
            max_speed = self.max_speed
            max_spin_rate = self.max_spin_rate

        # Driving
        drive_x = -rescale_js(self.gamepad.getLeftY(), 0.05, 15) * max_speed
        drive_y = -rescale_js(self.gamepad.getLeftX(), 0.05, 15) * max_speed
        drive_z = (
            -rescale_js(self.gamepad.getRightX(), 0.1, exponential=20) * max_spin_rate
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

        if (
            self.algae_manipulator_component.should_be_holding_algae()
            or self.floor_intake.is_executing
            or self.reef_intake.is_executing
        ):
            inversion_factor = (
                -1
            )  # inverts direction through multiplication of this value
        else:
            inversion_factor = 1

        if dpad != -1:
            self.chassis.drive_local(
                (dpad_max_speed * math.cos(math.radians(dpad))) * inversion_factor,
                (dpad_max_speed * math.sin(math.radians(dpad))) * -inversion_factor,
                0,
            )

        if self.gamepad.getLeftTriggerAxis() > 0.5:
            self.floor_intake.intake()

        if self.gamepad.getLeftBumperButton():
            self.reef_intake.intake()

        if self.gamepad.getYButton():
            self.climber.retract()
        if self.gamepad.getAButton():
            self.climber.deploy()
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
        if self.gamepad.getYButton():
            self.climber_state_machine.climb()

        if self.gamepad.getBButton():
            self.reef_intake.done()
            self.floor_intake.done()

        self.chassis.update_odometry()

        if self.gamepad.getRightTriggerAxis() > 0.5:
            self.algae_shooter.shoot()
        if self.gamepad.getAButton():
            self.climber.deploy()

        if self.gamepad.getLeftBumperButton():
            self.reef_intake.intake()

        if self.gamepad.getLeftTriggerAxis() > 0.5:
            self.floor_intake.intake()

        # Controllers
        self.coral_placer.execute()
        self.reef_intake.execute()
        self.algae_shooter.execute()
        self.floor_intake.execute()
        self.feeler.execute()
        self.climber_state_machine.execute()

        # Components
        self.chassis.execute()
        self.climber.execute()
        self.coral_placer_component.execute()
        self.algae_manipulator_component.execute()
        if self.gamepad.getStartButton():
            self.vision.zero_servo_()
        else:
            self.vision.execute()
        self.wrist.execute()
        self.intake_component.execute()
        self.status_lights.execute()
        self.feeler_component.execute()
        # self.ballistics_component.execute()

    def disabledPeriodic(self) -> None:
        self.chassis.update_alliance()
        self.chassis.update_odometry()

        self.wrist.reset_windup()
        self.wrist.execute()

        self.vision.execute()

        if self.vision.sees_target():
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

        if not self.coast_button_debouncer.calculate(self.coast_button.get()):
            self.chassis.toggle_coast_in_neutral()

    def robotPeriodic(self) -> None:
        super().robotPeriodic()
        # Clear component per-loop caches.
        self.vision._per_loop_cache.clear()
