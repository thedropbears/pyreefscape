import math

import magicbot
import ntcore
import wpilib
import wpilib.event
from magicbot import tunable
from wpimath.geometry import Rotation3d, Translation3d

from components.chassis import ChassisComponent
from components.climber import ClimberComponent
from components.coral_placer import CoralPlacerComponent
from components.manipulator import ManipulatorComponent
from components.vision import VisualLocalizer
from controllers.coral_placer import CoralPlacer
from utilities.game import is_red
from utilities.scalers import rescale_js


class MyRobot(magicbot.MagicRobot):
    # Controllers
    coral_placer: CoralPlacer

    # Components
    chassis: ChassisComponent
    climber: ClimberComponent
    coral_placer_component: CoralPlacerComponent
    manipulator_component: ManipulatorComponent
    vision: VisualLocalizer

    max_speed = magicbot.tunable(5)  # m/s
    lower_max_speed = magicbot.tunable(2)  # m/s
    max_spin_rate = magicbot.tunable(4)  # m/s
    lower_max_spin_rate = magicbot.tunable(2)  # m/s
    inclination_angle = tunable(0.0)

    START_POS_TOLERANCE = 1

    def createObjects(self) -> None:
        self.data_log = wpilib.DataLogManager.getLog()

        # Log deploy info to show in AdvantageScope.
        deploy_info = wpilib.deployinfo.getDeployData()
        if deploy_info is not None:
            meta_table = ntcore.NetworkTableInstance.getDefault().getTable("Metadata")
            for k, v in deploy_info.items():
                meta_table.putString(k, v)

        self.gamepad = wpilib.XboxController(0)

        self.field = wpilib.Field2d()
        wpilib.SmartDashboard.putData(self.field)

        # side: (28*3)*2 + front: (30*3) - 2 (R.I.P)
        self.status_lights_strip_length = (28 * 3) * 2 + (30 * 3) - 2

        self.vision_name = "ardu_cam"
        self.vision_pos = Translation3d(0.22, 0, 0.295)
        self.vision_rot = Rotation3d(0, -math.radians(20), 0)

    def teleopInit(self) -> None:
        self.field.getObject("Intended start pos").setPoses([])

    def teleopPeriodic(self) -> None:
        if self.gamepad.getYButton():
            self.coral_placer.place()

        # Set max speed
        max_speed = self.max_speed
        max_spin_rate = self.max_spin_rate
        if self.gamepad.getRightBumper():
            max_speed = self.lower_max_speed
            max_spin_rate = self.lower_max_spin_rate

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

        # Set current robot direction to forward
        if dpad in (135, 180, 235):
            self.chassis.reset_yaw()

        # Reset Odometry
        if dpad in (0, 45, 315):
            self.chassis.reset_odometry()

    def testInit(self) -> None:
        pass

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

        if self.gamepad.getRightBumper():
            self.climber.pullRope()
        if self.gamepad.getRightTriggerAxis() >= 0.3:
            self.climber.loosenRope()

        self.coral_placer_component.execute()

        self.chassis.execute()

        self.climber.execute()

        self.chassis.update_odometry()

        self.vision.execute()

        if self.gamepad.getXButton():
            self.manipulator_component.spin_flywheels()
        if self.gamepad.getYButton():
            self.manipulator_component.inject()
        if self.gamepad.getAButton():
            self.manipulator_component.intake()

        self.manipulator_component.execute()

    def disabledPeriodic(self) -> None:
        self.chassis.update_alliance()
        self.chassis.update_odometry()

        self.vision.execute()
