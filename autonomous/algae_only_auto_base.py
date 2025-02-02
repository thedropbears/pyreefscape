import math

import choreo
import choreo.trajectory
import wpilib
from choreo.trajectory import SwerveSample, SwerveTrajectory
from magicbot import AutonomousStateMachine, state
from wpilib import RobotBase
from wpimath.controller import PIDController
from wpimath.kinematics import ChassisSpeeds

from components.chassis import ChassisComponent
from controllers.algae_shooter import AlgaeShooter
from controllers.reef_intake import ReefIntake
from utilities import game


class AlgaeOnlyAutoBase(AutonomousStateMachine):
    reef_intake: ReefIntake
    algae_shooter: AlgaeShooter

    chassis: ChassisComponent

    DISTANCE_TOLERANCE = 0.05  # metres

    def __init__(self, starting_trajectory: str) -> None:
        # We want to parameterise these by paths and potentially a sequence of events
        super().__init__()
        self.x_controller = PIDController(0.5, 0.0, 0.0)
        self.y_controller = PIDController(0.5, 0.0, 0.0)
        self.heading_controller = PIDController(3.5, 0, 0)
        self.heading_controller.enableContinuousInput(-math.pi, math.pi)
        self.snapping_to_heading = False
        self.heading_controller.setTolerance(0.05)

        self.starting_time: float = 0
        wpilib.SmartDashboard.putData("AUTO Heading PID", self.heading_controller)
        wpilib.SmartDashboard.putData("AUTO X PID", self.x_controller)
        wpilib.SmartDashboard.putData("AUTO Y PID", self.y_controller)

        try:
            self.trajectory = choreo.load_swerve_trajectory(starting_trajectory)

        except ValueError:
            # If the trajectory is not found, ChoreoLib already prints to DriverStation
            self.trajectory = SwerveTrajectory("", [], [], [])

    def setup(self) -> None:
        #  setup path tracking controllers

        # init any other defaults
        pass

    def on_enable(self) -> None:
        # configure defaults for pose in sim

        # Setup starting position in the simulator

        self.starting_pose = self.trajectory.get_initial_pose(game.is_red())

        if RobotBase.isSimulation() and self.starting_pose is not None:
            self.chassis.set_pose(self.starting_pose)
        super().on_enable()

    @state(first=True)
    def preparing_intake(self, initial_call: bool) -> None:
        if initial_call:
            self.reef_intake.intake()
        if self.reef_intake.wrist.at_setpoint() or RobotBase.isSimulation():
            self.next_state("tracking_trajectory")

    @state
    def tracking_trajectory(self, state_tm) -> None:
        # get next leg on entry
        current_pose = self.chassis.get_pose()
        final_pose = self.trajectory.get_final_pose(game.is_red())
        if final_pose is None:
            self.done()
            return

        distance = (current_pose.translation() - final_pose.translation()).norm()

        if distance < self.DISTANCE_TOLERANCE:
            self.next_state("shooting")

        sample = self.trajectory.sample_at(state_tm, game.is_red())

        assert sample is not None

        self.follow_trajectory(sample)

    def follow_trajectory(self, sample: SwerveSample):
        # track path

        pose = self.chassis.get_pose()

        # Generate the next speeds for the robot
        speeds = ChassisSpeeds(
            sample.vx + self.x_controller.calculate(pose.X(), sample.x),
            sample.vy + self.y_controller.calculate(pose.Y(), sample.y),
            sample.omega
            + self.heading_controller.calculate(
                pose.rotation().radians(), sample.heading
            ),
        )

        # Apply the generated speeds
        self.chassis.drive_field(speeds.vx, speeds.vy, speeds.omega)

    @state
    def shooting(self, initial_call: bool) -> None:
        if initial_call:
            if self.reef_intake.is_executing:
                self.reef_intake.done()
            else:
                self.algae_shooter.shoot()
