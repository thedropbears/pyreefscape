import math

import choreo
import wpilib
from choreo.trajectory import SwerveSample
from magicbot import AutonomousStateMachine, state
from wpilib import RobotBase
from wpimath.controller import PIDController
from wpimath.geometry import Pose2d
from wpimath.kinematics import ChassisSpeeds

from components.chassis import ChassisComponent
from components.injector import InjectorComponent
from components.shooter import ShooterComponent
from controllers.algae_shooter import AlgaeShooter
from controllers.reef_intake import ReefIntake
from utilities import game

x_controller = PIDController(0.5, 0.0, 0.0)
y_controller = PIDController(0.5, 0.0, 0.0)
heading_controller = PIDController(3.0, 0, 0)
heading_controller.enableContinuousInput(-math.pi, math.pi)

wpilib.SmartDashboard.putData("Auto X PID", x_controller)
wpilib.SmartDashboard.putData("Auto Y PID", y_controller)
wpilib.SmartDashboard.putData("Auto Heading PID", heading_controller)


class AutoBase(AutonomousStateMachine):
    algae_shooter: AlgaeShooter
    reef_intake: ReefIntake

    shooter_component: ShooterComponent
    injector_component: InjectorComponent
    chassis: ChassisComponent

    DISTANCE_TOLERANCE = 0.2  # metres
    ANGLE_TOLERANCE = math.radians(3)

    def __init__(self, trajectory_names: list[str]) -> None:
        # We want to parameterise these by paths and potentially a sequence of events
        super().__init__()

        self.current_leg = -1
        self.starting_pose = None
        self.trajectories = []
        for trajectory_name in trajectory_names:
            try:
                self.trajectories.append(choreo.load_swerve_trajectory(trajectory_name))
                if self.starting_pose is None:
                    self.starting_pose = self.get_starting_pose()
            except ValueError:
                # If the trajectory is not found, ChoreoLib already prints to DriverStation
                pass

    def setup(self) -> None:
        #  setup path tracking controllers

        # init any other defaults
        pass

    def on_enable(self) -> None:
        # configure defaults for pose in sim

        # Setup starting position in the simulator
        starting_pose = self.get_starting_pose()
        if RobotBase.isSimulation() and starting_pose is not None:
            self.chassis.set_pose(starting_pose)
        # Reset the counter for which leg we are executing
        self.current_leg = -1

        super().on_enable()

    def get_starting_pose(self) -> Pose2d | None:
        return self.trajectories[0].get_initial_pose(game.is_red())

    @state(first=True)
    def initialising(self) -> None:
        # Add any tasks that need doing first
        self.next_state("driving_to_coral")

    @state
    def tracking_trajectory(self, initial_call, state_tm) -> None:
        if initial_call:
            self.current_leg += 1

        if self.current_leg == len(self.trajectories):
            self.done()
            return

        # get next leg on entry
        current_pose = self.chassis.get_pose()
        final_pose = self.trajectories[self.current_leg].get_final_pose(game.is_red())
        if final_pose is None:
            self.done()
            return

        distance = current_pose.translation().distance(final_pose.translation())
        angle_error = (final_pose.rotation() - current_pose.rotation()).radians()

        if self.current_leg > 0 and not self.injector_component.has_algae():
            self.reef_intake.intake()

        if (
            distance < self.DISTANCE_TOLERANCE
            and math.isclose(angle_error, 0.0, abs_tol=self.ANGLE_TOLERANCE)
            and state_tm > self.trajectories[self.current_leg].get_total_time() / 2.0
        ):
            # First leg is to score coral, then we run cycles of pick up -> shoot
            if self.current_leg == 0:
                self.next_state("scoring_coral")
            elif self.injector_component.has_algae():
                self.next_state("shooting_algae")
            else:
                self.next_state("intaking_algae")
            return

        sample = self.trajectories[self.current_leg].sample_at(state_tm, game.is_red())
        if sample is not None:
            self.follow_trajectory(sample)

    def follow_trajectory(self, sample: SwerveSample):
        # track path

        pose = self.chassis.get_pose()

        # Generate the next speeds for the robot
        speeds = ChassisSpeeds(
            sample.vx + x_controller.calculate(pose.X(), sample.x),
            sample.vy + y_controller.calculate(pose.Y(), sample.y),
            sample.omega
            + heading_controller.calculate(pose.rotation().radians(), sample.heading),
        )

        # Apply the generated speeds
        self.chassis.drive_field(speeds.vx, speeds.vy, speeds.omega)

    @state
    def driving_to_coral(self) -> None:
        # TODO - Set up coral placer in correct position
        self.next_state("tracking_trajectory")

    @state
    def scoring_coral(self, initial_call: bool) -> None:
        # TODO: ADD CORAL BACK WHEN ITS FUNCTIONAL
        self.next_state("retreating")

    @state
    def retreating(self) -> None:
        self.next_state("tracking_trajectory")

    @state
    def intaking_algae(self) -> None:
        if self.injector_component.has_algae():
            self.next_state("tracking_trajectory")

    @state
    def shooting_algae(self, initial_call: bool) -> None:
        if initial_call:
            self.algae_shooter.shoot()
        elif not self.algae_shooter.is_executing:
            self.next_state("tracking_trajectory")
