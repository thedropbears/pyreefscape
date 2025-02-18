import math

import choreo
from choreo.trajectory import SwerveSample
from magicbot import AutonomousStateMachine, state
from wpilib import RobotBase
from wpimath.controller import PIDController
from wpimath.geometry import Pose2d
from wpimath.kinematics import ChassisSpeeds

from components.algae_manipulator import AlgaeManipulatorComponent
from components.chassis import ChassisComponent
from controllers.algae_shooter import AlgaeShooter
from controllers.coral_placer import CoralPlacer
from controllers.reef_intake import ReefIntake
from utilities import game


class AutoBase(AutonomousStateMachine):
    algae_shooter: AlgaeShooter
    coral_placer: CoralPlacer
    reef_intake: ReefIntake

    algae_manipulator_component: AlgaeManipulatorComponent
    chassis: ChassisComponent

    DISTANCE_TOLERANCE = 0.05  # metres

    def __init__(self, trajectory_names: list[str]) -> None:
        # We want to parameterise these by paths and potentially a sequence of events
        super().__init__()
        self.x_controller = PIDController(1.0, 0.0, 0.0)
        self.y_controller = PIDController(1.0, 0.0, 0.0)
        self.heading_controller = PIDController(1.0, 0, 0)
        self.heading_controller.enableContinuousInput(-math.pi, math.pi)

        self.current_leg = -1
        self.trajectories = []
        for trajectory_name in trajectory_names:
            try:  # noqa: SIM105
                self.trajectories.append(choreo.load_swerve_trajectory(trajectory_name))
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

        if (
            self.current_leg > 0
            and not self.algae_manipulator_component.should_be_holding_algae()
        ):
            self.reef_intake.intake()

        if distance < self.DISTANCE_TOLERANCE:
            # First leg is to score coral, then we run cycles of pick up -> shoot
            if self.current_leg == 0:
                self.next_state("scoring_coral")
            elif self.algae_manipulator_component.has_algae():
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
    def driving_to_coral(self) -> None:
        # TODO - Set up coral placer in correct position
        self.next_state("tracking_trajectory")

    @state
    def scoring_coral(self, initial_call: bool) -> None:
        if initial_call:
            self.coral_placer.place()
        elif not self.coral_placer.is_executing:
            self.next_state("retreating")

    @state
    def retreating(self) -> None:
        self.next_state("tracking_trajectory")

    @state
    def intaking_algae(self) -> None:
        if self.algae_manipulator_component.should_be_holding_algae():
            self.next_state("tracking_trajectory")

    @state
    def shooting_algae(self, initial_call: bool) -> None:
        if initial_call:
            self.algae_shooter.shoot()
        elif not self.algae_shooter.is_executing:
            self.next_state("tracking_trajectory")
