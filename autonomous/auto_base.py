import math

import choreo
import wpilib
from choreo.trajectory import SwerveSample, SwerveTrajectory
from magicbot import AutonomousStateMachine, state, timed_state
from wpilib import RobotBase
from wpimath.controller import PIDController
from wpimath.geometry import Pose2d
from wpimath.kinematics import ChassisSpeeds

from components.chassis import ChassisComponent
from components.coral_depositor import CoralDepositorComponent
from components.injector import InjectorComponent
from components.shooter import ShooterComponent
from controllers.algae_shooter import AlgaeShooter
from controllers.reef_intake import ReefIntake
from utilities import game

x_controller = PIDController(2.0, 0.0, 0.0)
y_controller = PIDController(2.0, 0.0, 0.0)

wpilib.SmartDashboard.putData("Auto X PID", x_controller)
wpilib.SmartDashboard.putData("Auto Y PID", y_controller)


class AutoBase(AutonomousStateMachine):
    algae_shooter: AlgaeShooter
    reef_intake: ReefIntake

    coral_depositor_component: CoralDepositorComponent
    shooter_component: ShooterComponent
    injector_component: InjectorComponent
    chassis: ChassisComponent

    field: wpilib.Field2d

    DISTANCE_TOLERANCE = 0.1  # metres
    ANGLE_TOLERANCE = math.radians(3)
    CORAL_DISTANCE_TOLERANCE = 0.2  # metres

    def __init__(self, trajectory_names: list[str]) -> None:
        # We want to parameterise these by paths and potentially a sequence of events
        super().__init__()

        self.current_leg = -1
        self.starting_pose = None
        self.trajectories: list[SwerveTrajectory] = []
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

    def _get_full_path_poses(self) -> list[Pose2d]:
        """Get a list of poses for the full path for display."""
        return [
            sample.get_pose()
            for trajectory in self.trajectories
            for sample in trajectory.get_samples()
        ]

    def display_trajectory(self) -> None:
        self.field.getObject("trajectory").setPoses(self._get_full_path_poses())

    def on_disable(self) -> None:
        super().on_disable()
        self.field.getObject("trajectory").setPoses([])

    @state(first=True)
    def initialising(self) -> None:
        # Add any tasks that need doing first
        self.reef_intake.holding_coral = True
        self.chassis.do_smooth = False
        self.next_state("tracking_trajectory")

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

        if self.current_leg % 2 != 0:
            self.algae_shooter.shoot()
        else:
            self.reef_intake.intake()
        # Check if we are close enough to deposit coral
        if distance < self.CORAL_DISTANCE_TOLERANCE:
            self.reef_intake.holding_coral = False

        if (
            distance < self.DISTANCE_TOLERANCE
            and math.isclose(angle_error, 0.0, abs_tol=self.ANGLE_TOLERANCE)
            and state_tm > self.trajectories[self.current_leg].get_total_time() / 2.0
        ):
            # run cycles of pick up -> shoot
            if self.injector_component.has_algae():
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
            + self.chassis.heading_controller.calculate(
                pose.rotation().radians(), sample.heading
            ),
        )

        # Apply the generated speeds
        self.chassis.drive_field(speeds.vx, speeds.vy, speeds.omega)

    @timed_state(duration=1.0, next_state="tracking_trajectory")
    def intaking_algae(self) -> None:
        if self.current_leg == 0:
            self.coral_depositor_component.deposit()
        if self.injector_component.has_algae():
            self.coral_depositor_component.tuck()
            self.next_state("tracking_trajectory")

    @timed_state(duration=1.0, next_state="tracking_trajectory")
    def shooting_algae(self) -> None:
        self.algae_shooter.shoot()

        if not self.algae_shooter.is_executing:
            self.next_state("tracking_trajectory")
