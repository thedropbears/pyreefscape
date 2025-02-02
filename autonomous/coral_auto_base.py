import choreo
from choreo.trajectory import SwerveTrajectory
from magicbot import AutonomousStateMachine, state
from wpilib import RobotBase
from wpimath.controller import PIDController
from wpimath.geometry import Pose2d
from wpimath.kinematics import ChassisSpeeds

from components.chassis import ChassisComponent
from controllers.coral_placer import CoralPlacer
from utilities import game


class CoralAutoBase(AutonomousStateMachine):
    coral_placer: CoralPlacer

    chassis: ChassisComponent

    DISTANCE_TOLERANCE = 0.05  # metres

    def __init__(self, trajectory_name: str) -> None:
        # We want to parameterise these by paths and potentially a sequence of events
        super().__init__()
        self.x_controller = PIDController(1.0, 0.0, 0.0)
        self.y_controller = PIDController(1.0, 0.0, 0.0)

        try:
            self.trajectory = choreo.load_swerve_trajectory(trajectory_name)
            self.starting_pose = self.trajectory.get_initial_pose(game.is_red())
        except ValueError:
            # If the trajectory is not found, ChoreoLib already prints to DriverStation
            self.trajectory = SwerveTrajectory("", [], [], [])
            self.starting_pose = None

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
        super().on_enable()

    def get_starting_pose(self) -> Pose2d | None:
        starting_pose = self.starting_pose
        if starting_pose is None:
            return None
        if game.is_red():
            starting_pose = game.field_flip_pose2d(starting_pose)
        return starting_pose

    @state(first=True)
    def initialising(self) -> None:
        # Copy path instances to be able to reset auto without reset robot

        self.next_state("tracking_trajectory")

    @state
    def tracking_trajectory(self, state_tm) -> None:
        # get next leg on entry
        current_pose = self.chassis.get_pose()
        final_pose = self.trajectory.get_final_pose(game.is_red())
        if final_pose is None:
            self.next_state("scoring_coral")
            return

        distance = current_pose.translation().distance(final_pose.translation())

        if distance < self.DISTANCE_TOLERANCE:
            self.next_state("scoring_coral")

        sample = self.trajectory.sample_at(state_tm, game.is_red())

        self.follow_trajectory(sample)

        # next state on leg completion

    def follow_trajectory(self, sample):
        # track path

        pose = self.chassis.get_pose()

        # Generate the next speeds for the robot
        speeds = ChassisSpeeds(
            sample.vx + self.x_controller.calculate(pose.X(), sample.x),
            sample.vy + self.y_controller.calculate(pose.Y(), sample.y),
            sample.omega
            + self.chassis.heading_controller.calculate(
                pose.rotation().radians(), sample.heading
            ),
        )

        # Apply the generated speeds
        self.chassis.drive_field(speeds.vx, speeds.vy, speeds.omega)

    @state
    def scoring_coral(self, initial_call: bool) -> None:
        if initial_call:
            self.coral_placer.place()
