import choreo
import choreo.trajectory
from choreo.trajectory import SwerveSample, SwerveTrajectory
from magicbot import AutonomousStateMachine, state
from wpimath.controller import PIDController
from wpimath.kinematics import ChassisSpeeds

from components.chassis import ChassisComponent
from utilities import game


class TestAutoBase(AutonomousStateMachine):
    chassis: ChassisComponent

    DISTANCE_TOLERANCE = 0.05  # metres

    def __init__(self, starting_trajectory: str) -> None:
        # We want to parameterise these by paths and potentially a sequence of events
        super().__init__()
        self.x_controller = PIDController(1.0, 0.0, 0.0)
        self.y_controller = PIDController(1.0, 0.0, 0.0)

        self.starting_time: float = 0

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

        # if RobotBase.isSimulation() and self.starting_pose is not None:
        if self.starting_pose is not None:
            self.chassis.set_pose(self.starting_pose)
        super().on_enable()

    @state(first=True)
    def initialising(self) -> None:
        # Copy path instances to be able to reset auto without reset robot

        self.next_state("tracking_trajectory")

    @state
    def tracking_trajectory(self, state_tm) -> None:
        final_pose = self.trajectory.get_final_pose(game.is_red())
        if final_pose is None:
            self.done()
            return

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
            + self.chassis.heading_controller.calculate(
                pose.rotation().radians(), sample.heading
            ),
        )

        # Apply the generated speeds
        self.chassis.drive_field(speeds.vx, speeds.vy, speeds.omega)
