import math

from magicbot import StateMachine, feedback, state, tunable

from components.algae_manipulator import AlgaeManipulatorComponent
from components.chassis import ChassisComponent
from components.wrist import WristComponent
from utilities import game


class ReefIntake(StateMachine):
    algae_manipulator_component: AlgaeManipulatorComponent
    wrist: WristComponent
    chassis: ChassisComponent

    L2_INTAKE_ANGLE = tunable(math.radians(-40.0))
    L3_INTAKE_ANGLE = tunable(math.radians(-10.0))

    RETREAT_DISTANCE = tunable(0.6)

    FEELER_DETECT_SPEED = tunable(0.6)  # degrees per cycle
    FEELER_START_ANGLE = tunable(90)
    FEELER_START_OFFEST = tunable(17)

    current_feeler_angle = 0.0

    FEELER_START_OFFSET = tunable(17)

    def __init__(self):
        self.last_l3 = False

    def intake(self) -> None:
        self.engage()

    @state(first=True, must_finish=True)
    def intaking(self, initial_call: bool):
        if self.algae_manipulator_component.has_algae():
            self.next_state("safing")

        current_is_L3 = self.is_L3()

        if self.last_l3 != current_is_L3 or initial_call:
            if current_is_L3:
                self.wrist.tilt_to(self.L3_INTAKE_ANGLE)
            else:
                self.wrist.tilt_to(self.L2_INTAKE_ANGLE)
            self.last_l3 = current_is_L3

        self.algae_manipulator_component.intake()

    @state(must_finish=True)
    def safing(self, initial_call: bool):
        has_touched_algae = False
        if initial_call:
            self.origin_robot_pose = self.chassis.get_pose()

            self.current_feeler_algae = (
                self.FEELER_START_ANGLE + self.FEELER_START_OFFEST
            )
            self.algae_manipulator_component.algae_size = 0.0

        if self.current_feeler_angle >= 160:
            self.current_feeler_angle = 0.0
            self.algae_manipulator_component.algae_size = 0.0

        self.algae_manipulator_component.set_feeler(self.current_feeler_angle, False)

        robot_pose = self.chassis.get_pose()

        distance = self.origin_robot_pose.translation() - robot_pose.translation()

        if self.algae_manipulator_component.feeler_touching_algae():
            self.algae_manipulator_component.algae_size = self.current_feeler_angle
            self.algae_manipulator_component.set_feeler(self.FEELER_START_ANGLE)
            self.current_feeler_angle = self.FEELER_START_ANGLE
            has_touched_algae = True

        if distance.norm() >= self.RETREAT_DISTANCE and (
            self.algae_manipulator_component.feeler_touching_algae()
            or has_touched_algae
        ):
            self.done()

        if (
            not self.algae_manipulator_component.feeler_touching_algae()
            and not has_touched_algae
        ):
            self.current_feeler_angle += self.FEELER_DETECT_SPEED

    @feedback
    def is_L3(self) -> bool:
        return game.is_L3(game.nearest_reef_tag(self.chassis.get_pose()))

    def done(self) -> None:
        super().done()
        self.wrist.go_to_neutral()
        self.current_feeler_angle = self.FEELER_START_ANGLE
        self.algae_manipulator_component.set_feeler(self.current_feeler_angle, False)
