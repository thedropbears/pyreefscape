import math

from magicbot import StateMachine, state

from components.chassis import ChassisComponent
from components.climber import ClimberComponent
from utilities.game import cage_pos, is_red


class ClimberStateMachine(StateMachine):
    climber: ClimberComponent
    chassis: ChassisComponent

    def __init__(self):
        self.heading_to_cage = 0.0

    def deploy(self) -> None:
        self.engage("deploying")

    def retract(self) -> None:
        self.engage("retracting", force=True)

    @state(first=True, must_finish=True)
    def deploying(self, initial_call) -> None:
        if initial_call:
            self.climber.go_to_deploy()

        cage_positions = cage_pos(is_red())
        closest_cage_position = cage_positions[0]
        closest_cage_dist = 999.0
        robot_position = self.chassis.get_pose().translation()

        for cage_position in cage_positions:
            dist = robot_position.distance(cage_position)
            if dist < closest_cage_dist:
                closest_cage_dist = dist
                closest_cage_position = cage_position

        if closest_cage_dist > 1.0:
            self.heading_to_cage = math.atan2(
                closest_cage_position.y - robot_position.y,
                closest_cage_position.x - robot_position.x,
            )
        self.chassis.snap_to_heading(self.heading_to_cage)

    @state(must_finish=True)
    def retracting(self) -> None:
        self.climber.go_to_retract()

    def done(self) -> None:
        super().done()
