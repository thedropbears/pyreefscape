import math

from magicbot import StateMachine, state

from components.chassis import ChassisComponent
from components.climber import ClimberComponent
from components.led_component import LightStrip
from utilities.game import cage_pos, is_red


class ClimberStateMachine(StateMachine):
    status_lights: LightStrip
    climber: ClimberComponent
    chassis: ChassisComponent

    def __init__(self):
        self.has_deployed = False
        self.heading_to_cage = 0.0
        self.should_localize = True

    def on_disable(self) -> None:
        self.has_deployed = False
        super().on_disable()

    def deploy(self, *, localize: bool = True) -> None:
        self.should_localize = localize
        self.engage("deploying", force=True)

    def deploy_without_localization(self) -> None:
        self.deploy(localize=False)

    def retract(self) -> None:
        if self.has_deployed:
            self.engage("retracting", force=True)

    @state(first=True, must_finish=True)
    def deploying(self, initial_call) -> None:
        self.status_lights.climber_deploying(
            left_okay=self.climber.is_left_engaged(),
            right_okay=self.climber.is_right_engaged(),
        )
        if initial_call:
            self.climber.go_to_deploy()

        if self.climber.is_deployed():
            self.climber.stop_pid_update()
            self.has_deployed = True

        if self.should_localize:
            cage_positions = cage_pos(is_red())
            closest_cage_position = cage_positions[0]
            closest_cage_dist = 999.0
            robot_position = self.chassis.get_pose().translation()

            for cage_position in cage_positions:
                dist = robot_position.distance(cage_position)
                if dist < closest_cage_dist:
                    closest_cage_dist = dist
                    closest_cage_position = cage_position

            if closest_cage_dist > 0.5:
                self.heading_to_cage = math.atan2(
                    closest_cage_position.y - robot_position.y,
                    closest_cage_position.x - robot_position.x,
                )
            self.chassis.snap_to_heading(self.heading_to_cage)

    def is_ready_to_climb(self) -> bool:
        return self.climber.is_left_engaged() and self.climber.is_right_engaged()

    def pre_climb(self) -> None:
        self.engage("pre_climbing", force=True)

    @state(must_finish=True)
    def pre_climbing(self) -> None:
        self.status_lights.climber_pre_climb(
            left_okay=self.climber.is_left_engaged(),
            right_okay=self.climber.is_right_engaged(),
        )
        self.climber.start_pid_update()
        self.chassis.stop_snapping()
        self.climber.go_to_pre_climb()

    @state(must_finish=True)
    def retracting(self) -> None:
        self.status_lights.climber_retracting()
        self.climber.start_pid_update()
        self.chassis.stop_snapping()
        self.climber.go_to_retract()
        if self.climber.is_retracted():
            self.done()

    def done(self) -> None:
        self.chassis.stop_snapping()
        self.climber.stop()
        super().done()
