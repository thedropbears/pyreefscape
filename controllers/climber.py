from magicbot import StateMachine, state

from components.chassis import ChassisComponent
from components.climber import ClimberComponent
from components.led_component import LightStrip


class ClimberStateMachine(StateMachine):
    status_lights: LightStrip
    climber: ClimberComponent
    chassis: ChassisComponent

    def __init__(self):
        self.has_deployed = False

    def on_disable(self) -> None:
        self.has_deployed = False
        super().on_disable()

    def deploy(self) -> None:
        self.engage("deploying", force=True)

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
