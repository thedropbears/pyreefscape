from magicbot import StateMachine, state, timed_state

from components.chassis import ChassisComponent
from components.manipulator import ManipulatorComponent


class ManipulatorInject(StateMachine):
    manipulator_component: ManipulatorComponent
    chassis_component: ChassisComponent

    def __init__(self) -> None:
        self.set_range = 0.0
        self.pose = 0.0

    def shoot(self) -> None:
        self.engage()

    @state(first=True)
    def spinning_up(self) -> None:
        if self.manipulator_component.flywheels_at_speed():
            self.next_state("shooting")

    @timed_state(duration=1, must_finish=True)
    def shooting(self) -> None:
        self.manipulator_component.inject()

        self.pose = self.chassis_component.get_pose()

        self.set_range = 1.0
