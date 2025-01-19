from magicbot import StateMachine, state, timed_state

from components.algae_manipulator import AlgaeManipulatorComponent
from components.chassis import ChassisComponent


class AlgaeShooter(StateMachine):
    algae_manipulator_component: AlgaeManipulatorComponent
    chassis: ChassisComponent

    def __init__(self) -> None:
        pass

    def shoot(self) -> None:
        self.engage()

    @state(first=True)
    def spinning_up(self) -> None:
        # self.algae_manipulator_component.set_range(self.calculate_range_to_barge())
        # if self.algae_manipulator_component.flywheels_at_speed():
        self.next_state("shooting")

    @timed_state(duration=1, must_finish=True)
    def shooting(self) -> None:
        self.algae_manipulator_component.inject()

        # self.algae_manipulator_component.set_range(self.calculate_range_to_barge())
