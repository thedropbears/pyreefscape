from magicbot import StateMachine, timed_state

from components.algae_manipulator import AlgaeManipulatorComponent


class AlgaeIntake(StateMachine):
    algae_manipulator_component: AlgaeManipulatorComponent

    def __init__(self):
        pass

    @timed_state(duration=3.0, first=True, must_finish=True)
    def intaking(self):
        self.algae_manipulator_component.intake()

    def intake(self):
        self.engage()
