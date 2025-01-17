from magicbot import StateMachine, timed_state

from components.manipulator import ManipulatorComponent


class ManipulatorIntake(StateMachine):
    manipulator_component: ManipulatorComponent

    def __init__(self):
        pass

    @timed_state(duration=3.0, first=True, must_finish=True)
    def intaking(self):
        self.manipulator_component.intake()

    def intake(self):
        self.engage()
