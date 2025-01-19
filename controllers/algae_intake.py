from magicbot import StateMachine, state

from components.algae_manipulator import AlgaeManipulatorComponent


class AlgaeIntake(StateMachine):
    algae_manipulator_component: AlgaeManipulatorComponent

    def __init__(self):
        pass

    @state(first=True)
    def intaking(self):
        if self.algae_manipulator_component.has_algae():
            self.done()
            return

        self.algae_manipulator_component.intake()

    def intake(self):
        self.engage()
