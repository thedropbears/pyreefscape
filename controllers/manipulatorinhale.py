from magicbot import StateMachine

from components.manipulator import ManipulatorComponent

class ManipulatorInhale(StateMachine):
    manipulator_component: ManipulatorComponent

    def __init__(self):
        pass

    def intakealgae(self):
        self.manipulator_component.intake()
        pass