from magicbot import StateMachine

from components.manipulator import ManipulatorComponent

class ManipulatorExhale(StateMachine):
    manipulator_component: ManipulatorComponent

    def __init__(self):
        pass

    def exhale(self):
        self.manipulator_component.intake()

    def spiningflywheels(self):
        self.manipulator_component.spin_flywheels()

    def exhaling(self):
        self.manipulator_component.inject()