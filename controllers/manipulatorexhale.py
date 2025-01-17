from magicbot import StateMachine

from components.manipulator import ManipulatorComponent

class ManipulatorExhale(StateMachine):
    manipulator_component: ManipulatorComponent

    def __init__(self):
        pass

    def exhale(self):
        self.engage()
    
    @state(first=True)
    def spiningflywheels(self):
        self.manipulator_component.spin_flywheels()

    @timedstate(duration=1, must_finish=True)
    def exhaling(self):
        self.manipulator_component.inject()