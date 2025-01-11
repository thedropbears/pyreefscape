from magicbot import StateMachine, timed_state

from components.coral_placer import CoralPlacerComponent


class CoralPlacer(StateMachine):
    coral_placer_compnent: CoralPlacerComponent

    def __init__(self):
        pass

    def place(self):
        self.engage()

    @timed_state(duration=2.0, first=True, must_finish=True)
    def placing(self):
        self.coral_placer_compnent.place()
