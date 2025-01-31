from magicbot import StateMachine, state

from components.feeler import FeelerComponent


class Feeler(StateMachine):
    feeler: FeelerComponent

    def __init__(self):
        pass

    def feel(self):
        self.engage()

    @state(first=True, must_finish=True)
    def searching(self):
        self.next_state("found")

    @state(must_finish=True)
    def found(self):
        self.done()

    def done(self) -> None:
        super().done()
        self.feeler.current_angle = self.feeler.START_ANGLE
        self.feeler.set_angle(self.feeler.current_angle)
