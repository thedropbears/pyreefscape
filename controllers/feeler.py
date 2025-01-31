from magicbot import StateMachine, state

from components.feeler import FeelerComponent


class Feeler(StateMachine):
    feeler: FeelerComponent

    def __init__(self):
        pass

    def feel(self):
        self.engage()

    @state(first=True, must_finish=True)
    def searching(self, initial_call: bool):
        if initial_call:
            self.current_feeler_algae = (
                self.feeler.START_ANGLE + self.feeler.START_OFFEST
            )
            self.feeler.algae_size = 0.0

        if self.current_feeler_angle >= 160:
            self.current_feeler_angle = 0.0
            self.feeler.algae_size = 0.0

        self.feeler.set_angle(self.current_feeler_angle)

        if self.feeler.touching_algae():
            self.next_state("found")
            return

        self.current_feeler_angle += self.feeler.DETECT_SPEED

    @state(must_finish=True)
    def found(self):
        self.feeler.algae_size = self.current_feeler_angle
        self.done()

    def done(self) -> None:
        super().done()
        self.feeler.set_angle(self.feeler.START_ANGLE)
        self.current_feeler_angle = self.feeler.START_ANGLE
