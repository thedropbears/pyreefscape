from magicbot import StateMachine, state, tunable

from components.feeler import FeelerComponent


class Feeler(StateMachine):
    feeler_component: FeelerComponent

    DETECT_SPEED = tunable(0.6)  # degrees per cycle
    START_ANGLE = tunable(90)
    START_OFFSET = tunable(17)

    def __init__(self):
        pass

    def feel(self):
        self.engage()

    @state(first=True, must_finish=True)
    def searching(self, initial_call: bool):
        if initial_call:
            self.current_feeler_algae = self.START_ANGLE + self.START_OFFSET
            self.feeler_component.algae_size = 0.0

        if self.current_feeler_angle >= 160:
            self.current_feeler_angle = 0.0
            self.feeler_component.algae_size = 0.0

        self.feeler_component.set_angle(self.current_feeler_angle)

        if self.feeler_component.touching_algae():
            self.next_state("found")
            return

        self.current_feeler_angle += self.DETECT_SPEED

    @state(must_finish=True)
    def found(self):
        self.feeler_component.algae_size = self.current_feeler_angle
        self.done()

    def done(self) -> None:
        super().done()
        self.feeler_component.set_angle(self.START_ANGLE)
        self.current_feeler_angle = self.START_ANGLE
