from magicbot import StateMachine, state, tunable

from components.feeler import FeelerComponent


class Feeler(StateMachine):
    feeler_component: FeelerComponent

    DETECT_SPEED = tunable(1.0)  # degrees per cycle
    START_ANGLE = tunable(45)
    START_OFFSET = tunable(45)

    def __init__(self):
        pass

    def feel(self):
        self.engage()

    @state(first=True, must_finish=True)
    def searching(self, initial_call: bool):
        if initial_call:
            self.feeler_component.current_angle = self.START_ANGLE + self.START_OFFSET
            self.feeler_component.algae_size = 0.0

        if self.feeler_component.current_angle >= 160:
            self.feeler_component.current_angle = 0.0
            self.feeler_component.algae_size = 0.0

        self.feeler_component.set_angle(self.feeler_component.current_angle)

        if self.feeler_component.touching_algae():
            self.next_state("found")
            return

        self.feeler_component.current_angle += self.DETECT_SPEED

    @state(must_finish=True)
    def found(self):
        self.feeler_component.algae_size = self.feeler_component.current_angle
        self.done()

    def done(self) -> None:
        super().done()
        self.feeler_component.set_angle(self.START_ANGLE)
        self.feeler_component.current_angle = self.START_ANGLE
