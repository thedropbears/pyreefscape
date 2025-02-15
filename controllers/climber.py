from magicbot import StateMachine, state

from components.climber import ClimberComponent


class ClimberStateMachine(StateMachine):
    climber: ClimberComponent

    def __init__(self):
        pass

    def climb(self) -> None:
        self.engage()

    @state(first=True, must_finish=True)
    def retracting(self) -> None:
        if self.climber.is_retracted():
            self.done()
            return
        self.climber.retract()

    def done(self) -> None:
        super().done()
