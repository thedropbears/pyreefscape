import math

from magicbot import StateMachine, state, tunable

from components.injector import InjectorComponent
from components.intake import IntakeComponent
from components.shooter import ShooterComponent
from components.wrist import WristComponent


class FloorIntake(StateMachine):
    shooter_component: ShooterComponent
    injector_component: InjectorComponent
    wrist: WristComponent
    intake_component: IntakeComponent

    HANDOFF_POSITION = tunable(math.radians(-112.0))

    def __init__(self):
        self.intake_upper = False
        pass

    def intake(self) -> None:
        if self.injector_component.has_algae():
            return
        self.engage()

    @state(first=True, must_finish=True)
    def intaking(self):
        if self.injector_component.has_algae():
            self.injector_component.increment_segment()
            self.done()
            return

        self.intake_component.intake(self.intake_upper)

        self.wrist.tilt_to_shooter_FOR(self.HANDOFF_POSITION)

        self.shooter_component.intake()
        self.injector_component.intake()

    def done(self) -> None:
        self.wrist.go_to_neutral()
        self.intake_component.retract()
        self.intake_upper = False
        super().done()
