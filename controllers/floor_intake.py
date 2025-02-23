import math

from magicbot import StateMachine, state, tunable

from components.injector import InjectorComponent
from components.intake import IntakeComponent
from components.shooter import ShooterComponent
from components.wrist import WristComponent
from controllers.algae_measurement import AlgaeMeasurement


class FloorIntake(StateMachine):
    shooter_component: ShooterComponent
    injector_component: InjectorComponent
    wrist: WristComponent
    intake_component: IntakeComponent
    algae_measurement: AlgaeMeasurement

    HANDOFF_POSITION = tunable(math.radians(-112.0))

    def __init__(self):
        pass

    def intake(self) -> None:
        self.engage()

    @state(first=True, must_finish=True)
    def intaking(self, initial_call: bool):
        # This is different to the regular business logic below because these two things are mutually exclusive unless we have already got something
        if initial_call and self.injector_component.has_algae():
            self.done()
            return

        if self.injector_component.has_algae():
            self.next_state("measuring")
            return

        self.intake_component.intake()

        if initial_call:
            self.wrist.tilt_to(self.HANDOFF_POSITION)

        self.shooter_component.intake()
        self.injector_component.intake()

    @state(must_finish=True)
    def measuring(self, initial_call):
        if initial_call:
            self.algae_measurement.measure()
        elif not self.algae_measurement.is_executing:
            self.done()

    def done(self) -> None:
        super().done()
        self.wrist.go_to_neutral()
        self.intake_component.retract()
