import math

from magicbot import StateMachine, state

from components.algae_manipulator import AlgaeManipulatorComponent


class AlgaeMeasurement(StateMachine):
    algae_manipulator_component: AlgaeManipulatorComponent

    def __init__(self) -> None:
        self.stopped_counter = 0
        self.starting_position = 0.0

    @state(
        first=True,
        must_finish=True,
    )
    def measuring(self, initial_call) -> None:
        if initial_call:
            self.stopped_counter = 0
            self.starting_position = (
                self.algae_manipulator_component.get_injector_position()
            )

        self.algae_manipulator_component.measure_algae()

        if math.isclose(
            self.algae_manipulator_component.get_injector_velocity(), 0.0, abs_tol=0.1
        ):
            self.stopped_counter += 1

        else:
            self.stopped_counter = 0

        if (
            self.algae_manipulator_component.bottom_flywheel_speed() >= 0.25
            and self.algae_manipulator_component.top_flywheel_speed() >= 0.25
        ):
            self.algae_manipulator_component.algae_size = (
                self.algae_manipulator_component.get_injector_position()
                - self.starting_position
            )
            self.stopped_counter = 0
            self.done()

    @state(must_finish=True)
    def ball_retraction(self) -> None:
        self.algae_manipulator_component.intake()
        if self.algae_manipulator_component.has_algae():
            self.done()
