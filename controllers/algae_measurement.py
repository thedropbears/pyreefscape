import math

from magicbot import StateMachine, state

from components.algae_manipulator import AlgaeManipulatorComponent


class AlgaeMeasurement(StateMachine):
    algae_manipulator: AlgaeManipulatorComponent

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
            self.starting_position = self.algae_manipulator.get_injector_position()

        self.algae_manipulator.measure_algae()

        if math.isclose(
            self.algae_manipulator.get_injector_velocity(), 0.0, abs_tol=0.1
        ):
            self.stopped_counter += 1

        else:
            self.stopped_counter = 0

        if self.stopped_counter >= 5:
            self.algae_manipulator.algae_size = (
                self.algae_manipulator.get_injector_position() - self.starting_position
            )
            self.stopped_counter = 0
            self.done()
