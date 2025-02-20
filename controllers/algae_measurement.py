from magicbot import StateMachine, state, timed_state

from components.algae_manipulator import AlgaeManipulatorComponent
from utilities.scalers import scale_value


class AlgaeMeasurement(StateMachine):
    algae_manipulator_component: AlgaeManipulatorComponent

    def __init__(self) -> None:
        self.injector_1_starting_position = 0.0
        self.injector_2_starting_position = 0.0
        self.flywheel_top_starting_position = 0.0
        self.flywheel_bottom_starting_position = 0.0
        self.measured_sizes = []

    def measure(self) -> None:
        self.engage()

    @state(must_finish=True, first=True)
    def initialising(self) -> None:
        self.measured_sizes = []

    @timed_state(
        duration=0.5,
        next_state="measuring",
        must_finish=True,
    )
    def pre_measure(self) -> None:
        pass

    @state(must_finish=True)
    def calculating(self) -> None:
        if len(self.measured_sizes) == 3:
            # Throw away the first one and average the last two
            self.algae_manipulator_component.algae_size = sum(
                self.measured_sizes[1:]
            ) / (len(self.measured_sizes) - 1)
            self.done()
        else:
            self.next_state("pre_measure")

    @state(must_finish=True)
    def measuring(self, initial_call) -> None:
        if initial_call:
            self.injector_1_starting_position = (
                self.algae_manipulator_component.get_injector_1_position()
            )
            self.injector_2_starting_position = (
                self.algae_manipulator_component.get_injector_2_position()
            )

            self.flywheel_top_starting_position = (
                self.algae_manipulator_component.flywheel_top_position()
            )
            self.flywheel_bottom_starting_position = (
                self.algae_manipulator_component.flywheel_bottom_position()
            )

        self.algae_manipulator_component.measure_algae()

        if (
            self.algae_manipulator_component.flywheel_top_position()
            - self.flywheel_top_starting_position
        ) >= 0.001 and (
            self.algae_manipulator_component.flywheel_bottom_position()
            - self.flywheel_bottom_starting_position
        ) >= 0.001:
            injector_position_delta = (
                self.algae_manipulator_component.get_injector_1_position()
                - self.injector_1_starting_position
            ) + (
                self.algae_manipulator_component.get_injector_2_position()
                - self.injector_2_starting_position
            )
            self.measured_sizes.append(
                scale_value(injector_position_delta, 4.55, 1.62, 16.0, 17.0)
            )

            self.next_state("ball_retraction")

    @state(must_finish=True)
    def ball_retraction(self) -> None:
        self.algae_manipulator_component.intake()
        if self.algae_manipulator_component.has_algae():
            self.next_state("calculating")
