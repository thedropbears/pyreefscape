from magicbot import StateMachine, feedback, state, timed_state

from components.injector import InjectorComponent
from components.shooter import ShooterComponent
from utilities.scalers import scale_value


class AlgaeMeasurement(StateMachine):
    shooter_component: ShooterComponent
    injector_component: InjectorComponent

    def __init__(self) -> None:
        self.injector_starting_positions = (0.0, 0.0)
        self.flywheel_starting_positions = (0.0, 0.0)
        self.measured_sizes: list[float] = []
        self.measured_raw_sizes: list[float] = []

    def measure(self) -> None:
        self.engage()

    @state(must_finish=True, first=True)
    def initialising(self) -> None:
        self.measured_sizes = []
        self.measured_raw_sizes = []
        if all(abs(v) <= 0.0001 for v in self.shooter_component.flywheel_speeds()):
            self.next_state("calculating")

    @timed_state(
        duration=0.5,
        next_state="measuring",
        must_finish=True,
    )
    def pre_measure(self) -> None:
        self.injector_component.desired_injector_voltage = -2.0

    @state(must_finish=True)
    def calculating(self) -> None:
        if len(self.measured_sizes) == 3:
            # Throw away the first one and average the last two
            self.shooter_component.algae_size = sum(self.measured_sizes[1:]) / (
                len(self.measured_sizes) - 1
            )
            self.next_state("recovering")
        else:
            self.next_state("pre_measure")

    @state(must_finish=True)
    def measuring(self, initial_call) -> None:
        if initial_call:
            self.injector_starting_positions = (
                self.injector_component.get_injector_positions()
            )

            self.flywheel_starting_positions = (
                self.shooter_component.flywheel_positions()
            )

        self.injector_component.measure_algae()

        if (
            sum(
                current - start
                for current, start in zip(
                    self.shooter_component.flywheel_positions(),
                    self.flywheel_starting_positions,
                )
            )
            > 0.01
        ):
            injector_position_delta = sum(
                self.injector_component.get_injector_positions()
            ) - sum(self.injector_starting_positions)
            self.measured_raw_sizes.append(injector_position_delta)
            self.measured_sizes.append(
                scale_value(injector_position_delta, 7.2, 4.9, 16.0, 17.0)
            )

            self.next_state("calculating")

    @timed_state(duration=0.5, must_finish=True)
    def recovering(self) -> None:
        self.injector_component.desired_injector_voltage = -2.0

    @feedback
    def raw_ball_measurments(self) -> list[float]:
        return self.measured_raw_sizes
