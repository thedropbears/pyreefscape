from magicbot import StateMachine, feedback, state, tunable

from components.injector import InjectorComponent
from components.shooter import ShooterComponent
from utilities.functions import clamp
from utilities.game import ALGAE_MAX_DIAMETER, ALGAE_MIN_DIAMETER
from utilities.scalers import scale_value


class AlgaeMeasurement(StateMachine):
    shooter_component: ShooterComponent
    injector_component: InjectorComponent

    retraction_voltage = tunable(-4.0)
    number_of_iterations = tunable(1)

    def __init__(self) -> None:
        self.injector_starting_positions = (0.0, 0.0)
        self.flywheel_starting_positions = (0.0, 0.0)
        self.measured_sizes: list[float] = []
        self.measured_raw_sizes: list[float] = []
        self._tm = 0.0

    def measure(self) -> None:
        self.engage()

    @state(must_finish=True, first=True)
    def initialising(self) -> None:
        self.measured_sizes = []
        self.measured_raw_sizes = []
        if all(abs(v) <= 0.0001 for v in self.shooter_component.flywheel_speeds()):
            self.next_state(self.calculating)

    @state(must_finish=True)
    def pre_measure(self, initial_call, state_tm) -> None:
        if self.retract(initial_call, state_tm):
            self.next_state(self.measuring)

    @state(must_finish=True)
    def calculating(self) -> None:
        if len(self.measured_sizes) == self.number_of_iterations:
            # Simple average
            self.shooter_component.algae_size = sum(self.measured_sizes) / (
                len(self.measured_sizes)
            )
            self.next_state(self.recovering)
        else:
            self.next_state(self.pre_measure)

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
                clamp(
                    scale_value(
                        injector_position_delta,
                        7.5,
                        5.5,
                        ALGAE_MIN_DIAMETER,
                        ALGAE_MAX_DIAMETER,
                    ),
                    ALGAE_MIN_DIAMETER,
                    ALGAE_MAX_DIAMETER,
                )
            )

            self.next_state(self.calculating)

    @state(must_finish=True)
    def recovering(self, initial_call, state_tm) -> None:
        if self.retract(initial_call, state_tm):
            self.done()

    def retract(self, initial_call, state_tm) -> bool:
        self.injector_component.desired_injector_voltage = self.retraction_voltage
        if initial_call:
            self._tm = state_tm
            # We cannot be ready on the first iteration
            return False
        # TODO determine the correct speed threshold
        dt = state_tm - self._tm
        return (
            all(
                abs(v) < 0.01 for v in self.injector_component.get_injector_velocities()
            )
            or dt > 0.5
        )

    @feedback
    def raw_ball_measurments(self) -> list[float]:
        return self.measured_raw_sizes
