import math

from magicbot import StateMachine, state, timed_state, tunable

from components.ballistics import BallisticsComponent
from components.chassis import ChassisComponent
from components.injector import InjectorComponent
from components.shooter import ShooterComponent
from components.wrist import WristComponent
from controllers.algae_measurement import AlgaeMeasurement


class AlgaeShooter(StateMachine):
    shooter_component: ShooterComponent
    injector_component: InjectorComponent
    ballistics_component: BallisticsComponent
    chassis: ChassisComponent
    wrist: WristComponent
    algae_measurement: AlgaeMeasurement

    SHOOT_ANGLE = tunable(-50.0)
    TOP_SHOOT_SPEED = tunable(60.0)
    BOTTOM_SHOOT_SPEED = tunable(65.0)
    use_ballistics = tunable(True)

    def __init__(self) -> None:
        pass

    def shoot(self) -> None:
        self.engage()

    @state(first=True, must_finish=True)
    def preparing(self, initial_call: bool):
        if self.algae_measurement.is_executing:
            self.done()
            return
        if self.use_ballistics:
            if initial_call and (
                not self.ballistics_component.is_in_range()
                or not self.ballistics_component.is_aligned()
            ):
                self.done()
                return
            solution = self.ballistics_component.current_solution()
            self.wrist.tilt_to(solution.inclination)
            self.shooter_component.spin_flywheels(
                solution.top_speed, solution.bottom_speed
            )
        else:
            self.wrist.tilt_to(math.radians(self.SHOOT_ANGLE))
            self.shooter_component.spin_flywheels(
                self.TOP_SHOOT_SPEED, self.BOTTOM_SHOOT_SPEED
            )

        if (
            self.wrist.at_setpoint()
            and self.shooter_component.top_flywheels_up_to_speed()
            and self.shooter_component.bottom_flywheels_up_to_speed()
        ):
            self.next_state("shooting")

    @timed_state(duration=1, must_finish=True)
    def shooting(self) -> None:
        if self.use_ballistics:
            solution = self.ballistics_component.current_solution()
            self.shooter_component.spin_flywheels(
                solution.top_speed, solution.bottom_speed
            )
        else:
            self.shooter_component.spin_flywheels(
                self.TOP_SHOOT_SPEED, self.BOTTOM_SHOOT_SPEED
            )
        self.injector_component.inject()

    def done(self) -> None:
        super().done()
        self.wrist.go_to_neutral()
