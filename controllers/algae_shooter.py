import math

from magicbot import StateMachine, state, timed_state, tunable

from components.algae_manipulator import AlgaeManipulatorComponent
from components.ballistics import BallisticsComponent
from components.chassis import ChassisComponent
from components.wrist import WristComponent


class AlgaeShooter(StateMachine):
    algae_manipulator_component: AlgaeManipulatorComponent
    ballistics_component: BallisticsComponent
    chassis: ChassisComponent
    wrist: WristComponent

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
        if self.use_ballistics:
            if initial_call and (
                not self.ballistics_component.is_in_range()
                or not self.ballistics_component.is_aligned()
            ):
                self.done()
                return
            solution = self.ballistics_component.current_solution()
            self.wrist.tilt_to(solution.inclination)
            self.algae_manipulator_component.spin_flywheels(
                solution.top_speed, solution.bottom_speed
            )
        else:
            self.wrist.tilt_to(math.radians(self.SHOOT_ANGLE))
            self.algae_manipulator_component.spin_flywheels(
                self.TOP_SHOOT_SPEED, self.BOTTOM_SHOOT_SPEED
            )

        if (
            self.wrist.at_setpoint()
            and self.algae_manipulator_component.top_flywheels_up_to_speed()
            and self.algae_manipulator_component.bottom_flywheels_up_to_speed()
        ):
            self.next_state("shooting")

    @timed_state(duration=1, must_finish=True)
    def shooting(self) -> None:
        if self.use_ballistics:
            solution = self.ballistics_component.current_solution()
            self.algae_manipulator_component.spin_flywheels(
                solution.top_speed, solution.bottom_speed
            )
        else:
            self.algae_manipulator_component.spin_flywheels(
                self.TOP_SHOOT_SPEED, self.BOTTOM_SHOOT_SPEED
            )
        self.algae_manipulator_component.inject()

    def done(self) -> None:
        super().done()
        self.wrist.go_to_neutral()
