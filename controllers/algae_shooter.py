import math

from magicbot import StateMachine, state, timed_state, tunable

from components.ballistics import BallisticsComponent
from components.chassis import ChassisComponent
from components.injector import InjectorComponent
from components.shooter import ShooterComponent
from components.wrist import WristComponent
from controllers.algae_measurement import AlgaeMeasurement
from controllers.floor_intake import FloorIntake
from controllers.reef_intake import ReefIntake


class AlgaeShooter(StateMachine):
    shooter_component: ShooterComponent
    injector_component: InjectorComponent
    ballistics_component: BallisticsComponent
    chassis: ChassisComponent
    wrist: WristComponent
    algae_measurement: AlgaeMeasurement
    floor_intake: FloorIntake
    reef_intake: ReefIntake

    SHOOT_ANGLE = tunable(-50.0)
    TOP_SHOOT_SPEED = tunable(60.0)
    BOTTOM_SHOOT_SPEED = tunable(65.0)
    use_ballistics = tunable(True)

    def __init__(self) -> None:
        pass

    def shoot(self) -> None:
        if self.floor_intake.is_executing:
            return
        if self.reef_intake.is_executing:
            return
        if (
            self.use_ballistics and not self.ballistics_component.is_in_range()
        ) or not self.ballistics_component.is_aligned():
            return
        self.engage()

    @state(first=True, must_finish=True)
    def preparing(self):
        if self.use_ballistics:
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
            and self.chassis.is_stationary()
        ):
            self.next_state(self.shooting)

    @timed_state(duration=0.2, must_finish=True)
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
