import math

from magicbot import StateMachine, state, timed_state, tunable

from components.algae_manipulator import AlgaeManipulatorComponent
from components.chassis import ChassisComponent
from components.wrist import WristComponent


class AlgaeShooter(StateMachine):
    algae_manipulator_component: AlgaeManipulatorComponent
    chassis: ChassisComponent
    wrist: WristComponent

    SHOOT_ANGLE = tunable(-50.0)
    SHOOT_SPEED = tunable(60.0)

    def __init__(self) -> None:
        pass

    def shoot(self) -> None:
        self.engage()

    @state(first=True, must_finish=True)
    def preparing(self, initial_call: bool):
        if initial_call:
            self.wrist.tilt_to(math.radians(self.SHOOT_ANGLE))
        self.algae_manipulator_component.spin_flywheels(self.SHOOT_SPEED)

        if (
            self.wrist.at_setpoint()
            and self.algae_manipulator_component.flywheels_up_to_speed()
        ):
            self.next_state("shooting")

    @timed_state(duration=1, must_finish=True)
    def shooting(self) -> None:
        self.algae_manipulator_component.spin_flywheels(self.SHOOT_SPEED)
        self.algae_manipulator_component.inject()

    def done(self) -> None:
        super().done()
        self.wrist.go_to_neutral()