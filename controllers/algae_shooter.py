from magicbot import StateMachine, state, timed_state, tunable

from components.algae_manipulator import AlgaeManipulatorComponent
from components.chassis import ChassisComponent
from components.wrist import WristComponent


class AlgaeShooter(StateMachine):
    algae_manipulator_component: AlgaeManipulatorComponent
    chassis: ChassisComponent
    wrist: WristComponent

    SHOOT_ANGLE = tunable(40.0)
    SHOOT_SPEED = tunable(60.0)

    def __init__(self) -> None:
        pass

    def shoot(self) -> None:
        self.engage("tilting")

    @state(first=True, must_finish=True)
    def preparing(self):
        self.wrist.tilt_to(self.SHOOT_ANGLE)
        self.algae_manipulator_component.spin_flywheels(self.SHOOT_SPEED)
        # self.algae_manipulator_component.set_range(self.calculate_range_to_barge()

        if (
            self.wrist.at_setpoint()
            and self.algae_manipulator_component.flywheels_up_to_speed()
        ):
            self.next_state("shooting")

    @timed_state(duration=1, must_finish=True)
    def shooting(self) -> None:
        self.algae_manipulator_component.spin_flywheels(self.SHOOT_SPEED)
        self.algae_manipulator_component.inject()

        # self.algae_manipulator_component.set_range(self.calculate_range_to_barge())
