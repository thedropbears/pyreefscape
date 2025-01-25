from magicbot import StateMachine, state, timed_state, tunable

from components.algae_manipulator import AlgaeManipulatorComponent
from components.chassis import ChassisComponent
from components.wrist import WristComponent


class AlgaeShooter(StateMachine):
    algae_manipulator_component: AlgaeManipulatorComponent
    chassis: ChassisComponent
    wrist: WristComponent

    L2_SHOOT_ANGLE = tunable(40.0)
    L3_SHOOT_ANGLE = tunable(40.0)

    def __init__(self) -> None:
        pass

    def shoot_L2(self) -> None:
        self.engage("raising_to_L2")

    def shoot_L3(self) -> None:
        self.engage("raising_to_L3")

    @state(first=True, must_finish=True)
    def raising_to_L2(self):
        self.wrist.tilt_to(self.L2_SHOOT_ANGLE)
        if self.wrist.at_setpoint():
            self.next_state("spinning_up")

    @state(must_finish=True)
    def raising_to_L3(self):
        self.wrist.tilt_to(self.L3_SHOOT_ANGLE)
        if self.wrist.at_setpoint():
            self.next_state("spinning_up")

    @state()
    def spinning_up(self) -> None:
        self.algae_manipulator_component.spin_flywheels()

        # self.algae_manipulator_component.set_range(self.calculate_range_to_barge()

        if self.algae_manipulator_component.flywheels_up_to_speed():
            self.next_state("shooting")

    @timed_state(duration=1, must_finish=True)
    def shooting(self) -> None:
        self.algae_manipulator_component.spin_flywheels()
        self.algae_manipulator_component.inject()

        # self.algae_manipulator_component.set_range(self.calculate_range_to_barge())
