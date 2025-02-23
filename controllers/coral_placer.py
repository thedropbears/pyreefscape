import math

from magicbot import StateMachine, state, tunable

from components.chassis import ChassisComponent
from components.coral_placer import CoralPlacerComponent
from components.wrist import WristComponent


class CoralPlacer(StateMachine):
    wrist: WristComponent
    chassis: ChassisComponent
    coral_placer: CoralPlacerComponent

    # In degrees for tuning, converted to radians in tilt-to call
    CORAL_PLACE_ANGLE = tunable(0.0)
    CORAL_LOWER_ANGLE = tunable(-20.0)

    RETREAT_DISTANCE = tunable(0.2)

    def __init__(self):
        self.coral_scored = False

    def place(self) -> None:
        self.engage("lowering", force=True)

    def lift(self) -> None:
        self.engage()

    def coral_is_scored(self) -> bool:
        return self.coral_scored

    @state(first=True, must_finish=True)
    def raising(self, initial_call: bool) -> None:
        if initial_call:
            self.coral_scored = False
            self.wrist.tilt_to(math.radians(self.CORAL_PLACE_ANGLE))

    @state(must_finish=True)
    def lowering(self, initial_call: bool) -> None:
        if initial_call:
            self.wrist.tilt_to(math.radians(self.CORAL_LOWER_ANGLE))
        if self.wrist.at_setpoint():
            self.coral_scored = True
            self.next_state(self.safing)

    @state(must_finish=True)
    def safing(self, initial_call: bool) -> None:
        if initial_call:
            self.score_pos = self.chassis.get_pose()

        current_pose = self.chassis.get_pose()

        distance = self.score_pos.translation().distance(current_pose.translation())

        if distance >= self.RETREAT_DISTANCE:
            self.wrist.go_to_neutral()

    @state(must_finish=True)
    def coral_place_retraction(self) -> None:
        self.coral_placer.coral_latch_open()
        self.done()
        return
