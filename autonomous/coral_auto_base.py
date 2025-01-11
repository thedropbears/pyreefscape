from magicbot import AutonomousStateMachine, state

from components.chassis import ChassisComponent


class CoralAutoBase(AutonomousStateMachine):
    chassis: ChassisComponent

    def __init__(self) -> None:
        # We want to parameterise these by paths and potentially a sequence of events
        super().__init__()

    def setup(self) -> None:
        #  setup path tracking controllers

        # init any other defaults
        pass

    def on_enable(self) -> None:
        # configure defaults for pose in sim

        return super().on_enable()

    @state(first=True)
    def initialising(self) -> None:
        # Copy path instances to be able to reset auto without reset robot

        self.next_state("tracking_trajectory")

    @state
    def tracking_trajectory(self) -> None:
        # get next leg on entry

        # track path

        # next state on leg completion
        self.next_state("scoring_coral")

    @state
    def scoring_coral(self) -> None:
        pass
