class Climber:
    SPEED = 1.0

    def __init__(self) -> None:
        self.will_reset_to = 0.0

    def deploy(self) -> None:
        self.will_reset_to = Climber.SPEED

    def retract(self) -> None:
        self.will_reset_to = -Climber.SPEED

    def is_deployed(self) -> bool:
        return False

    def is_retracted(self) -> bool:
        return False

    def elevation(self) -> float:
        return 0.0
        # current place holder

    def execute(self) -> None:
        if (
            self.is_deployed()
            and self.will_reset_to > 0
            or self.is_retracted()
            and self.will_reset_to < 0
        ):
            self.target_speed = 0.0
            # stop motor if fully retracted
