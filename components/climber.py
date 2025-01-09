class Climber:
    def __init__(self) -> None:
        self.deployed = False
        self.retracted = False
        self.target_speed = 0.0

    def deploy(self) -> None:
        self.target_speed = 1.0

    def retract(self) -> None:
        self.target_speed = -1.0

    def is_deployed(self) -> bool:
        return self.deployed

    def is_retracted(self) -> bool:
        return self.retracted

    def elevation(self) -> float:
        return 0.0
        # current place holder

    def execute(self) -> None:
        if (
            self.is_deployed()
            and self.target_speed > 0
            or self.is_retracted()
            and self.target_speed < 0
        ):
            self.target_speed = 0.0
            # stop motor if fully retracted
