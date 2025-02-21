from magicbot import feedback, tunable
from rev import LimitSwitchConfig, SparkMax, SparkMaxConfig

from ids import SparkId


class ClimberComponent:
    target_speed = 0.0
    winch_voltage = tunable(12.0)

    def __init__(self) -> None:
        self.motor = SparkMax(SparkId.CLIMBER, SparkMax.MotorType.kBrushless)

        motor_config = SparkMaxConfig()
        motor_config.inverted(True)
        motor_config.setIdleMode(SparkMaxConfig.IdleMode.kBrake)
        motor_config.openLoopRampRate(1.5)
        motor_config.limitSwitch.forwardLimitSwitchType(
            LimitSwitchConfig.Type.kNormallyOpen
        )
        motor_config.limitSwitch.reverseLimitSwitchType(
            LimitSwitchConfig.Type.kNormallyOpen
        )
        motor_config.limitSwitch.forwardLimitSwitchEnabled(True)
        motor_config.limitSwitch.reverseLimitSwitchEnabled(True)

        self.motor.configure(
            motor_config,
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )

    def deploy(self) -> None:
        self.target_speed = self.winch_voltage

    def retract(self) -> None:
        self.target_speed = -self.winch_voltage

    @feedback
    def is_deployed(self) -> bool:
        return self.motor.getForwardLimitSwitch().get()

    @feedback
    def is_retracted(self) -> bool:
        return self.motor.getReverseLimitSwitch().get()

    def elevation(self) -> float:
        return 0.0
        # current place holder

    def execute(self) -> None:
        self.motor.setVoltage(self.target_speed)
        self.target_speed = 0.0
