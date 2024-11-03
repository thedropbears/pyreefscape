import enum


@enum.unique
class TalonId(enum.IntEnum):
    """CAN ID for CTRE Talon motor controllers (e.g. Talon FX, Talon SRX)."""

    drive_1 = 1
    steer_1 = 5

    drive_2 = 2
    steer_2 = 6

    drive_3 = 3
    steer_3 = 7

    drive_4 = 4
    steer_4 = 8


@enum.unique
class CancoderId(enum.IntEnum):
    """CAN ID for CTRE CANcoder."""

    swerve_1 = 1
    swerve_2 = 2
    swerve_3 = 3
    swerve_4 = 4


@enum.unique
class SparkId(enum.IntEnum):
    """CAN ID for REV SPARK motor controllers (Spark Max, Spark Flex)."""


@enum.unique
class DioChannel(enum.IntEnum):
    """roboRIO Digital I/O channel number."""


@enum.unique
class PwmChannel(enum.IntEnum):
    """roboRIO PWM output channel number."""
