import enum


@enum.unique
class TalonId(enum.IntEnum):
    """CAN ID for CTRE Talon motor controllers (e.g. Talon FX, Talon SRX)."""

    DRIVE_1 = 1
    STEER_1 = 5

    DRIVE_2 = 2
    STEER_2 = 6

    DRIVE_3 = 3
    STEER_3 = 7

    DRIVE_4 = 4
    STEER_4 = 8


@enum.unique
class CancoderId(enum.IntEnum):
    """CAN ID for CTRE CANcoder."""

    SWERVE_1 = 1
    SWERVE_2 = 2
    SWERVE_3 = 3
    SWERVE_4 = 4


@enum.unique
class SparkId(enum.IntEnum):
    """CAN ID for REV SPARK motor controllers (Spark Max, Spark Flex)."""

    CORAL_SHOOTER = 20


@enum.unique
class DioChannel(enum.IntEnum):
    """roboRIO Digital I/O channel number."""


@enum.unique
class PwmChannel(enum.IntEnum):
    """roboRIO PWM output channel number."""
