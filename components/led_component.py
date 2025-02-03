import dataclasses
import math
import time
from abc import ABC, abstractmethod
from collections.abc import Callable
from enum import Enum
from typing import Protocol

import wpilib
from magicbot import feedback

from ids import PwmChannel
from utilities.game import is_red

MAX_BRIGHTNESS = 50  # Integer value 0-255
Hsv = tuple[int, int, int]

FLASH_SPEED = 2
BREATHE_SPEED = 0.5
RAINBOW_SPEED = 1.5
RESET_TIMEOUT = 2.0


class HsvColour(Enum):
    RED = (0, 255, MAX_BRIGHTNESS)
    ORANGE = (20, 255, MAX_BRIGHTNESS)
    YELLOW = (30, 255, MAX_BRIGHTNESS)
    MAGENTA = (150, 255, MAX_BRIGHTNESS)
    BLUE = (120, 255, MAX_BRIGHTNESS)
    CYAN = (90, 255, MAX_BRIGHTNESS)
    GREEN = (60, 255, MAX_BRIGHTNESS)
    WHITE = (0, 0, MAX_BRIGHTNESS)
    OFF = (0, 0, 0)

    def with_hue(self, hue: int) -> Hsv:
        """
        Change the hue of the colour.

        Args:
            hue: The desired hue in [0,180).
        """
        _, s, v = self.value
        return (hue, s, v)

    def with_relative_brightness(self, multiplier: float) -> Hsv:
        """
        Scale the brightness of the colour.

        `multiplier` MUST be non-negative, and SHOULD be <= 1.
        """
        h, s, v = self.value
        return (h, s, int(v * multiplier))


class LightStrip:
    def __init__(self, strip_length: int = 5) -> None:
        self.leds = wpilib.AddressableLED(PwmChannel.LIGHT_STRIP)
        self.leds.setLength(strip_length)

        self.led_data = wpilib.AddressableLED.LEDData()
        self.strip_data = [self.led_data] * strip_length

        self.pattern: Pattern = Solid(HsvColour.CYAN)

        self.leds.setData(self.strip_data)
        self.leds.start()

        self.last_update_time = time.monotonic()

    @feedback
    def is_red_right(self) -> bool:
        return is_red()

    def team_colour(self) -> None:
        if is_red():
            self.pattern = Solid(HsvColour.RED)
        else:
            self.pattern = Solid(HsvColour.BLUE)
        self.keep_alive()

    def facing_in_range(self) -> None:
        self.pattern = Solid(HsvColour.GREEN)
        self.keep_alive()

    def not_facing_in_range(self) -> None:
        self.pattern = Flash(HsvColour.YELLOW)
        self.keep_alive()

    def not_in_range(self) -> None:
        self.pattern = Solid(HsvColour.RED)
        self.keep_alive()

    def too_close_to_reef(self) -> None:
        self.pattern = Flash(HsvColour.ORANGE)
        self.keep_alive()

    def rave(self) -> None:
        self.pattern = Rainbow(HsvColour.MAGENTA)
        self.keep_alive()

    def keep_alive(self) -> None:
        # Refresh the timer to stop the LEDs being turned off
        self.last_update_time = time.monotonic()

    def execute(self) -> None:
        if time.monotonic() - self.last_update_time > RESET_TIMEOUT:
            self.pattern = Solid(HsvColour.OFF)

        colour = self.pattern.update()
        self.led_data.setHSV(*colour)
        self.leds.setData(self.strip_data)


class Pattern(Protocol):
    def update(self) -> Hsv: ...


@dataclasses.dataclass
class TimeBasedPattern(ABC, Pattern):
    colour: HsvColour
    clock: Callable[[], float] = time.monotonic

    @abstractmethod
    def update(self) -> Hsv: ...


@dataclasses.dataclass
class Rainbow(TimeBasedPattern):
    speed: float = RAINBOW_SPEED

    def update(self) -> Hsv:
        hue = round(360 * (self.clock() / self.speed % 1))
        return self.colour.with_hue(hue)


@dataclasses.dataclass
class Solid(Pattern):
    colour: HsvColour

    def update(self) -> Hsv:
        return self.colour.value


@dataclasses.dataclass
class Flash(TimeBasedPattern):
    speed: float = FLASH_SPEED

    def update(self) -> Hsv:
        brightness = math.cos(self.speed * self.clock() * math.tau) >= 0
        return self.colour.with_relative_brightness(brightness)


@dataclasses.dataclass
class Breathe(TimeBasedPattern):
    speed: float = BREATHE_SPEED

    def update(self) -> Hsv:
        brightness = (math.sin(self.speed * self.clock() * math.tau) + 1) / 2
        return self.colour.with_relative_brightness(brightness)
