import time

import wpilib
from magicbot import feedback
from wpilib import AddressableLED, Color, LEDPattern

from ids import PwmChannel
from utilities.game import is_red

RESET_TIMEOUT = 2.0


class LightStrip:
    def __init__(self, strip_length: int = 5) -> None:
        self.leds = AddressableLED(PwmChannel.LIGHT_STRIP)
        self.leds.setLength(strip_length)

        self.strip_data = [AddressableLED.LEDData()] * strip_length

        self.pattern: LEDPattern = LEDPattern.solid(Color.kBlack)

        self.leds.setData(self.strip_data)
        self.leds.start()

        self.last_update_time = time.monotonic()

    @feedback
    def is_red_right(self) -> bool:
        return is_red()

    def team_colour(self) -> None:
        if is_red():
            self.pattern = LEDPattern.solid(wpilib.Color.kRed)
        else:
            self.pattern = LEDPattern.solid(wpilib.Color.kBlue)
        self.keep_alive()

    def facing_in_range(self) -> None:
        self.pattern = LEDPattern.solid(wpilib.Color.kGreen)
        self.keep_alive()

    def not_facing_in_range(self) -> None:
        self.pattern = LEDPattern.blink(LEDPattern.solid(wpilib.Color.kYellow), 0.5)
        self.keep_alive()

    def not_in_range(self) -> None:
        self.pattern = LEDPattern.solid(wpilib.Color.kRed)
        self.keep_alive()

    def too_close_to_reef(self) -> None:
        self.pattern = LEDPattern.blink(LEDPattern.solid(wpilib.Color.kOrange), 0.5)
        self.keep_alive()

    def keep_alive(self) -> None:
        # Refresh the timer to stop the LEDs being turned off
        self.last_update_time = time.monotonic()

    def execute(self) -> None:
        if time.monotonic() - self.last_update_time > RESET_TIMEOUT:
            self.pattern = LEDPattern.solid(wpilib.Color.kBlack)

        self.pattern.applyTo(
            self.strip_data, lambda idx, color: self.strip_data[idx].setLED(color)
        )
        self.leds.setData(self.strip_data)
