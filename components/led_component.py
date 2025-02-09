import time

import wpilib
from magicbot import feedback
from wpilib import AddressableLED, Color, LEDPattern

from ids import PwmChannel
from utilities.game import is_red

RESET_TIMEOUT = 0.5
LED_SPACING = 1.0 / 144.0  # 144 LEDs per metre


def is_off(led_data: AddressableLED.LEDData) -> bool:
    return led_data.b == 0 and led_data.g == 0 and led_data.r == 0


class LightStrip:
    def __init__(self, strip_length: int = 5) -> None:
        self.leds = AddressableLED(PwmChannel.LIGHT_STRIP)
        self.leds.setLength(strip_length)

        self.strip_data = [AddressableLED.LEDData() for _ in range(strip_length)]

        self.pattern: LEDPattern = LEDPattern.solid(Color.kBlack)

        self.leds.setData(self.strip_data)
        self.leds.start()

        self.last_update_time = time.monotonic()
        self.is_reef_offset_flashing = False

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

    def rainbow(self) -> None:
        self.pattern = LEDPattern.rainbow(255, 100).scrollAtAbsoluteSpeed(
            0.1, LED_SPACING
        )
        self.keep_alive()

    def vision_timeout(self) -> None:
        self.pattern = LEDPattern.breathe(LEDPattern.solid(Color.kPurple), 2.0)
        self.keep_alive()

    def keep_alive(self) -> None:
        # Refresh the timer to stop the LEDs being turned off
        self.last_update_time = time.monotonic()

    def reef_offset(self, offset: float) -> None:
        flash_delay = round(min(1.0, abs(offset)) * 0.5, 1)
        if abs(offset) < 0.1:
            self.pattern = LEDPattern.solid(Color.kGreen)
        elif (
            not self.is_reef_offset_flashing
            or not is_off(self.strip_data[0])
            or not is_off(self.strip_data[-1])
        ):
            # We only set the new pattern if the previous one has finished a cycle
            if offset > 0.0:
                self.pattern = LEDPattern.blink(
                    LEDPattern.steps(
                        [
                            (0.0, Color.kBlack),
                            (0.5, Color.kOrange),
                        ]
                    ),
                    flash_delay,
                )
            else:
                self.pattern = LEDPattern.blink(
                    LEDPattern.steps(
                        [
                            (0.0, Color.kOrange),
                            (0.5, Color.kBlack),
                        ]
                    ),
                    flash_delay,
                )
        self.keep_alive()
        self.is_reef_offset_flashing = True

    def execute(self) -> None:
        if time.monotonic() - self.last_update_time > RESET_TIMEOUT:
            self.pattern = LEDPattern.solid(wpilib.Color.kBlack)
            self.is_reef_offset_flashing = False
        self.pattern.applyTo(
            self.strip_data, lambda idx, color: self.strip_data[idx].setLED(color)
        )
        self.leds.setData(self.strip_data)
