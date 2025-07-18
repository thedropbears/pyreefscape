import time

import wpilib
from magicbot import feedback
from wpilib import AddressableLED, Color, LEDPattern
from wpimath.geometry import Translation2d

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
        self.right_rear_front_split = 0.25
        self.halfway_split = 0.5
        self.left_front_rear_split = 0.75

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

    def not_measured(self) -> None:
        self.pattern = LEDPattern.blink(LEDPattern.solid(wpilib.Color.kPurple), 0.125)
        self.keep_alive()

    def rainbow(self) -> None:
        self.pattern = LEDPattern.rainbow(255, 100).scrollAtAbsoluteSpeed(
            0.1, LED_SPACING
        )
        self.keep_alive()

    def vision_timeout(self) -> None:
        self.pattern = LEDPattern.breathe(LEDPattern.solid(Color.kPurple), 2.0)
        self.keep_alive()

    def no_auto(self) -> None:
        self.pattern = LEDPattern.blink(LEDPattern.solid(Color.kRed), 0.25)
        self.keep_alive()

    def invalid_start(self, translation: Translation2d, tol: float) -> None:
        # Tolerance is total, so scale to the worse case component - ie 45 deg triangle
        tol = tol / (2.0**0.5)
        self.pattern = LEDPattern.blink(
            LEDPattern.steps(
                [
                    # Rear right
                    (
                        0.0,
                        Color.kBlue
                        if translation.x > tol or translation.y > tol
                        else Color.kBlack,
                    ),
                    # Front right
                    (
                        self.right_rear_front_split,
                        Color.kBlue
                        if translation.x < -tol or translation.y > tol
                        else Color.kBlack,
                    ),
                    # Front left
                    (
                        self.halfway_split,
                        Color.kBlue
                        if translation.x < -tol or translation.y < -tol
                        else Color.kBlack,
                    ),
                    # Rear left
                    (
                        self.left_front_rear_split,
                        Color.kBlue
                        if translation.x > tol or translation.y < -tol
                        else Color.kBlack,
                    ),
                ]
            ),
            0.5,
        )
        self.keep_alive()

    def keep_alive(self) -> None:
        # Refresh the timer to stop the LEDs being turned off
        self.last_update_time = time.monotonic()

    def reef_offset(self, offset: float, offset_tol: float) -> None:
        flash_delay = max(min(1.0, abs(offset)), 0.1)
        if abs(offset) < offset_tol:
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
                            (self.halfway_split, Color.kOrange),
                        ]
                    ),
                    flash_delay,
                )
            else:
                self.pattern = LEDPattern.blink(
                    LEDPattern.steps(
                        [
                            (0.0, Color.kOrange),
                            (self.halfway_split, Color.kBlack),
                        ]
                    ),
                    flash_delay,
                )
        self.keep_alive()
        self.is_reef_offset_flashing = True

    def climber_deploying(self, left_okay: bool, right_okay: bool) -> None:
        self.pattern = LEDPattern.breathe(
            LEDPattern.steps(
                [
                    (0.0, Color.kGreen if left_okay else Color.kRed),
                    (self.halfway_split, Color.kGreen if right_okay else Color.kRed),
                ]
            ),
            0.5,
        )
        self.keep_alive()

    def climber_deployed(self, left_okay: bool, right_okay: bool) -> None:
        self.pattern = LEDPattern.steps(
            [
                (0.0, Color.kGreen if left_okay else Color.kRed),
                (self.halfway_split, Color.kGreen if right_okay else Color.kRed),
            ]
        )

        self.keep_alive()

    def climber_retracting(self) -> None:
        colour = Color.kRed if is_red() else Color.kBlue
        steps = []
        stripes = 16
        increment = 1.0 / 16
        for v in range(0, stripes, 2):
            steps.append((v * increment, colour))
            steps.append(((v + 1) * increment, Color.kBlack))

        forward_pattern = (
            LEDPattern.steps(steps).scrollAtAbsoluteSpeed(1.0, LED_SPACING).reversed()  # type: ignore[arg-type]
        )

        def remapper(length: int, idx: int) -> int:
            idx = int(idx)
            splits = [
                0,
                int(length * self.right_rear_front_split),
                int(length * self.halfway_split),
                int(length * self.left_front_rear_split),
            ]
            if splits[0] <= int(idx) < splits[1]:
                return int(splits[1] - splits[0]) - idx
            elif splits[2] <= int(idx) < splits[3]:
                return int(splits[3] - splits[2]) - (idx - splits[2]) + splits[2]
            else:
                return int(idx)

        forward = forward_pattern.mapIndex(remapper)  # type: ignore[arg-type]
        self.pattern = forward
        self.keep_alive()

    def execute(self) -> None:
        if time.monotonic() - self.last_update_time > RESET_TIMEOUT:
            self.pattern = LEDPattern.solid(wpilib.Color.kBlack)
            self.is_reef_offset_flashing = False
        self.pattern.applyTo(
            self.strip_data,
            lambda idx, color: self.strip_data[idx].setLED(color),  # type: ignore[call-overload]
        )
        self.leds.setData(self.strip_data)
