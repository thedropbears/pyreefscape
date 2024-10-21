from components import led


def test_morse_messages_are_valid() -> None:
    for message in led.Morse.MESSAGES:
        led.Morse.translate_message(message)
