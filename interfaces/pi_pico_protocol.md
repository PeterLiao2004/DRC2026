# Pi 5 to Pico Protocol

Initial communication contract between the Raspberry Pi 5 and Raspberry Pi Pico.

## Link

- **Physical link:** USB serial or UART.
- **Owner:** Pi 5 sends commands, Pico enforces timing and safety.
- **Safety rule:** The Pico must stop the motors if valid commands are not received before the watchdog timeout.

## Command Direction

Pi 5 -> Pico:

```text
DRIVE left_speed right_speed
STOP
PING
```

Pico -> Pi 5:

```text
OK
ERR code message
TELEM left_ticks right_ticks left_pwm right_pwm battery_mv
PONG
```

## Field Notes

- `left_speed` and `right_speed` should start as normalized values from `-1.0` to `1.0`.
- The Pico should clamp all motor outputs to configured limits.
- Add a sequence number or checksum before relying on this link during autonomous operation.
- Keep this document updated before changing either implementation.
