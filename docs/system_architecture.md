# System Architecture

## Overview

The car is split into a high-level computer and a low-level controller.

```text
Camera/Sensors -> Raspberry Pi 5 -> Pi/Pico Protocol -> Raspberry Pi Pico -> Motor Driver -> Motors
                                                     <- Telemetry/Encoders <-
```

## Raspberry Pi 5

- Runs autonomy and higher-level control loops.
- Handles sensors that do not need hard real-time timing.
- Sends desired motion commands to the Pico.
- Logs system state for debugging.

## Raspberry Pi Pico

- Runs timing-sensitive motor and encoder logic.
- Owns fail-safe motor shutdown.
- Reports encoder and motor telemetry back to the Pi 5.
- Should remain simple and deterministic.

## PCB

- Carries the Pico and motor-driver wiring.
- Provides power distribution and regulated logic rails.
- Exposes encoder and expansion connectors.
- Stores fabrication outputs under `PCB/gerbers/`.
