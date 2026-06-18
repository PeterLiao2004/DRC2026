# DRC2026 Autonomous Car

Autonomous car project using a Raspberry Pi 5 for high-level autonomy and a Raspberry Pi Pico for low-level motor control.

## Repository Layout

```text
DRC2026/
+-- pi5_software/       High-level autonomy, perception, planning, and Pi-side services
+-- pico_firmware/      Low-level motor, encoder, and safety firmware for the Raspberry Pi Pico
+-- PCB/                KiCad PCB project, footprints, symbols, and fabrication outputs
+-- interfaces/         Communication contracts between the Pi 5, Pico, and hardware
+-- docs/               System architecture, bring-up notes, and design decisions
```

## System Roles

- **Raspberry Pi 5:** Main computer for camera processing, navigation, decision making, logging, and sending drive commands.
- **Raspberry Pi Pico:** Real-time controller for motors, encoders, watchdogs, and low-level safety behavior.
- **PCB:** Custom carrier/control board that connects power, Pico, motor driver, encoders, and expansion headers.

## Suggested Development Flow

1. Define or update the Pi-to-Pico protocol in `interfaces/`.
2. Implement low-level behavior in `pico_firmware/` and test it on the bench.
3. Implement high-level autonomy in `pi5_software/` against the same protocol.
4. Capture wiring, power, and bring-up notes in `docs/`.
5. Keep PCB source files and generated fabrication outputs in `PCB/`.

## Bring-Up Checklist

- Confirm PCB power rails before connecting the Pi 5 or Pico.
- Flash Pico firmware and verify motor disable/stop behavior first.
- Verify encoder telemetry with the car lifted off the ground.
- Connect Pi 5 software to the Pico protocol and test low-speed commands.
- Add autonomy features only after manual command and safety paths are reliable.
