# Pi 5 Software

High-level software for the autonomous car. This side should own perception, planning, telemetry logging, and drive-command generation.

## Layout

```text
pi5_software/
+-- src/        Application code and reusable modules
+-- tests/      Unit and integration tests for Pi-side code
+-- config/     Runtime configuration, calibration, and robot profiles
+-- scripts/    Developer utilities and launch scripts
```

## Responsibilities

- Read sensors and camera inputs connected to the Raspberry Pi 5.
- Decide target speed and steering/differential-drive commands.
- Send commands to the Pico using the protocol in `../interfaces/`.
- Record logs that help debug autonomy and hardware behavior.

## Initial Modules To Add

- `src/main.py` or a ROS 2 launch entry point, depending on the software stack chosen.
- `src/pico_link/` for serial communication with the Pico.
- `src/control/` for high-level driving decisions.
- `src/perception/` for camera and sensor processing.
