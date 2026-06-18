# Pico Firmware

Low-level firmware for the Raspberry Pi Pico. This side should own real-time motor control, encoder handling, watchdog behavior, and safety fallbacks.

## Layout

```text
pico_firmware/
+-- CMakeLists.txt
+-- src/        Firmware source files
+-- include/    Public headers shared across firmware modules
+-- tests/      Host-side or hardware-in-the-loop tests
+-- cmake/      CMake helper files for the Pico SDK build
```

## Pico SDK Setup

Install the Raspberry Pi Pico SDK outside this repository, then point `PICO_SDK_PATH` at it.

Example Windows PowerShell setup:

```powershell
cd C:\Users\peter\source\repos
git clone https://github.com/raspberrypi/pico-sdk.git
cd pico-sdk
git submodule update --init
$env:PICO_SDK_PATH = "C:\Users\peter\source\repos\pico-sdk"
```

To make the environment variable permanent for new terminals:

```powershell
[Environment]::SetEnvironmentVariable("PICO_SDK_PATH", "C:\Users\peter\source\repos\pico-sdk", "User")
```

Build this firmware:

```powershell
cd C:\Users\peter\source\repos\DRC2026\pico_firmware
cmake -S . -B build -G "NMake Makefiles"
cmake --build build
```

The generated `.uf2` file will be under `pico_firmware/build/`.

## Responsibilities

- Receive drive commands from the Raspberry Pi 5.
- Control motor driver outputs with predictable timing.
- Read encoder signals and publish telemetry.
- Stop motors if commands time out or invalid input is received.
- Keep the protocol behavior aligned with `../interfaces/pi_pico_protocol.md`.

## Initial Modules To Add

- Motor driver abstraction.
- Encoder reader.
- Serial command parser.
- Watchdog and emergency-stop logic.
- Telemetry packet publisher.
