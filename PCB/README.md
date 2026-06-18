# DRC Robot PCB

Custom PCB for a differential-drive robotics competition robot.

## Overview

This board integrates:

- Raspberry Pi Pico
- Dual motor driver
- 2S LiPo power input (VBAT)
- Regulated 5V logic rail
- Encoder headers
- LED expansion headers

Designed as a 2-layer PCB for low-cost fabrication.

## Power System

- **Battery:** 2S LiPo, about 7.4 V nominal
- **VBAT:** Motor power
- **+5V:** Logic and peripherals
- **GND:** Common ground plane

## KiCad Version

Created with **KiCad 9.0**.

## Usage

Clone the repository:

```bash
git clone https://github.com/PeterLiao2004/DRC2026.git
```

Open `DRC 2026.kicad_pro` in KiCad.
