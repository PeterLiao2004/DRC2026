DRC Robot PCB

Custom PCB design for a differential drive robot built for robotics competition use.

This board integrates:

Raspberry Pi Pico

Motor driver

2S LiPo power input

5V regulation

Encoder headers

Addressable LED headers

Power protection components

📌 Project Overview

This PCB is designed to:

Drive two DC motors with encoders

Distribute regulated 5V and battery voltage (VBAT)

Provide clean signal routing for encoders and control

Support modular expansion (LEDs, sensors, headers)

Be manufacturable via JLCPCB

🔋 Power Architecture

Battery: 2S LiPo (~7.4V nominal)

Motor Rail: Direct VBAT

Logic Rail: Regulated 5V

Ground: Common ground plane

Protection includes:

Reverse polarity protection

Bulk capacitance near motor driver

Optional TVS diode (if enabled)

🧠 Microcontroller

Raspberry Pi Pico

External header rows for prototyping

GPIO exposed for:

Motor control

Encoder inputs

LED control

Expansion

🔌 Connectors

XT60 battery input

Screw terminal (motor outputs)

JST headers for encoders

3-pin headers for addressable LEDs

Auxiliary expansion headers
