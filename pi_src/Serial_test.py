import serial
import time

PORT = "/dev/ttyACM0"
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)

numbers = [-1000, -750, -100, -10, -6, 0, 25, 50, 75, 100, 300, 500, 750, 1000]

for n in numbers:
    msg = f"{n}\n"
    ser.write(msg.encode("utf-8"))

    time.sleep(0.5)

ser.close()