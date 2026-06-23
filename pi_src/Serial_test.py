import serial
import time

PORT = "/dev/ttyACM0"
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)

numbers = [-100, -75, -10, -8, -2, 0, 2, 5, 7, 10, 30, 50, 75, 100]

for n in numbers:
    msg = f"{n}\n"
    ser.write(msg.encode("utf-8"))
    print(f"Sent: {n}")

    time.sleep(2)

ser.close()