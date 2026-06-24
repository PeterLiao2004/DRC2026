import serial
import time

PORT = "/dev/ttyACM0"
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)

numbers = [2, 5, 7, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100]

for n in numbers:
    msg = f"D,0,{n}\n"
    ser.write(msg.encode("utf-8"))
    print(f"Sent: {n}")

    time.sleep(1)

ser.close()