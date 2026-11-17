import serial
import time

# UPDATE THIS PORT!
arduino = serial.Serial(port='/dev/ttyACM0', baudrate=9600, timeout=.1)

def send_angle(angle):
    arduino.write(bytes(str(angle), 'utf-8'))
    time.sleep(0.05) # Give Arduino a tiny moment to read

print("Moving to 0...")
send_angle(0)
time.sleep(1)

print("Moving to 180...")
send_angle(180)
time.sleep(1)

print("Moving to 90...")
send_angle(90)

print("Done! Connection works.")
arduino.close()
