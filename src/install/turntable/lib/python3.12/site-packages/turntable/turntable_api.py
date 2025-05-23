import serial
import time

ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
time.sleep(2)

def send_command(cmd):
    ser.write((cmd + "\n").encode())
    time.sleep(0.5)
    return ser.readline().decode().strip()

def set_speed(speed):
    return send_command(f"S{speed}")

def start_rotation():
    return send_command("R")

def is_done():
    return send_command("I")