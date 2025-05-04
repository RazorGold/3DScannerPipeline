#Hard Coding the block 
import time
import sys
import os
from xarm.wrapper import XArmAPI
from configparser import ConfigParser

def get_arm_ip(default_ip="192.168.1.205"):
    try:
        parser = ConfigParser()
        parser.read('/robot.conf')
        ip = parser.get('xArm', 'ip')
        if ip:
            return ip
    except Exception as e:
        print(f"Could not load IP from config file: {e}")
    print(f"Using default IP: {default_ip}")
    return default_ip

def connect_arm(ip):
    arm = XArmAPI(ip)
    arm.connect()
    return arm

#Object Height: 15 cm
#Object Width: 5 cm
#Object Thickness: 3 cm 

def init(arm):
    print("Initializing arm...")
    arm.motion_enable(True)
    arm.set_mode(0)  # Position mode
    arm.set_state(0)  # Ready
    arm.set_gripper_mode(0)
    arm.set_gripper_enable(True)
    arm.set_gripper_position(850)
    time.sleep(1)

def moveto_base(arm, speed=100):
    print("Moving to base position...")
    joints = [0,-41.4,1.4,0.9,40,0]
    move_joints(arm, joints, speed)

def move_into_scanning_side(arm, speed=30):
    print("Moving to initial scanning position...")
    coords = [430.4, 4.6, 223.1, 180, -61.8, 0]
    move_coords(arm, coords, speed)

def move_into_scanning_up(arm, speed=30):
    print("Moving to initial scanning position...")
    coords = [308.9,62.2,114.2,141.7,-83.8,8.1]
    move_coords(arm, coords, speed)

def move_coords(arm, coords, speed=100):
    code = arm.set_position(x=coords[0], y=coords[1], z=coords[2],
                            roll=coords[3], pitch=coords[4], yaw=coords[5],
                            speed=speed, wait=True)
    if code != 0:
        print(f"Coord move failed: {code}")
    print("Current position:", arm.get_position())
    print("Joint angles:", arm.get_servo_angle())

def move_joints(arm, joints, speed=100):
    code = arm.set_servo_angle(angle=joints, speed=speed, wait=True)
    if code != 0:
        print(f"Joint move failed: {code}")
    print("Current position:", arm.get_position())
    print("Joint angles:", arm.get_servo_angle())

def grab_object(arm):
    print("Grabbing...")
    move_coords(arm, [568, -13.8, 182.3, 180, -89.4, 0])
    time.sleep(3)
    arm.set_gripper_position(450)
    print("Grabbed")

def rotate_object_faceup(arm):
    print("Rotating...")
    move_coords(arm, [568, -13.8, 242.3, 180, -89.4, 0])
    time.sleep(3)
    move_coords(arm, [568, -13.8, 242.3, 180, -0.5, 0])
    time.sleep(3)

def place_object(arm):
    print("Placing...")
    move_coords(arm, [560.3, -13.8, 110.8, 180, -0.5, 0])
    time.sleep(3)

def drop_object(arm):
    print("Dropping...")
    arm.set_gripper_position(600)
    time.sleep(3)
    move_coords(arm, [560.3, -13.8, 170.8, 180, -0.5, 0])
    time.sleep(3)

def pickup_object(arm):
    print("Placing...")
    move_coords(arm, [568, -13.8, 242.3, 180, -0.5, 0])
    time.sleep(3)
    move_coords(arm, [560.3, -13.8, 110.8, 180, -0.5, 0])
    time.sleep(3)
    arm.set_gripper_position(450)

def rotate_object_faceside(arm):
    print("Rotating...")
    move_coords(arm, [568, -13.8, 242.3, 180, -0.5, 0])
    time.sleep(3)
    move_coords(arm, [568, -13.8, 242.3, 180, -89.4, 0])
    time.sleep(3)

def set_object(arm):
    print("Setting...")
    move_coords(arm, [568, -13.8, 182.3, 180, -89.4, 0])
    time.sleep(3)
    arm.set_gripper_position(800)
    move_coords(arm, [568, -13.8, 242.3, 180, -89.4, 0])
    time.sleep(3)

def run_sequence():
    ip = get_arm_ip()
    arm = connect_arm(ip)
    speed = 100
    init(arm)

    moveto_base(arm, speed)
    time.sleep(3)

    move_into_scanning_side(arm, speed)
    time.sleep(3)

    grab_object(arm)
    time.sleep(3)

    rotate_object_faceup(arm)
    time.sleep(3)

    place_object(arm)
    time.sleep(3)

    drop_object(arm)
    time.sleep(3)

    move_into_scanning_side(arm, speed)
    time.sleep(3)

    move_into_scanning_up(arm, speed)
    time.sleep(3)

    move_into_scanning_side(arm, speed)
    time.sleep(3)

    pickup_object(arm)
    time.sleep(3)

    rotate_object_faceside(arm)
    time.sleep(3)

    set_object(arm)
    time.sleep(3)

    move_into_scanning_side(arm, speed)
    time.sleep(3)
    

if __name__ == "__main__":
    run_sequence()
