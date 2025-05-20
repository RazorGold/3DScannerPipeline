import time
from xarm.wrapper import XArmAPI
from configparser import ConfigParser

arm = None
speed = None 

def get_arm_ip(default_ip="192.168.1.205"):
    # Try to get ip based on robot.conf file
    try:
        parser = ConfigParser()
        parser.read('/robot.conf')
        ip = parser.get('xArm', 'ip')
        if ip:
            return ip
    except Exception as e:
        print(f"Could not load IP from config file: {e}")
    # If fail then use the given ip address 
    print(f"Using default IP: {default_ip}")
    return default_ip


def connect_arm(ip):
    # Connect to the arm's server  
    arm = XArmAPI(ip)
    arm.connect()
    return arm 


def init():
    # Initialize the arm 
    print("Initializing arm...")
    global arm 
    ip = get_arm_ip()
    arm = connect_arm(ip)
    speed = 100
    arm.motion_enable(True) # Enable the joints 
    arm.set_mode(0) # Position control mode
    arm.set_state(0) # Sport State
    arm.set_gripper_mode(0) # Set Gripper as location mode
    arm.set_gripper_enable(True) # Enable the gripper 
    # arm.set_collision_tool_model(1) # Set the collision mode (Need to test)
    gripper_pos = 850
    arm.set_gripper_position(gripper_pos)
    print(f"Current Gripper Position: {gripper_pos}")
    time.sleep(1)


def object_width_to_gripper_position(width_mm):
    # Maps object width [0, 90] mm to gripper position [0, 850].
    # Tweaks to close more for smaller widths (0 to 50 mm).
    if width_mm < 0:
        raise ValueError("Width must be >= 0 mm")
    elif width_mm > 90:
        raise ValueError("Width exceeds maximum of 90 mm")

    # Non-linear mapping to close more for smaller objects
    if width_mm <= 50:
        grip = int((width_mm / 50) * 472)  # More closed for 50 mm
        print(grip)
        return grip  # More closed for 50 mm
    else:
        # Smooth transition from 472 at 50 mm to 850 at 90 mm
        grip = int(((width_mm - 50) / 40) * (850 - 472) + 472)
        print(grip)
        return grip



def move_coords(arm, coords, speed=50):
    # Move based on coordinates 
    code = arm.set_position(
        x=coords[0], y=coords[1], z=coords[2],
        roll=coords[3], pitch=coords[4], yaw=coords[5],
        speed=speed, wait=True
    )
    if code != 0:
        print(f"Coord move failed: {code}")
    print("Current position:", arm.get_position())
    print("Joint angles:", arm.get_servo_angle())

def move_joints(arm, joints, speed=50):
    # Move based on joint angles 
    code = arm.set_servo_angle(angle=joints, speed=speed, wait=True)
    if code != 0:
        print(f"Joint move failed: {code}")
    print("Current position:", arm.get_position())
    print("Joint angles:", arm.get_servo_angle())

def moveto_base(arm, speed=30):
    # Move to initalize position 
    joints = [0, -41.4, 1.4, 0.9, 40, 0]
    move_joints(arm, joints, speed)

def move_into_scanning(arm, speed=50):
    # Move to scanning position 
    #coords = [311.1, -54.5, 335, 180, -31.9, 0]  # Small circle 
    coords = [332.9, -50.8, 358.4, 180, -25.8, 0] # Bigger circle 
    move_coords(arm, coords, speed)
    move_joints(arm, [-13.538076, -50.71937, -39.856377, 7.713674, 65.59725, -14.685825, 0.0], speed)

def get_center_z(obj_height_mm):
    # Calculate the object's height 
    Z_CENTER_BASE = 182.3  # for 150 mm object
    return Z_CENTER_BASE + (obj_height_mm - 150) / 2 

def rotate_faceup(obj_height_mm, obj_width_mm, obj_thickness_mm):
    grab_object_standing(arm, obj_height_mm, obj_width_mm) # Grab object when it standing 
    lift_and_rotate_faceup(arm, obj_height_mm) # Rotate it to lay it down
    place_object_faceup(arm, obj_height_mm, obj_thickness_mm)    # Place the object while laying down 
    move_into_scanning(arm, speed=100)

# Rotating the object to lay

def grab_object_standing(arm, obj_height_mm, obj_width_mm):
    # Move to grab the object 
    print("Grabbing...")
    z_center = get_center_z(obj_height_mm)
    move_coords(arm, [568, -13.8, z_center, 180, -89.4, 0])
    # Wait two second for before grabbing 
    time.sleep(2)
    
    # Grab the object 
    grip_pos = object_width_to_gripper_position(obj_width_mm)
    print(f"Current Gripper Position: {grip_pos}")
    arm.set_gripper_position(grip_pos)
    time.sleep(1)
    print("Grabbed")

def lift_and_rotate_faceup(arm, obj_height_mm):
    # After grabbing, need to rotate faceup
    print("Rotating face up...")
    z_center = get_center_z(obj_height_mm)
    z_lifted = z_center + 70  # lift clearance
    move_coords(arm, [568, -13.8, z_lifted, 180, -89.4, 0])
    time.sleep(2)
    move_coords(arm, [568, -13.8, z_lifted, 180, -0.5, 0])
    time.sleep(2)

def place_object_faceup(arm, obj_height_mm, obj_thickness_mm):
    print("Placing object...")
    # Place the object after grabbing it 
    z_center = get_center_z(obj_height_mm)
    z_place = z_center - (obj_height_mm / 2 ) + (obj_thickness_mm / 2 )# go to bottom of object
    move_coords(arm, [560.3, -13.8, z_place, 180, -0.5, 0])
    time.sleep(2)
    drop_object_faceup(arm, obj_height_mm)

def drop_object_faceup(arm, obj_height_mm):
    print("Dropping...")
    # Releasing the gripper 
    arm.set_gripper_position(850)
    time.sleep(1)
    # Moving away from the object 
    z_center = get_center_z(obj_height_mm)
    z_lifted = z_center + 30
    move_coords(arm, [560.3, -13.8, z_lifted, 180, -0.5, 0])
    time.sleep(2)

# Rotating the object to stand
def rotate_faceside(obj_height_mm, obj_width_mm, obj_thickness_mm):
    grab_object_laying(arm, obj_height_mm, obj_width_mm, obj_thickness_mm)
    life_and_rotate_faceside(arm, obj_height_mm)
    set_object_down(arm, obj_height_mm)
    move_into_scanning(arm, speed=100)

def grab_object_laying(arm, obj_height_mm, obj_width_mm, obj_thickness_mm):
    # Move to grab the object
    print("Picking up again...")
    z_center = get_center_z(obj_height_mm)
    z_lifted = z_center + 60
    move_coords(arm, [568, -13.8, z_lifted, 180, -0.5, 0])
    time.sleep(2)
    z_place = z_center - (obj_height_mm / 2 ) +  (obj_thickness_mm / 2 )
    move_coords(arm, [560.3, -13.8, z_place, 180, -0.5, 0])
    time.sleep(2)
    # Grab the object
    grip_pos = object_width_to_gripper_position(obj_width_mm)
    print(grip_pos)
    arm.set_gripper_position(grip_pos)
    time.sleep(2)

def life_and_rotate_faceside(arm, obj_height_mm):
    # After grabbing, need to rotate faceside
    print("Rotating to side...")
    z_center = get_center_z(obj_height_mm)
    z_lifted = z_center + 60
    move_coords(arm, [568, -13.8, z_lifted, 180, -0.5, 0])
    time.sleep(2)
    move_coords(arm, [568, -13.8, z_lifted, 180, -89.4, 0])
    time.sleep(2)

def set_object_down(arm, obj_height_mm):
    print("Setting object down...")
    # Set the object down after grabbing it 
    z_center = get_center_z(obj_height_mm)
    move_coords(arm, [568, -13.8, z_center, 180, -89.4, 0])
    time.sleep(2)
    arm.set_gripper_position(600)  # open
    time.sleep(1)
    z_lifted = z_center + 100
    move_coords(arm, [568, -13.8, z_lifted, 180, -89.4, 0])
    time.sleep(2)


# Use this function to run any function on the arm
def run_sequence(choice, obj_height_mm, obj_width_mm, obj_thickness_mm):
    print(obj_height_mm, obj_width_mm, obj_thickness_mm)
    global arm
    speed = 100
    init()


    # Testing
    print("\n--- Control Menu ---")
    print("1: Move to base")
    print("2: Move to scanning position")
    print("3: Rotate face up")
    print("4: Rotate face side")
    print("q: Quit")
    print("---------------------")

    while True:
        # For testing
        choice = input("Enter a command: ").strip()

            
        if choice == "1":
            moveto_base(arm, speed)
            time.sleep(2)
        elif choice == "2":
            move_into_scanning(arm, speed)
            time.sleep(2)
        elif choice == "3":
            rotate_faceup(obj_height_mm, obj_width_mm, obj_thickness_mm)
            time.sleep(2)
        elif choice == "4":
            rotate_faceside(obj_height_mm, obj_width_mm, obj_thickness_mm)
            time.sleep(2)
        elif choice.lower() == "q":
            print("Exiting...")
            break
        else:
            print("Invalid input. Please enter 1–5 or 'q'.")

    
# Ignore this funcion (Testing)
if __name__ == "__main__":
    # Example usage:
    # block sequence:
    run_sequence(obj_height_mm=150, obj_width_mm=49, obj_thickness_mm=30)
    # Green Block Sequence:
    # run_sequence(obj_height_mm=50, obj_width_mm=40, obj_thickness_mm=20)
    # Breadboard:
    #run_sequence(obj_height_mm=80, obj_width_mm=55, obj_thickness_mm=10)