import time
import sys
import random
import math
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__SportModeState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
from unitree_sdk2py.go2.sport.sport_client import (
    SportClient,
    PathPoint,
    SPORT_PATH_POINT_SIZE,
)
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
from dataclasses import dataclass

@dataclass
class TestOption:
    name: str
    id: int

option_list = [
    # Basic control commands (0-9)
    TestOption(name="damp", id=0),         
    TestOption(name="stand_up", id=1),     
    TestOption(name="stand_down", id=2),   
    TestOption(name="stop_move", id=3),
    # TestOption(name="switch_gait_0", id=4),    
    # TestOption(name="switch_gait_1", id=5),
    TestOption(name="balanced_stand", id=6),     
    TestOption(name="recovery", id=7),
    # Basic movement commands (10-19)
    TestOption(name="move_forward", id=10),         
    TestOption(name="move_backward", id=11),
    TestOption(name="move_lateral_left", id=12),
    TestOption(name="move_lateral_right", id=13),
    TestOption(name="rotate_left", id=14),
    TestOption(name="rotate_right", id=15),
    
    # Distance and angle specific commands (20-29)
    TestOption(name="move_forward_distance", id=20),
    TestOption(name="move_backward_distance", id=21),
    TestOption(name="move_left_distance", id=22),
    TestOption(name="move_right_distance", id=23),
    TestOption(name="rotate_left_angle", id=24),
    TestOption(name="rotate_right_angle", id=25),
    
    # Mode switching commands (30-31)
    TestOption(name="switch_to_normal_mode", id=30),
    TestOption(name="switch_to_ai_mode", id=31),
    
    # Fun actions / Tricks (40-59)
    TestOption(name="hello", id=40),         
    TestOption(name="sit", id=41),     
    TestOption(name="rise_sit", id=42),   
    TestOption(name="stretch", id=43),         
    TestOption(name="dance", id=44),    
    TestOption(name="scrape", id=45),  
    TestOption(name="wiggle_hips", id=46),     
    TestOption(name="heart", id=47),
    
    # Advanced movements (require AI mode) (50-51-64)
    TestOption(name="left_flip", id=50),      
    TestOption(name="back_flip", id=51),
    TestOption(name="front_flip", id=52),
    TestOption(name="front_jump", id=53),    
    TestOption(name="front_pounce", id=54),
    
    # Special movements (60-69)
    # TestOption(name="free_walk", id=60),  
    # TestOption(name="free_bound", id=61), 
    TestOption(name="free_avoid", id=62),  
    TestOption(name="walk_stair", id=63), 
    TestOption(name="walk_upright", id=64),
    TestOption(name="cross_step", id=65),
    # TestOption(name="free_jump", id=66),
]

# List of advanced commands that work better in AI mode
ADVANCED_COMMANDS = [50, 51, 64]  # left_flip (50), back_flip (51), walk_upright (64)

def switch_robot_mode(mode_name):
    """Switch the robot to the specified mode"""
    msc = MotionSwitcherClient()
    msc.SetTimeout(5.0)
    msc.Init()
    
    print(f"Switching to {mode_name} mode...")
    ret = msc.SelectMode(mode_name)
    
    if ret == 0:
        print(f"Successfully switched to {mode_name} mode")
        return True
    else:
        print(f"Failed to switch to {mode_name} mode. Error code: {ret}")
        return False

def move_distance(sport_client, vx, vy, distance, default_velocity=0.5):
    """
    Move the robot a specific distance in the x-y plane
    
    Args:
        sport_client: The SportClient instance
        vx: x velocity direction (-1, 0, or 1)
        vy: y velocity direction (-1, 0, or 1)
        distance: Distance to move in meters
        default_velocity: Speed to move at (m/s)
    """
    if distance <= 0:
        print("Distance must be positive")
        return
    
    # Normalize direction vector and scale by velocity
    magnitude = math.sqrt(vx**2 + vy**2)
    if magnitude == 0:
        print("At least one of vx or vy must be non-zero")
        return
    
    vx_norm = (vx / magnitude) * default_velocity
    vy_norm = (vy / magnitude) * default_velocity
    
    # Calculate duration based on distance and velocity
    duration = distance / default_velocity
    
    print(f"Moving with vx={vx_norm:.2f}, vy={vy_norm:.2f} for {duration:.2f} seconds...")
    
    # Execute movement for duration
    start_time = time.time()
    cmd_interval = 0.01  # 100Hz
    
    while time.time() - start_time < duration:
        sport_client.Move(vx_norm, vy_norm, 0)
        time.sleep(cmd_interval)
    
    # Stop the robot
    sport_client.StopMove()
    print("Movement completed.")

def rotate_angle(sport_client, direction, angle_degrees, default_angular_velocity=0.5):
    """
    Rotate the robot by a specific angle
    
    Args:
        sport_client: The SportClient instance
        direction: 1 for left/CCW, -1 for right/CW
        angle_degrees: Angle to rotate in degrees
        default_angular_velocity: Angular velocity in rad/s
    """
    if angle_degrees <= 0:
        print("Angle must be positive")
        return
    
    # Convert degrees to radians
    angle_radians = math.radians(angle_degrees)
    
    # Calculate angular velocity with direction
    angular_velocity = direction * default_angular_velocity
    
    # Calculate duration based on angle and angular velocity
    duration = angle_radians / abs(default_angular_velocity)
    
    print(f"Rotating with angular_velocity={angular_velocity:.2f} for {duration:.2f} seconds...")
    
    # Execute rotation for duration
    start_time = time.time()
    cmd_interval = 0.01  # 100Hz
    
    while time.time() - start_time < duration:
        sport_client.Move(0, 0, angular_velocity)
        time.sleep(cmd_interval)
    
    # Stop the robot
    sport_client.StopMove()
    print("Rotation completed.")

def execute_command(command_id, interface_name, additional_args=None):
    """Execute a single command based on the provided ID"""
    
    print("WARNING: Please ensure there are no obstacles around the robot.")
    
    # Initialize the channel
    ChannelFactoryInitialize(0, interface_name)

    # Find the command by ID
    command_name = None
    for option in option_list:
        if option.id == command_id:
            command_name = option.name
            break
    
    if command_name is None:
        print(f"Error: Invalid command ID: {command_id}")
        print("Available commands:")
        for option in option_list:
            print(f"{option.id}: {option.name}")
        return False
    
    print(f"Executing command: {command_name} (ID: {command_id})")
    
    # Handle mode switching commands
    if command_id == 30:  # switch to normal mode
        return switch_robot_mode("normal")
    elif command_id == 31:  # switch to AI mode
        return switch_robot_mode("ai")
    
    # If command is an advanced command, warn if not in AI mode
    if command_id in ADVANCED_COMMANDS:
        print("NOTE: This is an advanced command that works best in AI mode.")
        print("If you experience issues, try switching to AI mode first with command 31.")
    
    # Initialize Sport Client for regular commands
    sport_client = SportClient()
    sport_client.SetTimeout(10.0)
    sport_client.Init()
    
    # Parse additional arguments for distance/angle-based commands
    distance = 1.0  # Default 1 meter
    angle = 90.0    # Default 90 degrees
    
    if additional_args and len(additional_args) > 0:
        try:
            if command_id in [20, 21, 22, 23]:  # distance-based commands
                distance = float(additional_args[0])
            elif command_id in [24, 25]:  # angle-based commands
                angle = float(additional_args[0])
        except ValueError:
            print("Warning: Could not parse additional argument as a number. Using default value.")
    
    # Execute the specific command
    ret = None
    if command_id == 0:  # damp
        sport_client.Damp()
    elif command_id == 1:  # stand_up
        sport_client.StandUp()
    elif command_id == 2:  # stand_down
        sport_client.StandDown()
    elif command_id == 3:  # stop_move
        sport_client.StopMove()
    elif command_id == 4:  # switch_gait_0
        sport_client.SwitchGait(0)
    elif command_id == 5:  # switch_gait_1
        sport_client.SwitchGait(1)
    elif command_id == 6:  # balanced_stand
        sport_client.BalanceStand()
    elif command_id == 7:  # recovery
        sport_client.RecoveryStand()
        
    # Basic movement commands
    elif command_id == 10:  # move_forward
        move_distance(sport_client, 1, 0, 1.0)
    elif command_id == 11:  # move_backward
        move_distance(sport_client, -1, 0, 1.0)
    elif command_id == 12:  # move_lateral_left
        move_distance(sport_client, 0, 1, 1.0)
    elif command_id == 13:  # move_lateral_right
        move_distance(sport_client, 0, -1, 1.0)
    elif command_id == 14:  # rotate_left
        rotate_angle(sport_client, 1, 90.0)
    elif command_id == 15:  # rotate_right
        rotate_angle(sport_client, -1, 90.0)
    
    # Distance and angle specific commands
    elif command_id == 20:  # move_forward_distance
        move_distance(sport_client, 1, 0, distance)
    elif command_id == 21:  # move_backward_distance
        move_distance(sport_client, -1, 0, distance)
    elif command_id == 22:  # move_left_distance
        move_distance(sport_client, 0, 1, distance)
    elif command_id == 23:  # move_right_distance
        move_distance(sport_client, 0, -1, distance)
    elif command_id == 24:  # rotate_left_angle
        rotate_angle(sport_client, 1, angle)
    elif command_id == 25:  # rotate_right_angle
        rotate_angle(sport_client, -1, angle)
    
    # Fun actions / Tricks
    elif command_id == 40:  # hello
        ret = sport_client.Hello()
        print("ret: ", ret)
    elif command_id == 41:  # sit
        ret = sport_client.Sit()
        print("ret: ", ret)
    elif command_id == 42:  # rise_sit
        ret = sport_client.RiseSit()
        print("ret: ", ret)
    elif command_id == 43:  # stretch
        ret = sport_client.Stretch()
        print("ret: ", ret)
    elif command_id == 44:  # dance
        # Randomly choose between Dance1 and Dance2
        dance_choice = random.choice([1, 2])
        if dance_choice == 1:
            ret = sport_client.Dance1()
            print("Dance1 ret: ", ret)
        else:
            ret = sport_client.Dance2()
            print("Dance2 ret: ", ret)
    elif command_id == 45:  # scrape
        ret = sport_client.Scrape()
        print("ret: ", ret)
    elif command_id == 46:  # wiggle_hips
        ret = sport_client.WiggleHips()
        print("ret: ", ret)
    elif command_id == 47:  # heart
        ret = sport_client.Heart()
        print("ret: ", ret)
    
    # Advanced movements
    elif command_id == 50:  # left_flip
        ret = sport_client.LeftFlip()
        print("ret: ", ret)
    elif command_id == 51:  # back_flip
        ret = sport_client.BackFlip()
        print("ret: ", ret)
    elif command_id == 52:  # front_flip
        ret = sport_client.FrontFlip()
        print("ret: ", ret)
    elif command_id == 53:  # front_jump
        ret = sport_client.FrontJump()
        print("ret: ", ret)
    elif command_id == 54:  # front_pounce
        ret = sport_client.FrontPounce()
        print("ret: ", ret)
    
    # Special movements
    elif command_id == 60:  # free_walk
        ret = sport_client.FreeWalk(True)
        print("ret: ", ret)
    elif command_id == 61:  # free_bound
        ret = sport_client.FreeBound(True)
        print("ret: ", ret)
        time.sleep(2)
        ret = sport_client.FreeBound(False)
        print("ret: ", ret)
    elif command_id == 62:  # free_avoid
        ret = sport_client.FreeAvoid(True)
        print("ret: ", ret)
        time.sleep(2)
        ret = sport_client.FreeAvoid(False)
        print("ret: ", ret)
    elif command_id == 63:  # walk_stair
        ret = sport_client.WalkStair(True)
        print("ret: ", ret)
        time.sleep(10)
        ret = sport_client.WalkStair(False)
        print("ret: ", ret)
    elif command_id == 64:  # walk_upright
        ret = sport_client.WalkUpright(True)
        print("ret: ", ret)
        time.sleep(4)
        ret = sport_client.WalkUpright(False)
        print("ret: ", ret)
    elif command_id == 65:  # cross_step
        ret = sport_client.CrossStep(True)
        print("ret: ", ret)
        time.sleep(4)
        ret = sport_client.CrossStep(False)
        print("ret: ", ret)
    elif command_id == 66:  # free_jump
        ret = sport_client.FreeJump(True)
        print("ret: ", ret)
        time.sleep(4)
        ret = sport_client.FreeJump(False)
        print("ret: ", ret)
    
    return True

def print_available_commands():
    """Print all available commands with their IDs"""
    
    # Basic control commands
    print("\nBasic control commands:")
    for option in option_list:
        if 0 <= option.id <= 9:
            print(f"{option.id}: {option.name}")
    
    # Basic movement commands
    print("\nBasic movement commands:")
    for option in option_list:
        if 10 <= option.id <= 19:
            print(f"{option.id}: {option.name}")
    
    # Distance and angle specific commands
    print("\nPrecise movement commands (with distance/angle):")
    for option in option_list:
        if 20 <= option.id <= 29:
            print(f"{option.id}: {option.name}")
    
    # Mode switching commands
    print("\nMode switching commands:")
    for option in option_list:
        if 30 <= option.id <= 39:
            print(f"{option.id}: {option.name}")
    
    # Fun actions / Tricks
    print("\nFun actions and tricks:")
    for option in option_list:
        if 40 <= option.id <= 49:
            print(f"{option.id}: {option.name}")
    
    # Advanced movements
    print("\nAdvanced movements (work best in AI mode):")
    for option in option_list:
        if 50 <= option.id <= 59:
            print(f"{option.id}: {option.name}")
    
    # Special movements
    print("\nSpecial movements:")
    for option in option_list:
        if 60 <= option.id <= 69:
            print(f"{option.id}: {option.name}")
    
    print("\nUsage examples:")
    print("Move forward 2 meters: python robot_control.py <interface> 20 2")
    print("Rotate left 45 degrees: python robot_control.py <interface> 24 45")

if __name__ == "__main__":
    # Check arguments
    if len(sys.argv) < 3:
        print(f"Usage: python3 {sys.argv[0]} <network_interface> <command_id> [additional_args]")
        print("Example: python3 robot_control.py eth0 1")
        print("Example with distance: python3 robot_control.py eth0 20 2.5  # move forward 2.5 meters")
        print_available_commands()
        sys.exit(1)
    
    # Get network interface and command ID from arguments
    interface_name = sys.argv[1]
    
    try:
        command_id = int(sys.argv[2])
    except ValueError:
        print(f"Error: Command ID must be an integer")
        print_available_commands()
        sys.exit(1)
    
    # Get additional arguments if any
    additional_args = sys.argv[3:] if len(sys.argv) > 3 else None
    
    # Execute the command
    success = execute_command(command_id, interface_name, additional_args)
    
    if not success:
        sys.exit(1)