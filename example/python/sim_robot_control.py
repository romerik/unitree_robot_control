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
from dataclasses import dataclass

@dataclass
class TestOption:
    name: str
    id: int

# Simplified option list for simulation (some commands may not work in simulation)
option_list = [
    # Basic control commands (0-9)
    TestOption(name="damp", id=0),         
    TestOption(name="stand_up", id=1),     
    TestOption(name="stand_down", id=2),   
    TestOption(name="stop_move", id=3),
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
    
    # Fun actions / Tricks (40-59) - these may not work in basic simulation
    TestOption(name="hello", id=40),         
    TestOption(name="sit", id=41),     
    TestOption(name="rise_sit", id=42),   
    TestOption(name="stretch", id=43),         
    TestOption(name="dance", id=44),    
    TestOption(name="scrape", id=45),  
    TestOption(name="wiggle_hips", id=46),     
    TestOption(name="heart", id=47),
]

def move_distance(sport_client, vx, vy, distance, default_velocity=0.3):
    """
    Move the robot a specific distance in the x-y plane
    Reduced default velocity for simulation stability
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
    cmd_interval = 0.02  # 50Hz for simulation
    
    while time.time() - start_time < duration:
        sport_client.Move(vx_norm, vy_norm, 0)
        time.sleep(cmd_interval)
    
    # Stop the robot
    sport_client.StopMove()
    print("Movement completed.")

def rotate_angle(sport_client, direction, angle_degrees, default_angular_velocity=0.3):
    """
    Rotate the robot by a specific angle
    Reduced angular velocity for simulation stability
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
    cmd_interval = 0.02  # 50Hz
    
    while time.time() - start_time < duration:
        sport_client.Move(0, 0, angular_velocity)
        time.sleep(cmd_interval)
    
    # Stop the robot
    sport_client.StopMove()
    print("Rotation completed.")

def execute_command(command_id, additional_args=None):
    """Execute a single command based on the provided ID - adapted for simulation"""
    
    print("Executing command in simulation mode...")
    
    # Initialize the channel for simulation (domain_id=1, interface="lo")
    ChannelFactoryInitialize(1, "lo")

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
    
    # Initialize Sport Client
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
    try:
        if command_id == 0:  # damp
            ret = sport_client.Damp()
        elif command_id == 1:  # stand_up
            ret = sport_client.StandUp()
        elif command_id == 2:  # stand_down
            ret = sport_client.StandDown()
        elif command_id == 3:  # stop_move
            ret = sport_client.StopMove()
        elif command_id == 6:  # balanced_stand
            ret = sport_client.BalanceStand()
        elif command_id == 7:  # recovery
            ret = sport_client.RecoveryStand()
            
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
        
        # Fun actions / Tricks (may not work in simulation)
        elif command_id == 40:  # hello
            ret = sport_client.Hello()
            print("Hello ret:", ret)
        elif command_id == 41:  # sit
            ret = sport_client.Sit()
            print("Sit ret:", ret)
        elif command_id == 42:  # rise_sit
            ret = sport_client.RiseSit()
            print("RiseSit ret:", ret)
        elif command_id == 43:  # stretch
            ret = sport_client.Stretch()
            print("Stretch ret:", ret)
        elif command_id == 44:  # dance
            # Randomly choose between Dance1 and Dance2
            dance_choice = random.choice([1, 2])
            if dance_choice == 1:
                ret = sport_client.Dance1()
                print("Dance1 ret:", ret)
            else:
                ret = sport_client.Dance2()
                print("Dance2 ret:", ret)
        elif command_id == 45:  # scrape
            ret = sport_client.Scrape()
            print("Scrape ret:", ret)
        elif command_id == 46:  # wiggle_hips
            ret = sport_client.WiggleHips()
            print("WiggleHips ret:", ret)
        elif command_id == 47:  # heart
            ret = sport_client.Heart()
            print("Heart ret:", ret)
        
        if ret is not None:
            if ret == 0:
                print("Command executed successfully")
            else:
                print(f"Command returned error code: {ret}")
                
    except Exception as e:
        print(f"Error executing command: {e}")
        print("This command may not be supported in simulation mode")
        return False
    
    return True

def print_available_commands():
    """Print all available commands with their IDs"""
    
    print("\nBasic control commands:")
    for option in option_list:
        if 0 <= option.id <= 9:
            print(f"{option.id}: {option.name}")
    
    print("\nBasic movement commands:")
    for option in option_list:
        if 10 <= option.id <= 19:
            print(f"{option.id}: {option.name}")
    
    print("\nPrecise movement commands (with distance/angle):")
    for option in option_list:
        if 20 <= option.id <= 29:
            print(f"{option.id}: {option.name}")
    
    print("\nFun actions and tricks (may not work in simulation):")
    for option in option_list:
        if 40 <= option.id <= 49:
            print(f"{option.id}: {option.name}")
    
    print("\nUsage examples:")
    print("Basic stand up: python sim_robot_control.py 1")
    print("Move forward 2 meters: python sim_robot_control.py 20 2")
    print("Rotate left 45 degrees: python sim_robot_control.py 24 45")
    print("Try dance: python sim_robot_control.py 44")

if __name__ == "__main__":
    # Check arguments
    if len(sys.argv) < 2:
        print(f"Usage: python3 {sys.argv[0]} <command_id> [additional_args]")
        print("Example: python3 sim_robot_control.py 1")
        print("Example with distance: python3 sim_robot_control.py 20 2.5  # move forward 2.5 meters")
        print_available_commands()
        sys.exit(1)
    
    try:
        command_id = int(sys.argv[1])
    except ValueError:
        print(f"Error: Command ID must be an integer")
        print_available_commands()
        sys.exit(1)
    
    # Get additional arguments if any
    additional_args = sys.argv[2:] if len(sys.argv) > 2 else None
    
    # Execute the command
    success = execute_command(command_id, additional_args)
    
    if not success:
        sys.exit(1)