import time
import sys
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_, unitree_go_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC

# Global variables to store sensor data
current_positions = [0.0] * 12
current_velocities = [0.0] * 12

def LowStateHandler(msg: LowState_):
    global current_positions, current_velocities
    for i in range(12):
        current_positions[i] = msg.motor_state[i].q
        current_velocities[i] = msg.motor_state[i].dq

if len(sys.argv) < 2:
    ChannelFactoryInitialize(1, "lo")
    print("Using simulation mode")
else:
    ChannelFactoryInitialize(0, sys.argv[1])

# Subscribe to sensor data
low_state_suber = ChannelSubscriber("rt/lowstate", LowState_)
low_state_suber.Init(LowStateHandler, 10)

# Publisher for commands
pub = ChannelPublisher("rt/lowcmd", LowCmd_)
pub.Init()

cmd = unitree_go_msg_dds__LowCmd_()
cmd.head[0] = 0xFE
cmd.head[1] = 0xEF
cmd.level_flag = 0xFF
cmd.gpio = 0

crc = CRC()

# Initialize all motors
for i in range(20):
    cmd.motor_cmd[i].mode = 0x01
    cmd.motor_cmd[i].q = 0.0
    cmd.motor_cmd[i].kp = 0.0
    cmd.motor_cmd[i].dq = 0.0
    cmd.motor_cmd[i].kd = 0.0
    cmd.motor_cmd[i].tau = 0.0

print("Reading current sensor values...")
time.sleep(1.0)  # Let sensor data arrive

print("\nCurrent motor positions and velocities:")
motor_names = [
    "FR_hip", "FR_thigh", "FR_calf",
    "FL_hip", "FL_thigh", "FL_calf", 
    "RR_hip", "RR_thigh", "RR_calf",
    "RL_hip", "RL_thigh", "RL_calf"
]

for i in range(12):
    print(f"Motor {i} ({motor_names[i]}): pos={current_positions[i]:.4f}, vel={current_velocities[i]:.4f}")

print(f"\nNow testing position control...")
print(f"Target: Move motor 0 (FR_hip) to 0.5 radians with high gain")

# Test position control with debug
target_position = 0.5
for t in range(300):  # 6 seconds
    # Reset all commands
    for i in range(12):
        cmd.motor_cmd[i].q = current_positions[i]  # Hold current position
        cmd.motor_cmd[i].kp = 10.0
        cmd.motor_cmd[i].dq = 0.0
        cmd.motor_cmd[i].kd = 1.0
        cmd.motor_cmd[i].tau = 0.0
    
    # Command motor 0 to target position
    cmd.motor_cmd[0].q = target_position
    cmd.motor_cmd[0].kp = 100.0  # High gain
    cmd.motor_cmd[0].kd = 5.0
    
    # Calculate what the bridge should compute
    position_error = target_position - current_positions[0]
    velocity_error = 0.0 - current_velocities[0]  # Target velocity = 0
    expected_torque = 100.0 * position_error + 5.0 * velocity_error
    
    if t % 25 == 0:  # Print every 0.5 seconds
        print(f"Time: {t*0.02:.1f}s")
        print(f"  Current pos: {current_positions[0]:.4f}, target: {target_position:.4f}")
        print(f"  Position error: {position_error:.4f}")
        print(f"  Expected torque: {expected_torque:.2f}")
        print(f"  Current velocity: {current_velocities[0]:.4f}")
    
    cmd.crc = crc.Crc(cmd)
    pub.Write(cmd)
    time.sleep(0.02)

print("Test complete")