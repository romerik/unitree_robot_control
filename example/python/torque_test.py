import time
import sys
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
from unitree_sdk2py.utils.crc import CRC

print("Starting torque test...")

if len(sys.argv) < 2:
    ChannelFactoryInitialize(1, "lo")
    print("Using simulation mode")
else:
    ChannelFactoryInitialize(0, sys.argv[1])
    print("Using real robot mode")

# Create publisher
pub = ChannelPublisher("rt/lowcmd", LowCmd_)
pub.Init()
print("Publisher initialized")

cmd = unitree_go_msg_dds__LowCmd_()
cmd.head[0] = 0xFE
cmd.head[1] = 0xEF
cmd.level_flag = 0xFF
cmd.gpio = 0

crc = CRC()

# Initialize all motors
for i in range(20):
    cmd.motor_cmd[i].mode = 0x01  # PMSM mode
    cmd.motor_cmd[i].q = 0.0
    cmd.motor_cmd[i].kp = 0.0
    cmd.motor_cmd[i].dq = 0.0
    cmd.motor_cmd[i].kd = 0.0
    cmd.motor_cmd[i].tau = 0.0

print("Applying 5Nm torque to front left hip (motor 0) for 3 seconds...")
print("You should see the robot's front left leg move!")

# Apply larger torque to one motor
for i in range(1500):  # 3 seconds at 500Hz
    # Reset all torques
    for j in range(20):
        cmd.motor_cmd[j].tau = 0.0
    
    # Apply significant torque to front left hip
    cmd.motor_cmd[0].tau = 5.0  # 5 Nm - should be very noticeable
    
    if i % 250 == 0:  # Print every 0.5 seconds
        print(f"Sending 5Nm to motor 0... ({i/500:.1f}s)")
    
    cmd.crc = crc.Crc(cmd)
    pub.Write(cmd)
    time.sleep(0.002)

print("Test complete. Did you see any movement?")