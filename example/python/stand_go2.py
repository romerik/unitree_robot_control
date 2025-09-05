import time
import sys
import numpy as np
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
from unitree_sdk2py.utils.crc import CRC

stand_up_joint_pos = np.array([
    0.00571868, 0.608813, -1.21763, -0.00571868, 0.608813, -1.21763,
    0.00571868, 0.608813, -1.21763, -0.00571868, 0.608813, -1.21763
], dtype=float)

stand_down_joint_pos = np.array([
    0.0473455, 1.22187, -2.44375, -0.0473455, 1.22187, -2.44375, 0.0473455,
    1.22187, -2.44375, -0.0473455, 1.22187, -2.44375
], dtype=float)

dt = 0.002
running_time = 0.0
crc = CRC()

input("Press enter to start")

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Using simulation mode (domain_id=1, interface=lo)")
        ChannelFactoryInitialize(1, "lo")
    else:
        print(f"Using real robot mode (domain_id=0, interface={sys.argv[1]})")
        ChannelFactoryInitialize(0, sys.argv[1])
    
    # Create a publisher to publish the data defined in UserData class
    pub = ChannelPublisher("rt/lowcmd", LowCmd_)
    pub.Init()
    print("Publisher initialized")
    
    cmd = unitree_go_msg_dds__LowCmd_()
    cmd.head[0] = 0xFE
    cmd.head[1] = 0xEF
    cmd.level_flag = 0xFF
    cmd.gpio = 0
    
    for i in range(20):
        cmd.motor_cmd[i].mode = 0x01  # (PMSM) mode
        cmd.motor_cmd[i].q = 0.0
        cmd.motor_cmd[i].kp = 0.0
        cmd.motor_cmd[i].dq = 0.0
        cmd.motor_cmd[i].kd = 0.0
        cmd.motor_cmd[i].tau = 0.0
    
    print("Starting movement sequence...")
    
    while True:
        step_start = time.perf_counter()
        running_time += dt
        
        if running_time < 3.0:
            # Stand up in first 3 seconds
            phase = np.tanh(running_time / 1.2)
            print(f"Standing up - Time: {running_time:.2f}s, Phase: {phase:.3f}")
            
            for i in range(12):
                cmd.motor_cmd[i].q = phase * stand_up_joint_pos[i] + (1 - phase) * stand_down_joint_pos[i]
                cmd.motor_cmd[i].kp = phase * 50.0 + (1 - phase) * 20.0
                cmd.motor_cmd[i].dq = 0.0
                cmd.motor_cmd[i].kd = 3.5
                cmd.motor_cmd[i].tau = 0.0
        else:
            # Then stand down
            phase = np.tanh((running_time - 3.0) / 1.2)
            print(f"Standing down - Time: {running_time:.2f}s, Phase: {phase:.3f}")
            
            for i in range(12):
                cmd.motor_cmd[i].q = phase * stand_down_joint_pos[i] + (1 - phase) * stand_up_joint_pos[i]
                cmd.motor_cmd[i].kp = 50.0
                cmd.motor_cmd[i].dq = 0.0
                cmd.motor_cmd[i].kd = 3.5
                cmd.motor_cmd[i].tau = 0.0
        
        # Print some joint positions for debugging
        if running_time % 0.5 < dt:  # Print every 0.5 seconds
            print(f"Joint 0 target: {cmd.motor_cmd[0].q:.3f}, kp: {cmd.motor_cmd[0].kp:.1f}")
        
        cmd.crc = crc.Crc(cmd)
        pub.Write(cmd)
        
        time_until_next_step = dt - (time.perf_counter() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)