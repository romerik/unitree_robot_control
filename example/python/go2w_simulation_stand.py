import time
import sys
import numpy as np

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
from unitree_sdk2py.utils.crc import CRC

# Constants for Go2W
class LegID:
    FR_0 = 0  # Front right hip
    FR_1 = 1  # Front right thigh  
    FR_2 = 2  # Front right calf
    FL_0 = 3  # Front left hip
    FL_1 = 4  # Front left thigh
    FL_2 = 5  # Front left calf
    RR_0 = 6  # Rear right hip
    RR_1 = 7  # Rear right thigh
    RR_2 = 8  # Rear right calf
    RL_0 = 9  # Rear left hip
    RL_1 = 10 # Rear left thigh
    RL_2 = 11 # Rear left calf
    FR_w = 12 # Front right wheel
    FL_w = 13 # Front left wheel
    RR_w = 14 # Rear right wheel
    RL_w = 15 # Rear left wheel

PosStopF = 2.146e9
VelStopF = 16000.0

class Go2WSimulationStand:
    def __init__(self):
        # Control parameters
        self.Kp = 70.0
        self.Kd = 5.0
        self.dt = 0.002
        
        # Command and state
        self.low_cmd = unitree_go_msg_dds__LowCmd_()
        self.low_state = None
        self.crc = CRC()
        
        # Target poses for the sequence
        self.targetPos_1 = [0.0, 1.36, -2.65, 0.0, 1.36, -2.65,    # Sit pose
                           -0.2, 1.36, -2.65, 0.2, 1.36, -2.65]
        
        self.targetPos_2 = [0.0, 0.67, -1.3, 0.0, 0.67, -1.3,      # Stand pose  
                           0.0, 0.67, -1.3, 0.0, 0.67, -1.3]
        
        self.targetPos_3 = [-0.35, 1.36, -2.65, 0.35, 1.36, -2.65, # Wide sit
                           -0.5, 1.36, -2.65, 0.5, 1.36, -2.65]
        
        self.startPos = [0.0] * 12
        
        # Phase durations (in control cycles)
        self.duration_1 = 500   # 1 second to sit
        self.duration_2 = 500   # 1 second to stand  
        self.duration_3 = 2000  # 4 seconds for wheel movement
        self.duration_4 = 900   # 1.8 seconds to wide sit
        
        # Phase progress tracking
        self.percent_1 = 0.0
        self.percent_2 = 0.0  
        self.percent_3 = 0.0
        self.percent_4 = 0.0
        
        self.firstRun = True
        self.done = False
        
        # Initialize for simulation
        ChannelFactoryInitialize(1, "lo")  # Simulation mode
        
        # Setup publishers and subscribers
        self.lowcmd_publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.lowcmd_publisher.Init()
        
        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.LowStateMessageHandler, 10)
        
        self.InitLowCmd()
        
        print("Go2W Simulation Controller Initialized")
        print("Waiting for robot state...")
        time.sleep(2.0)
    
    def InitLowCmd(self):
        """Initialize low-level command structure"""
        self.low_cmd.head[0] = 0xFE
        self.low_cmd.head[1] = 0xEF
        self.low_cmd.level_flag = 0xFF
        self.low_cmd.gpio = 0
        
        for i in range(20):
            self.low_cmd.motor_cmd[i].mode = 0x01
            self.low_cmd.motor_cmd[i].q = PosStopF
            self.low_cmd.motor_cmd[i].kp = 0
            self.low_cmd.motor_cmd[i].dq = VelStopF  
            self.low_cmd.motor_cmd[i].kd = 0
            self.low_cmd.motor_cmd[i].tau = 0
    
    def LowStateMessageHandler(self, msg: LowState_):
        """Handle incoming state messages"""
        self.low_state = msg
    
    def run_sequence(self):
        """Run the 4-stage movement sequence"""
        print("Starting Go2W movement sequence...")
        print("Stage 1: Moving to sit position...")
        print("Stage 2: Standing up...")
        print("Stage 3: Wheel movement demonstration...")
        print("Stage 4: Moving to wide sit...")
        
        cycle_count = 0
        
        while not self.done:
            if self.low_state is None:
                time.sleep(0.1)
                continue
            
            # Capture starting position on first run
            if self.firstRun:
                for i in range(12):
                    self.startPos[i] = self.low_state.motor_state[i].q
                self.firstRun = False
                print(f"Starting positions captured: {self.startPos[:4]}...")
            
            # Stage 1: Move to sit position
            if self.percent_1 < 1.0:
                self.percent_1 += 1.0 / self.duration_1
                self.percent_1 = min(self.percent_1, 1.0)
                
                for i in range(12):
                    self.low_cmd.motor_cmd[i].q = ((1 - self.percent_1) * self.startPos[i] + 
                                                  self.percent_1 * self.targetPos_1[i])
                    self.low_cmd.motor_cmd[i].dq = 0
                    self.low_cmd.motor_cmd[i].kp = self.Kp
                    self.low_cmd.motor_cmd[i].kd = self.Kd
                    self.low_cmd.motor_cmd[i].tau = 0
                
                if cycle_count % 250 == 0:  # Print every 0.5 seconds
                    print(f"Stage 1 progress: {self.percent_1*100:.1f}%")
            
            # Stage 2: Move to stand position
            elif self.percent_1 >= 1.0 and self.percent_2 < 1.0:
                if self.percent_2 == 0:
                    print("Stage 2: Standing up...")
                
                self.percent_2 += 1.0 / self.duration_2
                self.percent_2 = min(self.percent_2, 1.0)
                
                for i in range(12):
                    self.low_cmd.motor_cmd[i].q = ((1 - self.percent_2) * self.targetPos_1[i] + 
                                                  self.percent_2 * self.targetPos_2[i])
                    self.low_cmd.motor_cmd[i].dq = 0
                    self.low_cmd.motor_cmd[i].kp = self.Kp
                    self.low_cmd.motor_cmd[i].kd = self.Kd
                    self.low_cmd.motor_cmd[i].tau = 0
                
                if cycle_count % 250 == 0:
                    print(f"Stage 2 progress: {self.percent_2*100:.1f}%")
            
            # Stage 3: Hold stand pose and demonstrate wheel movement
            elif self.percent_2 >= 1.0 and self.percent_3 < 1.0:
                if self.percent_3 == 0:
                    print("Stage 3: Wheel movement demonstration...")
                
                self.percent_3 += 1.0 / self.duration_3
                self.percent_3 = min(self.percent_3, 1.0)
                
                # Hold legs in standing position
                for i in range(12):
                    self.low_cmd.motor_cmd[i].q = self.targetPos_2[i]
                    self.low_cmd.motor_cmd[i].dq = 0
                    self.low_cmd.motor_cmd[i].kp = self.Kp
                    self.low_cmd.motor_cmd[i].kd = self.Kd
                    self.low_cmd.motor_cmd[i].tau = 0
                
                # Wheel movement sequence (if Go2W has wheels in simulation)
                # Note: Regular Go2 simulation may not have wheels
                if self.percent_3 < 0.4:
                    # Forward wheel movement
                    for i in range(12, 16):
                        if i < 20:  # Safety check
                            self.low_cmd.motor_cmd[i].q = 0
                            self.low_cmd.motor_cmd[i].kp = 0.0
                            self.low_cmd.motor_cmd[i].dq = 3.0  # Forward
                            self.low_cmd.motor_cmd[i].kd = self.Kd
                            self.low_cmd.motor_cmd[i].tau = 0
                    
                    if cycle_count % 400 == 0:
                        print("Stage 3: Wheels moving forward...")
                
                elif 0.4 <= self.percent_3 < 0.8:
                    # Backward wheel movement
                    for i in range(12, 16):
                        if i < 20:
                            self.low_cmd.motor_cmd[i].q = 0
                            self.low_cmd.motor_cmd[i].kp = 0
                            self.low_cmd.motor_cmd[i].dq = -3.0  # Backward
                            self.low_cmd.motor_cmd[i].kd = self.Kd
                            self.low_cmd.motor_cmd[i].tau = 0
                    
                    if cycle_count % 400 == 0:
                        print("Stage 3: Wheels moving backward...")
                
                else:
                    # Stop wheels
                    for i in range(12, 16):
                        if i < 20:
                            self.low_cmd.motor_cmd[i].q = 0
                            self.low_cmd.motor_cmd[i].kp = 0
                            self.low_cmd.motor_cmd[i].dq = 0
                            self.low_cmd.motor_cmd[i].kd = self.Kd
                            self.low_cmd.motor_cmd[i].tau = 0
                    
                    if cycle_count % 400 == 0:
                        print("Stage 3: Wheels stopped...")
            
            # Stage 4: Move to wide sit position
            elif self.percent_3 >= 1.0 and self.percent_4 < 1.0:
                # if not self.done:
                #     print("Sequence completed! Holding final position...")
                #     self.done = True
                if self.percent_4 == 0:
                    print("Stage 4: Moving to wide sit...")
                
                self.percent_4 += 1.0 / self.duration_4
                self.percent_4 = min(self.percent_4, 1.0)
                
                for i in range(12):
                    self.low_cmd.motor_cmd[i].q = ((1 - self.percent_4) * self.targetPos_2[i] + 
                                                  self.percent_4 * self.targetPos_3[i])
                    self.low_cmd.motor_cmd[i].dq = 0
                    self.low_cmd.motor_cmd[i].kp = self.Kp
                    self.low_cmd.motor_cmd[i].kd = self.Kd
                    self.low_cmd.motor_cmd[i].tau = 0
                
                if cycle_count % 180 == 0:
                    print(f"Stage 4 progress: {self.percent_4*100:.1f}%")
            
            # Sequence complete
            else:
                if not self.done:
                    print("Sequence completed successfully!")
                    self.done = True
                break
            
            # Send command
            self.low_cmd.crc = self.crc.Crc(self.low_cmd)
            self.lowcmd_publisher.Write(self.low_cmd)
            
            time.sleep(self.dt)
            cycle_count += 1

def main():
    print("Go2W Simulation Stand Example")
    print("This will demonstrate a 4-stage movement sequence:")
    print("1. Sit down")
    print("2. Stand up") 
    print("3. Wheel movement (if available)")
    print("4. Wide sit position")
    print()
    
    input("Press Enter to start...")
    
    controller = Go2WSimulationStand()
    
    try:
        controller.run_sequence()
        print("Demo completed successfully!")
        time.sleep(2.0)
    except KeyboardInterrupt:
        print("\nDemo interrupted by user")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == '__main__':
    main()