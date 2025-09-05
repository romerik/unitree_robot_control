import time
import sys
import numpy as np

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
from unitree_sdk2py.utils.crc import CRC

G1_NUM_MOTOR = 29

# Control gains for each motor
Kp = [
    60, 60, 60, 100, 40, 40,      # left leg
    60, 60, 60, 100, 40, 40,      # right leg
    60, 40, 40,                   # waist
    40, 40, 40, 40,  40, 40, 40,  # left arm
    40, 40, 40, 40,  40, 40, 40   # right arm
]

Kd = [
    1, 1, 1, 2, 1, 1,     # left leg
    1, 1, 1, 2, 1, 1,     # right leg
    1, 1, 1,              # waist
    1, 1, 1, 1, 1, 1, 1,  # left arm
    1, 1, 1, 1, 1, 1, 1   # right arm 
]

class G1JointIndex:
    # Legs
    LeftHipPitch = 0
    LeftHipRoll = 1
    LeftHipYaw = 2
    LeftKnee = 3
    LeftAnklePitch = 4
    LeftAnkleB = 4
    LeftAnkleRoll = 5
    LeftAnkleA = 5
    RightHipPitch = 6
    RightHipRoll = 7
    RightHipYaw = 8
    RightKnee = 9
    RightAnklePitch = 10
    RightAnkleB = 10
    RightAnkleRoll = 11
    RightAnkleA = 11
    
    # Waist
    WaistYaw = 12
    WaistRoll = 13
    WaistA = 13
    WaistPitch = 14
    WaistB = 14
    
    # Arms
    LeftShoulderPitch = 15
    LeftShoulderRoll = 16
    LeftShoulderYaw = 17
    LeftElbow = 18
    LeftWristRoll = 19
    LeftWristPitch = 20
    LeftWristYaw = 21
    RightShoulderPitch = 22
    RightShoulderRoll = 23
    RightShoulderYaw = 24
    RightElbow = 25
    RightWristRoll = 26
    RightWristPitch = 27
    RightWristYaw = 28

class Mode:
    PR = 0  # Series Control for Pitch/Roll Joints
    AB = 1  # Parallel Control for A/B Joints

class G1SimulationController:
    def __init__(self):
        self.time_ = 0.0
        self.control_dt_ = 0.002  # 2ms control loop
        self.duration_ = 3.0      # 3 second phases
        self.counter_ = 0
        self.mode_pr_ = Mode.PR
        self.mode_machine_ = 1    # Fixed for simulation
        
        # Command and state
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()  
        self.low_state = None 
        self.crc = CRC()
        
        # Initialize for simulation
        ChannelFactoryInitialize(1, "lo")  # Simulation mode
        
        # Publishers and subscribers
        self.lowcmd_publisher_ = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.lowcmd_publisher_.Init()
        
        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.LowStateHandler, 10)
        
        print("G1 simulation controller initialized")
        print("Waiting for robot state...")
        time.sleep(2.0)  # Wait for initial state
    
    def LowStateHandler(self, msg: LowState_):
        """Handle incoming state messages"""
        self.low_state = msg
        self.counter_ += 1
        
        # Print IMU data every 500 cycles (1 second)
        if (self.counter_ % 500 == 0):
            if hasattr(msg.imu_state, 'rpy'):
                print(f"IMU RPY: {msg.imu_state.rpy}")
            self.counter_ = 0
    
    def run_demo(self):
        """Run the G1 ankle and wrist demo"""
        print("Starting G1 demo sequence...")
        print("Stage 1: Moving to zero posture...")
        
        start_time = time.time()
        
        while True:
            current_time = time.time() - start_time
            self.time_ = current_time
            
            if self.low_state is None:
                print("Waiting for robot state...")
                time.sleep(0.1)
                continue
            
            # Stage 1: Move to zero posture (0-3 seconds)
            if self.time_ < self.duration_:
                ratio = np.clip(self.time_ / self.duration_, 0.0, 1.0)
                print(f"Stage 1 progress: {ratio*100:.1f}%")
                
                self.low_cmd.mode_pr = Mode.PR
                self.low_cmd.mode_machine = self.mode_machine_
                
                for i in range(G1_NUM_MOTOR):
                    self.low_cmd.motor_cmd[i].mode = 1  # Enable
                    self.low_cmd.motor_cmd[i].tau = 0.0
                    self.low_cmd.motor_cmd[i].q = (1.0 - ratio) * self.low_state.motor_state[i].q
                    self.low_cmd.motor_cmd[i].dq = 0.0
                    self.low_cmd.motor_cmd[i].kp = Kp[i]
                    self.low_cmd.motor_cmd[i].kd = Kd[i]
            
            # Stage 2: Swing ankles using PR mode (3-6 seconds)
            elif self.time_ < self.duration_ * 2:
                if self.time_ == self.duration_ or abs(self.time_ - self.duration_) < 0.01:
                    print("Stage 2: Swinging ankles (PR mode)...")
                
                max_P = np.pi * 30.0 / 180.0  # 30 degrees
                max_R = np.pi * 10.0 / 180.0  # 10 degrees
                t = self.time_ - self.duration_
                
                L_P_des = max_P * np.sin(2.0 * np.pi * t)
                L_R_des = max_R * np.sin(2.0 * np.pi * t)
                R_P_des = max_P * np.sin(2.0 * np.pi * t)
                R_R_des = -max_R * np.sin(2.0 * np.pi * t)
                
                self.low_cmd.mode_pr = Mode.PR
                self.low_cmd.mode_machine = self.mode_machine_
                
                # Set ankle positions
                self.low_cmd.motor_cmd[G1JointIndex.LeftAnklePitch].q = L_P_des
                self.low_cmd.motor_cmd[G1JointIndex.LeftAnkleRoll].q = L_R_des
                self.low_cmd.motor_cmd[G1JointIndex.RightAnklePitch].q = R_P_des
                self.low_cmd.motor_cmd[G1JointIndex.RightAnkleRoll].q = R_R_des
            
            # Stage 3: Swing ankles using AB mode + wrists (6+ seconds)
            else:
                if abs(self.time_ - self.duration_ * 2) < 0.01:
                    print("Stage 3: Swinging ankles (AB mode) + wrists...")
                
                max_A = np.pi * 30.0 / 180.0
                max_B = np.pi * 10.0 / 180.0
                t = self.time_ - self.duration_ * 2
                
                L_A_des = max_A * np.sin(2.0 * np.pi * t)
                L_B_des = max_B * np.sin(2.0 * np.pi * t + np.pi)
                R_A_des = -max_A * np.sin(2.0 * np.pi * t)
                R_B_des = -max_B * np.sin(2.0 * np.pi * t + np.pi)
                
                self.low_cmd.mode_pr = Mode.AB
                self.low_cmd.mode_machine = self.mode_machine_
                
                # Set ankle positions (AB mode)
                self.low_cmd.motor_cmd[G1JointIndex.LeftAnkleA].q = L_A_des
                self.low_cmd.motor_cmd[G1JointIndex.LeftAnkleB].q = L_B_des
                self.low_cmd.motor_cmd[G1JointIndex.RightAnkleA].q = R_A_des
                self.low_cmd.motor_cmd[G1JointIndex.RightAnkleB].q = R_B_des
                
                # Add wrist movement
                max_WristYaw = np.pi * 30.0 / 180.0
                L_WristYaw_des = max_WristYaw * np.sin(2.0 * np.pi * t)
                R_WristYaw_des = max_WristYaw * np.sin(2.0 * np.pi * t)
                
                self.low_cmd.motor_cmd[G1JointIndex.LeftWristRoll].q = L_WristYaw_des
                self.low_cmd.motor_cmd[G1JointIndex.RightWristRoll].q = R_WristYaw_des
            
            # Send command
            self.low_cmd.crc = self.crc.Crc(self.low_cmd)
            self.lowcmd_publisher_.Write(self.low_cmd)
            
            time.sleep(self.control_dt_)

def main():
    print("WARNING: Please ensure there are no obstacles around the robot.")
    print("This demo will move the G1 robot's ankles and wrists.")
    input("Press Enter to continue...")
    
    controller = G1SimulationController()
    
    try:
        controller.run_demo()
    except KeyboardInterrupt:
        print("\nDemo stopped by user")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == '__main__':
    main()