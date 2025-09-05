import time
import sys
import math
import numpy as np
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_, unitree_go_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC

class Go2LowLevelController:
    def __init__(self):
        # Initialize communication
        ChannelFactoryInitialize(1, "lo")
        
        # Publisher for commands
        self.pub = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.pub.Init()
        
        # Subscriber for state
        self.low_state_suber = ChannelSubscriber("rt/lowstate", LowState_)
        self.low_state_suber.Init(self._state_handler, 10)
        
        # Current robot state
        self.current_positions = [0.0] * 12
        self.current_velocities = [0.0] * 12
        
        # Command setup
        self.cmd = unitree_go_msg_dds__LowCmd_()
        self.cmd.head[0] = 0xFE
        self.cmd.head[1] = 0xEF
        self.cmd.level_flag = 0xFF
        self.cmd.gpio = 0
        
        self.crc = CRC()
        
        # Initialize motor commands
        for i in range(20):
            self.cmd.motor_cmd[i].mode = 0x01
            self.cmd.motor_cmd[i].q = 0.0
            self.cmd.motor_cmd[i].kp = 0.0
            self.cmd.motor_cmd[i].dq = 0.0
            self.cmd.motor_cmd[i].kd = 0.0
            self.cmd.motor_cmd[i].tau = 0.0
        
        # Robot poses
        self.stand_pose = np.array([
            0.0, 0.67, -1.3,  # FR leg
            0.0, 0.67, -1.3,  # FL leg  
            0.0, 0.67, -1.3,  # RR leg
            0.0, 0.67, -1.3   # RL leg
        ])
        
        self.sit_pose = np.array([
            0.0, 1.2, -2.4,   # FR leg
            0.0, 1.2, -2.4,   # FL leg
            0.0, 1.2, -2.4,   # RR leg  
            0.0, 1.2, -2.4    # RL leg
        ])
        
        self.hello_poses = [
            # Wave sequence - lift front right leg
            np.array([0.5, 0.5, -1.0, 0.0, 0.67, -1.3, 0.0, 0.67, -1.3, 0.0, 0.67, -1.3]),
            np.array([0.8, 0.3, -0.8, 0.0, 0.67, -1.3, 0.0, 0.67, -1.3, 0.0, 0.67, -1.3]),
            np.array([0.5, 0.5, -1.0, 0.0, 0.67, -1.3, 0.0, 0.67, -1.3, 0.0, 0.67, -1.3]),
        ]
        
        # Wait for initial state
        print("Initializing controller...")
        time.sleep(1.0)
    
    def _state_handler(self, msg: LowState_):
        """Handle incoming state messages"""
        for i in range(12):
            self.current_positions[i] = msg.motor_state[i].q
            self.current_velocities[i] = msg.motor_state[i].dq
    
    def _send_command(self):
        """Send current command to robot"""
        self.cmd.crc = self.crc.Crc(self.cmd)
        self.pub.Write(self.cmd)
    
    def move_to_pose(self, target_pose, duration=2.0, kp=50.0, kd=3.0):
        """Smoothly move to target pose"""
        print(f"Moving to target pose over {duration} seconds...")
        
        start_pose = np.array(self.current_positions[:12])
        dt = 0.02  # 50Hz
        steps = int(duration / dt)
        
        for step in range(steps):
            # Smooth interpolation using tanh
            alpha = np.tanh(4 * step / steps)  # 0 to ~1
            current_target = (1 - alpha) * start_pose + alpha * target_pose
            
            # Set motor commands
            for i in range(12):
                self.cmd.motor_cmd[i].q = current_target[i]
                self.cmd.motor_cmd[i].kp = kp
                self.cmd.motor_cmd[i].dq = 0.0
                self.cmd.motor_cmd[i].kd = kd
                self.cmd.motor_cmd[i].tau = 0.0
            
            self._send_command()
            time.sleep(dt)
        
        print("Movement completed.")
    
    def hold_pose(self, duration=1.0, kp=30.0, kd=3.0):
        """Hold current position"""
        target_pose = np.array(self.current_positions[:12])
        dt = 0.02
        steps = int(duration / dt)
        
        for step in range(steps):
            for i in range(12):
                self.cmd.motor_cmd[i].q = target_pose[i]
                self.cmd.motor_cmd[i].kp = kp
                self.cmd.motor_cmd[i].dq = 0.0
                self.cmd.motor_cmd[i].kd = kd
                self.cmd.motor_cmd[i].tau = 0.0
            
            self._send_command()
            time.sleep(dt)
    
    def stand_up(self):
        """Stand up the robot"""
        print("Standing up...")
        self.move_to_pose(self.stand_pose, duration=3.0)
        self.hold_pose(1.0)
    
    def sit_down(self):
        """Sit down the robot"""
        print("Sitting down...")
        self.move_to_pose(self.sit_pose, duration=3.0)
        self.hold_pose(1.0)
    
    def hello_wave(self):
        """Perform a hello wave with front right leg"""
        print("Waving hello...")
        
        # First stand up
        self.move_to_pose(self.stand_pose, duration=2.0)
        time.sleep(0.5)
        
        # Wave sequence
        for pose in self.hello_poses:
            self.move_to_pose(pose, duration=0.8, kp=30.0)
            time.sleep(0.2)
        
        # Return to stand
        self.move_to_pose(self.stand_pose, duration=1.5)
        self.hold_pose(1.0)
    
    def simple_walk_forward(self, steps=4):
        """Simple walking pattern forward"""
        print(f"Walking forward {steps} steps...")
        
        # Start from standing
        self.move_to_pose(self.stand_pose, duration=2.0)
        
        # Walking gait - simplified
        for step in range(steps):
            print(f"Step {step + 1}/{steps}")
            
            # Lift front right and rear left legs
            walk_pose1 = self.stand_pose.copy()
            walk_pose1[1] += 0.3  # FR thigh up
            walk_pose1[7] += 0.3  # RR thigh up
            self.move_to_pose(walk_pose1, duration=0.5, kp=40.0)
            
            # Move body forward slightly
            walk_pose2 = walk_pose1.copy()
            # This would require body position control which is complex in low-level mode
            self.move_to_pose(walk_pose2, duration=0.3)
            
            # Put legs down
            self.move_to_pose(self.stand_pose, duration=0.5)
            
            # Lift front left and rear right legs  
            walk_pose3 = self.stand_pose.copy()
            walk_pose3[4] += 0.3  # FL thigh up
            walk_pose3[10] += 0.3  # RL thigh up
            self.move_to_pose(walk_pose3, duration=0.5, kp=40.0)
            
            # Move and put down
            self.move_to_pose(self.stand_pose, duration=0.5)
        
        print("Walking completed.")
    
    def dance_simple(self):
        """Simple dance routine"""
        print("Dancing...")
        
        # Stand up first
        self.move_to_pose(self.stand_pose, duration=2.0)
        
        # Dance moves
        moves = [
            # Crouch and stand
            self.sit_pose,
            self.stand_pose,
            # Lean left (shift weight)
            np.array([0.2, 0.67, -1.3, -0.2, 0.67, -1.3, 0.2, 0.67, -1.3, -0.2, 0.67, -1.3]),
            # Lean right
            np.array([-0.2, 0.67, -1.3, 0.2, 0.67, -1.3, -0.2, 0.67, -1.3, 0.2, 0.67, -1.3]),
            # Back to center
            self.stand_pose,
        ]
        
        for i, move in enumerate(moves):
            print(f"Dance move {i + 1}/{len(moves)}")
            self.move_to_pose(move, duration=1.0, kp=40.0)
            time.sleep(0.2)
        
        print("Dance completed!")
    
    def shutdown(self):
        """Safe shutdown - relax all motors"""
        print("Shutting down - relaxing motors...")
        for i in range(12):
            self.cmd.motor_cmd[i].q = 0.0
            self.cmd.motor_cmd[i].kp = 0.0
            self.cmd.motor_cmd[i].dq = 0.0
            self.cmd.motor_cmd[i].kd = 0.0
            self.cmd.motor_cmd[i].tau = 0.0
        
        for _ in range(10):
            self._send_command()
            time.sleep(0.1)

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 low_level_movements.py <command>")
        print("Commands:")
        print("  stand    - Stand up")
        print("  sit      - Sit down") 
        print("  hello    - Wave hello")
        print("  walk     - Simple walk forward")
        print("  dance    - Simple dance routine")
        return
    
    command = sys.argv[1].lower()
    
    # Create controller
    controller = Go2LowLevelController()
    
    try:
        if command == "stand":
            controller.stand_up()
        elif command == "sit":
            controller.sit_down()
        elif command == "hello":
            controller.hello_wave()
        elif command == "walk":
            controller.simple_walk_forward()
        elif command == "dance":
            controller.dance_simple()
        else:
            print(f"Unknown command: {command}")
            return
        
        print("Command completed successfully!")
        
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        controller.shutdown()

if __name__ == "__main__":
    main()