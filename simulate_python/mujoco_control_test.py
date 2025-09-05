import mujoco
import mujoco.viewer
import time
import numpy as np
import math

# Load the robot model
model_path = "../unitree_robots/go2/scene.xml"
print(f"Loading model from: {model_path}")

model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

print(f"Number of actuators: {model.nu}")
print(f"Number of joints: {model.nq}")

# Print actuator names to understand the mapping
print("\nActuator names:")
for i in range(model.nu):
    actuator_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
    print(f"  Actuator {i}: {actuator_name}")

print("\nJoint names:")
for i in range(model.nq):
    joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
    print(f"  Joint {i}: {joint_name}")

# Launch viewer
viewer = mujoco.viewer.launch_passive(model, data)

print("\nStarting direct motor control test...")
print("This will move the motors directly through MuJoCo")

t = 0
while viewer.is_running():
    # Reset the robot to a reasonable position first
    if t < 1.0:
        # Set a standing position directly
        if model.nq >= 12:
            # Hip joints (assuming they exist)
            for i in range(0, min(12, model.nu), 3):
                data.ctrl[i] = 0.0      # Hip
                if i+1 < model.nu:
                    data.ctrl[i+1] = 0.6   # Thigh  
                if i+2 < model.nu:
                    data.ctrl[i+2] = -1.2  # Calf
    else:
        # After 1 second, start moving motors
        # Move first actuator with sine wave
        if model.nu > 0:
            data.ctrl[0] = math.sin(t) * 0.5
            print(f"Time: {t:.2f}s, Motor 0 command: {data.ctrl[0]:.3f}")
    
    # Step physics
    mujoco.mj_step(model, data)
    
    # Update viewer
    viewer.sync()
    
    time.sleep(0.01)
    t += 0.01