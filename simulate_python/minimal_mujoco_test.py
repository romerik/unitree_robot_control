import mujoco
import mujoco.viewer
import time
import numpy as np

# Load the robot model
model_path = "../unitree_robots/go2/scene.xml"
print(f"Loading model from: {model_path}")

try:
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    print("Model loaded successfully!")
    print(f"Number of DOF: {model.nv}")
    print(f"Number of actuators: {model.nu}")
    
    # Launch viewer
    print("Starting viewer...")
    viewer = mujoco.viewer.launch_passive(model, data)
    
    print("Viewer started. Testing physics...")
    print("You should see the robot fall due to gravity")
    print("Try dragging the robot with your mouse")
    print("Press Ctrl+C to exit")
    
    # Simple physics loop
    step_count = 0
    while viewer.is_running():
        # Apply small random forces to make movement obvious
        if step_count % 100 == 0:
            # Add small random force to the base
            data.qfrc_applied[0] = np.random.normal(0, 1)
            data.qfrc_applied[1] = np.random.normal(0, 1)
            print(f"Step {step_count}: Applied random force")
        
        # Step physics
        mujoco.mj_step(model, data)
        
        # Update viewer
        viewer.sync()
        
        # Small delay
        time.sleep(0.01)
        step_count += 1
        
        if step_count > 1000:  # Reset forces after some steps
            data.qfrc_applied[:] = 0
            step_count = 0

except Exception as e:
    print(f"Error: {e}")
    import traceback
    traceback.print_exc()