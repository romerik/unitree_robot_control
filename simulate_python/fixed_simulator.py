import time
import mujoco
import mujoco.viewer
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py_bridge import UnitreeSdk2Bridge
import config

# Load model and create data
print("Loading robot model...")
mj_model = mujoco.MjModel.from_xml_path(config.ROBOT_SCENE)
mj_data = mujoco.MjData(mj_model)

print(f"Model loaded: {config.ROBOT}")
print(f"Number of actuators: {mj_model.nu}")
print(f"Timestep: {mj_model.opt.timestep}")

# Launch viewer
print("Starting MuJoCo viewer...")
viewer = mujoco.viewer.launch_passive(mj_model, mj_data)

# Initialize Unitree bridge
print("Initializing Unitree SDK bridge...")
ChannelFactoryInitialize(config.DOMAIN_ID, config.INTERFACE)
unitree_bridge = UnitreeSdk2Bridge(mj_model, mj_data)

if config.PRINT_SCENE_INFORMATION:
    unitree_bridge.PrintSceneInformation()

print("Starting simulation loop...")
print("Press Ctrl+C to exit")

step_count = 0
last_print_time = time.time()

try:
    while viewer.is_running():
        step_start = time.perf_counter()
        
        # Step physics - this is critical!
        mujoco.mj_step(mj_model, mj_data)
        
        # Update viewer
        viewer.sync()
        
        step_count += 1
        
        # Print debug info every 2 seconds
        current_time = time.time()
        if current_time - last_print_time > 2.0:
            print(f"Simulation running - Step: {step_count}")
            print(f"  Motor 0 position: {mj_data.sensordata[0]:.4f}")
            print(f"  Motor 0 control: {mj_data.ctrl[0]:.4f}")
            print(f"  Robot base height: {mj_data.qpos[2]:.4f}")
            last_print_time = current_time
        
        # Maintain timestep
        time_until_next_step = mj_model.opt.timestep - (time.perf_counter() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)

except KeyboardInterrupt:
    print("\nShutting down simulator...")
    
print("Simulator stopped.")