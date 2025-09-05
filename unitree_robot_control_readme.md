# Unitree Robot Control

This repository contains simulation and control examples for Unitree robots (Go2, Go2W, G1) using the Unitree SDK2 Python interface with MuJoCo physics simulation.

## About

This project is based on and extends the official [Unitree MuJoCo simulation](https://github.com/unitreerobotics/unitree_mujoco) repository. Key modifications and additions include:

- Fixed threading synchronization issues in the simulator for more reliable operation
- Added comprehensive movement examples for different robot types
- Created low-level control demonstrations for Go2, Go2W, and G1 robots
- Enhanced documentation and usage examples
- Improved sim-to-real transition examples

**Original Repository**: [unitreerobotics/unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco)  
**License**: Please refer to the original repository's license terms.

## Prerequisites

- Python 3.10 or compatible version
- Ubuntu/Linux system
- MuJoCo physics engine
- Unitree SDK2 Python

## Installation

### 1. Clone the Repository

```bash
git clone https://github.com/romerik/unitree_robot_control.git
cd unitree_robot_control
```

### 2. Create Virtual Environment

```bash
python3 -m venv env
source env/bin/activate
```

### 3. Install Unitree SDK2 Python

```bash
cd ~
sudo apt install python3-pip
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
cd unitree_sdk2_python
pip3 install -e .
```

### 4. Install MuJoCo Python

```bash
pip3 install mujoco
```

### 5. Install Joystick Support

```bash
pip3 install pygame
```

## Quick Test

### Start the Simulator

```bash
cd ./simulate_python
python3 fixed_simulator.py
```

You should see the MuJoCo simulator with the Go2 robot loaded.

### Test Basic Communication

In a new terminal:

```bash
cd ./example/python
python3 debug_sensor_values.py
```

The program will output the robot's pose and position information in the simulator, and test motor control functionality.

**Note:** The testing program sends the unitree_go message. If you want to test G1 robot, you need to modify the program to use the unitree_hg message.

## Configuration

### Python Simulator Configuration

The configuration file for the Python simulator is located at `/simulate_python/config.py`:

```python
# Robot name loaded by the simulator
# "go2", "b2", "b2w", "h1", "g1"
ROBOT = "go2"

# Robot simulation scene file
ROBOT_SCENE = "../unitree_robots/" + ROBOT + "/scene.xml"  # Robot scene
# For G1, use: ROBOT_SCENE = "../unitree_robots/g1/scene_29dof.xml"

# DDS domain id, it is recommended to distinguish from the real robot (default is 0 on the real robot)
DOMAIN_ID = 1  # Domain id

# Network interface name, for simulation, it is recommended to use the local loopback "lo"
INTERFACE = "lo"  # Interface

# Whether to output robot link, joint, sensor information, True for output
PRINT_SCENE_INFORMATION = True

USE_JOYSTICK = 1 # Simulate Unitree WirelessController using a gamepad
JOYSTICK_TYPE = "xbox" # support "xbox" and "switch" gamepad layout
JOYSTICK_DEVICE = 0 # Joystick number

# Whether to use virtual tape, 1 to enable
# Mainly used to simulate the hanging process of H1 robot initialization
ENABLE_ELASTIC_BAND = False

# Simulation time step (unit: s)
# To ensure the reliability of the simulation, it needs to be greater than the time required for viewer.sync() to render once
SIMULATE_DT = 0.003  

# Visualization interface runtime step, 0.02 corresponds to 50fps/s
VIEWER_DT = 0.02
```

## Usage Examples

### Go2 Quadruped Robot

#### Basic Stand/Sit Sequence

```bash
# Control robot in simulation (make sure Go2 simulation scene is loaded)
python3 stand_go2.py

# Control physical robot (replace enp3s0 with your network interface)
python3 stand_go2.py enp3s0
```

### Go2W (Go2 with Wheels)

```bash
# Complex 4-stage movement sequence with wheel control
python3 go2w_simulation_stand.py
```

This demonstrates:
1. Sit down (1 second)
2. Stand up (1 second)  
3. Wheel movement demonstration (4 seconds)
4. Wide sit position (1.8 seconds)

### G1 Humanoid Robot

First, update config.py for G1:

```python
ROBOT = "g1"
ROBOT_SCENE = "../unitree_robots/g1/scene_29dof.xml"
```

Then run:

```bash
python3 g1_simulation_control.py
```

This demonstrates G1-specific movements including ankle control in different modes (PR/AB) and wrist movements.

## Sim to Real

All example scripts include sim-to-real capability using this pattern:

```python
if len(sys.argv) < 2:
    # If no network card is input, use the simulated domain id and the local network card
    ChannelFactoryInitialize(1, "lo")
else:
    # Otherwise, use the specified network card
    ChannelFactoryInitialize(0, sys.argv[1])
```

## Robot Models

- **Go2**: 12 DOF quadruped robot
- **Go2W**: Go2 with additional wheel actuators (hybrid locomotion)
- **G1**: 29 DOF humanoid robot with full arm and leg control

## Key Files

- `fixed_simulator.py`: Main simulator with proper physics threading
- `stand_go2.py`: Basic standing/sitting demonstration
- `go2w_simulation_stand.py`: Go2W-specific multi-stage sequence
- `g1_simulation_control.py`: G1 humanoid control example
- `debug_sensor_values.py`: Sensor feedback testing

## Troubleshooting

### Common Issues

1. **Robot falls down in simulation**: This is normal for humanoid robots (G1) without active balance control
2. **Threading issues**: Use `fixed_simulator.py` instead of the original `unitree_mujoco.py`
3. **Sensor values stuck at zero**: Ensure proper physics simulation timing and threading

### Performance Notes

- Simulation runs at 500Hz control frequency (2ms timestep)
- MuJoCo viewer renders at 50Hz for smooth visualization
- Use the fixed simulator to avoid physics/control synchronization issues

## License

This project builds upon the Unitree SDK2 and MuJoCo simulator. Please refer to their respective licenses for usage terms.

## Contributing

Feel free to submit issues and enhancement requests through GitHub issues.