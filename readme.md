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


## Real Robot High-Level Control

This section contains high-level control scripts for commanding physical Unitree robots using the SportClient API. These scripts are designed for real robot operation and require proper network connectivity to the robot.

### Prerequisites for Real Robot Control

- Physical Unitree robot (Go2, Go2W, or G1)
- Network connection to the robot
- Robot in appropriate mode for receiving commands
- Sufficient clear space around the robot

### Available High-Level Controllers

#### Go2 Quadruped Robot

```bash
cd real_robot_high_level

# Display all available commands
python3 go2_high_level.py

# Basic commands
python3 go2_high_level.py eth0 1          # Stand up
python3 go2_high_level.py eth0 2          # Stand down
python3 go2_high_level.py eth0 40         # Hello gesture

# Movement commands
python3 go2_high_level.py eth0 10         # Move forward 1m
python3 go2_high_level.py eth0 20 2.5     # Move forward 2.5m
python3 go2_high_level.py eth0 24 45      # Rotate left 45 degrees

# Fun actions
python3 go2_high_level.py eth0 44         # Dance (random selection)
python3 go2_high_level.py eth0 47         # Heart gesture
```

#### Go2W (Go2 with Wheels)

```bash
# Go2W-specific commands including wheel control
...
```

#### G1 Humanoid Robot

```bash
# G1-specific humanoid commands
...
```


### Network Interface Setup

Replace `eth0` with your actual network interface connected to the robot:

```bash
# Find your network interface
ip addr show

# Common interface names:
# eth0, enp3s0 - Ethernet connections
# wlan0, wlp2s0 - WiFi connections
```

### Safety Guidelines

⚠️ **Important Safety Notes:**

1. **Clear Area**: Ensure at least 3 meters of clear space around the robot
2. **Emergency Stop**: Always have the physical emergency stop accessible
3. **Supervision**: Never leave the robot running unattended
4. **Stable Surface**: Operate only on flat, stable surfaces unless using terrain-specific modes
5. **Mode Awareness**: Some advanced commands require switching to AI mode first

### Troubleshooting

#### Common Issues

1. **Connection Failed**: Check network interface name and robot connectivity
2. **Command Rejected**: Ensure robot is in correct mode (try switching modes)
3. **Timeout Errors**: Verify network stability and robot responsiveness
4. **Mode Errors**: Some commands require specific robot modes

#### Error Codes

- `3102`: Service not available or robot not ready
- `4201`: Command timeout
- `4202`: Service not initialized

### Network Configuration

For optimal performance, ensure:

```bash
# Check network connectivity
ping [robot_ip]

# Verify interface is up
sudo ip link set [interface] up

# Check routing
ip route show
```

### Integration with Simulation

These high-level scripts are designed specifically for real robots and will not work with the simulation environment. For testing movements safely, use the simulation examples first before deploying on physical hardware.

## License

This project builds upon the Unitree SDK2 and MuJoCo simulator. Please refer to their respective licenses for usage terms.

## Contributing

Feel free to submit issues and enhancement requests through GitHub issues.