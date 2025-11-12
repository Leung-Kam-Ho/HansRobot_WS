# HansRobot-WS

A Python library for controlling Elfin robot arms via TCP/IP communication, featuring behavior tree integration for complex robotic tasks.

## Features

- Full control of Elfin robot arm movements (joint and linear)
- TCP/IP communication protocol implementation
- Behavior tree support using py-trees for task orchestration
- Gripper control (Robotiq compatible)
- I/O operations
- Speed and position monitoring

## Installation

This project uses `uv` for dependency management. Install dependencies with:

```bash
uv sync
```

## Usage

### Basic Robot Arm Control

```python
from robot_arm.robot_arm import RobotArm
from robot_arm.position import Cartesian

# Connect to robot
ra = RobotArm('192.168.1.100')  # Replace with your robot's IP

# Power on and initialize
ra.power_on()
ra.start_master()
ra.servo_on()

# Move to a position
pos = Cartesian(x=100, y=200, z=300, rx=0, ry=0, rz=0)
ra.move_linear(pos)

# Control gripper
ra.move_gripper(100)  # Open
ra.move_gripper(0)    # Close

# Shutdown
ra.servo_off()
ra.close_master()
ra.power_off()
```

### Behavior Tree Integration

See `MoveInCube.py` and `pour_wine.py` for examples of using behavior trees for complex sequences.

***Pour Water***

***Move In Cube***

## Project Structure

- `robot_arm/`: Core library modules
  - `robot_arm.py`: Main RobotArm class
  - `tcp.py`: TCP communication
  - `position.py`: Position data structures
  - `ra_error.py`: Error handling
- `MoveInCube.py`: Example behavior tree for moving in a cube pattern
- `pour_wine.py`: Example wine pouring task
- `main.py`: Simple hello world script

## Protocol Reference

See `Reference/Elfin-TCPIP_Communication_Protocol.pdf` for detailed protocol documentation.

## Requirements

- Python >= 3.10
- py-trees >= 2.3.0
- Elfin robot arm with TCP/IP interface