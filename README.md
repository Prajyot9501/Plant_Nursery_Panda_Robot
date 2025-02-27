# 🌱 Project Overview
The Plant Nursery Robot is an innovative automation solution that combines the precision of the Franka Emika Panda robot with intelligent plant care systems. This project aims to address the challenge of maintaining consistent, round-the-clock care for growing plants while optimizing human resources in modern nurseries.
![sim](https://github.com/user-attachments/assets/1726abf8-75ce-4307-b324-8e6e03fa8cca)

<img src="https://github.com/user-attachments/assets/797ba25b-1852-4861-96c1-2cfc7b43bcd1" width="400">

# Key Features

- Automated Plant Care Sequence: Complete end-to-end care from pot positioning to watering
- Intelligent Soil Moisture Sensing: Real-time feedback system to guide watering decisions
- Interactive User Interface: Flexible menu system for task selection and operation control
- Precision Handling: Careful manipulation of plants, pots, and watering equipment
- Safety-First Design: Comprehensive error handling and operational safety features

# User Interface
The system provides an interactive menu that allows operators to select specific tasks:
<img src="https://github.com/user-attachments/assets/186e6b3a-234c-4be2-af8d-d03c664efbe1" width="400"> 
- Pot Positioning: Places pots in optimal locations
- Soil Moisture Check: Evaluates soil conditions using sensors
- Watering: Provides precise watering based on moisture readings
- Plant Handling: Carefully places and adjusts plants
- Complete Care Sequence: Performs all tasks in sequence

# 🤖 System Components
## Hardware

- Franka Emika Panda robot arm
- Custom end effectors for different tasks
- Soil moisture sensors
- Plant pots of various sizes
- Specialized watering equipment

## Software

- ROS (Robot Operating System) integration
- Motion planning and control system
- Sensor data processing
- User interface for task selection
- Safety monitoring system

# 🔧 Installation
## Prerequisites

- [Ubuntu 20.04](https://releases.ubuntu.com/focal/)
- [ROS Noetic](https://wiki.ros.org/noetic)
- [Franka Panda Robotic Arm Model](https://robodk.com/robot/Franka/Emika-Panda)
- [Python 3.8+](https://www.python.org/downloads/release/python-380/)

# Setup Instructions

### Clone the repository

```
git clone https://github.com/your-username/Nursery_Robot_Panda.git
cd Nursery_Robot_Panda/catkin_ws
```

### Install dependencies
```
rosdep install --from-paths src --ignore-src -r -y
```
### Build the workspace
```
catkin_make
```
>[!NOTE]
>Source the workspace after building it

# Launching the System
```
# Launch the main system
roslaunch nursery_robot main.launch

# Launch just the simulation environment
roslaunch nursery_robot simulation.launch
```

#  Contributing
Contributions to improve the Plant Nursery Robot are welcome! Please feel free to submit a Pull Request.

#  License
This project is licensed under the MIT License 

#  Acknowledgments
* Franka Emika for their collaborative robot platform
*  The ROS community for their invaluable tools and resources
