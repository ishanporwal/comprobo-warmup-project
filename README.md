# RoboBehaviors and Finite State Machines Project

**Ishan Porwal and Ahan Trivedi**  
Olin College of Engineering — ENGR3590 Computational Introduction to Robotics

## Overview

For this project, we built a finite state controller in ROS 2 using C++ to control a Neato robot using LiDAR and bumper sensor data.

The controller has three main states:

- **Move Forward:** The Neato drives forward while the path ahead is clear.
- **Rotate:** If the LiDAR detects an obstacle in front of the robot, the Neato rotates in place until the path is clear again.
- **Stop:** If one of the physical bump sensors is triggered, the Neato enters a stop state and remains stopped.

The controller uses `/scan` and `/bump` as sensor inputs and publishes velocity commands to `/cmd_vel`.

For a full discussion of our design process, implementation, testing, challenges, and takeaways, see the [full project report](./Warmup%20Project%20Report.pdf).

## Project Structure

```text
src/ros_behaviors_fsm/
├── bags/
│   └── finite_state_controller_demo/
├── src/
│   └── finite_state_controller.cpp
├── CMakeLists.txt
└── package.xml
```

The main finite state controller implementation can be found here:

[`finite_state_controller.cpp`](src/ros_behaviors_fsm/src/finite_state_controller.cpp)

The recorded rosbag from our physical Neato demo can be found here:

[`finite_state_controller_demo`](src/ros_behaviors_fsm/bags/finite_state_controller_demo)

## How To Run

These instructions assume ROS 2 Jazzy and the required Neato ROS packages are already installed.

### Clone and Build

Clone the repository into the `src` directory of a ROS 2 workspace:

```bash
cd ~/ros2_ws/src
git clone https://github.com/ishanporwal/comprobo-warmup-project.git
```

Return to the workspace root, source ROS 2, build the package, and source the workspace:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ros_behaviors_fsm
source install/setup.bash
```

### Connect to the Neato

Make sure the laptop and Neato are connected to the same network and note the IP address displayed on the Neato.

In a separate terminal:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch neato_node2 bringup.py host:=<NEATO_IP>
```

### Run the Finite State Controller

Once the Neato is connected, open another terminal and run:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run ros_behaviors_fsm finite_state_controller
```

The Neato will begin moving forward. When an obstacle is detected in front of it, it will rotate until the path is clear and then continue moving forward. If a bump sensor is triggered, the robot will enter the stop state.

## Rosbag Playback

To inspect the included rosbag:

```bash
ros2 bag info src/ros_behaviors_fsm/bags/finite_state_controller_demo
```

To replay it:

```bash
ros2 bag play src/ros_behaviors_fsm/bags/finite_state_controller_demo
```

The bag contains LiDAR scans, bump sensor data, velocity commands, odometry, and transforms from our physical robot test.

## Demo

[Watch the full finite state controller demo](https://drive.google.com/file/d/1fRjs4UO5VT6rQYAgjOJW_Ndnz92QzVk4/view?usp=drivesdk)

## Full Writeup

For the complete project writeup, including our design process, FSM architecture, debugging process, challenges, future improvements, and learning takeaways, see the:

**[Full Project Report](./Report/Warmup%20Project%20Report.pdf)**

## Authors

**Ishan Porwal and Ahan Trivedi**

We worked together throughout the entire project, including planning the FSM and code architecture, implementation, debugging, simulation, physical Neato testing, rosbag recording, and the final demonstration.
