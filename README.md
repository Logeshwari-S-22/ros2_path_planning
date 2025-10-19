# ROS2 Path Planning Project

## Overview
This project demonstrates **path planning for a mobile robot** using **ROS 2 Jazzy**, **Nav2**, and **RViz 2**.  
The robot can navigate from a start position to a goal position using the navigation stack in a Gazebo simulation environment.  
This project includes all steps from installing required packages to running Navigation2 with a saved map (Steps 1–4).

---

## Tools & Technologies
- ROS 2 Jazzy
- Nav2 (Navigation2 stack)
- RViz 2
- TurtleBot3 Simulation
- SLAM Toolbox
- Gazebo

---

## Project Steps

### Step 1 — Install Required Packages
Run these commands once (if not already installed):
```bash
sudo apt update
sudo apt install ros-jazzy-turtlebot3* ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox
Then permanently set the robot model:

bash
Copy code
echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
source ~/.bashrc
Step 2 — Launch Simulation (Gazebo)
bash
Copy code
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
Keep this terminal open — this is your simulated robot.

Step 3 — Build a Map (SLAM)
Open a new terminal and run:

bash
Copy code
source /opt/ros/jazzy/setup.bash
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
Open another terminal for keyboard control:

bash
Copy code
source /opt/ros/jazzy/setup.bash
ros2 run turtlebot3_teleop teleop_keyboard
Drive the robot around until the map forms in RViz.

Save the map:

bash
Copy code
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/map
This creates map.yaml and map.pgm in your workspace.

Step 4 — Run Navigation 2 with Saved Map
Stop SLAM nodes (Ctrl+C), keep Gazebo running.
Then launch Navigation2:

bash
Copy code
ros2 launch nav2_bringup bringup_launch.py use_sim_time:=True map:=~/ros2_ws/src/map.yaml
In RViz:

Use “2D Pose Estimate” to set the robot’s initial position.

Use “2D Nav Goal” to click the goal point.

✅ You should see a green global path and blue local trajectory as the robot moves.

Folder Structure
python
Copy code
ros2_ws/
 ├── src/         # ROS2 packages
 ├── map.yaml     # Saved map
 ├── map.pgm
 └── README.md
## URDF (Robot Description)

This project uses the **TurtleBot3 URDF model** (`turtlebot3_burger.urdf.xacro`) for:

- Defining the robot’s physical structure (links, joints, dimensions)
- Simulating sensors (LIDAR, camera) in Gazebo
- Providing the footprint and transforms to Navigation2 for path planning

The URDF file is included in the **TurtleBot3 description package** and loaded automatically during simulation.


Notes
This project demonstrates autonomous navigation up to Step 4.

Robot moves automatically using Nav2’s default global and local planners.

No custom planner plugin is implemented in this version.


