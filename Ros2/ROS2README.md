# ROS2 Robotics: Publisher-Subscriber, LaserScan, and Waypoint Navigation  

This repository contains **three ROS2 implementations**:  
✅ **Publisher-Subscriber:** A simple text-based communication between nodes.  
✅ **LaserScan Visualization:** Reads and displays LiDAR distance data.  
✅ **Waypoint Navigation:** Sends navigation goals to a robot.  

---

## Getting Started  

### 1️⃣ **Installing ROS2**  
Ensure ROS2 is installed. Follow the official guide:  
🔗 [ROS2 Installation Guide](https://docs.ros.org/en/rolling/Installation.html)  

### 2️⃣ **Cloning the Repository**  
```sh
git clone https://github.com/your_username/ros2_robotics.git
cd ros2_robotics
```

### 3️⃣ **Running the Publisher-Subscriber**
Open two terminals:
Terminal 1 (Publisher):

```sh
ros2 run my_package talker
Terminal 2 (Subscriber):
```

```sh
ros2 run my_package listener
```

Messages will be sent and received!

### 4️⃣ **Running LaserScan Visualization**
Ensure a robot with a LiDAR sensor is running (e.g., TurtleBot3).

Run the visualizer:

```sh
ros2 run my_package laserscan_visualizer
```
The minimum detected obstacle distance will be displayed!

### Alternate way : Using Rviz2
If you prefer RViz2, you don't need this script. Instead:

1. Launch the Robot Simulation
Before running RViz2, ensure the robot is running and publishing LaserScan data:

```sh
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
or, if using a real TurtleBot3:

```sh
ros2 launch turtlebot3_bringup robot.launch.py
```

2. Launch RViz2
Start RViz2 in a new terminal:
```sh
ros2 run rviz2 rviz2
```

3. Add LaserScan Data to RViz2
Once RViz2 opens:
- Click "Add" (Bottom left in Displays panel).
- Select "By topic" → /scan → "LaserScan".
- Click OK.
- Expand "LaserScan" settings and modify:
    - Style: Points (or Lines for connected points)
    - Size: Increase to make points visible
    - Decay Time: 0 (so points don't disappear)
    - Color: Change if needed for better visibility


5️⃣ Running Waypoint Navigation
1. Ensure Navigation2 is running, then send waypoints:

```sh
ros2 run my_package waypoint_nav
```

### Alternate method:
1. Launch the Robot in Simulation
Ensure that your robot is running in Gazebo:
```sh
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
If using a real TurtleBot3:
```sh
ros2 launch turtlebot3_bringup robot.launch.py
```

2. Launch Navigation2
Run the Navigation2 stack so that the robot can navigate:
```sh
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=~/my_map.yaml
```
Make sure you have already created and saved your map!
If you haven’t, create a map first using SLAM Toolbox.

3. Open RViz2 for Navigation
Start RViz2 with Navigation2 settings:
```sh
ros2 launch nav2_bringup rviz_launch.py
```

4. Set an Initial Pose in RViz2
In RViz2:
- Click the “2D Pose Estimate” tool (top bar).
- Click on the map where the robot actually is.

This tells Navigation2 where the robot’s current position is.

5. Use the "Nav2 Goal" Tool for Waypoints
- Click "Nav2 Goal" in RViz2.
- Click on the map to set the target waypoint.

The robot should move to that location.

To add multiple waypoints, repeat this process. You can click different places on the map, and the robot will follow them one by one.

The robot will navigate to the defined waypoints!
