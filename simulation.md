# PROTOBOT – Simulation (Gazebo)

This guide explains how to run **Protobot** in simulation using Gazebo.  
Gazebo is useful for testing Navigation2 (Nav2), SLAM, and robot behavior without a physical robot.  

> ⚠️ Note: Gazebo is resource-intensive and may not run smoothly on a Raspberry Pi.  
> For full Nav2 simulation, it is recommended to use either:
> - A virtual machine (VirtualBox) running Ubuntu 24.04, or  
> - A dual-boot Ubuntu setup (more performance but more extreme setup)

---

# 🛠 1. Prepare Gazebo Worlds

Protobot comes with several pre-configured Gazebo worlds located in:

```

protobot/worlds

```

Available worlds include:

- `empty.world`  
- `exam_hall.world`  
- `obstacles.world`  
- `turtlebot3_dqn_stage1.world`  
- `turtlebot3_dqn_stage2.world`  
- `turtlebot3_dqn_stage3.world`  
- `turtlebot3_house.world`  
- `turtlebot3_world.world`  

---

## 1️⃣ Edit Launch File

Open:

```

protobot/launch/launch_sim_slam.launch.py

````

Locate the `default_world` variable:

```python
default_world = os.path.join(
    get_package_share_directory(package_name),
    'worlds',
    'exam_hall.world'
)
````

Change `exam_hall.world` to your desired world, for example:

```python
default_world = os.path.join(
    get_package_share_directory(package_name),
    'worlds',
    'turtlebot3_world.world'
)
```

Save the file.

---

# 🚀 2. Run Simulation

### Terminal 1 — Launch Gazebo

```bash
ros2 launch protobot launch_sim_slam.launch.py
```

This will start:

* Gazebo simulation with the chosen world
* Protobot robot model in the simulated environment

### Terminal 2 — Launch Control + SLAM

```bash
ros2 launch protobot launch_sim_control.launch.py \
use_slam_option:=online_async_slam
```

This will start:

* `slam_toolbox` (online_async mode)
* Navigation2 stack
* Control interfaces

---

# 🕹 3. Move the Robot

Just like on the real robot, you can control Protobot using:

* **Joystick** (if configured)
  Tutorial: [Joystick Setup & Operation](https://youtu.be/F5XlNiCKbrY)
* **Keyboard Teleoperation**

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
--ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped
```

Drive the robot around the world to create the SLAM map.

---

# 💾 4. Save Map in Simulation

⚠ **Important:**
Before saving, ensure the robot is back at **home_position**.

### Option A — Save Using RViz2

Follow this tutorial: [RViz2 Map Saving](https://youtu.be/ZaiA3hWaRzE?si=wmxskfIKMullYxp_)

### Option B — Save Using Terminal (CLI)

```bash
ros2 run nav2_map_server map_saver_cli \
-f ~/ros2_ws/src/protobot/maps/my_map
```

```bash
ros2 service call /slam_toolbox/serialize_map \
slam_toolbox/srv/SerializePoseGraph \
"{filename: '/home/ammar/ros2_ws/src/protobot/maps/my_map'}"
```

Replace `my_map` with the filename you prefer.

---

# 📍 5. Localization in Simulation

After mapping is complete, you can run localization using the saved map.

### Step 1 — Edit Map File Path

Open:

```
protobot/config/mapper_params_localization.yaml
```

Change:

```yaml
map_file_name: ./src/protobot/maps/exam_hall
```

To your saved map:

```yaml
map_file_name: ./src/protobot/maps/my_map
```

### Step 2 — Launch Robot & Localization

Terminal 1 — Launch Gazebo + Robot:

```bash
ros2 launch protobot launch_sim_slam.launch.py
```

Terminal 2 — Launch Control + Localization:

```bash
ros2 launch protobot lauch_sim_control.launch.py \
use_slam_option:=mapper_params_localization
```

---

# ⚠ Simulation Notes

* Robot MUST start at **home_position** before localization
* Check that map files (.pgm + .yaml) exist in `protobot/maps`
* You can save named waypoints (tables) and navigate to them just like on the real robot
* Simulation is useful for debugging Nav2 behavior, tuning SLAM, and testing maps

---

# 📌 Summary

1. Choose Gazebo world in `launch_sim_slam.launch.py`
2. Launch Gazebo + Robot
3. Launch Control + SLAM
4. Drive robot to create map
5. Return to home_position
6. Save map (grid + pose graph)
7. Edit map path for localization
8. Launch localization mode and navigate

---

**Simulation helps understand Nav2 and SLAM without requiring the physical robot, ideal for testing and debugging.**



