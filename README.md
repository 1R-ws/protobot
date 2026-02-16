# PROTOBOT – Autonomous Differential Drive Robot (ROS 2 Jazzy)

This repository contains the main robot stack for **Protobot**, a differential drive mobile robot developed using ROS 2 Jazzy.

---

## Features

- ros2_control
- SLAM (slam_toolbox)
- Localization (Nav2)
- Waypoint / table saving system
- Joystick & Keyboard teleoperation
- Autonomous navigation to saved locations

Tested on:
- Ubuntu 24.04
- ROS 2 Jazzy
- Raspberry Pi 4 / 5

---

## Simulation Setup

For setup steps and to run the **simulation (Gazebo)**, please refer to the [Simulation Instructions](simulation.md#introduction).  



---

# 🗺️ 1. MAPPING MODE (SLAM)

Mapping is performed using `slam_toolbox` with `online_async_slam`.

---

## Step 1 — Launch Robot

Open Terminal 1:

```bash
ros2 launch protobot lauch_robot.lauch.py
```

---

## Step 2 — Launch Control + SLAM

Open Terminal 2:

```bash
ros2 launch protobot lauch_sim_control.launch.py use_slam_option:=online_async_slam use_sim_time:=false
```

This will start:
- `slam_toolbox` (online_async mode)
- Navigation2 stack
- Control interfaces

---

## Step 3 — Move the Robot

### Option A — Joystick
(if configured)  
Tutorial: [Joystick Setup & Operation](https://youtu.be/F5XlNiCKbrY)

### Option B — Keyboard Teleoperation

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
--ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped
```

Drive the robot around the entire area until mapping is complete.

---

# 💾 2. SAVE MAP AFTER MAPPING COMPLETE

⚠ **Important:**  
Before saving the map, ensure the robot is back at the **home_position** (starting position).  
Localization will only work properly if the robot starts at this same position.

---

## Option A — Save Using RViz2

Follow this tutorial: [RViz2 Map Saving](https://youtu.be/ZaiA3hWaRzE)

---

## Option B — Save Using Terminal (CLI)

### Save Occupancy Grid Map

```bash
ros2 run nav2_map_server map_saver_cli \
-f ~/ros2_ws/src/protobot/maps/my_map
```

### Save SLAM Pose Graph

```bash
ros2 service call /slam_toolbox/serialize_map \
slam_toolbox/srv/SerializePoseGraph \
"{filename: '/home/ammar/ros2_ws/src/protobot/maps/my_map'}"
```

This saves both:
- Occupancy grid (.yaml + .pgm)
- SLAM pose graph data

---

# 🪑 3. SAVE NAMED LOCATIONS (TABLES / WAYPOINTS)

During mapping or localization, save goal positions using:

```bash
ros2 topic pub /save_pose std_msgs/msg/String "{data: 'table1'}" --once
```

To save additional tables, replace `table1` with `table2`, `table3`, etc.

All saved poses are stored in:

```
protobot/config/saved_poses.yaml
```

---

## Test Navigation to Saved Pose

```bash
ros2 topic pub /go_to_pose std_msgs/msg/String "{data: 'table1'}" --once
```

Robot should navigate to the saved location.

---


# 📍 4. LOCALIZATION MODE (USE SAVED MAP)

After mapping is complete, switch to localization mode.

---

## Step 1 — Edit Map File Before Localization

Open:

```

protobot/config/mapper_params_localization.yaml

````

Change:

```yaml
map_file_name: ./src/protobot/maps/exam_hall
````

To:

```yaml
map_file_name: ./src/protobot/maps/my_map
```

Replace `my_map` with your saved map filename.

---

## Step 2 — Launch Robot

Terminal 1:

```bash
ros2 launch protobot lauch_robot.lauch.py
```

---

## Step 3 — Launch Localization

Terminal 2:

```bash
ros2 launch protobot lauch_sim_control.launch.py use_slam_option:=mapper_params_localization use_sim_time:=false
```

---


# ⚠ IMPORTANT REMINDERS

- Robot MUST start at **home_position** before localization
- Ensure correct map path is configured
- Ensure map files (.pgm + .yaml) exist inside `protobot/maps`
- If localization fails, check robot initial pose in RViz

---

# 🔄 6. QUICK COMMAND SUMMARY

### Mapping Mode

```bash
ros2 launch protobot lauch_robot.lauch.py
ros2 launch protobot lauch_sim_control.launch.py use_slam_option:=online_async_slam use_sim_time:=false
```

### Localization Mode

```bash
ros2 launch protobot lauch_robot.lauch.py
ros2 launch protobot lauch_sim_control.launch.py use_slam_option:=mapper_params_localization use_sim_time:=false
```

### Save Table Pose

```bash
ros2 topic pub /save_pose std_msgs/msg/String "{data: 'table1'}" --once
```

### Go to Saved Pose

```bash
ros2 topic pub /go_to_pose std_msgs/msg/String "{data: 'table1'}" --once
```

---

# 🧭 7. FULL WORKFLOW SUMMARY

**Mapping:**
1. Launch robot  
2. Launch SLAM  
3. Drive robot around environment  
4. Return to home_position  
5. Save map (grid + pose graph)  
6. Save waypoint poses (tables)

**Localization:**
1. Start at home_position  
2. Launch robot  
3. Launch localization mode
4. Navigate to saved tables

---

# 📌 8. FUTURE IMPROVEMENTS

- Improved Autonomous table delivery queue system  
- Add weigh sensor
- Improved Nav2 tuning parameters  

---

**Developed as part of Autonomous Carrier System using ROS 2 Jazzy**
