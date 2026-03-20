# robot_square_controller

ROS 2 action server that moves an **iRobot Create 3** in a square pattern from its current position.

---

## Prerequisites

- ROS 2 **Humble** installed (or use the provided Docker container)
- `turtle_square_interfaces` package built in the same workspace
- iRobot Create 3 robot connected and publishing on `/Robot2/*` topics
- **CycloneDDS** — required to match the Create 3's default middleware

Cyclone DDS is one of the two ROS 2 middleware implementations supported by the iRobot Create 3, alongside FastDDS. 
---

## Environment Setup

### 1. Install CycloneDDS

```bash
sudo apt update && sudo apt install -y ros-humble-rmw-cyclonedds-cpp
```

### 2. Set CycloneDDS as default middleware

Add to your `~/.bashrc` (do this once):

```bash
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
source ~/.bashrc
```

Or export it manually in every new terminal before running any `ros2` command:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

> **Why?** The Create 3 uses CycloneDDS by default. Using a different RMW (e.g. FastDDS)
> causes a deserialization mismatch on `/Robot2/odom` → `sequence size exceeds remaining buffer` error.

---

## Build

```bash
cd ~/ros2_ws

# Build the interface package first
colcon build --packages-select turtle_square_interfaces
source install/setup.bash

# Then build the controller
colcon build --packages-select robot_square_controller
source install/setup.bash
```

---

## Verify Robot Connectivity

Before running the server, confirm the robot's topics are visible:

```bash
ros2 topic list | grep Robot2
```

You should see `/Robot2/odom` and `/Robot2/cmd_vel` in the list.

---

## Run

### Terminal 1 — Start the action server

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source ~/ros2_ws/install/setup.bash
ros2 run robot_square_controller square_action_server
```

You should see:
```
[INFO] [square_action_server]: Square Action Server started!
```
followed by continuous position logs from odometry.

### Terminal 2 — Send a goal

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source ~/ros2_ws/install/setup.bash
ros2 action send_goal /draw_square turtle_square_interfaces/action/DrawSquare \
  "{side_length: 1.0, speed: 0.3}" --feedback
```

- `side_length`: side of the square in **meters** (start with `1.0`)
- `speed`: linear velocity in **m/s** (start with `0.3` for safety)

### Cancel mid-run

Press `Ctrl+C` in the goal terminal. The server will stop the robot immediately.

---

## Topics Used

| Topic | Type | Direction |
|-------|------|-----------|
| `/Robot2/cmd_vel` | `geometry_msgs/Twist` | Published (commands) |
| `/Robot2/odom` | `nav_msgs/Odometry` | Subscribed (position) |

---

## Debugging

```bash
# Check action server is up
ros2 action list           # should show /draw_square
ros2 node list             # should show /square_action_server

# Monitor what the robot receives
ros2 topic echo /Robot2/cmd_vel

# Monitor odometry
ros2 topic echo /Robot2/odom --no-arr
```

---

## Common Issues

### `sequence size exceeds remaining buffer`
RMW mismatch. Make sure `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` is exported in **every** terminal.

### `The passed action type is invalid`
`irobot_create_msgs` not installed or not sourced. Install with:
```bash
sudo apt install ros-humble-irobot-create-msgs
```

### Robot doesn't move / `current_pose is None`
The server hasn't received an odometry message yet. Check `/Robot2/odom` is publishing:
```bash
ros2 topic hz /Robot2/odom
```
