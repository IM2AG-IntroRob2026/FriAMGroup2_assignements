# Si le dock ou undock ne marche pas, essaie ceci :

```bash
ros2 interface list | grep irobot
```

If nothing appear do this:

```bash
apt update && apt install -y ros-humble-irobot-create-msgs
```

THEN source and retry

```bash
source /opt/ros/humble/setup.bash
ros2 action send_goal /Robot2/undock irobot_create_msgs/action/Undock "{}"

```

Another error:

The `sequence size exceeds remaining buffer` error is a CDR deserialization failure — it means your node is receiving the /Robot2/odom message but can't parse it. This is almost always a RMW (middleware) mismatch: the Create 3 uses CycloneDDS by default, but ROS 2 Humble containers default to FastDDS.

Check what you're currently using:

```bash
echo $RMW_IMPLEMENTATION
```
Then switch to CycloneDDS to match the robot:
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 run robot_square_controller square_action_server
```

If still:
```bash
apt update && apt install -y ros-humble-rmw-cyclonedds-cpp
```

Then find if it's installed:

```bash
find /opt/ros/humble -name "librmw_cyclonedds_cpp.so" 2>/dev/null
```

If nothing shows up :

```bash
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add -
apt update
apt install -y ros-humble-rmw-cyclonedds-cpp
```