# FriAMGroup2_assignements

# Assignement 1

# turtle_square_controller

ROS2 action server that moves the turtlesim turtle in a square pattern.

## Prerequisites

Make sure turtlesim is running in a separate terminal:
```bash
ros2 run turtlesim turtlesim_node
```

## Reset the turtle before each test

**Always reset the turtle before running a new test**, otherwise the turtle starts from wherever it stopped last, which can cause it to hit a wall mid-run and loop forever.

```bash
ros2 service call /reset std_srvs/srv/Empty
```

This teleports the turtle back to the center (5.5, 5.5) facing right and clears the drawing.

## Run the action server

In a separate terminal:
```bash
ros2 run turtle_square_controller square_action_server
```

## Send a goal

### Custom parameters
```bash
ros2 action send_goal /draw_square turtle_square_interfaces/action/DrawSquare "{side_length: 2.0, speed: 1.0}" --feedback
```

## Cancel a running goal

Press `Ctrl+C` in the terminal where you sent the goal. The server will stop the turtle immediately.

## Full test sequence (copy-paste)

```bash
# Terminal 1 — turtlesim
ros2 run turtlesim turtlesim_node

# Terminal 2 — action server
ros2 run turtle_square_controller square_action_server

# Terminal 3 — reset then send goal
ros2 service call /reset std_srvs/srv/Empty
ros2 action send_goal /draw_square turtle_square_interfaces/action/DrawSquare "{side_length: 2.0, speed: 1.0}" --feedback
```
