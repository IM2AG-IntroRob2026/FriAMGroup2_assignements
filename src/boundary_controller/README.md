# boundary_controller

Autonomous turtle boundary tracer for ROS 2 Humble / turtlesim, with
interruptible manual teleop control.

## Quick start

```bash
# One-time per container session (xterm is required for the teleop window)
apt-get install -y xterm

ros2 launch boundary_controller draw_boundaries.launch.py
```

## Behavior

1. The turtle starts moving in a straight line from its spawn point (pen up).
2. On reaching the domain edge, the turtle drives to the **nearest corner**
   (pen still up), then puts the **pen down** and traces the full rectangular
   boundary corner by corner.
3. Once all four corners are visited and the loop is closed, the pen is lifted
   and the turtle returns to its **initial position**.
4. **Press SPACE** at any time to toggle between autonomous and manual mode.
5. In manual mode use **arrow keys** to drive freely.
6. **Press SPACE again** to resume autonomous drawing from the exact state
   that was interrupted.

## FSM

```
INIT ──(pose received)──► EXPLORING ──(near wall)──► CORNERING
                                                          │
                                               (first corner reached)
                                                          ▼
                                                      FOLLOWING ──(all corners)──► RETURNING ──► DONE
                                                          ▲                             ▲
                                ◄──── MANUAL ────────────┘─────────────────────────────┘
                                      (SPACE toggles MANUAL ↔ any autonomous state)
```

## Launch options

All arguments are optional.

| Argument      | Default | Description                    |
|---------------|---------|--------------------------------|
| `pen_color_r` | `255`   | Pen red channel   (0 – 255)    |
| `pen_color_g` | `0`     | Pen green channel (0 – 255)    |
| `pen_color_b` | `0`     | Pen blue channel  (0 – 255)    |
| `pen_width`   | `3`     | Pen width in pixels            |

Example – blue pen, width 5:

```bash
ros2 launch boundary_controller draw_boundaries.launch.py \
    pen_color_r:=0 pen_color_g:=0 pen_color_b:=255 pen_width:=5
```

## Architecture

Three nodes are started by the launch file:

| Node              | Role                                                         |
|-------------------|--------------------------------------------------------------|
| `turtlesim`       | Simulator                                                    |
| `boundary_fsm`    | Autonomous FSM controller; subscribes `/manual_mode` (Bool) |
| `keyboard_teleop` | Reads keyboard; publishes `/manual_mode` and `/turtle1/cmd_vel` |

The `keyboard_teleop` node opens in a dedicated **xterm** window so it has its
own TTY for raw keyboard reading.

Topic graph:

```
keyboard_teleop ──/manual_mode──────────────► boundary_fsm
keyboard_teleop ──/turtle1/cmd_vel──┐
boundary_fsm    ──/turtle1/cmd_vel──┴──► turtlesim
turtlesim       ──/turtle1/pose─────────► boundary_fsm
```

## Domain

The boundary is drawn between corners `(1, 1)` and `(10, 10)` in turtlesim
units.  Wall detection triggers at 0.6 units from each edge.
