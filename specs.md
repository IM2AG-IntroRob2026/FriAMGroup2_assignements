# Security Patrol Robot Specifications

## Concept

The robot acts as an autonomous security guard. It patrols a square perimeter,
detects intrusions (obstacles), triggers an alarm, avoids the obstacle, then
resumes patrol. After completing all laps it returns to its dock.

---

## Robot Platform

- iRobot Create3 (ROS 2 Humble)
- Namespace: `/Robot2`

---

## States (FSM)

```
DOCKED
  → UNDOCKING
  → PATROLLING
      → INTRUDER_ALERT   (on obstacle detected)
          → AVOIDING
              → PATROLLING  (resume)
  → RETURNING
  → DOCKING
  → MANUAL_OVERRIDE      (reachable from any state)
```

### State Descriptions

| State | Entry Condition | Behavior | Exit Condition |
|---|---|---|---|
| DOCKED | Initial state | Idle | Start command received |
| UNDOCKING | Start command | Call `/undock` action | Action succeeds |
| PATROLLING | Undock succeeded OR avoidance done | Run `DrawSquare` action (1 lap), repeat N times | N laps done → RETURNING; obstacle → INTRUDER_ALERT |
| INTRUDER_ALERT | Hazard detected during patrol | Cancel DrawSquare, play siren 3 s via `/audio_sequence` | Siren ends |
| AVOIDING | Siren ends | Back up (0.2 m), rotate 90°, move forward 0.3 m to clear obstacle | Maneuver done |
| RETURNING | N laps done | Drive back toward dock using odometry (reverse patrol path or direct) | Near dock |
| DOCKING | Near dock | Call `/dock` action | Action succeeds |
| MANUAL_OVERRIDE | `/patrol/stop` service called | Publish zero Twist, cancel all actions, freeze FSM | `/patrol/start` service called |

---

## Parameters

| Parameter | Default | Description |
|---|---|---|
| `num_laps` | 3 | Number of square laps before returning to dock |
| `side_length` | 2.0 m | Side length of the patrol square |
| `patrol_speed` | 0.3 m/s | Linear speed during patrol |
| `siren_duration` | 3.0 s | Duration of the alarm sound |

---

## Interfaces

### Actions Used (client side)

| Action | Type | Purpose |
|---|---|---|
| `/undock` | `irobot_create_msgs/action/Undock` | Undock from station |
| `/dock` | `irobot_create_msgs/action/Dock` | Dock to station |
| `/draw_square` | `turtle_square_interfaces/action/DrawSquare` | Execute one patrol lap |
| `/Robot2/audio_sequence` | `irobot_create_msgs/action/AudioNoteSequence` | Play siren |

### Services Exposed (server side)

| Service | Type | Purpose |
|---|---|---|
| `/patrol/start` | `std_srvs/srv/Trigger` | Start the patrol sequence |
| `/patrol/stop` | `std_srvs/srv/Trigger` | Stop and enter MANUAL_OVERRIDE |

### Topics Subscribed

| Topic | Type | Purpose |
|---|---|---|
| `/Robot2/hazard_detection` | `irobot_create_msgs/msg/HazardDetectionVector` | Detect obstacles / bumps |

---

## Siren Specification

Alternate two tones to approximate a police siren:
- High tone: 880 Hz, 0.4 s
- Low tone: 659 Hz, 0.4 s
- Repeat ~4 times to fill 3 s

---

## Avoidance Maneuver

1. Stop immediately (zero Twist)
2. Back up 0.2 m at 0.15 m/s
3. Rotate 90° (direction: away from bump sensor that triggered)
4. Move forward 0.3 m at 0.2 m/s
5. Resume patrol from current lap (restart current side)

---

## Testability Requirements

Each feature must be testable independently:

| Test | Environment | What to validate |
|---|---|---|
| T1 — FSM state transitions | turtlesim or unit test | All state changes fire correctly |
| T2 — DrawSquare lap counting | Gazebo or robot | N laps then RETURNING |
| T3 — Siren playback | Robot only | Correct tones, correct duration |
| T4 — Avoidance maneuver | Gazebo or robot | Robot clears obstacle and resumes |
| T5 — Manual override | Any | Stop service freezes robot immediately |
| T6 — Full integration | Robot | Complete undock → patrol → alert → avoid → dock cycle |

---

