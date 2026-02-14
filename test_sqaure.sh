#!/bin/bash
cd ~/ros2_ws
source install/setup.bash

ros2 action send_goal /draw_square turtle_square_interfaces/action/DrawSquare \
  "{side_length: 2.0, speed: 1.0}" --feedback