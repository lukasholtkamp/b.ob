#!/bin/bash

rm -r build install log

colcon build --symlink-install

source install/setup.bash

ros2 launch bob launch_sim.launch.py 

