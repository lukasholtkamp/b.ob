#!/bin/bash

rm -r -f build install log

colcon build --symlink-install

source install/setup.bash