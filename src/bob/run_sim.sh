#!/bin/bash

rm -r build install log

colcon build --symlink-install

source install/setup.bash
