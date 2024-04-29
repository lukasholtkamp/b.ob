#!/bin/bash

rm -r build

mkdir build

cd build

cmake ..

make

sudo ./Wheel_test