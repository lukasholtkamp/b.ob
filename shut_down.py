#!/usr/bin/env python
import os
import signal 

nodes = os.popen("ros2 node list").readlines()
for i in range(len(nodes)):
    nodes[i] = nodes[i].replace("\n","")
    nodes[i] = nodes[i][1:]

for node in nodes:
    os.system("ros2 lifecycle set "+node+" shutdown")
    os.system("killall "+node)

os.system("kill $bob")
