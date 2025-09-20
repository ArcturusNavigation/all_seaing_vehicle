#!/bin/bash
source /opt/ros/humble/setup.bash
ros2 run mavros mav sys rate --all 30 # need to run after launching the mavros node