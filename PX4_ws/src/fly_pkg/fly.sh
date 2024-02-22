#!/bin/bash
sudo chmod 777 /dev/ttyTHS0
sudo chmod 777 /dev/ttyACM0
gnome-terminal -- bash -c "roscore; exec bash"
gnome-terminal -- bash -c "roslaunch fly_pkg all_node.launch; exec bash"

