#!/bin/bash
source /opt/ros/noetic/setup.bash
source /home/ubuntu/work/ros1_ws1/devel/setup.bash
roslaunch namiashi_config bringup.launch

# sudo systemctl daemon-reload
# sudo systemctl enable namiashi_bringup.service
# sudo systemctl start namiashi_bringup.service
