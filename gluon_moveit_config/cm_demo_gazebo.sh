#!/bin/bash 

roslaunch gluon_moveit_config cm_gazebo.launch &
sleep 10
echo "launch cm_gazebo successfully"

roslaunch gluon_moveit_config demo.launch &
sleep 10
echo "launch demo successfully"

wait
exit 0
