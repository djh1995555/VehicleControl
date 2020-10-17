#!/bin/bash 

roslaunch control run_with_carsim.launch &
echo "launch world successfully"
sleep 1

roslaunch control trajectory_plot.launch &
echo "launch trajectory_plot successfully"

sleep 10

roslaunch plan publish_mat_trajectory.launch &
echo "publish trajectory successfully"


wait
exit 0
