#!/bin/bash 

roslaunch control run_in_simulation_world.launch &
echo "launch world successfully"
sleep 1

roslaunch control trajectory_plot.launch &
echo "launch trajectory_plot successfully"
sleep 1

roslaunch plan publish_ontime_trajectory.launch &
echo "publish local trajectory successfully"
sleep 10

roslaunch plan publish_recorded_trajectory.launch &
echo "publish trajectory successfully"

wait
exit 0
