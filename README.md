# Table of Contents
* [Introduction](#introduction)
* [Simulation Platform](#simulation-platform)
* [WorkFlow](#workflow)
* [Planning Module](#planning-module)
* [Control Module](#control-module)
    * [Run Example](#run-example)
    * [Choose Test Trajectory](#choose-test-trajectory)
    * [Record Trajectory](#record-trajectory)
    * [Choose Controller and Adjust Their Parameters](#choose-controller-and-adjust-their-parameters)
* [Controllers](#controllers)
    * [Controller list](#controllers)
    * [Result PLot](#result-plot)
    * [Other Plot](#other-plot)
* [Maintainers](#maintainers)

# Introduction
This repository includes the **planning algorithm based on frenet optimal polynomial** and the commonly used **control algorithms** for vehicle motion like **PID, LQR, MPC** 
and other algorithms based on geometry model like **Stanley, Pure Pursuit** and **Rear Wheel Feedback**.  

**Two scenarios** are tested here:
* forward driving in curved road with differernt velocity and radius
* reverse driving in curved road

The velocity of test trajectory is in the range from 10m/s to 20m/s, a part of result plot is post in below, all the plots can be found in the folder named **result_plot**.

# Simulation Platform
The simulation is conducted in GAZEBO, the vehicle model and the senario are obtained from the Demo of Prius in ROS/GAZEBO(https://github.com/osrf/car_demo.git). 

![Prius_image](https://github.com/djh1995555/VehicleControl/blob/master/image_folder/Prius_image.png)
![Prius_image_2](https://github.com/djh1995555/VehicleControl/blob/master/image_folder/Prius_image_1.jpg)
![senario](https://github.com/djh1995555/VehicleControl/blob/master/image_folder/senario_map.png)


# WorkFlow
![work_flow](https://github.com/djh1995555/VehicleControl/blob/master/image_folder/workflow.png)

# Planning Module
## result
![planning_result_git](https://github.com/djh1995555/VehicleControl/blob/master/result_plot/planning_result.gif)

![planning_result](https://github.com/djh1995555/VehicleControl/blob/master/result_plot/planning_result.png)

## Planning Strategy
![planning_strategy](https://github.com/djh1995555/VehicleControl/blob/master/image_folder/planning_strategy.png)

# Control Module
It is recommended to switch off the planning module to observe the performance of control module better by modifying the Activate_planning_function in src/plan/conf/planning_conf.txt to 0;
## Run Example
```
~$ cd catkin_ws/src/multi_launch/
# forward running in an empty world with smooth, artificial trjaectory
~/catkin_ws/src/multi_launch/$ ./forward.sh
# reverse running in an empty world with smooth, artificial trjaectory
~/catkin_ws/src/multi_launch/$ ./reverse.sh
# running in an simulated environment with recored trajectory
~/catkin_ws/src/multi_launch/$ ./simulated_world.sh
# running in an simulated environment with ramp
~/catkin_ws/src/multi_launch/$ ./run_in_ramp.sh
```
![working process](https://github.com/djh1995555/VehicleControl/blob/master/image_folder/working_process.gif)

## Choose Test Trajectory
```
~$ cd catkin_ws/src/plan/launch/
# change the trajectory in empty world
~/catkin_ws/src/plan/launch$ gedit publish_mat_trajectory.launch
```
Modify the value of test trajectory file path to change the test trajectories. The avaliable test trajectories can be found in the folder named **"test_trajectories"**.  

In this folder, the trajectory file starting with "mat" means that it is a artificial trajectory, and the recorded trajectories are recorded in the GAZEBO senario, which are not so smooth, but closer to the real situation. 

The last num of trajectory file name means the velocity of vehicle running in curve, **only for the "mat trajectories"**, the velocity of recorded trajectories are not constant.

## Record Trajectory
Recording a trajectory is a good method to generate an availble trajectory in the city environment.  

1. find the plugin code  
```
~$ cd catkin_ws/src/car_demo/plugins/
~/catkin_ws/src//car_demo/plugins$ gedit PriusHybridPlugin.cc
```

2. Switch on the recording function  
correct the **record_enabled** into **True**  

3. roslaunch 
```
~$ roslaunch car_demo record_trajectory.launch
```

4. record trajectory  
"W","A","D" are the control keys  
"S" is "reverse"  
"E" is "brake"

5. Get recorded trajectory  
The recorded trajectory is stored in the folder (catkin_ws/src/car_demo/recorded_trajectories), you can trim it and move it to the **test_trajectories** folder.


## Choose Controller and Adjust Their Parameters
```
~$ cd catkin_ws/src/control/conf/
~/catkin_ws/src/control/conf$ gedit control_conf.txt
```
This file includes the physical parameters of vehicle model (can not be modified) and all the parameters (can be adjusted) required by the controllers and other important objects necessary during the operation of control module.  

The controller can be classified into the distributed controllers (control longitudinal and lateral motion respectively) and centralized controllers (control the vehicle motion coupled).  

To active distributed controller, the **CONTROLLER_NUM** must be 2, the next step is to set the **LONGITUDINAL_CONTROLLER** and **LATERAL_CONTROLLER**, the options are listed below them. 

To active centralized controller, the **CONTROLLER_NUM** must be 1, the next step is to set the **CENTRALIZED_CONTROLLER**.  

## Choose the Plot shown
Since the FPS is low if we plot more than one figures simultaneously, so now there is only one plot can be shown during the simulation (after simulation, we can get all the plots, see [Other Plot](#other-plot)), the steps to choose the plot shown is:
```
~$ cd catkin_ws/src/control/scripts/
~/catkin_ws/src/control/scripts$ gedit TrajectoryPlotter.py
```
change the **TARGET_PLOT** to choose which plot you want to see during simulation, now there are three options
* trajectory comparison
* velocity comparison with station
* heading angle comarison with station


# Controllers
* Longitudinal Controllers
    * [PID controller](#result_plot)
* Lateral Controllers
    * [Pure Pursuit Controller](#pure-pursuit-controller)
    * [Stanley Controller](#stanley-ontroller)
    * [Rear Wheel Feedback Controller](#rear-wheel-feedback-controller)
    * [LQR](#lqr)
    * [LQR with prediction](#lqr-with-prediction)
    * [MPC](#mpc)
* Centralized Controllers
    * MPC (working in process)
## Result PLot(reverse)
![v=10m/s](https://github.com/djh1995555/VehicleControl/blob/master/result_plot/gif/pure_pursuit_reverse_10.gif) 
## Result PLot(forward)
### Tracking Process (gif)
![path tracking](https://github.com/djh1995555/VehicleControl/blob/master/result_plot/path_tracking_result.gif)
![velocity tracking](https://github.com/djh1995555/VehicleControl/blob/master/result_plot/velocity_tracking_result.gif)
![heading tracking](https://github.com/djh1995555/VehicleControl/blob/master/result_plot/heading_tracking_result.gif)
### Pure Pursuit Controller  
**Pure Pursuit Controller V=10m/s**
![Trajectory Tracking](https://github.com/djh1995555/VehicleControl/blob/master/control/plot_py/Pure_Pursuit_Controller/Pure_Pursuit_Controller_10_Path.png)
![Velocity Tracking](https://github.com/djh1995555/VehicleControl/blob/master/control/plot_py/Pure_Pursuit_Controller/Pure_Pursuit_Controller_10_Velocity.png)
![Heading Tracking](https://github.com/djh1995555/VehicleControl/blob/master/control/plot_py/Pure_Pursuit_Controller/Pure_Pursuit_Controller_10_Heading.png)
![Lateral Distance Error](https://github.com/djh1995555/VehicleControl/blob/master/control/plot_py/Pure_Pursuit_Controller/Pure_Pursuit_Controller_10_Lateral_Distance_Error.png)
![Velocity Error](https://github.com/djh1995555/VehicleControl/blob/master/control/plot_py/Pure_Pursuit_Controller/Pure_Pursuit_Controller_10_Velocity_Error.png)
![Heading Error](https://github.com/djh1995555/VehicleControl/blob/master/control/plot_py/Pure_Pursuit_Controller/Pure_Pursuit_Controller_10_Heading_Error.png)
**Pure Pursuit Controller V=15m/s**
![Trajectory Tracking](https://github.com/djh1995555/VehicleControl/blob/master/control/plot_py/Pure_Pursuit_Controller/Pure_Pursuit_Controller_15_Path.png)
![Velocity Tracking](https://github.com/djh1995555/VehicleControl/blob/master/control/plot_py/Pure_Pursuit_Controller/Pure_Pursuit_Controller_15_Velocity.png)
![Heading Tracking](https://github.com/djh1995555/VehicleControl/blob/master/control/plot_py/Pure_Pursuit_Controller/Pure_Pursuit_Controller_15_Heading.png)

### Stanley Controller  
**Stanley Controller  V=10m/s**
![Trajectory Tracking](https://github.com/djh1995555/VehicleControl/blob/master/control/plot_py/Stanley_Controller/Stanley_Controller_10_Path.png)
![Velocity Tracking](https://github.com/djh1995555/VehicleControl/blob/master/control/plot_py/Stanley_Controller/Stanley_Controller_10_Velocity.png)
![Heading Tracking](https://github.com/djh1995555/VehicleControl/blob/master/control/plot_py/Stanley_Controller/Stanley_Controller_10_Heading.png)
![Lateral Distance Error](https://github.com/djh1995555/VehicleControl/blob/master/control/plot_py/Stanley_Controller/Stanley_Controller_10_Lateral_Distance_Error.png)
![Velocity Tracking](https://github.com/djh1995555/VehicleControl/blob/master/control/plot_py/Stanley_Controller/Stanley_Controller_10_Velocity_Error.png)
![Heading Error](https://github.com/djh1995555/VehicleControl/blob/master/control/plot_py/Stanley_Controller/Stanley_Controller_10_Heading_Error.png)
**Stanley Controller  V=15m/s**
![Trajectory Tracking](https://github.com/djh1995555/VehicleControl/blob/master/control/plot_py/Stanley_Controller/Stanley_Controller_15_Path.png)
![Velocity Tracking](https://github.com/djh1995555/VehicleControl/blob/master/control/plot_py/Stanley_Controller/Stanley_Controller_15_Velocity.png)
![Heading Tracking](https://github.com/djh1995555/VehicleControl/blob/master/control/plot_py/Stanley_Controller/Stanley_Controller_15_Heading.png)

### Rear Wheel Feedback Controller  

**Rear Wheel Feedback Controller V=10m/s**

![Trajectory Tracking](https://github.com/djh1995555/VehicleControl/blob/master/control/plot_py/Rear_Wheel_Feedback_Controller/Rear_Wheel_Feedback_Controller_10_Path.png)
![Velocity Tracking](https://github.com/djh1995555/VehicleControl/blob/master/control/plot_py/Rear_Wheel_Feedback_Controller/Rear_Wheel_Feedback_Controller_10_Velocity.png)
![Heading Tracking](https://github.com/djh1995555/VehicleControl/blob/master/control/plot_py/Rear_Wheel_Feedback_Controller/Rear_Wheel_Feedback_Controller_10_Heading.png)
![Lateral Distance Error](https://github.com/djh1995555/VehicleControl/blob/master/control/plot_py/Rear_Wheel_Feedback_Controller/Rear_Wheel_Feedback_Controller_10_Lateral_Distance_Error.png)
![Velocity Error](https://github.com/djh1995555/VehicleControl/blob/master/control/plot_py/Rear_Wheel_Feedback_Controller/Rear_Wheel_Feedback_Controller_10_Velocity_Error.png)
![Heading Error](https://github.com/djh1995555/VehicleControl/blob/master/control/plot_py/Rear_Wheel_Feedback_Controller/Rear_Wheel_Feedback_Controller_10_Heading_Error.png)
**Rear Wheel Feedback Controller   V=15m/s**
![Trajectory Tracking](https://github.com/djh1995555/VehicleControl/blob/master/control/plot_py/Rear_Wheel_Feedback_Controller/Rear_Wheel_Feedback_Controller_15_Path.png)
![Velocity Tracking](https://github.com/djh1995555/VehicleControl/blob/master/control/plot_py/Rear_Wheel_Feedback_Controller/Rear_Wheel_Feedback_Controller_15_Velocity.png)
![Heading Tracking](https://github.com/djh1995555/VehicleControl/blob/master/control/plot_py/Rear_Wheel_Feedback_Controller/Rear_Wheel_Feedback_Controller_15_Heading.png)

### LQR  

**LQR Controller V=10m/s**

![Trajectory Tracking](https://github.com/djh1995555/VehicleControl/blob/master/control/plot_py/OLQR_Controller/OLQR_Controller_10_Path.png)
![Velocity Tracking](https://github.com/djh1995555/VehicleControl/blob/master/control/plot_py/OLQR_Controller/OLQR_Controller_10_Velocity.png)
![Heading Tracking](https://github.com/djh1995555/VehicleControl/blob/master/control/plot_py/OLQR_Controller/OLQR_Controller_10_Heading.png)
![Lateral Distance Error](https://github.com/djh1995555/VehicleControl/blob/master/control/plot_py/OLQR_Controller/OLQR_ControllerR_10_Lateral_Distance_Error.png)
![Velocity Error](https://github.com/djh1995555/VehicleControl/blob/master/control/plot_py/OLQR_Controller/OLQR_Controller_10_Velocity_Error.png)
![Heading Erro](https://github.com/djh1995555/VehicleControl/blob/master/control/plot_py/OLQR_Controller/OLQR_Controller_10_Heading_Error.png)
**LQR Controller V=15m/s**
![Trajectory Tracking](https://github.com/djh1995555/VehicleControl/blob/master/control/plot_py/OLQR_Controller/OLQR_Controller_15_Path.png)
![Velocity Tracking](https://github.com/djh1995555/VehicleControl/blob/master/control/plot_py/OLQR_Controller/OLQR_Controller_15_Velocity.png)
![Heading Tracking](https://github.com/djh1995555/VehicleControl/blob/master/control/plot_py/OLQR_Controller/OLQR_Controller_15_Heading.png)

### LQR with prediction  

**LQR Controller with prediction V=10m/s**

![Trajectory Tracking](https://github.com/djh1995555/VehicleControl/blob/master/control/plot_py/LQR_Controller/LQR_Controller_10_Path.png)
![Velocity Tracking](https://github.com/djh1995555/VehicleControl/blob/master/control/plot_py/LQR_Controller/LQR_Controller_10_Velocity.png)
![Heading Tracking](https://github.com/djh1995555/VehicleControl/blob/master/control/plot_py/LQR_Controller/LQR_Controller_10_Heading.png)
![Lateral Distance Error](https://github.com/djh1995555/VehicleControl/blob/master/control/plot_py/LQR_Controller/LQR_Controller_10_Lateral_Distance_Error.png)
![Velocity Error](https://github.com/djh1995555/VehicleControl/blob/master/control/plot_py/LQR_Controller/LQR_Controller_10_Velocity_Error.png)
![Heading Error](https://github.com/djh1995555/VehicleControl/blob/master/control/plot_py/LQR_Controller/LQR_Controller_10_Heading_Error.png)
**LQR Controller with prediction V=15m/s**
![Trajectory Tracking](https://github.com/djh1995555/VehicleControl/blob/master/control/plot_py/LQR_Controller/LQR_Controller_15_Path.png)
![Velocity Tracking](https://github.com/djh1995555/VehicleControl/blob/master/control/plot_py/LQR_Controller/LQR_Controller_15_Velocity.png)
![Heading Tracking](https://github.com/djh1995555/VehicleControl/blob/master/control/plot_py/LQR_Controller/LQR_Controller_15_Heading.png)

### MPC  

**MPC Controller  V=10m/s**

![Trajectory Tracking](https://github.com/djh1995555/VehicleControl/blob/master/control/plot_py/MPC_Controller/MPC_Controller_10_Path.png)
![Velocity Tracking](https://github.com/djh1995555/VehicleControl/blob/master/control/plot_py/MPC_Controller/MPC_Controller_10_Velocity.png)
![Heading Tracking](https://github.com/djh1995555/VehicleControl/blob/master/control/plot_py/MPC_Controller/MPC_Controller_10_Heading.png)
![Lateral Distance Error](https://github.com/djh1995555/VehicleControl/blob/master/control/plot_py/MPC_Controller/MPC_Controller_10_Lateral_Distance_Error.png)
![Velocity Error](https://github.com/djh1995555/VehicleControl/blob/master/control/plot_py/MPC_Controller/MPC_Controller_10_Velocity_Error.png)
![Heading Error](https://github.com/djh1995555/VehicleControl/blob/master/control/plot_py/MPC_Controller/MPC_Controller_10_Heading_Error.png)
**MPC Controller V=15m/s**
![Trajectory Tracking](https://github.com/djh1995555/VehicleControl/blob/master/control/plot_py/MPC_Controller/MPC_Controller_15_Path.png)
![Velocity Tracking](https://github.com/djh1995555/VehicleControl/blob/master/control/plot_py/MPC_Controller/MPC_Controller_15_Velocity.png)
![Heading Tracking](https://github.com/djh1995555/VehicleControl/blob/master/control/plot_py/MPC_Controller/MPC_Controller_15_Heading.png)


## Plot
```
~$ cd catkin_ws/src/control/plot_py/
~/catkin_ws/src/control/plot_py$ python result_plot.py
```
According to the output simulatin result, the scrtipt can plot the result of path tracking, velocity tracking and heading tracking, as well as the figures of error.


# Maintainers
[@djh1995555](https://github.com/djh1995555/)




