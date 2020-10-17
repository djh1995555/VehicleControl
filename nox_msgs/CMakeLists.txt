cmake_minimum_required(VERSION 2.8.3)
project(nox_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
        nav_msgs
  message_generation
  message_runtime
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
        ## nox basic message
        Group.msg
        Bound.msg
        Function.msg
        Status.msg
        StatusArray.msg
        Signal.msg
        SignalArray.msg

        ## nox scene message
        DrivingCommand.msg
        Chassis.msg
        Vehicle.msg
        PathPoint.msg
        Path.msg
        TrajectoryPoint.msg
        Trajectory.msg
        Obstacle.msg
        ObstacleArray.msg
        Scene.msg

        ## nox guide line message
        Overlap.msg
        SpeedControl.msg
        StopLine.msg
        Boundary.msg
        GuideLine.msg

        ## nox old message
        RoadPoint.msg
        RoadBlock.msg
        Road.msg
        Location.msg
        Lane.msg
)

add_service_files(
        FILES
        GetScene.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
        nav_msgs
)


###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs message_generation message_runtime
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

install(DIRECTORY include/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/../
        )
