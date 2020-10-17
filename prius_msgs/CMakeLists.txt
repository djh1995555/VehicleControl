cmake_minimum_required(VERSION 2.8.12)
project(prius_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
)

add_message_files(
  FILES
  My_Trajectory.msg
  My_Trajectory_Point.msg
  VehicleInfo.msg
  DimensionInfo.msg
  LateralData.msg
  LongitudinalData.msg
  Control.msg
  Augmented_My_Trajectory_Point.msg
  ObstacleAttr.msg
  Obstacles.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)
