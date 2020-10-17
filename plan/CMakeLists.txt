cmake_minimum_required(VERSION 2.8.3)
project(plan)


find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  prius_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES plan
  CATKIN_DEPENDS gazebo_ros prius_msgs
#  DEPENDS system_lib
)

include_directories(include 
	${catkin_INCLUDE_DIRS}/include/plan/ 

  )

add_library(Planning
  src/PlanningConf.cpp
  src/GlobalTrajectory.cpp
  src/Planning.cpp
  src/VehicleState.cpp
  src/BoundingBox.cpp
  src/Obstacle.cpp
  src/Frame.cpp
  src/Planner/PlannerAgent.cpp
  src/Planner/Planner.cpp
  src/Planner/LatticePlanner.cpp
  src/FrenetPath.cpp
  src/Polynomial.cpp
  src/TrajectoryGenerator.cpp

)


add_executable(publish_path_once src/publish_path_once.cpp)
add_dependencies(publish_path_once ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(publish_path_once ${catkin_LIBRARIES})

add_executable(PlanningMain src/PlanningMain.cpp)
add_dependencies(PlanningMain ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(PlanningMain Planning ${catkin_LIBRARIES})

