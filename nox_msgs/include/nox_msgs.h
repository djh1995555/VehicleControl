/**
 * @brief nox_msgs总头文件
 *
 * @author Yarten
 * @date 2018-08-12
 */
#pragma once

/// basic message
#include "nox_msgs/Group.h"
#include "nox_msgs/Bound.h"
#include "nox_msgs/Function.h"
#include "nox_msgs/Status.h"
#include "nox_msgs/StatusArray.h"
#include "nox_msgs/Signal.h"
#include "nox_msgs/SignalArray.h"

/// scene message
#include "nox_msgs/DrivingCommand.h"
#include "nox_msgs/Chassis.h"
#include "nox_msgs/Vehicle.h"
#include "nox_msgs/PathPoint.h"
#include "nox_msgs/Path.h"
#include "nox_msgs/TrajectoryPoint.h"
#include "nox_msgs/Trajectory.h"
#include "nox_msgs/Obstacle.h"
#include "nox_msgs/ObstacleArray.h"
#include "nox_msgs/Scene.h"

/// guide line message
#include "nox_msgs/Overlap.h"
#include "nox_msgs/SpeedControl.h"
#include "nox_msgs/StopLine.h"
#include "nox_msgs/Boundary.h"
#include "nox_msgs/GuideLine.h"

/// service
#include "nox_msgs/GetScene.h"
