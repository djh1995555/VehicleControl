#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from nox_msgs import SignalArray
import numpy as np

CHASSIS_STATES_NUM=8
LOCALIZATION_NUM=3
GPS_ODOM_NUM=12
IMU_DATA_NUM=9



def ReadChassiState(current_chassis_states):
    temp = np.zeros((CHASSIS_STATES_NUM,1),dtype=float)
    temp[0]=panel.Steer.Get()
    
    return temp
    
def ReadLocalization(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
def ReadGPSOdometry(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
def ReadIMU(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    


def ReadRosBag():
    chassis_states = np.zeros((CHASSIS_STATES_NUM,1),dtype=float)
    localization = np.zeros((LOCALIZATION_NUM,1),dtype=float)
    gps_odom = np.zeros((GPS_ODOM_NUM,1),dtype=float)
    imu_data = np.zeros((IMU_DATA_NUM,1),dtype=float)

    while not rospy.is_shutdown():
        current_chassis_states = rospy.wait_for_message("/chassis_states", SignalArray, timeout=None)
        temp=ReadChassiState(current_chassis_states)
        chassis_states=np.hstack([chassis_states,temp])

        current_localization = rospy.wait_for_message("/gps/fix", NavSatFix, timeout=None)
        temp=ReadLocalization(current_localization)
        localization=np.hstack([localization,temp])

        current_gps_odom = rospy.wait_for_message("/gps/odom", Odometry, timeout=None)
        temp=ReadGPSOdometry(current_gps_odom)
        gps_odom=np.hstack([gps_odom,temp])

        current_imu_data = rospy.wait_for_message("/imu/data", Imu, timeout=None)
        temp=ReadIMU(current_imu_data)
        imu_data=np.hstack([imu_data,temp])



if __name__ == '__main__':
    rospy.init_node('read_ros_bag')
    ReadRosBag()