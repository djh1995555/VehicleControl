import numpy as np
import math
import matplotlib.pyplot as plt
# from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from nox_msgs.msg import SignalArray
from dateutil.parser import parse

IMU_DATA_NUM = 11
VEHICLE_STATE_NUM = 14
GPS_DATA_NUM = 5
CHASSIS_DATA_NUM = 9\

PI = 3.1415926

FILE_NAME= "empty_flat_10.txt"
global INITIAL_TIME_STAMP


def FindAll(s, char):
    temp = []
    index = 0
    for i in s:
        if char == i:
            temp.append(index)
        index+=1
    return temp

def GetData(s,container,size):
    temp = np.zeros(size,dtype=float)
    pos = FindAll(s,"\t")
    length = len(pos)
    for i in range(length-1):
        temp[i]=float(s[pos[i]+1:pos[i+1]])
    temp[-2]=float(s[pos[-1]:-1])
    pos_INFO = s.find("INFO")
    time_str = s[11:pos_INFO-1]
    time_stamp = parse(time_str)
    time_interval = time_stamp-initial_time_stamp
    temp[-1]=time_interval.total_seconds()

    container = np.vstack([container,temp])
    return container


def ReadFile(file_name):
    IMU = np.zeros(IMU_DATA_NUM,dtype=float)
    vehicle_state = np.zeros(VEHICLE_STATE_NUM,dtype=float)
    gps_data = np.zeros(GPS_DATA_NUM,dtype=float)
    chassis = np.zeros(CHASSIS_DATA_NUM,dtype=float)
    with open(file_name, 'r') as fp:
        s = fp.readline()
        pos_INFO = s.find("INFO")
        time_str = s[11:pos_INFO-1]
        global initial_time_stamp
        initial_time_stamp= parse(time_str)
        while True:
            s = fp.readline()
            if s == '':
                break
            else:
                index_0 = s.find("@")
                index_1 = s.find("_",index_0)
                index_2 = s.find(":",index_1)
                sign = s[index_1:index_2]
                if(sign=="_imu_data"):
                    IMU = GetData(s,IMU,IMU_DATA_NUM)
                elif (sign=="_gps_odom"):
                    vehicle_state = GetData(s,vehicle_state,VEHICLE_STATE_NUM)
                elif (sign=="_gps_fix"):
                    gps_data = GetData(s,gps_data,GPS_DATA_NUM)
                elif (sign=="_chassis_states"):
                    chassis = GetData(s,chassis,CHASSIS_DATA_NUM)
    DATA = [IMU,vehicle_state,gps_data,chassis]
    return DATA

def DataReader():
    data = ReadFile(FILE_NAME)

    # IMU
    imu_msg = Imu()
    imu_msg.orientation.x = data[0][1:-1,0]
    imu_msg.orientation.y = data[0][1:-1,1]
    imu_msg.orientation.z = data[0][1:-1,2]
    imu_msg.orientation.w = data[0][1:-1,3]
    imu_msg.linear_acceleration.x=data[0][1:-1,4]
    imu_msg.linear_acceleration.y=data[0][1:-1,5]
    imu_msg.linear_acceleration.z=data[0][1:-1,6]
    imu_msg.angular_velocity.x=data[0][1:-1,7] 
    imu_msg.angular_velocity.y=data[0][1:-1,8]
    imu_msg.angular_velocity.z=data[0][1:-1,9]
    imu_time = data[0][1:-1,10]

    imu_length = len(imu_msg.orientation.x)
    imu_roll_angle=np.zeros(imu_length,dtype=float)
    imu_pitch_angle=np.zeros(imu_length,dtype=float)
    imu_yaw_angle=np.zeros(imu_length,dtype=float)
    for i in range(imu_length):
        x = imu_msg.orientation.x[i]
        y = imu_msg.orientation.y[i]
        z = imu_msg.orientation.z[i]
        w = imu_msg.orientation.w[i]
        imu_roll_angle[i] = math.atan2(2 * (y*z + w*x), w*w - x*x - y*y + z*z)*180/PI
        imu_pitch_angle[i] = math.asin(-2 * (x*z - w*y))*180/PI
        imu_yaw_angle[i] = math.atan2(2 * (x*y + w*z), w*w + x*x - y*y - z*z)*180/PI


    # vehicle_state
    vehicle_msg =Odometry()
    vehicle_msg.pose.pose.position.x =data[1][1:-1,0]
    vehicle_msg.pose.pose.position.y =data[1][1:-1,1]
    vehicle_msg.pose.pose.position.z =data[1][1:-1,2]
    vehicle_msg.pose.pose.orientation.x = data[1][1:-1,3]
    vehicle_msg.pose.pose.orientation.y = data[1][1:-1,4]
    vehicle_msg.pose.pose.orientation.z = data[1][1:-1,5]
    vehicle_msg.pose.pose.orientation.w = data[1][1:-1,6] 
    vehicle_msg.twist.twist.linear.x= data[1][1:-1,7]
    vehicle_msg.twist.twist.linear.y= data[1][1:-1,8]
    vehicle_msg.twist.twist.linear.z= data[1][1:-1,9]
    vehicle_msg.twist.twist.angular.x= data[1][1:-1,10]
    vehicle_msg.twist.twist.angular.y= data[1][1:-1,11]
    vehicle_msg.twist.twist.angular.z= data[1][1:-1,12]
    vehicle_time = data[1][1:-1,13]

    odom_length = len(vehicle_msg.pose.pose.orientation.x)
    odom_roll_angle=np.zeros(odom_length,dtype=float)
    odom_pitch_angle=np.zeros(odom_length,dtype=float)
    odom_yaw_angle=np.zeros(odom_length,dtype=float)
    for i in range(odom_length):
        x = vehicle_msg.pose.pose.orientation.x[i]
        y = vehicle_msg.pose.pose.orientation.y[i]
        z = vehicle_msg.pose.pose.orientation.z[i]
        w = vehicle_msg.pose.pose.orientation.w[i]
        odom_roll_angle[i] = math.atan2(2 * (y*z + w*x), w*w - x*x - y*y + z*z)*180/PI
        odom_pitch_angle[i] = math.asin(-2 * (x*z - w*y))*180/PI
        odom_yaw_angle[i] = math.atan2(2 * (x*y + w*z), w*w + x*x - y*y - z*z)*180/PI



    # GPS
    pos_gps = NavSatFix()
    pos_gps.latitude = data[2][1:-1,1]
    pos_gps.longitude = data[2][1:-1,2]
    pos_gps.altitude = data[2][1:-1,3]
    gps_time = data[2][1:-1,4]

    # chassis
    chassis_msg_Steer = data[3][1:-1,0]
    chassis_msg_Throttle = data[3][1:-1,1]
    chassis_msg_Brake = data[3][1:-1,2]
    chassis_msg_Wheel = data[3][1:-1,3]
    chassis_msg_Speed = data[3][1:-1,4]
    chassis_msg_BucketAngle = data[3][1:-1,5]
    chassis_msg_EngineSpeed = data[3][1:-1,6]
    chassis_msg_EngineTorque = data[3][1:-1,7]
    chassis_time = data[3][1:-1,8]

    plt.figure(1)
    loc='center'
    font_dict={'fontsize': 20,\
            'fontweight' : 8.2,\
            'verticalalignment': 'baseline',\
            'horizontalalignment': loc}
    plt.title('trajectory',fontdict=font_dict,loc=loc) 
    plt.plot(pos_gps.latitude,pos_gps.longitude, color='cyan', label='desired_trajectory')
    plt.xlabel('latitude')
    plt.ylabel('longitude')


    plt.figure(2)
    loc='center'
    font_dict={'fontsize': 20,\
            'fontweight' : 8.2,\
            'verticalalignment': 'baseline',\
            'horizontalalignment': loc}
    plt.title('Pitch Angle Comparison',fontdict=font_dict,loc=loc) 
    plt.plot(vehicle_time,odom_pitch_angle, color='cyan', label='pitch angle from odom')
    plt.plot(imu_time,-imu_pitch_angle, color='r', label='pitch angle from imu')
    plt.legend()
    plt.ylabel('pitch angle/degree')
    plt.xlabel('time/s')

    plt.figure(3)
    loc='center'
    font_dict={'fontsize': 20,\
            'fontweight' : 8.2,\
            'verticalalignment': 'baseline',\
            'horizontalalignment': loc}
    plt.title('trajectory',fontdict=font_dict,loc=loc) 
    plt.plot(vehicle_msg.pose.pose.position.x ,vehicle_msg.pose.pose.position.y, color='cyan', label='desired_trajectory')
    plt.xlabel('X')
    plt.ylabel('Y')


    plt.show()
if __name__ == '__main__':
    DataReader()