#!/usr/bin/env python
import rospy
from prius_msgs.msg import My_Trajectory
from prius_msgs.msg import Augmented_My_Trajectory_Point
from prius_msgs.msg import Obstacles
from prius_msgs.msg import ObstacleAttr
import numpy as np
import matplotlib.pyplot as plt
from math import cos, sin, radians, atan2

DESIRED_TRAJECTORY_DATA_NUM = 11
POSE_INFO_NUM = 7
PI = 3.1415926
ROAD_WIDTH = 3.75
TARGET_PLOT=1
# 1 means the plot of trajectory with time
# 2 means the plot of velocity with station
# 3 means the plot of yaw with stationB
class TrajectoryPlotter:
    def __init__(self):
        self.global_trajectory_received = False
        self.local_trajectory_received = False
        self.obstacle_received =False
        self.pose_received =False
        self.global_trajectory_num = 0
        self.local_trajectory_num = 0
        self.obstacle_num =0
        self.pose_num =0
        self.pose_info = np.zeros((POSE_INFO_NUM,1),dtype=float)
        self.global_trajectory_sub = rospy.Subscriber("/global_trajectory", My_Trajectory, self.ReadTrajectoryCallback, queue_size=1)
        self.vehicle_info_sub = rospy.Subscriber("/prius/vehicle_info", Augmented_My_Trajectory_Point, self.ReadPoseCallback, queue_size=1)
        self.obstacle_sub = rospy.Subscriber("obstacle_info",Obstacles,self.ReadObstacleCallback, queue_size=1)
        self.local_trajectory_sub = rospy.Subscriber("/ontime_trajectory", My_Trajectory,self.ReadLocalTrajectoryCallback, queue_size=1)
        

    def PlotObstacle(self, x, y,length, width, yaw, zoom, obstacle_color="-k"):
        LENGTH = length*zoom  # [m]
        WIDTH = width*zoom   # [m]
        outline = np.array(
            [[-LENGTH/2, LENGTH/2, LENGTH/2, -LENGTH/2, -LENGTH/2],
            [WIDTH / 2, WIDTH / 2, - WIDTH / 2, -WIDTH / 2, WIDTH / 2]])

        Rot1 = np.array([[cos(yaw), sin(yaw)],[-sin(yaw), cos(yaw)]])

        outline = (outline.T.dot(Rot1)).T
        outline[0, :] += x
        outline[1, :] += y
        plt.plot(np.array(outline[0, :]).flatten(),
                np.array(outline[1, :]).flatten(), obstacle_color)
        plt.plot(x, y, "*")


    def PlotVehicle(self,x, y, yaw, zoom, steer=0.0, truck_color="-k"):  # pragma: no cover
        rospy.loginfo( "plot vehicle start")
        # Vehicle parameters
        LENGTH = 4*zoom  # [m]
        WIDTH = 2*zoom   # [m]
        BACK_TO_WHEEL = 1*zoom   # [m]
        WHEEL_LEN = 0.3*zoom   # [m]
        WHEEL_WIDTH = 0.2*zoom   # [m]
        TREAD = 0.7*zoom   # [m]
        WB = 2.86*zoom 

        outline = np.array(
            [[-BACK_TO_WHEEL, (LENGTH - BACK_TO_WHEEL), (LENGTH - BACK_TO_WHEEL),
            -BACK_TO_WHEEL, -BACK_TO_WHEEL],
            [WIDTH / 2, WIDTH / 2, - WIDTH / 2, -WIDTH / 2, WIDTH / 2]])

        fr_wheel = np.array(
            [[WHEEL_LEN, -WHEEL_LEN, -WHEEL_LEN, WHEEL_LEN, WHEEL_LEN],
            [-WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD, WHEEL_WIDTH -
            TREAD, WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD]])

        rr_wheel = np.copy(fr_wheel)

        fl_wheel = np.copy(fr_wheel)
        fl_wheel[1, :] *= -1
        rl_wheel = np.copy(rr_wheel)
        rl_wheel[1, :] *= -1

        Rot1 = np.array([[cos(yaw), sin(yaw)],
                        [-sin(yaw), cos(yaw)]])
        Rot2 = np.array([[cos(steer), sin(steer)],
                        [-sin(steer), cos(steer)]])

        fr_wheel = (fr_wheel.T.dot(Rot2)).T
        fl_wheel = (fl_wheel.T.dot(Rot2)).T
        fr_wheel[0, :] += WB
        fl_wheel[0, :] += WB

        fr_wheel = (fr_wheel.T.dot(Rot1)).T
        fl_wheel = (fl_wheel.T.dot(Rot1)).T

        outline = (outline.T.dot(Rot1)).T
        rr_wheel = (rr_wheel.T.dot(Rot1)).T
        rl_wheel = (rl_wheel.T.dot(Rot1)).T

        outline[0, :] += x
        outline[1, :] += y
        fr_wheel[0, :] += x
        fr_wheel[1, :] += y
        rr_wheel[0, :] += x
        rr_wheel[1, :] += y
        fl_wheel[0, :] += x
        fl_wheel[1, :] += y
        rl_wheel[0, :] += x
        rl_wheel[1, :] += y

        plt.plot(np.array(outline[0, :]).flatten(),
                np.array(outline[1, :]).flatten(), truck_color)
        plt.plot(np.array(fr_wheel[0, :]).flatten(),
                np.array(fr_wheel[1, :]).flatten(), truck_color)
        plt.plot(np.array(rr_wheel[0, :]).flatten(),
                np.array(rr_wheel[1, :]).flatten(), truck_color)
        plt.plot(np.array(fl_wheel[0, :]).flatten(),
                np.array(fl_wheel[1, :]).flatten(), truck_color)
        plt.plot(np.array(rl_wheel[0, :]).flatten(),
                np.array(rl_wheel[1, :]).flatten(), truck_color)
        plt.plot(x, y, "*")
        rospy.loginfo( "plot vehicle finish")

    def TrajectoryPlot(self):

        lateral_controller=rospy.get_param("/lateral_controller")
        centralized_controller=rospy.get_param("/centralized_controller")
        trajectory_velocity=rospy.get_param("/trajectory_velocity")
        # trajectory_velocity = "10"
        controllers_num=rospy.get_param("/controllers_num")
        longitudinal_controller=rospy.get_param("/longitudinal_controller")  


        if controllers_num==2:
            if trajectory_velocity=="record":
                label1 = longitudinal_controller+" and "+lateral_controller+"\n"+"curve velocity is not constant"
                label2 = longitudinal_controller+" and "+lateral_controller+"(Velocity Comparison With Station)"+"\n"+"curve velocity is not constant"
                label3 = longitudinal_controller+" and "+lateral_controller+"(Heading Comparison with Station)"+"\n"+"curve velocity is not constant"
            else:
                label1 = longitudinal_controller+" and "+lateral_controller+"\n"+"curve velocity = "+trajectory_velocity+" m/s"
                label2 = longitudinal_controller+" and "+lateral_controller+"(Velocity Comparison With Station)"+"\n"+"curve velocity = "+trajectory_velocity+" m/s"
                label3 = longitudinal_controller+" and "+lateral_controller+"(Heading Comparison with Station)"+"\n"+"curve velocity = "+trajectory_velocity+" m/s"
        elif controllers_num==1:
            if trajectory_velocity=="record":
                label1 = centralized_controller+"\n"+"curve velocity is not constant"
                label2 = centralized_controller+"(Velocity Comparison With Station)"+"\n"+"curve velocity is not constant"
                label3 = centralized_controller+"(Heading Comparison with Station)"+"\n"+"curve velocity is not constant"
            else:
                label1 = centralized_controller+"\n"+"curve velocity = "+trajectory_velocity+" m/s"
                label2 = centralized_controller+"(Velocity Comparison With Station)"+"\n"+"curve velocity = "+trajectory_velocity+" m/s"
                label3 = centralized_controller+"(Heading Comparison with Station)"+"\n"+"curve velocity = "+trajectory_velocity+" m/s"

        loc='center'
        font_dict={'fontsize': 20, 'fontweight' : 8.2, 'verticalalignment': 'baseline', 'horizontalalignment': loc}
        

        range = max(self.trajectory_info[1])-min(self.trajectory_info[1])
        tra_ymin=min(self.trajectory_info[1])-20
        tra_ymax=tra_ymin+range+90
        tra_xmin=min(self.trajectory_info[0])-60
        tra_xmax=tra_xmin+(range+50)*16/9

        zoom = (tra_xmax-tra_xmin)/200
        road_width=ROAD_WIDTH*zoom   

        velocity_xmin=min(self.trajectory_info[5])-5
        velocity_xmax=max(self.trajectory_info[5])+5
        velocity_ymin=min(self.trajectory_info[7])-3
        velocity_ymax=max(self.trajectory_info[7])+5

        yaw_xmin=min(self.trajectory_info[5])-5
        yaw_xmax=max(self.trajectory_info[5])+5
        yaw_ymin=min(self.trajectory_info[3])-0.5
        yaw_ymax=max(self.trajectory_info[3])+1

        self.road = self.CreateRoad(self.planned_trajectory,road_width)
           
        fig=plt.figure(1)      
          
        if self.local_trajectory_original.ESTOP:
            goal_state.x = 0
            goal_state.y = 0
            goal_state.kappa = 0
            goal_state.v = 0
            goal_state.gear =1
        else:
            goal_state=self.local_trajectory_original.trajectory_points[int(self.current_info.goal_id)]
            #preview_state=local_trajectory_original.trajectory_points[int(current_info.preview_id)]

        if TARGET_PLOT==1:
            plt.plot(self.trajectory_info[0,:],self.trajectory_info[1,:],"--",c="c",linewidth=2.0,label="global trajectory")
            plt.plot(self.road[0,:],self.road[1,:],".",c="c",linewidth=3.0)
            plt.plot(self.road[2,:],self.road[3,:],".",c="c",linewidth=3.0)
            # plt.plot(road[0,:],road[1,:],c="c",linewidth=3.0)
            # plt.plot(road[2,:],road[3,:],c="c",linewidth=3.0)

            for obstacle in self.obstacle_list:
                self.PlotObstacle(obstacle.x,obstacle.y,obstacle.length,obstacle.width,obstacle.heading,zoom)

            plt.plot(self.local_trajectory[0,:],self.local_trajectory[1,:],".",c="brown",linewidth=2.0,label="local trajectory")
            plt.plot(self.local_trajectory[0,:],self.local_trajectory[1,:],c="brown",linewidth=2.0,label="local trajectory")

            plt.plot(self.pose_info[0,1:-1],self.pose_info[1,1:-1],c="r",linewidth=2.0,label="actual trajectory")
            plt.plot(goal_state.x,goal_state.y,"bo",label = "goal point",)
            # plt.plot(preview_state.x,preview_state.y,"go",label="preview point")
            plt.title(label1,fontdict=font_dict,loc=loc)
            font_size = 12 
            plt.text(tra_xmin+10, tra_ymax-10, "target velocity = "+str(round(goal_state.v,4))+" m/s",fontsize=font_size) 
            plt.text(tra_xmin+100, tra_ymax-10, "current velocity = "+str(round(self.current_info.trajectory_point.v,4))+" m/s",fontsize=font_size)
            plt.text(tra_xmin+210, tra_ymax-10, "current curvature = "+str(round(goal_state.kappa,4)),fontsize=font_size) 
            plt.text(tra_xmin+320, tra_ymax-10, "current slope = "+str(round(self.current_info.slope,4)),fontsize=font_size) 

            plt.text(tra_xmin+10, tra_ymax-20, "velocity error = "+str(round(self.current_info.velocity_error,4))+" m/s",fontsize=font_size) 
            plt.text(tra_xmin+10, tra_ymax-30, "lateral distance error = "+str(round(self.current_info.lateral_distance_error*100,4))+" cm",fontsize=font_size) 
            plt.text(tra_xmin+10, tra_ymax-40, "heading error = "+str(round(self.current_info.heading_error*180/PI,4))+" deg",fontsize=font_size)
            plt.text(tra_xmin+10, tra_ymax-50, "computational time = "+str(round(self.current_info.computational_time,4))+" ms",fontsize=font_size) 

            plt.text(tra_xmin+210, tra_ymax-20, "average velocity error = "+str(round(self.current_info.average_velocity_error,4))+" m/s",fontsize=font_size) 
            plt.text(tra_xmin+210, tra_ymax-30, "average lateral distance error = "+str(round(self.current_info.average_lateral_distance_error*100,4))+" cm",fontsize=font_size) 
            plt.text(tra_xmin+210, tra_ymax-40, "average heading error = "+str(round(self.current_info.average_heading_error*180/PI,4))+" deg",fontsize=font_size) 
            plt.text(tra_xmin+210, tra_ymax-50, "average computational time = "+str(round(self.current_info.average_computational_time,4))+" ms",fontsize=font_size) 

            plt.text(tra_xmin+100, tra_ymax-20, "max velocity error = "+str(round(self.current_info.max_velocity_error,4))+" m/s",fontsize=font_size) 
            plt.text(tra_xmin+100, tra_ymax-30, "max lateral distance error = "+str(round(self.current_info.max_lateral_distance_error*100,4))+" cm",fontsize=font_size)  
            plt.text(tra_xmin+100, tra_ymax-40, "max heading error = "+str(round(self.current_info.max_heading_error*180/PI,4))+" deg",fontsize=font_size) 
            plt.text(tra_xmin+100, tra_ymax-50, "max computational time = "+str(round(self.current_info.max_computational_time,4))+" ms",fontsize=font_size)            

            vehicle_heading = self.current_info.trajectory_point.theta
            if(goal_state.gear == -1):
                vehicle_heading = vehicle_heading-PI
            rospy.loginfo( "current x = %0.4f",self.current_info.trajectory_point.x)
            rospy.loginfo( "current y = %0.4f",self.current_info.trajectory_point.y)

            self.PlotVehicle(self.current_info.trajectory_point.x,self.current_info.trajectory_point.y,vehicle_heading,zoom,0.0)

            rospy.loginfo("1")
            plt.legend()
            plt.xlabel('X/m')
            plt.ylabel('Y/m')
            rospy.loginfo("2")
            axes = plt.gca()
            axes.set_xlim([tra_xmin,tra_xmax])
            axes.set_ylim([tra_ymin,tra_ymax])
            rospy.loginfo("3")


        elif TARGET_PLOT==2:
            plt.plot(self.trajectory_info[5,:],self.trajectory_info[7,:],"-",c="c",linewidth=2.0,label="planned velocity")
            plt.plot(self.local_trajectory[5,:],self.local_trajectory[7,:],"brown",linewidth=2.0,label="local trajectory")
            plt.plot(self.pose_info[5,1:-1],self.pose_info[2,1:-1],c="r",linewidth=2.0,label="actual velocity")
            # plt.plot(pose_info[5,1:-1],pose_info[6,1:-1],c="b",linewidth=2.0,label="actual slope")
            plt.plot(goal_state.s,goal_state.v,"bo",label = "goal point",)
            #plt.plot(preview_state.s,preview_state.v,"go",label="preview point")
            plt.text(velocity_xmin+35,velocity_ymax-1, "velocity error = "+str(round(self.current_info.velocity_error,4))+" m/s",fontsize=14) 
            plt.text(velocity_xmin+35,velocity_ymax-2, "current velocity = "+str(round(self.current_info.trajectory_point.v,4))+" m/s",fontsize=14)
            plt.text(velocity_xmin+35,velocity_ymax-3, "current slope = "+str(round(self.current_info.slope,4)),fontsize=14) 
            plt.title(label2,fontdict=font_dict,loc=loc)
            plt.legend()
            plt.grid()
            plt.xlabel('Traveled Distance/m')
            plt.ylabel('Velocity/m')
            ax1 = plt.gca()
            ax1.set_xlim([velocity_xmin,velocity_xmax])
            ax1.set_ylim([velocity_ymin,velocity_ymax])


        elif TARGET_PLOT==3:
            plt.plot(self.trajectory_info[5,:],self.trajectory_info[3,:],"-",c="c",linewidth=2.0,label="planned heading")
            plt.plot(self.local_trajectory[5,:],self.local_trajectory[3,:],"brown",linewidth=2.0,label="local trajectory")
            plt.plot(self.pose_info[5,1:-1],self.pose_info[3,1:-1],c="r",linewidth=2.0,label="actual heading")
            plt.plot(goal_state.s,goal_state.theta,"bo",label = "goal point",)
            #plt.plot(preview_state.s,preview_state.theta,"go",label="preview point")
            plt.title(label3,fontdict=font_dict,loc=loc)
            plt.legend()
            plt.xlabel('Traveled Distance/m')
            plt.ylabel('Heading/rad')
            axes = plt.gca()
            axes.set_xlim([yaw_xmin,yaw_xmax])
            axes.set_ylim([yaw_ymin,yaw_ymax])

        # plt.show()
        # plt.cla()
        plt.pause(0.1)
        plt.cla()
        rospy.loginfo( "plot finish")


        
        # while not rospy.is_shutdown():
        #     rospy.loginfo( "plot start")
        #     if self.local_trajectory_original.ESTOP:
        #         goal_state.x = 0
        #         goal_state.y = 0
        #         goal_state.kappa = 0
        #         goal_state.v = 0
        #         goal_state.gear =1
        #     else:
        #         goal_state=self.local_trajectory_original.trajectory_points[int(self.current_info.goal_id)]
        #         #preview_state=local_trajectory_original.trajectory_points[int(current_info.preview_id)]

        #     if TARGET_PLOT==1:
        #         plt.plot(self.trajectory_info[0,:],self.trajectory_info[1,:],"--",c="c",linewidth=2.0,label="global trajectory")
        #         plt.plot(self.road[0,:],self.road[1,:],".",c="c",linewidth=3.0)
        #         plt.plot(self.road[2,:],self.road[3,:],".",c="c",linewidth=3.0)
        #         # plt.plot(road[0,:],road[1,:],c="c",linewidth=3.0)
        #         # plt.plot(road[2,:],road[3,:],c="c",linewidth=3.0)

        #         for obstacle in self.obstacle_list:
        #             self.PlotObstacle(obstacle.x,obstacle.y,obstacle.length,obstacle.width,obstacle.heading,zoom)

        #         plt.plot(self.local_trajectory[0,:],self.local_trajectory[1,:],"brown",linewidth=2.0,label="local trajectory")

        #         plt.plot(self.pose_info[0,1:-1],self.pose_info[1,1:-1],c="r",linewidth=2.0,label="actual trajectory")
        #         plt.plot(goal_state.x,goal_state.y,"bo",label = "goal point",)
        #         # plt.plot(preview_state.x,preview_state.y,"go",label="preview point")
        #         plt.title(label1,fontdict=font_dict,loc=loc)
        #         plt.text(tra_xmin+15, tra_ymax-10, "distance error = "+str(round(self.current_info.lateral_distance_error,4))+" m",fontsize=14) 
        #         plt.text(tra_xmin+15, tra_ymax-20, "heading error = "+str(round(self.current_info.heading_error*180/PI,4))+" degree",fontsize=14) 
        #         plt.text(tra_xmin+15, tra_ymax-30, "current curvature = "+str(round(goal_state.kappa,4)),fontsize=14) 
        #         plt.text(tra_xmin+15, tra_ymax-40, "current slope = "+str(round(self.current_info.slope,4)),fontsize=14) 
        #         plt.text(tra_xmin+115, tra_ymax-10, "velocity error = "+str(round(self.current_info.velocity_error,4))+" m/s",fontsize=14) 
        #         plt.text(tra_xmin+115, tra_ymax-20, "current velocity = "+str(round(self.current_info.trajectory_point.v,4))+" m/s",fontsize=14) 
        #         plt.text(tra_xmin+115, tra_ymax-30, "target velocity = "+str(round(goal_state.v,4))+" m/s",fontsize=14) 
        #         if(goal_state.gear == 1 ):
        #             vehicle_heading = self.current_info.trajectory_point.theta
        #         elif(goal_state.gear == -1):
        #             vehicle_heading = self.current_info.trajectory_point.theta-PI
        #         rospy.loginfo( "current x = %0.4f",self.current_info.trajectory_point.x)
        #         rospy.loginfo( "current y = %0.4f",self.current_info.trajectory_point.y)
        #         self.PlotVehicle(self.current_info.trajectory_point.x,self.current_info.trajectory_point.y,vehicle_heading,zoom,0.0)
        #         plt.legend()
        #         plt.xlabel('X/m')
        #         plt.ylabel('Y/m')
        #         axes = plt.gca()
        #         axes.set_xlim([tra_xmin,tra_xmax])
        #         axes.set_ylim([tra_ymin,tra_ymax])

        #         # plt.annotate("R=80",xy=(164.4,13.47), xycoords='data',xytext= (-60,+60),textcoords = 'offset points'
        #         #      ,fontsize = 16,arrowprops = dict(arrowstyle = '->',connectionstyle = 'arc3,rad = .2'))
        #         # plt.annotate("R=40",xy=(188.3,228.2), xycoords='data',xytext= (-80,-80),textcoords = 'offset points'
        #         #      ,fontsize = 16,arrowprops = dict(arrowstyle = '->',connectionstyle = 'arc3,rad = .2'))
        #         # plt.annotate("R=40",xy=(55.43,88.43), xycoords='data',xytext= (+40,+40),textcoords = 'offset points'
        #         #      ,fontsize = 16,arrowprops = dict(arrowstyle = '->',connectionstyle = 'arc3,rad = .2'))
        #         # plt.annotate("R=60",xy=(26.39,218.1), xycoords='data',xytext= (-100,-60),textcoords = 'offset points'
        #         #      ,fontsize = 16,arrowprops = dict(arrowstyle = '->',connectionstyle = 'arc3,rad = .2'))
        #         # plt.annotate("R=60",xy=(-247.8,216.3), xycoords='data',xytext= (+60,-60),textcoords = 'offset points'
        #         #      ,fontsize = 16,arrowprops = dict(arrowstyle = '->',connectionstyle = 'arc3,rad = .2'))
        #         # plt.annotate("R=20",xy=(-184.3,112.4), xycoords='data',xytext= (+40,+40),textcoords = 'offset points'
        #         #      ,fontsize = 16,arrowprops = dict(arrowstyle = '->',connectionstyle = 'arc3,rad = .2'))
        #         # plt.annotate("R=40",xy=(-236.1,22.73), xycoords='data',xytext= (+40,+40),textcoords = 'offset points'
        #         #      ,fontsize = 16,arrowprops = dict(arrowstyle = '->',connectionstyle = 'arc3,rad = .2'))

        #     elif TARGET_PLOT==2:
        #         plt.plot(self.trajectory_info[5,:],self.trajectory_info[7,:],"-",c="c",linewidth=2.0,label="planned velocity")
        #         plt.plot(self.local_trajectory[5,:],self.local_trajectory[7,:],"brown",linewidth=2.0,label="local trajectory")
        #         plt.plot(self.pose_info[5,1:-1],self.pose_info[2,1:-1],c="r",linewidth=2.0,label="actual velocity")
        #         # plt.plot(pose_info[5,1:-1],pose_info[6,1:-1],c="b",linewidth=2.0,label="actual slope")
        #         plt.plot(goal_state.s,goal_state.v,"bo",label = "goal point",)
        #         #plt.plot(preview_state.s,preview_state.v,"go",label="preview point")
        #         plt.text(velocity_xmin+35,velocity_ymax-1,"velocity error = "+str(round(self.current_info.velocity_error,4))+" m/s",fontsize=14) 
        #         plt.text(velocity_xmin+35,velocity_ymax-2, "current velocity = "+str(round(self.current_info.trajectory_point.v,4))+" m/s",fontsize=14)
        #         plt.text(velocity_xmin+35,velocity_ymax-3, "current slope = "+str(round(self.current_info.slope,4)),fontsize=14) 
        #         plt.title(label2,fontdict=font_dict,loc=loc)
        #         plt.legend()
        #         plt.grid()
        #         plt.xlabel('Traveled Distance/m')
        #         plt.ylabel('Velocity/m')
        #         ax1 = plt.gca()
        #         ax1.set_xlim([velocity_xmin,velocity_xmax])
        #         ax1.set_ylim([velocity_ymin,velocity_ymax])


        #     elif TARGET_PLOT==3:
        #         plt.plot(self.trajectory_info[5,:],self.trajectory_info[3,:],"-",c="c",linewidth=2.0,label="planned heading")
        #         plt.plot(self.local_trajectory[5,:],self.local_trajectory[3,:],"brown",linewidth=2.0,label="local trajectory")
        #         plt.plot(self.pose_info[5,1:-1],self.pose_info[3,1:-1],c="r",linewidth=2.0,label="actual heading")
        #         plt.plot(goal_state.s,goal_state.theta,"bo",label = "goal point",)
        #         #plt.plot(preview_state.s,preview_state.theta,"go",label="preview point")
        #         plt.title(label3,fontdict=font_dict,loc=loc)
        #         plt.legend()
        #         plt.xlabel('Traveled Distance/m')
        #         plt.ylabel('Heading/rad')
        #         axes = plt.gca()
        #         axes.set_xlim([yaw_xmin,yaw_xmax])
        #         axes.set_ylim([yaw_ymin,yaw_ymax])

        #     plt.pause(0.1)
        #     plt.cla()
        #     rospy.loginfo( "plot finish")

    def ReadTrajectoryCallback(self,planned_trajectory):
        self.planned_trajectory = planned_trajectory
        length = len(planned_trajectory.trajectory_points)
        trajectory_info = np.zeros((DESIRED_TRAJECTORY_DATA_NUM,length),dtype=float)
        for i in range(length):
            trajectory_info[0,i]=planned_trajectory.trajectory_points[i].x
            trajectory_info[1,i]=planned_trajectory.trajectory_points[i].y
            trajectory_info[2,i]=planned_trajectory.trajectory_points[i].z
            trajectory_info[3,i]=planned_trajectory.trajectory_points[i].theta
            trajectory_info[4,i]=planned_trajectory.trajectory_points[i].kappa
            trajectory_info[5,i]=planned_trajectory.trajectory_points[i].s
            trajectory_info[6,i]=planned_trajectory.trajectory_points[i].dkappa
            trajectory_info[7,i]=planned_trajectory.trajectory_points[i].v
            trajectory_info[8,i]=planned_trajectory.trajectory_points[i].a
            trajectory_info[9,i]=planned_trajectory.trajectory_points[i].relative_time
            trajectory_info[10,i]=planned_trajectory.trajectory_points[i].gear
        self.trajectory_info = trajectory_info
        self.global_trajectory_num=self.global_trajectory_num+1
        #rospy.loginfo( "the %d global_trajectory is received",self.global_trajectory_num)
        self.global_trajectory_received =True

    def ReadLocalTrajectoryCallback(self,planned_trajectory):
        self.local_trajectory_original = planned_trajectory
        length = len(planned_trajectory.trajectory_points)
        trajectory_info = np.zeros((DESIRED_TRAJECTORY_DATA_NUM,length),dtype=float)
        for i in range(length):
            
            trajectory_info[0,i]=planned_trajectory.trajectory_points[i].x
            trajectory_info[1,i]=planned_trajectory.trajectory_points[i].y
            trajectory_info[2,i]=planned_trajectory.trajectory_points[i].z
            trajectory_info[3,i]=planned_trajectory.trajectory_points[i].theta
            trajectory_info[4,i]=planned_trajectory.trajectory_points[i].kappa
            trajectory_info[5,i]=planned_trajectory.trajectory_points[i].s
            trajectory_info[6,i]=planned_trajectory.trajectory_points[i].dkappa
            trajectory_info[7,i]=planned_trajectory.trajectory_points[i].v
            trajectory_info[8,i]=planned_trajectory.trajectory_points[i].a
            trajectory_info[9,i]=planned_trajectory.trajectory_points[i].relative_time
            trajectory_info[10,i]=planned_trajectory.trajectory_points[i].gear
        self.local_trajectory = trajectory_info
        self.local_trajectory_num=self.local_trajectory_num+1
        if(self.local_trajectory_num%100==0):
            rospy.loginfo( "the %d local_trajectory is received",self.local_trajectory_num)
        self.local_trajectory_received =True

    def ReadPoseCallback(self,current_info):
        self.current_info=current_info
        temp = np.zeros((POSE_INFO_NUM,1),dtype=float)
        temp[0]=current_info.trajectory_point.x
        temp[1]=current_info.trajectory_point.y
        temp[2]=current_info.trajectory_point.v
        theta=current_info.trajectory_point.theta
        if abs(abs(theta)-PI)<0.05:
            theta = PI
        temp[3]=theta
        temp[4]=current_info.trajectory_point.relative_time
        temp[5]=current_info.vehicle_info.longitudinal_data.traveled_distance
        temp[6]=current_info.slope

        self.pose_info = np.hstack([self.pose_info,temp])
        self.pose_num=self.pose_num+1
        if(self.pose_num%100==0):
            rospy.loginfo( "the %d pose is received",self.pose_num)
            rospy.loginfo( "current x = %0.4f",self.current_info.trajectory_point.x)
            rospy.loginfo( "current y = %0.4f",self.current_info.trajectory_point.y)
        self.pose_received = True

        if self.global_trajectory_received&self.local_trajectory_received & self.obstacle_received & self.pose_received:
            self.TrajectoryPlot()
        
    
    def ReadObstacleCallback(self,obstacles):
        self.obstacle_list = obstacles.Obstacles
        self.obstacle_num=self.obstacle_num+1
        #rospy.loginfo( "the %d obstacle is received",self.obstacle_num)
        self.obstacle_received =True

    def CreateRoad(self,planned_trajectory,road_width):
        length = len(planned_trajectory.trajectory_points)
        road= np.zeros((4,length),dtype=float)
        for i in range(length):
            point = planned_trajectory.trajectory_points[i]
            angle_right = point.theta-PI/2
            angle_left = angle_right-PI
            road[0,i]=point.x+road_width/2*cos(angle_left)
            road[1,i]=point.y+road_width/2*sin(angle_left)
            road[2,i]=point.x+road_width/2*cos(angle_right)
            road[3,i]=point.y+road_width/2*sin(angle_right)
        return road


    
    
if __name__ == "__main__":

    rospy.init_node("trajectory_plot")
    TP = TrajectoryPlotter()
    # if TP.global_trajectory_received&TP.local_trajectory_received & TP.obstacle_received & TP.pose_received:
    #     TP.TrajectoryPlot()
    rospy.spin()