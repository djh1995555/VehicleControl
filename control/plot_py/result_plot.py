import numpy as np
import matplotlib.pyplot as plt
import math
import os
import shutil

PI = 3.1415926
DESIRED_TRAJECTORY_DATA_NUM = 11
""" trajectory_point{
0 x: 0.000000
1 y: 0.000000
2 z: 1.000000
3 theta: 0.000000
4 kappa: 0.000000
5 s: 0.000000
6 dkappa: 0.000000
7 v: 15.000000
8 a: 0.000000
9 relative_time: 0.000000
10 gear: 1
} """

ACTUAL_TRAJECTORY_DATA_NUM = 19
""" trajectory_point{
0 x: -0.00616207
1 y: -1.25169e-06
2 velocity: 0.00651749
3 heading: 0.00035111
4 relative_time: 1342.89
5 travel_distance: 0.514427
6 slope: 0
7 lateral_distance_error: 0
8 velocity_error: 0
9 heading_error: 0
10 computational_time: 0
11 average_lateral_distance_error: 0
12 average_velocity_error: 0
13 average_heading_error: 0
14 average_computational_time: 0
15 max_lateral_distance_error: 0
16 max_velocity_error: 0
17max_heading_error: 0
18 max_computational_time: 0
} """

def CheckHeadingAngle(actual_trajectory):
    length = actual_trajectory.shape[1]
    # last_theta = actual_trajectory[3,0]
    for i in range(length):
        if((abs(actual_trajectory[3,i]-actual_trajectory[3,i-1])>PI) and (abs(actual_trajectory[3,i]-actual_trajectory[3,i+1])>PI)):
            actual_trajectory[3,i]=actual_trajectory[3,i-1]
        # last_theta=actual_trajectory[3,i]



def read_file(file_name, data_num):
    result = np.zeros((data_num,1),dtype=float)
    with open(file_name, 'r') as fp:
        while True:
            s = fp.readline()
            if s == '':
                break
            if s[-2]=="{":
                temp = np.zeros((data_num,1),dtype=float)
                i=0
                while True:
                    s = fp.readline()
                    index = s.find(":")
                    if(index!=-1):
                        ss=float(s[index+2:-1:1])
                        temp[i,0]=ss
                        i +=1
                    if s[0]=="}":
                        break
                result = np.hstack([result,temp])
    return result

def CreateRoad(planned_trajectory,road_width):
    # rospy.loginfo("CreateRoad")
    length = planned_trajectory.shape[1]
    road= np.zeros((4,length),dtype=float)
    for i in range(length):
        point = planned_trajectory[:,i]
        angle_right = point[3]-PI/2
        angle_left = angle_right-PI
        road[0,i]=point[0]+road_width/2*math.cos(angle_left)
        road[1,i]=point[1]+road_width/2*math.sin(angle_left)
        road[2,i]=point[0]+road_width/2*math.cos(angle_right)
        road[3,i]=point[1]+road_width/2*math.sin(angle_right)
    return road


velocity_list = ["10","15","20"]

longitudinal_controller_name = 'Double PID Controller'

lateral_controller_list=["Stanley Controller","Pure Pursuit Controller","Rear Wheel Feedback Controller","LQR Controller","OLQR Controller","MPC Controller"]


# lateral_controller_name = "Stanley Controller"
# lateral_controller_name = 'Pure Pursuit Controller'
# lateral_controller_name = "Rear Wheel Feedback Controller"
# lateral_controller_name = "LQR Controller"
# lateral_controller_name = "OLQR Controller"
# lateral_controller_name = "MPC Controller"

loc='center'
font_dict={'fontsize': 20,\
        'fontweight' : 8.2,\
        'verticalalignment': 'baseline',\
        'horizontalalignment': loc}
start_position = 20
size_l = 19.2
size_w = 10.8
scale = 1.2


def PlotofController():
    for lateral_controller_name in lateral_controller_list:
        lateral_controller=lateral_controller_name.replace(' ','_')
        if(os.path.exists(lateral_controller)):
            shutil.rmtree(os.getcwd()+'/'+lateral_controller)
        os.mkdir(lateral_controller)
        for velocity in velocity_list:
            label1 = longitudinal_controller_name+" and "+lateral_controller_name+" Controller\ncurve velocity = "+velocity+"m/s"
            label2 = longitudinal_controller_name+" and "+lateral_controller_name+" Controller(Velocity Comparison With Station)"+"\n"+"curve velocity = "+velocity+"m/s"
            label3 = longitudinal_controller_name+" and "+lateral_controller_name+" Controller(Heading Comparison with Station)"+"\n"+"curve velocity = "+velocity+"m/s"
            label4 = longitudinal_controller_name+" and "+lateral_controller_name+" Controller(Lateral Distance Error)"+"\n"+"curve velocity = "+velocity+"m/s"
            label5 = longitudinal_controller_name+" and "+lateral_controller_name+" Controller(Velocity Error)"+"\n"+"curve velocity = "+velocity+"m/s"
            label6 = longitudinal_controller_name+" and "+lateral_controller_name+" Controller(Heading Error)"+"\n"+"curve velocity = "+velocity+"m/s"
            desired_trajectory = '/home/djh/ControlModule/src/plan/test_trajectories/mat_trajectory_'+velocity+'.txt'
            actual_trajectory = 'simulation_result/'+lateral_controller+'_'+velocity+'.txt'
            
            

            trajectory_info=read_file(desired_trajectory,DESIRED_TRAJECTORY_DATA_NUM)
            pose_info=read_file(actual_trajectory,ACTUAL_TRAJECTORY_DATA_NUM)
            pose_info[7]=pose_info[7]*100
            pose_info[11]=pose_info[11]*100
            pose_info[15]=pose_info[15]*100
            CheckHeadingAngle(pose_info)

            tra_xmin=min(trajectory_info[0])-50
            tra_xmax=max(trajectory_info[0])+50
            tra_ymin=min(trajectory_info[1])-30
            tra_ymax=max(trajectory_info[1])+70

            velocity_xmin=min(trajectory_info[5])-5
            velocity_xmax=max(trajectory_info[5])+5
            velocity_ymin=min(trajectory_info[7])*scale
            velocity_ymax=max(trajectory_info[7])*scale

            yaw_xmin=min(trajectory_info[5])-5
            yaw_xmax=max(trajectory_info[5])+5
            yaw_ymin=min(trajectory_info[3])*scale
            yaw_ymax=max(trajectory_info[3])*scale

            zoom = (tra_xmax-tra_xmin)/200
            road_width=3.75*zoom
            road = CreateRoad(trajectory_info,road_width)



            plt.figure(figsize=(size_l, size_w))
            plt.plot(trajectory_info[0,:],trajectory_info[1,:],"--",c="c",linewidth=2.0,label="planned trajectory")
            plt.plot(road[0,:],road[1,:],c="c",linewidth=3.0)
            plt.plot(road[2,:],road[3,:],c="c",linewidth=3.0)
            plt.plot(pose_info[0,1:-1],pose_info[1,1:-1],c="r",linewidth=2.0,label="actual trajectory")

            plt.title(label1,fontdict=font_dict,loc=loc)
            font_size = 12
            h_interval = (tra_xmax-tra_xmin)/5
            v_interval = (tra_ymax-tra_ymin)/30


            plt.text(tra_xmin+start_position, tra_ymax-2*v_interval, "velocity error = "+str(round(pose_info[8,-1],4))+" m/s",fontsize=font_size) 
            plt.text(tra_xmin+start_position, tra_ymax-3*v_interval, "distance error = "+str(round(pose_info[7,-1],4))+" cm",fontsize=font_size) 
            plt.text(tra_xmin+start_position, tra_ymax-4*v_interval, "heading error = "+str(round(pose_info[9,-1]*180/PI,4))+" deg",fontsize=font_size)
            plt.text(tra_xmin+start_position, tra_ymax-5*v_interval, "computational time = "+str(round(pose_info[10,-1],4))+" ms",fontsize=font_size) 
            plt.text(tra_xmin+start_position+2*h_interval, tra_ymax-2*v_interval, "average velocity error = "+str(round(pose_info[12,-1],4))+" m/s",fontsize=font_size) 
            plt.text(tra_xmin+start_position+2*h_interval, tra_ymax-3*v_interval, "average distance error = "+str(round(pose_info[11,-1],4))+" cm",fontsize=font_size) 
            plt.text(tra_xmin+start_position+2*h_interval, tra_ymax-4*v_interval, "average heading error = "+str(round(pose_info[13,-1]*180/PI,4))+" deg",fontsize=font_size) 
            plt.text(tra_xmin+start_position+2*h_interval, tra_ymax-5*v_interval, "average computational time = "+str(round(pose_info[14,-1],4))+" ms",fontsize=font_size) 
            plt.text(tra_xmin+start_position+h_interval, tra_ymax-2*v_interval, "max velocity error = "+str(round(pose_info[16,-1],4))+" m/s",fontsize=font_size) 
            plt.text(tra_xmin+start_position+h_interval, tra_ymax-3*v_interval, "max distance error = "+str(round(pose_info[15,-1],4))+" cm",fontsize=font_size)  
            plt.text(tra_xmin+start_position+h_interval, tra_ymax-4*v_interval, "max heading error = "+str(round(pose_info[17,-1]*180/PI,4))+" deg",fontsize=font_size) 
            plt.text(tra_xmin+start_position+h_interval, tra_ymax-5*v_interval, "max computational time = "+str(round(pose_info[18,-1],4))+" ms",fontsize=font_size)                 
            plt.legend()
            plt.xlabel('X/m')
            plt.ylabel('Y/m')
            axes = plt.gca()
            axes.set_xlim([tra_xmin,tra_xmax])
            axes.set_ylim([tra_ymin,tra_ymax])
            
            plt.savefig(lateral_controller+'/'+lateral_controller+'_'+velocity+'_Path.png')



            plt.figure(figsize=(size_l, size_w))
            plt.plot(trajectory_info[5,:],trajectory_info[7,:],"-",c="c",linewidth=2.0,label="planned velocity")
            plt.plot(pose_info[5,1:-1],pose_info[2,1:-1],c="r",linewidth=2.0,label="actual velocity")
            font_size = 13
            h_interval = (velocity_xmax-velocity_xmin)/5
            v_interval = (velocity_ymax-velocity_ymin)/30
            plt.text(velocity_xmin+start_position, velocity_ymax-v_interval, "velocity error = "+str(round(pose_info[8,-1],4))+" m/s",fontsize=font_size) 
            plt.text(velocity_xmin+start_position, velocity_ymax-2*v_interval, "average velocity error = "+str(round(pose_info[12,-1],4))+" m/s",fontsize=font_size)  
            plt.text(velocity_xmin+start_position, velocity_ymax-3*v_interval, "max velocity error = "+str(round(pose_info[16,-1],4))+" m/s",fontsize=font_size)  
            plt.title(label2,fontdict=font_dict,loc=loc)
            # plt.legend(loc=3)
            plt.legend()
            plt.grid()
            plt.xlabel('Traveled Distance/m')
            plt.ylabel('Velocity/m')
            ax1 = plt.gca()
            ax1.set_xlim([velocity_xmin,velocity_xmax])
            ax1.set_ylim([velocity_ymin,velocity_ymax])
            plt.savefig(lateral_controller+'/'+lateral_controller+'_'+velocity+'_Velocity.png')

            plt.figure(figsize=(size_l, size_w))
            plt.plot(trajectory_info[5,:],trajectory_info[3,:],".",c="c",linewidth=2.0,label="planned heading")
            plt.plot(pose_info[5,1:-1],pose_info[3,1:-1],".",c="r",linewidth=1.0,label="actual heading")
            font_size = 13
            h_interval = (yaw_xmax-yaw_xmin)/5
            v_interval = (yaw_ymax-yaw_ymin)/30
            plt.text(yaw_xmin+start_position, yaw_ymax-v_interval,"heading error = "+str(round(pose_info[9,-1]*180/PI,4))+" deg",fontsize=font_size)
            plt.text(yaw_xmin+start_position, yaw_ymax-2*v_interval, "average heading error = "+str(round(pose_info[13,-1]*180/PI,4))+" deg",fontsize=font_size) 
            plt.text(yaw_xmin+start_position, yaw_ymax-3*v_interval, "max heading error = "+str(round(pose_info[17,-1]*180/PI,4))+" deg",fontsize=font_size) 
            plt.title(label3,fontdict=font_dict,loc=loc)
            plt.legend()
            plt.xlabel('Traveled Distance/m')
            plt.ylabel('Heading/rad')
            axes = plt.gca()
            axes.set_xlim([yaw_xmin,yaw_xmax])
            axes.set_ylim([yaw_ymin,yaw_ymax])
            plt.savefig(lateral_controller+'/'+lateral_controller+'_'+velocity+'_Heading.png')


            xmin=min(trajectory_info[5])-5
            xmax=max(trajectory_info[5])+5


            plt.figure(figsize=(size_l, size_w))
            d_error_ymax=max(pose_info[7])*scale
            d_error_ymin=min(pose_info[7])*scale
            plt.plot(pose_info[5,1:-1],pose_info[7,1:-1],"-",c="r",linewidth=1.0,label="lateral distance error")
            plt.plot(pose_info[5,1:-1],pose_info[11,1:-1],"-",c="c",linewidth=1.0,label="average lateral distance error")
            plt.text(xmin+start_position, d_error_ymax*0.95,"max lateral distance error = "+str(round(pose_info[15,-1],4))+" cm",fontsize=font_size) 
            plt.title(label4,fontdict=font_dict,loc=loc)
            plt.legend()
            plt.grid()
            plt.xlabel('Traveled Distance/m')
            plt.ylabel('lateral distance error/cm')
            axes = plt.gca()
            axes.set_xlim([xmin,yaw_xmax])
            axes.set_ylim([d_error_ymin,d_error_ymax])
            plt.savefig(lateral_controller+'/'+lateral_controller+'_'+velocity+'_Lateral_Distance_Error.png')


            plt.figure(figsize=(size_l, size_w))
            v_error_ymax=max(pose_info[8])*scale
            v_error_ymin=min(pose_info[8])*scale
            plt.plot(pose_info[5,1:-1],pose_info[8,1:-1],"-",c="r",linewidth=1.0,label="velocity error")
            plt.plot(pose_info[5,1:-1],pose_info[12,1:-1],"-",c="c",linewidth=1.0,label="average velocity error")
            plt.text(xmin+start_position, v_error_ymax*0.95,"max velocity error = "+str(round(pose_info[16,-1],4))+" m/s",fontsize=font_size)

            plt.title(label5,fontdict=font_dict,loc=loc)
            plt.legend()
            plt.grid()
            plt.xlabel('Traveled Distance/m')
            plt.ylabel('velocity error/cm')
            axes = plt.gca()
            axes.set_xlim([xmin,yaw_xmax])
            axes.set_ylim([v_error_ymin,v_error_ymax])
            plt.savefig(lateral_controller+'/'+lateral_controller+'_'+velocity+'_Velocity_Error.png')


            plt.figure(figsize=(size_l, size_w))
            h_error_ymax=max(pose_info[9])*scale
            h_error_ymin=min(pose_info[9])*scale
            plt.plot(pose_info[5,1:-1],pose_info[9,1:-1],"-",c="r",linewidth=1.0,label="heading error")
            plt.plot(pose_info[5,1:-1],pose_info[13,1:-1],"-",c="c",linewidth=1.0,label="average heading error")
            plt.text(xmin+start_position, v_error_ymax*0.95,"max heading error = "+str(round(pose_info[17,-1]*180/PI,4))+" deg",fontsize=font_size) 
            plt.title(label6,fontdict=font_dict,loc=loc)
            plt.legend()
            plt.grid()
            plt.xlabel('Traveled Distance/m')
            plt.ylabel('heading error/cm')
            axes = plt.gca()
            axes.set_xlim([xmin,yaw_xmax])
            axes.set_ylim([h_error_ymin,h_error_ymax])
            plt.savefig(lateral_controller+'/'+lateral_controller+'_'+velocity+'_Heading_Error.png')


            # plt.show()



def PlotofComaprison():  
    if(os.path.exists("Error_Comparison")):
        shutil.rmtree(os.getcwd()+'/Error_Comparison')
    os.mkdir("Error_Comparison")
    for velocity in velocity_list:
        plot_title1 = "Velocity Error Comparison when V="+velocity+"m/s"
        plot_title2 = "Lateral Distance Error Comparison when V="+velocity+"m/s"
        plot_title3 = "Heading Error Comparison when V="+velocity+"m/s"

        plot_title4 = "Average Velocity Error Comparison when V="+velocity+"m/s"
        plot_title5 = "Average Lateral Distance Error Comparison when V="+velocity+"m/s"
        plot_title6 = "Average Heading Error Comparison when V="+velocity+"m/s"
        folder_name = "Error_Comparison/V="+velocity
        if(os.path.exists(folder_name)):
            shutil.rmtree(os.getcwd()+'/'+folder_name)
        os.mkdir(folder_name)

        plt.figure(figsize=(size_l, size_w))
        for lateral_controller_name in lateral_controller_list:
            lateral_controller=lateral_controller_name.replace(' ','_')
            
            actual_trajectory = 'simulation_result/'+lateral_controller+'_'+velocity+'.txt'

            pose_info=read_file(actual_trajectory,ACTUAL_TRAJECTORY_DATA_NUM)


            plt.plot(pose_info[5,1:-1],pose_info[8,1:-1],"-",linewidth=1.0,label=lateral_controller)

            plt.title(plot_title1,fontdict=font_dict,loc=loc)
            plt.legend(loc=2)
            plt.grid()
            plt.xlabel('Traveled Distance/m')
            plt.ylabel('velocity error/cm')
            axes = plt.gca()
            plt.savefig(folder_name+'/'+"Velocity_Error_Comparison_when_V="+velocity+".png")

        plt.figure(figsize=(size_l, size_w))
        for lateral_controller_name in lateral_controller_list:
            lateral_controller=lateral_controller_name.replace(' ','_')
            
            actual_trajectory = 'simulation_result/'+lateral_controller+'_'+velocity+'.txt'

            pose_info=read_file(actual_trajectory,ACTUAL_TRAJECTORY_DATA_NUM)

            pose_info[7]=pose_info[7]*100
            pose_info[11]=pose_info[11]*100
            pose_info[15]=pose_info[15]*100
            CheckHeadingAngle(pose_info)

            plt.plot(pose_info[5,1:-1],pose_info[7,1:-1],"-",linewidth=1.0,label=lateral_controller)

            plt.title(plot_title2,fontdict=font_dict,loc=loc)
            plt.legend(loc=2)
            plt.grid()
            plt.xlabel('Traveled Distance/m')
            plt.ylabel('lateral distance error/cm')
            axes = plt.gca()
            plt.savefig(folder_name+'/'+"Lateral_Distance_Error_Comparison_when_V="+velocity+".png")

        plt.figure(figsize=(size_l, size_w))
        for lateral_controller_name in lateral_controller_list:
            lateral_controller=lateral_controller_name.replace(' ','_')
            
            actual_trajectory = 'simulation_result/'+lateral_controller+'_'+velocity+'.txt'

            pose_info=read_file(actual_trajectory,ACTUAL_TRAJECTORY_DATA_NUM)

            CheckHeadingAngle(pose_info)

            plt.plot(pose_info[5,1:-1],pose_info[9,1:-1],"-",linewidth=1.0,label=lateral_controller)

            plt.title(plot_title3,fontdict=font_dict,loc=loc)
            plt.legend(loc=2)
            plt.grid()
            plt.xlabel('Traveled Distance/m')
            plt.ylabel('lateral distance error/cm')
            axes = plt.gca()
            plt.savefig(folder_name+'/'+"Heading_Error_Comparison_when_V="+velocity+".png")

# PlotofController()
PlotofComaprison()