#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys, os 
import rospy
import rospkg 
import numpy as np 
from nav_msgs.msg import Path,Odometry 
from std_msgs.msg import Float64,Int16,Float64MultiArray,String 
from geometry_msgs.msg import PoseStamped,Point 
from morai_msgs.msg import EgoVehicleStatus,CtrlCmd 
from lib.utils import pathReader,findLocalPath,purePursuit,cruiseControl,vaildObject,pidController,velocityPlanning,latticePlanner
import tf
from math import cos,sin,sqrt,pow,atan2,pi 

class erp_planner() : 
    def __init__(self) :  
        rospy.init_node('Police_plan', anonymous=True)
        
        arg = rospy.myargv(argv=sys.argv)
        self.path_name = arg[1]
        
        global_path_pub = rospy.Publisher('/global_path', Path , queue_size= 1 )
        local_path_pub = rospy.Publisher('/local_path',Path, queue_size = 1)
        ctrl_pub = rospy.Publisher('/ctrl_cmd',CtrlCmd,queue_size=1)
        
        # lattice
        for i in range(1,8) : 
            globals()['lattice_path_{}_pub'.format(i)] = rospy.Publisher('lattice_path_{}'.format(i),Path,queue_size=1)
            
        ctrl_msg = CtrlCmd() 
        
        # sub
        rospy.Subscriber("/Ego_topic" , EgoVehicleStatus, self.statusCB) 
     
     
        # def  
        self.is_status = False 
        
        #class 
        
        path_reader = pathReader('erp_ros')
        pure_pursuit = purePursuit()
        self.cc = cruiseControl(0.5,1)
        pid = pidController()
        
        #reading_path 
        self.global_path = path_reader.read_txt(self.path_name+".txt")
        
        self.status_msg = EgoVehicleStatus()
        
        vel_planner = velocityPlanning(60/3.6,1.5)
        vel_profile = vel_planner.curveBasedVelocity(self.global_path,100)
        
        #time var 
        count = 0 
        rate = rospy.Rate(30) # 30hz 
        
        lattice_current_lane = 3 
        
        while not rospy.is_shutdown() : 
            
            if self.is_status == True : 
                local_path,self.current_waypoint = findLocalPath(self.global_path,self.status_msg)
                
                ##latice##
                a = []
                vehicle_status = [self.status_msg.position.x,self.status_msg.position.y,(self.status_msg.heading)/180*pi,self.status_msg.velocity.x/3.6]
                lattice_path,selected_lane=latticePlanner(local_path,a,vehicle_status,lattice_current_lane)
                lattice_current_lane=selected_lane
                
                if selected_lane != -1 : 
                    local_path = lattice_path[selected_lane]
                    
                if len(lattice_path) == 7 : 
                    for i in range(1,8) : 
                        globals()['lattice_path_{}_pub'.format(i)].publish(lattice_path[i-1])
            
            
                pure_pursuit.getPath(local_path)
                pure_pursuit.getEgoStatus(self.status_msg)
                # stanely_control.getPath(local_path)
                # stanely_control.getEgostatus(self.status_msg)
                
                
                
                # steering_stanely = stanely_control.stanely()
                ctrl_msg.steering = -pure_pursuit.steering_angle() / 180 * pi 
                # print("steering_stanely = ",stanely_control)
                print("steering = ",ctrl_msg.steering)
                cc_vel = self.cc.acc(0,self.status_msg.velocity.x,vel_profile[self.current_waypoint],self.status_msg)
                
                target_velocity = cc_vel 
                ctrl_msg.velocity = cc_vel 
                
                control_input= pid.pid(target_velocity,self.status_msg.velocity)
                
                if control_input > 0 : 
                    ctrl_msg.accel = control_input
                    ctrl_msg.brake = 0 
                else :
                    ctrl_msg.accel = 0 
                    ctrl_msg.brake = - control_input 
                
                if self.status_msg.velocity.x <3.0 and target_velocity <= 0.0 : 
                    ctrl_msg.accel = 0 
                    ctrl_msg.brake = 1 
                    
                local_path_pub.publish(local_path)
                ctrl_pub.publish(ctrl_msg) 
                
                self.steering_angle = ctrl_msg.steering
        
            if count/300==1 :
                global_path_pub.publish(self.global_path)
                count = 0 
            count +=  1 
            
            rate.sleep()
        

    def print_info(self):

        os.system('clear')
        print('--------------------status-------------------------')
        print('position :{0} ,{1}, {2}'.format(self.status_msg.position.x,self.status_msg.position.y,self.status_msg.position.z))
        print('velocity :{} km/h'.format(self.status_msg.velocity))
        print('heading :{} deg'.format(self.status_msg.heading))

        print('--------------------controller-------------------------')
        print('target steering_angle :{} deg'.format(self.steering_angle))

        print('--------------------localization-------------------------')
        print('all waypoint size: {} '.format(len(self.global_path.poses)))
        print('current waypoint : {} '.format(self.current_waypoint))

    def statusCB(self,data) : 
        self.status_msg = EgoVehicleStatus()
        self.status_msg = data 
        br = tf.TransformBroadcaster()
        br.sendTransform((self.status_msg.position.x, self.status_msg.position.y, self.status_msg.position.z),
                         tf.transformations.quaternion_from_euler(0, 0, self.status_msg.heading/180*pi),
                        rospy.Time.now(),
                        "gps",
                        "map")
        self.is_status = True 
        
if __name__ == '__main__' : 
    try:
        kcity_pathtracking = erp_planner()
    except rospy.ROSInternalException:
        pass 