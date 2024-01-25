#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from morai_msgs.msg  import Traffic_status, CtrlCmd, GetTrafficLightStatus


    
class erp_planner():  
    def __init__(self):
        rospy.init_node("light_status", anonymous = 1)
        rospy.Subscriber("/GetTrafficLightStatus", GetTrafficLightStatus, self.light_callback)
        ctrl_pub = rospy.Publisher('/ctrl_cmd',CtrlCmd, queue_size=1)
        
        ctrl_msg = CtrlCmd()
        

        if self.tf_light.trafficLightStatus == "2":
            ctrl_msg.accel = 0
            ctrl_msg.brake = 1            

                        
        elif self.tf_light.trafficLightStatus == "16":
            ctrl_msg.accel = 1
            ctrl_msg.brake = 0
            
        ctrl_pub.publish(ctrl_msg)
        
        
    def light_callback(self, msg):
        self.is_light = True
        self.tf_light = GetTrafficLightStatus()
        self.tf_light = msg
        print(self.tf_light)
        print("------------------------")
         
        


if __name__ == '__main__':
    try:
        kcity_pathtracking=erp_planner()
    except rospy.ROSInterruptException:
        pass