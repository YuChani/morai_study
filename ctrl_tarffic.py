#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
from morai_msgs.msg  import SetTrafficLight, Traffic_status ,CtrlCmd

    
class erp_planner():
    def __init__(self):
        rospy.init_node("light_status", anonymous = 1)
        rospy.Subscriber("/traffic_status", Traffic_status, self.tf_light_callback)
        self.ctrl_pub = rospy.Publisher('/ctrl_cmd',CtrlCmd, queue_size=1)
        self.tf_light_msg = Traffic_status() 
        rospy.spin()
        
    
    def tf_light_callback(self, msg):
        self.is_light = True
        self.tf_light_msg = msg
        

    def callback(self):
        
        ctrl_msg= CtrlCmd()
        
        print(self.tf_light_msg.traffic)
            
        if self.tf_light_msg.traffic == "RED":
            ctrl_msg.accel = 0
            ctrl_msg.brake = 1            

                    
        elif self.tf_light_msg.traffic == "GREEN":
            ctrl_msg.accel = 1
            ctrl_msg.brake = 0
            
        else :
            ctrl_msg.accel = 1
            ctrl_msg.brake = 0
        
        
        self.print_info()
        self.ctrl_pub.publish(ctrl_msg)

        


if __name__ == '__main__':
    try:
        kcity_pathtracking=erp_planner()
    except rospy.ROSInterruptException:
        pass