#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
from morai_msgs.msg  import TrafficLight, Traffic_status
from detection_msgs.msg import BoundingBox, BoundingBoxes
from utils.torch_utils import select_device


class IMGParser:
    def __init__(self): 
        rospy.init_node('camera', anonymous=True)
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)
        self.taffic_pub = rospy.Publisher("traffic_status", Traffic_status, queue_size = 1)
        self.bound_sub = rospy.Subscriber("/yolov5/detections", BoundingBox, self.callback)
        
        rospy.spin()
        
    
    def traffic_roi(self, img): # 신호등 roi 값
        x, y, w, h = 350, 140, 430, 180    
        roi = img[y:h, x:w]
        return roi
    
    
    def hsvExtraction(self, img, hsvLower, hsvUpper):   # hsv로 특정 색 추출
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        hsv_mask = cv2.inRange(hsv, hsvLower, hsvUpper)
        result = cv2.bitwise_and(img, img, mask = hsv_mask)  
        return result
    
        
    def traffic_color_name(self, img):
        traffic_msg = Traffic_status()
        
        # 빨강, 초록, 노록 범위 설정
        red_lower = np.array([0, 100, 100])
        red_upper = np.array([10, 255, 255])

        green_lower = np.array([35, 100, 100])
        green_upper = np.array([85, 255, 255])

        yellow_lower = np.array([20, 100, 100])
        yellow_upper = np.array([30, 255, 255])

        # HSV 변환
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    
        red_mask = cv2.inRange(img_hsv, red_lower, red_upper)
        green_mask = cv2.inRange(img_hsv, green_lower, green_upper)
        yellow_mask = cv2.inRange(img_hsv, yellow_lower, yellow_upper)
        

        # mask 에서 0이 아닌 픽셀 수를 계산
        red_pixel_count = cv2.countNonZero(red_mask)
        green_pixel_count = cv2.countNonZero(green_mask)
        yellow_pixel_count = cv2.countNonZero(yellow_mask)


        # 색상 범위값 if문으로 설정
        if red_pixel_count > green_pixel_count and red_pixel_count > yellow_pixel_count:
            print("Red")
            print("STOP")
            traffic_msg.traffic = "Red"
            traffic_msg.traffic_status = "STOP"
            
        elif green_pixel_count > red_pixel_count and green_pixel_count > yellow_pixel_count:
            print("Green")
            print("GO")
            traffic_msg.traffic = "Green"
            traffic_msg.traffic_status = "GO"

        elif yellow_pixel_count > red_pixel_count and yellow_pixel_count > green_pixel_count:
            print("Yellow")
            print("SLOW")
            traffic_msg.traffic = "Yellow"
            traffic_msg.traffic_status = "SLOW"
                  
        else:
            print("Unknown")
            traffic_msg.traffic = "Unknown"
            
        if yellow_pixel_count > red_pixel_count and green_pixel_count > red_pixel_count:
            print("Yellow and Green")
            traffic_msg.traffic = "Yellow and Green"
            
        self.taffic_pub.publish(traffic_msg)


    
    def callback(self, data):      
        np_arr = np.fromstring(data.data, np.uint8)
        img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

   
        # 빨간색 신호등 hsv 색 추출
        red_Lower = np.array([-10, 200, 100])  # 빨간색 검출 최솟값
        red_Upper = np.array([180, 255, 255])  # 빨간색 검출 최댓값 
        red_hsv = self.hsvExtraction(img_bgr, red_Lower, red_Upper)
    
        
        # 초록색 신호등 hsv 색 추출
        green_Lower = np.array([50, 150, 150])  # 초록색 검출 최솟값
        green_Upper = np.array([110, 255, 255])  # 초록색 검출 최댓값 
        green_hsv = self.hsvExtraction(img_bgr, green_Lower, green_Upper)   # 초록색 hsv 변환
        
        
        # 노란색 신호등 hsv 색 추출
        yellow_Lower = np.array([30, 20, 100])  # 노란색 검출 최솟값
        yellow_Upper = np.array([32, 255, 255])  # 노란색 검출 최댓값 
        yellow_hsv = self.hsvExtraction(img_bgr, yellow_Lower, yellow_Upper)    # 노란색 hsv 변환
        
        color_hsv = red_hsv + green_hsv + yellow_hsv
        
        
        tf_roi = self.traffic_roi(color_hsv)   # 빨간색 + 초록색 + 노란색 신호등 검출
        resize_tf = cv2.resize(tf_roi, (720, 480))  # tf_roi 이미지 확대   
        self.traffic_color = self.traffic_color_name(resize_tf) # rezise_tf 에서 색 탐지
        
             
        cv2.imshow("Image window", img_bgr)  # 원본
        cv2.imshow('Traffic window', resize_tf) # 신호등
        
        
        cv2.waitKey(1)


if __name__ == '__main__':
    try:
        image_parser = IMGParser()
    except rospy.ROSInterruptException:
        pass