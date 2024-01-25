#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CompressedImage


class IMGParser:
    def __init__(self):
        rospy.init_node('camera', anonymous=True)
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)
        rospy.spin()


    def grayscale(self, img): # 흑백이미지로 변환
        return cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)


    def canny(self, img, low_threshold, high_threshold): # Canny 알고리즘
        return cv2.Canny(img, low_threshold, high_threshold)


    def gaussian_blur(self, img, kernel_size): # 가우시안 필터
        return cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)


    def region_of_interest(self, img, vertices, color3=(255,255,255), color1=255): # ROI 셋팅
        mask = np.zeros_like(img) # mask = img와 같은 크기의 빈 이미지
    
        if len(img.shape) > 2: # Color 이미지(3채널)라면 :
            color = color3
        else: # 흑백 이미지(1채널)라면 :
            color = color1
            
        # vertices에 정한 점들로 이뤄진 다각형부분(ROI 설정부분)을 color로 채움 
        cv2.fillPoly(mask, vertices, color)
        
        # 이미지와 color로 채워진 ROI를 합침
        ROI_image = cv2.bitwise_and(img, mask)
        return ROI_image


    def draw_lines(self, img, lines, color=[0, 0, 255], thickness=2): # 선 그리기
        if lines is not None:
            for line in lines:
                for x1,y1,x2,y2 in line:
                    cv2.line(img, (x1, y1), (x2, y2), color, thickness)


    def hough_lines(self, img, rho, theta, threshold, min_line_len, max_line_gap): # 허프 변환
        lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)
        line_img = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)
        self.draw_lines(line_img, lines)

        return line_img 
    
    
    def weighted_img(self, img, initial_img, α=1, β=1., λ=0.): # 두 이미지 operlap 하기
        return cv2.addWeighted(initial_img, α, img, β, λ)
    
    
    def callback(self, data):
        np_arr = np.fromstring(data.data, np.uint8)
        img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        
        height, width = img_bgr.shape[:2] # 이미지 높이, 너비
        
        
        gray_img = self.grayscale(img_bgr)
        blur_img = self.gaussian_blur(gray_img, 3)
        canny_img = self.canny(blur_img, 70, 210)
        
        
        vertices = np.array([[(60, height),(0, 255), (width, 255), (width, height)]], dtype=np.int32)    #범위값 설정
        ROI_img = self.region_of_interest(canny_img, vertices) # ROI 설정
        
        
        hough_img = self.hough_lines(ROI_img, 1, 1 * np.pi/180, 30, 10, 20) # 허프 변환
        
        
        result_img = self.weighted_img(hough_img, img_bgr)
        
        
        cv2.imshow("Image window", result_img)
        cv2.imshow("hough window", hough_img)
        cv2.waitKey(1)


if __name__ == '__main__':
    try:
        image_parser = IMGParser()
    except rospy.ROSInterruptException:
        pass