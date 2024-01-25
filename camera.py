#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import math
import matplotlib.pyplot as plt
from sensor_msgs.msg import Image, CompressedImage

class IMGParser:
    def __init__(self):
        rospy.init_node('camera', anonymous=True)   
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)
        rospy.spin()

    def grayscale(self, img):   #gray 이미지 변환
        return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    def gaussian_blur(self, img, kernel_size):  #gaussian blur 필터
        return cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)
    
    def canny(self, img, low_threshold, high_threshold):    #canny 알고리즘
        return cv2.Canny(img, low_threshold, high_threshold)
    
    def region_of_interest(self, img, vertices, color3 = (255, 255, 255), color1 = 255):
        mask = np.zeros_like(img)   #mask = img와 같은 크기의 빈 이미지
        
        if len(img.shape) > 2:  #color 이미지 (3채널)
            color = color3
        else :  #흑백 이미지 화면
            color = color1
        
        #vertices에 정한 점들로 이루어진 다각형 부분(roi설정부분)을 컬러로 채움
        cv2.fillPoly(mask, vertices, color)
        #이미지와 color로 채워진 roi 합침
        ROI_img = cv2.bitwise_and(img, mask)
        
        return ROI_img
    
    def draw_lines(self, img, lines, color = [0, 0, 255]):
        if lines is not None:
            for line in lines:
                for x1, y1, x2, y2 in line:
                    cv2.line(img, (x1,y1), (x2,y2), color, thickness=2)
                    
    def draw_fit_line(self, img, lines, color = [255, 0, 0], thickness = 10): #대표선 그리기
        cv2.line(img, (lines[0], lines[1]), (lines[2], lines[3]), color, thickness=thickness)
    
    def hough_lines(self, img, rho, theta, threshold, min_line_len, max_line_gap):
        lines = cv2.HoughLinesP(img, 
                                rho , 
                                theta, 
                                threshold,
                                np.array([]),
                                minLineLength = min_line_len,
                                maxLineGap = max_line_gap)
        
        #line_img = np.zeros((img.shape[0], img.shape[1], 3), dtype = np.uint8)
        #self.draw_lines(line_img, lines)
        return lines
    
    def weighted_img(self, img, initial_img, α=1, β=1., λ=0.):    #두 이미지 오버랩 하기
        return cv2.addWeighted(initial_img, α, img, β, λ)
    
    def get_fitline(self, img, f_lines):    #대표선 구하기
        lines = np.squeeze(f_lines)
        lines = lines.reshape(lines.shape[0] * 2, 2)
        rows, cols = img.shape[:2]
        output = cv2.fitLine(lines, cv2.DIST_L2, 0, 0.01, 0.01)
        vx, vy, x, y = output[0], output[1], output[2], output[3]
        x1, y1 = int(((img.shape[0] - 1) - y) / vy * vx + x), img.shape[0] - 1
        x2, y2 = int(((img.shape[0] / 2 + 100) - y) / vy * vx + x), int(img.shape[0] / 2 + 100)
        
        result = [x1, y1, x2, y2]
        return result
    
    def callback(self, data):
        np_arr = np.fromstring(data.data, np.uint8)
        img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        height, width = img_bgr.shape[:2]   #d이미지의 너비, 높이
        
        gray_img = self.grayscale(img_bgr)  #gray 이미지 변환
        
        blur_img = self.gaussian_blur(gray_img, 3)  #blur 효과
        
        canny_img = self.canny(blur_img, 70, 210)   #canny edge 알고리즘
        
        #vertices = np.array([[(50,height),(width/2-45, height/2+60), (width/2+45, height/2+60), (width-50,height)]], dtype=np.int32)
        vertices = np.array([[(60, height),(0, 255), (width, 255), (width, height)]], dtype = np.int32)    #범위값 설정
        ROI_img = self.region_of_interest(canny_img, vertices)  #ROI 설정
        
        
        #hough = self.hough_lines(ROI_img, 1, 1 * np.pi/180, 30, 10, 20)   #허프 변환
        line_arr = self.hough_lines(ROI_img, 1, 1 * np.pi/180, 30, 10, 20)   #허프 변환
        line_arr = np.squeeze(line_arr)
        
        #기울기 구하기
        slope_degree = (np.arctan2(line_arr[:, 1] - line_arr[:, 3], line_arr[:, 0] - line_arr[:, 2]) * 180) / np.pi
        
        #수평 기울기 제한
        line_arr = line_arr[np.abs(slope_degree) < 160]
        slope_degree = slope_degree[np.abs(slope_degree) < 160]
        
        #수직 기울기 제한
        line_arr = line_arr[np.abs(slope_degree) > 95]
        slope_degree = slope_degree[np.abs(slope_degree) > 95]
        
        #필터링된 직선 버리기
        L_lines, R_lines = line_arr[(slope_degree > 0), :], line_arr[(slope_degree < 0), :]
        temp = np.zeros((img_bgr.shape[0], img_bgr.shape[1], 3), dtype = np.uint8)
        L_lines, R_lines = L_lines[:, None], R_lines[:, None]
        
        #왼쪽, 오른쪽 각각 대표선 구하기
        left_fit_line = self.get_fitline(img_bgr,L_lines)
        right_fit_line = self.get_fitline(img_bgr, R_lines)
        
        #대표선 그리기
        self.draw_fit_line(temp, left_fit_line)
        self.draw_fit_line(temp, right_fit_line)
        
        result = self.weighted_img(temp, img_bgr)  #원본 이미지에 검출된 선 overlap
        
        #cv2.imshow("Image window", img_bgr) #원본 카메라 이미지
        #cv2.imshow("Hough window", line_arr) #hough 이미지
        cv2.imshow("Result window", result) #원본 + hough 이미지
        
        cv2.waitKey(1)

if __name__ == '__main__':
    try:
        image_parser = IMGParser()
    except rospy.ROSInterruptException:
        pass