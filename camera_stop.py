#!/usr/bin/env python3
  
import rospy
import cv2
import numpy as np
import math
from sensor_msgs.msg import Image, CompressedImage


class IMGParser:
    def __init__(self):
        rospy.init_node('camera', anonymous=True)
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)
        rospy.spin()

    def grayscale(self, img): # 흑백이미지로 변환
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)


    def canny(self, img, low_threshold, high_threshold, edges, num): # Canny 알고리즘
        return cv2.Canny(img, low_threshold, high_threshold, edges, num)


    def gaussian_blur(self, img, kernel_size): # 가우시안 필터
        return cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)

    def stop_line_roi(self, img): # 신호등 roi 값
        x, y, w, h = 250, 280, 470, 440   
        stop_roi = img[y:h, x:w]
        return stop_roi 

     
    def white_filter(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        white_lower = np.array([0, 0, 200]) # V로 밝기 조절
        white_upper = np.array([90, 80, 255])

        white_mask = cv2.inRange(hsv, white_lower, white_upper)

        white_masked = cv2.bitwise_and(image, image, mask=white_mask)

        return white_masked
    
    
    def draw_red_lines(self, image, lines):
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(image, (x1, y1), (x2, y2), (0, 0, 255), 2)  # 빨간색으로 선 그리기

    

    def callback(self, data):
        np_arr = np.fromstring(data.data, np.uint8)
        img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    
        src = self.stop_line_roi(img_bgr)
        
        dy = cv2.Sobel(src, cv2.CV_64F, 0, 1, 2)
        dy_abs = np.abs(dy)
        dy_abs = np.clip(dy_abs, 0, 255).astype(np.uint8)
        
    
        white_src = self.white_filter(dy_abs)  # 흰색 검출
        white_gaussian = self.gaussian_blur(white_src, 3)
        white_dst = self.canny(white_gaussian, 300, 600, None, 3)  # 겉테두리만 검출
        white_cdst = self.grayscale(white_dst)  # 그레이 변경
        white_cdstP = np.copy(white_cdst)  # 그레이 영상 복사
    
        stop_lines = cv2.HoughLines(white_dst, 1, 5 * np.pi / 180, 150, None, 0, 0, 88, 93)

        if stop_lines is not None:
            stop_lines_array = stop_lines[:, 0, :]  # 선의 정보를 배열로 변환
            self.draw_red_lines(white_cdst, stop_lines_array)  # 검출된 정지선에 빨간색으로 표시

        stop_linesP = cv2.HoughLinesP(white_dst, 1, 15 * np.pi / 180, 2, None, 150, 1)  # 선분 검출
       
        if stop_linesP is not None:
            for i in range(0, len(stop_linesP)):
                l = stop_linesP[i][0]
                cv2.line(white_cdstP, (l[0], l[1]), (l[2], l[3]), (0, 0, 255), 2, cv2.LINE_AA)

        resize_stop = cv2.resize(white_cdst, (720, 480))  # tf_roi 이미지 확대

        cv2.imshow("Image window", img_bgr)
        cv2.imshow("Stop Lines (in red) - Standard Hough Line Transform", resize_stop)

        cv2.waitKey(1)

if __name__ == '__main__':
    try:
        image_parser = IMGParser()
    except rospy.ROSInterruptException:
        pass