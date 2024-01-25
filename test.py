#!/usr/bin/env python3

import cv2
import math
import cv2 as cv
import numpy as np

cap = cv2.VideoCapture(1) # cv2.VideoCapture(0)이면 실시간 웹캠 영상 받아옴

def roi_image(image):
    (x,y),(w,h) = (100,200),(400,500) #(시작 좌표), (지정하고자하는 폭과 넓이)
    roi_img = image[y:y+h, x:x+w]

    return roi_img

def white_filter(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    white_lower = np.array([0, 0, 200]) # V로 밝기 조절
    white_upper = np.array([90, 80, 255])

    white_mask = cv2.inRange(hsv, white_lower, white_upper)

    white_masked = cv2.bitwise_and(image, image, mask=white_mask)

    return white_masked


while (True):
    ret, src = cap.read() # 1. 이미지 받아옴

    src=roi_image(src) # 2. 이미지 크기, 범위 조절

    white_src = white_filter(src) # 3. 흰색만 검출

    white_dst = cv.Canny(white_src, 300, 600, None, 3) # 4. 겉테두리만 땀

    white_cdst = cv.cvtColor(white_dst, cv.COLOR_GRAY2BGR) # 5. 테두리 딴 영상을 BGR 로 바꿈 (전처리 완료)
    white_cdstP = np.copy(white_cdst) # 6. 5번 영상 복사

    stop_lines = cv.HoughLines(white_dst, 1, 5*np.pi / 180, 150, None, 0, 0, 88, 93) # 7. 테두리 딴 영상으로 "직선 검출" (88도부터 93도까지 5도 단위로 직선을 찾게 각도 조정)

    if stop_lines is not None:
        for i in range(0, len(stop_lines)):
            rho = stop_lines[0][0][0] # 직선과의 거리
            theta = stop_lines[0][0][1] # 직선과의 각도
            a = math.cos(theta)
            b = math.sin(theta)
            x0 = a * rho
            y0 = b * rho
            pt1 = (int(x0 + 1000 * (-b)), int(y0 + 1000 * (a))) # 시작점
            pt2 = (int(x0 - 1000 * (-b)), int(y0 - 1000 * (a))) # 끝점
            cv.line(white_cdst, pt1, pt2, (30, 225, 225), 2, cv.LINE_AA) #해당 직선을 그림

    stop_linesP = cv.HoughLinesP(white_dst, 1, 15*np.pi /180, 2, None, 150, 1) #minLineLength 값 사용해서 인식되는 선의 최소 길이 지정 // 수직 수평만 인식되게 각도 조정 (선분 검출)

    if stop_linesP is not None:
        for i in range(0, len(stop_linesP)):
            l = stop_linesP[i][0]
            cv.line(white_cdstP, (l[0], l[1]), (l[2], l[3]), (0, 0, 255), 2, cv.LINE_AA)



    cv.imshow("Source", src)
    cv.imshow("Stop Lines (in red) - Standard Hough Line Transform", white_cdst)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release(10)
cv2.destroyAllWindows()