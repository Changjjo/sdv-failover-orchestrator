#!/usr/bin/env python3

import cv2
import numpy as np
import keyboard
import math

# 이미지 파일 로드
background = cv2.imread('/home/cluster_ws/src/image/Slide1.jpg')  # 제공된 배경 이미지

# 속도계와 RPM 다이얼 중심 좌표 설정 (수동으로)
rpm_center = (290, 250)  # RPM 다이얼 중심 좌표 (이미지 내)
speed_center = (1000, 250)  # 속도 다이얼 중심 좌표 (이미지 내)

# 속도와 RPM 초기값 설정
speed = 0
rpm = 0

# 빨간 침을 그리는 함수
def draw_needle(image, center, angle, length, color=(0, 0, 255), thickness=4):
    # 각도를 라디안으로 변환
    radian = math.radians(angle)
    
    # 침 끝 좌표 계산
    x_end = int(center[0] + length * math.cos(radian))
    y_end = int(center[1] - length * math.sin(radian))
    
    # 침 그리기
    cv2.line(image, center, (x_end, y_end), color, thickness)

# 루프 시작
while True:
    # 키 입력 처리
    if keyboard.is_pressed("w"):  # 'w' 키로 속도 증가
        speed = min(speed + 2, 80)
        rpm = min(rpm + 100, 2500)
    elif keyboard.is_pressed("s"):  # 's' 키로 속도 감소
        speed = max(speed - 2, 0)
        rpm = max(rpm - 100, 0)
    else:
   	    speed = max(speed - 0.5, 0)
   	    rpm = max(rpm-10,0)
    	
        
        
        
    # 배경 복사
    frame = background.copy()

    # RPM과 속도에 따라 각도 계산
    rpm_angle = -135 - (rpm / 8000) * 270  # 0~8000 RPM -> -135도~135도
    speed_angle = -145 - (speed / 260) * 250  # 0~260 속도 -> -135도~135도

    #print(rpm_angle, speed_angle)
    
    # 빨간색 침 그리기
    draw_needle(frame, rpm_center, rpm_angle, 125, color=(0, 0, 255), thickness=5)  # RPM 침
    draw_needle(frame, speed_center, speed_angle, 125, color=(0, 0, 255), thickness=5)  # 속도계 침

    # 프레임 출력
    cv2.imshow("Cluster System", frame)

    # ESC 키를 누르면 종료
    if cv2.waitKey(1) & 0xFF == 27:
        break

# 모든 창 닫기
cv2.destroyAllWindows()

