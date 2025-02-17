#!/usr/bin/env python3

import cv2
import numpy as np
import Hobot.GPIO as GPIO  # 用于GPIO控制
from hobot_vio import libsrcampy as srcampy  # MIPI相机库
import time

import sys
import os

# 导入python串口库
import serial
import serial.tools.list_ports

# 定义颜色范围（HSV空间），假设红色和蓝色是要检测的颜色
lower_red = np.array([0, 120, 70])
upper_red = np.array([10, 255, 255])

lower_blue = np.array([100, 150, 0])
upper_blue = np.array([140, 255, 255])

lower_black = np.array([0, 0, 0])
upper_black = np.array([180, 255, 50])

height, width = 1080,1920

# 初始化MIPI相机
capture = cv2.VideoCapture(8)  # 常见的索引是 0 或 1

# 设置视频编码格式为MJPG
capture.set(6, cv2.VideoWriter_fourcc('M','J','P','G'))
capture.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

ret, frame = capture.read()
# 跳过一些缓冲区中的帧
for i in range(15):
    ret, frame = capture.read()
    if not ret:
        print("Failed to capture image")
        break


#串口发信息
def serialsend(info):
    uart_dev= "/dev/ttyS3"
    baudrate = 9600
    ser = serial.Serial(uart_dev, int(baudrate), timeout=1) # 1s timeout

    test_data = info
    #write_num = ser.write(test_data.encode('UTF-8'))
    write_num = ser.write(info)
    print("Send: ", test_data)
    #received_data = ser.read(write_num).decode('UTF-8')
    #print("Recv: ", received_data)
    #time.sleep(1)

    ser.close()
    return 0



# 定义小车控制函数
def control_car(direction):
    """
    根据球的位置发送相应的控制信号给小车
    'L' = 左转, 'R' = 右转, 'F' = 前进
    """
    if direction == 'L':
        # 左转：一个电机正转，另一个电机反转
        serialsend(b'\x03')
        print("向左移动")
    elif direction == 'R':
        # 右转：一个电机正转，另一个电机反转
        serialsend(b'\x02')
        print("向右移动")
    elif direction == 'F':
        # 前进：两个电机同时正转
        serialsend(b'\x01')
        print("向前移动")
    elif direction == 'S':
        # 停止电机
        serialsend(b'\x05')
        print("停止")
    elif direction == 'B':
        # 倒车
        serialsend(b'\x04')
        print("倒车")
    elif direction == 'C':
        # 舵机夹取
        serialsend(b'\x06')
        print("夹取")
    elif direction == 'O':
        # 舵机打开
        serialsend(b'\x07')
        print("放下")



#找球，确定最大圆的位置
def process_image_blue_ball(image):


    # 转换到 HSV 颜色空间并创建蓝色掩码
    hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_blue = np.array([100, 150, 70])
    upper_blue = np.array([140, 255, 255])
    mask_blue = cv2.inRange(hsv_img, lower_blue, upper_blue)


    # 进行形态学操作 - 闭操作去噪，腐蚀收缩轮廓
    kernel = np.ones((5, 5), np.uint8)
    mask_blue_clean = cv2.morphologyEx(mask_blue, cv2.MORPH_CLOSE, kernel, iterations=2)
    mask_blue_eroded = cv2.erode(mask_blue_clean, kernel, iterations=5)

    # 查找轮廓
    contours, _ = cv2.findContours(mask_blue_eroded, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # 初始化最大轮廓和其面积
    max_contour = None
    max_area = 0
    
    # 遍历轮廓，判断是否为小球并找出最大轮廓
    for contour in contours:
        # 最小外接圆
        ((x, y), radius) = cv2.minEnclosingCircle(contour)
        # 计算轮廓的面积
        area = cv2.contourArea(contour)

        # 计算圆的面积
        circle_area = np.pi * (radius ** 2)

        # 圆度判断条件
        if 0.4 * circle_area <= area <= 1.3 * circle_area and radius > 10:
            # 如果找到面积更大的轮廓，更新最大轮廓
            if area > max_area:
                max_area = area
                max_contour = contour

    # 绘制最大的轮廓（蓝色）
    if max_contour is not None:
        ((x, y), radius) = cv2.minEnclosingCircle(max_contour)
        return ((x, y), radius)
    # 保存检测结果
    return ((None, None), 0)

def process_image_red_ball(image):

    # 转换到 HSV 颜色空间并创建红色掩码
    hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_red1 = np.array([0, 120, 70])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 120, 70])
    upper_red2 = np.array([180, 255, 255])
    
    mask_red1 = cv2.inRange(hsv_img, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(hsv_img, lower_red2, upper_red2)
    
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)

    # 进行形态学操作
    kernel = np.ones((5, 5), np.uint8)
    mask_red_clean = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel, iterations=2)
    mask_red_eroded = cv2.erode(mask_red_clean, kernel, iterations=2)

    # 查找轮廓
    contours, _ = cv2.findContours(mask_red_eroded, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # 初始化最大轮廓和其面积
    max_contour = None
    max_area = 0
    
    # 遍历轮廓，判断是否为小球并找出最大轮廓
    for contour in contours:
        ((x, y), radius) = cv2.minEnclosingCircle(contour)
        area = cv2.contourArea(contour)
        circle_area = np.pi * (radius ** 2)

        if 0.6 * circle_area <= area <= 1.3 * circle_area and radius > 10:
            if area > max_area:
                max_area = area
                max_contour = contour

    # 返回最大红色小球的圆心坐标和半径
    if max_contour is not None:
        ((x, y), radius) = cv2.minEnclosingCircle(max_contour)
        return ((x, y), radius)
    return ((None, None), 0)


def process_image_black_ball(image):


    # 转换到 HSV 颜色空间并创建黑色掩码
    hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_black = np.array([0, 0, 0])
    upper_black = np.array([180, 255, 30])
    
    mask_black = cv2.inRange(hsv_img, lower_black, upper_black)

    # 进行形态学操作
    kernel = np.ones((5, 5), np.uint8)
    mask_black_clean = cv2.morphologyEx(mask_black, cv2.MORPH_CLOSE, kernel, iterations=2)
    mask_black_eroded = cv2.erode(mask_black_clean, kernel, iterations=8)

    # 查找轮廓
    contours, _ = cv2.findContours(mask_black_eroded, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # 初始化最大轮廓和其面积
    max_contour = None
    max_area = 0
    
    # 遍历轮廓，判断是否为小球并找出最大轮廓
    for contour in contours:
        ((x, y), radius) = cv2.minEnclosingCircle(contour)
        area = cv2.contourArea(contour)
        circle_area = np.pi * (radius ** 2)

        if 0.6 * circle_area <= area <= 1.3 * circle_area and radius > 10:
            if area > max_area:
                max_area = area
                max_contour = contour

    # 返回最大黑色小球的圆心坐标和半径
    if max_contour is not None:
        ((x, y), radius) = cv2.minEnclosingCircle(max_contour)
        return ((x, y), radius)
    return ((None, None), 0)

def process_image_orange_square(image):
    # 转换到 HSV 颜色空间并创建橙色掩码
    hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    # 定义橙色的HSV范围
    lower_orange = np.array([10, 120, 70])
    upper_orange = np.array([30, 255, 255])
    
    # 创建橙色的掩码
    mask_orange = cv2.inRange(hsv_img, lower_orange, upper_orange)

    # 进行形态学操作
    kernel = np.ones((5, 5), np.uint8)
    mask_orange_clean = cv2.morphologyEx(mask_orange, cv2.MORPH_CLOSE, kernel, iterations=2)
    mask_orange_eroded = cv2.erode(mask_orange_clean, kernel, iterations=2)

    # 查找轮廓
    contours, _ = cv2.findContours(mask_orange_eroded, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # 初始化最大轮廓和其面积
    max_contour = None
    max_area = 0
    best_rect = None
    
    # 遍历轮廓，找出最大方形轮廓
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        area = w * h
        
        # 判断是否为近似正方形 (宽高比接近 1)
        aspect_ratio = w / float(h)
        if 1.5 <= aspect_ratio <= 2.5 and area > max_area and w > 10 and h > 10:
            max_area = area
            best_rect = ((x, y), w*h)

    # 返回最大橙色方形的左上角坐标和宽高
    if best_rect is not None:
        return best_rect
    
    # 如果未找到合适的方形区域，返回None
    return ((None, None),  0)





# 将MIPI图像转换为cv2格式-RGB
def mipi_img2cv2_img_RGB(mipi_img, height, width):
    mipi_img = mipi_img.reshape(int(1.5 * height), width)
    cv2_img = cv2.cvtColor(mipi_img, cv2.COLOR_YUV420SP2BGR)
    image_rgb = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2RGB)
    return image_rgb

control_car('F')
time.sleep(0.1)
control_car('S')
time.sleep(0.5)
# 实时检测颜色并控制小车运动
while True:
    # 获取相机数据流
    ret, frame = capture.read()   

    # 处理图片，找出最大的球
    ((x, y), radius) = process_image_blue_ball(frame)

    # 只处理半径足够大的物体（过滤噪声）
    ball_found = False  # 初始化变量，表示未找到球

    while not ball_found:
        # 获取图像并处理
        ret, frame = capture.read()
        ((x, y), radius) = process_image_blue_ball(frame)
        print(((x, y), radius))
#        ret = cv2.imwrite("./mipi_camera_picture.png", cv2_img)


        # 只处理半径足够大的物体（过滤噪声）
        if radius > 10:
            ball_found = True
        else:
            # 没有找到球，开始寻找球
            control_car('F')
            time.sleep(0.5)
            control_car('S')

            # 再次获取图像数据，避免使用旧数据
            ret, frame = capture.read()
            ((x, y), radius) = process_image_blue_ball(frame)


            print(((x, y), radius))
#            output_image_path = './abc.png'
#            cv2.imwrite( './abc.png', cv2_img)
#            time.sleep(10)




            if radius > 10:
                ball_found = True  # 找到球，退出循环
            else:
                # 如果没找到，再向左转动
                control_car('L')
                time.sleep(0.3)
                control_car('S')
            ret, frame = capture.read()
            ((x, y), radius) = process_image_blue_ball(frame)
            if radius > 10:
                ball_found = True  # 找到球，退出循环
            else:
                # 如果没找到，再向左转动
                control_car('L')
                time.sleep(0.2)
                control_car('S')
            ret, frame = capture.read()
            ((x, y), radius) = process_image_blue_ball(frame)
            if radius > 10:
                ball_found = True  # 找到球，退出循环
            else:
                # 如果没找到，再向左转动
                control_car('L')
                time.sleep(0.2)
                control_car('S')
            ret, frame = capture.read()
            ((x, y), radius) = process_image_blue_ball(frame)
            if radius > 10:
                ball_found = True  # 找到球，退出循环
            else:
                # 如果没找到，再向左转动
                control_car('L')
                time.sleep(0.2)
                control_car('S')
            ret, frame = capture.read()
            ((x, y), radius) = process_image_blue_ball(frame)
            if radius > 10:
                ball_found = True  # 找到球，退出循环
            else:
                # 如果没找到，再向左转动
                control_car('L')
                time.sleep(0.2)
                control_car('S')
            ret, frame = capture.read()
            ((x, y), radius) = process_image_blue_ball(frame)
            if radius > 10:
                ball_found = True  # 找到球，退出循环
            else:
                # 如果没找到，再向左转动
                control_car('L')
                time.sleep(0.2)
                control_car('S')
            ret, frame = capture.read()
            ((x, y), radius) = process_image_blue_ball(frame)
            if radius > 10:
                ball_found = True  # 找到球，退出循环
            else:
                # 如果没找到，再向左转动
                control_car('L')
                time.sleep(0.2)
                control_car('S')
            ret, frame = capture.read()
            ((x, y), radius) = process_image_blue_ball(frame)
            if radius > 10:
                ball_found = True  # 找到球，退出循环
            else:
                # 如果没找到，再向左转动
                control_car('L')
                time.sleep(0.2)
                control_car('S')
            ret, frame = capture.read()
            ((x, y), radius) = process_image_blue_ball(frame)
            if radius > 10:
                ball_found = True  # 找到球，退出循环
            else:
                # 如果没找到，再向左转动
                control_car('L')
                time.sleep(0.2)
                control_car('S')
            ret, frame = capture.read()
            ((x, y), radius) = process_image_blue_ball(frame)
            if radius > 10:
                ball_found = True  # 找到球，退出循环
            else:
                # 如果没找到，再向左转动
                control_car('L')
                time.sleep(0.2)
                control_car('S')
            ret, frame = capture.read()
            ((x, y), radius) = process_image_blue_ball(frame)
            if radius > 10:
                ball_found = True  # 找到球，退出循环
            else:
                # 如果没找到，再向左转动
                control_car('L')
                time.sleep(0.2)
                control_car('S')
            ret, frame = capture.read()
            ((x, y), radius) = process_image_blue_ball(frame)
            if radius > 10:
                ball_found = True  # 找到球，退出循环
            else:
                # 如果没找到，再向左转动
                control_car('L')
                time.sleep(0.2)
                control_car('S')
            ret, frame = capture.read()
            ((x, y), radius) = process_image_blue_ball(frame)
            if radius > 10:
                ball_found = True  # 找到球，退出循环
            else:
                # 如果没找到，再向左转动
                control_car('L')
                time.sleep(0.2)
                control_car('S')
            ret, frame = capture.read()
            ((x, y), radius) = process_image_blue_ball(frame)
            if radius > 10:
                ball_found = True  # 找到球，退出循环
            else:
                # 如果没找到，再向左转动
                control_car('L')
                time.sleep(0.2)
                control_car('S')
            ret, frame = capture.read()
            ((x, y), radius) = process_image_blue_ball(frame)
            if radius > 10:
                ball_found = True  # 找到球，退出循环
            else:
                # 如果没找到，再向左转动
                control_car('L')
                time.sleep(0.2)
                control_car('S')
            ret, frame = capture.read()
            ((x, y), radius) = process_image_blue_ball(frame)
            if radius > 10:
                ball_found = True  # 找到球，退出循环
            else:
                # 如果没找到，再向左转动
                control_car('L')
                time.sleep(0.2)
                control_car('S')
            ret, frame = capture.read()
            ((x, y), radius) = process_image_blue_ball(frame)
            if radius > 10:
                ball_found = True  # 找到球，退出循环
            else:
                # 如果没找到，再向左转动
                control_car('L')
                time.sleep(0.2)
                control_car('S')
            ret, frame = capture.read()
            ((x, y), radius) = process_image_blue_ball(frame)
            if radius > 10:
                ball_found = True  # 找到球，退出循环
            else:
                # 如果没找到，再向左转动
                control_car('L')
                time.sleep(0.2)
                control_car('S')
            ret, frame = capture.read()
            ((x, y), radius) = process_image_blue_ball(frame)
            if radius > 10:
                ball_found = True  # 找到球，退出循环
            else:
                # 如果没找到，再向左转动
                control_car('L')
                time.sleep(0.2)
                control_car('S')
            ret, frame = capture.read()
            ((x, y), radius) = process_image_blue_ball(frame)
            if radius > 10:
                ball_found = True  # 找到球，退出循环
            else:
                # 如果没找到，再向左转动
                control_car('L')
                time.sleep(0.2)
                control_car('S')
    
    
    ret, frame = capture.read()   
    ((x, y), radius) = process_image_blue_ball(frame)
    if ball_found and radius > 10:
        # 计算图像的中心点
        cx = width // 2

        # 判断小球相对于图像中心的位置，并控制小车运动
        if x < cx - 50:
            control_car('L')
            time.sleep(0.1)
        elif x > cx + 50:
            control_car('R')
            time.sleep(0.1)
        else:
            control_car('F')
            time.sleep(0.2)
        control_car('S')

        # 小车移动后短暂停止，以避免连续发送移动指令
    else:
        ball_found =False
        

    

    flag11 = 0
    ret, frame = capture.read()   
    ((x, y), radius) = process_image_blue_ball(frame)
    cx = width // 2
    if radius > 60 and (x < cx + 100) and (x > cx - 100):
        flag11 = 1
        control_car('S')
        time.sleep(0.2)
        control_car('C')
        time.sleep(5)
    if flag11==1:
        break


# 转向目标区域
if flag11:  
    control_car('R')  # 右转1秒
    time.sleep(0.7)
    control_car('S')
    first_ball = 0



# 实时检测颜色并控制小车运动，找目标区域
while True:
    # 获取相机数据流
    ret, frame = capture.read()

    # 处理图片，找出最大矩形
    ((x, y), radius) = process_image_orange_square(frame)

    # 只处理半径足够大的物体（过滤噪声）
    if radius > 10:
        # 计算图像的中心点
        cx = width // 2

        # 判断小球相对于图像中心的位置，并控制小车运动
        if x < cx - 50:
            control_car('L')
        elif x > cx + 50:
            control_car('R')
        else:
            control_car('F')

        # 小车移动后短暂停止，以避免连续发送移动指令
        control_car('S')
    else:
        # 没有找到球，开始寻找球
        control_car('F')
        time.sleep(0.5)
        control_car('S')

        # 再次获取图像数据，避免使用旧数据
        ret, frame = capture.read()
        ((x, y), radius) = process_image_orange_square(frame)

        if radius > 10:
            continue  # 如果找到球，跳过后续的寻找逻辑

        # 如果没找到，再向左转动
        control_car('L')
        time.sleep(0.5)
        control_car('S')

        ret, frame = capture.read()
        ((x, y), radius) = process_image_orange_square(frame)

        if radius > 10:
            continue

        # 再向右转动
        control_car('R')
        time.sleep(0.4)
        control_car('S')

    flag12 = 0
    # 当小球靠近小车时，跳出找球
    if radius > 200:
        flag12 = 1
        control_car('F')
        time.sleep(0.7)
        control_car('S')
        control_car('C')
    
    if flag12 == 1:
        break  # 找到目标，跳出循环

# 转向目标区域
if flag12:  
    control_car('O')  # 放球
    time.sleep(1)
    control_car('B')#倒退
    time.sleep(1)
    control_car('L')#转到有球的位置
    time.sleep(2)



# 找第二个球,黑球
# 实时检测颜色并控制小车运动
while True:
    # 获取相机数据流
    ret, frame = capture.read() 
    ((x, y), radius) = process_image_black_ball(frame)# 处理图片，找出最大的球

    # 只处理半径足够大的物体（过滤噪声）
    if radius > 10:
        # 计算图像的中心点
        cx = width // 2

        # 判断小球相对于图像中心的位置，并控制小车运动
        if x < cx - 50:
            control_car('L')
        elif x > cx + 50:
            control_car('R')
        else:
            control_car('F')

        # 小车移动后短暂停止，以避免连续发送移动指令
        control_car('S')
    else:
        # 没有找到球，开始寻找球
        control_car('F')
        time.sleep(0.1)
        control_car('S')

        # 再次获取图像数据，避免使用旧数据
        ret, frame = capture.read()
        ((x, y), radius) = process_image_black_ball(frame)

        if radius > 10:
            continue  # 如果找到球，跳过后续的寻找逻辑

        # 如果没找到，再向左转动
        control_car('L')
        time.sleep(0.2)
        control_car('S')

        ret, frame = capture.read()
        ((x, y), radius) = process_image_black_ball(frame)

        if radius > 10:
            continue

        # 再向右转动
        control_car('R')
        time.sleep(0.4)
        control_car('S')

    flag21 = 0
    # 当小球靠近小车时，跳出找球
    if radius > 100:
        flag21 = 1
        control_car('F')
        time.sleep(0.7)
        control_car('S')
        control_car('C')
    if flag11==1:
        break


# 转向目标区域
if flag21:  
    control_car('L')  # 右转1秒
    time.sleep(0.7)
    control_car('S')
    first_ball = 0








# 放第二个球
# 实时检测颜色并控制小车运动，找目标区域
while True:
    # 获取相机数据流
    ret, frame = capture.read()

    # 处理图片，找出最大矩形
    ((x, y), radius) = process_image_orange_square(frame)

    # 只处理半径足够大的物体（过滤噪声）
    if radius > 10:
        # 计算图像的中心点
        cx = width // 2

        # 判断小球相对于图像中心的位置，并控制小车运动
        if x < cx - 50:
            control_car('L')
        elif x > cx + 50:
            control_car('R')
        else:
            control_car('F')

        # 小车移动后短暂停止，以避免连续发送移动指令
        control_car('S')
    else:
        # 没有找到球，开始寻找球
        control_car('F')
        time.sleep(0.1)
        control_car('S')

        # 再次获取图像数据，避免使用旧数据
        ret, frame = capture.read()
        ((x, y), radius) = process_image_orange_square(frame)

        if radius > 10:
            continue  # 如果找到球，跳过后续的寻找逻辑

        # 如果没找到，再向左转动
        control_car('L')
        time.sleep(0.2)
        control_car('S')

        ret, frame = capture.read()
        ((x, y), radius) = process_image_orange_square(frame)

        if radius > 10:
            continue

        # 再向右转动
        control_car('R')
        time.sleep(0.4)
        control_car('S')

    flag22 = 0
    # 当小球靠近小车时，跳出找球
    if radius > 200:
        flag22 = 1
        control_car('F')
        time.sleep(0.7)
        control_car('S')

    if flag22==1:
        break

# 转向目标区域
if flag22:  
    control_car('O')  # 放球
    time.sleep(1)
    control_car('B')#倒退
    time.sleep(1)
    control_car('L')#转到有放球的位置
    time.sleep(1)






# 关闭摄像头
capture.release()
cv2.destroyAllWindows()
