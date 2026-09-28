#!/usr/bin/python3
# ===================================== COPYRIGHT ===================================== #
#                                                                                       #
#  IFRA (Intelligent Flexible Robotics and Assembly) Group, CRANFIELD UNIVERSITY        #
#  Created on behalf of the IFRA Group at Cranfield University, United Kingdom          #
#  E-mail: IFRA@cranfield.ac.uk                                                         #
#                                                                                       #
#  Licensed under the Apache-2.0 License.                                               #
#  You may not use this file except in compliance with the License.                     #
#  You may obtain a copy of the License at: http://www.apache.org/licenses/LICENSE-2.0  #
#                                                                                       #
#  Unless required by applicable law or agreed to in writing, software distributed      #
#  under the License is distributed on an "as-is" basis, without warranties or          #
#  conditions of any kind, either express or implied. See the License for the specific  #
#  language governing permissions and limitations under the License.                    #
#                                                                                       #
#  IFRA Group - Cranfield University                                                    #
#  AUTHORS: Mikel Bueno Viso         - Mikel.Bueno-Viso@cranfield.ac.uk                 #
#           Irene Bernardino Sanchez - i.bernardinosanchez.854@cranfield.ac.uk          #
#           Seemal Asif              - s.asif@cranfield.ac.uk                           #
#           Phil Webb                - p.f.webb@cranfield.ac.uk                         #
#                                                                                       #
#  Date: November, 2023.                                                                #
#                                                                                       #
# ===================================== COPYRIGHT ===================================== #
# ======= CITE OUR WORK ======= #
# You can cite our work with the following statement:
# IFRA-Cranfield (2023). Object Detection and Pose Estimation within a Robot Cell. URL: https://github.com/IFRA-Cranfield/irb120_PoseEstimation
# detection.py
# 本脚本实现目标检测与图像处理模块
# 功能：图像订阅、ArUco标定透视变换、YOLOv8立方体检测、立方体平面位姿(x,y,yaw)求解、颜色识别
# ===== IMPORT REQUIRED COMPONENTS ===== #
# 导入ROS2核心库
import rclpy
from rclpy.node import Node
# ROS2图像消息类型
from sensor_msgs.msg import Image
# OpenCV库，包含ArUco标记、图像处理
import cv2
from cv2 import aruco
import numpy as np
# cv_bridge：ROS图像消息与OpenCV图像格式转换
from cv_bridge import CvBridge, CvBridgeError
# YOLOv8目标检测库
from ultralytics import YOLO
# 通用工具库
import os
import time
# GLOBAL VARIABLES: 全局变量
Gz_CAM = None          # 保存Gazebo相机输出的OpenCV图像
ENVIRONMENT = ""       # 环境标记：GAZEBO / ROBOT(真机摄像头)
# =============================================================================== #
# CLASS -> ImgSUB: ROS2图像订阅节点，专门接收Gazebo虚拟相机图像
class ImgSUB(Node):
    def __init__(self):
        # 节点名称：irb120pe_GzCAM_Subscriber
        super().__init__("irb120pe_GzCAM_Subscriber")
        # 创建订阅器，订阅Gazebo相机话题 /camera/image_raw，回调函数CALLBACK_FN，队列长度10
        self.SubIMAGE = self.create_subscription(Image, "/camera/image_raw", self.CALLBACK_FN, 10)
        # 实例化cv_bridge，用于ROS图像消息 <-> OpenCV图像互转
        self.BRIDGE = CvBridge()

    def CALLBACK_FN(self, ROS2img):
        """图像话题回调函数，收到ROS图像消息，转为BGR格式opencv图像存入全局变量Gz_CAM"""
        global Gz_CAM
        try:
            # ROS Image消息转OpenCV BGR图像
            Gz_CAM = self.BRIDGE.imgmsg_to_cv2(ROS2img, "bgr8")
        except CvBridgeError as ERR:
            print("(cv_bridge): ERROR -> " + ERR)
            print("")
# =============================================================================== #
# CLASS -> CubeDetection: 立方体检测主类，对外提供所有视觉相关接口（main_Gz.py调用这个类）
class CubeDetection():
    def __init__(self, ENV):
        """
        构造函数
        :param ENV: 环境标识 "GAZEBO"=仿真，"ROBOT"=真机摄像头
        """
        global ENVIRONMENT
        # YOLO模型路径：dev_ws工作空间下的预训练pt权重
        modelPATH = os.path.join(os.path.expanduser('~'), 'dev_ws', 'src', 'irb120_PoseEstimation', 'irb120pe_detection', 'yolov8')

        # Initialise CAMERA: 根据环境选择相机初始化逻辑
        if (ENV == "GAZEBO"):
            # Gazebo仿真：实例化ROS2图像订阅节点
            self.GzCAM_SUB = ImgSUB()
            self.InitCamGz()
            ENVIRONMENT = "GAZEBO"
            # 加载仿真场景专用YOLOv8n模型
            self.YOLOmodel = YOLO(modelPATH + '/cubeDETECTION_Gz.pt') # Pre-trained YOLOv8n model.
        else:
            # 真机环境，调用本地摄像头
            self.InitCam()
            ENVIRONMENT = "ROBOT"
            # 加载真机场景YOLOv8n模型
            self.YOLOmodel = YOLO(modelPATH + '/cubeDETECTION.pt') # Pre-trained YOLOv8n model.
        # 标定后透视图像的宽高(单位mm)，工作台平面尺寸
        self.w = 750
        self.h = 400

    def InitCam(self):
        """【真机摄像头初始化】打开本地USB摄像头，读取一帧图像存入self.inputImg"""
        print("=== WEBCAM: Initialization ===")
        print("Loading WEB-CAMERA...")
        print("")
        # 打开摄像头设备0
        self.camera = cv2.VideoCapture(0)
        # 等待1秒，读取图像，保证相机稳定
        T = time.time() + 1.0
        while time.time() < T:
            self.ret, self.inputImg = self.camera.read()
        # 图像为空，报错退出
        if self.inputImg is None:
            print("Error! Input image is empty. Please check the camera and the VideoCapture(i) index.")
            rclpy.shutdown()
            print("CLOSING PROGRAM...")
            exit()
        # 读取失败，报错退出
        if not self.ret:
            print("Error! Failed to capture input image. Please check the camera and the VideoCapture(i) index.")
            rclpy.shutdown()
            print("CLOSING PROGRAM...")
            exit()

    def InitCamGz(self):
        """【Gazebo虚拟相机初始化】持续spin ROS2订阅节点，获取Gazebo相机图像，存入self.inputImg"""
        global Gz_CAM
        print("=== Gazebo CAM: Initialization ===")
        print("Loading GAZEBO CAMERA...")
        print("")
        # 等待1s，循环spin获取图像，等待相机话题稳定
        T = time.time() + 1.0
        while time.time() < T:
            # 驱动ROS2节点执行一次回调，接收图像消息
            rclpy.spin_once(self.GzCAM_SUB)
            # 把全局Gz_CAM（最新帧图像）赋值给类内变量
            self.inputImg = Gz_CAM
        # 图像为空则报错退出
        if self.inputImg is None:
            print("Error! Input image is empty. Please check that the Gz WEBCAM is publishing the IMG correctly.")
            rclpy.shutdown()
            print("CLOSING PROGRAM...")
            exit()

    def GetPerspectiveImg(self, ShowPerspective):
        """
        ArUco标记标定 + 透视变换矫正
        原理：工作台四个角放4个ArUco标记，求出透视变换矩阵，把倾斜相机图像矫正成【工作台俯视图鸟瞰图】
        :param ShowPerspective: True 弹出窗口显示矫正后的俯视图
        输出：self.perspectiveImg 矫正后的鸟瞰图像（后续YOLO全部在这张图上做检测）
        """
        if ShowPerspective:
            print("=== DETECTION: Calibration ===")
            print("Transforming the InputIMG into PerspectiveIMG using the ARUCO Tags...")
            print("")
        # 初始化ArUco检测器参数与字典，使用5x5标记
        param = cv2.aruco.DetectorParameters()
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
        detector = aruco.ArucoDetector(aruco_dict,param)
        # 检测图像里所有ArUco标记，返回角点、ID、被拒绝候选标记
        markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(self.inputImg)
        # 在原图绘制检测到的ArUco标记框
        outputImage = self.inputImg.copy()
        cv2.aruco.drawDetectedMarkers(outputImage, markerCorners, markerIds)

        # 源点：原图中4个ArUco标记角点；目标点：矫正后俯视图四个角坐标
        src = np.zeros((4, 2), dtype=np.float32)
        dst = np.array([[0.0, 0.0], [self.w, 0.0], [0.0, self.h], [self.w, self.h]], dtype=np.float32)

        poly_corner = np.zeros((4, 3), dtype=np.int32)
        # 读取4个marker的ID和对应角点坐标
        for i in range(4):
            poly_corner[i][0] = int(markerIds[i][0])
            poly_corner[i][1] = int(markerCorners[i][0][0][0])
            poly_corner[i][2] = int(markerCorners[i][0][0][1])
        # 冒泡排序：按照ArUco的ID从小到大排序4个标记，保证四个角点顺序匹配
        for i in range(3):
            for j in range(i + 1, 4):
                if poly_corner[i][0] > poly_corner[j][0]:
                    for k in range(3):
                        aux = poly_corner[i][k]
                        poly_corner[i][k] = poly_corner[j][k]
                        poly_corner[j][k] = aux
        # 把排序后的4个标记角点填入src源点数组
        for i in range(4):
            src[i] = np.array([poly_corner[i][1], poly_corner[i][2]], dtype=np.float32)
        # 计算透视变换矩阵
        perspTransMatrix = cv2.getPerspectiveTransform(src, dst)
        # 执行透视变换，得到矫正后的鸟瞰俯视图
        self.perspectiveImg = cv2.warpPerspective(outputImage, perspTransMatrix, (int(self.w), int(self.h)))

        # 如果开启可视化，循环显示矫正后的俯视图，按q关闭窗口
        if ShowPerspective:
            while True:
                if self.perspectiveImg is not None:
                    self.perspectiveImgSHOW = cv2.resize(self.perspectiveImg, (750, 400))
                    cv2.imshow("IRB-120 PoseEstimation: Perspective Image", self.perspectiveImgSHOW)
                key = cv2.waitKey(1)
                if key == ord('q'):
                    cv2.destroyWindow("IRB-120 PoseEstimation: Perspective Image")
                    break

    def TestDetection(self):
        """YOLO模型可视化测试函数，实时画面显示YOLO检测框，用于调试，main_Gz中默认注释"""
        global Gz_CAM
        global ENVIRONMENT
        print("=== DETECTION: YOLOv8 MODEL ===")
        print("Showing the execution of the YOLOv8 model detection...")
        print("")
        while True:
            # 仿真环境：刷新Gazebo相机图像
            if (ENVIRONMENT == "GAZEBO"):
                rclpy.spin_once(self.GzCAM_SUB)
                self.inputImg = Gz_CAM
            else:
                # 真机：读取摄像头帧
                self.ret, self.inputImg = self.camera.read()
            # 生成透视矫正俯视图
            self.GetPerspectiveImg(ShowPerspective=False)
            if self.perspectiveImg is not None:
                # YOLO推理，绘制检测框
                results = self.YOLOmodel(self.perspectiveImg)
                annotated_frame = results[0].plot()
                cv2.imshow("IRB-120 PoseEstimation: YOLO Output", annotated_frame)
            # 按q退出可视化窗口
            key = cv2.waitKey(1)
            if key == ord('q'):
                cv2.destroyWindow("IRB-120 PoseEstimation: YOLO Output")
                break

    def CubeLocation(self):
        """
        【核心函数：获取立方体平面位姿】main_Gz调用这个函数
        输入：透视矫正后的俯视图self.perspectiveImg
        输出字典RESULT:
            x,y: 立方体在机器人基坐标系下平面坐标(米)
            yaw: 立方体绕Z轴旋转角（偏航角，弧度）
            detection: YOLO识别标签(WhiteCube/BlackCube/BlueCube/Sticker/Cube)
            success: 是否检测成功
        实现逻辑：YOLO检测立方体 → 提取ROI区域 → 轮廓检测 + 最小外接矩形求解旋转角度 → 像素坐标转为机器人世界坐标
        """
        print("=== DETECTION: YOLOv8 MODEL ===")
        print("DETECTING the cube with YOLOv8 and getting its POSE...")
        print("")
        # 初始化返回结果字典
        RESULT = {"x": 0.0, "y": 0.0, "yaw": 0.0, "detection": "", "success": False}
        # 预热YOLO模型（加载权重）
        results = self.YOLOmodel(self.perspectiveImg)
        names = self.YOLOmodel.names
        # 持续检测1秒，只要检测到目标就跳出循环
        t_end = time.time() + 1.0
        Detected = False
        while time.time() < t_end:
            results = self.YOLOmodel(self.perspectiveImg)
            boxes = results[0].boxes
            ids = []
            # 遍历所有检测框，收集类别名称
            for box in boxes:
                box = boxes.xyxy
                for c in boxes.cls:
                    ids.append(names[int(c)])
            if (len(ids) != 0):
                print("")
                Detected = True
                break
        # 1s内没有检测到立方体，返回失败
        if not Detected:
            print("")
            print("ERROR: The YOLOv8 model could not detect any cube in the workspace.")
            print("")
            return(RESULT)
        # 拿到立方体的bounding box（优先取cube/sticker类别框，得到立方体ROI）
        for box in boxes:
            name = names[int(box.cls[0])]
            if (name == "sticker" or name == "cube"): # ROI必须取整个立方体框，不能只用sticker贴纸的小框
                x1, y1, x2, y2 = box.xyxy[0]
                break
        # 框向外扩充3像素，作为ROI区域
        x = int(x1)-3
        y = int(y1)-3
        w = int(x2)+3
        h = int(y2)+3
        ROI = self.perspectiveImg[y:h, x:w]
        # ROI预处理：高斯模糊降噪，转HSV颜色空间，颜色阈值掩码
        ROI = cv2.GaussianBlur(ROI,(3,3),0)
        imgHSV = cv2.cvtColor(ROI, cv2.COLOR_BGR2HSV)
        lowLimit = (0, 10, 10)
        upLimit = (70, 255, 255)
        mask = cv2.inRange(imgHSV, lowLimit, upLimit)
        # 轮廓提取，只保留面积大于500像素的轮廓，过滤噪声
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        poly_contour = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 500:
                poly_contour.append(contour)
        if not poly_contour:
            print("ERROR: No contours detected.")
            return(RESULT)
        # 最小外接矩形：获取立方体中心像素坐标 + 旋转角度
        for cnt in poly_contour:
            rect = cv2.minAreaRect(cnt)
            (xo, yo), (wo, ho), angle = rect
            # 把ROI内局部像素坐标，还原到整张俯视图的全局像素坐标
            xo = xo + x
            yo = yo + y
        # 角度转为弧度，取负号（坐标系方向匹配）
        RESULT["yaw"] = -(angle * 3.1416/180.0)
        # 像素 -> 米单位转换，叠加固定偏移量（标定好的工作台原点到机器人base的偏移）
        RESULT["x"] = yo/1000 + 0.35
        RESULT["y"] = xo/1000 + 0.15
        RESULT["detection"] = "Cube"
        RESULT["success"] = True
        # 遍历YOLO检测类别，更新detection标签
        for id in ids:
            if id == "white":
                RESULT["detection"] = "WhiteCube"
                break
            if id == "black":
                RESULT["detection"] = "BlackCube"
                break
            if id == "blue":
                RESULT["detection"] = "BlueCube"
                break
            if id == "sticker":
                RESULT["detection"] = "Sticker"
        return(RESULT)

    def DetectColour(self):
        """
        抓取立方体之后调用，识别立方体朝上一面的颜色
        连续采集1s图像，统计各类目标检测计数，用投票方式选出最终颜色，提升鲁棒性
        返回："WhiteCube"/"BlackCube"/"BlueCube"/"Cube"(识别失败)
        """
        global Gz_CAM
        global ENVIRONMENT
        print("=== DETECTION: DetectColour ===")
        print("Detecting colour of the cube face...")
        print("")
        T = time.time() + 1.0
        # 类别计数器
        StickerCount = 0
        WhiteCount = 0
        BlackCount = 0
        BlueCount = 0

        while (time.time() <= T):
            # 获取图像
            if (ENVIRONMENT == "GAZEBO"):
                rclpy.spin_once(self.GzCAM_SUB)
                self.inputImg = Gz_CAM
            else:
                self.ret, self.inputImg = self.camera.read()
            if self.inputImg is not None:
                results = self.YOLOmodel(self.inputImg)
                names = self.YOLOmodel.names
                annotated_frame = results[0].plot()
                boxes = results[0].boxes
                ids = []
                for box in boxes:
                    box = boxes.xyxy
                    for c in boxes.cls:
                        ids.append(names[int(c)])
                # 统计每一类检测出现次数
                for ID in ids:
                    if (ID == "sticker"):
                        StickerCount = StickerCount + 1
                    elif (ID == "white"):
                        WhiteCount = WhiteCount + 1
                    elif (ID == "black"):
                        BlackCount = BlackCount + 1
                    elif (ID == "blue"):
                        BlueCount = BlueCount + 1
        # 投票判断颜色
        if (WhiteCount == BlueCount == BlackCount == 0 and StickerCount > 150):
            COLOUR = "Cube"
        elif (WhiteCount > 3 and WhiteCount > BlueCount and WhiteCount > BlackCount):
            COLOUR = "WhiteCube"
        elif (WhiteCount < BlueCount and BlueCount > BlackCount and BlueCount > 3):
            COLOUR = "BlueCube"
        elif (BlackCount > BlueCount and WhiteCount < BlackCount and BlackCount > 3):
            COLOUR = "BlackCube"
        else:
            COLOUR = "Cube"
        print("")
        print("(ColourDetection): RESULT -> " + COLOUR)
        print("")
        return(COLOUR)

    def ConstantVisualization(self):
        """持续可视化YOLO检测画面，调试用，按e退出"""
        global Gz_CAM
        global ENVIRONMENT
        while True:
            if (ENVIRONMENT == "GAZEBO"):
                rclpy.spin_once(self.GzCAM_SUB)
                self.inputImg = Gz_CAM
            else:
                self.ret, self.inputImg = self.camera.read()
            if self.inputImg is not None:
                results = self.YOLOmodel(self.inputImg)
                annotated_frame = results[0].plot()
                VISUALIZE = cv2.resize(annotated_frame, (1280, 720))
                cv2.imshow("IRB-120 PoseEstimation: YOLO Output", VISUALIZE)
            key = cv2.waitKey(1)
            if key == ord('e'):
                cv2.destroyWindow("IRB-120 PoseEstimation: YOLO Output")
                break
