#!/usr/bin/python3
# ===================================== COPYRIGHT ===================================== #
#  原作者: IFRA-Cranfield University (2023)
#  本文件: 在原 detection.py 基础上加了详细中文注释，未改动任何逻辑
#  重点注释区域:
#    1. ImgSUB 里的 TF2 Listener
#    2. _quat_to_rot 四元数→旋转矩阵
#    3. PixelToWorldPlane 像素→世界坐标（核心）
#    4. CubeLocation 里逆透视回映那一段
# =============================================================================== #

import rclpy
from rclpy.node import Node

# sensor_msgs: ROS2 标准消息类型
#   Image     -> 原始图像
#   CameraInfo-> 相机内参（K矩阵、宽高等），由相机驱动发布
from sensor_msgs.msg import Image, CameraInfo

import cv2
from cv2 import aruco
import numpy as np

# cv_bridge: ROS Image 消息 ↔ OpenCV Mat  的转换桥
from cv_bridge import CvBridge, CvBridgeError

# ====== TF2 相关（本次改造新加的 import）======
# Buffer:        TF 数据缓存，后台自动订阅 /tf 和 /tf_static
# TransformListener: 启动后后台一直监听，把变换存进 Buffer
# Time:          表示"最新一帧"的时间戳
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time

# YOLOv8 目标检测
from ultralytics import YOLO

import os
import time

# 全局变量：Gazebo 相机帧和相机内参，由回调写入
Gz_CAM = None
Gz_CAM_INFO = None
ENVIRONMENT = ""


# =============================================================================== #
# CLASS -> ImgSUB: 订阅相机图像和相机内参，同时挂一个 TF2 Listener
# =============================================================================== #
class ImgSUB(Node):
    def __init__(self):
        super().__init__("irb120pe_GzCAM_Subscriber")

        # 订阅原始图像话题
        self.SubIMAGE = self.create_subscription(
            Image,
            "/camera/image_raw",
            self.CALLBACK_FN,
            10
        )

        # 订阅相机内参话题（fx/fy/cx/cy 从这里来）
        self.SubCAMINFO = self.create_subscription(
            CameraInfo,
            "/camera/camera_info",
            self.CALLBACK_INFO,
            10
        )

        self.BRIDGE = CvBridge()

        # ====== TF2 Listener（本次改造新加）======
        # tf_buffer: 一个本地缓存，后台自动收 /tf、/tf_static，存着整个坐标树
        # tf_listener: 启动监听，所有变换自动进 buffer
        # 之后要查"camera_link 在 world 下的位姿"，直接问 buffer 就行
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

    def CALLBACK_FN(self, ROS2img):
        """收到一帧图像，ROS 格式 → OpenCV BGR 格式"""
        global Gz_CAM
        try:
            Gz_CAM = self.BRIDGE.imgmsg_to_cv2(ROS2img, "bgr8")
        except CvBridgeError as ERR:
            print("(cv_bridge): ERROR -> " + ERR)
            print("")

    def CALLBACK_INFO(self, ROS2info):
        """收到相机内参，存起来备用"""
        global Gz_CAM_INFO
        Gz_CAM_INFO = ROS2info


# =============================================================================== #
# CLASS -> CubeDetection: 视觉检测主类
# =============================================================================== #
class CubeDetection():
    def __init__(self, ENV):
        global ENVIRONMENT
        modelPATH = os.path.join(
            os.path.expanduser('~'),
            'dev_ws', 'src', 'irb120_PoseEstimation',
            'irb120pe_detection', 'yolov8'
        )

        if ENV == "GAZEBO":
            # 仿真模式：从 ROS 话题读相机图像
            self.GzCAM_SUB = ImgSUB()
            self.InitCamGz()
            ENVIRONMENT = "GAZEBO"
            self.YOLOmodel = YOLO(modelPATH + '/cubeDETECTION_Gz.pt')
        else:
            # 真机模式：直接读本机 USB 摄像头
            self.InitCam()
            ENVIRONMENT = "ROBOT"
            self.YOLOmodel = YOLO(modelPATH + '/cubeDETECTION.pt')

        # 鸟瞰图目标尺寸（像素）：750x400
        # 这两个数决定了透视变换后输出图像的大小
        self.w = 750
        self.h = 400

    # ---- 真机摄像头初始化 ----
    def InitCam(self):
        print("=== WEBCAM: Initialization ===")
        self.camera = cv2.VideoCapture(0)
        T = time.time() + 1.0
        while time.time() < T:
            self.ret, self.inputImg = self.camera.read()
        if self.inputImg is None:
            print("Error! Input image is empty.")
            rclpy.shutdown()
            exit()

    # ---- Gazebo 相机初始化：等第一帧图像 ----
    def InitCamGz(self):
        global Gz_CAM
        print("=== Gazebo CAM: Initialization ===")
        T = time.time() + 1.0
        while time.time() < T:
            rclpy.spin_once(self.GzCAM_SUB)
            self.inputImg = Gz_CAM
        if self.inputImg is None:
            print("Error! Input image is empty.")
            rclpy.shutdown()
            exit()

    # ======================================================================= #
    # 透视矫正：检测 4 个 ArUco 码，把斜视图拉成鸟瞰图
    # ======================================================================= #
    def GetPerspectiveImg(self, ShowPerspective):
        # 用 5x5 字典，最多 1000 个不同 ID
        param = cv2.aruco.DetectorParameters()
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
        detector = aruco.ArucoDetector(aruco_dict, param)
        markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(self.inputImg)

        if markerIds is None or len(markerIds) < 4:
            raise RuntimeError(
                f"ArUco calibration failed: expected 4 markers, "
                f"but detected {0 if markerIds is None else len(markerIds)}"
            )

        markerIds = np.asarray(markerIds).reshape(-1)
        print("Detected ArUco IDs:", markerIds)

        outputImage = self.inputImg.copy()
        cv2.aruco.drawDetectedMarkers(outputImage, markerCorners, markerIds.reshape(-1, 1))

        # dst: 鸟瞰图上四个角点的目标位置（左上角、右上角、左下角、右下角）
        dst = np.array([
            [0.0, 0.0],
            [self.w, 0.0],
            [0.0, self.h],
            [self.w, self.h]
        ], dtype=np.float32)

        # 把四个码的角点按 ID 排序（ID 0/1/2/3 分别对应四个角）
        src = np.zeros((4, 2), dtype=np.float32)
        poly_corner = np.zeros((4, 3), dtype=np.int32)
        for i in range(4):
            poly_corner[i][0] = int(markerIds[i])
            poly_corner[i][1] = int(markerCorners[i][0][0][0])
            poly_corner[i][2] = int(markerCorners[i][0][0][1])

        for i in range(3):
            for j in range(i + 1, 4):
                if poly_corner[i][0] > poly_corner[j][0]:
                    for k in range(3):
                        aux = poly_corner[i][k]
                        poly_corner[i][k] = poly_corner[j][k]
                        poly_corner[j][k] = aux

        for i in range(4):
            src[i] = np.array([poly_corner[i][1], poly_corner[i][2]], dtype=np.float32)

        # ====== 正向透视矩阵：原始图 -> 鸟瞰图 ======
        self.perspTransMatrix = cv2.getPerspectiveTransform(src, dst)

        # ====== 逆透视矩阵：鸟瞰图 -> 原始图（本次改造新加，关键！）======
        # 为什么要存逆矩阵？
        # YOLO 和轮廓检测都在鸟瞰图上做，得到的中心 (xo,yo) 是鸟瞰图坐标；
        # 但相机内参 K 只对原始图有效，必须先映回原始像素 (u,v) 才能反投影。
        self.invPerspTransMatrix = np.linalg.inv(self.perspTransMatrix)

        # 生成鸟瞰图
        self.perspectiveImg = cv2.warpPerspective(
            outputImage, self.perspTransMatrix, (int(self.w), int(self.h))
        )

        if ShowPerspective:
            while True:
                if self.perspectiveImg is not None:
                    self.perspectiveImgSHOW = cv2.resize(self.perspectiveImg, (750, 400))
                    cv2.imshow("Perspective Image", self.perspectiveImgSHOW)
                key = cv2.waitKey(1)
                if key == ord('q'):
                    cv2.destroyWindow("Perspective Image")
                    break

    # ---- 测试用：循环跑 YOLO 看效果 ----
    def TestDetection(self):
        global Gz_CAM, ENVIRONMENT
        while True:
            if ENVIRONMENT == "GAZEBO":
                rclpy.spin_once(self.GzCAM_SUB)
                self.inputImg = Gz_CAM
            else:
                self.ret, self.inputImg = self.camera.read()
            self.GetPerspectiveImg(ShowPerspective=False)
            if self.perspectiveImg is not None:
                results = self.YOLOmodel(self.perspectiveImg)
                annotated_frame = results[0].plot()
                cv2.imshow("YOLO Output", annotated_frame)
            key = cv2.waitKey(1)
            if key == ord('q'):
                cv2.destroyWindow("YOLO Output")
                break

    # ======================================================================= #
    # 工具函数：四元数 -> 3x3 旋转矩阵（本次改造新加）
    # 输入: qx,qy,qz,qw
    # 输出: R (3x3)
    # 数学公式:
    #   R = [[1-2(y²+z²),  2(xy-zw),   2(xz+yw)],
    #        [2(xy+zw),    1-2(x²+z²), 2(yz-xw)],
    #        [2(xz-yw),    2(yz+xw),   1-2(x²+y²)]]
    # ======================================================================= #
    @staticmethod
    def _quat_to_rot(qx, qy, qz, qw):
        q = np.array([qx, qy, qz, qw], dtype=np.float64)
        q = q / np.linalg.norm(q)   # 归一化，防止数值误差
        x, y, z, w = q
        return np.array([
            [1 - 2*(y*y + z*z),   2*(x*y - z*w),   2*(x*z + y*w)],
            [2*(x*y + z*w),     1 - 2*(x*x + z*z), 2*(y*z - x*w)],
            [2*(x*z - y*w),     2*(y*z + x*w),   1 - 2*(x*x + y*y)]
        ], dtype=np.float64)

    # ======================================================================= #
    # 核心函数：把原始图像素 (u,v) 变成世界坐标点 (x,y,z)
    # 步骤:
    #   1. 拿相机内参 K，归一化像素
    #   2. optical frame -> camera_link 轴变换
    #   3. TF2 查询 camera_link 在 world 下的位姿
    #   4. 射线与 z=z_plane 平面求交
    # ======================================================================= #
    def PixelToWorldPlane(self, u, v, z_plane=0.88):
        global Gz_CAM_INFO

        # ---- 1. 等相机内参到位 ----
        deadline = time.time() + 2.0
        while Gz_CAM_INFO is None and time.time() < deadline:
            rclpy.spin_once(self.GzCAM_SUB, timeout_sec=0.05)
        if Gz_CAM_INFO is None:
            raise RuntimeError("No /camera/camera_info received.")

        K = Gz_CAM_INFO.k
        fx = float(K[0])   # x 方向焦距（像素）
        fy = float(K[4])   # y 方向焦距（像素）
        cx = float(K[2])   # 主点 x（图像中心）
        cy = float(K[5])   # 主点 y

        # ---- 2. 像素归一化 ----
        # xn = (u-cx)/fx, yn = (v-cy)/fy
        # 物理意义: 在 optical frame 下，z=1 平面上的坐标
        xn = (u - cx) / fx
        yn = (v - cy) / fy

        # ---- 3. optical frame -> camera_link 轴变换 ----
        #
        # optical frame（OpenCV 惯例）:  x 右,  y 下,  z 前(光轴)
        # camera_link（ROS 机器人惯例）: x 前, y 左, z 上
        #
        # 对应关系:
        #   optical z(前)  -> camera_link x(前)
        #   optical x(右)  -> camera_link -y(因为 y 是左，右就是 -左)
        #   optical y(下)  -> camera_link -z(因为 z 是上，下就是 -上)
        #
        # 所以 ray_camera = [1, -xn, -yn]
        # 这个顺序写反了，算出来的世界坐标会整体转 90°
        ray_camera = np.array([1.0, -xn, -yn], dtype=np.float64)

        # ---- 4. TF2 查询 camera_link -> world 的变换 ----
        # lookup_transform("world", "camera_link") 的含义:
        #   "给我一个变换，把 camera_link 下的点/向量变到 world 下"
        # 注意: 这里之前踩过坑，一开始写成了 "base_link"，
        #       结果射线在 base_link 系、求交平面 z=0.88 在 world 系，坐标系混用
        transform = None
        deadline = time.time() + 2.0
        while time.time() < deadline:
            rclpy.spin_once(self.GzCAM_SUB, timeout_sec=0.05)
            try:
                transform = self.GzCAM_SUB.tf_buffer.lookup_transform(
                    "world", "camera_link", Time()
                )
                break
            except Exception:
                pass  # TF 还没就绪，等下一帧

        if transform is None:
            raise RuntimeError("Cannot get TF world <- camera_link.")

        # t: 相机光心在 world 下的位置 (平移)
        # q: 相机在 world 下的朝向 (四元数)
        t = transform.transform.translation
        q = transform.transform.rotation

        origin = np.array([t.x, t.y, t.z], dtype=np.float64)

        # 四元数 -> 旋转矩阵 R
        R = self._quat_to_rot(q.x, q.y, q.z, q.w)

        # ---- 5. 把射线方向从 camera_link 旋到 world ----
        # 注意: 射线是方向向量，只乘 R，不加 origin(平移)
        #       方向没有"位置"，平移不影响方向
        ray_world = R @ ray_camera

        # ---- DEBUG 输出 ----
        print("===== TF2 DEBUG =====")
        print(f"TF translation = [{t.x:.6f}, {t.y:.6f}, {t.z:.6f}]")
        print(f"TF quaternion  = [{q.x:.6f}, {q.y:.6f}, {q.z:.6f}, {q.w:.6f}]")
        print(f"normalized pixel: xn={xn:.6f}, yn={yn:.6f}")
        print(f"ray_camera = {ray_camera}")
        print(f"ray_world  = {ray_world}")

        # ---- 6. 射线与 z=z_plane 平面求交 ----
        # 射线上任意点: P(s) = origin + s * ray_world
        # 令 P_z(s) = z_plane, 解 s:
        #   s = (z_plane - origin_z) / ray_world_z
        if abs(ray_world[2]) < 1e-9:
            raise RuntimeError("Camera ray parallel to plane.")

        scale = (z_plane - origin[2]) / ray_world[2]

        if scale <= 0:
            raise RuntimeError("Plane is behind camera.")

        point_world = origin + scale * ray_world

        print(f"scale = {scale:.6f}")
        print(f"result = [{point_world[0]:.6f}, {point_world[1]:.6f}, {point_world[2]:.6f}]")
        print("=====================")
        print(f"(TF2 Pixel -> World): u={u:.2f}, v={v:.2f} -> "
              f"x={point_world[0]:.4f}, y={point_world[1]:.4f}, z={point_world[2]:.4f}")

        return (float(point_world[0]), float(point_world[1]), float(point_world[2]))

    # ======================================================================= #
    # CubeLocation: 检测一个方块，返回它的 world 坐标和 yaw
    # ======================================================================= #
    def CubeLocation(self):
        print("=== DETECTION: Getting cube POSE...")
        RESULT = {"x": 0.0, "y": 0.0, "yaw": 0.0, "detection": "", "success": False}

        # 先跑一次 YOLO 加载模型
        results = self.YOLOmodel(self.perspectiveImg)
        names = self.YOLOmodel.names

        # 持续检测 1 秒，直到看到方块
        t_end = time.time() + 1.0
        Detected = False
        while time.time() < t_end:
            results = self.YOLOmodel(self.perspectiveImg)
            boxes = results[0].boxes
            ids = []
            for box in boxes:
                box = boxes.xyxy
                for c in boxes.cls:
                    ids.append(names[int(c)])
            if len(ids) != 0:
                Detected = True
                break

        if not Detected:
            print("ERROR: No cube detected.")
            return RESULT

        # 选 sticker 或 cube 类别的框作为 ROI（整个方块的外接框）
        for box in boxes:
            name = names[int(box.cls[0])]
            if name == "sticker" or name == "cube":
                x1, y1, x2, y2 = box.xyxy[0]
                break

        # ROI 向外扩 3 个像素，防止贴边
        x = int(x1) - 3
        y = int(y1) - 3
        w = int(x2) + 3
        h = int(y2) + 3
        ROI = self.perspectiveImg[y:h, x:w]

        # ---- 在 ROI 上做精细的轮廓检测 ----
        ROI = cv2.GaussianBlur(ROI, (3, 3), 0)
        imgHSV = cv2.cvtColor(ROI, cv2.COLOR_BGR2HSV)
        lowLimit  = (0, 10, 10)
        upLimit   = (70, 255, 255)
        mask = cv2.inRange(imgHSV, lowLimit, upLimit)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        poly_contour = []
        for contour in contours:
            if cv2.contourArea(contour) > 500:
                poly_contour.append(contour)

        if not poly_contour:
            print("ERROR: No contours detected.")
            return RESULT

        # 取面积最大的轮廓
        cnt = max(poly_contour, key=cv2.contourArea)

        # minAreaRect: 拟合最小外接旋转矩形
        # (xo,yo) 是中心, (wo,ho) 是宽高, angle 是旋转角(度)
        rect = cv2.minAreaRect(cnt)
        (xo, yo), (wo, ho), angle = rect

        # 把 ROI 局部坐标换回整幅鸟瞰图的坐标
        xo = xo + x
        yo = yo + y

        print(f"(Contour Debug): center=({xo:.2f}, {yo:.2f}), angle={angle:.2f}")

        # yaw: 角度转弧度，加负号是因为鸟瞰图坐标轴和 world 坐标轴有 90° 错位
        RESULT["yaw"] = -(angle * 3.1416 / 180.0)

        # ====== STEP 1: 鸟瞰图中心 (xo,yo) -> 原始图像素 (u,v) ======
        # 用逆透视矩阵把鸟瞰图坐标映回原始相机图
        # 齐次坐标: [xo, yo, 1] 乘 H_inv 后第三维不再是 1，要除以它归一
        p_perspective = np.array([xo, yo, 1.0], dtype=np.float64)
        p_raw = self.invPerspTransMatrix @ p_perspective
        p_raw = p_raw / p_raw[2]   # 齐次坐标透视除法
        u = float(p_raw[0])
        v = float(p_raw[1])

        print(f"(Perspective -> Raw): xo={xo:.2f}, yo={yo:.2f} -> u={u:.2f}, v={v:.2f}")

        # ====== STEP 2: 原始像素 (u,v) -> world 坐标 ======
        # 调用核心函数：射线 + z=0.88 平面求交
        base_x, base_y, base_z = self.PixelToWorldPlane(u, v, z_plane=0.88)

        RESULT["x"] = base_x
        RESULT["y"] = base_y

        # ---- 老方案留作对比（调试用）----
        old_x = yo/1000 + 0.35
        old_y = xo/1000 + 0.15
        print(f"(OLD vs TF2): old=({old_x:.4f}, {old_y:.4f})  "
              f"tf2=({RESULT['x']:.4f}, {RESULT['y']:.4f})")

        # ---- 判断方块颜色类别 ----
        RESULT["detection"] = "Cube"
        RESULT["success"] = True
        for id in ids:
            if id == "white":  RESULT["detection"] = "WhiteCube"; break
            if id == "black":  RESULT["detection"] = "BlackCube"; break
            if id == "blue":   RESULT["detection"] = "BlueCube";  break
            if id == "sticker":RESULT["detection"] = "Sticker";   break

        return RESULT

    # ---- 连续检测颜色（用于分拣时二次确认）----
    def DetectColour(self):
        global Gz_CAM, ENVIRONMENT
        T = time.time() + 1.0
        StickerCount = WhiteCount = BlackCount = BlueCount = 0
        while time.time() <= T:
            if ENVIRONMENT == "GAZEBO":
                rclpy.spin_once(self.GzCAM_SUB)
                self.inputImg = Gz_CAM
            else:
                self.ret, self.inputImg = self.camera.read()
            if self.inputImg is not None:
                results = self.YOLOmodel(self.inputImg)
                names = self.YOLOmodel.names
                boxes = results[0].boxes
                ids = []
                for box in boxes:
                    box = boxes.xyxy
                    for c in boxes.cls:
                        ids.append(names[int(c)])
                for ID in ids:
                    if ID == "sticker": StickerCount += 1
                    elif ID == "white": WhiteCount += 1
                    elif ID == "black": BlackCount += 1
                    elif ID == "blue":  BlueCount += 1

        if WhiteCount == BlueCount == BlackCount == 0 and StickerCount > 150:
            COLOUR = "Cube"
        elif WhiteCount > 3 and WhiteCount > BlueCount and WhiteCount > BlackCount:
            COLOUR = "WhiteCube"
        elif BlueCount > BlackCount and BlueCount > WhiteCount and BlueCount > 3:
            COLOUR = "BlueCube"
        elif BlackCount > BlueCount and BlackCount > WhiteCount and BlackCount > 3:
            COLOUR = "BlackCube"
        else:
            COLOUR = "Cube"
        print(f"(ColourDetection): {COLOUR}")
        return COLOUR

    # ---- 持续可视化 YOLO 输出 ----
    def ConstantVisualization(self):
        global Gz_CAM, ENVIRONMENT
        while True:
            if ENVIRONMENT == "GAZEBO":
                rclpy.spin_once(self.GzCAM_SUB)
                self.inputImg = Gz_CAM
            else:
                self.ret, self.inputImg = self.camera.read()
            if self.inputImg is not None:
                results = self.YOLOmodel(self.inputImg)
                annotated_frame = results[0].plot()
                VISUALIZE = cv2.resize(annotated_frame, (1280, 720))
                cv2.imshow("YOLO Output", VISUALIZE)
            key = cv2.waitKey(1)
            if key == ord('e'):
                cv2.destroyWindow("YOLO Output")
                break
