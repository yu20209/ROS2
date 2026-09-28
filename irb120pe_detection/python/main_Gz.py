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
# 引用说明：
# IFRA-Cranfield (2023). Object Detection and Pose Estimation within a Robot Cell. 
# URL: https://github.com/IFRA-Cranfield/irb120_PoseEstimation
#
# main_Gz.py
# 立方体检测 + ABB IRB120机械臂位姿估计 仿真场景主脚本
# GAZEBO仿真版本：使用Gazebo内置相机插件采集图像
# 完整任务流程：生成立方体 → 相机检测立方体 → 位姿估计 → 抓取 → 判断立方体颜色/面 → 旋转调整 → 放置立方体
# ===== IMPORT REQUIRED COMPONENTS ===== #
# 导入ROS2核心库
import rclpy
# 通用辅助库
import time
import random
# ===== IMPORT FUNCTIONS ===== #
# 导入项目自定义模块
from spawn import SpawnCube        # Gazebo生成立方体模型
from spawn import DeleteCube       # Gazebo删除场景中的立方体模型
from routines import RoutineList   # 机械臂运动动作库（回零、抓取、放置、旋转立方体等动作封装）
from detection import CubeDetection# 立方体检测与位姿估计类（YOLOv8 + ArUco标定 + 透视变换）
# ===================================================================================== #
# ======================================= MAIN ======================================== #
# ===================================================================================== #
def main(args=None):
    print("")
    print(" --- Cranfield University --- ")
    print("        (c) IFRA Group        ")
    print("")
    print("ABB IRB120: Pose Estimation for CUBE Pick&Place")
    print("Python script -> main_Gz.py")
    print("")

    # 初始化ROS2上下文
    rclpy.init(args=None)

    # 实例化各个功能类
    SPAWN = SpawnCube()                 # 立方体生成器实例
    DELETE = DeleteCube()               # 立方体删除器实例
    ROUTINE = RoutineList("GAZEBO")     # 机械臂动作集，指定运行环境为Gazebo仿真
    print("")

    # Move ROBOT to HomePos:
    # 调用动作库，让IRB120机械臂回到预设Home初始零位
    ROUTINE.HomePos()

    # SPAWN random cube:
    # 可选立方体模型列表：白立方体、黑立方体、蓝立方体、基础立方体
    OPT_cube = ["WhiteCube","BlackCube","BlueCube","Cube"]
    # 随机选中一种立方体模型
    VAR_cube = random.choice(OPT_cube)
    print("Spawning random CUBE at a randome POSE to the Gazebo Environment...")
    # 在Gazebo场景生成立方体，"RANDOM"表示立方体放置位姿随机
    SpawnRES = SPAWN.SPAWN(VAR_cube,"RANDOM")

    # 判断立方体生成是否成功，失败则退出程序
    if (SpawnRES["success"] == False):
        rclpy.shutdown()
        print("CLOSING PROGRAM...")
        exit()  

    # Initalise DETECTION class (+ CAMERA INPUT):
    # 实例化立方体检测类，指定使用Gazebo虚拟相机作为图像输入源
    DETECTION = CubeDetection("GAZEBO")

    # CALIBRATION w/ ARUCOS -> Get PERSPECTIVE IMG:
    # 使用ArUco标记做标定，获取校正后的透视图像（透视变换，矫正图像畸变/透视倾斜）
    # ShowPerspective=True：弹出窗口显示校正后的图像
    DETECTION.GetPerspectiveImg(ShowPerspective=True)

    # Test YOLOv8 detection (UNCOMMENT THIS LINE TO TEST IF THE YOLOv8 MODEL IS DETECTING THE CUBE IN GAZEBO):
    # 【可选】YOLOv8目标检测测试函数，取消注释可单独测试模型能否识别场景中的立方体
    # DETECTION.TestDetection()

    # Obtain CUBE LOCATION:
    # 运行检测+位姿估计，获取立方体在机器人坐标系下的x,y坐标以及yaw偏航角
    DetectRES = DETECTION.CubeLocation()

    # 如果立方体检测/位姿估计失败，删除Gazebo中的立方体，退出程序
    if not DetectRES["success"]:
        DELETE.DELETE(VAR_cube)
        rclpy.shutdown()
        print("CLOSING PROGRAM...")
        exit()

    # PICK cube:
    # 调用机械臂抓取函数，传入检测得到的立方体x,y,yaw，执行抓取动作
    ROUTINE.PickCube(DetectRES["x"],DetectRES["y"],DetectRES["yaw"])

    # INIT -> CubeSide and CubeColour variables:
    CubeSide = ""                # 记录当前朝上的立方体面（前/后/底/侧面）
    CubeColour = DetectRES["detection"] # 从检测结果获取立方体颜色标签

    # Check colour -> TOP FACE:
    # 如果检测标签为Sticker贴纸标记，则调用CheckTop函数重新识别顶面颜色
    if (CubeColour == "Sticker"):
        CubeColour = ROUTINE.CheckTop(DETECTION)
        print(CubeColour)

    # Check colour + face (BOTTOM/FRONT/BACK/SIDE):
    # 如果识别标签为基础Cube，调用CheckCube识别颜色与立方体朝向面
    if (CubeColour == "Cube"):
        CheckRES = ROUTINE.CheckCube(DETECTION)
        CubeColour = CheckRES["COLOUR"]
        CubeSide = CheckRES["SIDE"]
        # 若上一轮识别仍然失败，再调用CheckCubeSides对立方体各个侧面做检测
        if (CubeColour == "Cube"):
            CheckRES = ROUTINE.CheckCubeSides(DETECTION)
            CubeColour = CheckRES["COLOUR"]
            CubeSide = CheckRES["SIDE"] 

    # ROTATE if REQUIRED:
    # 根据识别到的立方体面，决定是否旋转机械臂，调整立方体姿态，使目标面朝上
    if (CubeColour != "Cube"):
        if (CubeSide == "FRONT"):
            ROUTINE.RotateCube(RotBack = False)
        if (CubeSide == "BOTTOM"):
            ROUTINE.RotateCube(RotBack = False)
            ROUTINE.RotateCube(RotBack = False)
        if (CubeSide == "BACK"):
            ROUTINE.RotateCube(RotBack = True)

    # PLACE CUBE:
    # 根据识别到的立方体颜色，放到对应目标放置区域
    ROUTINE.PlaceCube(CubeColour)

    # Finish -> Back to HomePos:
    # 任务完成，机械臂回到初始Home位
    ROUTINE.HomePos()
    time.sleep(3.0)

    # Remove CUBE from workspace, to enable another execution after this (there can only be one cube at a time on top of the workspace for the program to work):
    # 从Gazebo仿真环境删除立方体，保证下一次运行时工作台只有一个立方体
    DELETE.DELETE(VAR_cube)

    print("Program execution successfully finished!")
    print("Closing .py script... Bye!")
    # 关闭ROS2节点资源
    rclpy.shutdown() 

if __name__ == '__main__':
    main()
