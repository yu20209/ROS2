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
# routines.py
# 本脚本封装机械臂的各类运动例程，包括回零、抓取、放置、旋转立方体以及视觉检测姿态调整。
# Python script containing the routines(executions) of the ROBOT:
# ===== IMPORT REQUIRED COMPONENTS ===== #
# 导入时间库，用于运动之间的等待和夹爪操作延时。
import time
# 导入Pose消息类型，用于定义末端执行器的目标位姿。
from geometry_msgs.msg import Pose
# ===== IMPORT FUNCTIONS ===== #
# 导入仿真环境夹爪控制接口。
from gripper_Gz import GzGripper
# 导入真实机器人夹爪/IO控制接口。
from gripper import abbRWS_IO
# 导入机器人运动控制核心类。
from robot import RBT
# 导入末端姿态四元数转换函数。
from spawn import EEQuaternion
# ===================================================================================== #
# ======================================= MAIN ======================================== #
# ===================================================================================== #
# 定义机械臂例程类，统一管理所有运动动作。
class RoutineList():
    # 初始化函数，根据环境选择机器人和夹爪接口。
    def __init__(self, ENVIRONMENT):
        # 创建机器人控制实例。
        self.ROBOT = RBT()
        # 如果是GAZEBO仿真环境，则使用仿真夹爪。
        if (ENVIRONMENT == "GAZEBO"):
            self.GRIPPER = GzGripper()
        # 否则使用真实机器人夹爪。
        else: 
            self.GRIPPER = abbRWS_IO()    
    # 机械臂回到初始零位。
    def HomePos(self):#waypoint文件定义机械臂预先定义好的“动作点”或者“动作参数”。
        print("(Robot Movement -> /Move): HomePos")
        # 调用机器人接口执行预定义HomePos动作。
        self.ROBOT.Move_EXECUTE("HomePos")
    # 执行立方体抓取动作。
    def PickCube(self, x, y, yaw):
        print("===== ROUTINE EXECUTION =====")
        print(" - Picking cube from workspace...")
        print("")
        
        # 根据yaw角计算末端执行器的旋转姿态。
        EEPose = EEQuaternion(yaw)
        # 移动到抓取预接近点，高度高于目标点，避免碰撞。
        print("(Robot Movement -> /RobMove): PickCube_App")
        # 创建目标位姿对象。
        TARGET_POSE = Pose()
        # 设置目标点x坐标。
        TARGET_POSE.position.x = x
        # 设置目标点y坐标。
        TARGET_POSE.position.y = y
        # 设置预接近点z高度。
        TARGET_POSE.position.z = 1.10
        # 设置末端执行器朝向。
        TARGET_POSE.orientation = EEPose.orientation
        # 使用PTP（点到点）运动，速度0.3移动到预接近点。
        self.ROBOT.RobMove_EXECUTE_cstm("PTP",0.3,TARGET_POSE)
        # 移动到实际抓取点。
        print("(Robot Movement -> /RobMove): PickCube")
        TARGET_POSE = Pose()
        TARGET_POSE.position.x = x
        TARGET_POSE.position.y = y
        # 下降到抓取高度。
        TARGET_POSE.position.z = 1.07
        TARGET_POSE.orientation = EEPose.orientation
        # 使用LIN（直线）运动，速度0.1下降到抓取点。
        self.ROBOT.RobMove_EXECUTE_cstm("LIN",0.1,TARGET_POSE)
        # CLOSE GRIPPER:
        # 等待稳定。
        time.sleep(0.2)
        # 关闭夹爪，夹住立方体。
        self.GRIPPER.CLOSE()
        time.sleep(0.2)
        # 再次上升到预接近高度。
        print("(Robot Movement -> /RobMove): PickCube_App")
        TARGET_POSE = Pose()
        TARGET_POSE.position.x = x
        TARGET_POSE.position.y = y
        TARGET_POSE.position.z = 1.10
        TARGET_POSE.orientation = EEPose.orientation
        # 直线上升离开工作台。
        self.ROBOT.RobMove_EXECUTE_cstm("LIN",0.1,TARGET_POSE)
    # 执行立方体旋转动作，用于调整立方体朝向。
    def RotateCube(self, RotBack):
        print("===== ROUTINE EXECUTION =====")
        print(" - Rotating cube 90 degrees...")
        print("")
        # 移动到旋转预接近位置。
        print("(Robot Movement -> /RobMove): RotApp")
        self.ROBOT.RobMove_EXECUTE("RotApp", "PTP", 0.3)
        # 如果RotBack为True，则执行反向旋转路径。
        if (RotBack == True):
            print("(Robot Movement -> /Move): RotBack")
            self.ROBOT.Move_EXECUTE("RotBack")
        # 移动到旋转放置点。
        print("(Robot Movement -> /Move): RotPlace")
        self.ROBOT.Move_EXECUTE("RotPlace")
        # OPEN GRIPPER:
        # 释放立方体，准备旋转。
        time.sleep(0.2)
        self.GRIPPER.OPEN()
        time.sleep(0.2)
        # 如果是反向旋转，继续执行额外调整步骤。
        if (RotBack == True):
            print("(Robot Movement -> /Move): RotZ")
            self.ROBOT.Move_EXECUTE("RotZ")
            print("(Robot Movement -> /Move): RotBack_2")
            self.ROBOT.Move_EXECUTE("RotBack_2")
            print("(Robot Movement -> /Move): RotPlace")
            self.ROBOT.Move_EXECUTE("RotPlace")
        # 重新抓取立方体。
        print("(Robot Movement -> /Move): RotationPick")
        self.ROBOT.Move_EXECUTE("RotationPick")
        print("(Robot Movement -> /Move): RotationPick")
        self.ROBOT.Move_EXECUTE("RotationPick")
        # CLOSE GRIPPER:
        time.sleep(0.2)
        self.GRIPPER.CLOSE()
        time.sleep(0.2)
        # 执行旋转后的位姿调整。
        print("(Robot Movement -> /Move): RotZ")
        self.ROBOT.Move_EXECUTE("RotZ")
        print("(Robot Movement -> /Move): RotationPlace")
        self.ROBOT.Move_EXECUTE("RotationPlace")
        print("(Robot Movement -> /Move): RotationPlace")
        self.ROBOT.Move_EXECUTE("RotationPlace")
        print("(Robot Movement -> /Move): RotX")
        self.ROBOT.Move_EXECUTE("RotX")
        print("(Robot Movement -> /Move): RotationPlace")
        self.ROBOT.Move_EXECUTE("RotationPlace")
        print("(Robot Movement -> /Move): RotPlace2")
        self.ROBOT.Move_EXECUTE("RotPlace2")
        # OPEN GRIPPER:
        # 再次释放，完成一次旋转调整。
        time.sleep(0.2)
        self.GRIPPER.OPEN()
        time.sleep(0.2)
        # 回到旋转等待位置。
        print("(Robot Movement -> /Move): RotZ")
        self.ROBOT.Move_EXECUTE("RotZ")
        
        print("(Robot Movement -> /Move): RotationPick")
        self.ROBOT.Move_EXECUTE("RotationPick")
        print("(Robot Movement -> /Move): RotPlace3")
        self.ROBOT.Move_EXECUTE("RotPlace3")
        # CLOSE GRIPPER:
        # 重新抓取，准备结束旋转。
        time.sleep(0.2)
        self.GRIPPER.CLOSE()
        time.sleep(0.2)
        print("(Robot Movement -> /Move): RotZ")
        self.ROBOT.Move_EXECUTE("RotZ")
    # 根据立方体颜色执行分类放置。
    def PlaceCube(self, CUBE):
        # 如果是白色立方体，放到白色放置区。
        if CUBE == "WhiteCube":
            print("(Robot Movement -> /RobMove): PlaceWHITE_app")
            self.ROBOT.RobMove_EXECUTE("PlaceWHITE_app", "PTP", 0.3)
            print("(Robot Movement -> /RobMove): PlaceWHITE")
            self.ROBOT.RobMove_EXECUTE("PlaceWHITE", "LIN", 0.1)
            # OPEN GRIPPER:
            time.sleep(0.2)
            self.GRIPPER.OPEN()
            time.sleep(0.2)
            print("(Robot Movement -> /RobMove): PlaceWHITE_app")
            self.ROBOT.RobMove_EXECUTE("PlaceWHITE_app", "LIN", 0.1)
            
        # 如果是黑色立方体，放到黑色放置区。
        elif CUBE == "BlackCube":
            print("(Robot Movement -> /RobMove): PlaceBLACK_app")
            self.ROBOT.RobMove_EXECUTE("PlaceBLACK_app", "PTP", 0.3)
            print("(Robot Movement -> /RobMove): PlaceBLACK")
            self.ROBOT.RobMove_EXECUTE("PlaceBLACK", "LIN", 0.1)
            # OPEN GRIPPER:
            time.sleep(0.2)
            self.GRIPPER.OPEN()
            time.sleep(0.2)
            print("(Robot Movement -> /RobMove): PlaceBLACK_app")
            self.ROBOT.RobMove_EXECUTE("PlaceBLACK_app", "LIN", 0.1)
        # 如果是蓝色立方体，放到蓝色放置区。
        elif CUBE == "BlueCube":
            print("(Robot Movement -> /RobMove): PlaceBLUE_app")
            self.ROBOT.RobMove_EXECUTE("PlaceBLUE_app", "PTP", 0.3)
            print("(Robot Movement -> /RobMove): PlaceBLUE")
            self.ROBOT.RobMove_EXECUTE("PlaceBLUE", "LIN", 0.1)
            # OPEN GRIPPER:
            time.sleep(0.2)
            self.GRIPPER.OPEN()
            time.sleep(0.2)
            print("(Robot Movement -> /RobMove): PlaceBLUE_app")
            self.ROBOT.RobMove_EXECUTE("PlaceBLUE_app", "LIN", 0.1)
        # 如果颜色仍为Cube，表示识别失败，放到默认放置区。
        elif CUBE == "Cube":
            
            print("(Robot Movement -> /RobMove): PlaceCUBE_app")
            self.ROBOT.RobMove_EXECUTE("PlaceCUBE_app", "PTP", 0.3)
            print("(Robot Movement -> /RobMove): PlaceCUBE")
            self.ROBOT.RobMove_EXECUTE("PlaceCUBE", "LIN", 0.1)
            # OPEN GRIPPER:
            time.sleep(0.2)
            self.GRIPPER.OPEN()
            time.sleep(0.2)
            print("(Robot Movement -> /RobMove): PlaceCUBE_app")
            self.ROBOT.RobMove_EXECUTE("PlaceCUBE_app", "LIN", 0.1)
    # 将立方体移动到中间检测位置，便于相机重新识别。
    def CubeMid(self):
        print("(Robot Movement -> /RobMove): PlaceMidApp")
        self.ROBOT.RobMove_EXECUTE("PlaceMidApp", "PTP", 0.3)
        print("(Robot Movement -> /RobMove): PlaceMid")
        self.ROBOT.RobMove_EXECUTE("PlaceMid", "LIN", 0.1)
        # OPEN GRIPPER:
        time.sleep(0.2)
        self.GRIPPER.OPEN()
        time.sleep(0.2)
    
    # 检查立方体顶面颜色。
    def CheckTop(self, DETECTION):
        print("===== ROUTINE EXECUTION =====")
        print(" - Checking colour of cube's TOP face...")
        print("")
        # 先移动到中间位置。
        self.CubeMid()
        # 移动到顶面检测抓取点。
        print("(Robot Movement -> /Move): PickTop")
        self.ROBOT.Move_EXECUTE("PickTop")
        print("(Robot Movement -> /Move): PickTop")
        self.ROBOT.Move_EXECUTE("PickTop")
        # CLOSE GRIPPER:
        # 夹取立方体。
        time.sleep(0.2)
        self.GRIPPER.CLOSE()
        time.sleep(0.2)
        # 移动到相机前的检测位姿。
        print("(Robot Movement -> /Move): PickTopApp")
        self.ROBOT.Move_EXECUTE("PickTopApp")
        print("(Robot Movement -> /RobMove): FacePose_4")
        self.ROBOT.RobMove_EXECUTE("FacePose_4", "PTP", 0.3)
        # 调用视觉模块检测顶面颜色。
        COLOUR = DETECTION.DetectColour()
        #DETECTION.TestDetection()
        # 回到重新抓取位置。
        print("(Robot Movement -> /RobMove): RePickTopApp_PREV")
        self.ROBOT.RobMove_EXECUTE("RePickTopApp_PREV", "PTP", 0.3)
        print("(Robot Movement -> /Move): RePickTop_PREV")
        self.ROBOT.Move_EXECUTE("RePickTop_PREV")
        # OPEN GRIPPER:
        time.sleep(0.2)
        self.GRIPPER.OPEN()
        time.sleep(0.2)
        print("(Robot Movement -> /Move): RePickTop")
        self.ROBOT.Move_EXECUTE("RePickTop")
        print("(Robot Movement -> /Move): RePickTop")
        self.ROBOT.Move_EXECUTE("RePickTop")
        # CLOSE GRIPPER:
        time.sleep(0.2)
        self.GRIPPER.CLOSE()
        time.sleep(0.2)
        print("(Robot Movement -> /Move): RePickTopApp")
        self.ROBOT.Move_EXECUTE("RePickTopApp")
        # 返回检测到的颜色。
        return(COLOUR)
    
    # 检查立方体当前朝向和颜色。
    def CheckCube(self, DETECTION):
        # 初始化结果，默认颜色为Cube，侧面为None。
        RESULT = {"COLOUR": "Cube", "SIDE": "None"}
        print("===== ROUTINE EXECUTION =====")
        print(" - Checking sticker location and feature colour...")
        print("")
        # 移动到第一个检测面位置。
        print("(Robot Movement -> /RobMove): FacePose_1")
        self.ROBOT.RobMove_EXECUTE("FacePose_1", "PTP", 0.3)
        # 检测当前朝向的颜色。
        COLOUR = DETECTION.DetectColour()
        # 如果检测到有效颜色，则认为当前面为FRONT。
        if COLOUR != "Cube":
            RESULT["COLOUR"] = COLOUR
            RESULT["SIDE"] = "FRONT"
            return(RESULT)
        
        # 移动到第二个检测面位置。
        print("(Robot Movement -> /Move): FacePose_2")
        self.ROBOT.Move_EXECUTE("FacePose_2")
        COLOUR = DETECTION.DetectColour()
        # 如果检测到有效颜色，则认为当前面为BACK。
        if COLOUR != "Cube":
            RESULT["COLOUR"] = COLOUR
            RESULT["SIDE"] = "BACK"
            return(RESULT)
        
        # 移动到第三个检测面位置。
        print("(Robot Movement -> /RobMove): FacePose_3")
        self.ROBOT.RobMove_EXECUTE("FacePose_3", "PTP", 0.3)
        COLOUR = DETECTION.DetectColour()
        # 回到第一个检测面。
        print("(Robot Movement -> /RobMove): FacePose_1")
        self.ROBOT.RobMove_EXECUTE("FacePose_1", "PTP", 0.3)
        # 如果检测到有效颜色，则认为当前面为BOTTOM。
        if COLOUR != "Cube":
            RESULT["COLOUR"] = COLOUR
            RESULT["SIDE"] = "BOTTOM"
            return(RESULT)
        # 三个面都未识别到有效颜色，返回默认结果。
        return(RESULT)
    
    # 进一步检查立方体侧面。
    def CheckCubeSides(self, DETECTION):
        # 先移动到中间位置。
        self.CubeMid()
        print("(Robot Movement -> /RobMove): PlaceMidApp")
        self.ROBOT.RobMove_EXECUTE("PlaceMidApp", "LIN", 0.3)
        # 移动到侧面预抓取点。
        print("(Robot Movement -> /Move): PickSideApp_1")
        self.ROBOT.Move_EXECUTE("PickSideApp_1")
        print("(Robot Movement -> /Move): PickSide")
        self.ROBOT.Move_EXECUTE("PickSide")
        # CLOSE GRIPPER:
        # 抓取立方体。
        time.sleep(0.2)
        self.GRIPPER.CLOSE()
        time.sleep(0.2)
        # 移动到侧面检测点。
        print("(Robot Movement -> /Move): PickSideApp_2")
        self.ROBOT.Move_EXECUTE("PickSideApp_2")
        # 调用CheckCube进行颜色和侧面识别。
        CheckRES = self.CheckCube(DETECTION)
        # 返回识别结果。
        return(CheckRES)
