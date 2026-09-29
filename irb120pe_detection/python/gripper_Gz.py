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
#  under the License is distributed on an "as‑is" basis, without warranties or          #
#  conditions of any kind, either express or implied. See the License for the specific  #
#  language governing permissions and limitations under the License.                    #
#                                                                                       #
#  IFRA Group - Cranfield University                                                    #
#  AUTHORS: Mikel Bueno Viso         - Mikel.Bueno‑Viso@cranfield.ac.uk                 #
#           Irene Bernardino Sanchez - i.bernardinosanchez.854@cranfield.ac.uk          #
#           Seemal Asif              - s.asif@cranfield.ac.uk                           #
#           Phil Webb                - p.f.webb@cranfield.ac.uk                         #
#                                                                                       #
#  Date: November, 2023.                                                                #
#                                                                                       #
# ===================================== COPYRIGHT ===================================== #
# ======= CITE OUR WORK ======= #
# You can cite our work with the following statement:
# IFRA‑Cranfield (2023). Object Detection and Pose Estimation within a Robot Cell. URL: https://github.com/IFRA‑Cranfield/irb120_PoseEstimation

# gripper_Gz.py
# 本文件功能：Gazebo仿真中控制Schunk EGP‑64夹爪张开、闭合；
# 抓取物体时调用LinkAttacher插件，**把方块物理绑定到夹爪上**（仿真模拟抓取，不然方块会直接掉落）

# ===== IMPORT REQUIRED COMPONENTS ===== #
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import time

# ROS2 Action接口：/Move 动作，用来控制夹爪开合
from ros2srrc_data.action import Move
# LinkAttacher 服务：实现两个模型之间物理绑定/解绑（仿真抓取核心）
from linkattacher_msgs.srv import AttachLink
from linkattacher_msgs.srv import DetachLink

# ROS消息类型
from std_msgs.msg import String
from ros2srrc_data.msg import Action
from objectpose_msgs.msg import ObjectPose    # 方块位姿消息
from linkpose_msgs.msg import LinkPose        # 机械臂末端夹爪位姿消息

# ===================== 全局变量 =====================
# CUBES字典：保存4种方块实时位姿 x,y,z + 四元数qx,qy,qz,qw
CUBES = {}
CUBES["WhiteCube"] = {"x": 0.0, "y": 0.0, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 0.0}
CUBES["BlackCube"] = {"x": 0.0, "y": 0.0, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 0.0}
CUBES["BlueCube"] = {"x": 0.0, "y": 0.0, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 0.0}
CUBES["Cube"] = {"x": 0.0, "y": 0.0, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 0.0}

# dataclass 数据类，保存抓取绑定状态
from dataclasses import dataclass
@dataclass
class AttDetCHECK:
    ATTACHED: bool      # True=已经绑定抓取物体
    MODEL: String        # 当前抓取的物体模型名字
    LINK: String         # 物体的link名称
AttachCheck = AttDetCHECK(False, "", "")

# 保存机械臂动作执行返回结果
@dataclass
class RobotRES:
    MESSAGE: String
    SUCCESS: bool
RES = RobotRES("null", False)

# EE_POSE：保存机械臂末端夹爪实时位姿
EE_POSE = LinkPose()

# 测试统计：统计夹爪运动成功/失败次数
TESTING = {"EEMoveOK": 0, "EEMoveNoOK": 0}

# =============================================================================== #
# /Move ACTION CLIENT：ROS2 Action客户端，发送夹爪开合指令
class MoveCLIENT(Node):
    def __init__(self):
        super().__init__('GzGripper_Move_Client')
        # 创建Action客户端，连接/Move动作服务器
        self._action_client = ActionClient(self, Move, 'Move')
        print("(/Move)‑Gripper: Initialising ROS2 Action Client!")
        print("(/Move)‑Gripper: Waiting for /Move ROS2 ActionServer to be available...")
        self._action_client.wait_for_server()   # 阻塞等待服务端启动
        print("(/Move)‑Gripper: /Move ACTION SERVER detected.")

    def send_goal(self, ACTION):
        """发送运动目标，ACTION是ros2srrc_data/Action消息"""
        goal_msg = Move.Goal()
        goal_msg.action = ACTION.action
        goal_msg.speed = ACTION.speed
        goal_msg.moveg = ACTION.moveg
        
        # 异步发送Action目标
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Action服务器回复：接收/拒绝目标"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            print('(/Move)‑Gripper: Move ACTION CALL → GOAL has been REJECTED.')
            return
        # 目标被接受，等待动作执行完成，获取最终结果
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """动作执行完毕回调，把结果存入全局RES"""
        global RES
        RESULT = future.result().result
        RES.MESSAGE = RESULT.result
        if "FAILED" in RES.MESSAGE:
            RES.SUCCESS = False
        else:
            RES.SUCCESS = True

# =============================================================================== #
# CubePose SUBSCRIBER：订阅各个cube的位姿话题，实时更新CUBES全局字典
class CubePose(Node):
    def __init__(self):
        super().__init__("irb120pe_CubePose_Subscriber")
        # 订阅4个方块各自的ObjectPose话题
        self.SUBWhite = self.create_subscription(ObjectPose, "/WhiteCube/ObjectPose", self.CALLBACK_FN, 10)
        self.SUBBlack = self.create_subscription(ObjectPose, "/BlackCube/ObjectPose", self.CALLBACK_FN, 10)
        self.SUBBlue = self.create_subscription(ObjectPose, "/BlueCube/ObjectPose", self.CALLBACK_FN, 10)
        self.SUBCube = self.create_subscription(ObjectPose, "/Cube/ObjectPose", self.CALLBACK_FN, 10)

    def CALLBACK_FN(self, OBJ):
        """收到方块位姿消息，更新全局CUBES字典"""
        global CUBES
        for Cube, Pose in CUBES.items():
            if (OBJ.objectname == Cube):
                Pose["x"] = OBJ.x
                Pose["y"] = OBJ.y
                Pose["z"] = OBJ.z
                Pose["qx"] = OBJ.qx
                Pose["qy"] = OBJ.qy
                Pose["qz"] = OBJ.qz
                Pose["qw"] = OBJ.qw

# =============================================================================== #
# EEPose SUBSCRIBER：订阅机械臂末端夹爪位姿话题
class EEPose(Node):
    def __init__(self):
        super().__init__("irb120pe_EEPose_Subscriber")
        self.SUB_EE = self.create_subscription(LinkPose, "/LinkPose_irb120_EE_egp64", self.CALLBACK_FN, 10)

    def CALLBACK_FN(self, EE):
        """收到夹爪位姿，赋值给全局EE_POSE"""
        global EE_POSE
        EE_POSE = EE

# =============================================================================== #
# LinkAttacher SERVICE CLIENT：【仿真抓取关键】实现模型绑定与解绑
# Gazebo物理仿真本身没有摩擦力，单纯闭合夹爪抓不住方块，方块会直接掉落。
# LinkAttacher插件作用：把cube方块link 和 夹爪link做**刚性绑定**，方块跟随夹爪一起运动。
class LinkAttacher_Client(Node):
    def __init__(self):
        super().__init__("irb120pe_LinkAttacher_Client")
        # 创建绑定、解绑两个服务客户端
        self.AttachClient = self.create_client(AttachLink, "/ATTACHLINK")
        self.DetachClient = self.create_client(DetachLink, "/DETACHLINK")
        # 等待绑定服务上线
        while not self.AttachClient.wait_for_service(timeout_sec=1.0): 
            print("(LinkAttacher): /ATTACHLINK ROS2 Service not still available, waiting...")
        print("(LinkAttacher): /ATTACHLINK ROS2 Service ready.")
        # 等待解绑服务上线
        while not self.DetachClient.wait_for_service(timeout_sec=1.0): 
            print("(LinkAttacher): /DETACHLINK ROS2 Service not still available, waiting...")
        print("(LinkAttacher): /DETACHLINK ROS2 Service ready.")
        self.AttachRequest = AttachLink.Request()
        self.DetachRequest = DetachLink.Request()

    def ATTACHService(self, MODEL, LINK):
        """绑定：irb120机械臂末端EE_egp64 与 方块模型link刚性连接"""
        self.AttachRequest.model1_name = "irb120"
        self.AttachRequest.link1_name = "EE_egp64"
        self.AttachRequest.model2_name = MODEL
        self.AttachRequest.link2_name = LINK
        self.AttachFuture = self.AttachClient.call_async(self.AttachRequest)

    def DETACHService(self, MODEL, LINK):
        """解绑：切断夹爪和方块之间刚性绑定，方块自由，实现松开放下物体"""
        self.DetachRequest.model1_name = "irb120"
        self.DetachRequest.link1_name = "EE_egp64"
        self.DetachRequest.model2_name = MODEL
        self.DetachRequest.link2_name = LINK
        self.DetachFuture = self.DetachClient.call_async(self.DetachRequest)

# 上层封装类，给main_Gz调用，自旋等待服务返回结果
class LinkAttacher():
    def __init__(self):
        self.CLIENT = LinkAttacher_Client()

    def ATTACH(self, MODEL, LINK):
        """执行绑定，返回True绑定成功 / False失败"""
        global AttachCheck
        self.CLIENT.ATTACHService(MODEL,LINK)
        while rclpy.ok():
            rclpy.spin_once(self.CLIENT)
            if self.CLIENT.AttachFuture.done():
                try:
                    AttachRES = self.CLIENT.AttachFuture.result()
                except Exception as exc:
                    print("(LinkAttacher): /ATTACHLINK Service call failed → " + str(exc))
                    return(False)
                else:
                    if (AttachRES.success):
                        print("(LinkAttacher): /ATTACHLINK successful → " + str(AttachRES.message))
                        AttachCheck.ATTACHED = True
                        AttachCheck.MODEL = MODEL
                        AttachCheck.LINK = LINK
                        return(True)
                    else:
                        print("(LinkAttacher): /ATTACHLINK unuccessful → " + str(AttachRES.message))
                        return(False)
                    
    def DETACH(self, MODEL, LINK):
        """执行解绑，返回True解绑成功 / False失败"""
        global AttachCheck
        self.CLIENT.DETACHService(MODEL,LINK)
        while rclpy.ok():
            rclpy.spin_once(self.CLIENT)
            if self.CLIENT.DetachFuture.done():
                try:
                    DetachRES = self.CLIENT.DetachFuture.result()
                except Exception as exc:
                    print("(LinkAttacher): /DETACHLINK Service call failed → " + str(exc))
                    return(False)
                else:
                    if (DetachRES.success):
                        print("(LinkAttacher): /DETACHLINK successful → " + str(DetachRES.message))
                        AttachCheck.ATTACHED = False
                        AttachCheck.MODEL = ""
                        AttachCheck.LINK = ""
                        return(True)
                    else:
                        print("(LinkAttacher): /DETACHLINK unuccessful → " + str(DetachRES.message))
                        return(False)

# =============================================================================== #
# GzGripper 对外接口类，main_Gz.py直接调用 self.GRIPPER.CLOSE() / self.GRIPPER.OPEN()
class GzGripper():
    def __init__(self):
        self.MoveClient = MoveCLIENT()                 # 夹爪动作客户端
        self.CubeLocation = CubePose()                 # 订阅方块位姿
        self.EEPose = EEPose()                         # 订阅夹爪末端位姿
        self.LinkAttacher = LinkAttacher()             # 绑定解绑工具

    def CLOSE(self):
        """夹爪闭合，仿真抓取逻辑"""
        WP = Action()
        WP.action = "MoveG"        # MoveG代表夹爪运动
        WP.speed = 1.0
        WP.moveg = 0.008           # 夹爪闭合缝隙0.008m，闭合状态
        self.MoveClient.send_goal(WP)

        # 自旋等待动作执行完成，RES.MESSAGE不为null代表返回结果
        while rclpy.ok():
            rclpy.spin_once(self.MoveClient)
            if (RES.MESSAGE != "null"):
                break

        if (RES.SUCCESS == True):
            print("Result → " + RES.MESSAGE)
            TESTING["EEMoveOK"] = TESTING["EEMoveOK"] + 1
            print(TESTING)
            print()
            RES.MESSAGE = "null"
            RES.SUCCESS = False
        elif (RES.SUCCESS == False):
            print("Result → " + RES.MESSAGE)
            TESTING["EEMoveNoOK"] = TESTING["EEMoveNoOK"] + 1
            print(TESTING)
            print("")
            rclpy.shutdown()
            print("CLOSING PROGRAM...")
            exit()

        # ========= 抓取判断逻辑 =========
        global CUBES
        global EE_POSE
        # 自旋0.1秒，刷新获取最新方块、夹爪位姿
        t_end = time.time() + 0.1
        while time.time() < t_end:
            rclpy.spin_once(self.CubeLocation)
            rclpy.spin_once(self.EEPose)

        # 遍历字典，找到场景中存在的cube（坐标不全为0代表真实存在）
        OBJ = ObjectPose()
        Found = False
        for Cube, Pose in CUBES.items():
            for VALUE in Pose.values():
                if VALUE != 0.0:
                    Found = True
                    break
            if Found:
                OBJ.objectname = Cube
                OBJ.x = Pose["x"]
                OBJ.y = Pose["y"]
                OBJ.z = Pose["z"]
                OBJ.qx = Pose["qx"]
                OBJ.qy = Pose["qy"]
                OBJ.qz = Pose["qz"]
                OBJ.qw = Pose["qw"]
                break
        if not Found:
            print("(GzGripper): Gripper closed without grasping any object.")
            print("")
            return()
        
        # 判断：方块坐标和夹爪末端坐标是否接近（误差±0.01m），判定夹爪确实包住方块
        CheckATT = True
        if (EE_POSE.x - 0.01 > OBJ.x) or (EE_POSE.x + 0.01 < OBJ.x):
            CheckATT = False
        if (EE_POSE.y - 0.01 > OBJ.y) or (EE_POSE.y + 0.01 < OBJ.y):
            CheckATT = False
        if (EE_POSE.z - 0.01 > OBJ.z) or (EE_POSE.z + 0.01 < OBJ.z):
            CheckATT = False

        # 坐标接近 → 执行LinkAttacher刚性绑定，模拟抓住物体
        if CheckATT:
            AttRES = self.LinkAttacher.ATTACH(OBJ.objectname, OBJ.objectname)
            if AttRES:
                print("(GzGripper): Gripper closed, object→" + str(OBJ.objectname) + " attached.")
                print("")
            else: 
                print("(GzGripper): Gripper closed, object→" + str(OBJ.objectname) + " not attached, LinkAttacher did not work properly.")
                print("")
        else:
            print("(GzGripper): Gripper closed, object→" + str(OBJ.objectname) + " not attached, gripper and object poses don't match.")
            print("")

    def OPEN(self):
        """夹爪张开，解绑物体，实现放下方块"""
        WP = Action()
        WP.action = "MoveG"
        WP.speed = 1.0
        WP.moveg = 0.0         # moveg=0 夹爪完全张开
        self.MoveClient.send_goal(WP)

        while rclpy.ok():
            rclpy.spin_once(self.MoveClient)
            if (RES.MESSAGE != "null"):
                break

        if (RES.SUCCESS == True):
            print("Result → " + RES.MESSAGE)
            TESTING["EEMoveOK"] = TESTING["EEMoveOK"] + 1
            print(TESTING)
            print()
            RES.MESSAGE = "null"
            RES.SUCCESS = False
        elif (RES.SUCCESS == False):
            print("Result → " + RES.MESSAGE)
            TESTING["EEMoveNoOK"] = TESTING["EEMoveNoOK"] + 1
            print(TESTING)
            print("")
            rclpy.shutdown()
            print("CLOSING PROGRAM...")
            exit()

        # 如果当前已经绑定物体，执行解绑；解绑后方块脱离夹爪，落到桌面上
        global AttachCheck 
        MODELname = AttachCheck.MODEL
        if AttachCheck.ATTACHED:
            DetRES = self.LinkAttacher.DETACH(AttachCheck.MODEL, AttachCheck.LINK)
            if DetRES:
                print("(GzGripper): Gripper opened, object→" + str(MODELname) + " detached.")
                print("")
            else: 
                print("(GzGripper): Gripper opened, object→" + str(MODELname) + " not detached, LinkAttacher did not work properly.")
                print("")
        else:
            print("(GzGripper): Gripper opened without dropping any object.")
            print("")
