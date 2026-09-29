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
# robot.py
# 本类用于执行机器人运动，调用下面两个ROS2 Action服务：
#   - /Robmove：控制机器人运动到指定末端执行器位姿
#   - /Move：执行预定义机器人运动，支持笛卡尔空间、关节空间、单关节、旋转等运动模式
# ===== IMPORT REQUIRED COMPONENTS ===== #
# 导入ROS2核心库
import rclpy
# 导入ROS2节点基类
from rclpy.node import Node
# 导入Action客户端，用于调用ROS2 Action服务
from rclpy.action import ActionClient
# 导入/Move和/RobMove对应的ROS2 Action自定义消息
from ros2srrc_data.action import Move
from ros2srrc_data.action import Robmove
# 导入ROS2基础消息类型
from std_msgs.msg import String
# 导入Action动作描述消息
from ros2srrc_data.msg import Action
# 导入位姿消息，包含位置+四元数姿态
from geometry_msgs.msg import Pose
# 导入dataclass用于定义结果返回结构体
from dataclasses import dataclass
# 定义机器人动作返回结果数据类，存储返回信息与成功标记
@dataclass
class RobotRES:
    MESSAGE: String  # 动作返回文本信息
    SUCCESS: bool    # 动作是否执行成功
# 全局变量RES，用于跨回调函数保存Action执行结果
RES = RobotRES("null", False)
# 导入航点类waypoints，用于读取预定义位姿
from waypoints import waypoints
# 测试统计字典，记录各类动作成功/失败次数
TESTING = {"MoveOK": 0, "MoveNoOK": 0, "RobMoveOK": 0, "RobMoveNoOK": 0}
# =============================================================================== #
# /RobMove ACTION CLIENT：/Robmove动作客户端类
class RobMoveCLIENT(Node):
    # 构造函数，创建RobMove客户端节点
    def __init__(self):
        # 调用父类Node构造，设置节点名称
        super().__init__('irb120pe_RobMove_Client')
        # 创建Action客户端，绑定Robmove动作，服务名称为Robmove
        self._action_client = ActionClient(self, Robmove, 'Robmove')
        print("(/RobMove): Initialising ROS2 Action Client!")
        print("(/RobMove): Waiting for /Robmove ROS2 ActionServer to be available...")
        # 阻塞等待Robmove服务端上线
        self._action_client.wait_for_server()
        print("(/RobMove): /Robmove ACTION SERVER detected.")
    # 发送运动目标请求：TYPE运动类型，SPEED速度，TARGET_POSE目标末端位姿
    def send_goal(self, TYPE, SPEED, TARGET_POSE):
        # 实例化Robmove目标消息
        goal_msg = Robmove.Goal()
        # 设置运动类型（PTP/LIN等）
        goal_msg.type = TYPE
        # 设置运动速度
        goal_msg.speed = SPEED
        # 目标位置X坐标
        goal_msg.x = TARGET_POSE.position.x
        # 目标位置Y坐标
        goal_msg.y = TARGET_POSE.position.y
        # 目标位置Z坐标
        goal_msg.z = TARGET_POSE.position.z
        # 姿态四元数qx
        goal_msg.qx = TARGET_POSE.orientation.x
        # 姿态四元数qy
        goal_msg.qy = TARGET_POSE.orientation.y
        # 姿态四元数qz
        goal_msg.qz = TARGET_POSE.orientation.z
        # 姿态四元数qw
        goal_msg.qw = TARGET_POSE.orientation.w
        # 异步发送目标请求，返回future对象
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        # 注册回调函数，目标被服务端接收/拒绝时触发
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    # 目标响应回调函数，接收服务端是否接受目标
    def goal_response_callback(self, future):
        # 获取目标句柄
        goal_handle = future.result()
        # 如果目标被拒绝
        if not goal_handle.accepted:
            print('(/RobMove): RobMove ACTION CALL -> GOAL has been REJECTED.')
            return
        # 异步获取动作最终执行结果
        self._get_result_future = goal_handle.get_result_async()
        # 注册结果回调，动作执行完成后触发
        self._get_result_future.add_done_callback(self.get_result_callback)
    # 获取动作执行结果的回调函数
    def get_result_callback(self, future):
        # 声明使用全局RES变量
        global RES
        # 取出返回结果
        RESULT = future.result().result
        # 将返回消息存入全局结果
        RES.MESSAGE = RESULT.message
        # 将成功标记存入全局结果
        RES.SUCCESS = RESULT.success
# =============================================================================== #
# /Move ACTION CLIENT：/Move动作客户端类
class MoveCLIENT(Node):
    # 构造函数，创建Move客户端节点
    def __init__(self):
        # 设置节点名称
        super().__init__('irb120pe_Move_Client')
        # 创建Action客户端，绑定Move动作，服务名Move
        self._action_client = ActionClient(self, Move, 'Move')
        print("(/Move): Initialising ROS2 Action Client!")
        print("(/Move): Waiting for /Move ROS2 ActionServer to be available...")
        # 阻塞等待Move服务端上线
        self._action_client.wait_for_server()
        print("(/Move): /Move ACTION SERVER detected.")
    # 发送Move动作目标，参数ACTION为预定义动作描述
    def send_goal(self, ACTION):
        # 实例化Move目标消息
        goal_msg = Move.Goal()
        # 动作名称
        goal_msg.action = ACTION.action
        # 运动速度
        goal_msg.speed = ACTION.speed
        # 关节运动参数movej
        goal_msg.movej = ACTION.movej
        # 旋转运动参数mover
        goal_msg.mover = ACTION.mover
        # 直线运动参数movel
        goal_msg.movel = ACTION.movel
        # xyzw运动参数movexyzw
        goal_msg.movexyzw = ACTION.movexyzw
        # xyz平移参数movexyz
        goal_msg.movexyz = ACTION.movexyz
        # 旋转参数moverot
        goal_msg.moverot = ACTION.moverot
        # ypr姿态角moveypr
        goal_msg.moveypr = ACTION.moveypr
        # rp姿态角moverp
        goal_msg.moverp = ACTION.moverp
        # 夹爪moveg参数
        goal_msg.moveg = ACTION.moveg
        # 异步发送目标
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        # 注册目标响应回调
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    # Move目标响应回调，判断目标是否被接受
    def goal_response_callback(self, future):
        # 获取目标句柄
        goal_handle = future.result()
        # 目标被拒绝
        if not goal_handle.accepted:
            print('(/Move): Move ACTION CALL -> GOAL has been REJECTED.')
            return
        # 异步获取动作执行结果
        self._get_result_future = goal_handle.get_result_async()
        # 注册结果回调
        self._get_result_future.add_done_callback(self.get_result_callback)
    # Move动作执行结果回调
    def get_result_callback(self, future):
        # 使用全局RES变量
        global RES
        # 获取动作返回结果
        RESULT = future.result().result
        # 保存返回文本
        RES.MESSAGE = RESULT.result
        # 如果返回信息包含FAILED标记，置为失败
        if "FAILED" in RES.MESSAGE:
            RES.SUCCESS = False
        else:
            RES.SUCCESS = True
# =============================================================================== #
# ROBOT类，封装机器人运动执行接口，供上层routines.py调用
class RBT():
    # RBT类初始化
    def __init__(self):
        # 实例化/Move动作客户端
        self.MoveClient = MoveCLIENT()
        # 实例化/RobMove动作客户端
        self.RobMoveClient = RobMoveCLIENT()
        # 实例化航点类，加载预定义位姿
        self.Waypoints = waypoints()
    # 执行预命名位姿的Move动作（调用/Move服务）
    def Move_EXECUTE(self, PoseName):
        # 声明使用全局TESTING统计变量
        global TESTING
        # 定义Action消息对象
        WP = Action()
        # 从waypoints读取指定名称Pose对应的动作参数
        WP = self.Waypoints.RobotPose(PoseName)
        # 发送动作目标给Move客户端
        self.MoveClient.send_goal(WP)
        # ROS2自旋循环，等待Action回调完成
        while rclpy.ok():
            # 单次自旋，处理回调事件
            rclpy.spin_once(self.MoveClient)
            # 如果全局RES消息不再是null，代表动作执行完毕，退出循环
            if (RES.MESSAGE != "null"):
                break
        # 判断动作执行成功
        if (RES.SUCCESS == True):
            # 打印执行结果信息
            print("Result -> " + RES.MESSAGE)
            # Move成功计数+1
            TESTING["MoveOK"] = TESTING["MoveOK"] + 1
            # 打印当前统计字典
            print(TESTING)
            print()
            # 重置全局RES状态，供下一次动作调用
            RES.MESSAGE = "null"
            RES.SUCCESS = False
        # 动作执行失败
        elif (RES.SUCCESS == False):
            # 打印失败信息
            print("Result -> " + RES.MESSAGE)
            # Move失败计数+1
            TESTING["MoveNoOK"] = TESTING["MoveNoOK"] + 1
            print(TESTING)
            print("")
            # 关闭ROS2上下文
            rclpy.shutdown()
            print("CLOSING PROGRAM...")
            # 直接退出程序
            exit()
    # RobMove_EXECUTE：使用预定义航点名调用/Robmove服务
    def RobMove_EXECUTE(self, PoseName, Type, Speed):
        # 定义Pose消息对象
        WP = Pose()
        # 从waypoints读取预定义位姿
        WP = self.Waypoints.RobotPose(PoseName)
        # 发送目标给RobMove客户端
        self.RobMoveClient.send_goal(Type, Speed, WP)
        # 自旋等待动作完成
        while rclpy.ok():
            rclpy.spin_once(self.RobMoveClient)
            if (RES.MESSAGE != "null"):
                break
        # 动作成功
        if (RES.SUCCESS == True):
            print("Result -> " + RES.MESSAGE)
            # RobMove成功计数+1
            TESTING["RobMoveOK"] = TESTING["RobMoveOK"] + 1
            print(TESTING)
            print()
            # 重置全局RES
            RES.MESSAGE = "null"
            RES.SUCCESS = False
        # 动作失败
        elif (RES.SUCCESS == False):
            print("Result -> " + RES.MESSAGE)
            # RobMove失败计数+1
            TESTING["RobMoveNoOK"] = TESTING["RobMoveNoOK"] + 1
            print(TESTING)
            print("")
            rclpy.shutdown()
            print("CLOSING PROGRAM...")
            exit()
    # RobMove_EXECUTE_cstm：自定义位姿调用/Robmove（不读取waypoints，直接传入Pose对象）
    def RobMove_EXECUTE_cstm(self, Type, Speed, Pose):
        # 发送自定义目标位姿到RobMove客户端
        self.RobMoveClient.send_goal(Type, Speed, Pose)
        # 自旋等待动作执行完成
        while rclpy.ok():
            rclpy.spin_once(self.RobMoveClient)
            if (RES.MESSAGE != "null"):
                break
        # 动作成功
        if (RES.SUCCESS == True):
            print("Result -> " + RES.MESSAGE)
            print("")
            TESTING["RobMoveOK"] = TESTING["RobMoveOK"] + 1
            print(TESTING)
            print()
            # 重置全局RES
            RES.MESSAGE = "null"
            RES.SUCCESS = False
        # 动作失败
        elif (RES.SUCCESS == False):
            print("Result -> " + RES.MESSAGE)
            TESTING["RobMoveNoOK"] = TESTING["RobMoveNoOK"] + 1
            print(TESTING)
            print("")
            rclpy.shutdown()
            print("CLOSING PROGRAM...")
            exit()
