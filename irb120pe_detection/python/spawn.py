#!/usr/bin/python3

# spawn.py
# 本脚本作用：在Gazebo仿真环境中生成（生成/投放）立方体cube工件模型

# ===== IMPORT REQUIRED COMPONENTS ===== #
# ROS2核心库，创建节点必备
import rclpy
from rclpy.node import Node
# 导入Gazebo生成模型的ROS2服务接口
from gazebo_msgs.srv import SpawnEntity
# 导入Gazebo删除模型的ROS2服务接口
from gazebo_msgs.srv import DeleteEntity
# ROS2位姿消息，包含位置position + 四元数姿态orientation
from geometry_msgs.msg import Pose
# 文件路径相关，读取urdf模型文件
import os
# 获取ROS功能包的安装路径
from ament_index_python.packages import get_package_share_directory
# xacro解析工具，把xacro模板转为urdf原始xml
import xacro
# 用于生成随机位置、随机姿态，cube每次投放位置不一样
import random
# 数学库，欧拉角<->四元数转换要用
import math

# =============================================================================== #
# CLASS -> SpawnEntityGz (Gazebo):
# 封装【Gazebo生成实体】的ROS2服务客户端，底层调用 /spawn_entity 服务
class SpawnEntityGz(Node):
def __init__(self):
    # 调用父类 Node 的构造函数，初始化 ROS 2 节点，并指定节点名称
    super().__init__('irb120pe_SpawnEntity_Client')

    # 创建 ROS 2 服务客户端，用于向 Gazebo 的 /spawn_entity 服务发送生成模型的请求
    # SpawnEntity：服务类型，定义请求和响应的数据结构，规定请求和响应应该有哪些字段。
    # /spawn_entity：服务名称，由 Gazebo 提供对应的服务端，指定请求发送给哪个服务。
    # self.cli_SPAWN：保存创建的服务客户端对象
    self.cli_SPAWN = self.create_client(SpawnEntity, "/spawn_entity")

    # 创建 SpawnEntity 服务的请求对象，用于存放待生成模型的名称、XML 描述和初始位姿等信息
    # 此处仅创建请求对象，尚未向 Gazebo 发送请求
    self.req_SPAWN = SpawnEntity.Request()

    def SPAWN(self, CUBE, POSE):
        """
        向Gazebo发送请求，生成cube模型
        :param CUBE: cube模型名字符串，例如 cube_blue
        :param POSE: geometry_msgs/Pose 消息，物体生成的位置姿态
        """
        # 拼接urdf文件名字
        urdf_file = CUBE + ".urdf"
        # 拼接完整文件路径：irb120pe_gazebo包下 urdf/cube/xxx.urdf
        urdf_file_path = os.path.join(get_package_share_directory('irb120pe_gazebo'), 'urdf', 'cube', urdf_file)
        # xacro处理，传入name参数，解析得到urdf文档对象
        xacro_file = xacro.process_file(urdf_file_path, mappings={"name": CUBE})

        # 填充服务请求参数：物体名称
        self.req_SPAWN.name = CUBE
        # 把解析好的urdf转为xml字符串，传给Gazebo
        self.req_SPAWN.xml = xacro_file.toxml()

        # 设置生成物体的初始位置
        self.req_SPAWN.initial_pose.position.x = POSE.position.x
        self.req_SPAWN.initial_pose.position.y = POSE.position.y
        self.req_SPAWN.initial_pose.position.z = POSE.position.z
        # 设置生成物体的初始姿态（四元数）
        self.req_SPAWN.initial_pose.orientation.x = POSE.orientation.x
        self.req_SPAWN.initial_pose.orientation.y = POSE.orientation.y
        self.req_SPAWN.initial_pose.orientation.z = POSE.orientation.z
        self.req_SPAWN.initial_pose.orientation.w = POSE.orientation.w

        # 异步调用ROS2服务，非阻塞，返回future等待结果。self.req_SPAWN：要发送的数据
        self.future_SPAWN = self.cli_SPAWN.call_async(self.req_SPAWN)

# =============================================================================== #
# CLASS -> SpawnCube:
# 对外提供的生成Cube工具类，上层main_Gz.py调用这个类，不用直接操作底层服务
class SpawnCube():
    def __init__(self):
        # 实例化底层Gazebo生成实体客户端对象
        self.SpawnEntity = SpawnEntityGz()

    def SPAWN(self, CUBE, ORIENTATION):
        """
        生成cube对外接口
        :param CUBE: cube名称
        :param ORIENTATION: 姿态模式：RANDOM / FRONT / BOTTOM / TOP
        :return: dict字典，包含生成的x,y,yaw角度，success标记是否生成成功
        """
        POSE = Pose()

        # 设置cube生成的XY随机范围，Z固定高度，从空中落下来
        POSE.position.x = random.uniform(0.47,0.63)
        POSE.position.y = random.uniform(0.27,0.78)
        POSE.position.z = 0.88

        # 根据传入模式，设置欧拉角 roll pitch yaw
        if (ORIENTATION == "RANDOM"):
            # 完全随机姿态，三个轴全部随机 -pi ~ +pi
            roll = random.uniform(-3.1416,3.1416)
            pitch = random.uniform(-3.1416,3.1416)
            yaw = random.uniform(-3.1416,3.1416)
        elif (ORIENTATION == "FRONT"):
            # ArUco标记朝前，pitch=90度(1.5708rad)，yaw小范围随机
            roll = 0.0
            pitch = 1.5708
            yaw = random.uniform(-0.7854,0.7854)
        elif (ORIENTATION == "BOTTOM"):
            # ArUco标记朝下，pitch=180度(3.1416rad)
            roll = 0.0
            pitch = 3.1416
            yaw = random.uniform(-0.7854,0.7854)
        elif (ORIENTATION == "TOP"):
            # ArUco标记朝上，pitch=0
            roll = 0.0
            pitch = 0.0
            yaw = random.uniform(0.0,1.5708)

        # 欧拉角(roll,pitch,yaw)转换为四元数，赋值给pose姿态
        OrientationRES = EulerToQuat(roll,pitch,yaw)
        POSE.orientation.x = OrientationRES.orientation.x
        POSE.orientation.y = OrientationRES.orientation.y
        POSE.orientation.z = OrientationRES.orientation.z
        POSE.orientation.w = OrientationRES.orientation.w

        # 返回结果字典，保存cube生成出来的位置yaw，默认success=False失败
        #生成时的理论初始位姿
        RETURN = dict()
        RETURN["x"] = POSE.position.x
        RETURN["y"] = POSE.position.y
        RETURN["yaw"] = yaw
        RETURN["success"] = False

        # 调用底层，向Gazebo发送生成请求
        self.SpawnEntity.SPAWN(CUBE, POSE)

        # 循环等待ROS2异步服务返回结果
        while rclpy.ok():
            # 执行一次节点自旋，处理回调、等待服务应答
            rclpy.spin_once(self.SpawnEntity)
            # 判断服务调用是否完成
            if self.SpawnEntity.future_SPAWN.done():
                try:
                    # 获取服务返回结果
                    spawnRES = self.SpawnEntity.future_SPAWN.result()
                except Exception as exc:
                    # 捕获异常，服务调用出错，打印错误信息，返回失败字典
                    print("(SpawnCube): /SpawnEntity ROS2 Service call failed. ERROR: " + str(exc))
                    print("")
                    return(RETURN)
                else:
                    if (spawnRES.success):
                        # Gazebo返回成功，修改success标记，返回位置信息
                        print("(SpawnCube): Successful. RESULT -> " + str(spawnRES.status_message))
                        print("")
                        RETURN["success"] = True
                        return(RETURN)
                    else:
                        # Gazebo内部执行失败，返回失败
                        print("(SpawnCube): Failed. RESULT -> " + str(spawnRES.status_message))
                        print("")
                        return(RETURN)

# =============================================================================== #
# CLASS -> DeleteEntityGz (Gazebo):
# Gazebo删除模型的ROS2服务客户端，底层调用 /delete_entity
class DeleteEntityGz(Node):
    def __init__(self):
        # 创建删除实体的ROS2节点
        super().__init__('irb120pe_DeleteEntity_Client')
        # 创建/delete_entity服务客户端
        self.cli_DELETE = self.create_client(DeleteEntity, "/delete_entity")
        # 实例化删除请求消息
        self.req_DELETE = DeleteEntity.Request()

    def DELETE(self, CUBE):
        """发送删除cube的请求"""
        # 设置要删除物体的名字
        self.req_DELETE.name = CUBE
        # 异步调用删除服务
        self.future_DELETE = self.cli_DELETE.call_async(self.req_DELETE)

# =============================================================================== #
# CLASS -> DeleteCube:
# 对外提供删除cube工具类，上层main_Gz.py调用
class DeleteCube():
    def __init__(self):
        self.DeleteEntity = DeleteEntityGz()

    def DELETE(self, CUBE):
        """
        删除Gazebo中指定cube
        :param CUBE: cube模型名称
        :return: True删除成功 / False删除失败
        """
        self.DeleteEntity.DELETE(CUBE)
        # 自旋等待删除服务应答
        while rclpy.ok():
            rclpy.spin_once(self.DeleteEntity)
            if self.DeleteEntity.future_DELETE.done():
                try:
                    deleteRES = self.DeleteEntity.future_DELETE.result()
                except Exception as exc:
                    print("(DeleteCube): /DeleteEntity ROS2 Service call failed. ERROR: " + str(exc))
                    print("")
                    return(False)
                else:
                    if (deleteRES.success):
                        print("(DeleteCube): Successful. RESULT -> " + str(deleteRES.status_message))
                        print("")
                        return(True)
                    else:
                        print("(DeleteCube): Failed. RESULT -> " + str(deleteRES.status_message))
                        print("")
                        return(False)

# =============================================================================== #
# FUNCTIONS -> Rotation CALCULATIONS:
# 欧拉角 roll pitch yaw 转四元数，返回Pose消息（只填充orientation部分）
def EulerToQuat(roll,pitch,yaw):
    RESULT = Pose()
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    # 欧拉角转四元数标准公式
    RESULT.orientation.x = sr * cp * cy - cr * sp * sy
    RESULT.orientation.y = cr * sp * cy + sr * cp * sy
    RESULT.orientation.z = cr * cp * sy - sr * sp * cy
    RESULT.orientation.w = cr * cp * cy + sr * sp * sy
    return(RESULT)

# EEQuaternion：末端执行器四元数计算，四元数乘法，实现姿态叠加旋转
def EEQuaternion(yaw):
    RESULT = Pose()
    # 1. Initial pose: 机械臂末端初始基准四元数
    Ax = 0.0
    Ay = 1.0
    Az = 0.0
    Aw = 0.0
    # 2. Get desired RELATIVE ROTATION: 想要叠加的相对旋转(-yaw)
    pitch = 0.0
    roll = 0.0
    cy = math.cos(-yaw * 0.5)
    sy = math.sin(-yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    Bx = sr * cp * cy - cr * sp * sy
    By = cr * sp * cy + sr * cp * sy
    Bz = cr * cp * sy - sr * sp * cy
    Bw = cr * cp * cy + sr * sp * sy
    # 3. Quaternion MULTIPLICATION: 四元数乘法，基准姿态 * 相对旋转 = 最终姿态
    RESULT.orientation.x = Aw*Bx + Ax*Bw + Ay*Bz - Az*By
    RESULT.orientation.y = Aw*By - Ax*Bz + Ay*Bw + Az*Bx
    RESULT.orientation.z = Aw*Bz + Ax*By - Ay*Bx + Az*Bw
    RESULT.orientation.w = Aw*Bw - Ax*Bx - Ay*By - Az*Bz
    return(RESULT)
