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
#
# 文件：waypoints.py
# 功能：专门存放IRB120机械臂所有预设航点（点位）
# 核心函数RobotPose：输入点位名字符串，返回对应的ROS消息对象
# 两种消息类型：
#   Pose → 给 /RobMove 接口使用：直接描述末端执行器【笛卡尔坐标 + 四元数姿态】
#   Action → 给 /Move 接口使用：描述运动类型（关节运动、直线、增量运动、纯旋转）
# ======================================================================================

# ===== IMPORT REQUIRED COMPONENTS ===== #
# 导入ROS2内置消息类型：Pose 用来描述位姿（位置+四元数姿态）
from geometry_msgs.msg import Pose
# 导入项目自定义消息：Action，封装各类机械臂运动指令（MoveJ/MoveL/MoveROT等）
from ros2srrc_data.msg import Action

# =============================================================================== #
# 定义类：waypoints 航点管理类
# 全部预定义点位保存在字典 RobotPoseDict 中
class waypoints():
    # 类初始化函数，实例化waypoints对象时自动执行，加载所有点位
    def __init__(self):
        # 字典：key=点位名称字符串，value=Pose对象 或者 Action对象
        self.RobotPoseDict = {}

        # ========== HOME 机械臂原点/回家点 ========== #
        # HomePos：关节空间回到原点，Action类型，给/Move接口调用
        self.RobotPoseDict["HomePos"] = Action()
        self.RobotPoseDict["HomePos"].action = "MoveJ"   # MoveJ：关节运动（各关节独立运动，轨迹不一定是直线）
        self.RobotPoseDict["HomePos"].speed = 1.0         # 运动速度
        self.RobotPoseDict["HomePos"].movej.joint1 = 0.0
        self.RobotPoseDict["HomePos"].movej.joint2 = -30.0
        self.RobotPoseDict["HomePos"].movej.joint3 = 30.0
        self.RobotPoseDict["HomePos"].movej.joint4 = 0.0
        self.RobotPoseDict["HomePos"].movej.joint5 = 90.0
        self.RobotPoseDict["HomePos"].movej.joint6 = 0.0

        # ========== PLACE CUBE 放置方块点位 ========== #
        # 命名规则：xxx_app = approach 接近点（上方预备点，z坐标更高，不碰物体，安全靠近）
        # xxx = 实际落下去的点位，z更小，下降到工作台放置物体
        # Blue Cube 蓝色方块放置点位
        self.RobotPoseDict["PlaceBLUE_app"] = Pose()
        self.RobotPoseDict["PlaceBLUE_app"].position.x = 0.15
        self.RobotPoseDict["PlaceBLUE_app"].position.y = 0.145
        self.RobotPoseDict["PlaceBLUE_app"].position.z = 1.111   # z高，接近预备点，在方块上方
        self.RobotPoseDict["PlaceBLUE_app"].orientation.x = 0.707
        self.RobotPoseDict["PlaceBLUE_app"].orientation.y = 0.707
        self.RobotPoseDict["PlaceBLUE_app"].orientation.z = 0.0
        self.RobotPoseDict["PlaceBLUE_app"].orientation.w = 0.0

        self.RobotPoseDict["PlaceBLUE"] = Pose()
        self.RobotPoseDict["PlaceBLUE"].position.x = 0.15
        self.RobotPoseDict["PlaceBLUE"].position.y = 0.145
        self.RobotPoseDict["PlaceBLUE"].position.z = 1.08    # z降低，落到放置高度
        self.RobotPoseDict["PlaceBLUE"].orientation.x = 0.707
        self.RobotPoseDict["PlaceBLUE"].orientation.y = 0.707
        self.RobotPoseDict["PlaceBLUE"].orientation.z = 0.0
        self.RobotPoseDict["PlaceBLUE"].orientation.w = 0.0

        # Black Cube 黑色方块放置点位
        self.RobotPoseDict["PlaceBLACK_app"] = Pose()
        self.RobotPoseDict["PlaceBLACK_app"].position.x = 0.105
        self.RobotPoseDict["PlaceBLACK_app"].position.y = 0.145
        self.RobotPoseDict["PlaceBLACK_app"].position.z = 1.111
        self.RobotPoseDict["PlaceBLACK_app"].orientation.x = 0.707
        self.RobotPoseDict["PlaceBLACK_app"].orientation.y = 0.707
        self.RobotPoseDict["PlaceBLACK_app"].orientation.z = 0.0
        self.RobotPoseDict["PlaceBLACK_app"].orientation.w = 0.0

        self.RobotPoseDict["PlaceBLACK"] = Pose()
        self.RobotPoseDict["PlaceBLACK"].position.x = 0.105
        self.RobotPoseDict["PlaceBLACK"].position.y = 0.145
        self.RobotPoseDict["PlaceBLACK"].position.z = 1.08
        self.RobotPoseDict["PlaceBLACK"].orientation.x = 0.707
        self.RobotPoseDict["PlaceBLACK"].orientation.y = 0.707
        self.RobotPoseDict["PlaceBLACK"].orientation.z = 0.0
        self.RobotPoseDict["PlaceBLACK"].orientation.w = 0.0

        # White Cube 白色方块放置点位
        self.RobotPoseDict["PlaceWHITE_app"] = Pose()
        self.RobotPoseDict["PlaceWHITE_app"].position.x = 0.105
        self.RobotPoseDict["PlaceWHITE_app"].position.y = 0.100
        self.RobotPoseDict["PlaceWHITE_app"].position.z = 1.111
        self.RobotPoseDict["PlaceWHITE_app"].orientation.x = 0.707
        self.RobotPoseDict["PlaceWHITE_app"].orientation.y = 0.707
        self.RobotPoseDict["PlaceWHITE_app"].orientation.z = 0.0
        self.RobotPoseDict["PlaceWHITE_app"].orientation.w = 0.0

        self.RobotPoseDict["PlaceWHITE"] = Pose()
        self.RobotPoseDict["PlaceWHITE"].position.x = 0.105
        self.RobotPoseDict["PlaceWHITE"].position.y = 0.100
        self.RobotPoseDict["PlaceWHITE"].position.z = 1.08
        self.RobotPoseDict["PlaceWHITE"].orientation.x = 0.707
        self.RobotPoseDict["PlaceWHITE"].orientation.y = 0.707
        self.RobotPoseDict["PlaceWHITE"].orientation.z = 0.0
        self.RobotPoseDict["PlaceWHITE"].orientation.w = 0.0

        # No‑sticker Cube 无贴纸方块放置点位
        self.RobotPoseDict["PlaceCUBE_app"] = Pose()
        self.RobotPoseDict["PlaceCUBE_app"].position.x = 0.15
        self.RobotPoseDict["PlaceCUBE_app"].position.y = 0.100
        self.RobotPoseDict["PlaceCUBE_app"].position.z = 1.111
        self.RobotPoseDict["PlaceCUBE_app"].orientation.x = 0.707
        self.RobotPoseDict["PlaceCUBE_app"].orientation.y = 0.707
        self.RobotPoseDict["PlaceCUBE_app"].orientation.z = 0.0
        self.RobotPoseDict["PlaceCUBE_app"].orientation.w = 0.0

        self.RobotPoseDict["PlaceCUBE"] = Pose()
        self.RobotPoseDict["PlaceCUBE"].position.x = 0.15
        self.RobotPoseDict["PlaceCUBE"].position.y = 0.100
        self.RobotPoseDict["PlaceCUBE"].position.z = 1.08
        self.RobotPoseDict["PlaceCUBE"].orientation.x = 0.707
        self.RobotPoseDict["PlaceCUBE"].orientation.y = 0.707
        self.RobotPoseDict["PlaceCUBE"].orientation.z = 0.0
        self.RobotPoseDict["PlaceCUBE"].orientation.w = 0.0

        # ========== CHECK CUBE FACE 查看方块各个面（相机拍照识别贴纸） ========== #
        # Front face 方块正面拍照位姿：机械臂抓着方块，移动到相机视野拍正面
        self.RobotPoseDict["FacePose_1"] = Pose()
        self.RobotPoseDict["FacePose_1"].position.x = 0.501
        self.RobotPoseDict["FacePose_1"].position.y = 0.525
        self.RobotPoseDict["FacePose_1"].position.z = 1.48
        self.RobotPoseDict["FacePose_1"].orientation.x = 0.0
        self.RobotPoseDict["FacePose_1"].orientation.y = 0.708
        self.RobotPoseDict["FacePose_1"].orientation.z = 0.0
        self.RobotPoseDict["FacePose_1"].orientation.w = 0.707

        # Back face：原地旋转180度看背面，MoveROT 只旋转末端姿态，不改变空间位置
        self.RobotPoseDict["FacePose_2"] = Action()
        self.RobotPoseDict["FacePose_2"].action = "MoveROT"
        self.RobotPoseDict["FacePose_2"].speed = 1.0
        self.RobotPoseDict["FacePose_2"].moverot.yaw = 180.0
        self.RobotPoseDict["FacePose_2"].moverot.pitch = 0.0
        self.RobotPoseDict["FacePose_2"].moverot.roll = 0.0

        # Bottom face 方块底面拍照点位
        self.RobotPoseDict["FacePose_3"] = Pose()
        self.RobotPoseDict["FacePose_3"].position.x = 0.63
        self.RobotPoseDict["FacePose_3"].position.y = 0.525
        self.RobotPoseDict["FacePose_3"].position.z = 1.353
        self.RobotPoseDict["FacePose_3"].orientation.x = 0.001
        self.RobotPoseDict["FacePose_3"].orientation.y = 0.147
        self.RobotPoseDict["FacePose_3"].orientation.z = -0.003
        self.RobotPoseDict["FacePose_3"].orientation.w = 0.989

        # Top face 方块顶面拍照点位
        self.RobotPoseDict["FacePose_4"] = Pose()
        self.RobotPoseDict["FacePose_4"].position.x = 0.525
        self.RobotPoseDict["FacePose_4"].position.y = 0.527
        self.RobotPoseDict["FacePose_4"].position.z = 1.58
        self.RobotPoseDict["FacePose_4"].orientation.x = -0.004
        self.RobotPoseDict["FacePose_4"].orientation.y = 0.940
        self.RobotPoseDict["FacePose_4"].orientation.z = 0.002
        self.RobotPoseDict["FacePose_4"].orientation.w = 0.342

        # ========== IF STICKER -> TOP... 贴纸在顶面的整套动作点位 ========== #
        # Pick cube before checking top face (1): 准备拾取方块，抬升靠近
        self.RobotPoseDict["PickTopApp"] = Action()
        self.RobotPoseDict["PickTopApp"].action = "MoveL"      # MoveL 笛卡尔直线运动，末端走直线
        self.RobotPoseDict["PickTopApp"].speed = 0.1
        self.RobotPoseDict["PickTopApp"].movel.x = 0.0
        self.RobotPoseDict["PickTopApp"].movel.y = 0.0
        self.RobotPoseDict["PickTopApp"].movel.z = 0.05

        self.RobotPoseDict["PickTop"] = Action()
        self.RobotPoseDict["PickTop"].action = "MoveRP"        # MoveRP：位姿增量运动，在当前位置基础上叠加偏移
        self.RobotPoseDict["PickTop"].speed = 0.1
        self.RobotPoseDict["PickTop"].moverp.x = 0.0
        self.RobotPoseDict["PickTop"].moverp.y = 0.0
        self.RobotPoseDict["PickTop"].moverp.z = 0.19
        self.RobotPoseDict["PickTop"].moverp.yaw = 0.0
        self.RobotPoseDict["PickTop"].moverp.pitch = -20.0
        self.RobotPoseDict["PickTop"].moverp.roll = 0.0

        # Place it on top of workspace after checking top face (2): 拍完顶面，临时放到工作台
        self.RobotPoseDict["RePickTopApp_PREV"] = Pose()
        self.RobotPoseDict["RePickTopApp_PREV"].position.x = 0.528
        self.RobotPoseDict["RePickTopApp_PREV"].position.y = 0.527
        self.RobotPoseDict["RePickTopApp_PREV"].position.z = 1.057
        self.RobotPoseDict["RePickTopApp_PREV"].orientation.x = 0.0
        self.RobotPoseDict["RePickTopApp_PREV"].orientation.y = 0.938
        self.RobotPoseDict["RePickTopApp_PREV"].orientation.z = 0.0
        self.RobotPoseDict["RePickTopApp_PREV"].orientation.w = 0.346

        self.RobotPoseDict["RePickTop_PREV"] = Action()
        self.RobotPoseDict["RePickTop_PREV"].action = "MoveL"
        self.RobotPoseDict["RePickTop_PREV"].speed = 0.1
        self.RobotPoseDict["RePickTop_PREV"].movel.x = 0.0
        self.RobotPoseDict["RePickTop_PREV"].movel.y = 0.0
        self.RobotPoseDict["RePickTop_PREV"].movel.z = -0.03

        # Pick it back before placing on tray (3): 再次拾取方块，准备放到分类托盘tray
        self.RobotPoseDict["RePickTop"] = Action()
        self.RobotPoseDict["RePickTop"].action = "MoveRP"
        self.RobotPoseDict["RePickTop"].speed = 0.1
        self.RobotPoseDict["RePickTop"].moverp.x = 0.0
        self.RobotPoseDict["RePickTop"].moverp.y = 0.0
        self.RobotPoseDict["RePickTop"].moverp.z = 0.19
        self.RobotPoseDict["RePickTop"].moverp.yaw = 0.0
        self.RobotPoseDict["RePickTop"].moverp.pitch = 20.0
        self.RobotPoseDict["RePickTop"].moverp.roll = 0.0

        self.RobotPoseDict["RePickTopApp"] = Action()
        self.RobotPoseDict["RePickTopApp"].action = "MoveL"
        self.RobotPoseDict["RePickTopApp"].speed = 0.1
        self.RobotPoseDict["RePickTopApp"].movel.x = 0.0
        self.RobotPoseDict["RePickTopApp"].movel.y = 0.0
        self.RobotPoseDict["RePickTopApp"].movel.z = 0.05

        # ========== IF STICKER -> SIDE FACES... 贴纸在侧面整套动作点位 ========== #
        # Place cube in the middle of the workspace (1): 把方块放到工作台中间，用来旋转检测侧面贴纸
        self.RobotPoseDict["PlaceMidApp"] = Pose()
        self.RobotPoseDict["PlaceMidApp"].position.x = 0.65
        self.RobotPoseDict["PlaceMidApp"].position.y = 0.53
        self.RobotPoseDict["PlaceMidApp"].position.z = 1.1
        self.RobotPoseDict["PlaceMidApp"].orientation.x = 0.0
        self.RobotPoseDict["PlaceMidApp"].orientation.y = 1.0
        self.RobotPoseDict["PlaceMidApp"].orientation.z = 0.0
        self.RobotPoseDict["PlaceMidApp"].orientation.w = 0.0

        self.RobotPoseDict["PlaceMid"] = Pose()
        self.RobotPoseDict["PlaceMid"].position.x = 0.65
        self.RobotPoseDict["PlaceMid"].position.y = 0.53
        self.RobotPoseDict["PlaceMid"].position.z = 1.07
        self.RobotPoseDict["PlaceMid"].orientation.x = 0.0
        self.RobotPoseDict["PlaceMid"].orientation.y = 1.0
        self.RobotPoseDict["PlaceMid"].orientation.z = 0.0
        self.RobotPoseDict["PlaceMid"].orientation.w = 0.0

        # Pick cube before checking the side faces (2): 拾取方块，旋转查看各个侧面
        self.RobotPoseDict["PickSideApp_1"] = Action()
        self.RobotPoseDict["PickSideApp_1"].action = "MoveROT"
        self.RobotPoseDict["PickSideApp_1"].speed = 1.0
        self.RobotPoseDict["PickSideApp_1"].moverot.yaw = 90.0
        self.RobotPoseDict["PickSideApp_1"].moverot.pitch = 0.0
        self.RobotPoseDict["PickSideApp_1"].moverot.roll = 0.0

        self.RobotPoseDict["PickSideApp_2"] = Action()
        self.RobotPoseDict["PickSideApp_2"].action = "MoveL"
        self.RobotPoseDict["PickSideApp_2"].speed = 0.1
        self.RobotPoseDict["PickSideApp_2"].movel.x = 0.0
        self.RobotPoseDict["PickSideApp_2"].movel.y = 0.0
        self.RobotPoseDict["PickSideApp_2"].movel.z = 0.05

        self.RobotPoseDict["PickSide"] = Action()
        self.RobotPoseDict["PickSide"].action = "MoveL"
        self.RobotPoseDict["PickSide"].speed = 0.1
        self.RobotPoseDict["PickSide"].movel.x = 0.0
        self.RobotPoseDict["PickSide"].movel.y = 0.0
        self.RobotPoseDict["PickSide"].movel.z = -0.03

        # ========== CUBE ROTATION 方块旋转相关点位（对应前面RotateCube函数） ========== #
        # 旋转预备接近点位
        self.RobotPoseDict["RotApp"] = Pose()
        self.RobotPoseDict["RotApp"].position.x = 0.50
        self.RobotPoseDict["RotApp"].position.y = 0.53
        self.RobotPoseDict["RotApp"].position.z = 1.1
        self.RobotPoseDict["RotApp"].orientation.x = 0.0
        self.RobotPoseDict["RotApp"].orientation.y = 1.0
        self.RobotPoseDict["RotApp"].orientation.z = 0.0
        self.RobotPoseDict["RotApp"].orientation.w = 0.0

        # RotBack：末端yaw旋转180°，反向旋转
        self.RobotPoseDict["RotBack"] = Action()
        self.RobotPoseDict["RotBack"].action = "MoveROT"
        self.RobotPoseDict["RotBack"].speed = 1.0
        self.RobotPoseDict["RotBack"].moverot.yaw = 180.0
        self.RobotPoseDict["RotBack"].moverot.pitch = 0.0
        self.RobotPoseDict["RotBack"].moverot.roll = 0.0

        # RotBack_2：反向-180度旋转（和RotBack旋转方向相反）
        self.RobotPoseDict["RotBack_2"] = Action()
        self.RobotPoseDict["RotBack_2"].action = "MoveROT"
        self.RobotPoseDict["RotBack_2"].speed = 1.0
        self.RobotPoseDict["RotBack_2"].moverot.yaw = -180.0
        self.RobotPoseDict["RotBack_2"].moverot.pitch = 0.0
        self.RobotPoseDict["RotBack_2"].moverot.roll = 0.0

        # RotPlace：直线向下移动，把方块放到工作台，实现方块翻面
        self.RobotPoseDict["RotPlace"] = Action()
        self.RobotPoseDict["RotPlace"].action = "MoveL"
        self.RobotPoseDict["RotPlace"].speed = 0.1
        self.RobotPoseDict["RotPlace"].movel.x = 0.0
        self.RobotPoseDict["RotPlace"].movel.y = 0.0
        self.RobotPoseDict["RotPlace"].movel.z = -0.03

        self.RobotPoseDict["RotPlace2"] = Action()
        self.RobotPoseDict["RotPlace2"].action = "MoveL"
        self.RobotPoseDict["RotPlace2"].speed = 0.1
        self.RobotPoseDict["RotPlace2"].movel.x = 0.0
        self.RobotPoseDict["RotPlace2"].movel.y = 0.0
        self.RobotPoseDict["RotPlace2"].movel.z = -0.035

        self.RobotPoseDict["RotPlace3"] = Action()
        self.RobotPoseDict["RotPlace3"].action = "MoveL"
        self.RobotPoseDict["RotPlace3"].speed = 0.1
        self.RobotPoseDict["RotPlace3"].movel.x = 0.005
        self.RobotPoseDict["RotPlace3"].movel.y = 0.0
        self.RobotPoseDict["RotPlace3"].movel.z = -0.03

        # 旋转拾取：MoveRP增量运动，抬升+倾斜末端，拾取方块
        self.RobotPoseDict["RotationPick"] = Action()
        self.RobotPoseDict["RotationPick"].action = "MoveRP"
        self.RobotPoseDict["RotationPick"].speed = 0.1
        self.RobotPoseDict["RotationPick"].moverp.x = 0.0
        self.RobotPoseDict["RotationPick"].moverp.y = 0.0
        self.RobotPoseDict["RotationPick"].moverp.z = 0.19
        self.RobotPoseDict["RotationPick"].moverp.yaw = 0.0
        self.RobotPoseDict["RotationPick"].moverp.pitch = 30.0
        self.RobotPoseDict["RotationPick"].moverp.roll = 0.0

        # 旋转放置：增量运动，倾斜末端放下方块
        self.RobotPoseDict["RotationPlace"] = Action()
        self.RobotPoseDict["RotationPlace"].action = "MoveRP"
        self.RobotPoseDict["RotationPlace"].speed = 0.1
        self.RobotPoseDict["RotationPlace"].moverp.x = 0.0
        self.RobotPoseDict["RotationPlace"].moverp.y = 0.0
        self.RobotPoseDict["RotationPlace"].moverp.z = 0.19
        self.RobotPoseDict["RotationPlace"].moverp.yaw = 0.0
        self.RobotPoseDict["RotationPlace"].moverp.pitch = -30.0
        self.RobotPoseDict["RotationPlace"].moverp.roll = 0.0

        # RotZ：Z轴方向直线移动
        self.RobotPoseDict["RotZ"] = Action()
        self.RobotPoseDict["RotZ"].action = "MoveL"
        self.RobotPoseDict["RotZ"].speed = 0.1
        self.RobotPoseDict["RotZ"].movel.x = 0.0
        self.RobotPoseDict["RotZ"].movel.y = 0.0
        self.RobotPoseDict["RotZ"].movel.z = 0.03

        # RotX：X轴方向直线移动
        self.RobotPoseDict["RotX"] = Action()
        self.RobotPoseDict["RotX"].action = "MoveL"
        self.RobotPoseDict["RotX"].speed = 0.3
        self.RobotPoseDict["RotX"].movel.x = 0.15
        self.RobotPoseDict["RotX"].movel.y = 0.0
        self.RobotPoseDict["RotX"].movel.z = 0.0

    # 对外提供接口函数：输入点位名字符串，从字典取出并返回存好的Pose/Action消息对象
    def RobotPose(self, PoseName):
        result = self.RobotPoseDict[PoseName]
        return(result)
