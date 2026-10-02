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
#  AUTHORS: Mikel Bueno Viso - Mikel.Bueno-Viso@cranfield.ac.uk                         #
#           Dr. Seemal Asif  - s.asif@cranfield.ac.uk                                   #
#           Prof. Phil Webb  - p.f.webb@cranfield.ac.uk                                 #
#                                                                                       #
#  Date: April, 2023.                                                                   #
#                                                                                       #
# ===================================== COPYRIGHT ===================================== #
# ======= CITE OUR WORK ======= #
# You can cite our work with the following statement:
# IFRA-Cranfield (2023) ROS 2 Sim-to-Real Robot Control. URL: https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl.
# COPYRIGHT:
# The irb120pe_moveit2.launch.py script is based on the irb120_interface.launch.py file in IFRA-Cranfield/ros2_SimRealRobotControl.
#
# 文件：irb120pe_moveit2.launch.py
# 功能：ROS2 Humble 启动文件，一次性拉起 Gazebo仿真 + MoveIt2，用于ABB IRB120机械臂仿真
# 包含：机械臂本体、Schunk EGP-64夹爪、RViz可视化、运动规划、ros2srrc运动接口服务
# 这个文件就是你运行 ros2 launch xxx 时执行的启动脚本
# Import libraries:
import os
# 获取ROS包的share目录路径
from ament_index_python.packages import get_package_share_directory
# Launch框架核心：描述整个启动任务
from launch import LaunchDescription
# 用来启动ROS2节点
from launch_ros.actions import Node
# 启动参数变量
from launch.substitutions import LaunchConfiguration
# 执行外部命令、包含其他launch文件
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument, TimerAction
# 条件判断：满足/不满足条件才启动节点
from launch.conditions import IfCondition, UnlessCondition
# 进程退出事件监听（顺序启动控制器核心）
from launch.event_handlers import OnProcessExit
# 加载其他python类型launch文件
from launch.launch_description_sources import PythonLaunchDescriptionSource
# xacro：机器人描述文件预处理，把xacro转成urdf
import xacro
# yaml：读取配置文件（运动学、限制、规划器参数）
import yaml

# LOAD FILE:
# 函数：读取包内任意文本文件（SRDF等），返回字符串
def load_file(package_name, file_path):
    # 获取这个包的share目录
    package_path = get_package_share_directory(package_name)
    # 拼接得到文件绝对路径
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:
        # 文件读取失败返回空
        return None

# LOAD YAML:
# 函数：读取yaml配置文件，返回字典
def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

# ========== **GENERATE LAUNCH DESCRIPTION** ========== #
# launch入口函数，所有要启动的节点都在这里定义
def generate_launch_description():
    
    # *********************** Gazebo 仿真环境部分 *********************** # 
    
    # DECLARE Gazebo WORLD file: 指定Gazebo使用的仿真世界文件（带工作台、方块环境）
    irb120pe_gazebo = os.path.join(
        get_package_share_directory('irb120pe_gazebo'),
        'worlds',
        'irb120.world')

    # DECLARE Gazebo LAUNCH file: 调用gazebo官方launch，启动Gazebo仿真器，加载上面的world场景
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
                # 传入参数：world文件路径
                launch_arguments={'world': irb120pe_gazebo}.items(),
             )

    # ========== 控制台打印提示信息 ========== #
    print("")
    print("===== ABB IRB-120 Pose Estimation: Robot Simulation+Control (irb120pe_moveit2) =====")
    print("Robot configuration:")
    print("")
    # Cell Layout:
    print("- Cell layout: Cranfield University - IA Lab enclosure.")
    # End-Effector:
    print("- End-effector: Schunk EGP-64 parallel gripper.")
    print("")

    # ***** ROBOT DESCRIPTION 机器人模型 ***** #
    # ABB-IRB120 Description file package: 机器人模型所在包
    irb120_description_path = os.path.join(
        get_package_share_directory('irb120pe_gazebo'))
    # ABB-IRB120 ROBOT urdf xacro文件路径：机器人本体+夹爪模型
    xacro_file = os.path.join(irb120_description_path,
                              'urdf',
                              'irb120.urdf.xacro')
    # Generate ROBOT_DESCRIPTION for ABB-IRB120: 解析xacro，生成urdf xml字符串
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc, mappings={})
    robot_description_config = doc.toxml()
    # 封装成ROS参数，robot_state_publisher会读取这个参数
    robot_description = {'robot_description': robot_description_config}

    # SPAWN ROBOT TO GAZEBO: 在Gazebo世界里生成机器人实体，名字叫irb120
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'irb120'],
                        output='screen')

    # ***** STATIC TRANSFORM 静态坐标变换 ***** #
    # NODE -> Static TF: 发布world到base_link的静态tf（世界坐标系→机器人底座坐标系）
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        # 参数：x,y,z,roll,pitch,yaw  父坐标系world，子坐标系base_link
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    # Publish TF: 机器人状态发布器，读取urdf，实时发布所有关节的TF坐标变换
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[
            robot_description,
            {"use_sim_time": True} # 使用仿真时间，不使用系统真实时间
        ]
    )

    # ***** ROS2_CONTROL -> 加载控制器 ***** #
    # Joint STATE BROADCASTER：关节状态广播器，发布每个关节角度
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    # Joint TRAJECTORY Controller: 关节轨迹控制器，用来控制机械臂6个关节运动
    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["irb120_controller", "-c", "/controller_manager"],
    )
    
    # === SCHUNK EGP-64 夹爪控制器 === #
    # 夹爪左手指控制器
    egp64left_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["egp64_finger_left_controller", "-c", "/controller_manager"],
    )
    # 夹爪右手指控制器
    egp64right_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["egp64_finger_right_controller", "-c", "/controller_manager"],
    )
    # === SCHUNK EGP-64 === #

    # *********************** MoveIt!2 运动规划模块 *********************** #   
    
    # Command-line argument: RVIZ file? 声明启动参数，用来选择是否加载rviz配置
    rviz_arg = DeclareLaunchArgument(
        "rviz_file", default_value="False", description="Load RVIZ file."
    )

    # *** PLANNING CONTEXT 运动规划上下文 *** #
    # Robot description, SRDF: SRDF是MoveIt专用，描述机器人关节组、碰撞禁区、虚拟关节
    robot_description_semantic_config = load_file("irb120pe_moveit2", "config/irb120egp64.srdf")
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_config}

    # Kinematics.yaml文件：运动学求解器参数（IK逆解）
    kinematics_yaml = load_yaml(
        "irb120pe_moveit2", "config/kinematics.yaml"
    )
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

    # joint_limits.yaml文件：关节最大最小角度、最大速度、加速度限制
    joint_limits_yaml = load_yaml("irb120pe_moveit2", "config/joint_limits.yaml")
    joint_limits = {'robot_description_planning': joint_limits_yaml}

    # pilz_planning_pipeline_config.yaml：PILZ工业运动规划器（支持PTP关节运动、直线MoveL，就是前面waypoints里的MoveJ/MoveL）
    pilz_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "pilz_industrial_motion_planner/CommandPlanner",
            "request_adapters": """ """,
            "start_state_max_bounds_error": 0.1,
            "default_planner_config": "PTP",
        }
    }
    pilz_cartesian_limits_yaml = load_yaml(
        "irb120pe_moveit2", "config/pilz_cartesian_limits.yaml"
    )
    pilz_cartesian_limits = {'robot_description_planning': pilz_cartesian_limits_yaml}

    # Move group: OMPL Planning. 另一个规划器OMPL（随机采样规划，这里注释没有启用）
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    # Load ompl_planning.yaml file:
    ompl_planning_yaml = load_yaml("irb120pe_moveit2", "config/ompl_planning_egp64.yaml")
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # MoveIt!2 Controllers: MoveIt控制器管理器配置
    moveit_simple_controllers_yaml = load_yaml("irb120pe_moveit2", "config/irb120egp64_controllers.yaml"  )
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    # 轨迹执行参数
    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    # 规划场景监视器：发布碰撞、几何、tf信息
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # 启用PILZ的序列动作接口（支持连续执行多段运动，main_Gz调用的Action服务底层就是这个）
    move_group_capabilities = {
        "capabilities": """pilz_industrial_motion_planner/MoveGroupSequenceAction \
            pilz_industrial_motion_planner/MoveGroupSequenceService"""
    }

    # START NODE -> MOVE GROUP：MoveGroup核心节点，运动规划服务
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            
            pilz_planning_pipeline_config,
            #ompl_planning_pipeline_config,
            joint_limits,
            pilz_cartesian_limits,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            move_group_capabilities,
            {"use_sim_time": True},
        ],
    )

    # RVIZ可视化界面
    load_RVIZfile = LaunchConfiguration("rviz_file")
    rviz_base = os.path.join(get_package_share_directory("irb120pe_moveit2"), "config")
    rviz_full_config = os.path.join(rviz_base, "irb120egp64_moveit2.rviz")
    rviz_node_full = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_full_config], # 加载rviz保存的配置文件
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            
            pilz_planning_pipeline_config,
            #ompl_planning_pipeline_config,
            joint_limits,
            pilz_cartesian_limits,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            move_group_capabilities,
            {"use_sim_time": True},
        ],
        # UnlessCondition：rviz_file=False 才启动rviz
        condition=UnlessCondition(load_RVIZfile),
    )

    # ========= ros2srrc_execution 运动接口节点（重点！main_Gz.py调用的服务） =========
    # Move接口：对应 /Move 服务，接收Action消息（MoveJ/MoveL/MoveROT/MoveRP）
    MoveInterface = Node(
        name="move",
        package="ros2srrc_execution",
        executable="move",
        output="screen",
        parameters=[robot_description, robot_description_semantic, kinematics_yaml, {"use_sim_time": True}, {"ROB_PARAM": "irb120"}, {"EE_PARAM": "egp64"}, {"ENV_PARAM": "gazebo"}],
    )
    # RobMove接口：对应 /RobMove 服务，接收Pose绝对位姿，做笛卡尔运动
    RobMoveInterface = Node(
        name="robmove",
        package="ros2srrc_execution",
        executable="robmove",
        output="screen",
        parameters=[robot_description, robot_description_semantic, kinematics_yaml, {"use_sim_time": True}, {"ROB_PARAM": "irb120"}, {"EE_PARAM": "egp64"}, {"ENV_PARAM": "gazebo"}],
    )
    # RobPose接口：获取机器人当前末端位姿
    RobPoseInterface = Node(
        name="robpose",
        package="ros2srrc_execution",
        executable="robpose",
        output="screen",
        parameters=[robot_description, robot_description_semantic, kinematics_yaml, {"ROB_PARAM": "irb120"}],
    )
    # Sequence接口：批量执行一连串运动动作序列
    SequenceInterface = Node(
        name="sequence",
        package="ros2srrc_execution",
        executable="sequence",
        output="screen",
        parameters=[robot_description, robot_description_semantic, kinematics_yaml, {"use_sim_time": True}, {"ROB_PARAM": "irb120"}, {"EE_PARAM": "egp64"}, {"ENV_PARAM": "gazebo"}],
    )
    
    # 返回LaunchDescription，里面放所有节点和事件规则
    return LaunchDescription(
        [
            # Gazebo nodes:
            gazebo, 
            spawn_entity,
            # ROS2_CONTROL:
            static_tf,
            robot_state_publisher,
            
            # ROS2 Controllers 顺序启动控制器：
            # OnProcessExit：等上一个进程成功退出之后，才启动下一个，防止控制器加载顺序错乱
            RegisterEventHandler(
                OnProcessExit(
                    target_action = spawn_entity,
                    on_exit = [
                        joint_state_broadcaster_spawner,
                    ]
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action = joint_state_broadcaster_spawner,
                    on_exit = [
                        joint_trajectory_controller_spawner,
                    ]
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action = joint_trajectory_controller_spawner,
                    on_exit = [
                        egp64left_controller_spawner,
                    ]
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action = egp64left_controller_spawner,
                    on_exit = [
                        egp64right_controller_spawner,
                    ]
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action = egp64right_controller_spawner,
                    on_exit = [
                        # MoveIt!2: 夹爪控制器启动完成后，延时2秒启动RVIZ和move_group
                        TimerAction(
                            period=2.0,
                            actions=[
                                rviz_arg,
                                rviz_node_full,
                                run_move_group_node
                            ]
                        ),
                        # 再延时5秒，启动ros2srrc的4个运动接口（/Move /RobMove /RobPose /Sequence）
                        # main_Gz.py 就是调用这几个节点提供的服务！
                        TimerAction(
                            period=5.0,
                            actions=[
                                MoveInterface,
                                SequenceInterface,
                                RobMoveInterface,
                                RobPoseInterface,
                            ]
                        ),
                    ]
                )
            )
        ]
    )
