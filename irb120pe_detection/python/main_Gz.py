# main_Gz.py
# Main script of the Cube Detection -> ABB IRB120 Pose Estimation Use-Case:
# GAZEBO SIMULATION -> With cube detection using the Gz CAMERA PLUGIN.
# 【说明】这是Gazebo仿真版本的主程序main_Gz.py
# 功能：立方体检测 + ABB IRB120机械臂位姿估计，完成拾取-识别-旋转-放置整套任务
# 使用Gazebo里相机插件获取图像做视觉
# ===== IMPORT REQUIRED COMPONENTS ===== #
# Required to include ROS2 and its components:
import rclpy                  # 导入ROS2 Python核心库，用于创建节点、订阅/发布话题
# Extra required libraries:
import time                   # 时间库，用于延时等待
import random                 # 随机数库，用来随机生成不同类型的方块
# ===== IMPORT FUNCTIONS ===== #
from spawn import SpawnCube         # 从spawn.py导入SpawnCube类：在Gazebo生成模型（生成方块）
from spawn import DeleteCube        # 从spawn.py导入DeleteCube类：删除Gazebo里的模型（删除方块）
from routines import RoutineList    # 从routines.py导入RoutineList类：封装机械臂所有运动子程序（回零、抓取、旋转、放置等）
from detection import CubeDetection # 从detection.py导入CubeDetection类：视觉检测、位姿估计、颜色识别相关
# ===================================================================================== #
# ======================================= MAIN ======================================== #
# ===================================================================================== #
def main(args=None):                 # 程序入口主函数，args接收命令行参数
    print("")
    print(" --- Cranfield University --- ")
    print("        (c) IFRA Group        ")
    print("")
    print("ABB IRB120: Pose Estimation for CUBE Pick&Place")
    print("Python script -> main_Gz.py")
    print("")
    # Initialise ROS2:
    rclpy.init(args=None)           # 初始化ROS2环境，必须第一步，才能创建ROS节点
    # Initialise CLASSES:
    SPAWN = SpawnCube()             # 实例化SpawnCube对象，用来生成方块模型
    DELETE = DeleteCube()           # 实例化DeleteCube对象，用来删除方块模型
    ROUTINE = RoutineList("GAZEBO") # 实例化机械臂运动类，传入GAZEBO，表示使用仿真版本（区别于真实机械臂）
    print("")
    # Move ROBOT to HomePos:
    ROUTINE.HomePos()               # 调用机械臂回零函数，让IRB120移动到初始HOME安全位置
    # SPAWN random cube:
    OPT_cube = ["WhiteCube","BlackCube","BlueCube","Cube"] # 可选方块模型列表：白、黑、蓝、普通方块
    VAR_cube = random.choice(OPT_cube) # 随机从上面列表挑选一种方块
    print("Spawning random CUBE at a randome POSE to the Gazebo Environment...")
    SpawnRES = SPAWN.SPAWN(VAR_cube,"RANDOM") # 在Gazebo生成选中的方块，位姿设为随机位置
    if (SpawnRES["success"] == False): # 判断方块生成是否失败
        rclpy.shutdown()               # 关闭ROS2
        print("CLOSING PROGRAM...")
        exit()                         # 退出整个程序
    # Initalise DETECTION class (+ CAMERA INPUT):
    DETECTION = CubeDetection("GAZEBO") # 实例化视觉检测类，GAZEBO代表读取Gazebo仿真相机图像
    # CALIBRATION w/ ARUCOS -> Get PERSPECTIVE IMG:
    DETECTION.GetPerspectiveImg(ShowPerspective=True) # 使用ArUco标记做透视矫正，获取校正后的俯视图；True=弹出窗口显示矫正图
    # Test YOLOv8 detection (UNCOMMENT THIS LINE TO TEST IF THE YOLOv8 MODEL IS DETECTING THE CUBE IN GAZEBO):
    # DETECTION.TestDetection() # YOLOv8测试函数，注释状态；去掉注释就可以单独测试目标检测效果
    # Obtain CUBE LOCATION:
    DetectRES = DETECTION.CubeLocation() # 视觉检测，得到方块的x,y坐标和yaw旋转角（方块在工作台的位姿）
    if not DetectRES["success"]:         # 如果视觉没有检测到方块（检测失败）
        
        DELETE.DELETE(VAR_cube)          # 先把Gazebo里的方块删掉
        
        rclpy.shutdown()                 # 关闭ROS2
        print("CLOSING PROGRAM...")
        exit()                           # 退出程序
    # PICK cube:
    ROUTINE.PickCube(DetectRES["x"],DetectRES["y"],DetectRES["yaw"]) # 机械臂执行抓取：传入视觉得到的方块x,y,yaw位姿
    # INIT -> CubeSide and CubeColour variables:
    CubeSide = ""                          # 初始化变量：记录方块当前是哪一个面朝上（前/后/底面等）
    CubeColour = DetectRES["detection"]    # 初始化颜色变量，从检测结果拿到初步识别结果
    
    # Check colour -> TOP FACE:
    if (CubeColour == "Sticker"):          # 如果识别到顶面是贴纸标记
        CubeColour = ROUTINE.CheckTop(DETECTION) # 调用机械臂子程序，抬到相机视野，重新识别顶面颜色
        print(CubeColour)
    # Check colour + face (BOTTOM/FRONT/BACK/SIDE):
    if (CubeColour == "Cube"):             # 如果初步识别结果只是普通Cube，没有识别出颜色
        CheckRES = ROUTINE.CheckCube(DETECTION) # 机械臂移动到拍照点位，拍照识别方块颜色和当前朝向
        CubeColour = CheckRES["COLOUR"]         # 更新识别出来的颜色
        CubeSide = CheckRES["SIDE"]             # 更新识别出来的面（FRONT/BOTTOM/BACK等）
        if (CubeColour == "Cube"):              # 如果上面一次识别还是失败，继续调用另一套识别函数
            CheckRES = ROUTINE.CheckCubeSides(DETECTION)
            CubeColour = CheckRES["COLOUR"]
            CubeSide = CheckRES["SIDE"] 
    # ROTATE if REQUIRED:
    if (CubeColour != "Cube"):                # 已经成功识别到方块颜色，判断是否需要旋转方块
        if (CubeSide == "FRONT"):             # 如果当前是FRONT面朝上
            ROUTINE.RotateCube(RotBack = False) # 执行一次正向90度旋转方块
        if (CubeSide == "BOTTOM"):            # 如果是底面朝上，需要旋转两次
            ROUTINE.RotateCube(RotBack = False)
            ROUTINE.RotateCube(RotBack = False)
        if (CubeSide == "BACK"):              # 如果是BACK面朝上，反向旋转
            ROUTINE.RotateCube(RotBack = True)
    # PLACE CUBE:
    ROUTINE.PlaceCube(CubeColour)             # 根据识别出来的颜色，放到对应的分类托盘tray
    # Finish -> Back to HomePos:
    ROUTINE.HomePos()                         # 任务完成，机械臂回到HOME安全位置
    time.sleep(3.0)                           # 等待3秒，动作稳定
    # Remove CUBE from workspace, to enable another execution after this (there can only be one cube at a time on top of the workspace for the program to work):
    DELETE.DELETE(VAR_cube)                   # 删除Gazebo中的方块，方便下一轮重新运行程序（同一时刻只能有一个方块）
    print("Program execution successfully finished!")
    print("Closing .py script... Bye!")
    rclpy.shutdown()                           # 关闭ROS2，释放资源
if __name__ == '__main__':                     # Python入口判断：直接运行本文件时，执行main函数
    main()
