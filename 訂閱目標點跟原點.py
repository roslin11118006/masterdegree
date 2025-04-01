#!/usr/bin/env python3
# coding=utf-8
import rospy
from geometry_msgs.msg import Twist, PoseStamped
from detection_msgs.msg import BoundingBoxes  # 確保 detection_msgs 正確定義在你的工作空間
from std_srvs.srv import Empty
from sensor_msgs.msg import LaserScan

# 全域變數
stopflag = False  # 是否需要停止機器人
rotateflag = False  # 是否需要旋轉機器人
navigationflag = False  # 是否需要導航機器人
rotation_completed = False  # 旋轉是否完成
nav_goal = PoseStamped()  # 導航目標訊息
start_point = PoseStamped()  # AMCL 的起始點
current_goal = PoseStamped()  # move_base 的當前目標
obstacle_detected = False  # 是否檢測到30公分內的障礙物
vel_cmd = Twist()  # 速度指令訊息

def ThumbCallback(msg):
    global stopflag, rotateflag, navigationflag, rotation_completed, nav_goal, start_point, current_goal
    for bbox in msg.bounding_boxes:
        if bbox.Class == "thumb1" and bbox.probability > 0.9:
            stopflag = False
            rotateflag = False
            navigationflag = True
            rotation_completed = False
            nav_goal = current_goal  # 使用當前目標作為新的導航目標
            break
        elif bbox.Class == "thumb2" and bbox.probability > 0.9:
            stopflag = False
            rotateflag = False
            navigationflag = True
            rotation_completed = False
            nav_goal = start_point  # 導航至起始點
            break
        else:
            stopflag = False
            rotateflag = False
            navigationflag = False
            rotation_completed = False

def GoalCallback(msg):
    global current_goal
    current_goal = msg  # 更新當前目標點

def StartPointCallback(msg):
    global start_point
    start_point = msg  # 更新 AMCL 的起始點

def LaserScanCallback(msg):
    global obstacle_detected
    # 檢查是否有障礙物在機器人前方30公分內
    if min(msg.ranges) < 0.3:
        obstacle_detected = True
    else:
        obstacle_detected = False

if __name__ == '__main__':
    # 初始化ROS節點
    rospy.init_node("thumbdetect")

    # 訂閱邊界框檢測主題
    rospy.Subscriber("/yolov5/detections", BoundingBoxes, ThumbCallback)

    # 訂閱 move_base 的目標點主題
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, GoalCallback)

    # 訂閱 AMCL 初始位姿主題
    rospy.Subscriber("/amcl_pose", PoseStamped, StartPointCallback)

    # 訂閱 LiDAR 掃描主題
    rospy.Subscriber("/scan", LaserScan, LaserScanCallback)

    # 設置速度指令發布者
    vel_pub = rospy.Publisher("cmd_vel_4", Twist, queue_size=10)
    rate = rospy.Rate(20)
    # 設置速度指令發布者
    vel_pub = rospy.Publisher("cmd_vel_5", Twist, queue_size=10)
    rate = rospy.Rate(20)
    # 設置導航目標發布者
    nav_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)

    # 旋轉計時器
    rotate_duration = rospy.Duration(5.0)  # 假設旋轉一圈需要5秒
    rotate_start_time = None

    while not rospy.is_shutdown():
        if stopflag:
            vel_cmd.linear.x = 0.0
            vel_cmd.angular.z = 0.0
            vel_pub.publish(vel_cmd)
            rospy.loginfo("緊急停止")
        elif obstacle_detected:
            vel_cmd.linear.x = -0.1  # 向後移動
            vel_cmd.angular.z = 0.0
            vel_pub.publish(vel_cmd)
            rospy.sleep(3)  # 向後移動3秒
            obstacle_detected = False
            rotateflag = True  # 設置旋轉標誌
        elif rotateflag and not rotation_completed:
            if rotate_start_time is None:
                rotate_start_time = rospy.Time.now()

            elapsed_time = rospy.Time.now() - rotate_start_time
            if elapsed_time < rotate_duration:
                vel_cmd.linear.x = 0.0
                vel_cmd.angular.z = 0.5  # 旋轉速度
                vel_pub.publish(vel_cmd)
                rospy.loginfo("旋轉中")
            else:
                vel_cmd.linear.x = 0.0
                vel_cmd.angular.z = 0.0
                vel_pub.publish(vel_cmd)
                rospy.loginfo("旋轉完成")

                # 清除全局和局部成本地圖
                rospy.wait_for_service('/move_base/clear_costmaps')
                try:
                    clear_costmaps = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
                    clear_costmaps()
                    rospy.loginfo("成本地圖已清除")
                except rospy.ServiceException as e:
                    rospy.logerr("服務調用失敗: %s" % e)

                rotation_completed = True
                rotateflag = False
                rotate_start_time = None
        elif navigationflag:
            nav_pub.publish(nav_goal)
            rospy.loginfo("導航至目標")
            navigationflag = False

        rate.sleep()
