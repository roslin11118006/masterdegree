#!/usr/bin/env python3
# coding=utf-8
import rospy
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from detection_msgs.msg import BoundingBoxes  # 确保 detection_msgs 正确定义在你的工作空间
from std_srvs.srv import Empty
from move_base_msgs.msg import MoveBaseActionResult
import math

# 全局变量
stopflag = False  # 表示是否需要停止机器人
navigationflag = False  # 表示是否需要导航机器人
navigation_completed = False  # 表示导航是否完成
vel_cmd = Twist()  # 速度指令消息
nav_goal = PoseStamped()  # 导航目标消息
current_goal = PoseStamped()  # move_base当前目标
min_distance = float('inf')  # 最近障碍物距离
scan_sub = None  # 激光扫描数据的订阅者

# 目标点（转换后的地图坐标）
goal_x = 0.1028457608955656
goal_y = -0.24373510447303082
goal_z = 0.0
goal_orientation_x = 0.0
goal_orientation_y = 0.0
goal_orientation_z = -0.7447992380767343
goal_orientation_w = 0.6672886144392964

def LaserCallback(msg):
    global min_distance
    valid_ranges = [r for r in msg.ranges if 0.05 < r < msg.range_max]  # 过滤掉无效的零距离读数和过大读数
    if valid_ranges:
        min_distance = min(valid_ranges)
    else:
        min_distance = float('inf')  # 如果没有有效数据，设置为无限大
    rospy.loginfo(f"Current minimum distance to obstacle: {min_distance:.2f} meters")

def ThumbCallback(msg):
    global stopflag, navigationflag, nav_goal, scan_sub, navigation_completed
    navigation_completed = False  # 重置导航完成标志
    for bbox in msg.bounding_boxes:
        if bbox.Class == "thumb1" and bbox.probability > 0.9:
            stopflag = False
            navigationflag = True
            handle_obstacle_detection("thumb1")
            break
        elif bbox.Class == "thumb2" and bbox.probability > 0.9:
            stopflag = False
            navigationflag = True
            handle_obstacle_detection("thumb2")
            break
        else:
            stopflag = False
            navigationflag = False

def GoalCallback(msg):
    global current_goal
    current_goal = msg  # 更新当前目标点

def handle_obstacle_detection(gesture):
    global nav_goal, current_goal, vel_cmd, scan_sub

    rospy.loginfo(f"{gesture} detected, initiating obstacle detection.")
    
    # 订阅激光扫描数据主题
    scan_sub = rospy.Subscriber("/scan", LaserScan, LaserCallback)
    rate = rospy.Rate(20)
    
    # 等待一段时间以确保收到激光扫描数据
    rospy.sleep(2)
    
    if min_distance < 0.4:  # 更新为40厘米
        rospy.loginfo("Obstacle too close, moving backward")
        while min_distance < 0.4:  # 更新为40厘米
            vel_cmd.linear.x = -0.1  # 后退
            vel_cmd.angular.z = 0.0
            vel_pub.publish(vel_cmd)
            rate.sleep()
        vel_cmd.linear.x = 0.0
        vel_pub.publish(vel_cmd)
        rospy.loginfo("Moved to a safe distance")
    
    # 停止激光测距
    if scan_sub:
        scan_sub.unregister()
    
    # 自转一圈清除本地地图
    rotate_360(rate)
    clear_local_map()

    # 设置导航目标点
    if gesture == "thumb1":
        nav_goal.header.frame_id = "map"
        nav_goal.header.stamp = rospy.Time.now()
        nav_goal.pose.position.x = goal_x
        nav_goal.pose.position.y = goal_y
        nav_goal.pose.position.z = goal_z
        nav_goal.pose.orientation.x = goal_orientation_x
        nav_goal.pose.orientation.y = goal_orientation_y
        nav_goal.pose.orientation.z = goal_orientation_z
        nav_goal.pose.orientation.w = goal_orientation_w
        rospy.loginfo(f"Navigating to predefined goal: ({goal_x}, {goal_y}, {goal_z})")
    elif gesture == "thumb2":
        nav_goal = current_goal  # 使用当前目标点作为新的导航目标
        rospy.loginfo("Navigating to current goal")

def rotate_360(rate):
    global vel_cmd
    rotation_speed = 0.3  # rad/s
    rotation_duration = (2 * math.pi) / rotation_speed  # 时间 = 角度 / 角速度
    start_time = rospy.Time.now()
    while rospy.Time.now() - start_time < rospy.Duration(rotation_duration):
        vel_cmd.angular.z = rotation_speed
        vel_pub.publish(vel_cmd)
        rate.sleep()
    vel_cmd.angular.z = 0.0
    vel_pub.publish(vel_cmd)
    rospy.loginfo("Completed 360-degree rotation")

def clear_local_map():
    rospy.wait_for_service('/move_base/clear_costmaps')
    try:
        clear_costmaps = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        clear_costmaps()
        rospy.loginfo("Local map cleared")
    except rospy.ServiceException as e:
        rospy.logwarn(f"Service call failed: {e}")

def navigation_result_callback(msg):
    global navigation_completed
    if msg.status.status == 3:  # Goal reached
        navigation_completed = True
        rospy.loginfo("Navigation goal reached")

if __name__ == '__main__':
    # 初始化ROS节点
    rospy.init_node("thumbdetect")

    # 订阅边界框检测主题
    rospy.Subscriber("/yolov5/detections", BoundingBoxes, ThumbCallback)

    # 订阅move_base的目标点主题
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, GoalCallback)

    # 订阅move_base的导航结果主题
    rospy.Subscriber("/move_base/result", MoveBaseActionResult, navigation_result_callback)

    # 设置速度命令发布者
    vel_pub = rospy.Publisher("cmd_vel_1", Twist, queue_size=10)
    rate = rospy.Rate(20)

    # 设置导航目标发布者
    nav_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)

    while not rospy.is_shutdown():
        if stopflag:
            vel_cmd.linear.x = 0.0
            vel_cmd.angular.z = 0.0
            vel_pub.publish(vel_cmd)
            rospy.loginfo("Emergency Stop")
        elif navigationflag:
            nav_pub.publish(nav_goal)
            rospy.loginfo(f"Navigating to goal: ({nav_goal.pose.position.x}, {nav_goal.pose.position.y}, {nav_goal.pose.position.z})")
            navigationflag = False

        if navigation_completed:
            rospy.loginfo("Stopping navigation as goal is reached")
            nav_goal = PoseStamped()  # 清空导航目标
            stopflag = True  # 停止机器人
            navigation_completed = False  # 重置导航完成标志

        rate.sleep()

