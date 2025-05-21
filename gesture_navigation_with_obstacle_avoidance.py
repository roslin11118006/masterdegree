#!/usr/bin/env python3
# coding=utf-8
import rospy
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from detection_msgs.msg import BoundingBoxes  # 確保 detection_msgs 正確定義在你的工作空間
from std_srvs.srv import Empty
from move_base_msgs.msg import MoveBaseActionResult
import math

# 全域變數
stopflag = False  # 表示是否需要停止機器人
navigationflag = False  # 表示是否需要導航機器人
navigation_completed = False  # 表示導航是否完成
vel_cmd = Twist()  # 速度指令訊息
nav_goal = PoseStamped()  # 導航目標訊息
current_goal = PoseStamped()  # move_base目前目標
min_distance = float('inf')  # 最近障礙物距離
scan_sub = None  # 雷射掃描資料的訂閱者

# 目標點（轉換後的地圖座標）
goal_x = 0.1028457608955656
goal_y = -0.24373510447303082
goal_z = 0.0
goal_orientation_x = 0.0
goal_orientation_y = 0.0
goal_orientation_z = -0.7447992380767343
goal_orientation_w = 0.6672886144392964

def LaserCallback(msg):
    global min_distance
    valid_ranges = [r for r in msg.ranges if 0.05 < r < msg.range_max]  # 過濾掉無效的零距離讀數和過大讀數
    if valid_ranges:
        min_distance = min(valid_ranges)
    else:
        min_distance = float('inf')  # 如果沒有有效資料，設定為無限大
    rospy.loginfo(f"目前距離障礙物的最短距離: {min_distance:.2f} 公尺")

def ThumbCallback(msg):
    global stopflag, navigationflag, nav_goal, scan_sub, navigation_completed
    navigation_completed = False  # 重置導航完成標誌
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
    current_goal = msg  # 更新目前目標點

def handle_obstacle_detection(gesture):
    global nav_goal, current_goal, vel_cmd, scan_sub

    rospy.loginfo(f"偵測到 {gesture}，開始進行障礙物檢測。")
    
    # 訂閱雷射掃描資料主題
    scan_sub = rospy.Subscriber("/scan", LaserScan, LaserCallback)
    rate = rospy.Rate(20)
    
    # 等待一段時間以確保收到雷射掃描資料
    rospy.sleep(2)
    
    if min_distance < 0.4:  # 更新為40公分
        rospy.loginfo("障礙物過近，開始後退")
        while min_distance < 0.4:  # 更新為40公分
            vel_cmd.linear.x = -0.1  # 後退
            vel_cmd.angular.z = 0.0
            vel_pub.publish(vel_cmd)
            rate.sleep()
        vel_cmd.linear.x = 0.0
        vel_pub.publish(vel_cmd)
        rospy.loginfo("後退至安全距離")
    
    # 停止雷射掃描訂閱
    if scan_sub:
        scan_sub.unregister()
    
    # 原地自轉一圈以清除本地地圖
    rotate_360(rate)
    clear_local_map()

    # 設定導航目標點
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
        rospy.loginfo(f"導航至預設目標點: ({goal_x}, {goal_y}, {goal_z})")
    elif gesture == "thumb2":
        nav_goal = current_goal  # 使用目前目標點作為新的導航目標
        rospy.loginfo("導航至目前目標點")

def rotate_360(rate):
    global vel_cmd
    rotation_speed = 0.3  # 弧度/秒
    rotation_duration = (2 * math.pi) / rotation_speed  # 時間 = 角度 / 角速度
    start_time = rospy.Time.now()
    while rospy.Time.now() - start_time < rospy.Duration(rotation_duration):
        vel_cmd.angular.z = rotation_speed
        vel_pub.publish(vel_cmd)
        rate.sleep()
    vel_cmd.angular.z = 0.0
    vel_pub.publish(vel_cmd)
    rospy.loginfo("完成360度旋轉")

def clear_local_map():
    rospy.wait_for_service('/move_base/clear_costmaps')
    try:
        clear_costmaps = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        clear_costmaps()
        rospy.loginfo("本地地圖已清除")
    except rospy.ServiceException as e:
        rospy.logwarn(f"服務呼叫失敗: {e}")

def navigation_result_callback(msg):
    global navigation_completed
    if msg.status.status == 3:  # 目標達成
        navigation_completed = True
        rospy.loginfo("導航目標已達成")

if __name__ == '__main__':
    # 初始化ROS節點
    rospy.init_node("thumbdetect")

    # 訂閱邊界框偵測主題
    rospy.Subscriber("/yolov5/detections", BoundingBoxes, ThumbCallback)

    # 訂閱move_base的目標點主題
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, GoalCallback)

    # 訂閱move_base的導航結果主題
    rospy.Subscriber("/move_base/result", MoveBaseActionResult, navigation_result_callback)

    # 設定速度指令發布者
    vel_pub = rospy.Publisher("cmd_vel_1", Twist, queue_size=10)
    rate = rospy.Rate(20)

    # 設定導航目標發布者
    nav_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)

    while not rospy.is_shutdown():
        if stopflag:
            vel_cmd.linear.x = 0.0
            vel_cmd.angular.z = 0.0
            vel_pub.publish(vel_cmd)
            rospy.loginfo("緊急停止")
        elif navigationflag:
            nav_pub.publish(nav_goal)
            rospy.loginfo(f"導航至目標點: ({nav_goal.pose.position.x}, {nav_goal.pose.position.y}, {nav_goal.pose.position.z})")
            navigationflag = False

        if navigation_completed:
            rospy.loginfo("目標已達成，停止導航")
            nav_goal = PoseStamped()  # 清空導航目標
            stopflag = True  # 停止機器人
            navigation_completed = False  # 重置導航完成標誌

        rate.sleep()
