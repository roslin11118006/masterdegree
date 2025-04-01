#!/usr/bin/env python3
# coding=utf-8
import rospy
from geometry_msgs.msg import Twist, PoseStamped
from detection_msgs.msg import BoundingBoxes  # 确保 detection_msgs 正确定义在你的工作空间
from std_srvs.srv import Empty

# 全局变量
stopflag = False  # 表示是否需要停止机器人
rotateflag = False  # 表示是否需要旋转机器人
navigationflag = False  # 表示是否需要导航机器人
rotation_completed = False  # 表示旋转是否完成
vel_cmd = Twist()  # 速度指令消息
nav_goal = PoseStamped()  # 导航目标消息
current_goal = PoseStamped()  # move_base当前目标


def ThumbCallback(msg):
    global stopflag, rotateflag, navigationflag, rotation_completed, nav_goal, current_goal
    for bbox in msg.bounding_boxes:
        if bbox.Class == "thumb1" and bbox.probability > 0.9:
            stopflag = True
            rotateflag = False
            navigationflag = False
            rotation_completed = False
            break
        elif bbox.Class == "close" and bbox.probability > 0.9:
            stopflag = False
            rotateflag = True
            navigationflag = False
            rotation_completed = False
            break
        elif bbox.Class == "thumb2" and bbox.probability > 0.9:
            stopflag = False
            rotateflag = False
            navigationflag = True
            rotation_completed = False
            nav_goal = current_goal  # 使用当前目标点作为新的导航目标
            break
        else:
            stopflag = False
            rotateflag = False
            navigationflag = False
            rotation_completed = False


def GoalCallback(msg):
    global current_goal
    current_goal = msg  # 更新当前目标点


if __name__ == '__main__':
    # 初始化ROS节点
    rospy.init_node("thumbdetect")

    # 订阅边界框检测主题
    rospy.Subscriber("/yolov5/detections", BoundingBoxes, ThumbCallback)

    # 订阅move_base的目标点主题
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, GoalCallback)

    # 设置速度命令发布者
    vel_pub = rospy.Publisher("cmd_vel_1", Twist, queue_size=10)
    rate = rospy.Rate(20)

    # 设置导航目标发布者
    nav_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)

    # 旋转计时器
    rotate_duration = rospy.Duration(5.0)  # 假设旋转一圈需要5秒
    rotate_start_time = None

    while not rospy.is_shutdown():
        if stopflag:
            vel_cmd.linear.x = 0.0
            vel_cmd.angular.z = 0.0
            vel_pub.publish(vel_cmd)
            rospy.loginfo("Emergency Stop")
        elif rotateflag and not rotation_completed:
            if rotate_start_time is None:
                rotate_start_time = rospy.Time.now()

            elapsed_time = rospy.Time.now() - rotate_start_time
            if elapsed_time < rotate_duration:
                vel_cmd.linear.x = 0.0
                vel_cmd.angular.z = 0.5  # 旋转速度
                vel_pub.publish(vel_cmd)
                rospy.loginfo("Rotating")
            else:
                vel_cmd.linear.x = 0.0
                vel_cmd.angular.z = 0.0
                vel_pub.publish(vel_cmd)
                rospy.loginfo("Rotation Completed")

                # 清除全局和本地地图
                rospy.wait_for_service('/move_base/clear_costmaps')
                try:
                    clear_costmaps = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
                    clear_costmaps()
                    rospy.loginfo("Costmaps Cleared")
                except rospy.ServiceException as e:
                    rospy.logerr("Service call failed: %s" % e)

                rotation_completed = True
                rotateflag = False
                rotate_start_time = None
        elif navigationflag:
            nav_pub.publish(nav_goal)
            rospy.loginfo("Navigating to goal")
            navigationflag = False

        rate.sleep()
