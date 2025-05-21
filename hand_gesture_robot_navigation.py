#!/usr/bin/env python3
import rospy  # 匯入ROS的Python庫
from geometry_msgs.msg import Twist  # 匯入幾何訊息中的Twist消息
from detection_msgs.msg import BoundingBoxes  # 匯入自定義的BoundingBoxes消息

# 初始化速度命令
vel_cmd = Twist()

# 定義速度和距離參數
forward_speed = 0.3  # 前進速度（公尺/秒）
backward_speed = -0.3  # 後退速度（公尺/秒）
distance = 0.3  # 移動距離（公尺）
linear_duration = distance / abs(forward_speed)  # 移動所需時間（秒）

# 定義角速度和旋轉角度
angular_speed = 1.57  # 角速度（弧度/秒），約每秒旋轉90度
rotation_angle = 1.57  # 旋轉角度（弧度），90度
angular_duration = rotation_angle / angular_speed  # 旋轉所需時間（秒）

# 手勢檢測計數器和閾值
gesture_count = {"open": 0, "close": 0, "six": 0}  # 計數每種手勢檢測到的次數
gesture_threshold = 5  # 需要連續檢測的次數以觸發動作

# 重置所有手勢計數器
def reset_gesture_count():
    global gesture_count  # 宣告使用全域變數gesture_count
    for key in gesture_count:  # 對每個手勢類型
        gesture_count[key] = 0  # 將計數重置為0

# 手勢檢測的回呼函數
def ThumbCallback(msg):
    global vel_cmd, gesture_count  # 宣告使用全域變數vel_cmd和gesture_count

    for bbox in msg.bounding_boxes:  # 對每個邊界框進行迭代
        if bbox.probability > 0.93 and bbox.Class in gesture_count:  # 如果概率大於0.93且手勢類型在計數字典中
            gesture_count[bbox.Class] += 1  # 增加該手勢類型的計數
        else:
            gesture_count[bbox.Class] = 0  # 否則重置該手勢類型的計數

        if gesture_count[bbox.Class] >= gesture_threshold:  # 如果手勢計數達到閾值
            if bbox.Class == "open":  # 如果手勢類型是open
                vel_cmd.linear.x = forward_speed  # 設置線速度為前進速度
                vel_cmd.angular.z = 0.0  # 設置角速度為0
                vel_pub.publish(vel_cmd)  # 發佈速度命令
                rospy.sleep(linear_duration)  # 暫停運行以移動指定時間
                vel_cmd.linear.x = 0.0  # 停止移動
                vel_pub.publish(vel_cmd)  # 發佈停止命令
                rospy.loginfo("檢測到 open，向前移動 30 公分")  # 記錄日誌信息
                reset_gesture_count()  # 重置手勢計數器
                break  # 跳出迴圈
            elif bbox.Class == "close":  # 如果手勢類型是close
                vel_cmd.linear.x = backward_speed  # 設置線速度為後退速度
                vel_cmd.angular.z = 0.0  # 設置角速度為0
                vel_pub.publish(vel_cmd)  # 發佈速度命令
                rospy.sleep(linear_duration)  # 暫停運行以移動指定時間
                vel_cmd.linear.x = 0.0  # 停止移動
                vel_pub.publish(vel_cmd)  # 發佈停止命令
                rospy.loginfo("檢測到 close，向後移動 30 公分")  # 記錄日誌信息
                reset_gesture_count()  # 重置手勢計數器
                break  # 跳出迴圈
            elif bbox.Class == "six":  # 如果手勢類型是six
                vel_cmd.linear.x = 0.0  # 設置線速度為0
                vel_cmd.angular.z = angular_speed  # 設置角速度為旋轉速度
                vel_pub.publish(vel_cmd)  # 發佈速度命令
                rospy.sleep(angular_duration)  # 暫停運行以旋轉指定時間
                vel_cmd.angular.z = 0.0  # 停止旋轉
                vel_pub.publish(vel_cmd)  # 發佈停止命令
                rospy.loginfo("檢測到 six，向左旋轉 90 度")  # 記錄日誌信息
                reset_gesture_count()  # 重置手勢計數器
                break  # 跳出迴圈

if __name__ == '__main__':
    rospy.init_node("thumbdetect")  # 初始化ROS節點
    rospy.Subscriber("/yolov5/detections", BoundingBoxes, ThumbCallback)  # 訂閱手勢檢測消息
    vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)  # 創建速度命令發佈者
    rate = rospy.Rate(20)  # 設置發布頻率為20Hz

    while not rospy.is_shutdown():  # 當ROS未關閉時
        rate.sleep()  # 保持迴圈頻率
