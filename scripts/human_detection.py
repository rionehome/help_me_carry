#!/usr/bin/env python
# coding: UTF-8
import math

from move.msg import AmountGoal, AmountAction
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
import rospy
import actionlib
from sound_system.srv import StringService
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from ros_posenet.msg import *
from geometry_msgs.msg import Point, Quaternion
from module.rviz_marker import RvizMarker

MARGIN = 0.8


class HumanDetection:
    def __init__(self):
        self.cv_image = None
        self.point_list = ["nose", "leftEye", "rightEye", "leftEar", "rightEar", "leftShoulder", "rightShoulder"]
        self.sensor_x = 0
        self.sensor_y = 0
        self.sensor_degree = 0
        self.flag = True
        self.marker = RvizMarker()
        self.person_position = {}
        
        rospy.init_node("human_detection")
        rospy.Subscriber("/ros_posenet/result", Poses, self.pose_callback, queue_size=1)
        rospy.Subscriber("/human_detection/control", String, self.control_callback, queue_size=1)
        rospy.Subscriber("/odom", Odometry, self.odometry_callback)
        self.move_base_client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        self.amount_client = actionlib.SimpleActionClient("/move/amount", AmountAction)
        self.finish_pub = rospy.Publisher("/human_detection/finish", String, queue_size=1)
    
    @staticmethod
    def print_node(name):
        # type: (str) -> None
        """
        ノード名を表示
        :return: なし
        """
        print("\n###########################################\n")
        print("     Node: {}".format(name))
        print("\n###########################################\n")
    
    @staticmethod
    def to_angle(rad):
        return rad * 180 / math.pi
    
    @staticmethod
    def to_radian(angle):
        return (angle * math.pi) / 180
    
    def to_quaternion_ang(self, w, z):
        if abs(z > 0):
            return self.to_angle(math.acos(w) * 2)
        else:
            return 360 - self.to_angle(math.acos(w) * 2)
    
    def send_move_base(self, point):
        # type:(tuple)->int
        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position = Point(point[0], point[1], point[2])
        goal.target_pose.pose.orientation = Quaternion(0, 0, 0, 1)
        self.marker.register(goal.target_pose.pose)
        self.move_base_client.wait_for_server()
        print "move_baseに送信"
        self.move_base_client.send_goal(goal)
        self.move_base_client.wait_for_result()
        return self.move_base_client.get_state()
    
    @staticmethod
    def speak(sentence):
        # type: (str) -> None
        """
        発話関数
        :param sentence:
        :return:
        """
        rospy.wait_for_service("/sound_system/speak")
        rospy.ServiceProxy("/sound_system/speak", StringService)(sentence)
    
    def move_turn(self, angle):
        """
        角度送信
        :param angle:
        :return:
        """
        goal = AmountGoal()
        goal.amount.angle = angle
        goal.velocity.angular_rate = 0.3
        
        self.amount_client.wait_for_server()
        self.amount_client.send_goal(goal)
        self.amount_client.wait_for_result(rospy.Duration(30))
    
    def calc_person_position(self, pose):
        try:
            # 人間の三次元座標を計算
            count = 0
            sum_x = 0
            sum_y = 0
            sum_z = 0
            for key in pose.keypoints:
                if key.part in self.point_list:
                    sum_x += key.position.x
                    sum_y += key.position.y
                    sum_z += key.position.z
                    count += 1
            if count < 5:
                return None
            ave_x = sum_x / count
            ave_y = sum_y / count
            ave_z = sum_z / count
            distance = math.sqrt(ave_x ** 2 + ave_y ** 2 + ave_z ** 2)
            return distance, (ave_x, ave_y, ave_z)
        except ZeroDivisionError:
            print "countが0 @raise_hand"
        return None
    
    def calc_raise_person_position(self, persons):
        """
        手を上げている人の3D場所を計算
        :return:
        """
        sum_x = 0
        sum_y = 0
        for person in persons:
            sum_x += person[0]
            sum_y += person[1]
        ave_x = sum_x / len(persons)
        ave_y = sum_y / len(persons)
        
        result = self.calc_safe_position(MARGIN, (ave_x, ave_y))
        
        print result
        return result[0], result[1]
    
    def calc_real_position(self, point):
        # type:(tuple)->tuple
        relative_theta = self.sensor_degree
        relative_x = point[2]
        relative_y = point[0]
        
        delta_theta = math.atan(relative_y / relative_x)
        # x = relative_x * math.cos(relative_theta) - relative_y * math.sin(relative_theta)
        # y = relative_x * math.sin(relative_theta) + relative_y * math.cos(relative_theta)
        x = math.sqrt(relative_x ** 2 + relative_y ** 2) * math.cos(relative_theta + delta_theta)
        y = math.sqrt(relative_x ** 2 + relative_y ** 2) * math.sin(relative_theta + delta_theta)
        print "real", x, y
        return x, y
    
    @staticmethod
    def calc_safe_position(margin, person_position):
        # type: (float,tuple)->tuple
        """
        人間を中心にdistanceを半径とした円と、人間からロボットまで結んだ直線の交点を計算
        :param margin:
        :param person_position:
        :return:
        """
        real_person_x = person_position[0]
        real_person_y = person_position[1]
        """
        distance = math.sqrt((real_person_x - self.sensor_x) ** 2 + (real_person_y - self.sensor_y) ** 2)
        # 三角形の相似を利用して算出
        safety_person_x = real_person_x - (margin * abs(real_person_x - self.sensor_x)) / distance
        safety_person_y = real_person_y - (margin * (real_person_y - self.sensor_y)) / distance
        """
        safety_person_x = real_person_x - margin
        safety_person_y = real_person_y
        
        return safety_person_x, safety_person_y
    
    ################################################################################
    def control_callback(self, msg):
        # type:(String)->None
        self.print_node("human_detection")
        if msg.data == "start":
            self.flag = False
            self.move_turn(360)
            self.flag = True
            if len(self.person_position) == 0:
                self.speak("sorry, not found.")
                self.finish_pub.publish(String(data="finish"))
                return
            print min(self.person_position)
            self.move_turn(self.person_position[min(self.person_position)][1] + 30)
            self.finish_pub.publish(String(data="finish"))
    
    def odometry_callback(self, msg):
        # type: (Odometry)->None
        """
        位置情報の受け取り
        :param msg:
        :return:
        """
        self.sensor_x = msg.pose.pose.position.x
        self.sensor_y = msg.pose.pose.position.y
        self.sensor_degree = self.to_quaternion_ang(msg.pose.pose.orientation.w, msg.pose.pose.orientation.z)
    
    def pose_callback(self, msgs):
        # type: (Poses)->None
        """
        関節推定の結果を受け取り
        :param msgs:
        :return:
        """
        if self.flag:
            return
        
        for pose in msgs.poses:
            result = self.calc_person_position(pose)
            if result is None:
                continue
            print self.sensor_degree
            self.person_position.setdefault(result[0], (result[1], self.sensor_degree))


if __name__ == '__main__':
    HumanDetection()
    rospy.spin()
