#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import actionlib
from std_msgs.msg import String, Int32
from location.srv import *
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from geometry_msgs.msg import Point, Quaternion

from abstract_module import AbstractModule
from module.rviz_marker import RvizMarker


class HmcSendPlaceMsg(AbstractModule):
    def __init__(self):
        super(HmcSendPlaceMsg, self).__init__(node_name="hmc_send_place_msg")
        self.place = ""
        
        self.arm_pub = rospy.Publisher("/arm/control", Int32, queue_size=10)
        self.human_detection_pub = rospy.Publisher("/human_detection/control", String, queue_size=10)
        self.move_base_client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        self.marker = RvizMarker()
        
        rospy.Subscriber("/natural_language_processing/send_place_msg", String, self.send_place_msg)
        rospy.Subscriber("/hmc/send_place_msg", String, self.save_place_info)
    
    def send_place_msg(self, argument):
        # type: (String) -> None
        """
        bagを運ぶ先の場所情報をpublishする
        :param argument: ゴミ
        :return: なし
        """
        self.print_node(argument.data)
        self.arm_pub.publish()
        print self.place
        self.speak("OK, I will go to the {}.".format(self.place))
        self.arm_pub.publish(2)
        rospy.sleep(3)
        rospy.wait_for_service('/location/request_location', timeout=1)
        coordinate = rospy.ServiceProxy('/location/request_location', RequestLocation)(self.place).location
        self.send_move_base(coordinate)
        self.release_bag()
        self.human_detection_pub.publish("start")
    
    def release_bag(self):
        # type: () -> None
        """
        目的地到着後,バッグを置く.
        :param: argument: 関数用の引数が格納されている.
        本関数では空である.
        :return: なし
        """
        self.speak('I put the bag')
        self.arm_pub.publish(3)
        rospy.sleep(6)
        self.arm_pub.publish(4)
        rospy.sleep(4)
    
    def save_place_info(self, place):
        """
        場所情報を保管するためのcallback関数
        :param place: 場所名
        :return: なし
        """
        self.place = place.data
        print self.place
    
    def send_move_base(self, point):
        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position = Point(point.x, point.y, point.z)
        goal.target_pose.pose.orientation = Quaternion(0, 0, 0, 1)
        self.marker.register(goal.target_pose.pose)
        self.move_base_client.wait_for_server()
        print "move_baseに送信"
        self.move_base_client.send_goal(goal)
        self.move_base_client.wait_for_result()
        return self.move_base_client.get_state()


if __name__ == "__main__":
    HmcSendPlaceMsg()
    rospy.spin()
