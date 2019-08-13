#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import actionlib
from std_msgs.msg import String
from location.srv import *
from move.msg import AmountAction
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from geometry_msgs.msg import Point, Quaternion

from abstract_module import AbstractModule
from module.rviz_marker import RvizMarker


class HmcBackToTheCar(AbstractModule):
    def __init__(self):
        super(HmcBackToTheCar, self).__init__(node_name="hmc_back_to_the_car")
        
        self.move_base_client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        self.amount_client = actionlib.SimpleActionClient("/move/amount", AmountAction)
        self.marker = RvizMarker()
        
        rospy.Subscriber("/human_detection/finish", String, self.back_to_the_car)
    
    def back_to_the_car(self, argument):
        # type: (String) -> None
        """
        human_detection終了後,協力者を連れて車に戻る
        :param: argument: 関数用の引数が格納されている.
        本関数では空である.
        :return: なし
        """
        self.print_node(argument.data)
        self.speak("I want you to help carrying groceries into the house.")
        self.speak("Please follow me.")
        rospy.wait_for_service('/location/request_location', timeout=1)
        coordinate = rospy.ServiceProxy('/location/request_location', RequestLocation)("car").location
        self.send_move_base(coordinate)
        self.speak("Here is the car")
    
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
    HmcBackToTheCar()
    rospy.spin()
