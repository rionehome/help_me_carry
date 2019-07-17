#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from hmc_start_node.msg import Activate
from sound_system.srv import *
from abstract_module import AbstractModule


class ThirdHumanDetect(AbstractModule):

    def __init__(self):
        # クラス継承を行う
        super(ThirdHumanDetect, self).__init__(node_name="hmc_human_detect", activate_no=3)

    ########################################################
    #       この main 関数内が一連の処理の流れとなる           #
    ########################################################
    def main(self):
        # 現状、人間検出の処理がどうなるか不明なので一旦保留

        self.speak("Skip human detection.")
        self.next_node_activate()

    ########################################################
    #               それぞれの分岐する処理                    #
    ########################################################




if __name__ == "__main__":
    ThirdHumanDetect()
    rospy.spin()
