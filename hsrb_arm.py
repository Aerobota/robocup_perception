#!/usr/bin/env python
# -*- coding: utf-8 -*-

import hsrb_interface
import rospy
import sys
from hsrb_interface import geometry
from std_msgs.msg import String
from std_msgs.msg import Bool

#オブジェクトAR
_GRASP_OBJECT_ = 'recognized_object/15'
#ハンド（手の平のtf名)
_HAND_TF_= 'hand_palm_link'
#リストのtf名
_WRIST_ROLL_ = 'wrist_roll_link'

#トルク指定
_GRASP_TORQUE_=-0.01

#接続の確立とインスタンスの生成
robot = hsrb_interface.Robot()
whole_body = robot.get('whole_body')
gripper = robot.get('gripper')

grip_hand = geometry.pose(z=-0.02, ek=-1.57)

if __name__=='__main__':
    gripper.command(1.2)
    whole_body.move_to_neutral()

    try:
        whole_body.move_end_effector_pose(grip_hand, _GRASP_OBJECT_)
        gripper.grasp(_GRASP_TORQUE_)
        rospy.sleep(2.0)
        whole_body.move_to_neutral()
        sys.exit()
    except:
        sys.exit()
