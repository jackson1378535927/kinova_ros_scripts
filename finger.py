#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose
from copy import deepcopy

import roslib; roslib.load_manifest('kinova_demo')
import actionlib
import kinova_msgs.msg
import argparse

class MoveItIkDemo:

    def __init__(self):
       
    
        
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        # 初始化ROS节点
        rospy.init_node('real_jaco_moveit_fixed_grasp')
        cartesian = rospy.get_param('~cartesian', True)
        # 初始化需要使用move group控制的机械臂中的arm group
        arm = moveit_commander.MoveGroupCommander('arm')
        # 初始化需要使用move group控制的机械臂中的gripper group
        gripper = moveit_commander.MoveGroupCommander('gripper')
        # 获取终端link的名称
        end_effector_link = arm.get_end_effector_link()
        # 设置目标位置所使用的参考坐标系
        reference_frame = 'world'
        arm.set_pose_reference_frame(reference_frame)
        # 当运动规划失败后，允许重新规划
        arm.allow_replanning(True)
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        arm.set_goal_position_tolerance(0.001)
        arm.set_goal_orientation_tolerance(0.001)
        gripper.set_goal_joint_tolerance(0.001)
        # 设置允许的最大速度和加速度
        arm.set_max_acceleration_scaling_factor(0.5)
        arm.set_max_velocity_scaling_factor(0.4)
        # 初始化场景对象
        scene = PlanningSceneInterface()
        rospy.sleep(1)


        finger_positions=[3700,3700,0]
        action_address = '/' + 'j2s7s300_' + 'driver/fingers_action/finger_positions'

        client = actionlib.SimpleActionClient(action_address,
                                          kinova_msgs.msg.SetFingersPositionAction)
        client.wait_for_server()

        goal = kinova_msgs.msg.SetFingersPositionGoal()
        goal.fingers.finger1 = float(finger_positions[0])
        goal.fingers.finger2 = float(finger_positions[1])
        # The MICO arm has only two fingers, but the same action definition is used
        if len(finger_positions) < 3:
                goal.fingers.finger3 = 0.0
        else:
                goal.fingers.finger3 = float(finger_positions[2])
        client.send_goal(goal)
        

        # 控制机械臂先回到初始化位置，手爪打开
        #arm.set_named_target('Home')
        #arm.go()
        #gripper.set_named_target('Open')
        #gripper.go()
        #rospy.sleep(1)
'''
        gripper.set_joint_value_target([0.5,1.0,0.0])
        gripper.go()
        rospy.sleep(1)
'''


if __name__ == "__main__":
    MoveItIkDemo()