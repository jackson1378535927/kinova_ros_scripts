#! /usr/bin/env python
"""A helper program to test cartesian goals for the JACO and MICO arms."""

import roslib; roslib.load_manifest('kinova_demo')
import rospy
import sys
import numpy as np
import actionlib
import kinova_msgs.msg
import std_msgs.msg
import geometry_msgs.msg
import math
import argparse
import time

def gripper_client(finger_positions):
    """Send a gripper goal to the action server."""
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

def joint_angle_client(angle_set):
    """Send a joint angle goal to the action server."""
    action_address = '/' + 'j2s7s300_' + 'driver/joints_action/joint_angles'
    client = actionlib.SimpleActionClient(action_address,
                                          kinova_msgs.msg.ArmJointAnglesAction)
    client.wait_for_server()

    goal = kinova_msgs.msg.ArmJointAnglesGoal()

    goal.angles.joint1 = angle_set[0]
    goal.angles.joint2 = angle_set[1]
    goal.angles.joint3 = angle_set[2]
    goal.angles.joint4 = angle_set[3]
    goal.angles.joint5 = angle_set[4]
    goal.angles.joint6 = angle_set[5]
    goal.angles.joint7 = angle_set[6]

    client.send_goal(goal)

def cartesian_pose_client(position, orientation):
    """Send a cartesian goal to the action server."""
    action_address = '/' + 'j2s7s300_' + 'driver/pose_action/tool_pose'
    client = actionlib.SimpleActionClient(action_address, kinova_msgs.msg.ArmPoseAction)
    client.wait_for_server()

    goal = kinova_msgs.msg.ArmPoseGoal()
    goal.pose.header = std_msgs.msg.Header(frame_id=('j2s7s300_' + 'link_base'))
    goal.pose.pose.position = geometry_msgs.msg.Point(
        x=position[0], y=position[1], z=position[2])
    goal.pose.pose.orientation = geometry_msgs.msg.Quaternion(
        x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3])

    # print('goal.pose in client 1: {}'.format(goal.pose.pose)) # debug

    client.send_goal(goal)


if __name__=='__main__':
    rospy.init_node('j2s7s300_control')
    target_position=[0.35,-0.05,0.535]
    target_orientation=[0.506526,0.506129,0.493391,0.493792]
    cartesian_pose_client(target_position,target_orientation)
    time.sleep(5)
    target_position[0]+=0.1
    cartesian_pose_client(target_position,target_orientation)
    time.sleep(5)
    target_finger=[6000,6000,0]
    gripper_client(target_finger)
    time.sleep(5)
    target_position[0]-=0.15
    cartesian_pose_client(target_position,target_orientation)
