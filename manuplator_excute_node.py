#!/usr/bin/env python

import rospy, sys
import time
import moveit_commander
from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from copy import deepcopy
from moveit_commander import MoveGroupCommander
def __init__():
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        # 初始化ROS节点
        rospy.init_node('real_jaco_moveit_fixed_grasp')
        #允许笛卡尔坐标空间规划
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
        arm.set_max_velocity_scaling_factor(0.5)
        # 初始化场景对象
        scene = PlanningSceneInterface()
        rospy.sleep(1)
        arm.set_named_target('Home')
        arm.go()
def gohome():
        arm.set_named_target('Home')
        arm.go()
        rospy.sleep(3)
def grasp():
        gripper.set_named_target('Close')
        gripper.go()
        rospy.sleep(3)
def release():
        gripper.set_named_target('Open')
        gripper.go()
        rospy.sleep(3)
def goposition():
        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.header.stamp = rospy.Time.now()
        target_pose.pose.position.x = 0.2
        target_pose.pose.position.y = -0.2
        target_pose.pose.position.z = 0.235
        target_pose.pose.orientation.x = 0.14578
        target_pose.pose.orientation.y = 0.98924
        target_pose.pose.orientation.z = -0.0085346
        target_pose.pose.orientation.w = 0.0084136
        arm.set_start_state_to_current_state()
        # 设置机械臂终端运动的目标位姿
        arm.set_pose_target(target_pose, end_effector_link)
        # 规划运动路径
        traj = arm.plan()
        # 按照规划的运动路径控制机械臂运动
        arm.execute(traj)
        rospy.sleep(3)
def go_line_position(cur_position):
        target_pose = Pose()
        #target_pose.position.x = 0.265873
        #target_pose.position.y =- 0.212123
        #target_pose.position.z = 0.503074
        #target_pose.orientation.x = 0.637132
        #target_pose.orientation.y = 0.335003
        #target_pose.orientation.z = 0.517789
        #target_pose.orientation.w = 0.46231
        target_pose=cur_position
        # 设置机器臂当前的状态作为运动初始状态
        waypoints = []
        # 将初始位姿加入路点列表
        waypoints.append(target_pose)
        # 设置路点数据，并加入路点列表
        wpose = deepcopy(target_pose)
        wpose.position.x -= 0.15
        wpose.position.y -= 0.15
        wpose.position.z += 0.11
        waypoints.append(deepcopy(wpose))
        fraction = 0.0   #路径规划覆盖率
    	maxtries = 100   #最大尝试规划次数
    	attempts = 0     #已经尝试规划次数
    	# 设置机器臂当前的状态作为运动初始状态
    	arm.set_start_state_to_current_state()
    	# 尝试规划一条笛卡尔空间下的路径，依次通过所有路点
        while fraction < 1.0 and attempts < maxtries:
		    (plan, fraction) = arm.compute_cartesian_path (
		                waypoints,   # waypoint poses，路点列表
		                0.01,        # eef_step，终端步进值
		                0.0,         # jump_threshold，跳跃阈值
		                 True)        # avoid_collisions，避障规划
		    # 尝试次数累加
		    attempts += 1
		    # 打印运动规划进程
		    if attempts % 10 == 0:
		        rospy.loginfo("Still trying after " + str(attempts) + " attempts...")
	    # 如果路径规划成功（覆盖率100%）,则开始控制机械臂运动
        if fraction == 1.0:
		    rospy.loginfo("Path computed successfully. Moving the arm.")
		    arm.execute(plan)
		    rospy.loginfo("Path execution complete.")
		# 如果路径规划失败，则打印失败信息
        else:
		    rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")  
        rospy.sleep(5)
def callback(data):
        info = data.data
        pubF = rospy.Publisher("$[publish a feedback]", String, queue_size=1)

        if info == '$[commamd name]':
            rospy.Subscriber("$[sub the pose info]",Pose,go_line_position)
def manucontroller():
        rospy.Subscriber("$[give me a task publisher name]", String, callback)
        rospy.spin()