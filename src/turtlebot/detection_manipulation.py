#!/usr/bin/env python

# import move_group_python_interface as mv
# import rospy
# import numpy as np

# def main():
#   try:
#     manipulation = mv.MoveGroupPythonInteface()
#     posi = np.array([0.4,0.0,0.05])
#     manipulation.go_to_pose_goal(posi)

#     print "============detectation and manipualtion complete============"
#   except rospy.ROSInterruptException:
#     return
#   except KeyboardInterrupt:
#     return

# if __name__ == '__main__':
#   main()

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import math
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float64MultiArray, Float64
import sensor_msgs
import ctypes
import struct
import time

color_diff = 20
# red_ref = [115,67,47]
# yellow_ref = [150,150,50]
# blue_ref = [15,84,90]


def isColorCorrect(rgbList,colorname):
	if (colorname == 'red'):
		return ((rgbList[0]-rgbList[1] >= color_diff) and (rgbList[0]-rgbList[2] >= color_diff))
	elif (colorname == 'green'):
		return ((rgbList[1]-rgbList[0] >= color_diff) and (rgbList[1]-rgbList[2] >= color_diff))
	elif (colorname == 'blue'):
		return ((rgbList[2]-rgbList[0] >= color_diff) and (rgbList[2]-rgbList[1] >= color_diff))

# subscribe to pointcloud topic define the function that does the parsing
def pointcloudCallBack(PointCloud):
	# self.lock.acquire()
	gen = pc2.read_points(PointCloud, skip_nans=True)
	int_data = list(gen)
	x_bar = 0 
	y_bar = 0 
	z_bar = 0 
	num_points = 0
	for x in int_data:
		test = x[3]
		# cast float32 to int so that bitwise operations are possible
		s = struct.pack('>f', test)
		i = struct.unpack('>l', s)[0]
		# you can get back the float value by the inverse operations
		pack = ctypes.c_uint32(i).value
		# r,g,b values in the 0-255 range
		r = (pack & 0x00FF0000)>> 16
		g = (pack & 0x0000FF00)>> 8
		b = (pack & 0x000000FF)
		rgbList = [r,g,b]
		# print rgbList
		if isColorCorrect(rgbList,'red') : 
			num_points += 1
			# x,y,z can be retrieved from the x[0],x[1],x[2]
			x_bar += x[1]
			y_bar += x[2]
			z_bar += x[3]
	if (num_points != 0):
		x_bar /= num_points
		y_bar /= num_points
		z_bar /= num_points
		print('Marker Position')
		print(num_points)
		print([x_bar,y_bar,z_bar])
		posi = Float64MultiArray(data = [x_bar,y_bar,z_bar])
		manipulation_pub = rospy.Publisher('/manipulation_target', Float64MultiArray, queue_size = 10)
		manipulation_pub.publish(posi)
	
def manipulationCallBack(posi):
	moveit_commander.roscpp_initialize(sys.argv)
	robot = moveit_commander.RobotCommander()
	scene = moveit_commander.PlanningSceneInterface()
	group = moveit_commander.MoveGroupCommander("pincher_arm")
	display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size = 10)
	pose_target = geometry_msgs.msg.Pose()
	pose_target.orientation.w = 1
	pose_target.position.x = posi.data[1] - 0.7
	pose_target.position.y = -posi.data[0]
	pose_target.position.z = posi.data[2] + 0.2
	pose_target.position.x = 0.35
	pose_target.position.y = 0.05
	pose_target.position.z = 0.05
	group.set_pose_target(pose_target)
	group.set_goal_orientation_tolerance(30*math.pi/180)
	plan = group.plan()
	group.go(wait=True)

def manipulationDestinationCallBack(posi): # desired destination
	robot = moveit_commander.RobotCommander()
	scene = moveit_commander.PlanningSceneInterface()
	group = moveit_commander.MoveGroupCommander("pincher_arm")
	manipulation_publisher = rospy.Publisher('/move_group/manipulation_target',moveit_msgs.msg.DisplayTrajectory,queue_size = 10)
	pose_target = geometry_msgs.msg.Pose()
	pose_target.orientation.w = 1
	pose_target.position.x = float(posi[0])
	pose_target.position.y = float(posi[1])
	pose_target.position.z = float(posi[2])
	group.set_pose_target(pose_target)
	group.set_goal_orientation_tolerance(30*math.pi/180)
	plan = group.plan()
	group.go(wait=True)
def manipulationFixedDestinationCallBack(): # desired destination
	robot = moveit_commander.RobotCommander()
	scene = moveit_commander.PlanningSceneInterface()
	group = moveit_commander.MoveGroupCommander("pincher_arm")
	manipulation_publisher = rospy.Publisher('/move_group/manipulation_target',moveit_msgs.msg.DisplayTrajectory,queue_size = 10)
	pose_target = geometry_msgs.msg.Pose()
	pose_target.orientation.w = 1
	pose_target.position.x = 0.35
	pose_target.position.y = 0
	pose_target.position.z = 0.3
	group.set_pose_target(pose_target)
	group.set_goal_orientation_tolerance(30*math.pi/180)
	plan = group.plan()
	group.go(wait=True)

def gripperOpenCallBack():
	gripper_publisher = rospy.Publisher('gripper_joint/command',Float64, queue_size = 1)
  	gripper_publisher.publish(float(-0))
  	time.sleep(1)

def gripperCloseCallBack():
	gripper_publisher = rospy.Publisher('gripper_joint/command',Float64, queue_size = 1)
  	gripper_publisher.publish(float(-1.1))
  	time.sleep(1)


def main():
	try:
		rospy.init_node('move_group_python_interface',anonymous = True)
		manipulationFixedDestinationCallBack()
		gripperOpenCallBack()
		# while not rospy.is_shutdown():
			# object_detection_subscriber = rospy.Subscriber('/vox_camera_sr300/depth/points', PointCloud2, pointcloudCallBack)
			# manipulation_subscriber = rospy.Subscriber('/manipulation_target',Float64MultiArray,manipulationCallBack)
			# manipulationFixedDestinationCallBack()
			# manipulationDestinationCallBack()
			# gripperCloseCallBack()
			# manipulationFixedDestinationCallBack()
			# manipulationDestinationCallBack()
			# gripperOpenCallBack()
			# rospy.spin()
			# except rospy.ROSInterruptException:
		# 	return
		position1 = [0.32,0.05,0.01]
		destination1 = [0.38,-0.1,0.1]
		position2 = [0.36,0.1,0.01]
		destination2 = [0.33,-0.1,0.1]
		position3 = [0.4,0,0.01]
		destination3 = [0.28,-0.1,0.1]
		manipulationFixedDestinationCallBack()
		manipulationDestinationCallBack(position1)
		gripperCloseCallBack()
		manipulationFixedDestinationCallBack()
		manipulationDestinationCallBack(destination1)
		gripperOpenCallBack()

		manipulationFixedDestinationCallBack()
		manipulationDestinationCallBack(position2)
		gripperCloseCallBack()
		manipulationFixedDestinationCallBack()
		manipulationDestinationCallBack(destination2)
		gripperOpenCallBack()

		manipulationFixedDestinationCallBack()
		manipulationDestinationCallBack(position3)
		gripperCloseCallBack()
		manipulationFixedDestinationCallBack()
		manipulationDestinationCallBack(destination3)
		gripperOpenCallBack()
	except KeyboardInterrupt:
		return

if __name__ == '__main__':
  main()