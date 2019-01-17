#!/usr/bin/env python

# This program was written by: Abdulrhman Saad & Mohanned Ahmed 

# USAGE
# python bug0_v4c.py -x 2 -y 2

import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
from geometry_msgs.msg import Twist
import time 
import tf
import argparse

# Globals 
x_robot = 0
y_robot = 0
yaw = 0
x_goal = 2.0
y_goal = 2.0
min_value_right = 0
min_value_front = 0
min_value_left = 0 
is_stop_moving = False 
vel_msg = Twist()

# Construct the argument parser and parse the arguments.
ap = argparse.ArgumentParser()
ap.add_argument("-x", "--x-goal", type=int, required=True,
	help="x coordinate of the goal.")
ap.add_argument("-y", "--y-goal", type=int, required=True,
	help="y coordinate of the goal.")
args = vars(ap.parse_args())

x_goal = args['x_goal']
y_goal = args['y_goal']

def scan_callback(scan_data):
    global min_value_right, min_value_front, min_value_left
    ranges = scan_data.ranges
    front_range = ranges[350:359] + ranges[0:20]
    right_range = ranges[80:90]
    left_range = ranges[290:299]
    min_value_left = min(left_range)
    min_value_right = min(right_range)
    min_value_front = min(front_range)

def pose_callback(pose_data):
	global x_robot, y_robot, yaw
	
	x_robot = pose_data.pose.pose.position.x
	y_robot = pose_data.pose.pose.position.y

	quaternion = (
    pose_data.pose.pose.orientation.x,
    pose_data.pose.pose.orientation.y,
    pose_data.pose.pose.orientation.z,
    pose_data.pose.pose.orientation.w)
	rpy = tf.transformations.euler_from_quaternion(quaternion)
	yaw = rpy[2]

def rotate_to_goal_state(): # Rotate towards goal. 
	print(">>>>> Rotate towards goal")
	global x_goal,y_goal,yaw, vel_msg
	desired_angle_goal = math.atan2(y_goal-y_robot,x_goal-x_robot)
	K_angular = 0.5
	angular_speed = (desired_angle_goal-yaw)*K_angular
	while True:
		vel_msg.angular.z = angular_speed
		velocity_publisher.publish(vel_msg)
		if desired_angle_goal < 0: 
			if ((desired_angle_goal)-yaw) > -0.1:
				break
		else: 
			if ((desired_angle_goal)-yaw) < 0.1:
				break
	vel_msg.angular.z=0
	velocity_publisher.publish(vel_msg)	

def move_forward_state():
	print(">>>>> Move forward")
	vel_msg = Twist()
	vel_msg.linear.x = 0.3
	velocity_publisher.publish(vel_msg)
	return

def follow_wall_state():
	if min_value_right > min_value_left:
		angle = get_wall_angle(True) 
		print(">>>>> Change angle to right (90 degrees): ", angle) 				
	else: 
		angle = get_wall_angle(False) 
		print(">>>>> Change angle to left (90 degrees): ", angle)					
	if (angle-yaw) < 0:					
		while (angle-yaw) < -0.1: 	
			vel_msg.angular.z = (angle-yaw)*0.9
			velocity_publisher.publish(vel_msg)
	else: 
		while (angle-yaw) > 0.1: 	
			vel_msg.angular.z = (angle-yaw)*0.9
			velocity_publisher.publish(vel_msg)
	vel_msg.angular.z = 0
	# Move robot forward (for amount of time) after each rotation to prevent it from getting stuck.
	if min_value_front > 0.4:
		time = 0
		t0 = rospy.Time.now().to_sec()
		while time < 1.3 and min_value_front > 0.4:
			t1 = rospy.Time.now().to_sec()
			time = t1-t0
			vel_msg.linear.x = 0.3
			velocity_publisher.publish(vel_msg)


def is_goal_reached(): # Check if goal is reached.
	if (x_goal - 0.5 < x_robot < x_goal + 0.5) and (y_goal - 0.5 < y_robot < y_goal + 0.5):  
		return True 
	return False

def is_towards_goal(): # Check if robot in direction of the goal.
	desired_angle_goal = math.atan2(y_goal-y_robot,x_goal-x_robot)
	if abs((desired_angle_goal)-yaw) < 0.4:
		return True
	return False 

def get_wall_angle(is_right): # Get the wall angle. 
	angle = 0 
	if ((min_value_left > 0.6 and yaw > 0) or (min_value_right > 0.6 and yaw < 0)) and not ((0.0 < yaw < 0.002) or (-3.16< yaw <-3.0)):
		angle = 0.0 if is_right else -math.pi
	else: 
		angle = -math.pi/2 if is_right else math.pi/2
	return angle     
 
if __name__ == '__main__':
	# Set ROS nodes. 
	rospy.init_node('scan_node', anonymous=True)
	rospy.Subscriber("/scan", LaserScan, scan_callback)
	rospy.Subscriber("/odom", Odometry, pose_callback)
	cmd_vel_topic = "/cmd_vel"
	velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
	time.sleep(2.0)
	while True:
		# Print data. 
		print(">>>>> Data")
		print("min_value_right: " + str(min_value_right))
		print("min_value_left: " + str(min_value_left))
		print("min_value_front: " + str(min_value_front))
		print("yaw: " + str(yaw))
		if min_value_front > 0.4:
			# If robot is not following wall...
			if not ((min_value_right < 0.4 and yaw < 0) or (min_value_left < 0.4 and yaw > 0)):  
				rotate_to_goal_state()
				is_stop_moving = False 
			while min_value_front > 0.4:
				move_forward_state()
				# If robot is not following wall...
				if not ((min_value_right < 0.4 and yaw < 0) or (min_value_left < 0.4 and yaw > 0)):
					# If robot is not in the direction of the goal...
					if (not is_towards_goal()) and not is_stop_moving: 
						print(">>>>> Stop moving forward")
						vel_msg.angular.z = 0.0
						vel_msg.linear.x = 0.0
						velocity_publisher.publish(vel_msg)
						is_stop_moving = True 
						break
							
		else:
			follow_wall_state()

		# Do some cleaning.
		vel_msg.angular.z = 0.0
		vel_msg.linear.x = 0.0
		velocity_publisher.publish(vel_msg)
		
		print(">>>>> Iteration Completed")
		

		# Check is goal reached. 
		is_reached = is_goal_reached()
		if is_reached == True:
			print(">>>>> Goal is reached")
			break
