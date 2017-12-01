#!/usr/bin/env python

'''
		Project - Lakshya Priyadarshi : official.18lakshya@gmail.com
			  B.Tech 3rd Semester, Computer Science & Engineering
			  Institute of Engineering & Technology, Lucknow
		

		Overview- Chaser Drone based on ROS, ROS Whycon technology, simulated in Gazebo real-world physics simulator
			  Whycon employs flood-filling image-processing algorithm to detect markers
			  Proportional–integral–derivative control algorithm employed to control drone motion
	
			  Contributors : Lekhraj Singh (ECE), Yaduveer Singh (EE)

		Design -  Robot simulation in real-world environment simulator Gazebo
			  Codes + Simulator are integrated with ROS Core ( Robotics Operating System ) 
			  All nodes are published under Subscriber/Publisher scheme of ROScore / ROSTopic
			  The robot motion parameters are six-dimensional phase space [x,y,z,p_x,p_y,p_x]
			  Algorithm applies feedback control to linear velocity [x,y,z] and angular velocity [th_x, th_y, th_z]
			  Proportional–integral–derivative control algorithm employed to control drone motion				

'''

# Import libraries and packages for ROS
# Gazebo simulation environment is integrated with ROS
# Image tracking system ROS Whycon implementing Flood-Filling Image Processing Algorithm
# All actions synchronised with system time
# Design model is object-oriented


'''
A proportional–integral–derivative controller is a control loop feedback mechanism widely used in industrial control systems and applications requiring continuously modulated control. A PID controller continuously calculates an error value e ( t ) as the difference between a desired setpoint (SP) and a measured process variable (PV) and applies a correction based on proportional, integral, and derivative terms (denoted P, I, and D respectively) which give the controller its name. In practical terms it automatically applies accurate and responsive correction to a control function. 
'''

'''
	Import relevant libraries and packages

'''


# Import library for roscore
import roslib

# ROS integrated with Python, library rospy
import rospy
import geometry_msgs.msg
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseArray

# Operating System utilities
import time, sys, select, termios, tty

''' 
	Code to set drone into motion and control the motion

'''

# Set counter to 1
counter = 1

# Points in space to be traversed by drone, described in [x,y,z] format
waypoint=[ [4.25, 4.05, 11], [-4.0, 3.25, 11], [-3.8, -4.2, 11],[4.25, -4.2, 11], [0, 0, 11] ]

# Facotors to scale velocity changes in [x,y,x] direction
velocity_modified_values_by_pid=[]

# Drone PoseArray at different instants of time
current_position_tracker=[]
drone_position_state=[]
track_to_change=[]

# Define class drone to subscribe data from ROSTopic whycon/poses
# Subscriber node initialized
class drone:
	def GetData(self,data):
		self.data = data

	def __init__(self):
		rospy.init_node('drone_move')
		self.data = None
	       	rospy.Subscriber("/whycon/poses",geometry_msgs.msg.PoseArray,self.GetData)
		
	def PrintPosition(self):
		current_position_tracker=self.data.poses[0].position.z
		drone_position_state=[self.data.poses[0].position.x,self.data.poses[0].position.y,self.data.poses[0].position.z]
		return drone_position_state		

# Set drone into motion by changing velocity components in x,y,z direction
# Planar motion is considered, hence angular velocity component in each direction is 0
def motion(x1,y1,z1):

	twist = Twist()

	twist.linear.x = x1; twist.linear.y = y1; twist.linear.z = z1
	twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0

	return twist

# Publish velocity (x,y,x=0,0,0) to stop drone motion momentarily to stablize the drone
def stop():
		stopper=0	
		while(stopper<70000):

			twist = Twist()
			twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
			twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0

			pub_twist.publish(twist)
			stopper=stopper+1

''' 
		Coefficients for controller algorithm:
		K(p) = 0.39000
		K(i) = 0.00095
		K(d) = 0.09500
		Coefficients determined experimentally by evaluating possible configurations


'''
# Implement Proportional+Integrator+Differentiator Control Algorithm for velocity component x
def pidx(x,i,j,last_error_x,errorsum_x):
	time = 1
	sp_x=waypoint[i][j]

	pv_x = x
	kp_x = 0.39
	ki_x = 0.00095
	kd_x = 0.095

	vf_x = 0.0

	errordif_x = 0.0
	error_x = sp_x - pv_x
	errordif_x = (error_x - last_error_x) / time
	errorsum_x = errorsum_x + (error_x * time)		

	vf_x = kp_x * error_x + ki_x * errorsum_x + kd_x * errordif_x
	last_error_x = error_x 

	velocity_modified_values_by_pid = [vf_x,last_error_x,errorsum_x]
	return velocity_modified_values_by_pid

# Implement Proportional+Integrator+Differentiator Control Algorithm for velocity component y
def pidy(y,i,j,last_error_y,errorsum_y):
	time = 1
	sp_y=waypoint[i][j]

	pv_y = y
	kp_y = 0.39
	ki_y = 0.00095
	kd_y = 0.095

	vf_y = 0.0

	errordif_y = 0.0
	error_y = sp_y - pv_y
	errordif_y = (error_y - last_error_y)/time
	errorsum_y = errorsum_y + (error_y * time)		

	vf_y = kp_y * error_y + ki_y * errorsum_y + kd_y * errordif_y
	last_error_y = error_y 

	velocity_modified_values_by_pid = [vf_y,last_error_y,errorsum_y]
	return velocity_modified_values_by_pid


# Operations to TAKEOFF, MOVE, LAND drone
# ROSTopic /cmd_vel publishes velocity to the node
# ROSTopic /ardrone/takeoff publishes the command to takeoff
# ROSTopic /ardrone/land publishes the command to land

if __name__=="__main__":
	track=drone()
	pub_twist = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
	pub_empty_takeoff = rospy.Publisher('/ardrone/takeoff', Empty, queue_size=10)
	pub_empty_landing = rospy.Publisher('/ardrone/land', Empty,queue_size=10)

	# Sleep rate=1, the ROS node system inactivates
	rospy.sleep(1)
	r=rospy.Rate(10)
	
	# Initial construction to publish TAKEOFF command
	# Precision limit of 0.2 is maintained
	while 1:
		track_to_change=track.PrintPosition()
		if abs(waypoint[0][2]-track_to_change[2]) < 0.2:
			counter = 3
			break
		
		pub_empty_takeoff.publish(Empty())	
		r.sleep()
		
	# Initialize all errors to 0, errors will be udated for PID Controller Algorithm
	last_error_in_x = 0
	last_error_in_y = 0
	error_sum_of_x = 0
	error_sum_of_y = 0

	# Construction to set drone into motion
	while 1:
		current_position_tracker=track.PrintPosition()
		while counter == 3:

			twist=motion(0,0,0)
			pub_twist.publish(twist)

			current_position_tracker_new=track.PrintPosition()
			pub_twist.publish(twist)

			if not (abs(current_position_tracker[2] - current_position_tracker_new[2]) < 0.1):
				pub_twist.publish(twist)
				r.sleep()
			else:
				break
			current_position_tracker=current_position_tracker_new
		break_counter=0
		
		# Construction to set drone into waypoint traversal 
		for i in range (5):
			while 1:

				current_position_tracker=track.PrintPosition()
				
				# If location is reached, stablize the drone to momentary rest 
				if abs(waypoint[i][0] - current_position_tracker[0]) < 0.2 and abs(waypoint[i][1] - current_position_tracker[1]) < 0.2 and abs(waypoint[i][2] - current_position_tracker[2]) < 0.2:

					i = i + 1
					last_error_in_x = 0
					last_error_in_y = 0

					t=motion(0,0,0)
					pub_twist.publish(t)

					r.sleep()
									
				
				last_error_in_x = 0
				error_sum_of_x = 0
				last_error_in_y = 0
				error_sum_of_y = 0 
				last_error_in_z = 0
				error_sum_of_z = 0

				# Control drone motion by PID Controller Algorithm
				# PID acts simultaneously in [x,y] direction
				# Control precision of 0.2 maintained
				while not (abs(waypoint[i][0]-current_position_tracker[0])<0.2 and abs(waypoint[i][1]-current_position_tracker[1])<0.2):

					current_position_tracker=track.PrintPosition()
					get_factor_x = pidx(current_position_tracker[0],i,0,last_error_in_x,error_sum_of_x)
					get_factor_y = pidy(current_position_tracker[1],i,1,lery,error_sum_of_y)

					t=motion(-get_factor_y[0],-get_factor_x[0],0)
					pub_twist.publish(t)

					r.sleep()

					last_error_in_x = get_factor_x[1]
					error_sum_of_x = get_factor_x[2]
					last_erron_in_y = get_factor_y[1]
					error_sum_of_y = get_factor_y[2]

				# Reassign errors to 0 for next iteration
				last_error_in_x = 0
				error_sum_of_x = 0
				last_error_in_y = 0
				error_sum_of_y = 0 
				last_error_in_z = 0
				error_sum_of_z = 0

				i = i + 1
				break_counter = i

				# STOP condition
				if break_counter == 5:

					# Stablize the drone to rest before LAND
					stopper = 1
					while stopper < 50000:

						stopper = stopper + 1

						twist = motion(0,0,0)
						pub_twist.publish(twist)

					stopper = 1

					# Actvate the node to publish command for LAND
					while stopper < 50000:

						stopper = stopper + 1
						pub_empty_landing.publish(Empty())

					break

				# Exit motion-controller loop 
				if break_counter == 5:
					break
	# Reactivate the ROS Node
	rospy.spin()
