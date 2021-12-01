#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math 

def get_rotation(msg):
	orientation_q = msg.pose.pose.orientation
	arr_orient = [orientation_q.x, orientation_q.y,orientation_q.z,orientation_q.w]
	arr.data = arr_orient
	(roll.data, pitch.data, yaw.data) = euler_from_quaternion(arr_orient)

def get_rotation_states(msg):
	orientation_q = msg.pose[1].orientation
	arr_orient = [orientation_q.x, orientation_q.y,orientation_q.z,orientation_q.w]
	(roll2.data, pitch2.data, yaw2.data) = euler_from_quaternion(arr_orient)


def main():

	#always need to be initialized like that in the beginning 

	global arr 
	global roll, pitch, yaw, roll2, pitch2, yaw2
	yaw = Float64()
	pitch = Float64()
	roll = Float64()
	pitch2 = yaw2 = roll2= Float64()
	arr = Float64MultiArray() 
	

	rospy.init_node('my_quaternion_to_euler')

	sub = rospy.Subscriber ('/odom', Odometry, get_rotation)
	sub_m = rospy.Subscriber ('/gazebo/model_states', ModelStates, get_rotation_states)
	pub_q = rospy.Publisher('/quad', Float64MultiArray, queue_size=10)
	pub_e = rospy.Publisher('/euler_yaw', Float64, queue_size=10)

	rate = rospy.Rate(1) # 10hz


	while not rospy.is_shutdown():
		
		#yaw.data=yaw.data/math.pi*180
		
		pub_q.publish(arr)
		rospy.loginfo(arr)
		pub_e.publish(yaw)
		rospy.loginfo(yaw)
		print("yaw1 = ", yaw, "yaw2 = ", yaw2)

		rate.sleep()


if __name__ == '__main__':

    main()



