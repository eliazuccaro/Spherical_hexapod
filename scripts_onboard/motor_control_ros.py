#!/usr/bin/python3

from time import sleep, time
from serial import Serial
import RPi.GPIO as gpio	# GPIO control module
import rospy
import numpy as np
from math import pi
from std_msgs.msg import Float64MultiArray, MultiArrayDimension

from support.ax12 import Ax12
from support.servo import Servo

# ping
# setID
# move
# moveSpeed
# setTorqueStatus
# setLedStatus
# setAngleLimit
# setTorqueLimit
# readPosition
# learnServos

# Update frequency for joints desired angles (pay attention to
# gait_handler.DT value, something faster than that is just useless)
DT = 0.5  # [s]

joints_msg = Float64MultiArray()
joints_msg.layout.dim = [MultiArrayDimension()]
joints_msg.layout.dim[0].stride = 19

s = Servo(used_servo=19,minID=0,maxID=18,baudrate=1000000,setup=True,verbose=True)

command_rad = []

legs = 6
command_deg = [0] * 19
command_deg[0] = s.conv_to_servo(0, 0.)	# command servo 0 folding fan

for i in range(1,legs+1):

	command_deg[3*i-2] = s.conv_to_servo(3*i-2, 0.)
	command_deg[3*i-1] = s.conv_to_servo(3*i-1, -4.*pi/180)
	command_deg[3*i] =	 s.conv_to_servo(3*i, 89.*pi/180)

joint_angles = [0] * 19
joint_angles[0] = 0.

for i in range(1,legs+1):
	
	joint_angles[3*i-2] = 0.
	joint_angles[3*i-1] = -4.*pi/180
	joint_angles[3*i] =	  89.*pi/180


def command_listener(command_msg):

	global command_deg
	command_rad = command_msg.data
	
	command_deg[0] = s.conv_to_servo(0, -command_rad[0])
	for i in range(1, legs+1):

		command_deg[3*i-2] = s.conv_to_servo(3*i-2, -command_rad[3*i-2])
		command_deg[3*i-1] = s.conv_to_servo(3*i-1, command_rad[3*i-1])
		command_deg[3*i] =	 s.conv_to_servo(3*i, command_rad[3*i])


if __name__ == '__main__' :
	
	rospy.init_node('motor_control_ros', anonymous = False)
	rospy.Subscriber('joint_commands', Float64MultiArray, command_listener)
	joint_states_pub = rospy.Publisher('joint_states_real', Float64MultiArray, queue_size=1)

	rate = rospy.Rate(1 / DT)  # Hz

	try:
		prev_t = time()

		while not rospy.is_shutdown():
			t2 = time()

			if s.checkChangedAngles(command_deg):

				s.setAngleAll(command_deg)

				print("command_deg = ",end="")

				for i in range(0, len(command_deg)):
					print(round(command_deg[i],2),end="\t")
				print()
			
			
			s.updateAngleAll()
			elapsed2 = time() - t2
			print("elapsed2 = ", elapsed2)

			if time() - prev_t > 5:
				s.printAngles()
				s.printVoltage()
				prev_t = time()

			# get joints angles
			joint_angles[0] = s.conv_from_servo(0, s.curr_angles[0])
			for i in range(1, s.num):
				joint_angles[i] = s.conv_from_servo(i, s.curr_angles[i])

			joints_msg.data = joint_angles
			joint_states_pub.publish(joints_msg)


	except KeyboardInterrupt:

		print("Exiting Program")
	

	finally:
		s.offLED()
		s.ax.port.close()
		gpio.cleanup()
		pass