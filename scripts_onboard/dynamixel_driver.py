#!/usr/bin/env python3
#
# Listen to '/hex_legs_controller/command' topic, containing joint desired angles,
# and controls Dynamixel AX-12 motors through 'jetsonDynamixel' library.
# Periodically publish current joint values in '/joint_states'.

import os
import rospy
import numpy as np
from sys import path, exit
from math import pi, asin, sqrt, pow
from serial.serialutil import SerialException

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

dir_path = os.path.dirname(os.path.realpath(__file__))
path.append(dir_path + "/jetsonDynamixel/")
from ax12 import Ax12


N_MOTORS = 31  # expected number of motors (for safety checks on received messages)
MOTOR_VEL = 256  # commanded speed for any move [1-1024] -> therefore a quarter of max speed

# mechanical offsets and limits due to hardware assembly
# TODO --> anche questi vanno settati coi valori di shellbot
theta_offset_link_2 = asin(0.0145 / sqrt(pow(0.0645, 2) + pow(0.0145, 2)))  # [rads]
theta_offset_link_3 = asin(0.02 / sqrt(pow(0.0832, 2) + pow(0.02, 2)))  # [rads]

# dichiarazione array di offset
motor_offsets = []  # [rads]
motor_limits_lower = []  # [rads]
motor_limits_upper = []  # [rads]
# riempimenti array di offset
for i in range(6):
    motor_offsets.extend([0, theta_offset_link_2, theta_offset_link_3])
    motor_limits_lower.extend([-.81, -2.0, -2.2])  # readed as read_joints output,
    motor_limits_upper.extend([.81, 1.45, 0.45])    # thus considering offsets

########## TRASFORMAZIONI GEOMETRICHE ##########
def rad2deg(angle_rad): return angle_rad / pi * 180.0

def deg2rad(angle_deg): return angle_deg * pi / 180.0

def deg2bin(angle_deg):
    """ convert from [-150; +150]deg (AX-12 limits) to 0-1023 """
    angle_bin = int(angle_deg * 1024.0 / 300.0) + 512
    if angle_bin < 0:
        return 0
    if angle_bin > 1023:
        return 1023
    return angle_bin

# convert from 0-1023 to [-150; +150]deg
def bin2deg(angle_bin): return angle_bin * 300.0 / 1024.0 - 150.0

def rad2bin(angle_rad): return deg2bin(rad2deg(angle_rad))

def bin2rad(angle_bin): return deg2rad(bin2deg(angle_bin))
##################################################

# Callback for '/hex_legs_controller/command' topic listener
def desired_joint_command_callback(des_motor_rads_msg):

    global mServos, last_des_joint_rads

    if des_motor_rads_msg.layout.dim[0].size != N_MOTORS:
        print(f"MEGAWARNING!! {N_MOTORS} motors expected, but {des_motor_rads_msg.layout.dim[0].size} desired angles received!")
    else:
        # add motor offsets (due to hardware setup) --> TODO: vedere se questo serve anche a noi
        des_motor_rads = np.add(des_motor_rads_msg.data, motor_offsets)

        for i in range(N_MOTORS):
            motorId = i + 1
            try:
                # check limits
                des_motor_rad = des_motor_rads[i]
                if des_motor_rad > motor_limits_upper[i]:
                    print("WARNING: motor " + str(motorId) + " is tring to reach an angle above its limits!")
                    des_motor_rad = motor_limits_upper[i]
                elif des_motor_rad < motor_limits_lower[i]:
                    print("WARNING: motor " + str(motorId) + " is tring to reach an angle below its limits!")
                    des_motor_rad = motor_limits_lower[i]

                last_des_joint_rads[i] = des_motor_rad          # aggiornamento di queste variabili utili alla memorizzazione

                des_motor_bin = rad2bin(des_motor_rad)          # conversione degli angoli desiderati in dati intellegibili dai motori

                mServos.moveSpeed(motorId, des_motor_bin, MOTOR_VEL)        # effettivo comando di movimento dei motori

            except (Ax12.timeoutError, Ax12.axError, SerialException) as e:
                # print("W: Motor " + str(motorId) + " seems to be unreachable")
                pass

# Exception that occours when read_joints have some throuble reading joint values
class ReadJointsError(Exception):
    
    pass

# read all joint values and return them as an numpy array, corrected with the offsets
def read_joints():
    
    # TODO: verificare che l'ulteriore definizione di questa variabile sia effettivamente utile
    global mServos

    actual_positions = np.zeros(N_MOTORS)
    for i in range(N_MOTORS):
        motorId = i + 1
        try:
            readed_pos = bin2rad(mServos.readPosition(motorId))
        except (Ax12.timeoutError, Ax12.axError, SerialException) as e:
            raise ReadJointsError
        actual_positions[i] = readed_pos

    return np.subtract(actual_positions, motor_offsets)


if __name__ == "__main__":

    global mServos, last_des_joint_rads

    # check user privileges
    userId = os.getuid()
    if userId != 0:
        exit("You aren't root, I need superuser privileges to interface with GPIOs")

    # definizione dell'oggetto "motore"
    mServos = Ax12()
    last_des_joint_rads = np.zeros(N_MOTORS)

    rospy.init_node('dynamixel_driver_node', anonymous=False)
    rospy.Subscriber('/hex_legs_controller/command', Float64MultiArray, desired_joint_command_callback)
    state_pub = rospy.Publisher('/joint_states', JointState, queue_size=1)

    print("dynamixel_driver_node is online.")

    # prepare the joint state message
    joint_states = JointState()
    # TODO: inserire i giunti di shellbot
    joint_states.name = ['joint_1_1', 'joint_1_2', 'joint_1_3', 'joint_2_1', 'joint_2_2', 'joint_2_3',
                         'joint_3_1', 'joint_3_2', 'joint_3_3', 'joint_4_1', 'joint_4_2', 'joint_4_3',
                         'joint_5_1', 'joint_5_2', 'joint_5_3', 'joint_6_1', 'joint_6_2', 'joint_6_3']
    joint_states.velocity = np.zeros(N_MOTORS)
    joint_states.effort = np.zeros(N_MOTORS)


    actual_t = rospy.Time.now()

    rate = rospy.Rate(50)  # [Hz]
    
    try:
        while not rospy.is_shutdown():  # check for Ctrl-C
            # read joints and publish them in '/joint_states'
            joint_states.header.stamp = rospy.Time.now()
            try:
                joint_states.position = read_joints()
            except ReadJointsError as e:
                print("Some problems while reading the last joint state. Using desired value instead.")
                joint_states.position = last_des_joint_rads
            state_pub.publish(joint_states)

            rate.sleep()

    except rospy.ROSInterruptException:
        pass
