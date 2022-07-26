#!/usr/bin/env python3
#
# Forward and inverse kinematics. Reference systems transformations.

import rospy
from threading import Lock
from math import pi, cos, sin, atan2, asin, acos, sqrt, pow
import numpy as np
from numpy.linalg import inv, norm
from iDynTree import vectorize, forwardKinematicsDH


# robot parameters
STARTING_Z = 0.             # [mm]  initial body quote
l_o0_z = -29.               # [mm]  h0
l_o0_x = 66.                # [mm]  d0/2
l_01_x = 120.               # [mm]  l1
# l_01_z = 0                  [mm]  h1
l_12_z = -24.25             # [mm]  h2
l_12_x = 38.                # [mm]  l2
l_23_x = 66.                # [mm]  l3
l_23_z = 24.25              # [mm]  h3
l_34_x = 122.               # [mm]  l4
xinf = 36.75                # [mm]
yinf = -16.3 -19            # [mm]
r1 = 162.                   # [mm]
alpha = 85 * pi / 180
beta = 35 * pi / 180
delta = 90 * pi / 180
epsilon = 75 * pi / 180
r2 = 161.
xsup = 10.75
ysup = -19.
ls1 = 85.
d1 = 153.5
h1_sup = 63.5

# noticeable joint angles
theta_1_open = 0.
theta_1_close = 0.
theta_2_still = 0 * pi / 180
theta_3_still = 0 * pi / 180
theta_4_still = 0 * pi / 180
theta_u_1_close = 0 * pi / 180
theta_u_2_close = 0 * pi / 180

# offset motors variables WARNING: must be coherent with URDF
theta_1_offset = 0.
theta_2_offset = 0.
theta_3_offset = -0.2 * pi / 180
theta_4_offset = 95 * pi / 180
theta_1u_offset = 0.7326
theta_2u_offset = 1.1

# bottom legs direct kinematics
def f(leg_num, q1, q2, q3, q4):

    x_feedback = l_23_z*cos((1+2*leg_num)*pi/6+q1+q2) + l_o0_x*sin((1+2*leg_num)*pi/6) + l_01_x*sin((1+2*leg_num)*pi/6+q1) + (l_12_x + l_23_x*cos(q3) + l_34_x*cos(q3 + q4))*sin((1+2*leg_num)*pi/6+q1+q2)
    y_feedback = -l_o0_x*cos((1+2*leg_num)*pi/6) - l_01_x*cos((1+2*leg_num)*pi/6+q1) - cos((1+2*leg_num)*pi/6+q1+q2)*(l_12_x+l_23_x*cos(q3)+l_34_x*cos(q3+q4)) + l_23_z*sin((1+2*leg_num)*pi/6+q1+q2)
    z_feedback = l_o0_z + l_12_z - l_23_x*sin(q3) - l_34_x*sin(q3+q4)
    psi_feedback = -pi/3 + leg_num*pi/3 + q1 + q2
    theta_feedback = q3 + q4
    phi_feedback = 0

    pose_feedback = vectorize([x_feedback, y_feedback, z_feedback, psi_feedback, theta_feedback, phi_feedback])
    return pose_feedback

# upper legs direct kinematics
def f_upper(q1, q2):

    x_feedback = d1/2 + ls1 * cos(q1) + xsup * cos(q1 + q2) + r2 * cos(delta + q1 + q2) - ysup * sin(q1 + q2) 
    z_feedback = h1_sup + ysup * cos(q1 + q2) + ls1 * sin(q1) + xsup * sin(q1 + q2) + r2 * sin(delta + q1 + q2)
    angle_feedback = q1 + q2 + epsilon + delta

    up_feedback = vectorize([x_feedback, z_feedback, angle_feedback])

    return up_feedback

# robot center direct kinematics
def f_center(leg_num, q1, q2, q3, q4):

    x_feedback = l_23_z*cos((1+2*leg_num)*pi/6+q1+q2) + l_o0_x*sin((1+2*leg_num)*pi/6) + l_01_x*sin((1+2*leg_num)*pi/6+q1) + sin((1+2*leg_num)*pi/6+q1+q2)*(l_12_x+l_23_x*cos(q3)+xinf*cos(q3+q4)+r1*cos(alpha+q3+q4)-yinf*sin(q3+q4))
    y_feedback = -l_o0_x*cos((1+2*leg_num)*pi/6)-l_01_x*cos((1+2*leg_num)*pi/6+q1)+l_23_z*sin((1+2*leg_num)*pi/6+q1+q2)-cos((1+2*leg_num)*pi/6+q1+q2)*(l_12_x+l_23_x*cos(q3)+xinf*cos(q3+q4)+r1*cos(alpha+q3+q4)-yinf*sin(q3+q4))
    z_feedback = l_o0_z+l_12_z*-yinf*cos(q3+q4)-l_23_x*sin(q3)-xinf*sin(q3+q4)-r1*cos(alpha+q3+q4)
    phi_feedback = -pi/3+leg_num*pi/3+q1+q2
    theta_feedback = q3+q4
    psi_feedback = 0

    center_feedback = vectorize([x_feedback, y_feedback, z_feedback, phi_feedback, theta_feedback, psi_feedback])

    return center_feedback

# jacobian - bottom legs end effector (3x3)
def qbj_jacob(leg_num_j, joint_angles_j):

    # q1 = joint_angles_j[0]
    # q2 = joint_angles_j[1]
    # q3 = joint_angles_j[2]
    # q4 = joint_angles_j[3]
    q1 = 0.
    q2 = joint_angles_j[0]
    q3 = joint_angles_j[1]
    q4 = joint_angles_j[2]
    leg_num = leg_num_j

    # print("helloooooo")

    J = [[cos((1+2*leg_num)*pi/6+q1+q2)*(l_12_x+l_23_x*cos(q3)+l_34_x*cos(q3+q4))-l_23_z*sin((1+2*leg_num)*pi/6+q1+q2), -sin((1+2*leg_num)*pi/6+q1+q2)*(l_23_x*sin(q3)+l_34_x*sin(q3+q4)), -l_34_x*sin((1+2*leg_num)*pi/6+q1+q2)*sin(q3+q4)],
         [l_23_z*cos((1+2*leg_num)*pi/6+q1+q2)+(l_12_x+l_23_x*cos(q3)+l_34_x*cos(q3+q4))*sin((1+2*leg_num)*pi/6+q1+q2), cos((1+2*leg_num)*pi/6+q1+q2)*(l_23_x*sin(q3)+l_34_x*sin(q3+q4)), l_34_x*cos((1+2*leg_num)*pi/6+q1+q2)*sin(q3+q4)],
         [0, -l_23_x*cos(q3)-l_34_x*cos(q3+q4), -l_34_x*cos(q3+q4)]]

    J_a_inv = np.linalg.inv(J)

    return J_a_inv

# jacobian - robot center (6x4)
def centerj_jacob(leg_num_j, joint_angles_j):

    q1 = joint_angles_j[0]
    q2 = joint_angles_j[1]
    q3 = joint_angles_j[2]
    q4 = joint_angles_j[3]
    leg_num = leg_num_j

    j11 = l_01_x*cos((1+2*leg_num)*pi/6+q1)-l_23_z*sin((1+2*leg_num)*pi/6+q1+q2)+cos((1+2*leg_num)*pi/6+q1+q2)*(l_12_x+l_23_x*cos(q3)+xinf*cos(q3+q4)+r1*cos(alpha+q3+q4)-yinf*sin(q3+q4))
    j12 = -l_23_z*sin((1+2*leg_num)*pi/6+q1+q2)+cos((1+2*leg_num)*pi/6+q1+q2)*(l_12_x+l_23_x*cos(q3)+xinf*cos(q3+q4)+r1*cos(alpha+q3+q4)-yinf*sin(q3+q4))
    j13 = -sin((1+2*leg_num)*pi/6+q1+q2)*(yinf*cos(q3+q4)+l_23_x*sin(q3)+xinf*sin(q3+q4)+r1*sin(alpha+q3+q4))
    j14 = -sin((1+2*leg_num)*pi/6+q1+q2)*(yinf*cos(q3+q4)+xinf*sin(q3+q4)+r1*sin(alpha+q3+q4))
    
    j21 = l_23_z*cos((1+2*leg_num)*pi/6+q1+q2)+l_01_x*sin((1+2*leg_num)*pi/6+q1)+sin((1+2*leg_num)*pi/6+q1+q2)*(l_12_x+l_23_x*cos(q3)+xinf*cos(q3+q4)+r1*cos(alpha+q3+q4)-yinf*sin(q3+q4))
    j22 = l_23_z*cos((1+2*leg_num)*pi/6+q1+q2)+sin((1+2*leg_num)*pi/6+q1+q2)*(l_12_x+l_23_x*cos(q3)+xinf*cos(q3+q4)+r1*cos(alpha+q3+q4)-yinf*sin(q3+q4))
    j23 = cos((1+2*leg_num)*pi/6+q1+q2)*(yinf*cos(q3+q4)+l_23_x*sin(q3)+xinf*sin(q3+q4)+r1*sin(alpha+q3+q4))
    j24 = cos((1+2*leg_num)*pi/6+q1+q2)*(yinf*cos(q3+q4)+xinf*sin(q3+q4)+r1*sin(alpha+q3+q4))

    j31 = 0
    j32 = 0
    j33 = -l_23_x*cos(q3)-xinf*cos(q3+q4)-r1*cos(alpha+q3+q4)+yinf*sin(q3+q4)
    j34 = -xinf*cos(q3+q4)-r1*cos(alpha+q3+q4)+yinf*sin(q3+q4)

    j41 = 1
    j42 = 1
    j43 = 0
    j44 = 0

    j51 = 0
    j52 = 0
    j53 = 1
    j54 = 1

    j61 = 0
    j62 = 0
    j63 = 0
    j64 = 0

    J = [[j11, j12, j13, j14],
        [j21, j22, j23, j24],
        [j31, j32, j33, j34],
        [j41, j42, j43, j44],
        [j51, j52, j53, j54],
        [j61, j62, j63, j64]]

    J_pinv = np.linalg.pinv(J)

    return J_pinv

# upper legs close-form kinematic inversion
def up_inversion(quote, angle):

    sum = - angle - delta - epsilon + 2*pi
    theta_1 = asin((quote - ysup * cos(sum) - xsup * sin(sum) - r2 * sin(delta + sum) - h1_sup) / ls1)  # TODO: verificare non sia il complementare
    theta_2 = sum - theta_1

    return np.array([theta_1, theta_2])

# angles for closed upper legs  -- TODO: REMOVE
def quj_close(upper_leg_num_j):

    q1_u = theta_u_1_close
    q2_u = theta_u_2_close

    return [q1_u, q2_u]

# angles values for still robot -- TODO: REMOVE
def qbj_still(j):

    q1 = theta_1_open
    q2 = theta_2_still
    q3 = theta_3_still
    q4 = theta_4_still

    return q1, q2, q3, q4

def flatten(list_of_lists): return [el for sublist in list_of_lists for el in sublist]

if __name__ == "__main__":

    print("You can't run kinematic_laws.py directly! It's an internal utility library used by inverse_kinematic_server.py")
