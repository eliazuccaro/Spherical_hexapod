#!/usr/bin/env python3

# --------- Trajectories generation node ---------

import rospy
from math import pi, sin, cos, sqrt, tan
import numpy as np
import time
import shellbot_kinematic_laws as kinematic_laws
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, String, Float64
from shellbot_pkg.msg import Pose

# Discrete time sampling interval for gait desired positions
DT = .05  # [s]

# ------------- gait values ------------
# Here is defined maximum distances for one single step:
# the topic '/cmd_vel' will indicate a percentage of this values, not a real velocity in m/s.
# The time of a single gait is fixed, the motion velocity is imposed by gait distances.
GAIT_STEP_TIME = 1  # [s] time interval for a single step
FW_MAX_DIST = 30.   # [mm] maximum distance of one single gait (forward)
LAT_MAX_DIST = 30.  # [mm] maximum distance of one single gait (lateral)
MAX_ROT = pi / 20   # [rad] maximum allowed rotation
Z_STEP_RAISE = 30.  # [mm] Z lift for non-resting foot
UP_MAX_POS = 50.    # [mm] Z lift for non-resting foot (upper legs)

# trajectories velocities
FW_MAX_VEL = FW_MAX_DIST / GAIT_STEP_TIME
LAT_MAX_VEL = LAT_MAX_DIST / GAIT_STEP_TIME
ROT_MAX_VEL = MAX_ROT / GAIT_STEP_TIME
UP_POS_MAX_VEL = UP_MAX_POS / GAIT_STEP_TIME

is_moving = False
gait_state = 0.             # rest: 0 | step right: 1 | step left: -1
upper_legs_state = 0.       # up: 0 | down: 1

# upper coordinates initialization
up_quote = 1.
up_angle = 0.

# pose angles initializtion
phi = 0.
theta = 0.
psi = 0.

# current velocities as percentual of the maximum --> sono i valori che compaiono nella schermata di teleop_key.py
forward_speed_perc = 0.   # [-1, .. , 1]
lateral_speed_perc = 0.   # [-1, .. , 1]
rotation_speed_perc = 0.  # [-1, .. , 1]
up_pos_speed_perc = 0.    # [-1, .. , 1]
up_rot_speed_perc = 0.    # [-1, .. , 1]

# des_feet-related matrices initialization
des_feet_position = np.array([[0. for j in range(4)] for i in range(6)])
des_feet_velocity = np.array([[0. for j in range(4)] for i in range(6)])

# message to be sent in /des_feet_position topic
des_feet_msg = Float64MultiArray()
des_feet_msg.layout.dim = [MultiArrayDimension()]
des_feet_msg.layout.dim[0].stride = 1
des_feet_msg.layout.dim[0].size = 24

# message to be sent in /des_feet_velocity topic
des_feet_vel_msg = Float64MultiArray()
des_feet_vel_msg.layout.dim = [MultiArrayDimension()]
des_feet_vel_msg.layout.dim[0].stride = 1
des_feet_vel_msg.layout.dim[0].size = 24

# message to be sent in /theta1 topic
theta1_msg = Float64()

# message to be sent in /clik_index topic
clik_index_msg = Float64()

# message to be sent in /upper_positions topic
des_upper_positions_msg = Float64MultiArray()
des_upper_positions_msg.layout.dim = [MultiArrayDimension()]
des_upper_positions_msg.layout.dim[0].stride = 1
des_upper_positions_msg.layout.dim[0].size = 2

# robot body parameters, taken from shellbot_kinematic_laws
starting_z = kinematic_laws.STARTING_Z
l_o0_z = kinematic_laws.l_o0_z
l_o0_x = kinematic_laws.l_o0_x
# l_01_z = 0 --> h1 = 0
l_01_x = kinematic_laws.l_01_x
l_12_x = kinematic_laws.l_12_x
l_12_z = kinematic_laws.l_12_z
l_23_x = kinematic_laws.l_23_x
l_23_z = kinematic_laws.l_23_z
l_34_x = kinematic_laws.l_34_x
# l_34_x = 0 --> h4 = 0
delta = kinematic_laws.delta
epsilon = kinematic_laws.epsilon
xsup = kinematic_laws.xsup
ysup = kinematic_laws.ysup
h1 = kinematic_laws.h1_sup
r2 = kinematic_laws.r2
ls1 = kinematic_laws.ls1
d1 = kinematic_laws.d1
theta_1_open = kinematic_laws.theta_1_open
theta_1_close = kinematic_laws.theta_1_close
theta_2_still = kinematic_laws.theta_2_still
theta_3_still = kinematic_laws.theta_3_still
theta_4_still = kinematic_laws.theta_4_still
radius = kinematic_laws.radius

# trajectory constants
height = -162.3      # robot center - ground distance (in robot frame)
dist = 270

current_position = np.array([])    # current positions vector initialization

# modes: hexapod, shell, to_hexapod, to_shell
mode = "hexapod"     # mode initialization

# boolean value for commands enabling
# enable_commands = True

# bottom legs desidered positions
def next_positions(leg_j, start_position, is_left_step, tau):

    if is_left_step:
        resting_legs = [1, 3, 5]
        moving_legs = [2, 4, 6]
    else:
        resting_legs = [2, 4, 6]
        moving_legs = [1, 3, 5]

    # these feet will be kept to the ground "pushing backward"
    if leg_j in resting_legs:

        posx = dist * cos(MAX_ROT*rotation_speed_perc + 2*leg_j*pi/6 - pi/3) - l_23_z*sin(MAX_ROT*rotation_speed_perc + 2*leg_j*pi/6 - pi/3) - FW_MAX_DIST * forward_speed_perc
        posy = dist * sin(MAX_ROT*rotation_speed_perc + 2*leg_j*pi/6 - pi/3) + l_23_z*cos(MAX_ROT*rotation_speed_perc + 2*leg_j*pi/6 - pi/3) - LAT_MAX_DIST * lateral_speed_perc
        posz = height + posx * tan(theta) + posy * tan(psi)

    # these feet will advance
    elif leg_j in moving_legs:

        posx = dist * cos(-MAX_ROT*rotation_speed_perc + 2*leg_j*pi/6 - pi/3) - l_23_z*sin(-MAX_ROT*rotation_speed_perc + 2*leg_j*pi/6 - pi/3) + FW_MAX_DIST * forward_speed_perc
        posy = dist * sin(-MAX_ROT*rotation_speed_perc + 2*leg_j*pi/6 - pi/3) + l_23_z*cos(-MAX_ROT*rotation_speed_perc + 2*leg_j*pi/6 - pi/3) + LAT_MAX_DIST * lateral_speed_perc
        posz = height + Z_STEP_RAISE * sin((tau)*pi) + posx * tan(theta) + posy * tan(psi)

    else:

        raise Exception('left_step: leg_j must be in [1-6]')

    final_position = np.array([posx, posy, posz])                   # filled by this function
    start_position = np.array(actual_feet[3*leg_j-3 : 3*leg_j])     # filled by actual_feet callback

    # convex combination of initial and final state (position), plus a lift on Z.
    current_position = start_position[0:2] * (1-tau) + final_position[0:2] * tau
    current_position = np.append(current_position, posz)

    return current_position

# bottom legs desidered velocities
def next_velocities(leg_j, is_left_step, tau):

    if is_left_step:
        resting_legs = [1, 3, 5]
        moving_legs = [2, 4, 6]
    else:
        resting_legs = [2, 4, 6]
        moving_legs = [1, 3, 5]

    if leg_j in resting_legs:        # these feet will be kept to the ground "pushing backward"

        posx_dot = dist*MAX_ROT*rotation_speed_perc*cos(pi/6 + leg_j*pi/3 + MAX_ROT*rotation_speed_perc*tau) - FW_MAX_DIST * forward_speed_perc * tau * (1-tau)
        posy_dot = dist*MAX_ROT*rotation_speed_perc*sin(pi/6 + leg_j*pi/3 + MAX_ROT*rotation_speed_perc*tau) - LAT_MAX_DIST * lateral_speed_perc * tau * (1-tau)
        posz_dot = 0

    elif leg_j in moving_legs:       # these feet will advance

        posx_dot = -dist*MAX_ROT*rotation_speed_perc*cos(pi/6 + leg_j*pi/3 - MAX_ROT*rotation_speed_perc*tau) + FW_MAX_DIST * forward_speed_perc * tau * (1-tau)
        posy_dot = -dist*MAX_ROT*rotation_speed_perc*sin(pi/6 + leg_j*pi/3 - MAX_ROT*rotation_speed_perc*tau) + LAT_MAX_DIST * lateral_speed_perc * tau * (1-tau)
        posz_dot = Z_STEP_RAISE*pi*cos(pi*tau)

    else:

        raise Exception('left_step: leg_j must be in [1-6]')

    # new final and start velocities
    final_velocity = np.array([posx_dot, posy_dot, posz_dot])         # filled by this function

    return final_velocity

# upper legs trajectories
def upper_trajectories(up_quote, up_angle):

    z_desidered = z_down + (z_up - z_down) * up_quote
    theta_desidered = theta_close + theta_open * up_angle

    z_next = z_desidered + radius/2 * tan(theta_desidered - pi/2)
    theta_next = theta_desidered

    return z_next, theta_next


# bottom legs trajectories for making robot close
def bottom_closure_positions(leg_j, is_first_step, tau, q1, q1_old, q1_radial):

    if is_first_step:
        resting_legs = [0, 2, 4]
        moving_legs = [1, 3, 5]
    else:
        resting_legs = [1, 3, 5]
        moving_legs = [0, 2, 4]

    l_01_x = 100.        # value temporary changed for better equations behaviour

    # these feet will be kept to the ground "pushing backward"
    if leg_j in resting_legs and is_first_step:

        posx = (dist - l_01_x * (1-cos(q1_old))) * cos(2*leg_j*pi/6 + q1 * q1_radial) - l_23_z*sin(2*leg_j*pi/6 + q1 * q1_radial)
        posy = (dist - l_01_x * (1-cos(q1_old))) * sin(2*leg_j*pi/6 + q1 * q1_radial) + l_23_z*cos(2*leg_j*pi/6 + q1 * q1_radial)
        posz = height

    # these feet will advance
    elif leg_j in moving_legs and is_first_step:

        posx = (dist - l_01_x * (1-cos(q1))) * cos(2*leg_j*pi/6 + q1 * q1_radial) - l_23_z*sin(2*leg_j*pi/6 + q1 * q1_radial)
        posy = (dist - l_01_x * (1-cos(q1))) * sin(2*leg_j*pi/6 + q1 * q1_radial) + l_23_z*cos(2*leg_j*pi/6 + q1 * q1_radial)
        posz = height + Z_STEP_RAISE * sin((tau)*pi)

    # these feet will be kept to the ground "pushing backward"
    elif leg_j in resting_legs and not is_first_step:

        posx = (dist - l_01_x * (1-cos(q1))) * cos(2*leg_j*pi/6 + q1 * q1_radial) - l_23_z*sin(2*leg_j*pi/6 + q1 * q1_radial)
        posy = (dist - l_01_x * (1-cos(q1))) * sin(2*leg_j*pi/6 + q1 * q1_radial) + l_23_z*cos(2*leg_j*pi/6 + q1 * q1_radial)
        posz = height

    # these feet will advance
    elif leg_j in moving_legs and not is_first_step:

        posx = (dist - l_01_x * (1-cos(q1))) * cos(2*leg_j*pi/6 + q1 * q1_radial) - l_23_z*sin(2*leg_j*pi/6 + q1 * q1_radial)
        posy = (dist - l_01_x * (1-cos(q1))) * sin(2*leg_j*pi/6 + q1 * q1_radial) + l_23_z*cos(2*leg_j*pi/6 + q1 * q1_radial)
        posz = height + Z_STEP_RAISE * sin((tau)*pi)

    else:

        raise Exception('left_step: leg_j must be in [1-6]')

    final_position = np.array([posx, posy, posz])                       # filled by this function
    start_position = np.array(actual_feet[3*leg_j : 3*leg_j + 3])       # filled by actual_feet callback

    if leg_j in moving_legs:

        # convex combination of initial and final state (position), plus a lift on Z.
        current_position = start_position[0:2] * (1-tau) + final_position[0:2] * tau
        current_position = np.append(current_position, posz)

    else:

        current_position = np.array([posx, posy, posz])

    l_01_x = 120.

    return current_position


# bottom legs trajectories for making robot close
def bottom_closure_centers(leg_j, is_first_step, tau):

    if is_first_step:
        resting_legs = [0, 2, 4]
        moving_legs = [1, 3, 5]
    else:
        resting_legs = [1, 3, 5]
        moving_legs = [0, 2, 4]

    # these feet will be kept to the ground "pushing backward"
    if leg_j in resting_legs and is_first_step:

        final_position = np.array([17*cos(leg_j*pi/3)+50.6*sin(leg_j*pi/3), -50.6*cos(leg_j*pi/3)+17*sin(leg_j*pi/3), -1.34+leg_j*pi/3, 6.02])

    # these feet will advance
    elif leg_j in moving_legs and is_first_step:

        final_position = np.array([(17*cos(leg_j*pi/3)+50.6*sin(leg_j*pi/3))*(1-tau) + radius*(1-cos(sin(tau*pi)))*cos(-72*pi/180 + leg_j*pi/3), (-50.6*cos(leg_j*pi/3)+17*sin(leg_j*pi/3))*(1-tau) + radius*(1-cos(sin(tau*pi)))*sin(-72*pi/180 + leg_j*pi/3), -72*pi/180 + leg_j*pi/3, 2*pi+0.5*sin(tau*pi)])

    # these feet will be kept to the ground "pushing backward"
    elif leg_j in resting_legs and not is_first_step:

        final_position = np.array([0, 0, -72*pi/180 + leg_j*pi/3, 2*pi])

    # these feet will advance
    elif leg_j in moving_legs and not is_first_step:

        final_position = np.array([(17*cos(leg_j*pi/3)+50.6*sin(leg_j*pi/3))*(1-tau) + radius*(1-cos(sin(tau*pi)))*cos(-72*pi/180 + leg_j*pi/3), (-50.6*cos(leg_j*pi/3)+17*sin(leg_j*pi/3))*(1-tau) + radius*(1-cos(sin(tau*pi)))*sin(-72*pi/180 + leg_j*pi/3), -72*pi/180 + leg_j*pi/3, 2*pi+0.5*sin(tau*pi)])

    else:

        raise Exception('left_step: leg_j must be in [1-6]')

    return final_position


# bottom legs velocities for making robot close
def bottom_closure_centers_velocities(leg_j, is_first_step, tau):

    if is_first_step:
        resting_legs = [0, 2, 4]
        moving_legs = [1, 3, 5]
    else:
        resting_legs = [1, 3, 5]
        moving_legs = [0, 2, 4]

    if leg_j in resting_legs and is_first_step:

        final_velocity = np.array([0, 0, 0, 0])

    elif leg_j in moving_legs and is_first_step:

        final_velocity = np.array([-17*cos(leg_j*pi/3)-50.6*sin(leg_j*pi/3) + pi*radius*cos(pi*tau)*sin(sin(pi*tau))*cos(-72*pi/180 + leg_j*pi/3), 50.6*cos(leg_j*pi/3)-17*sin(leg_j*pi/3) + pi*radius*cos(pi*tau)*sin(sin(pi*tau))*sin(-72*pi/180 + leg_j*pi/3), 0, 0.5*pi*cos(tau*pi)])

    elif leg_j in resting_legs and not is_first_step:

        final_velocity = np.array([0, 0, 0, 0])

    elif leg_j in moving_legs and not is_first_step:

        final_velocity = np.array([-17*cos(leg_j*pi/3)-50.6*sin(leg_j*pi/3) + pi*radius*cos(pi*tau)*sin(sin(pi*tau))*cos(-72*pi/180 + leg_j*pi/3), 50.6*cos(leg_j*pi/3)-17*sin(leg_j*pi/3) + pi*radius*cos(pi*tau)*sin(sin(pi*tau))*sin(-72*pi/180 + leg_j*pi/3), 0, 0.5*pi*cos(tau*pi)])

    else:

        raise Exception('left_step: leg_j must be in [1-6]')

    return final_velocity


# bottom legs trajectories for making robot open ### here first_step is reverted ###
def bottom_opening_centers(leg_j, is_first_step, tau):

    x_final = -26.6
    y_final = -29.9
    third_angle = -2.29

    if is_first_step:
        resting_legs = [6, 2, 4]
        moving_legs = [1, 3, 5]
    else:
        resting_legs = [1, 3, 5]
        moving_legs = [6, 2, 4]

    # these legs will be kept to the ground
    if leg_j in resting_legs and is_first_step:

        final_position = np.array([x_final*cos(leg_j*pi/3)-y_final*sin(leg_j*pi/3), y_final*cos(leg_j*pi/3)+x_final*sin(leg_j*pi/3), third_angle+leg_j*pi/3, 2*pi-0.38])

    # these legs will advance 
    elif leg_j in moving_legs and is_first_step:

        final_position = np.array([(x_final*cos(leg_j*pi/3)-y_final*sin(leg_j*pi/3))*(1-tau) + radius*(1-cos(sin(tau*pi)))*cos(third_angle + leg_j*pi/3), (y_final*cos(leg_j*pi/3)+x_final*sin(leg_j*pi/3))*(1-tau) + radius*(1-cos(sin(tau*pi)))*sin(third_angle + leg_j*pi/3), third_angle + leg_j*pi/3, 2*pi+0.5*sin(tau*pi)-0.38*(1-tau)])

    # these feet will be kept to the ground "pushing backward"
    elif leg_j in resting_legs and not is_first_step:

        final_position = np.array([0, 0, third_angle + leg_j*pi/3, 2*pi])

    # these feet will advance
    elif leg_j in moving_legs and not is_first_step:

        final_position = np.array([(x_final*cos(leg_j*pi/3)-y_final*sin(leg_j*pi/3))*(1-tau) + radius*(1-cos(sin(tau*pi)))*cos(third_angle + leg_j*pi/3), (y_final*cos(leg_j*pi/3)+x_final*sin(leg_j*pi/3))*(1-tau) + radius*(1-cos(sin(tau*pi)))*sin(third_angle + leg_j*pi/3), third_angle + leg_j*pi/3, 2*pi+0.5*sin(tau*pi)-0.38*(1-tau)])

    else:

        raise Exception('left_step: leg_j must be in [1-6]')

    return final_position


# bottom legs trajectories for making robot open
def bottom_opening_centers_velocities(leg_j, is_first_step, tau):

    x_final = -26.6
    y_final = -29.9
    third_angle = -2.29

    if is_first_step:
        resting_legs = [6, 2, 4]
        moving_legs = [1, 3, 5]
    else:
        resting_legs = [1, 3, 5]
        moving_legs = [6, 2, 4]

    # these feet will be kept to the ground "pushing backward"
    if leg_j in resting_legs and is_first_step:

        final_velocity = np.array([0, 0, 0, 0])

    # these feet will advance
    elif leg_j in moving_legs and is_first_step:

        final_velocity = np.array([-x_final*cos(leg_j*pi/3)+y_final*sin(leg_j*pi/3) + pi*radius*cos(pi*tau)*sin(sin(pi*tau))*cos(third_angle + leg_j*pi/3), -y_final*cos(leg_j*pi/3)-x_final*sin(leg_j*pi/3) + pi*radius*cos(pi*tau)*sin(sin(pi*tau))*sin(third_angle + leg_j*pi/3), 0, 0.25*pi*cos(tau*pi)+0.38])

    # these feet will be kept to the ground "pushing backward"
    elif leg_j in resting_legs and not is_first_step:

        final_velocity = np.array([0, 0, 0, 0])

    # these feet will advance
    elif leg_j in moving_legs and not is_first_step:

        final_velocity = np.array([-x_final*cos(leg_j*pi/3)+y_final*sin(leg_j*pi/3) + pi*radius*cos(pi*tau)*sin(sin(pi*tau))*cos(third_angle + leg_j*pi/3), -y_final*cos(leg_j*pi/3)-x_final*sin(leg_j*pi/3) + pi*radius*cos(pi*tau)*sin(sin(pi*tau))*sin(third_angle + leg_j*pi/3), 0, 0.25*pi*cos(tau*pi)+0.38])

    else:

        raise Exception('left_step: leg_j must be in [1-6]')

    return -final_velocity

# bottom legs trajectories for making robot open
def bottom_opening_tiptoes(leg_j, is_first_step, tau, q1, q1_radial):

    x_start = 41
    y_start = -120

    if is_first_step:
        resting_legs = [0, 2, 4]
        moving_legs = [1, 3, 5]
    else:
        resting_legs = [1, 3, 5]
        moving_legs = [0, 2, 4]

    l_01_x = 100.        # value temporary changed for better equations behaviour

    # these feet will be kept to the ground "pushing backward"
    if leg_j in resting_legs and is_first_step:

        posx = x_start*cos(leg_j*pi/3) - y_start*sin(leg_j*pi/3)
        posy = x_start*sin(leg_j*pi/3) + y_start*cos(leg_j*pi/3)
        posz = height*tau - 182*(1-tau)

    # these feet will advance
    elif leg_j in moving_legs and is_first_step:

        posx = (dist - l_01_x * (1-cos(q1))) * cos(2*leg_j*pi/6 + q1 * q1_radial) - l_23_z*sin(2*leg_j*pi/6 + q1 * q1_radial)
        posy = (dist - l_01_x * (1-cos(q1))) * sin(2*leg_j*pi/6 + q1 * q1_radial) + l_23_z*cos(2*leg_j*pi/6 + q1 * q1_radial)
        posz = height + Z_STEP_RAISE * sin((tau)*pi)

    # these feet will be kept to the ground "pushing backward"
    elif leg_j in resting_legs and not is_first_step:

        posx = (dist - l_01_x * (1-cos(q1))) * cos(2*leg_j*pi/6 + q1 * q1_radial) - l_23_z*sin(2*leg_j*pi/6 + q1 * q1_radial)
        posy = (dist - l_01_x * (1-cos(q1))) * sin(2*leg_j*pi/6 + q1 * q1_radial) + l_23_z*cos(2*leg_j*pi/6 + q1 * q1_radial)
        posz = height

    # these feet will advance
    elif leg_j in moving_legs and not is_first_step:

        posx = (dist - l_01_x * (1-cos(q1))) * cos(2*leg_j*pi/6 + q1 * q1_radial) - l_23_z*sin(2*leg_j*pi/6 + q1 * q1_radial)
        posy = (dist - l_01_x * (1-cos(q1))) * sin(2*leg_j*pi/6 + q1 * q1_radial) + l_23_z*cos(2*leg_j*pi/6 + q1 * q1_radial)
        posz = height + Z_STEP_RAISE * sin((tau)*pi)

    else:

        raise Exception('left_step: leg_j must be in [1-6]')

    final_position = np.array([posx, posy, posz])                       # filled by this function
    start_position = np.array(actual_feet[3*leg_j : 3*leg_j + 3])       # filled by actual_feet callback

    if leg_j in moving_legs:

        # convex combination of initial and final state
        current_position = start_position[0:2] * (1-tau) + final_position[0:2] * tau
        current_position = np.append(current_position, posz)

    else:

        current_position = np.array([posx, posy, posz])

    l_01_x = 120.

    return current_position


# bottom legs velocities for making robot open
def bottom_opening_tiptoes_velocities(leg_j, is_first_step, tau):

    x_start = 19.5
    y_start = -53.5
    x_final = 35.2
    y_final = -120

    if is_first_step:
        resting_legs = [0, 2, 4]
        moving_legs = [1, 3, 5]
    else:
        resting_legs = [1, 3, 5]
        moving_legs = [0, 2, 4]

    # these feet will be kept to the ground "pushing backward"
    if leg_j in resting_legs and is_first_step:

        final_velocity = np.array([0, 0, 0])

    # these feet will advance
    elif leg_j in moving_legs and is_first_step:

        final_velocity = np.array([x_final*cos(leg_j*pi/3)-y_final*sin(leg_j*pi/3), y_final*cos(leg_j*pi/3)+x_final*sin(leg_j*pi/3), Z_STEP_RAISE*cos(pi*tau)]) - np.array([x_start*cos(leg_j*pi/3)-y_start*sin(leg_j*pi/3), y_start*cos(leg_j*pi/3)+x_start*sin(leg_j*pi/3), 0])

    # these feet will be kept to the ground "pushing backward"
    elif leg_j in resting_legs and not is_first_step:

        final_velocity = np.array([0, 0, 0])

    # these feet will advance
    elif leg_j in moving_legs and not is_first_step:

        final_velocity = np.array([x_final*cos(leg_j*pi/3)-y_final*sin(leg_j*pi/3), y_final*cos(leg_j*pi/3)+x_final*sin(leg_j*pi/3), Z_STEP_RAISE*cos(pi*tau)]) - np.array([x_start*cos(leg_j*pi/3)-y_start*sin(leg_j*pi/3), y_start*cos(leg_j*pi/3)+x_start*sin(leg_j*pi/3), 0])

    else:

        raise Exception('left_step: leg_j must be in [1-6]')

    return final_velocity

# ---------------------------------------------- callbacks ----------------------------------------------
# callback for /cmd_vel topic listener
def on_move_update(msg):

    global forward_speed_perc, lateral_speed_perc, rotation_speed_perc, is_moving

    # if enable_commands == True:
    forward_speed_perc = msg.linear.x
    lateral_speed_perc = msg.linear.y
    rotation_speed_perc = msg.angular.z
    # if enable_commands == False:
    #     forward_speed_perc = 0.
    #     lateral_speed_perc = 0.
    #     rotation_speed_perc = 0.
    
    if (forward_speed_perc != 0) or (lateral_speed_perc != 0) or (rotation_speed_perc != 0):
        forward_speed_perc = forward_speed_perc * abs(forward_speed_perc) / (abs(forward_speed_perc) + abs(lateral_speed_perc) + abs(rotation_speed_perc))
        lateral_speed_perc = lateral_speed_perc * abs(lateral_speed_perc) / (abs(forward_speed_perc) + abs(lateral_speed_perc) + abs(rotation_speed_perc))
        rotation_speed_perc = rotation_speed_perc * abs(rotation_speed_perc) / (abs(forward_speed_perc) + abs(lateral_speed_perc) + abs(rotation_speed_perc))

    # flag for distinguishing moving or still robot
    is_moving = (forward_speed_perc != 0) or (lateral_speed_perc != 0) or (rotation_speed_perc != 0) or (tau != 0)


# callback for /mode topic listener
def mode_update(mode_msg):

    global mode
    mode_received = mode_msg.data

    if mode == "hexapod" and mode_received == "shell":
        print("3...2...1...")
        time.sleep(3)    # time given in order the robot to stop his movements, if any
        mode = "to_shell"
    elif mode == "shell" and mode_received == "hexapod":
        print("3...2...1...")
        time.sleep(3)    # time given in order the robot to stop his movements, if any
        mode = "to_hexapod"


# callback for /des_pose topic listener
def pose_update(pose_msg):

    global phi, theta, psi
    phi = pose_msg.phi
    theta = pose_msg.theta
    psi = pose_msg.psi


# callback for /actual_feet topic listener
def actual_feet_update(actual_feet_msg):

    global actual_feet
    actual_feet = actual_feet_msg.data


# callback for /up_coord topic listener
def up_coord_update(up_coord_msg):

    global up_quote, up_angle
    up_coordinates = up_coord_msg.data
    up_quote = up_coordinates[0]
    up_angle = up_coordinates[1]


# -------------------------------------------------------------------------------------------------------


if __name__ == "__main__":

    rospy.init_node('trajectory_handler', anonymous = False)
    
    rospy.Subscriber('cmd_vel', Twist, on_move_update)
    rospy.Subscriber('mode', String, mode_update)
    rospy.Subscriber('actual_feet', Float64MultiArray, actual_feet_update)
    rospy.Subscriber('des_pose', Pose, pose_update)
    rospy.Subscriber('up_coord', Float64MultiArray, up_coord_update)

    des_feet_pub = rospy.Publisher('des_feet_position', Float64MultiArray, queue_size = 1)
    des_feet_vel_pub = rospy.Publisher('des_feet_velocity', Float64MultiArray, queue_size = 1)
    des_upper_pos_pub = rospy.Publisher('des_upper_positions', Float64MultiArray, queue_size = 1)
    theta1_pub = rospy.Publisher('theta1', Float64, queue_size = 1)
    clik_index_pub = rospy.Publisher('clik_index', Float64, queue_size = 1)

    # initializations
    start_position = np.zeros(6 * 3).reshape(6, 3)      # lower legs joint positions
    start_velocity = np.zeros(6 * 3).reshape(6, 3)      # lower legs joint velocities
    tau = 0.                                            # normalized time interval [0, .. , 1]
    q1 = 0                                              # central gear motor position
    closure_step = 0                                    # starting step of closing procedure
    opening_step = 1                                    # starting step of opening procedure
    is_first_step = True
    m = 1
    q1_old = 0.
    q1_radial = 0.72
    q1_radial_2 = 1.7
    current_up_quote = 0
    current_up_angle = 0
    clik_index = 0                                      # value for choosing correct clik inversion to be performed: 0 --> bottom tips | 1 --> bottom centers

    # motion range upper legs
    z_up = h1 - r2*cos(epsilon) - xsup*cos(delta+epsilon) - ysup*sin(delta+epsilon) + sqrt(ls1**2 - ((d1+2*ysup*cos(delta+epsilon)-2*r2*sin(epsilon)-2*xsup*sin(delta+epsilon))**2)/4)
    z_down = h1 - r2*cos(epsilon) - xsup*cos(delta+epsilon) - ysup*sin(delta+epsilon) - sqrt(ls1**2 - ((d1+2*ysup*cos(delta+epsilon)-2*r2*sin(epsilon)-2*xsup*sin(delta+epsilon))**2)/4)
    theta_close = pi/2
    theta_open = pi/3

    actual_t = rospy.Time.now()     # current time
    rate = rospy.Rate(1 / DT)       # [Hz] publication frequency

    print("trajectory_handler is online at rate {}Hz.".format(1 / DT))

    try:

        while not rospy.is_shutdown():

            prev_t = actual_t                                                        # previous time refreshing
            actual_t = rospy.Time.now()                                              # current time refreshing
            real_dt = (actual_t - prev_t).secs + (actual_t - prev_t).nsecs * 1.0e-9  # [s] - time elapsed in current step
            
            # ------------------------------ hexapod mode ------------------------------
            if mode == "hexapod":

                # rest condition
                if not is_moving and tau == 0.:

                    for leg_j in range(1, 7):

                        # still robot trajectories
                        x_still = dist * cos(2*(leg_j-1)*pi/6) - l_23_z * sin(2*(leg_j-1)*pi/6)
                        y_still = l_23_z * cos(2*(leg_j-1)*pi/6) + dist * sin(2*(leg_j-1)*pi/6)
                        z_still = height + x_still * tan(theta) + y_still * tan(psi)

                        des_feet_position[(leg_j-1),0:3] = np.array([x_still, y_still, z_still])    # fills des_feet_position with rest conditions
                        des_feet_velocity[(leg_j-1),0:3] = np.array([0, 0, 0])                      # fills des_feet_velocity with rest conditions

                    gait_state = 0

                # general moving in hexapod mode
                elif is_moving:

                    tau += real_dt / GAIT_STEP_TIME     # current time in normalized time interval

                    # first step
                    if gait_state == 0:

                        gait_state = 1          # right step
                        tau = 0.                # re-initialization normalized step time
                        n = 0                   # step number

                    # further steps
                    if tau >= 1.:   # time exceedes normalized interval --> switch from left to right step and vice-versa
                        
                        gait_state *= -1        # invert resting foot
                        tau = 0.                # re-initialization normalized step time
                        n = 0
                    
                    # BOTTOM LEGS # -- compute next time-step position of feet
                    for leg_j in range(1, 7):

                        des_feet_position[leg_j-1,0:3] = next_positions(leg_j, start_position[leg_j-1], gait_state == 1, tau)
                        des_feet_velocity[leg_j-1,0:3] = next_velocities(leg_j, gait_state == 1, tau)

                des_feet_msg.data = kinematic_laws.flatten(des_feet_position)
                des_feet_vel_msg.data = kinematic_laws.flatten(des_feet_velocity)
                theta1_msg.data = 0

                # upper legs positions trajectories
                des_up_position, des_up_angle = upper_trajectories(up_quote, up_angle)


            # ------------------------------ closing procedure ------------------------------
            elif mode == "to_shell":

                # step 0 - make upper legs close and go up
                if closure_step == 0:
                    
                    tau += real_dt / GAIT_STEP_TIME     # current time in normalized time interval

                    up_angle = up_angle * (1 - tau)
                    up_quote = up_quote * (1 - tau) + tau

                    if  up_angle <= 0.1 and up_quote > 0.95:
                        up_angle = 0
                        up_quote = 1
                        tau = 0
                        closure_step = 1

                    des_up_position, des_up_angle = upper_trajectories(up_quote, up_angle)
                    des_upper_positions_msg.data = [des_up_position, des_up_angle]
                    des_upper_pos_pub.publish(des_upper_positions_msg)

                # step 1 - make folding fan close
                elif closure_step == 1:

                    if m == 5:
                        closure_step = 3
                    
                    q1 -= 0.05  # q1 closing step
                    q1_radial = 0.63 + 0.025 * m
                                        
                    for leg_j in range(1, 7):

                            # still robot trajectories
                            x_still = (dist - l_01_x * (1-cos(q1_old))) * cos(2*(leg_j-1)*pi/6 + q1 * q1_radial) - l_23_z*sin(2*(leg_j-1)*pi/6 + q1 * q1_radial)
                            y_still = (dist - l_01_x * (1-cos(q1_old))) * sin(2*(leg_j-1)*pi/6 + q1 * q1_radial) + l_23_z*cos(2*(leg_j-1)*pi/6 + q1 * q1_radial)
                            z_still = height

                            des_feet_position[(leg_j-1),0:3] = np.array([x_still, y_still, z_still])    # fills des_feet_position with rest conditions
                            des_feet_velocity[(leg_j-1),0:3] = np.array([0, 0, 0])                      # fills des_feet_velocity with rest conditions
                    
                    # q1 should be 118 degrees for full closure, here he gets closer to that value
                    if q1 <= -1.95 * m/4:
                        closure_step = 2

                    des_feet_msg.data = kinematic_laws.flatten(des_feet_position)
                    des_feet_vel_msg.data = kinematic_laws.flatten(des_feet_velocity)
                    theta1_msg.data = q1
                    des_feet_pub.publish(des_feet_msg)
                    des_feet_vel_pub.publish(des_feet_vel_msg)
                    theta1_pub.publish(theta1_msg)

                # step 2 - make bottom legs move and center
                elif closure_step == 2:

                    tau += real_dt / GAIT_STEP_TIME
                    q1_radial = 0.63 + 0.025 * m

                    for leg_j in range(1, 7):
                        des_feet_position[(leg_j-1),0:3] = bottom_closure_positions((leg_j-1), is_first_step, tau, q1, q1_old, q1_radial)
                        des_feet_velocity[(leg_j-1),0:3] = np.array([0, 0, 0])

                    if tau >= 1 and is_first_step:
                        is_first_step = False
                        tau = 0

                    elif tau >= 1 and not is_first_step:
                        is_first_step = True
                        tau = 0
                        closure_step = 1
                        m += 1
                        q1_old = q1

                    des_feet_msg.data = kinematic_laws.flatten(des_feet_position)
                    des_feet_vel_msg.data = kinematic_laws.flatten(des_feet_velocity)
                    theta1_msg.data = q1
                    clik_index_msg.data = clik_index
                    des_feet_pub.publish(des_feet_msg)
                    des_feet_vel_pub.publish(des_feet_vel_msg)
                    theta1_pub.publish(theta1_msg)

                # step 3 - make upper legs close and go down
                elif closure_step == 3:

                    tau += real_dt / GAIT_STEP_TIME
                    up_quote = (1 - tau)

                    if tau > 1:
                        tau = 0
                        up_quote = 0
                        closure_step = 4

                    des_up_position, des_up_angle = upper_trajectories(up_quote, 0)
                    des_upper_positions_msg.data = [des_up_position, des_up_angle]
                    des_upper_pos_pub.publish(des_upper_positions_msg)
                
                # full robot closing
                elif closure_step == 4:
                    
                    tau += real_dt / (GAIT_STEP_TIME * 2)

                    for leg_j in range(1, 7):
                        des_feet_position[(leg_j-1),:] = bottom_closure_centers(leg_j-1, is_first_step, tau)
                        des_feet_velocity[(leg_j-1),:] = bottom_closure_centers_velocities(leg_j-1, is_first_step, tau)

                    if tau >= 1 and is_first_step:
                        is_first_step = False
                        tau = 0

                    elif tau >= 1 and not is_first_step:
                        tau = 0
                        closure_step = 5
                    
                    clik_index = 1

                    des_feet_msg.data = kinematic_laws.flatten(des_feet_position)
                    des_feet_vel_msg.data = kinematic_laws.flatten(des_feet_velocity)
                    theta1_msg.data = q1
                    
                    des_feet_pub.publish(des_feet_msg)
                    des_feet_vel_pub.publish(des_feet_vel_msg)
                    clik_index_msg.data = clik_index
                    clik_index_pub.publish(clik_index_msg)
                    theta1_pub.publish(theta1_msg)

                # final step, go to shell mode
                elif closure_step == 5:

                    mode = "shell"
                    is_first_step = False
                    print("shell mode")
                    
            # ------------------------------ opening procedure ------------------------------
            elif mode == "to_hexapod":

                # step 0 - make upper legs open - TODO: se ne può fare a meno #############################################
                # if opening_step == 0:

                #     tau += real_dt / GAIT_STEP_TIME
                #     up_angle = tau

                #     if tau > 1:
                #         tau = 0
                #         up_angle = 1
                #         opening_step = 1

                #     for leg_j in range(1, 7):
                #         des_feet_position[(leg_j-1),:] = np.array([radius*(1-cos(0.2))*cos(-72*pi/180 + (leg_j-1)*pi/3), radius*(1-cos(0.2))*sin(-72*pi/180 + (leg_j-1)*pi/3), -72*pi/180 + (leg_j-1)*pi/3, 2*pi+0.1])
                #         des_feet_velocity[(leg_j-1),:] = np.array([0, 0, 0, 0])

                #     des_feet_msg.data = kinematic_laws.flatten(des_feet_position)
                #     des_feet_vel_msg.data = kinematic_laws.flatten(des_feet_velocity)
                #     des_feet_pub.publish(des_feet_msg)
                #     des_feet_vel_pub.publish(des_feet_vel_msg)

                #     des_up_position, des_up_angle = upper_trajectories(0, up_angle)
                #     des_upper_positions_msg.data = [des_up_position, des_up_angle]
                #     des_upper_pos_pub.publish(des_upper_positions_msg)
                ################################################################################################

                # step 1 - make bottom legs open and make the robot lift
                if opening_step == 1:

                    tau += real_dt / (GAIT_STEP_TIME * 2)

                    for leg_j in range(1, 7):
                        des_feet_position[(leg_j-1),:] = bottom_opening_centers(leg_j, is_first_step, 1 - tau)
                        des_feet_velocity[(leg_j-1),:] = bottom_opening_centers_velocities(leg_j, is_first_step, 1 - tau)

                    if tau >= 1 and not is_first_step:
                        is_first_step = True
                        tau = 0

                    elif tau >= 1 and is_first_step:
                        tau = 0
                        is_first_step = True
                        opening_step = 2

                    des_feet_msg.data = kinematic_laws.flatten(des_feet_position)
                    des_feet_vel_msg.data = kinematic_laws.flatten(des_feet_velocity)
                    theta1_msg.data = q1
                    
                    des_feet_pub.publish(des_feet_msg)
                    des_feet_vel_pub.publish(des_feet_vel_msg)
                    clik_index_msg.data = clik_index
                    clik_index_pub.publish(clik_index_msg)
                    theta1_pub.publish(theta1_msg)

                # step 2 - make the robot reach a suitable position to start opening at small steps (reverse of closing)
                elif opening_step == 2:

                    m = 4 
                    tau += real_dt / GAIT_STEP_TIME
                    q1_radial = 0.63 + 0.025 * m
                    q1 = -1.95

                    for leg_j in range(1, 7):
                        des_feet_position[(leg_j-1),0:3] = bottom_opening_tiptoes(leg_j-1, is_first_step, tau, q1, q1_radial)
                        des_feet_velocity[(leg_j-1),0:3] = np.array([0, 0, 0])

                    if tau >= 1 and is_first_step:
                        is_first_step = False
                        tau = 0

                    elif tau >= 1 and not is_first_step:
                        tau = 0
                        is_first_step = False
                        opening_step = 3

                    clik_index = 0
                    clik_index_msg.data = clik_index
                    clik_index_pub.publish(clik_index_msg)

                    des_feet_msg.data = kinematic_laws.flatten(des_feet_position)
                    des_feet_vel_msg.data = kinematic_laws.flatten(des_feet_velocity)
                    theta1_msg.data = q1
                    
                    des_feet_pub.publish(des_feet_msg)
                    des_feet_vel_pub.publish(des_feet_vel_msg)
                    theta1_pub.publish(theta1_msg)

                # step 3 - make upper legs lift to hexapod standard position
                elif opening_step == 3:

                    tau += real_dt / GAIT_STEP_TIME
                    up_quote = tau

                    if tau > 1:
                        tau = 0
                        up_quote = 1
                        m = 4
                        opening_step = 4

                    des_up_position, des_up_angle = upper_trajectories(up_quote, 0)
                    des_upper_positions_msg.data = [des_up_position, des_up_angle]
                    des_upper_pos_pub.publish(des_upper_positions_msg)

                # step 4 - make bottom legs move and center
                elif opening_step == 4:

                    tau += real_dt / GAIT_STEP_TIME
                    q1_radial = 0.63 + 0.025 * m
                    q1_new = -1.95 * (m-1)/4

                    for leg_j in range(1, 7):
                        des_feet_position[(leg_j-1),0:3] = bottom_closure_positions(leg_j-1, is_first_step, tau, q1, q1_new, q1_radial)
                        des_feet_velocity[(leg_j-1),0:3] = np.array([0, 0, 0])

                    if tau >= 1 and is_first_step:
                        is_first_step = False
                        tau = 0

                    elif tau >= 1 and not is_first_step:
                        is_first_step = True
                        tau = 0
                        opening_step = 5
                        m -= 1
                        # q1_old = q1

                    des_feet_msg.data = kinematic_laws.flatten(des_feet_position)
                    des_feet_vel_msg.data = kinematic_laws.flatten(des_feet_velocity)
                    theta1_msg.data = q1
                    clik_index_msg.data = clik_index
                    des_feet_pub.publish(des_feet_msg)
                    des_feet_vel_pub.publish(des_feet_vel_msg)
                    theta1_pub.publish(theta1_msg)
                
                # step 5 - make folding fan open
                elif opening_step == 5:

                    if m == 0:
                        q1 = 0
                        opening_step = 6

                    else:
                        q1 += 0.05  # q1 opening step
                        q1_radial = 0.63 + 0.025 * m
                                            
                        for leg_j in range(1, 7):

                                # still robot trajectories
                                x_still = (dist - l_01_x * (1-cos(q1))) * cos(2*(leg_j-1)*pi/6 + q1 * q1_radial) - l_23_z*sin(2*(leg_j-1)*pi/6 + q1 * q1_radial)
                                y_still = (dist - l_01_x * (1-cos(q1))) * sin(2*(leg_j-1)*pi/6 + q1 * q1_radial) + l_23_z*cos(2*(leg_j-1)*pi/6 + q1 * q1_radial)
                                z_still = height

                                des_feet_position[(leg_j-1),0:3] = np.array([x_still, y_still, z_still])    # fills des_feet_position with rest conditions
                                des_feet_velocity[(leg_j-1),0:3] = np.array([0, 0, 0])                      # fills des_feet_velocity with rest conditions
                        
                        # q1 should be 118 degrees for full closure, here he gets closer to that value
                        if q1 >= q1_new:
                            opening_step = 4

                        des_feet_msg.data = kinematic_laws.flatten(des_feet_position)
                        des_feet_vel_msg.data = kinematic_laws.flatten(des_feet_velocity)
                        theta1_msg.data = q1
                        des_feet_pub.publish(des_feet_msg)
                        des_feet_vel_pub.publish(des_feet_vel_msg)
                        theta1_pub.publish(theta1_msg)

                # final step, go to hexapod mode
                elif opening_step == 6:

                    mode = "hexapod"


            # ------------------------------ shell mode ------------------------------
            elif mode == "shell":

                # shell still positions
                for leg_j in range(1, 7):
                    des_feet_position[(leg_j-1),:] = [0, 0, -72*pi/180 + (leg_j-1)*pi/3, 2*pi]
                    des_feet_velocity[(leg_j-1),:] = np.array([0, 0, 0, 0])


            # data publishing - TODO questi potrebbero essere tolti
            des_feet_pub.publish(des_feet_msg)
            des_feet_vel_pub.publish(des_feet_vel_msg)
            theta1_pub.publish(theta1_msg)
            clik_index_msg.data = clik_index
            des_upper_positions_msg.data = [des_up_position, des_up_angle]
            des_upper_pos_pub.publish(des_upper_positions_msg)
            clik_index_pub.publish(clik_index_msg)

            # print("[Evaluation time: %s ms]\n" % (time.time() - start_time)*1000)
            # it seems to run at ~2ns per loop
            rate.sleep()

    except rospy.ROSInterruptException:
        pass