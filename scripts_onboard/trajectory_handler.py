#!/usr/bin/env python3
# from turtle import up
import rospy
from math import pi, sin, cos, sqrt, tan
import numpy as np
import time
# from iDynTree import vectorize, forwardKinematicsDH
import shellbot_kinematic_laws as kinematic_laws
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, String
from shellbot_pkg.msg import Pose

# Discrete time sampling interval for gait desired positions
DT = 0.5  # [s]

# ------------- gait values ------------
# Here is defined maximum distances for one single step:
# the topic '/cmd_vel' will indicate a percentage of this values, not a real velocity in m/s.
# The time of a single gait is fixed, the motion velocity is imposed by gait distances.
GAIT_STEP_TIME = 15 * DT  # [s] tempo dedicato al compimento di un passo
FW_MAX_DIST = 20.   # [mm]   maximum distance of one single gait (forward)
LAT_MAX_DIST = 20.  # [mm]  maximum distance of one single gait (lateral)
MAX_ROT = pi / 18   # [rad]  maximum allowed rotation
Z_STEP_RAISE = 80.  # [mm] Z lift for non-resting foot
UP_MAX_POS = 50.    # [mm] Z lift for non-resting foot

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
des_feet_position = np.array([[None for j in range(3)] for i in range(6)])
des_feet_velocity = np.array([[None for j in range(3)] for i in range(6)])

# message to be sent in /des_feet_position topic
des_feet_msg = Float64MultiArray()
des_feet_msg.layout.dim = [MultiArrayDimension()]
des_feet_msg.layout.dim[0].stride = 1
des_feet_msg.layout.dim[0].size = 18

# message to be sent in /des_feet_velocity topic
des_feet_vel_msg = Float64MultiArray()
des_feet_vel_msg.layout.dim = [MultiArrayDimension()]
des_feet_vel_msg.layout.dim[0].stride = 1
des_feet_vel_msg.layout.dim[0].size = 18

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

height = -170.      # robot center - ground distance (in robot frame)

# dist = sqrt((l_o0_x + l_12_x + l_23_x + sqrt(l_34_x**2 - (-height + l_o0_z + l_12_z - l_34_x)**2))**2 + l_23_z**2)
dist = 300.

current_position = np.array([])    # current positions vector initialization

mode = "hexapod"     # mode initialization

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
		posz = height + posx * tan(theta) + posy * tan(psi) - Z_STEP_RAISE/10

	# these feet will advance
	elif leg_j in moving_legs:

		posx = dist * cos(-MAX_ROT*rotation_speed_perc + 2*leg_j*pi/6 - pi/3) - l_23_z*sin(MAX_ROT*rotation_speed_perc + 2*leg_j*pi/6 - pi/3) + FW_MAX_DIST * forward_speed_perc
		posy = dist * sin(-MAX_ROT*rotation_speed_perc + 2*leg_j*pi/6 - pi/3) + l_23_z*cos(MAX_ROT*rotation_speed_perc + 2*leg_j*pi/6 - pi/3) + LAT_MAX_DIST * lateral_speed_perc
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

		posx_dot = dist*MAX_ROT*rotation_speed_perc*cos(pi/6 + leg_j*pi/3 + MAX_ROT*rotation_speed_perc*tau) - FW_MAX_DIST * forward_speed_perc
		posy_dot = dist*MAX_ROT*rotation_speed_perc*sin(pi/6 + leg_j*pi/3 + MAX_ROT*rotation_speed_perc*tau) - LAT_MAX_DIST * lateral_speed_perc
		posz_dot = 0

	elif leg_j in moving_legs:       # these feet will advance

		posx_dot = -dist*MAX_ROT*rotation_speed_perc*cos(pi/6 + leg_j*pi/3 - MAX_ROT*rotation_speed_perc*tau) + FW_MAX_DIST * forward_speed_perc
		posy_dot = -dist*MAX_ROT*rotation_speed_perc*sin(pi/6 + leg_j*pi/3 - MAX_ROT*rotation_speed_perc*tau) + LAT_MAX_DIST * lateral_speed_perc
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

	z_next = z_desidered + r2/2 * tan(theta_desidered - pi/2)
	theta_next = theta_desidered

	# TODO: utile per apertura e chiusura
	# z_uptodown = tau_up * z_down + (1 - tau_up) * z_up
	# z_downtoup = tau_up * z_up + (1 - tau_up) * z_down

	return z_next, theta_next


# ---------------------------------------------- callbacks ----------------------------------------------
# callback for /cmd_vel topic listener
def on_move_update(msg):

	global forward_speed_perc, lateral_speed_perc, rotation_speed_perc, is_moving

	forward_speed_perc = msg.linear.x
	lateral_speed_perc = msg.linear.y
	rotation_speed_perc = msg.angular.z

	if (forward_speed_perc != 0) or (lateral_speed_perc != 0) or (rotation_speed_perc != 0):
		forward_speed_perc = forward_speed_perc * abs(forward_speed_perc) / (abs(forward_speed_perc) + abs(lateral_speed_perc) + abs(rotation_speed_perc))
		lateral_speed_perc = lateral_speed_perc * abs(lateral_speed_perc) / (abs(forward_speed_perc) + abs(lateral_speed_perc) + abs(rotation_speed_perc))
		rotation_speed_perc = rotation_speed_perc * abs(rotation_speed_perc) / (abs(forward_speed_perc) + abs(lateral_speed_perc) + abs(rotation_speed_perc))

	# flag for distinguishing moving or still robot
	is_moving = (forward_speed_perc != 0) or (lateral_speed_perc != 0) or (rotation_speed_perc != 0) or (tau != 0)
	# rospy.loginfo('Received [%f %f %f]', msg.x, msg.y, msg.z)


# callback for /mode topic listener
def mode_update(mode_msg):

	global mode
	mode = mode_msg.data


# callback for/des_pose topic listener
def pose_update(pose_msg):

	global phi, theta, psi
	phi = pose_msg.phi
	theta = pose_msg.theta
	psi = pose_msg.psi
	# print("phi " + str(phi))
	# print("theta " + str(theta))
	# print("psi " + str(psi))
	# print("")


# callback for /actual_feet topic listener
def actual_feet_update(actual_feet_msg):

	global actual_feet
	actual_feet = actual_feet_msg.data


# callback for /actual_upper topic listener
def actual_upper_update(actual_upper_msg):

	global actual_upper
	actual_upper = actual_upper_msg.data


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
	# rospy.Subscriber('mode', Pose, mode_update)
	rospy.Subscriber('actual_feet', Float64MultiArray, actual_feet_update)
	rospy.Subscriber('des_pose', Pose, pose_update)
	rospy.Subscriber('up_coord', Float64MultiArray, up_coord_update)
	des_feet_pub = rospy.Publisher('des_feet_position', Float64MultiArray, queue_size = 1)
	des_feet_vel_pub = rospy.Publisher('des_feet_velocity', Float64MultiArray, queue_size = 1)
	des_upper_pos_pub = rospy.Publisher('des_upper_positions', Float64MultiArray, queue_size = 1)

	# initializations
	start_position = np.zeros(6 * 3).reshape(6, 3)      # lower legs joint positions
	start_velocity = np.zeros(6 * 3).reshape(6, 3)      # lower legs joint velocities
	tau = 0.                                            # normalized time interval [0, .. , 1]
	# tau_up = 0.                                         # normalized time interval for upper legs

	# motion range upper legs
	z_up = h1 - r2*cos(epsilon) - xsup*cos(delta+epsilon) - ysup*sin(delta+epsilon) + sqrt(ls1**2 - ((d1+2*ysup*cos(delta+epsilon)-2*r2*sin(epsilon)-2*xsup*sin(delta+epsilon))**2)/4)
	z_down = h1 - r2*cos(epsilon) - xsup*cos(delta+epsilon) - ysup*sin(delta+epsilon) - sqrt(ls1**2 - ((d1+2*ysup*cos(delta+epsilon)-2*r2*sin(epsilon)-2*xsup*sin(delta+epsilon))**2)/4)
	theta_close = pi/2
	theta_open = pi/4

	actual_t = rospy.Time.now()     # current time
	rate = rospy.Rate(1 / DT)       # [Hz] publication frequency

	print("trajectory_handler is online at rate {}Hz.".format(1 / DT))

	try:

		time.sleep(10)

		while not rospy.is_shutdown():

			prev_t = actual_t             # previous time refreshing
			actual_t = rospy.Time.now()   # current time refreshing
			real_dt = (actual_t - prev_t).secs + (actual_t - prev_t).nsecs * 1.0e-9  # [s] - time elapsed in current step
			
			# ------------------------------ bottom legs ------------------------------
			if mode == "hexapod":

				# rest condition
				if not is_moving and tau == 0.:

					for leg_j in range(1, 7):

						# still robot trajectories
						x_still = dist * cos(2*(leg_j-1)*pi/6) - l_23_z * sin(2*(leg_j-1)*pi/6)
						y_still = l_23_z * cos(2*(leg_j-1)*pi/6) + dist * sin(2*(leg_j-1)*pi/6)
						z_still = height + x_still * tan(theta) + y_still * tan(psi)

						des_feet_position[leg_j-1,:] = np.array([x_still, y_still, z_still])    # fills des_feet_position with rest conditions
						des_feet_velocity[leg_j-1,:] = np.array([0, 0, 0])                      # fills des_feet_velocity with rest conditions

					gait_state = 0

				# general moving in hexapod mode
				elif is_moving:

					# TODO: per modificare la velocità di camminata, permettere di modificare il GAIT_STEP_TIME
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
						n = 0 # TODO inserire gli effettivi valori di n
					
					# BOTTOM LEGS # -- compute next time-step position of feet
					for leg_j in range(1, 7):

						des_feet_position[leg_j-1,:] = next_positions(leg_j, start_position[leg_j-1], gait_state == 1, tau)
						des_feet_velocity[leg_j-1,:] = next_velocities(leg_j, gait_state == 1, tau)

					# des_feet_position[leg_j-1,:]

					# for leg_j in range(1, 7):

					#     des_feet_position[leg_j-1,:] = next_positions(leg_j, start_position[leg_j-1], gait_state == 1, tau)
					#     des_feet_velocity[leg_j-1,:] = next_velocities(leg_j, gait_state == 1, tau)

				# print("des_feet_position: ", des_feet_position)
				# print("")

				des_feet_msg.data = kinematic_laws.flatten(des_feet_position)
				des_feet_vel_msg.data = kinematic_laws.flatten(des_feet_velocity)
				des_feet_pub.publish(des_feet_msg)
				des_feet_vel_pub.publish(des_feet_vel_msg)

			# ------------------------------ upper legs ------------------------------
			if mode == "hexapod":

				# upper legs positions trajectories
				des_up_position, des_up_angle = upper_trajectories(up_quote, up_angle)

				des_upper_positions_msg.data = [des_up_position, des_up_angle]
				des_upper_pos_pub.publish(des_upper_positions_msg)

			elif mode == "shell":

				print("shell trajectories")
				pass

			# print("[Evaluation time: %s ms]\n" % (time.time() - start_time)*1000)
			# it seems to run at ~2ns per loop
			rate.sleep()

	except rospy.ROSInterruptException:
		pass