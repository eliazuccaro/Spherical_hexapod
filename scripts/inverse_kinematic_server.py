#!/usr/bin/env python3

# --------- Kinematic inversion node ---------

import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, Float64
import shellbot_kinematic_laws as kinematic_laws
from shellbot_pkg.msg import Pose
from sensor_msgs.msg import JointState

# update frequency for joints desired angles
DT = .05  # [s]

# message to be sent to motors
servo_positions = Float64MultiArray()
servo_positions.layout.dim = [MultiArrayDimension()]
servo_positions.layout.dim[0].stride = 1
motors_number = rospy.get_param("/motors_number")        # motors number loading from rosparam (36: simulated robot - 31: real robot)
motors_number = 36
servo_positions.layout.dim[0].size = motors_number
# motors_number = 36 (useful if this node is used without roslaunch)

# ROS messages declarations
x_msg = Float64MultiArray()
x_msg.layout.dim = [MultiArrayDimension()]
x_msg.layout.dim[0].stride = 18
x_up_msg = Float64MultiArray()
x_up_msg.layout.dim = [MultiArrayDimension()]
x_up_msg.layout.dim[0].stride = 3

# arrays declarations
x = []                                                               # current feet position
x_centers = []                                                       # current bottom centers positions
x_up = []                                                            # current upper coordinates
x_d = np.array([[None for j in range(4)] for i in range(6)])         # desidered feet positions
x_d_dot = np.array([[None for j in range(4)] for i in range(6)])     # desidered feet velocities

# initializations
qk = []                                                              # joint angles at k-th step
qk_old = []                                                          # joint angles at (k-1)-th step
q_dot = []                                                           # joint velocities at k-th step
upper_joint_angles = []
qk_offset = np.array([kinematic_laws.theta_1_offset, kinematic_laws.theta_2_offset, kinematic_laws.theta_3_offset, kinematic_laws.theta_4_offset])
q_u_offset = np.array([kinematic_laws.theta_1u_offset, kinematic_laws.theta_2u_offset])
delta = kinematic_laws.delta
epsilon = kinematic_laws.epsilon
k = 3
k_ang = 3
k_vec = np.array([2*k, 2*k, 2*k_ang, 3*k_ang])                       # CLIK proportional constant
theta1 = 0.
clik_index = 0.                                                      # numerical index indicating current CLIK


# evaluate joint angles for given body and tiptoes position and publish to motors
def eval_inversion(motor_pub):

    if not x_d.any():
        return

    # try:
        
    # take joints positions from real robot
    if (motors_number == 31):

        # not used in simulation. Code available in the on-board version of this program
        pass    
    
    # ---------------------- CLIK - bottom legs tips ----------------------
    if clik_index == 0:

        for j in range(1, 7):

            J_a_inv_j = kinematic_laws.qbj_jacob(j, joint_states[(4*j-4) : (4*j)])                         # inverse of jacobian matrix (3x3)
            qk_old = np.array([theta1, joint_states[4*j-3], joint_states[4*j-2], joint_states[4*j-1]])     # current joints positions
            x_j = x[3*j-3 : 3*j]                                                                           # position vector j_th bottom foot

            ke = np.multiply((x_d[j-1, 0:3] - x_j), k)    # 1x3

            alpha = np.multiply(x_d_dot[j-1, 0:3], 1)
            
            q_dot.append(theta1)
            q_dot.append(J_a_inv_j.dot(alpha + ke)[0])
            q_dot.append(J_a_inv_j.dot(alpha + ke)[1])
            q_dot.append(J_a_inv_j.dot(alpha + ke)[2])

            qk[4*j-4 : 4*j] = qk_old + np.multiply(q_dot[4*j-4 : 4*j], DT) - qk_offset                     # joint angles j-th bottom leg

    # ---------------------- CLIK - bottom legs shell centers ----------------------
    elif clik_index == 1:
    
        for j in range(1, 7):

            J_a_inv_j = kinematic_laws.centerj_jacob(j, joint_states[(4*j-4) : (4*j)])                                  # inverse of jacobian matrix (4x4)
            qk_old = np.array([joint_states[4*j-4], joint_states[4*j-3], joint_states[4*j-2], joint_states[4*j-1]])     # current joints positions
            x_j = x_centers[4*j-4 : 4*j]                                                                                # position vector j_th bottom center

            ke = np.multiply((x_d[j-1, :] - x_j), k_vec)    # 1x4

            alpha = np.multiply(x_d_dot[j-1, :], 0.3)
            
            q_dot.append(J_a_inv_j.dot(alpha + ke)[0])
            q_dot.append(J_a_inv_j.dot(alpha + ke)[1])
            q_dot.append(J_a_inv_j.dot(alpha + ke)[2])
            q_dot.append(J_a_inv_j.dot(alpha + ke)[3])

            qk[4*j-4 : 4*j] = qk_old + np.multiply(q_dot[4*j-4 : 4*j], DT) - qk_offset                                   # joint angles j-th bottom leg

    # ---------------------- upper legs inverse kinematics ----------------------
    upper_joint_angles_i = kinematic_laws.up_inversion(up_position, up_angle) - q_u_offset

    for i in range(1, 7):

        upper_joint_angles.append(upper_joint_angles_i[0])
        upper_joint_angles.append(upper_joint_angles_i[1])

    # full joint positions vector
    list_of_desired_angles = qk + upper_joint_angles

    # -----------  real robot: joint number reduction from 36 to 31 and transmission ratio division -----------
    if (motors_number == 31):

        leg_gear_joint_angle = list_of_desired_angles[0]                            # joint angle of first leg gear
        transmission_rate = 1.0                                                     # inserted in the on-board version
        central_motor_joint_angle = leg_gear_joint_angle * transmission_rate

        # starting legs gears angles removing. Indexes: 0, 4, 8, 12, 16, 20
        del list_of_desired_angles[20]
        del list_of_desired_angles[16]
        del list_of_desired_angles[12]
        del list_of_desired_angles[8]
        del list_of_desired_angles[4]
        del list_of_desired_angles[0]

        # central joint angle inserting
        list_of_desired_angles.insert(0, central_motor_joint_angle)

    # empty positions and velocities vectors
    qk.clear()
    upper_joint_angles.clear()
    q_dot.clear()

    servo_positions.data = list_of_desired_angles
    motor_pub.publish(servo_positions)
        
    # except ValueError as error:

    #     print(" !! POSE NOT FEASABLE !! ERROR: ")
    #     print(error)
    #     pass

# ----------------------------------------- callbacks -----------------------------------------

# desidered feet positions update
def update_des_feet_position(des_feet_msg):

    global x_d          # desidered feet positions
    x_d = np.reshape(des_feet_msg.data, (6, 4))


# desidered feet velocities update
def update_des_feet_velocity(des_feet_vel_msg):

    global x_d_dot      # desidered feet velocities
    x_d_dot = np.reshape(des_feet_vel_msg.data, (6, 4))


# callback for /theta1 topic listener
def theta1_update(theta1_msg):

    global theta1
    theta1 = theta1_msg.data


# callback for /clik_index topic listener
def clik_index_update(clik_index_msg):

    global clik_index

    if clik_index != clik_index_msg.data:

        clik_index = clik_index_msg.data
        print("Clik index changed: ", clik_index)


# desidered upper centers positions
def update_des_upper_positions(des_upper_msg):

    global up_coordinates, up_position, up_angle
    up_coordinates = des_upper_msg.data
    up_position = up_coordinates[0]
    up_angle = up_coordinates[1]


# joint angles reading from Gazebo and direct kinematics
def get_simulated_joints(joint_msgs):

    global joint_states
    global joint_velocities

    global x            # feet positions vector
    global x_dot        # feet velocities vector

    joint_states_tuple = joint_msgs.position
    joint_states = list(joint_states_tuple)

    # bottom legs offset
    joint_states[2] += kinematic_laws.theta_3_offset
    joint_states[3] += kinematic_laws.theta_4_offset
    joint_states[6] += kinematic_laws.theta_3_offset
    joint_states[7] += kinematic_laws.theta_4_offset
    joint_states[10] += kinematic_laws.theta_3_offset
    joint_states[11] += kinematic_laws.theta_4_offset
    joint_states[14] += kinematic_laws.theta_3_offset
    joint_states[15] += kinematic_laws.theta_4_offset
    joint_states[18] += kinematic_laws.theta_3_offset
    joint_states[19] += kinematic_laws.theta_4_offset
    joint_states[22] += kinematic_laws.theta_3_offset
    joint_states[23] += kinematic_laws.theta_4_offset

    # upper legs offset
    joint_states[24] += kinematic_laws.theta_1u_offset
    joint_states[25] += kinematic_laws.theta_2u_offset
    joint_states[26] += kinematic_laws.theta_1u_offset
    joint_states[27] += kinematic_laws.theta_2u_offset
    joint_states[28] += kinematic_laws.theta_1u_offset
    joint_states[29] += kinematic_laws.theta_2u_offset
    joint_states[30] += kinematic_laws.theta_1u_offset
    joint_states[31] += kinematic_laws.theta_2u_offset
    joint_states[32] += kinematic_laws.theta_1u_offset
    joint_states[33] += kinematic_laws.theta_2u_offset
    joint_states[34] += kinematic_laws.theta_1u_offset
    joint_states[35] += kinematic_laws.theta_2u_offset

    joint_velocities_tuple = joint_msgs.velocity
    joint_velocities = list(joint_velocities_tuple)

    # current feet positions calculation
    for j in range(1, 7):

        # x_j --> pose (x, y, z, phi, theta, psi)
        x_j = kinematic_laws.f_bottom(j, joint_states[4*j-4], joint_states[4*j-3], joint_states[4*j-2], joint_states[4*j-1])[0:3,0]    # take position but not orientation
        x[3*j-3 : 3*j] = x_j    # fill feet position vector - only positions

    # current upper legs coordinates
    x_up = kinematic_laws.f_upper(joint_states[24], joint_states[25])

    # current bottom centers coordinates
    if clik_index == 1:

        for j in range(1, 7):

            x_center_j = kinematic_laws.f_center(j, joint_states[4*j-4], joint_states[4*j-3], joint_states[4*j-2], joint_states[4*j-1])
            x_centers[4*j-4 : 4*j-2] = x_center_j[0:2,0]        # fill center pose vector - 4/6 values
            x_centers[4*j-2 : 4*j] = x_center_j[3:5,0]

    x_msg.data = x
    x_up_msg.data = x_up

    actual_feet_pub.publish(x_msg)
    actual_upper_pub.publish(x_up_msg)

# ---------------------------------------------------------------------------------------------

# listen to des_pose and publish desired joint angles at rate 1/DT
if __name__ == "__main__":

    rospy.init_node('inverse_kinematic_server', anonymous = False)

    rospy.Subscriber('des_feet_position', Float64MultiArray, update_des_feet_position)
    rospy.Subscriber('des_feet_velocity', Float64MultiArray, update_des_feet_velocity)
    rospy.Subscriber('theta1', Float64, theta1_update)
    rospy.Subscriber('clik_index', Float64, clik_index_update)
    rospy.Subscriber('des_upper_positions', Float64MultiArray, update_des_upper_positions)
    rospy.Subscriber('joint_states', JointState, get_simulated_joints)
    
    motor_controller_pub = rospy.Publisher('/shellbot_legs_controller/command', Float64MultiArray, queue_size=1)
    actual_feet_pub = rospy.Publisher('actual_feet', Float64MultiArray, queue_size=1)
    actual_upper_pub = rospy.Publisher('actual_upper', Float64MultiArray, queue_size=1)
    inv_kin_pos_pub = rospy.Publisher('inv_kin_pos', Float64MultiArray, queue_size=1)
    qdot_pub = rospy.Publisher('qdot', Float64MultiArray, queue_size=1)

    print("inverse_kinematic_server is online.")
    rate = rospy.Rate(1 / DT)  # Hz

    try:
        
        while not rospy.is_shutdown():  # check for Ctrl-C

            eval_inversion(motor_controller_pub)    # main function

            rate.sleep()  # "rate" (instead of "time") can handle simulated time

    except rospy.ROSInterruptException:
        pass
