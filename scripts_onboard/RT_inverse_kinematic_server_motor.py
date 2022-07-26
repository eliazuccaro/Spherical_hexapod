#!/usr/bin/env python3

import rospy
from math import pi, sin, cos, sqrt, tan
import numpy as np
from math import pi
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, String
import shellbot_kinematic_laws as kinematic_laws
from shellbot_pkg.msg import Pose
from sensor_msgs.msg import JointState
import time

# Update frequency for joints desired angles (pay attention to
# gait_handler.DT value, something faster than that is just useless)
DT = 2  # [s]

# message to be sent to motors
servo_positions = Float64MultiArray()
servo_positions.layout.dim = [MultiArrayDimension()]
servo_positions.layout.dim[0].stride = 1
# motors_number = rospy.get_param("/motors_number")       # caricamento numero motori da rosparam (36: simulazione - 31: robot vero)
motors_number = 31
servo_positions.layout.dim[0].size = motors_number       # robot simulato --> 36 motori - robot reale --> 31 motori

x_msg = Float64MultiArray()
x_msg.layout.dim = [MultiArrayDimension()]
x_msg.layout.dim[0].stride = 18
# x = np.array([[None for j in range(3)] for i in range(6)])    # current feet position
x = np.empty(18)
x_up_msg = Float64MultiArray()
x_up_msg.layout.dim = [MultiArrayDimension()]
x_up_msg.layout.dim[0].stride = 3
x_up = []    # current upper coordinates
# x_d = np.array([[None for j in range(3)] for i in range(6)])         # desidered feet positions
x_d = np.empty(18)
# x_d_dot = np.array([[0. for j in range(3)] for i in range(6)])     # desidered feet velocities
x_d_dot = np.empty(18)

# initializations
qk = []             # joint angles at k-th step
qk_old = []         # joint angles at (k-1)-th step
q_dot = []          # joint velocities at k-th step
global joint_states
global joint_states_legs

legs = 6
joint_states = [0] * 36
joint_states[0] = 0.
for i in range(1,legs+1):
	joint_states[3*i-2] = 0.
	joint_states[3*i-1] = -4.*pi/180
	joint_states[3*i] = 89.*pi/180


joint_states_legs = joint_states[1:]

upper_joint_angles = [0] * 12


height = -170.
dist = 300.
l_23_z = kinematic_laws.l_23_z
for leg_j in range(1, 7):

	# still robot trajectories
	x[3*leg_j - 3] = dist * cos(2*(leg_j-1)*pi/6) - l_23_z * sin(2*(leg_j-1)*pi/6)
	x[3*leg_j - 2] = l_23_z * cos(2*(leg_j-1)*pi/6) + dist * sin(2*(leg_j-1)*pi/6)
	x[3*leg_j - 1] = height



# upper_joint_angles = []
# qk_offset = np.array([kinematic_laws.theta_1_offset, kinematic_laws.theta_2_offset, kinematic_laws.theta_3_offset, kinematic_laws.theta_4_offset])
# q_u_offset = np.array([kinematic_laws.theta_1u_offset, kinematic_laws.theta_2u_offset])
delta = kinematic_laws.delta
epsilon = kinematic_laws.epsilon
k = 0.1       # CLIK proportional constant
tau = 0


# evaluate joint angles for given body and tiptoes position and publish to motors
def eval_inversion(motor_controller_pub,tempo):

	# if not x_d.any():
	# 	return

	# try:

	# time.sleep(10)
	# print("-----------------------------------------------------------------------------------------------")
	# print("")
	
	# # take joints positions from real robot
	# if (motors_number == 31):

	# 	# TODO: vedere lettura angoli di giunto dalla libreria di AX12 e sovrascrivere la variabile joint_states
	# 	pass    

	# ---------------------- CLIK --> suitable for all bottom legs movements ----------------------
	for j in range(1, 7):

		# print("inizio cin. inversa")
		# print(joint_states[(4*j-4) : (4*j)])
		# print("")
		# J_a_inv_j = kinematic_laws.qbj_jacob(j, joint_states_legs[(4*j-4) : (4*j)])                    # inverse of jacobian matrix (3x3)
		J_a_inv_j = kinematic_laws.qbj_jacob(j, [joint_states_legs[3*j-3], joint_states_legs[3*j-2], joint_states_legs[3*j-1]])                    # inverse of jacobian matrix (3x3)
		# print("")
		# print("J_a_inv_j")
		# print(J_a_inv_j)
		qk_old = np.array([0, joint_states_legs[3*j-3], joint_states_legs[3*j-2], joint_states_legs[3*j-1]])     # current joints positions
		x_j = x[3*j-3 : 3*j]       # position vector j_th bottom foot

		# print("x_d[j-1, :] = ",x_d[3*j-3 : 3*j])
		# print("x_j = ",x_j)
		# ke = np.multiply((x_d[j-1, :] - x_j), k)    # 1x3
		ke = np.multiply((x_d[3*j-3 : 3*j] - x_j), k)    # 1x3

		alpha = np.multiply(x_d_dot[3*j-3 : 3*j], 1)
		
		q_dot.append(0.)
		q_dot.append(J_a_inv_j.dot(alpha + ke)[0])
		q_dot.append(J_a_inv_j.dot(alpha + ke)[1])
		q_dot.append(J_a_inv_j.dot(alpha + ke)[2])
		# q_dot.append(J_a_inv_j.dot(ke)[0])
		# q_dot.append(J_a_inv_j.dot(ke)[1])
		# q_dot.append(J_a_inv_j.dot(ke)[2])

		qk[4*j-4 : 4*j] = qk_old + np.multiply(q_dot[4*j-4 : 4*j], DT)          # joint angles j-th bottom leg

	# print("qk: ", qk)
	# print(len(qk))
	# print("")

	# ---------------------- upper legs inverse kinematics ----------------------
	# upper_joint_angles_i = kinematic_laws.up_inversion(up_position, up_angle)

	# for i in range(1, 7):

	#     upper_joint_angles.append(upper_joint_angles_i[0])
	#     upper_joint_angles.append(upper_joint_angles_i[1])

	# list_of_desired_angles ha dimensione 36
	# qk deve avere dimensione 24
	# upper_joint_angles ha dimensione 12
	list_of_desired_angles = qk + upper_joint_angles        # full joint positions vector

	# prev_t = actual_t             # previous time refreshing
	# actual_t = rospy.Time.now()   # current time refreshing
	# real_dt = (actual_t - prev_t).secs + (actual_t - prev_t).nsecs * 1.0e-9  # [s] - time elapsed in current step

	# tau += real_dt     # current time in normalized time interval

	# list_of_desired_angles = []
	# for angle in range (0, 36):
	# 	list_of_desired_angles.append(0)

	# print("list_of_desired_angles: ", list_of_desired_angles)
	# print(len(list_of_desired_angles))
	# print("")

	# if tau >= 1:
	# 	tau = 0


	# -----------  robot reale: riduzione giunti da 36 a 31 e scalatura con rapporto di trasmissione -----------
	if (motors_number == 31):

		# angolo di giunto dell'ingranaggio della prima gamba
		leg_gear_joint_angle = list_of_desired_angles[0]
		trasnsmission_rate = 1/1.75
		central_motor_joint_angle = leg_gear_joint_angle * trasnsmission_rate

		# rimozione degli angoli di giunto degli ingranaggi di inzio gamba 0, 4, 8, 12, 16, 20
		del list_of_desired_angles[20]
		del list_of_desired_angles[16]
		del list_of_desired_angles[12]
		del list_of_desired_angles[8]
		del list_of_desired_angles[4]
		del list_of_desired_angles[0]

		# inserimento angolo di giunto ingranaggio centrale
		list_of_desired_angles.insert(0, central_motor_joint_angle)

	# list_of_desired_angles[1] = 0. + pi/18 * sin(tempo*pi)
	# list_of_desired_angles[2] = -4.*pi/180 + pi/18 * sin(tempo*pi)
	# list_of_desired_angles[3] = 89.*pi/180 + pi/18 * sin(tempo*pi)
	# list_of_desired_angles[1] = 0.
	# list_of_desired_angles[2] = -4.*pi/180
	# list_of_desired_angles[3] = 89.*pi/180

	# tempo += DT

	# ok, sono corretti
	# print("list_of_desired_angles: ", list_of_desired_angles)
	# print(len(list_of_desired_angles))
	# print("")

	# empty positions and velocities vectors
	qk.clear()
	# upper_joint_angles.clear()
	q_dot.clear()

	servo_positions.data = list_of_desired_angles
	motor_controller_pub.publish(servo_positions)
		
	# except ValueError as error:

	# 	print(" !! POSE NOT FEASABLE !! ERROR: ")
	# 	print(error)
	# 	pass
	return tempo

# ----------------------------------------- callbacks -----------------------------------------

# desidered pose update
# def update_des_pose(des_pose_msg):

#     global des_pose
#     des_pose = [des_pose_msg.phi, des_pose_msg.theta, des_pose_msg.psi,
#                 des_pose_msg.x, des_pose_msg.y, des_pose_msg.z]

# desidered feet positions update
def update_des_feet_position(des_feet_msg):

	global x_d          # desidered feet positions
	# x_d = np.reshape(des_feet_msg.data, (6, 3))
	x_d = des_feet_msg.data

# desidered feet velocities update
def update_des_feet_velocity(des_feet_vel_msg):

	global x_d_dot      # desidered feet velocities
	# x_d_dot = np.reshape(des_feet_vel_msg.data, (6, 3))
	x_d_dot = des_feet_vel_msg.data

# desidered upper centers positions
def update_des_upper_positions(des_upper_msg):

	global up_coordinates, up_position, up_angle
	up_coordinates = des_upper_msg.data
	up_position = up_coordinates[0]
	up_angle = up_coordinates[1]

# joint angles reading from real motors and direct kinematics
def get_real_joints(joint_msgs):

	global joint_states
	global joint_states_legs
	global joint_velocities

	global x            # feet positions vector
	global x_dot        # feet velocities vector

	joint_states = joint_msgs.data
	# print("joints angles received")
	# print(joint_states)
	# print("")

	joint_states_legs = joint_states[1:]
	# print("joint_states_legs: ", joint_states_legs)
	# print("")
	
    # current feet positions calculation (f is the direct kinematics function)
	for j in range(1, 7):

		# x_j --> pose (x, y, z, phi, theta, psi)
		x_j = kinematic_laws.f(j, 0, joint_states_legs[3*j-3], joint_states_legs[3*j-2], joint_states_legs[3*j-1])[0:3,0]    # take position but not orientation
		x[3*j-3 : 3*j] = x_j    # fill feet position vector
		# print("gamba numero: ",j)
		# print("posizioni EE: ", x_j)
	
	# print("actual_feet")
	# print(x)
	# print("")

	x_msg.data = x
	# x_up_msg.data = x_up

	actual_feet_pub.publish(x_msg)
	# actual_upper_pub.publish(x_up_msg)

# ---------------------------------------------------------------------------------------------

# listen to des_pose and publish desired joint angles at rate 1/DT
if __name__ == "__main__":

	des_pose = [0, 0, 0, 0, 0, kinematic_laws.STARTING_Z]   # desidered pose initialization
	rospy.init_node('RT_inverse_kinematic_server_motor', anonymous = False)
	# rospy.Subscriber('des_pose', Pose, update_des_pose)
	rospy.Subscriber('des_feet_position', Float64MultiArray, update_des_feet_position)
	rospy.Subscriber('des_feet_velocity', Float64MultiArray, update_des_feet_velocity)
	rospy.Subscriber('des_upper_positions', Float64MultiArray, update_des_upper_positions)
	# rospy.Subscriber('mode', String, mode_update) # non so se sia necessario
	# rospy.Subscriber('joint_states', JointState, get_simulated_joints)
	rospy.Subscriber('joint_states_real', Float64MultiArray, get_real_joints)

	motor_controller_pub = rospy.Publisher('joint_commands', Float64MultiArray, queue_size=1)
	actual_feet_pub = rospy.Publisher('actual_feet', Float64MultiArray, queue_size=1)
	actual_upper_pub = rospy.Publisher('actual_upper', Float64MultiArray, queue_size=1)
	inv_kin_pos_pub = rospy.Publisher('inv_kin_pos', Float64MultiArray, queue_size=1)
	qdot_pub = rospy.Publisher('qdot', Float64MultiArray, queue_size=1)


	print("RT_inverse_kinematic_server_motor is online.")
	rate = rospy.Rate(1 / DT)  # Hz

	actual_t = rospy.Time.now()     # current time
    # rate = rospy.Rate(1 / DT)       # [Hz] publication frequency

	# try:
	# 	tempo = 0.
	# 	time.sleep(10)
		
	# 	while not rospy.is_shutdown():  # check for Ctrl-C

	# 		tempo = eval_inversion(motor_controller_pub,tempo)    # main function

	# 		# start_time = time.time() # TODO: utile per la temporizzazione
	# 		# print("[Evaluation time: %s sec]" % (time.time() - start_time))
	# 		# it seems to run at ~1ms
	# 		rate.sleep()  # "rate" (instead of "time") can handle simulated time

	# except rospy.ROSInterruptException:
	# 	pass

###############################
###     THREAD PERIOD SET   ###
###############################
THREAD_PERIOD = 0.1 # [s]

###############################
###     EXECUTION MONITOR   ###
###############################
PRINT_MEAS_PERIOD =  True
PRINT_MEAS_EXE_TIME = True

PERIOD_PERC_ALERT = 75  # % of THREAD_PERIOD
                        # when PRINT_MEAS_EXE_TIME = True, and measured execution time is greater or equal of specified percentage of period, the text line on cmd will be displayed in red

################################
###     PRINT FORMATTING    ####
################################
class color:
   PURPLE = '\033[95m'
   CYAN = '\033[96m'
   DARKCYAN = '\033[36m'
   BLUE = '\033[94m'
   GREEN = '\033[92m'
   YELLOW = '\033[93m'
   RED = '\033[91m'
   BOLD = '\033[1m'
   UNDERLINE = '\033[4m'
   END = '\033[0m'


####################################################################################################################
###     AUX FUNCTIONS       ########################################################################################
####################################################################################################################
def check_master():
    ''' check the presence of a master node '''

    if rosgraph.is_master_online():
        print('ROS MASTER check: ' + color.GREEN + 'Online' + color.END)

    # if ROS master is not online, loop until it is online 
    # each loop waits for the user to press enter
    else:
        while not rosgraph.is_master_online(): # Checks the master uri and results boolean (True or False)
            print('ROS MASTER check: '+ color.RED + 'Offline' + color.END )
            input('press ENTER to check again...')
        
        print(color.GREEN + 'ROS MASTER is now Online' + color.END)




##############################################################################################################
###     THREAD        ########################################################################################
##############################################################################################################
class thread_test (threading.Thread):
    
    
    def __init__(self):
        
        # initialize thread
        threading.Thread.__init__(self) 

        # assign period     
        self.T_thread= THREAD_PERIOD  

        # initialize timestamp variable
        # t_init is used to store current time at the beginning of each execution
        self.t_init = 0     

        self.ROS_connector = None
        # initialize variables for execution analysis 
        if PRINT_MEAS_PERIOD:
            self.meas_period = 0

        if PRINT_MEAS_EXE_TIME:
            self.exe_time = 0
        ##############


        # initialize topic names for reception and transmission of data in ROS
        self.ROS_rx_topicID = "rx_topic"
        self.ROS_tx_topicID = "tx_topic"
        self.ROS_rx_data = None
        self.ROS_tx_data = None
    
    
    def __del__(self):
        pass

    

    #########################
    ### ROS_CONNECTION     ##
    #########################
    def ROS_connection_init(self):
        
        # Start ROS node
        try:
            #                                   add tx topic
            self.ROS_connector = ROS_connector(self.ROS_rx_topicID)
            self.ROS_connector.start()
        except Exception as e:
            print(e)

            

        
    ##############################
    ### ROS CONNECTION GET DATA ##
    ##############################
    def ROS_connection_get_data(self):

        # wait until there is no data
        while self.ROS_connector.DataAvailable():
            # get received data

            self.ROS_rx_data = self.ROS_connector.Get_data()
            self.data_TimeStamp  = time.time()

            ## notify JUST ONCE if data update stops and restarts
            # if there are no new data available
            if self.ROS_rx_data == None:
                # check if also previous execution retrieved no data from ROS
                # if previous data were present, then it is the first iteration without fresh data, notify to the user
                if (self.prev_ROS_rx_data != None):
                    print(color.YELLOW + 'Waiting for lidar data - first iteration without new lidar samples' + color.END)
                    
                
            # if there are new data, but the iteration before retrieved no data from ROS,
            # notify the user that this is the first iteration with new data avalaible 
            elif (self.prev_ROS_rx_data == None):
                print(color.GREEN + 'Receiving lidar data  - first iteration with new lidar samples' + color.END)
        
            

        # update prev_ROS_rx_data for the next iteration
        self.prev_ROS_rx_data = self.ROS_rx_data
        return self.ROS_rx_data


    ################################################################################################################## THREAD MAIN 
    def run(self):

        print('###############################')
        print('# Initializing ___ controller #')
        print('###############################')

        ####################### initialization BEGIN
        # check ros server
        check_master()

        # start ROS connection
        self.ROS_connection_init()

        print (color.GREEN + '\nInitialization completed!' + color.END)
        ####################### initialization END

        
        
        print ('\nStarting control loop')
        ### control loop
        while True:

            # for timing debug
            # before next iteration begins, measure how much time passed since the last itaration started
            if PRINT_MEAS_PERIOD:

                self.meas_period = (time.time()-self.t_init)*1000

            # new iteration starting point
            # save entry time
            self.t_init=time.time()


            ########################## THREAD MAIN CODE BEGIN

			tempo = 0.
			time.sleep(10)

            # retrieve data from ROS
            my_useful_data = self.ROS_connection_get_data()
            timestamp = time.time()
            tempo = eval_inversion(motor_controller_pub,tempo)
            print('exe kin inv', time.time() - timestamp)
            print('hi \n')


            ########################## THREAD MAIN CODE END
            

            # sleep fot the remaining time
            self.wait_for_next_period()








    def execution_time_analysis(self):
        # exe debug info
        if PRINT_MEAS_EXE_TIME and PRINT_MEAS_PERIOD:
            self.exe_time = (time.time() - self.t_init)*1000
            if (self.exe_time >= PERIOD_PERC_ALERT/100*THREAD_PERIOD*1000) and  (self.exe_time < THREAD_PERIOD*1000):
                print('||'+ color.BOLD + color.YELLOW + ' exe time[ms]: ' + color.END , round(self.exe_time,4), '  \t||' + color.BOLD + ' period[ms]: ' + color.END, round(self.meas_period,4),'  \t ||')
            
            elif self.exe_time >= THREAD_PERIOD*1000:
                print('||'+ color.BOLD + color.RED + ' exe time[ms]: ' + color.END , round(self.exe_time,4), '  \t||' + color.BOLD + ' period[ms]: ' + color.END, round(self.meas_period,4),'  \t ||')

            else:

                print('||'+ color.BOLD + ' exe time[ms]: ' + color.END , round(self.exe_time,4), '  \t||' + color.BOLD + ' period[ms]: ' + color.END, round(self.meas_period,4),'  \t ||')
        

        elif PRINT_MEAS_PERIOD:
            print('||'+ color.BOLD + ' period[ms]: ' + color.END , round(self.meas_period,4), '\t ||')
        

        elif PRINT_MEAS_EXE_TIME:
            self.exe_time = (time.time() - self.t_init)*1000

            if self.exe_time >= PERIOD_PERC_ALERT/100*THREAD_PERIOD*1000:
                print('||'+ color.BOLD + color.RED +  ' exe time[ms]: ' + color.END , round(self.exe_time,4), '\t ||')

            else:  
                print('||'+ color.BOLD + ' exe time[ms]: ' + color.END , round(self.exe_time,4), '\t ||')





    def wait_for_next_period(self):

        self.execution_time_analysis()
        # sleep until next period
        # eval REMAINING_TIME = PERIOD - EXE_TIME
        #   if REMAINING_TIME > 0, sleep for the REMAINING_TIME or  
        #   if REMAINING_TIME < 0, sleep for 0.001
        #                max(min(REMAINING_TIME, PERIOD), PERIOD)
        time_sleep = max(min(self.T_thread - (time.time() - self.t_init), self.T_thread),0.001)
        time.sleep(time_sleep)



if __name__ == '__main__':
    my_thread = thread_test()
    my_thread.start()