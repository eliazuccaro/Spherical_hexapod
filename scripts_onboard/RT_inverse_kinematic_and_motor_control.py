#!/usr/bin/env python3

from math import pi, sin, cos, sqrt, tan
import numpy as np
import shellbot_kinematic_laws as kinematic_laws
import time

from support.ax12 import Ax12
from support.servo import Servo
import threading
from ROS_connector import *
import rosgraph

##################### GENERAL INITIALIZATIONS #####################

# Update frequency for joints desired angles (pay attention to
# gait_handler.DT value, something faster than that is just useless)
DT = 0.5  # [s]

motors_number = 31
x = np.empty(18)
x_d = np.empty(18)
x_d_dot = np.empty(18)

# initializations
list_of_desired_angles = [0] * 19
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

global prev_t
prev_t = time.time()

joint_states_legs = joint_states[1:]


height = -170.
dist = 300.
l_23_z = kinematic_laws.l_23_z
for leg_j in range(1, 7):

	# still robot trajectories
	x[3*leg_j - 3] = dist * cos(2*(leg_j-1)*pi/6) - l_23_z * sin(2*(leg_j-1)*pi/6)
	x[3*leg_j - 2] = l_23_z * cos(2*(leg_j-1)*pi/6) + dist * sin(2*(leg_j-1)*pi/6)
	x[3*leg_j - 1] = height

delta = kinematic_laws.delta
epsilon = kinematic_laws.epsilon
k = 0.1       # CLIK proportional constant
tau = 0

s = Servo(used_servo=19,minID=0,maxID=18,baudrate=1000000,setup=True,verbose=True)

command_rad = [0] * 19

legs = 6
global command_deg 
command_deg = [0] * 19
command_deg[0] =		 s.conv_to_servo(0, 0.)	# command servo 0 folding fan
for i in range(1,legs+1):
	command_deg[3*i-2] = s.conv_to_servo(3*i-2, 0.)
	command_deg[3*i-1] = s.conv_to_servo(3*i-1, -4.*pi/180)
	command_deg[3*i] =	 s.conv_to_servo(3*i, 89.*pi/180)

joint_angles = [0] * 19
joint_angles[0] =		  0.
for i in range(1,legs+1):
	joint_angles[3*i-2] = 0.
	joint_angles[3*i-1] = -4.*pi/180
	joint_angles[3*i] =	  89.*pi/180

##################### END OF GENERAL INITIALIZATIONS #####################

##################### INVERSE KINEMATIC AND MOTOR CONTROL STUFF #####################

# evaluate joint angles for given body and tiptoes position and publish to motors
def eval_inversion_and_motor_control(x_d, x_d_dot):

	# ---------------- MOTOR CONTROL ROS READ ----------------
	s.updateAngleAll()

	# get joints angles
	joint_angles[0] = s.conv_from_servo(0, s.curr_angles[0])
	for i in range(1, s.num):
		joint_angles[i] = s.conv_from_servo(i, s.curr_angles[i])

	# ---------------- DIRECT KINEMATICS ----------------

	joint_states_legs = joint_angles[1:]
	
	# current feet positions calculation (f is the direct kinematics function)
	for j in range(1, 7):

		# x_j --> pose (x, y, z, phi, theta, psi)
		x_j = kinematic_laws.f(j, 0, joint_states_legs[3*j-3], joint_states_legs[3*j-2], joint_states_legs[3*j-1])[0:3,0]    # take position but not orientation
		x[3*j-3 : 3*j] = x_j    # fill feet position vector

	# ----------------------------------------------------

	# ---------------------- CLIK --> suitable for all bottom legs movements ----------------------
	for j in range(1, 7):

		J_a_inv_j = kinematic_laws.qbj_jacob(j, [joint_states_legs[3*j-3], joint_states_legs[3*j-2], joint_states_legs[3*j-1]])                    # inverse of jacobian matrix (3x3)
		qk_old = np.array([0, joint_states_legs[3*j-3], joint_states_legs[3*j-2], joint_states_legs[3*j-1]])     # current joints positions
		x_j = x[3*j-3 : 3*j]       # position vector j_th bottom foot

		ke = np.multiply((x_d[3*j-3 : 3*j] - x_j), k)    # 1x3

		alpha = np.multiply(x_d_dot[3*j-3 : 3*j], 1)
		
		q_dot.append(0.)
		q_dot.append(J_a_inv_j.dot(alpha + ke)[0])
		q_dot.append(J_a_inv_j.dot(alpha + ke)[1])
		q_dot.append(J_a_inv_j.dot(alpha + ke)[2])

		qk[4*j-4 : 4*j] = qk_old + np.multiply(q_dot[4*j-4 : 4*j], DT)          # joint angles j-th bottom leg

	list_of_desired_angles = qk        # full joint positions vector
	print("list_of_desired_angles: ", list_of_desired_angles)

	# -----------  robot reale: riduzione giunti da 36 a 31 e scalatura con rapporto di trasmissione -----------
	if (motors_number == 31):

		# angolo di giunto dell'ingranaggio della prima gamba
		leg_gear_joint_angle = list_of_desired_angles[0]
		transmission_rate = 1/1.75
		central_motor_joint_angle = leg_gear_joint_angle * transmission_rate

		# rimozione degli angoli di giunto degli ingranaggi di inzio gamba 0, 4, 8, 12, 16, 20
		del list_of_desired_angles[20]
		del list_of_desired_angles[16]
		del list_of_desired_angles[12]
		del list_of_desired_angles[8]
		del list_of_desired_angles[4]
		del list_of_desired_angles[0]

		# inserimento angolo di giunto ingranaggio centrale
		list_of_desired_angles.insert(0, central_motor_joint_angle)

	command_rad = list_of_desired_angles # NOTATION CHANGE

	# print("command_rad = ", command_rad)

	command_deg[0] = s.conv_to_servo(0, -command_rad[0])
	for i in range(1, legs+1):

		command_deg[3*i-2] = s.conv_to_servo(3*i-2, -command_rad[3*i-2])
		command_deg[3*i-1] = s.conv_to_servo(3*i-1, command_rad[3*i-1])
		command_deg[3*i] =	 s.conv_to_servo(3*i, command_rad[3*i])

	print("command deg = ", command_deg)

	# ---------------- MOTOR CONTROL ROS WRITE ----------------

	if s.checkChangedAngles(command_deg):

		s.setAngleAll(command_deg)

		print("command_deg = ",end="")

		for i in range(0, len(command_deg)):
			print(round(command_deg[i],2),end="\t")
		print()

	# empty positions and velocities vectors
	qk.clear()
	q_dot.clear()


##################### END OF INVERSE KINEMATIC STUFF #####################

##################### REAL TIME EXECUTION STUFF #####################

###############################
###     THREAD PERIOD SET   ###
###############################
THREAD_PERIOD = 0.5 # [s]

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


###############################
###     AUX FUNCTIONS       ###
###############################
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




#########################
###     THREAD        ###
#########################
class inversion_control (threading.Thread):
	
	def __init__(self):
		
		# initialize thread
		threading.Thread.__init__(self) 

		# assign period     
		self.T_thread= THREAD_PERIOD  

		# initialize timestamp variable
		# t_init is used to store current time at the beginning of each execution
		self.t_init = 0     

		self.ROS_connector_RT = None
		# initialize variables for execution analysis 
		if PRINT_MEAS_PERIOD:
			self.meas_period = 0

		if PRINT_MEAS_EXE_TIME:
			self.exe_time = 0
		##############
	
	
	def __del__(self):
		pass

	#########################
	### ROS_CONNECTION     ##
	#########################
	def ROS_connection_init(self):

		# initialize topic names for reception and transmission of data in ROS
		self.position_topic = "des_feet_position"
		self.velocity_topic = "des_feet_velocity"
		self.actual_feet_topic = "actual_feet"
		self.data_position = None
		self.data_velocity = None
		self.actual_feet_data = None
		
		# Start ROS node
		try:

			self.ROS_connector_RT = ROS_connector(self.position_topic, self.velocity_topic, self.actual_feet_topic)
			self.ROS_connector_RT.start_thread() # let the thread start
		except Exception as e:
			print(e)
		
	##############################
	### ROS CONNECTION GET DATA ##
	##############################
	def ROS_connection_get_data(self):

		data_position, data_velocity = self.ROS_connector_RT.Get_data()
		# data_TimeStamp  = time.time()

		general_data = [data_position, data_velocity]

		return general_data

	def run(self):

		print('###############################')
		print('# Initializing ___ controller #')
		print('###############################')

		####################### initialization BEGIN
		# check ros server
		check_master()

		# start ROS connection
		self.ROS_connection_init()

		time.sleep(2)

		print("Ready to set angles")

		s.setAngleAll(command_deg)

		print (color.GREEN + '\nInitialization completed!' + color.END)
		####################### initialization END
		
		print ('\nStarting control loop')

		### control loop
		while True:

			if self.ROS_connector_RT.DataAvailable():

				# for timing debug
				# before next iteration begins, measure how much time passed since the last itaration started
				if PRINT_MEAS_PERIOD:

					self.meas_period = (time.time()-self.t_init)*1000

				# new iteration starting point
				# save entry time
				self.t_init=time.time()

				# THREAD MAIN CODE BEGIN
				timestamp = time.time()

				x_d, x_d_dot = self.ROS_connection_get_data()

				eval_inversion_and_motor_control(x_d, x_d_dot) # read, calculate, write (for motors)

				global prev_t

				if time.time() - prev_t > 5:
					s.printAngles()
					s.printVoltage()
					prev_t = time.time()

				print('real-time execution period:', time.time() - timestamp)

				self.ROS_connector_RT.Set_data(x)
				# THREAD MAIN CODE END
				
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

		time_sleep = max(min(self.T_thread - (time.time() - self.t_init), self.T_thread),0.001)
		time.sleep(time_sleep)

##################### END OF REAL TIME EXECUTION STUFF #####################

# listen to des_pose and publish desired joint angles at rate 1/DT
if __name__ == "__main__":

	print("RT_inverse_kinematic_and_motor_control is online.")

	my_thread = inversion_control()
	my_thread.start()