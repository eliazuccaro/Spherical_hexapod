from math import ceil,floor
from time import sleep, time
from serial import Serial
import RPi.GPIO as gpio			# GPIO control module 
from support.ax12 import Ax12
from numpy import pi


class Servo:

	# /////////////////////////////// angle limits (folding fan + 1-2-3 joints)
	LIM_FF = [60,126]
	LIM_1J = [50,150]
	LIM_2J = [50,300]
	LIM_3J = [50,300]

	# /////////////////////////////// offset values (folding fan + 1-2-3 joints)
	OS_FF = 60
	OS_1J = 114
	OS_2J = 150
	OS_3J = 163

	# /////////////////////////////// conversion costants
	EXA_TO_DEG = 300/1023
	DEG_TO_EXA = 1023/300
	RAD_TO_DEG = 180/pi
	DEG_TO_RAD = pi/180

	# /////////////////////////////// other constants
	DELAY = 0.1
	MIN_ANGLE = 1
	MAX_ANGLE = 300
	MAX_LOOP = 20
	TOT_SERVO = 31
	TOT_LEGS = 6

	# Custom error class to report errors on checkServo
	class servoError(Exception):
		pass
	
	def conv2ax(a):
		a = int(a * Servo.DEG_TO_EXA)
		return a
	
	def rangeAngleEx(a):
		if a < Servo.MIN_ANGLE:
			a = Servo.MIN_ANGLE
			print("Error rangeAngleEx MIN_ANGLE")
		if a > Servo.MAX_ANGLE:
			a = Servo.MAX_ANGLE
			print("Error rangeAngleEx MAX_ANGLE")
		return a

	def __init__(self,used_servo=3,minID=0,maxID=20,baudrate=500000,setup=True,verbose=False):
		self.min_angle = Servo.MIN_ANGLE
		self.max_angle = Servo.MAX_ANGLE
		self.tot_servo = Servo.TOT_SERVO
		self.tot_legs = Servo.TOT_LEGS
		self.ax = Ax12(baud_rate=baudrate)
		try:
			id_list,curr_legs = self.checkServo(used_servo,minID,maxID,verbose) # create ID list
			self.id = id_list
			self.num = len(id_list)
			self.curr_legs = curr_legs
			self.lim_cw = [self.min_angle] * self.tot_servo
			self.lim_ccw = [self.max_angle] * self.tot_servo
			self.offset = [self.min_angle] * self.tot_servo
			self.curr_angles = [self.min_angle] * self.num
			if setup:
				self.setup()
		except Exception as detail:
			print(str(detail))
			pass

	def setup(self):
		print("curr_legs = ", self.curr_legs)
		self.setOffset()
		self.updateAngleAll() # update self.curr_angle
		print("Servo ID list = ", self.id)
		print("Starting angles = ", self.curr_angles)
		self.setupAngleLim() # setAngleLim on all servos
		# self.printTorqueLimit()
		for i in range(1,self.curr_legs+1):
			self.curr_angles[3*i-2] =	self.conv_to_servo(3*i-2, 0.)
			self.curr_angles[3*i-1] =	self.conv_to_servo(3*i-1, -4.*pi/180)
			self.curr_angles[3*i] =		self.conv_to_servo(3*i, 89.*pi/180)
		# for i in range(self.num):
			# self.ax.setVoltageLimit(self.id[i],88,130)
			# self.ax.setLedAlarm(self.id[i],"Input Voltage")
			# self.ax.setTorqueLimit(self.id[i],1023) # set maximum torque for all servos
			# v = self.ax.readVoltageLowerLimit(self.id[i])
			# print(f"Input Voltage Lower Limit servo ID {self.id[i]} = ({v})", end="     ")
		print()
		self.printAngles()
	
	def changeBaudRate(self,i,br):
		self.ax.setBaudRate(i,br)
	
	def checkServo(self,used_servo,minID=0,maxID=253,verbose=False):
		id_list = self.ax.learnServos(minID,maxID,verbose)
		confirm = len(id_list) == used_servo
		curr_legs = 0
		try:
			assert confirm == 1
		except:
			e = "\n================================================================================\n"
			e += "Error: discrepancy between given variable used_servo and number of servos found.\n"
			e += "Variable used_servo: " + str(used_servo) + "\n"
			e += "Actual number of servos: " + str(len(id_list)) + "\n"
			e += "Actual list of servos: " + str(id_list) + "\n"
			e += "================================================================================\n"
			raise Servo.servoError(e)
		if id_list[0] == 0: # the first is the folding fan servo
			curr_legs = floor((used_servo-1)/3) # -1 to single out the folding fan servo from the computation
		else:
			curr_legs = floor(used_servo/3)
		return id_list,curr_legs
	
	def printAngles(self):
		for i in range(self.num):
			a = self.curr_angles[i]
			# a = self.conv_from_servo(i,a)
			print(f"Angle servo ID {self.id[i]} = ({a})", end="")
			if a < 10:				print("       ", end="")
			if a >= 10 and a < 100:	print("      ", end="")
			else:					print("     ", end="")
		print("\n")

	def updateAngle(self,i): # i is always the position in order, not merely the ID of the servo
		r = self.ax.readPosition(self.id[i])
		count = 0
		while r <= 0 and count < Servo.MAX_LOOP: #r == -1
			r = self.ax.readPosition(self.id[i]) # try again if servo didn't respond
			count += 1							 # to avoid infinite loop
		# print()
		# print("id = ",i)
		# print("attempts = ",count+1)
		# print()
		if count >= Servo.MAX_LOOP:
			# pass
			print("Function readPosition inside updateAngle failed " + str(Servo.MAX_LOOP) + " times for servo " + str(self.id[i]))
			# self.ax.ping(self.id[i]) # checks again for timeout error (ping function bypasses -1 error return)
			# r = self.ax.readPosition(self.id[i])
		else:
			# print("id = ", i)
			# print("r = ", r)
			r_deg = r * Servo.EXA_TO_DEG
			# print("r_deg = ", r_deg)
			r_deg = int(r_deg)
			# print("(int)r_deg = ", r_deg)
			# print()
			if r_deg >= Servo.MIN_ANGLE and r_deg <= Servo.MAX_ANGLE:
				self.curr_angles[i] = r_deg
			else:
				print("Error: servo angle read returns unexpected value --", r_deg)

	def updateAngleAll(self):
		for i in range(self.num):
			# tempo = time()
			self.updateAngle(i)
			# passato = time() - tempo
			# print("elapsed = ", passato)

	def setAngle(self,i,a,speed=100): # i is always the position in order, not merely the ID of the servo
		# print("i = ", i)
		# print("set_ANGLE ++++++++++++++++++++++++++++++++++")
		a = Servo.rangeAngleEx(a)
		a = self.rangeAngleLim(i,a)
		a = Servo.conv2ax(a)
		r = self.ax.moveSpeed(self.id[i], a, speed)
		count = 0
		while r < 0 and count < Servo.MAX_LOOP:
			r = self.ax.moveSpeed(self.id[i], a, speed) # try again if servo didn't respond
			count += 1							 # to avoid infinite loop
		if count >= Servo.MAX_LOOP:
			print("Function moveSpeed inside setAngle failed " + str(Servo.MAX_LOOP) + " times for servo " + str(self.id[i]))
			# self.ax.ping(self.id[i]) # checks again for timeout error (ping function bypasses -1 error return)
			# r = self.ax.readPosition(self.id[i])
		# print("UPDATE_ANGLE ++++++++++++++++++++++++++++++++++")
		# self.updateAngle(i)
	
	def setAngleAll(self,b,speed=50):
		for i in range(self.num):
			self.setAngle(i, b[i], speed)
	
	def checkVoltage(self,i):
		r = self.ax.readVoltage(self.id[i])
		count = 0
		while r == -1 and count < Servo.MAX_LOOP:
			r = self.ax.readVoltage(self.id[i])	# try again if servo didn't respond
			count += 1							# to avoid infinite loop
		voltage = r/10
		return voltage
	
	def setAngleLim(self,i,lim_vett):
		lim_cw,lim_ccw = lim_vett	# clockwise - counter-clockwise
		self.lim_cw[i], self.lim_ccw[i] = lim_cw, lim_ccw
		lim_cw,lim_ccw = Servo.rangeAngleEx(lim_cw),Servo.rangeAngleEx(lim_ccw)
		lim_cw,lim_ccw = Servo.conv2ax(lim_cw),Servo.conv2ax(lim_ccw)
		self.ax.setAngleLimit(self.id[i], lim_cw, lim_ccw)
		sleep(Servo.DELAY)
	
	def setupAngleLim(self):
		if self.id[0] == 0:
			self.setAngleLim(0, Servo.LIM_FF) # servo 0 folding fan
			ff = 0 # to adjust next index
		else:
			ff = 1
		for i in range(1,self.curr_legs+1):
			self.setAngleLim(i*3-2-ff, Servo.LIM_1J)
			if self.num > 1:
				self.setAngleLim(i*3-1-ff, Servo.LIM_2J)
				self.setAngleLim(i*3-ff, Servo.LIM_3J)
		self.printAngleLim()
	
	def printAngleLim(self):
		for i in range(self.num):
			print(f"Angle limits servo ID {self.id[i]} = ({self.lim_cw[i]}, {self.lim_ccw[i]})")
	
	def setOffset(self):
		os = [0] * self.tot_servo
		os[0] = Servo.OS_FF	# servo 0 folding fan
		for i in range(1,self.tot_legs+1):
			os[i*3-2] = Servo.OS_1J
			os[i*3-1] = Servo.OS_2J
			os[i*3] =	Servo.OS_3J
		self.offset = os

	def conv_from_servo(self,i,a):
		return (a - self.offset[i]) * Servo.DEG_TO_RAD
	
	def conv_to_servo(self,i,a):
		return a * Servo.RAD_TO_DEG + self.offset[i]
	
	def rangeAngleLim(self,i,a):
		if a < self.lim_cw[i]:
			a = self.lim_cw[i]
			print("Error rangeAngleLim MIN limit")
		if a > self.lim_ccw[i]:
			a = self.lim_ccw[i]
			print("Error rangeAngleLim MAX limit")
		return a
	
	def checkChangedAngle(self,i,a):
		ca = self.curr_angles[i]
		if ca > a + 2 or ca < a - 2:
			return 1
		else:
			return 0
	
	def checkChangedAngles(self,a):
		for i in range(self.num):
			ca = self.curr_angles[i]
			if ca > a[i] + 2 or ca < a[i] - 2:
				return 1
		return 0
	
	def printVoltage(self):
		for i in range(self.num):
			v = self.checkVoltage(i)
			print(f"Voltage servo ID {self.id[i]} = ({v})", end="     ")
		print("\n")
	
	def enableTorqueAll(self):
		for i in range(self.num):
			self.ax.setTorqueStatus(self.id[i],1)
	
	def disableTorqueAll(self):
		for i in range(self.num):
			self.ax.setTorqueStatus(self.id[i],0)
	
	def printTorqueStatus(self):
		for i in range(self.num):
			print(f"Torque enable servo ID {self.id[i]} = ({self.ax.readTorqueStatus(self.id[i])})", end="     ")
		print()
	
	def printTorqueLimit(self):
		for i in range(self.num):
			print(f"Torque limit servo ID {self.id[i]} = ({self.ax.readTorqueLimit(self.id[i])})", end="     ")
		print()

	def changeID(self,i,newID):
		self.ax.setID(self.id[i], newID)
		self.id[i] = newID
		sleep(Servo.DELAY)
		print("Updated list of ID = ", self.id)
	
	def blink(self,time=1):
		for i in self.id: self.ax.setLedStatus(i,1)
		sleep(time)
		for i in self.id: self.ax.setLedStatus(i,0)
		sleep(time)
	
	def offLED(self):
		for i in self.id:
			self.ax.setLedStatus(i, 0)
			sleep(Servo.DELAY)