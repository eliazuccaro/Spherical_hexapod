from time import sleep
from serial import Serial
import RPi.GPIO as gpio			# GPIO control module 
from support.ax12 import Ax12
from numpy import pi

class Servo:

	# /////////////////////////////// offset values (folding fan + 1-2-3 joints)
	OS_FF = 0
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
	MIN_ANGLE = 0
	MAX_ANGLE = 300
	MAX_LOOP = 20
	TOT_SERVO = 31
	TOT_LEGS = 6

	# Custom error class to report errors on checkServo
	class servoError(Exception):
		pass

	def __init__(self,used_servo=3,minID=0,maxID=20):
		self.ax = Ax12()
		id_list = self.checkServo(used_servo,minID,maxID) # create ID list
		self.id = id_list
		self.num = len(id_list)
		self.lim_cw = [Servo.MIN_ANGLE] * self.num
		self.lim_ccw = [Servo.MAX_ANGLE] * self.num
		self.min_angle = Servo.MIN_ANGLE
		self.max_angle = Servo.MAX_ANGLE
		self.tot_legs = Servo.TOT_LEGS
		self.tot_servo = Servo.TOT_SERVO
		self.offset = self.setOffset()
		self.curr_angles = [Servo.MIN_ANGLE] * self.num
		self.updateAngles() # update self.curr_angle
		print("Servo ID list = ", self.id)
		print("Starting angles = ", self.curr_angles)

	def checkServo(self,used_servo,minID=0,maxID=253):
		id_list = self.ax.learnServos(minID,maxID)
		confirm = len(id_list) == used_servo
		count = 0
		while confirm == 0 and count < Servo.MAX_LOOP:
			id_list = self.ax.learnServos(minID,maxID) # try again if servo didn't respond
			confirm = len(id_list) == used_servo
			count += 1	# to avoid infinite loop
		try:
			assert confirm == 1
		except Exception as detail:
			print("\n================================================================================")
			print("Error: discrepancy between given variable used_servo and number of servos found.")
			print("Actual number of servos: " + str(len(id_list)))
			print("Actual list of servos: " + str(id_list))
			print("================================================================================\n")
			raise Servo.servoError(detail)
		return id_list
	
	def updateAngles(self):
		curr_angles = [0] * self.num
		for i in range(self.num):
			r = self.ax.readPosition(self.id[i])
			count = 0
			while r == -1 and count < Servo.MAX_LOOP:
				r = self.ax.readPosition(self.id[i])	# try again if servo didn't respond
				count += 1								# to avoid infinite loop
			if count >= Servo.MAX_LOOP:
				self.ax.ping(self.id[i]) # checks again for timeout error (ping function bypasses -1 error return)
				r = self.ax.readPosition(self.id[i])
			curr_angles[i] = int(r * Servo.EXA_TO_DEG)
			# sleep(Servo.DELAY)
		self.curr_angles = curr_angles
	
	def setOffset(self):
		os = [0] * self.tot_servo
		os[0] = Servo.OS_FF	# offset servo 0 folding fan
		for i in range(1,self.tot_legs+1):
			os[i*3-2] = Servo.OS_1J
			os[i*3-1] = Servo.OS_2J
			os[i*3] =	Servo.OS_3J
		return os

	def conv_from_servo(self,i,a):
		return (a - self.offset[i]) * Servo.DEG_TO_RAD
	
	def conv_to_servo(self,i,a):
		return a * Servo.RAD_TO_DEG + self.offset[i]

	def rangeAngleEx(self,a):
		if a < Servo.MIN_ANGLE: a = Servo.MIN_ANGLE
		if a > Servo.MAX_ANGLE: a = Servo.MAX_ANGLE
		a = int(a * Servo.DEG_TO_EXA)
		return a

	def setAngleLim(self,i,lim_cw,lim_ccw): # clockwise - counter-clockwise
		self.lim_cw[i-1], self.lim_ccw[i-1] = lim_cw, lim_ccw
		lim_cw,lim_ccw = self.rangeAngleEx(lim_cw),self.rangeAngleEx(lim_ccw)
		self.ax.setAngleLimit(self.id[i-1], lim_cw, lim_ccw)
		sleep(Servo.DELAY)
	
	def printAngleLim(self):
		for i in range(self.num):
			print(f"Angle limits servo id {self.id[i]} = ({self.lim_cw[i]}, {self.lim_ccw[i]})")
			
	def checkChangedAngle(self,i,a):
		ca = self.curr_angles[i-1]
		if ca > a + 1 or ca < a - 1:
			return 1
		else:
			return 0
	
	def checkChangedAngles(self,a):
		for i in range(self.num):
			ca = self.curr_angles[i]
			if ca > a[i] + 1 or ca < a[i] - 1:
				return 1
		return 0

	def printAngles(self):
		for i in range(self.num):
			a = self.curr_angles[i]
			print(f"Angle servo id {self.id[i]} = ({a})", end="")
			if a < 10:				print("       ", end="")
			if a >= 10 and a < 100:	print("      ", end="")
			else:					print("     ", end="")
		print()
	
	def setAngle(self,i,a,speed=100): # i is always the position in order, not the ID of the servo
		a = self.rangeAngleEx(a)
		self.ax.moveSpeed(self.id[i-1], a, speed)
		# sleep(Servo.DELAY)
		self.updateAngles()
	
	def setAngleAll(self,b,speed=100):
		for i in range(self.num):
			a = self.rangeAngleEx(b[i])
			self.ax.moveSpeed(self.id[i], a, speed)
			# sleep(Servo.DELAY)
		self.updateAngles()
	
	def enableTorque(self):
		self.ax.setTorqueStatus#################################################
	
	def changeID(self,i,newID):
		self.ax.setID(self.id[i-1], newID)
		self.id[i-1] = newID
		sleep(Servo.DELAY)
	
	def blink(self,time=1):
		for i in self.id: self.ax.setLedStatus(i,1)
		sleep(time)
		for i in self.id: self.ax.setLedStatus(i,0)
		sleep(time)
	
	def offLED(self):
		for i in self.id:
			self.ax.setLedStatus(i, 0)
			sleep(Servo.DELAY)