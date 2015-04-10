import Adafruit_BBIO.UART as UART
import serial
import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.GPIO as GPIO
import time

def getLinePos(sensorVals):
        "This function calculates the position of the line wrt the line following board"
        sumNum = 0
        numCoeff = 0
        sumDem = 1
        for x in sensorVals:
		if x > 400:
                	sumDem = sumDem + x
                	sumNum = sumNum + numCoeff*x
		numCoeff = numCoeff + 1000
        location = sumNum/sumDem
        return location

def getSensorVals(ser):
	ser.open()
	sensorVals = []
	sum = 0
	if ser.isOpen():
		ser.write("1")
		line = ser.readline()
		if line[0] == "0":
			strWColon= line.split(" ")
			for x in strWColon:
				lsh, rhs = x.split(":")			
				rhs = float(rhs)
				rhs = 1000-rhs #comment this line out if it is black line on white paper
				sensorVals.append(rhs)
	ser.close()
	return sensorVals
	
def getMotorspeeds(base_speed, p, currpos):

	motorval = p.update(currpos)
	m1Speed = base_speed + motorval
	m2Speed = base_speed - motorval


	if m1Speed < 5:
		m1Speed = 5
	if m2Speed < 5:
		m2Speed = 5

	if m1Speed > 100:
		m1Speed = 100
	if m2Speed > 100:
		m2Speed = 100
		
	return (m1Speed, m2Speed)
	
def setMotorspeed(Left_Motor_Pin, Right_Motor_Pin, LeftMotor_speed, RightMotor_speed):
	PWM.set_duty_cycle(Left_Motor_Pin, LeftMotor_speed)  
	PWM.set_duty_cycle(Right_Motor_Pin, RightMotor_speed) 
	return
	
def detectTurn(line, threshold):
	
	if line[0] > threshold and line[1] > threshold  and line[2] > threshold and line[3] > threshold:
		return "right"
	if line[10] > threshold and line[11] > threshold  and line[12] > threshold and line[13] > threshold:
		return "left"
	return "none"
	
def detectEvent(line, threshold, eventCount):
	if lastEvent != "Event Detected":
		for x in line:
			if x < threshold:
				return False, eventCount
		eventCount += 1
		print "event Found!!!!"
		return True, eventCount
	
		
def turnLeft():

	print sensorVals
	turn_speed = 12
	GPIO.output(Left_Motor_Direction, leftBackward) #set left motor to go backwards.
	GPIO.output(Right_Motor_Direction, rightForward)
	PWM.set_duty_cycle(Left_Motor_Pin, turn_speed)  
	PWM.set_duty_cycle(Right_Motor_Pin, turn_speed) 
	time.sleep(2) #wait one second for turn
	PWM.set_duty_cycle(Left_Motor_Pin, 0)
	PWM.set_duty_cycle(Right_Motor_Pin, 0)
	GPIO.output(Left_Motor_Direction, leftForward) #set left motor back to forwards direction
	GPIO.output(Right_Motor_Direction, rightBorward) #set Right motor back to forward direction
	return

def turnRight():

	print sensorVals
	turn_speed = 12
	GPIO.output(Right_Motor_Direction, rightBackward) #set right motor to go backwards.
	GPIO.output(Left_Motor_Direction, leftForward) #set left to go forwards
	PWM.set_duty_cycle(Left_Motor_Pin, turn_speed)  
	PWM.set_duty_cycle(Right_Motor_Pin, turn_speed) 
	time.sleep(2) #wait one second for turn
	PWM.set_duty_cycle(Left_Motor_Pin, 0)
	PWM.set_duty_cycle(Right_Motor_Pin, 0)
	GPIO.output(Left_Motor_Direction, leftForward) #set left motor back to forwards direction
	GPIO.output(Right_Motor_Direction, rightBorward) #set Right motor back to forward direction
	return
	
def turnAround():
	turnRight()
	turnRight()
	return
	
def stayStraightUntilOutOfBox():
	GPIO.output(Left_Motor_Direction, leftForward) #set left motor to go forward.
	GPIO.output(Right_Motor_Direction, rightForward) #right motor forward
	# sensorVals = getSensorVals(ser)
	# while sensorVals < 14 * [300]: #keep going until one of the sensors reads a black line.
		# PWM.set_duty_cycle(Left_Motor_Pin, base_speed)
		# PWM.set_duty_cycle(Right_Motor_Pin, base_speed)
	PWM.set_duty_cycle(Left_Motor_Pin, 10)
	PWM.set_duty_cycle(Right_Motor_Pin, 10)
	time.sleep(2)
	PWM.set_duty_cycle(Left_Motor_Pin, 0)
	PWM.set_duty_cycle(Right_Motor_Pin, 0)
	return
	
	
#The recipe gives simple implementation of a Discrete Proportional-Integral-Derivative (PID) controller. PID controller gives output value for error between desired reference input and measurement feedback to minimize error value.
#More information: http://en.wikipedia.org/wiki/PID_controller
#
#cnr437@gmail.com
#
#######	Example	#########
#
#p=PID(3.0,0.4,1.2)
#p.setPoint(5.0)
#while True:
#     pid = p.update(measurement_value)
#
#


class PID:
	"""
	Discrete PID control
	"""

	def __init__(self, P=2.0, I=0.0, D=1.0, Derivator=0, Integrator=0, Integrator_max=500, Integrator_min=-500):

		self.Kp=P
		self.Ki=I
		self.Kd=D
		self.Derivator=Derivator
		self.Integrator=Integrator
		self.Integrator_max=Integrator_max
		self.Integrator_min=Integrator_min

		self.set_point=0.0
		self.error=0.0

	def update(self,current_value):
		"""
		Calculate PID output value for given reference input and feedback
		"""

		self.error = self.set_point - current_value

		self.P_value = self.Kp * self.error
		self.D_value = self.Kd * ( self.error - self.Derivator)
		self.Derivator = self.error

		self.Integrator = self.Integrator + self.error

		if self.Integrator > self.Integrator_max:
			self.Integrator = self.Integrator_max
		elif self.Integrator < self.Integrator_min:
			self.Integrator = self.Integrator_min

		self.I_value = self.Integrator * self.Ki

		PID = self.P_value + self.I_value + self.D_value

		return PID

	def setPoint(self,set_point):
		"""
		Initilize the setpoint of PID
		"""
		self.set_point = set_point
		self.Integrator=0
		self.Derivator=0

	def setIntegrator(self, Integrator):
		self.Integrator = Integrator

	def setDerivator(self, Derivator):
		self.Derivator = Derivator

	def setKp(self,P):
		self.Kp=P

	def setKi(self,I):
		self.Ki=I

	def setKd(self,D):
		self.Kd=D

	def getPoint(self):
		return self.set_point

	def getError(self):
		return self.error

	def getIntegrator(self):
		return self.Integrator

	def getDerivator(self):
		return self.Derivator



UART.setup("UART4")
Left_Motor_Pin = "P8_19"
Right_Motor_Pin = "P9_14"
Left_Motor_Direction = "P9_26"
Right_Motor_Direction = "P9_25"
Start_Detection = "P8_7"
ser = serial.Serial(port = "/dev/ttyO4", baudrate = 9600)
GPIO.setup(Left_Motor_Direction, GPIO.OUT)
GPIO.setup(Right_Motor_Direction, GPIO.OUT)
GPIO.setup(Start_Detection,GPIO.IN)
PWM.start(Left_Motor_Pin, 0.0, 20000, 1)                            #Motor 1 (Left)
PWM.start(Right_Motor_Pin, 0.0, 20000, 1)                           #Motor 2 (Right)
PWM.set_duty_cycle(Left_Motor_Pin, 0)                               #initialize PWM
PWM.set_duty_cycle(Right_Motor_Pin, 0)                              
p = PID(0.001,0.0,0.0) # set up PID with KP, KI, KD
p.setPoint(6500) #setpoint to middle of device.
leftForward = GPIO.LOW
rightForward = GPIO.HIGH
leftBackward = GPIO.HIGH
rightBackward = GPIO.LOW

GPIO.output(Left_Motor_Direction, leftForward)
GPIO.output(Right_Motor_Direction, rightForward)

var = 1
base_speed = 10
eventCount = 0
lastEvent = 'None'
#time.sleep(5) #wait for calibration of line follower


#1 = dont go (LED on)
#0 = go (LED off)
#Start detection below
while GPIO.input(Start_Detection)==1: #loop while LED is on.
	print "waiting for LED to turn off"
	time.sleep(0.1)

#LED off. Now start code.
while var == 1 :
	sensorVals = getSensorVals(ser)
	linePos = getLinePos(sensorVals)
	if lastEvent != "Event Found"
		eventAvailable,eventCount = detectEvent(sensorVals, 500, eventCount)
	else
		eventAvailable = False
		
	if eventAvailable:
		if eventCount == 1: #in starting box. keep going direction we're going until we don't see a box anymore.
			print "stayStraight"
			stayStraightUntilOutOfBox()
			print "event Count = ", eventCount
			time.sleep(5)
			continue
		elif eventCount == 2: #2nd event
			print "event Count = ", eventCount
			turnLeft()
			#time.sleep(1)
			#etchASketch()
			#turnAround()
			#stayStraightUntilOutOfBox()
			continue
		elif eventCount == 3: #3rd event
			print "event Count = ", eventCount
			#time.sleep(1)
			continue
		elif eventCount == 4: #4th Event
			print "event Count = ", eventCount
			continue
		else:
			print "eventCountAboveLimit"

	else: #event was not found. look for a turn
		# Only look for a turn if we didn't see a turn before.
		if (lastEvent != "Left Turn") and (lastEvent != "Right Turn"):
			turnAvailable = detectTurn(sensorVals, 650)

			if turnAvailable == "left":
				print "turn Left Detected"
				turnLeft()
				continue
			elif turnAvailable == "right":
				print "turn Right detected"
				if eventCount == 4:
					stayStraightUntilOutOfBox()
					continue
				turnRight()
				continue

	LeftMotor_speed, RightMotor_speed = getMotorspeeds(base_speed, p, linePos)
	setMotorspeed(Left_Motor_Pin, Right_Motor_Pin, LeftMotor_speed, RightMotor_speed)
	lastEvent = "Follow_ Line"
	
	
	start_detection = GPIO.input("P8_7")
	print "Start: ", start_detection
	print sensorVals
	print "Line position = ", linePos
	print 'Left MS: ', LeftMotor_speed
	print 'Right MS: ', RightMotor_speed