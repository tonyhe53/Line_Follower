import Adafruit_BBIO.UART as UART
import serial
import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.GPIO as GPIO
import PID
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
				sensorVals.append(rhs)
	ser.close()
	return sensorVals
	
def getMotorspeeds(base_speed, p, currpos):

	motorval = p.update(currpos)
	m1Speed = base_speed + motorval
	m2Speed = base_speed - motorval


	if m1Speed < 10:
		m1Speed = 10
	if m2Speed < 10:
		m2Speed = 10

	if m1Speed > 100:
		m1Speed = 100
	if m2Speed > 100:
		m2Speed = 100
		
	return (m1Speed, m2Speed)
	
def setMotorspeed(Left_Motor_Pin, Right_Motor_Pin, LeftMotor_speed, RightMotor_speed):
	PWM.set_duty_cycle(Left_Motor_Pin, LeftMotor_speed)  
	PWM.set_duty_cycle(Right_Motor_Pin, RightMotor_speed) 
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
ser = serial.Serial(port = "/dev/ttyO4", baudrate = 9600)
GPIO.setup(Left_Motor_Direction, GPIO.OUT)
GPIO.setup(Right_Motor_Direction, GPIO.OUT)
PWM.start(Left_Motor_Pin, 0.0, 20000, 1) #Motor 1 (Left)
PWM.start(Right_Motor_Pin, 0.0, 20000, 1) #Motor 2 (Right)
p = PID(0.001,0.0,0.0) # set up PID with KP, KI, KD
p.setPoint(6500) #setpoint to middle of device.

GPIO.output(Left_Motor_Direction, GPIO.HIGH)
GPIO.output(Right_Motor_Direction, GPIO.HIGH)

var = 1
base_speed = 15

while var == 1 :
	sensorVals = getSensorVals(ser)
	linePos = getLinePos(sensorVals)
	LeftMotor_speed, RightMotor_speed = getMotorspeeds(base_speed, p, linePos)
	setMotorspeed(Left_Motor_Pin, Right_Motor_Pin, LeftMotor_speed, RightMotor_speed)
	print sensorVals
	print "Line position = ", linePos
	print 'Left MS: ', LeftMotor_speed
	print 'Right MS: ', RightMotor_speed


