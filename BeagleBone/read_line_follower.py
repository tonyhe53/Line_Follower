import Adafruit_BBIO.UART as UART
import serial
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
	

UART.setup("UART4")

ser = serial.Serial(port = "/dev/ttyO4", baudrate = 9600)

while True :
	sensorVals = getSensorVals(ser)
	linePos = getLinePos(sensorVals)
	
	print sensorVals
	print "Line position = ", linePos
	time.sleep(1)
	