#! /usr/bin/env python
# Ros libraries
import roslib
import rospy
from std_msgs.msg import String
#from bridge_example.msg import motors, motor

import serial
import time
import threading

startMarker = '<'
endMarker = '>'
dataStarted = False
dataBuf = ""
messageComplete = False
go=True
details = None
count = 0
#prevTime = time.time()
vel = 100
inc = 10
switch = 0



class Message_manager:

	def __init__(self):
		'''Initialize ros publisher and ros subscriber'''
		# publish to topic "arduino_msg"
		self.pub = rospy.Publisher('arduino_msg', String, queue_size=1)
		#self.rate = rospy.Rate(5) 
		
		# subscribed Topic
		rospy.Subscriber("chatter", String, self.callback)


	# ====================    Callback
	def callback(self,data): #if there is a data, start the callback funtion
		#rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
		global details
		details = String() #details will be a string received in the chatter topic
		details.data = data.data #put the data in details.data which will contain a string
		g = details.data #raw_input("Enter the velocity (rps) (between -2.0 and 2.0). Please enter the velocity in the format +(-)X.X (eg. +2.0 or -1.0 or +1.5).
		#print(g)
		self.sendToArduino(g)
		print("message sent to arduino")
		#if(g=="stop"):
			#sendToArduino("+0.0")
		#for n in range(0,1000000):  #I finished to send, now I check If I can receive from arduino. I receive until a new callback is called, this is why the loop is so big
		while(b==0):
			b=self.rec()
    ## ==================


	def rec(self):  #from arduino I will receive the message info for the RPM and PWM states of the motors
		arduinoReply = self.recvLikeArduino()  #call function for receiving a string (one character at time)
		if not (arduinoReply == 'XXX'):  #if there is a reply which is not XXX, print the time and the string received,if it is XXX I have not completely received a message so do not anything

			print ("Time %s  Reply %s" % (time.time(), arduinoReply))
			self.pub.publish(arduinoReply)
			return 1
			#self.rate.sleep()
		else:
			return 0


	def recvLikeArduino(self):	
		global startMarker, endMarker, serialPort, dataStarted, dataBuf, messageComplete
		
		if serialPort.inWaiting() > 0 and messageComplete == False: #inWaiting return the number of bytes in the receive buffer
		    x = serialPort.read().decode("utf-8") # decode needed for Python3; if the message is not complete, read (1 character at time)
		    #print("x:",x)

		    if dataStarted == True: #If data is started, I'm not in the startMarker position; if I'm in that position, data has to start, so dataStarted=False 
		        if x != endMarker: #if the data started but i'm not in the end,put the data in the buffer
		            dataBuf = dataBuf + x
		        else:
		            dataStarted = False #else I'm in the end, I have not to put something in the buffer, message is completed and new data has not started yet
		            messageComplete = True
		    elif x == startMarker: #if I'm in the start, initialize the buffer 
		        dataBuf = ''
		        dataStarted = True

		#I can be in this condition only if I had before dataStarted=true, I found the end marker so I put dataStarted=False and messageComplete=True	
		if (messageComplete == True): #if the message is completed put messageComplete=false for the next message and return the message
		    messageComplete = False 
		    return dataBuf
		else:
		    return "XXX"  #the message is not completed, return XXX and restart again until the message is not completed (XXX is a message used to say that
						  #I have not finished; I will check that in waitForArduino and rec() 
		      #Or message is not completed and I'm not receiving something at the moment (inwaiting is not  >0) 


	def waitForArduino(self):
		# wait until the Arduino sends 'Arduino is ready' - allows time for Arduino reset
		# it also ensures that any bytes left over from a previous message are discarded

		print("Waiting for Arduino to reset")

		msg = ""
		while msg.find("Arduino is ready") == -1: 
			msg = self.recvLikeArduino()  #receive the message and print if it is completed
			if not (msg == 'XXX'): #arduino is ready but maybe the message is not completed yet
				print(msg)
		self.pub.publish("RPM0:0.00  PWM:0.00     RPM1:0.00  PWM:0.00     RPM2:0.00  PWM:0.00     RPM3:0.00  PWM:0.00") #used to initialize the controller


	def setupSerial(self,baudRate, serialPortName):

		global  serialPort
		serialPort = serial.Serial(port= serialPortName, baudrate = baudRate, timeout=0, rtscts=True) #setup the serial
		print(serialPort.name)
		print("Serial port " + serialPortName + " opened  Baudrate " + str(baudRate))
		self.waitForArduino() #wait for a message of arduino, the message will be "Arduino is ready"


	def sendToArduino(self,stringToSend): #send a message to arduino

		# this adds the start and end markers before sending
		global startMarker, endMarker, serialPort

		stringWithMarkers = (startMarker) #the message is like:  <StringToSend> 
		stringWithMarkers += stringToSend
		stringWithMarkers += (endMarker)
		serialPort.write(stringWithMarkers.encode('utf-8')) # encode needed for Python3
		print(stringToSend)




if __name__ == '__main__':  

	rospy.init_node('message_manager', anonymous=True)
	ms = Message_manager()
	ms.setupSerial(115200, "/dev/ttyACM0")
	try:
		rospy.spin()
	except rospy.ROSInterruptException:
		print "Shutting down message manager"
		pass

#So, the main calls the listener which initialize the node and subscribes to "chatter" and then call the callback. Before this, some variables are initialized: empty dataBuf, start and end,markers, messageComplete,details, go, vel, inc, switch, count and setupSerial (which setup the serial with arduino, print it and call waitForArduino). This functon is used to wait the reset of arduino,it prints a string, it inizialize msg and it calls the function recvLikeArduino to receive something from arduino; if the message is not "XXX", it prints it. When it becomes "Arduino is ready", the program can continues.I will receive a string from chatter with velocities, call the callback and put it in "g" variable. The callback will send it to arduino and then receive again using rec() this time. If the reply of arduino is not "XXX" then I will print the reply and the message.
		
		
