#!/usr/bin/env python
# license removed for brevity

# Python libs
import sys
import math 
from datetime import datetime as d

# numpy and scipy
import numpy as np


import imutils

# OpenCV
#import cv2

# Ros libraries
import roslib
import rospy

# Ros Messages
from main.msg import data
from std_msgs.msg import String


#Tuning values!
t1=15				#for the minimum distance between the robot and the ball
t2=12				#for the angle between robot and objective position
t3=12				#for the distance between robot and objective position
t4=6				#for the number of cycles used to push the robot forward
isok=False			#this boolean variable is used to say either or not I can align the robot and push the ball even if it doesn't see the ball
ballseen=False		#this boolean variable is used to say either or not I have seen the ball

ScaleFactorV1=2.0			#To scale the velocity if the robot is too fast to control it
ScaleFactorV2=2.0
ScaleFactorRPM=1.5

			
init=0
Vmax=15				#maximum velocity in x and y directions is 1 cm/s in this case
		
L1=11.7  			#everything is in cm
L2=9.3
R=4					#radius of the wheel
r=10				#radius of the ball
T1=d.now()      						#initial time
V=np.array([[0.0,0.0,0.0]]).T 			#initial velocities
robot_pos=np.array([[0.0,0.0,0.0]]).T
camera_pos=np.array([[0.0,11.0,0.0]]).T 	#frame base has x and y axes inverted,so there are y,x,th in the array
goal_pos=np.array([[0,135.0,0.0]]).T  		#assume that the robot is in the center of his half part of arena initially
goaly_limit=60
arena_limit1=180
arena_limit2=60
f=0
distance=0
a_grades=0
NewCamPos=np.array([0,0])
angle=0
ball_x=0
ball_y=0
firstcheck=True
P=np.array([[1,1,-(L1+L2)], [1,-1,(L1+L2)], [1,-1,-(L1+L2)], [1,1,(L1+L2)]])    #V[0]is Vx, V[1] is Vy, V[2] is th


#all vectors have [y_coord,x_coord,angle] except V which has [Xveocity,Yvelocity,angle]



class compute_velocities:

	def __init__(self):
		'''Initialize ros publisher, ros subscriber'''

		self.rate = rospy.Rate(1)

        # topics where we publish
		self.chat = rospy.Publisher("/chatter", String, queue_size = 1)

        # subscribed Topics
		self.sub1 = rospy.Subscriber("/DistAndAngle",data, self.angle_received,  queue_size = 1)
            

		self.sub2 = rospy.Subscriber("/arduino_msg", String, self.controller,  queue_size = 1)



	def angle_received(self, data1):		

		global distance, angle, isok, ballseen

		if not math.isnan(data1.angle): 		#I am sure that I have seen the ball (noisy detections or objects are already filtered in Ball-detection)
			ballseen=True
			isok=True
			distance = data1.distance	#I can take the distance otherwise I will rely on the one that I already have
			angle = data1.angle
		else:
			ballseen=False



	def controller(self, data2):          

		global robot_pos, camera_pos, V, f, T1, isok, ballseen, ball_x, ball_y, a_grades, NewCamPos, distance, angle, firstcheck


		rpm=list()									#initialize rpm values	(in this case are rotations/sec)
		r1=list()
		s=data2.data																			
		for i in range(0,4):
			x='RPM'+str(i)                                  		#to find the correct rpm
			pos=s.find(x)   						    	  							
			r1.append(float(s[(pos+5):(pos+10)]))					#take rpm and put the result in the global variable
		rpm=np.array([[r1[0],r1[2],r1[1],r1[3]]]).T						#velocities of motor 3 and motor 2 are inverted for the computation of V
		print("rpm:",rpm.T)

		
		rpm=rpm*(2*math.pi)									#rpm are expressed as rotations/sec; put them as rad/sec
                           			
		V=R*np.dot(np.linalg.pinv(P),rpm)					#The vector V contains Vx,Vy and th; Vx and Vy will be in cm/s while th in rad/sec
		T2=d.now()

		dt=(T2-T1).total_seconds()
		#print("dt:",dt)									#initially the RPM are all 0 so,even if dt is big, dx and dy will be 0
		T1=T2

		dx=((V[1]*math.sin(V[2]) + V[0]*math.cos(V[2]))*dt)[0]		#distance traveleld between 2 time instants.Result is inside an array	
		dy=((V[1]*math.cos(V[2]) - V[0]*math.sin(V[2]))*dt)[0]		#V is vertical vector
		dteta=(V[2]*dt)[0]

		dyfin=dy*math.cos(camera_pos[2])-dx*math.sin(camera_pos[2])	
		dxfin=dx*math.cos(camera_pos[2])+dy*math.sin(camera_pos[2])	#this comes by considering the previous rotation of the robot w.r.t the world frame

		new_pos=np.array([[dyfin,dxfin,dteta]]).T						#update the robot positon
		robot_pos=np.add(new_pos,robot_pos)
		camera_pos=np.add(new_pos,camera_pos)
		camera_rot = 180*camera_pos[2]/(math.pi)
		print("camera_rot",camera_rot)
		print("camera_pos",camera_pos.T)
		

		#If I see the ball, to have more precision do the computations only if the dist is > (t1+r) cm

		if isok==True:				#if not, rotate to seach the ball			

			if distance > (t1+r) or ballseen==False or f!=0:					#if this condition is not verified, I have to move the robot back

				if ballseen==True and f==0 and firstcheck==True:		#do this computations only if I have seen the ball and I am not pushing it to the goal

					ball_y = (camera_pos[0]+distance*math.sin(camera_pos[2])-distance*math.cos(camera_pos[2])*math.tan(angle))[0] 	#position of the ball w.r.t the camera
					ball_x = (camera_pos[1]+distance*math.cos(camera_pos[2])+distance*math.sin(camera_pos[2])*math.tan(angle))[0]
				
					distance= math.sqrt((camera_pos[1] - ball_x)**2 + (camera_pos[0] - ball_y)**2)  #assign a new distance value in case I will not detect the ball next times

					angle_grades=180*angle/(math.pi)

					#print("angle:",angle_grades)
					#print("distance:",distance)
					print("ball_y:",ball_y)
					print("ball_x:",ball_x)                 

					if not goal_pos[1] <= ball_x:  #If they are equal I can't do the computation below

						angular_coeff = -((goal_pos[0] - ball_y) / (goal_pos[1] - ball_x))[0]    #angular coefficient of the line that connects the ball with the center of the goal
						
					elif ball_y > goaly_limit or ball_y < -goaly_limit:
	   
						s=self.printing(0.0,0.0,0.0,0.0)
						self.chat.publish(s)
						print ("The ball reached the goal so stop the robot") 
						#self.rate.sleep()
						return
					
					else:	   
						s=self.printing(0.0,0.0,0.0,0.0)
						self.chat.publish(s)
						print ("The ball is out") 
						#self.rate.sleep()
						return
				
					a=np.arctan(angular_coeff)		 #angle computation. if the ball is inside the arena, the angle in which I want the robot to be can never be > or < than 90 
					a_grades = 180*a/(math.pi)
					NewCamPos = self.manage_pos(angular_coeff,a_grades,ball_x,ball_y) 		#compute the position on which the robot must go to push the ball
					firstcheck=False
					#print ("camera_rot:",camera_rot)
					print ("NewCamPos",NewCamPos)
					#print ("camera_pos",camera_pos)



				#I want the camera of the robot to be in the line that connects the ball with the goal
			
	   			#ball is out from arena, stop the robot (if I use also "=", in some conditions I can't compute the angular coefficient)
				if (ball_x > goal_pos[1] or ball_x < goal_pos[1]- arena_limit1 or ball_y > arena_limit2 or ball_y < -arena_limit2) and f==0: 
								   
					s=self.printing(0.0,0.0,0.0,0.0)
					self.chat.publish(s)
					print("Ball out of the arena,robot is stopped")
					#self.rate.sleep()
					
					
				elif ball_x > goal_pos[1] and (ball_y > goaly_limit or ball_y < -goaly_limit) and f==0:		#ball is out of the goal, stop the robot

					s=self.printing(0.0,0.0,0.0,0.0)
					self.chat.publish(s)
					print("Ball out of the goal,robot is stopped")
					#self.rate.sleep()
	
				elif ball_x > goal_pos[1] and (ball_y < goaly_limit or ball_y > -goaly_limit) and f==0:

					s=self.printing(0.0,0.0,0.0,0.0)
					self.chat.publish(s)
					print("GOAL!!!robot is stopped")
					

				elif np.linalg.norm(camera_rot-a_grades) > t2 and f==0:   #if the difference between objective angle and robot angle is more than t2 grades, align the angle
		
					print("camera_rot",camera_rot,"a_grades",a_grades)
					if camera_rot > a_grades:					   #rotate right

						obj_vel=np.array([0,0,-0.3])			   #slow rotation, check if can be done better
						vel1,vel2,vel3,vel4=(np.dot(P,obj_vel))/R
						s=self.printing(vel1,vel2,vel3,vel4)
						self.chat.publish(s)
						print("align angle rotating right")
						#self.rate.sleep()
					else:
						obj_vel=np.array([0,0,+0.3])			   
						vel1,vel2,vel3,vel4=(np.dot(P,obj_vel))/R
						s=self.printing(vel1,vel2,vel3,vel4)
						self.chat.publish(s)
						print("align angle rotating left")
						#self.rate.sleep()


				elif math.sqrt((camera_pos[1] - NewCamPos[1])**2 + (camera_pos[0] - NewCamPos[0])**2) > t3 and f==0:	#align with the direction of minimal norm distance between objective position 																															and actual one
	
					if camera_pos[1] != NewCamPos[1]:							#otherwise I can't compute angular_coeff
						angular_coeff2 = -((camera_pos[0] - NewCamPos[0]) / (camera_pos[1] - NewCamPos[1]))[0]	#compute the angular coefficient and the corresponding angle
						a2= np.arctan(angular_coeff2)					   			#angle computation
						a2_grades=180*a2/(math.pi)
						print("a2_grades",a2_grades)

						#This condition is to avoid that the robot to touch the ball while it is going in the desired position following the minimal norm distance

						#While the robot is aligning with objective line, control that it is not too near to the ball and that the robot is ahead or back w.r.t the objective position
						if math.sqrt((robot_pos[1] - ball_x)**2 + (robot_pos[0] - ball_y)**2) < (t1+r) and (camera_pos[1] - NewCamPos[1]) > (t1+r): 

							if goal_pos[0] >= NewCamPos[0]:   # move little bit left 
								obj_vel = self.transform_velocities(camera_pos[2],0,-(Vmax/ScaleFactorV1))	   
								vel1,vel2,vel3,vel4=(np.dot(P,obj_vel))/R
								s=self.printing(vel1,vel2,vel3,vel4)
								self.chat.publish(s)
								print("Ball collission, move a little bit left")
								#self.rate.sleep()
							else: 					   # move little bit right 
								obj_vel = self.transform_velocities(camera_pos[2],0,(Vmax/ScaleFactorV1)) #the camera can't have the same y		   
								vel1,vel2,vel3,vel4=(np.dot(P,obj_vel))/R
								s=self.printing(vel1,vel2,vel3,vel4)
								self.chat.publish(s)
								print("Ball collission, move a little bit right")
								#self.rate.sleep()

						elif math.sqrt((robot_pos[1] - ball_x)**2 + (robot_pos[0] - ball_y)**2) < (t1+r) and (camera_pos[1] - NewCamPos[1]) <= (t1+r):
			
						    # move little bit back
							obj_vel=self.transform_velocities(camera_pos[2],-(Vmax/ScaleFactorV1),0)			   
							vel1,vel2,vel3,vel4=(np.dot(P,obj_vel))/R
							s=self.printing(vel1,vel2,vel3,vel4)
							self.chat.publish(s)
							print("Ball collission, move a little bit back")
							#self.rate.sleep()

						else:						   #If I'm not in the condition above, I can move the robot following the minimum norm distance
					
							V=self.compute_velocities(a2_grades,camera_pos,ball_x)
							print("Align the robot with the objective line")	   	   
							vel1,vel2,vel3,vel4=(np.dot(P,V))/R
							s=self.printing(vel1,vel2,vel3,vel4)
							#print("s:",s)
							self.chat.publish(s)
							#self.rate.sleep()
					
					else:						#move very slow the robot to respect the condition camera_pos[1] != NewCamPos[1] in the next callback

						obj_vel=np.array([0,3,0])			   
						vel1,vel2,vel3,vel4=(np.dot(P,obj_vel))/R
						s=self.printing(vel1,vel2,vel3,vel4)
						self.chat.publish(s)
						print("Problem with the computation,move a little bit")
						#self.rate.sleep()
						

				else:								   #Everything is fine! Move the robot straight to push the ball to the goal

					vel1=vel2=vel3=vel4 = 2*(2*math.pi)/ScaleFactorRPM
					s=self.printing(vel1,vel2,vel3,vel4)
					self.chat.publish(s)
					print("Everything is fine,push the ball to the goal")
					f+=1
					#self.rate.sleep()	
					if f==t4:		#The flag is used because otherwise the in the next cycle, since the distance camera-ball will be < (t1+r), the robot will go back
						f=0
						isok=False
						firstcheck=True
						vel1=vel2=vel3=vel4 = -(2*(2*math.pi)/ScaleFactorRPM)
						s=self.printing(vel1,vel2,vel3,vel4)
						self.chat.publish(s)
						print("Move back")

			else:									  #In this case I have to move back because the ball is too near to the robot
		
				isok=False
				#vel1=vel2=vel3=vel4 = -(2*(2*math.pi)/ScaleFactorRPM)
				obj_vel=self.transform_velocities(angle,-(Vmax/ScaleFactorV1),0)			   
				vel1,vel2,vel3,vel4=(np.dot(P,obj_vel))/R
				s=self.printing(vel1,vel2,vel3,vel4)
				self.chat.publish(s)
				print("Ball is too near to be seen correctly, move back")
				#self.rate.sleep()


		else:								#I have to search the ball				 
			obj_vel=np.array([0,0,-0.6])					 #rotate the robot fast until I see the ball			  
			vel1,vel2,vel3,vel4=(np.dot(P,obj_vel))/R
			s=self.printing(vel1,vel2,vel3,vel4)
			self.chat.publish(s)
			print("Can't see the ball,rotate")
			#self.rate.sleep()



	def manage_pos(self,acoeff,angle,xball,yball):

		if angle>=-45 and angle<=45:		#choose a point in the line by selecting X or Y of a point and computing the remaining coordinate such that the point will be in the line 
			xfin = xball-(t1+r)
			yfin = +acoeff*(xball-xfin) + yball

		elif angle>45 and angle<90:
			yfin = yball + (t1+r)
			xfin = +(yball-yfin)*(1/acoeff) + xball

		else:									#a<45 and a>-90
			yfin = yball - (t1+r)
			xfin = +(yball-yfin)*(1/acoeff) + xball

		#print("angle,acoeff",angle,acoeff)
		#print("xball,yball",xball,yball)
		#print("xfin,yfin",xfin,yfin)
		return np.array([yfin,xfin])


			
	def printing(self,vel1,vel2,vel3,vel4):
		v=np.array([vel1,vel3,vel2,vel4])   			#vel 3 and vel 2 are inverted in the arduino controller
		s=""
		for i in range(0,4):
			
			v[i]= v[i]/(2*math.pi)										#velocities are in radiants/sec, put in rotation/sec
			if v[i]>=0:
				a="+"+"{:.1f}".format(v[i]) 		#convert the number into a string of tipe +(-)n  where n is a number with only one decimal value
			else:
				a="{:.1f}".format(v[i])

			s=s+"motor"+str(i+1)+" "+a+" " 
				
		return s
			

	def transform_velocities(self,angle,x,y):				#put the desired velocities in the robot frame

		xnew=(x-y*math.tan(angle))/(math.cos(angle)*(1+math.tan(angle)))
		ynew=(y/math.cos(angle))+(xnew*math.tan(angle))
		V=np.array([xnew,ynew,0])

		return V


	def compute_velocities(self,angle,camera,xball):
		
		if camera[1] <= xball:
			angle=angle+180			#to have the correct angle considering that it was computed with tan
	
		a=(angle*math.pi)/180		#put angle in radiants
            
		Vx= -((Vmax/ScaleFactorV2)*math.cos(a))		#compute the correct velocities to reach the objective position
		Vy= (Vmax/ScaleFactorV2)*math.sin(a)
		
		Vfin=self.transform_velocities(camera[2],Vx,Vy)

		return Vfin


	
def main(args):
	'''Initializes and cleanup ros node'''

	rospy.init_node('compute_velocities', anonymous=True)	
	cv = compute_velocities()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting down ROS compute_velocities module"
		pass

if __name__ == '__main__':
    main(sys.argv)
