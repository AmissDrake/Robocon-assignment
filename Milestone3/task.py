'''
*****************************************************************************************
*
* All the functions in this file are used to control the robot in the CoppeliaSim
* simulation via APIs
*
*****************************************************************************************
'''

import  sys
import traceback
import time
import cv2
import cv2.aruco as aruco
import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import math

############################## GLOBAL VARIABLES ######################################
Waypoints = [[204, 612],[612, 612],[612, 918],[920, 918],[920, 300],[100, 302],[100, 96],[920, 96]]
coords = [0,0]


############################ USER DEFINED FUNCTIONS ##################################
def PID(error):
	error = np.abs(error)
	Kp = 1
	# Ki = 2
	# Kd = 1
	velocity = Kp*error
	return velocity

def arucodet(img):
	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	arucoDict = aruco.getPredefinedDictionary(aruco.DICT_4X4_1000)
	arucoParam = aruco.DetectorParameters()
	detector = aruco.ArucoDetector(arucoDict, arucoParam)
	bbox, ids, rejects = detector.detectMarkers(gray)
	return bbox

def locandalign(Target,bbox):
	corner_coords=np.array(bbox)
	# print(corner_coords)
	if len(corner_coords) == 0:
				print("No corners found")
				return
	#getting the coordinates of the center of the bot
	coords = [(corner_coords[0][0][0][0]+ corner_coords[0][0][1][0] + corner_coords[0][0][2][0] + corner_coords[0][0][3][0])/4, 
		   (corner_coords[0][0][0][1] + corner_coords[0][0][1][1] + corner_coords[0][0][2][1]+corner_coords[0][0][3][1])/4]
	#getting the coordinates of the corners of the bots
	a,b,c,d = corner_coords[0][0][0], corner_coords[0][0][1], corner_coords[0][0][2], corner_coords[0][0][3]
	# print(coords)

	#getting the alignment of the bot
	angle = np.arctan2(d[1]-c[1], d[0]-c[0])
	alignment = angle*180/np.pi
	# print('A',alignment)

	#getting the target alignment
	tangle = np.arctan2(Target[1]-coords[1],Target[0]-coords[0])
	Targetalignment = tangle*180/np.pi + 90
	# print('TA',Targetalignment)

	return coords,alignment,Targetalignment

def alignbot(alignment, Targetalignment, r_joint, l_joint,dist):
	print("aligning")
	vel = PID(alignment-Targetalignment)
	if dist<750:
		if Targetalignment-alignment>0:
			sim.setJointTargetVelocity(r_joint, dist*vel/5000)
			sim.setJointTargetVelocity(l_joint, dist*vel/5000)
		elif Targetalignment-alignment<0:
			sim.setJointTargetVelocity(r_joint, -dist*vel/5000)
			sim.setJointTargetVelocity(l_joint, -dist*vel/5000)
		else: return
	else:
		if Targetalignment-alignment>0:
			sim.setJointTargetVelocity(r_joint, vel/50)
			sim.setJointTargetVelocity(l_joint, vel/50)
		elif Targetalignment-alignment<0:
			sim.setJointTargetVelocity(r_joint, -vel/50)
			sim.setJointTargetVelocity(l_joint, -vel/50)
		else: return
	return

def movebot(coords, Target, r_joint, l_joint):
	print("moving")
	error = np.sqrt((Target[0] - coords[0])**2+(Target[1]-coords[1])**2)
	vel = PID(error)
	sim.setJointTargetVelocity(r_joint, vel/50)
	sim.setJointTargetVelocity(l_joint, -vel/50)
	return
################################ MAIN FUNCTION #######################################

def simulator(sim):
	"""
	Purpose:
	---
	This function should implement the control logic for the given problem statement
	You are required to actuate the rotary joints of the robot in this function, such that
	it does the required tasks.

	Input Arguments:
	---
	`sim`    :   [ object ]
		ZeroMQ RemoteAPI object

	Returns:
	---
	None

	Example call:
	---
	simulator(sim)
	"""
	
	#### YOUR CODE HERE ####
	#Getting the bot and the joints
	robot = sim.getObject('/crn_bot')
	r_joint = sim.getObject('/crn_bot/joint_r')
	l_joint = sim.getObject('/crn_bot/joint_l')
	sensor1Handle=sim.getObjectHandle('/vision_sensor')

	#Actual simulation part
	for Target in Waypoints:
		while sim.getSimulationState() != sim.simulation_stopped:
			# i=0
			# print(i)
			# Target = Waypoints[i]
			#getting the image from the vision sensor and setting it up
			img, res = sim.getVisionSensorImg(sensor1Handle)
			img1 = np.frombuffer(img, dtype = np.uint8)
			img1 = img1.reshape((1024, 1024, 3))
			img1 = cv2.cvtColor(img1, cv2.COLOR_RGB2BGR)

			#getting the location and alignment of the bot
			try:
				bbox = arucodet(img1)
				# print(bbox)
				coords, alignment, Targetalignment = locandalign(Target,bbox)
			except:
				continue

			#printing the image along with the text
			cv2.namedWindow('Image', cv2.WINDOW_KEEPRATIO)
			cv2.putText(img1,f'Location: {coords}',[40,60],cv2.FONT_HERSHEY_SIMPLEX,1,[0,0,0],8)
			cv2.putText(img1,f'Location: {coords}',[40,60],cv2.FONT_HERSHEY_SIMPLEX,1,[0,255,0],2)
			cv2.putText(img1,f'Alignment: {round(alignment,2)}',[40,100],cv2.FONT_HERSHEY_SIMPLEX,1,[0,0,0],8)
			cv2.putText(img1,f'Alignment: {round(alignment,2)}',[40,100],cv2.FONT_HERSHEY_SIMPLEX,1,[0,255,0],2)
			cv2.imshow('Image', img1)
			cv2.resizeWindow('Image', 768, 768)
			cv2.waitKey(1)

			#Setting Tolerances
			botvel = sim.getObjectVelocity(robot)
			maxbotvel = max(botvel[0])
			print('coords',coords,'target',Target)
			LocOk=np.isclose(coords,Target,rtol=0.05,atol=0.05)
			LocOk=all(LocOk)
			dist = np.sqrt((Target[0] - coords[0])**2+(Target[1]-coords[1])**2)
			print(dist)
			print('T', Targetalignment, 'A', alignment)
			AlignOk=np.isclose(alignment,Targetalignment,rtol=0.1,atol=12)

			# if dist<200 and dist>=100:
			# 	AlignOk=np.isclose(alignment,Targetalignment,rtol=0.05*dist,atol=0.1*dist)
			# elif dist<100 and dist>=50:
			# 	AlignOk=np.isclose(alignment,Targetalignment,rtol=0.15,atol=3)
			# elif dist>=200 and dist<=500:
			# 	AlignOk=np.isclose(alignment,Targetalignment,rtol=0.15,atol=5)
			# else:
			# 	AlignOk=np.isclose(alignment,Targetalignment,rtol=0.3,atol=5)
			# print("Align", AlignOk, "Loc", LocOk)

			###MAIN CONTROL LOOP
			if LocOk ==0 and AlignOk ==0:
				alignbot(alignment,Targetalignment,r_joint,l_joint,dist)
				print('A')
			elif LocOk==0 and AlignOk==1:	
				movebot(coords,Target,r_joint,l_joint)
			elif LocOk==1 and math.isclose(maxbotvel,0.1,rel_tol=1e-1,abs_tol=1e-1)==0:
				sim.setJointTargetVelocity(r_joint, 0)
				sim.setJointTargetVelocity(l_joint, 0)
				print('B')
			elif LocOk==1 and AlignOk==0:
				print('C')
				break
			elif LocOk==1 and math.isclose(maxbotvel,0.1,rel_tol=1e-1,abs_tol=1e-1)==1:
				print('D')
				break
			else:
				print('Wtf is the bot on?')
				continue

			if cv2.waitKey(1) & 0xFF == ord('q'): sim.stopSimulation;break 
	cv2.destroyAllWindows()	
	return None
	
######### YOU ARE NOT ALLOWED TO MAKE CHANGES TO THE MAIN CODE BELOW #########

if __name__ == "__main__":
	client = RemoteAPIClient()
	sim = client.getObject('sim')

	try:

		## Start the simulation using ZeroMQ RemoteAPI
		try:
			return_code = sim.startSimulation()
			if sim.getSimulationState() != sim.simulation_stopped:
				print('\nSimulation started correctly in CoppeliaSim.')
			else:
				print('\nSimulation could not be started correctly in CoppeliaSim.')
				sys.exit()

		except Exception:
			print('\n[ERROR] Simulation could not be started !!')
			traceback.print_exc(file=sys.stdout)
			sys.exit()

		## Runs the robot navigation logic written by participants
		try:
			simulator(sim)
			time.sleep(5)

		except Exception:
			print('\n[ERROR] Your simulator function throwed an Exception, kindly debug your code!')
			print('Stop the CoppeliaSim simulation manually if required.\n')
			traceback.print_exc(file=sys.stdout)
			print()
			sys.exit()

		
		## Stop the simulation using ZeroMQ RemoteAPI
		try:
			return_code = sim.stopSimulation()
			time.sleep(0.5)
			if sim.getSimulationState() == sim.simulation_stopped:
				print('\nSimulation stopped correctly in CoppeliaSim.')
			else:
				print('\nSimulation could not be stopped correctly in CoppeliaSim.')
				sys.exit()

		except Exception:
			print('\n[ERROR] Simulation could not be stopped !!')
			traceback.print_exc(file=sys.stdout)
			sys.exit()

	except KeyboardInterrupt:
		## Stop the simulation using ZeroMQ RemoteAPI
		return_code = sim.stopSimulation()
		time.sleep(0.5)
		if sim.getSimulationState() == sim.simulation_stopped:
			print('\nSimulation interrupted by user in CoppeliaSim.')
		else:
			print('\nSimulation could not be interrupted. Stop the simulation manually .')
			sys.exit()