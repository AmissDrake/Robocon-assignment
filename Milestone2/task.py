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

############################## GLOBAL VARIABLES ######################################



############################ USER DEFINED FUNCTIONS ##################################
def arucodet(img):
	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	arucoDict = aruco.getPredefinedDictionary(aruco.DICT_4X4_1000)
	arucoParam = aruco.DetectorParameters()
	detector = aruco.ArucoDetector(arucoDict, arucoParam)
	bbox, ids, rejects = detector.detectMarkers(gray)
	return bbox

def locandalign(bbox):
	corner_coords=np.array(bbox)
	print(corner_coords)
	coords = [(corner_coords[0][0][0][0]+ corner_coords[0][0][1][0] + corner_coords[0][0][2][0] + corner_coords[0][0][3][0])/4, 
		   (corner_coords[0][0][0][1] + corner_coords[0][0][1][1] + corner_coords[0][0][2][1]+corner_coords[0][0][3][1])/4]
	a,b,c,d = corner_coords[0][0][0], corner_coords[0][0][1], corner_coords[0][0][2], corner_coords[0][0][3]
	angle = np.arctan2(d[1]-c[1], d[0]-c[0])
	alignment = angle*180/np.pi
	return coords,alignment

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


	#Taking radius from user (radius is not exact value, its proportional)
	rad = int(input('enter radius level(1-10) :'))
	radscaled = 1 + (rad-1)*0.2
	time.sleep(2)


	#Actual simulation part
	while sim.getSimulationState() != sim.simulation_stopped:
		img, res = sim.getVisionSensorImg(sensor1Handle)
		img1 = np.frombuffer(img, dtype = np.uint8)
		img1 = img1.reshape((1024, 1024, 3))
		img1 = cv2.cvtColor(img1, cv2.COLOR_RGB2BGR)

		#making it revolve according to radius level
		sim.setJointTargetVelocity(r_joint, 10/radscaled) 
		sim.setJointTargetVelocity(l_joint, -2.5)
		bbox = arucodet(img1)
		coords, alignment = locandalign(bbox)
		print(coords)
		print(alignment)

		cv2.namedWindow('Image', cv2.WINDOW_KEEPRATIO)
		cv2.putText(img1,f'Location: {coords}',[40,60],cv2.FONT_HERSHEY_SIMPLEX,1,[0,0,0],8)
		cv2.putText(img1,f'Location: {coords}',[40,60],cv2.FONT_HERSHEY_SIMPLEX,1,[0,255,0],2)
		cv2.putText(img1,f'Alignment: {round(alignment,2)}',[40,100],cv2.FONT_HERSHEY_SIMPLEX,1,[0,0,0],8)
		cv2.putText(img1,f'Alignment: {round(alignment,2)}',[40,100],cv2.FONT_HERSHEY_SIMPLEX,1,[0,255,0],2)
		cv2.imshow('Image', img1)
		cv2.resizeWindow('Image', 768, 768)
		
		if cv2.waitKey(1) & 0xFF == ord('q'): break
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