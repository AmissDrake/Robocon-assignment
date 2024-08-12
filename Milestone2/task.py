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
import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

############################## GLOBAL VARIABLES ######################################



############################ USER DEFINED FUNCTIONS ##################################



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
	while sim.getSimulationState() != sim.simulation_stopped:
		img, res = sim.getVisionSensorImg(sensor1Handle)
		# print(img)
		img1 = np.frombuffer(img, dtype = np.uint8)
		print(img1)
		img1 = img1.reshape((1024, 1024, 3))
		cv2.imshow('Image', img1)

	# #Taking radius from user (radius is not exact value, its proportional)
	# rad = int(input('enter radius level(1-10) :'))
	# radscaled = 1 + (rad-1)*0.2
	# time.sleep(2)

	# while sim.getSimulationState() != sim.simulation_stopped:
	# 	#making it revolve according to radius level
	
	# 	sim.setJointTargetVelocity(r_joint, 10/radscaled) 
	# 	sim.setJointTargetVelocity(l_joint, -2.5)
	# 	image=sim.getVisionSensorImage(sensor1Handle)
	# 	img = np.array(image)*255
	# 	img = img.astype(np.uint8)
	# 	img.resize([resolution[0],resolution[1],3])
	# 	img = cv2.flip(img[...,::-1],0)
	# 	cv2.imshow('image',img)
	# 	if cv2.waitKey(1) & 0xFF == ord('q'): break

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