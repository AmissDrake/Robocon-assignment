# Milestone 3
## Showcase
<!-- <video src="https://youtu.be/dOrhktpeZAE" controls></video> -->
</video>
https://youtu.be/dOrhktpeZAE

Could use a little more tweaking in the numbers department, but it works.

## Explanation
Building on what we have done so far, we control the bot using the following control loop:
```
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
```
Main points of interest here are conditions A and B. 
From a neutral state, when the bot is randomly place, it will first try to align itself in the direction of the waypoint because of condition A.
Then, it will try to move in the direction of the waypoint because of condition B. 

The function for alignment is:
```
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
```

The function for moving the bot is:
```
def movebot(coords, Target, r_joint, l_joint):
	print("moving")
	error = np.sqrt((Target[0] - coords[0])**2+(Target[1]-coords[1])**2)
	vel = PID(error)
	sim.setJointTargetVelocity(r_joint, vel/50)
	sim.setJointTargetVelocity(l_joint, -vel/50)
	return
```
Both these functions call a PID function of which I could only get the P working:
```
def PID(error):
	error = np.abs(error)
	Kp = 1
	# Ki = 2
	# Kd = 1
	velocity = Kp*error
	return velocity
```

Statements B,C and D deal with stopping the bot once its reached the waypoint, and moving to the next waypoint.

The whole control loop is set in a while loop which runs when simulation is true, and the while loop is set in a for loop which picks a single waypoint from a predefined, hardcoded set of waypoints.

When statements B,C or D break the loop, it breaks the while loop and starts it again with the next waypoint from the for loop.


-Ranjit