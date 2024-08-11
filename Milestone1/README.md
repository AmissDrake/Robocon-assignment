# Milestone 1
## Showcase
<video src="./milestone1showcase.mp4" width='1920' height='auto' controls>
<video src="https://youtu.be/k4e2gJOnNEc" controls></video>
</video>
https://youtu.be/k4e2gJOnNEc

## Explanation
When you spin one wheel of the bot with a higher velocity than the other wheel, itll rotate in a circle. To achieve this effect, we use the following block of code:

	sim.setJointTargetVelocity(r_joint, 10/radscaled) 
	sim.setJointTargetVelocity(l_joint, -2.5)

Radscaled is a value that tunes the velocity of the right joint to vary the radius of the circle. Larger the velocity of the right joint, smaller the circle and vice versa.

The velocity of right joint MUST be greater than the velocity of left joint, orelse the bot starts spinning in the wrong direction.

Note: the velocity of the left joint is negative as its aligned opposite to the right joint in the global coordinates.

Radscaled is a value between 1 and 2.8 (anything less than 1 will make a really tiny circle and anything greater than 2.8 will leave the allotted area). However, to simplify it for the user, the program takes in upto 10 different values of radii and scales them accordingly. This block of code is supposed to do this:

	rad = int(input('enter radius level(1-10) :'))
	radscaled = 1 + (rad-1)*0.2

### And voila, you have a bot that rotates in a circle.

-Ranjit
