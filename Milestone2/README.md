# Milestone 2
## Showcase
<!-- <video src="https://youtu.be/PVvkhUi4fIc" controls></video> -->
</video>
https://youtu.be/PVvkhUi4fIc

## Frames of Reference
### Location:
The top left of the vision sensor feed is the origin, x increasing as we go left to right and y increasing as we go top to bottom.
### Alignment:
When the bot is aligned along a line parallel to the y-axis, its alignment is 0, which increases upto 360 as it rotates in the clockwise direction. For example, when the bot is aligned along +ve x-axis, its alignment is 90 degrees.
## Explanation
First we get the vision sensor from coppeliasim and process the video feed from that to find the AruCo markers.

The processing is done by openCV using the following lines of code and the arucodet function

        img, res = sim.getVisionSensorImg(sensor1Handle)
		img1 = np.frombuffer(img, dtype = np.uint8)
		img1 = img1.reshape((1024, 1024, 3))
		img1 = cv2.cvtColor(img1, cv2.COLOR_RGB2BGR)

Once we have the coordinates, we feed them into the locandalign function which spits out the current location and alignment of the bot.

Now we can use openCV's imshow() and puttext() capabilities to display the location and coordinates over the video feed.


-Ranjit