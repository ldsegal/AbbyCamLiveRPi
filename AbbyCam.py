"""
AbbyCam.py

Motion detection & cat recognition to trigger a youtube livestream
"""

# Imports
from gpiozero import MotionSensor # PIR sensor library 
import cv2                        # Cat image classification
import picamera                   # Camera functions
import subprocess                 # Subprocess for livestream pipe
import numpy as np                # Use numpy for drawing overlay rectangles
import time

# Constants
url = "rtmp://a.rtmp.youtube.com/live2/" # YouTube livestream url
key = open("StreamKey.txt", "r").read()  # YouTube livestream key
catSearchTimeout = 5 # Cat search time after motion detected (seconds)
catLostTimeout = 30 # Cat search time after target lost (seconds)

# Command to start the livestream
streamCmd = "ffmpeg -re -ar 44100 -ac 2 -acodec pcm_s16le -f s16le -ac 2 -i /dev/zero -f h264 -i - -vcodec copy -acodec aac -ab 128k -g 50 -strict experimental -f flv " + url + key

# Hardware pin setup
pir = MotionSensor(14)

# Initializations
classifier = cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_frontalcatface.xml") # Load cat image classifier
camera = picamera.PiCamera(resolution=(800,480), framerate=25) # Init camera
camera.hflip = True # Flip camera
camera.vflip = True


# Cat face detection funtion
def catSearch():
	camera.capture("temp.jpg") # Take an image
	img = cv2.imread("temp.jpg") # Open the image in the cv2 library
	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) # Convert to grayscale
	faces = classifier.detectMultiScale(gray, 1.04, 3) # search for cat faces
	return faces # return list of face locations found

# Mange camera overlay
def updateOverlay(faces):
	
	array = np.zeros((480, 800, 3), dtype=np.uint8)
	for (x,y,w,h) in faces:
		array[y:y+h, x:x+w, :] = 0xff
		
	#overlay = camera.add_overlay(np.asarray(array), layer=3, alpha=64)



# Main loop
while True:
	# Wait for movement
	pir.wait_for_motion()
	now = time.time()
	if __debug__:
		print("Motion Detected")
		camera.start_preview()
	
	# Look for a cat until timeout
	catFound = False
	while time.time() < now + catSearchTimeout:
		faces = catSearch() # Check for cat faces
		updateOverlay(faces)
		if len(faces) > 0:
			# Cat found
			catFound = True
			if __debug__:
				print("Cat found! Going live...")
			break
	
	# Start stream
	if catFound:
		streamPipe = subprocess.Popen(streamCmd, shell=True, stdin=subprocess.PIPE) # Create livestream pipe
		camera.start_recording(streamPipe.stdin, format="h264", bitrate=2000000) # Start rolling
		
		# Track cat until timeout
		lastDetection = time.time()
		while time.time() < lastDetection + catLostTimeout:
			faces = catSearch()
			updateOverlay(faces)
			if len(faces) > 0 or pir.motion_detected:
				lastDetection = time.time()
				
		# End stream
		camera.stop_recording()
		streamPipe.terminate()
		if __debug__:
			print("Ending stream")
	
	if __debug__:
		print("No activity")
		camera.stop_preview()
		
