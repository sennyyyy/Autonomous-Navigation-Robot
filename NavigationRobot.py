# Libraries

import picamera
import picamera.array                                   # This needs to be imported explicitly
import cv2
import numpy as np  
import time
import RPi.GPIO as GPIO

 
# GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BCM)
 
# set GPIO Pins
GPIO_Ain1 = 17
GPIO_Ain2 = 27
GPIO_Apwm = 22
GPIO_Bin1 = 5
GPIO_Bin2 = 6
GPIO_Bpwm = 13

# Set GPIO direction (IN / OUT)
GPIO.setup(GPIO_Ain1, GPIO.OUT)
GPIO.setup(GPIO_Ain2, GPIO.OUT)
GPIO.setup(GPIO_Apwm, GPIO.OUT)
GPIO.setup(GPIO_Bin1, GPIO.OUT)
GPIO.setup(GPIO_Bin2, GPIO.OUT)
GPIO.setup(GPIO_Bpwm, GPIO.OUT)

# Both motors are stopped 
GPIO.output(GPIO_Ain1, False)
GPIO.output(GPIO_Ain2, False)
GPIO.output(GPIO_Bin1, False)
GPIO.output(GPIO_Bin2, False)

# Set PWM parameters
pwm_frequency = 1000

# Create the PWM instances
pwmA = GPIO.PWM(GPIO_Apwm, pwm_frequency)
pwmB = GPIO.PWM(GPIO_Bpwm, pwm_frequency)

# Set the duty cycle (between 0 and 100)
# The duty cycle determines the speed of the wheels
pwmA.start(100)
pwmB.start(100)

def stop():
    GPIO.output(GPIO_Ain1, False)
    GPIO.output(GPIO_Ain2, False)
    GPIO.output(GPIO_Bin1, False)
    GPIO.output(GPIO_Bin2, False)
    pwmA.ChangeDutyCycle(0)               # duty cycle between 0 and 100
    pwmB.ChangeDutyCycle(0)
def backward():
    GPIO.output(GPIO_Ain1, True)
    GPIO.output(GPIO_Ain2, False)
    GPIO.output(GPIO_Bin1, True)
    GPIO.output(GPIO_Bin2, False)
    pwmA.ChangeDutyCycle(35)               # duty cycle between 0 and 100
    pwmB.ChangeDutyCycle(35)
def forward():
    GPIO.output(GPIO_Ain1, False)
    GPIO.output(GPIO_Ain2, True)
    GPIO.output(GPIO_Bin1, False)
    GPIO.output(GPIO_Bin2, True)
    pwmA.ChangeDutyCycle(35)               # duty cycle between 0 and 100
    pwmB.ChangeDutyCycle(35)
def pivotleft():
    GPIO.output(GPIO_Ain1, False)
    GPIO.output(GPIO_Ain2, False)
    GPIO.output(GPIO_Bin1, True)
    GPIO.output(GPIO_Bin2, False)
    pwmA.ChangeDutyCycle(100)               # duty cycle between 0 and 100
    pwmB.ChangeDutyCycle(100)
def pivotright():
    GPIO.output(GPIO_Ain1, True)
    GPIO.output(GPIO_Ain2, False)
    GPIO.output(GPIO_Bin1, False)
    GPIO.output(GPIO_Bin2, False)
    pwmA.ChangeDutyCycle(100)               # duty cycle between 0 and 100
    pwmB.ChangeDutyCycle(100)
def spinleft():
    GPIO.output(GPIO_Ain1, False)
    GPIO.output(GPIO_Ain2, True)
    GPIO.output(GPIO_Bin1, True)
    GPIO.output(GPIO_Bin2, False)
    pwmA.ChangeDutyCycle(24)               # duty cycle between 0 and 100
    pwmB.ChangeDutyCycle(24)
def spinright():
    GPIO.output(GPIO_Ain1, True)
    GPIO.output(GPIO_Ain2, False)
    GPIO.output(GPIO_Bin1, False)
    GPIO.output(GPIO_Bin2, True)
    pwmA.ChangeDutyCycle(24)               # duty cycle between 0 and 100
    pwmB.ChangeDutyCycle(24)
    
    
# ------------------------------------------------------------
# Define the range colors to filter; these numbers represent HSV
lowerColorThreshold = np.array([140,149,128])
upperColorThreshold = np.array([180,255,255])

# Initialize the camera
camera = picamera.PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
camera.vflip = False                            # Flip upside down or not
camera.hflip = True                             # Flip left-right or not

# Create a data structure to store a frame
rawframe = picamera.array.PiRGBArray(camera, size=(640, 480))


# ------------------------------------------------------------
# Keep track of the state
FSM1State = 0
FSM1NextState = 0

# Keep track of the timing
FSM1LastTime = time.time()

print("Press CTRL+C to end the program.\n")
print ("")

# Main code
try:
        # Allow the camera to warm up
        time.sleep(0.1)
  
        print("Program is starting")
        
        # Continuously capture frames from the camera
        # Note that the format is BGR instead of RGB because we want to use openCV later on and it only supports BGR
        for frame in camera.capture_continuous(rawframe, format = 'bgr', use_video_port = True):
            # Check the current time
            currentTime = time.time()

            # Update the state
            FSM1State = FSM1NextState
            
            # Create a numpy array representing the image
            image = frame.array
            
            # Convert for BGR to HSV color space, using openCV
            # The reason is that it is easier to extract colors in the HSV space
            # Note: the fact that we are using openCV is why the format for the camera.capture was chosen to be BGR
            image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            # Threshold the HWV image to get only colors in a range
            # The colors in range are set to white (255), while the colors not in range are set to black (0)
            ourmask = cv2.inRange(image_hsv, lowerColorThreshold, upperColorThreshold)

            # Count the number of white pixels in the mask
            numpixels = cv2.countNonZero(ourmask)
            print("Number of pixels in the color range:", numpixels)

            # Get the size of the array (the mask is of type 'numpy')
            # This should be 640 x 480 as defined earlier
            numx, numy = ourmask.shape
            
            # Select left part of image and count the number of white pixels
            ourmask_left = ourmask[ 0 : numx//7 , 0 : numy ]
            numpixels_left = cv2.countNonZero(ourmask_left)
            print("Number of pixels in the color range in the left part of the image:", numpixels_left)
            
            # Select center part of the image and count the number of white pixels
            ourmask_center = ourmask[ numx//7 : 6*numx//7 , 0 : numy ]
            numpixels_center = cv2.countNonZero(ourmask_center)
            print("Number of pixels in the color range in the center part of the image:", numpixels_center)
            
            # Select right part of the image and count the number of white pixels
            ourmask_right = ourmask[ 6*numx//7 : numx , 0 : numy ]
            numpixels_right = cv2.countNonZero(ourmask_right)
            print("Number of pixels in the color range in the right part of the image:", numpixels_right)
            
             # Bitwise AND of the mask and the original image
            image_masked = cv2.bitwise_and(image, image, mask = ourmask)
            
            # Show the frames
            # The waitKey command is needed to force openCV to show the image
            cv2.imshow("Frame", image)
            cv2.imshow("Result", image_masked)
            cv2.waitKey(1)

            #state 0: base
            if(FSM1State==0):
                #center is greater, go forward
                if(numpixels_center>numpixels_left and numpixels_center>numpixels_right):
                    print("Going forward")
                    forward()
                    FSM1NextState = 1
                #left greater, go left
                elif(numpixels_left>numpixels_center and numpixels_left>numpixels_right):
                    print("Going left")
                    spinleft()
                    FSM1NextState = 2
                #right greater, go right
                elif(numpixels_right>numpixels_center and numpixels_right>numpixels_left):
                    print("Going right")
                    spinright()
                    FSM1NextState = 3
                else:
                    spinleft()
                    time.sleep(1)
                    #center is greater, go forward
                    if(numpixels_center>numpixels_left and numpixels_center>numpixels_right):
                        print("Going forward")
                        forward()
                        FSM1NextState = 1
                    #left greater, go left
                    elif(numpixels_left>numpixels_center and numpixels_left>numpixels_right):
                        print("Going left")
                        spinleft()
                        FSM1NextState = 2
                    #right greater, go right
                    elif(numpixels_right>numpixels_center and numpixels_right>numpixels_left):
                        print("Going right")
                        spinright()
                        FSM1NextState = 3
                        
                    spinright()
                    time.sleep(1)
                    #center is greater, go forward
                    if(numpixels_center>numpixels_left and numpixels_center>numpixels_right):
                        print("Going forward")
                        forward()
                        FSM1NextState = 1
                    #left greater, go left
                    elif(numpixels_left>numpixels_center and numpixels_left>numpixels_right):
                        print("Going left")
                        spinleft()
                        FSM1NextState = 2
                    #right greater, go right
                    elif(numpixels_right>numpixels_center and numpixels_right>numpixels_left):
                        print("Going right")
                        spinright()
                        FSM1NextState = 3
                    
                    FSM1NextState=0
                    stop()
                    
                
            #state 1: forward
            if(FSM1State==1):
                #left greater, go left
                if(numpixels_left>numpixels_center and numpixels_left>numpixels_right):
                    print("Going left")
                    spinleft()
                    FSM1NextState = 2
                #right greater, go right
                elif(numpixels_right>numpixels_center and numpixels_right>numpixels_left):
                    print("Going right")
                    spinright()
                    FSM1NextState = 3
                #no stickies detected, stop!
                elif(numpixels_right==0 and numpixels_center==0 and numpixels_left==0):
                    print("No stickies detected, stopping")
                    FSM1NextState = 0
                else:
                    FSM1NextState = 1
                
            #state 2: left
            if(FSM1State==2):
                #center is greater, go forward
                if(numpixels_center>numpixels_left and numpixels_center>numpixels_right):
                    print("Going forward")
                    forward()
                    FSM1NextState = 1
                #right greater, go right
                elif(numpixels_right>numpixels_center and numpixels_right>numpixels_left):
                    print("Going right")
                    spinright()
                    FSM1NextState = 3
                #no stickies detected, stop!
                elif(numpixels_right==0 and numpixels_center==0 and numpixels_left==0):
                    print("No stickies detected, stopping")
                    FSM1NextState = 0
                else:
                    FSM1NextState=2
                    
            #state 3: right
            if(FSM1State==3):
                #center is greater, go forward
                if(numpixels_center>numpixels_left and numpixels_center>numpixels_right):
                    print("Going forward")
                    forward()
                    FSM1NextState = 1
                #left greater, go left
                elif(numpixels_left>numpixels_center and numpixels_left>numpixels_right):
                    print("Going left")
                    spinleft()
                    FSM1NextState = 2
                #no stickies detected, stop!
                elif(numpixels_right==0 and numpixels_center==0 and numpixels_left==0):
                    print("No stickies detected, stopping")
                    FSM1NextState = 0
                else:
                    FSM1NextState==3
            
            # Clear the stream in preparation for the next frame
            rawframe.truncate(0)
              
            
# Quit the program when the user presses CTRL + C
except KeyboardInterrupt:
        # Clean up the resources
        pwmA.stop()
        pwmB.stop()
        GPIO.cleanup()
        cv2.destroyAllWindows()
        camera.close() 

        

 

