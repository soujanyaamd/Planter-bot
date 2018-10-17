''' 
* Author List : Akshatha.S, Krupa Sindhu S, Pragna G Shastry, Soujanya Avadhani M.D
* Filename: final_code.py
* Theme: Planter Bot
* Functions: main(), process(), IP(), ZI(), rgb(), endrun(), cntshape(), readcsv(), forward(), right(), left(), right1(), left1(), stop(), overlay(), blend_transparent()
* Global Variables: filename, zi_areaL, zi_areaU, path_areaL, path_areaU, shed_areaL, shed_areaU, zi_count, ip_flag, shed_flag, cm_count, cm_shape, cm_color, lmspeed, rmspeed, rw, rh,cx,zi_read_once, background_img
'''

import cv2
import numpy as np
from picamera import PiCamera
from picamera.array import PiRGBArray
from time import sleep
import RPi.GPIO as GPIO

# the seedlings corresponding to a particular CM are to be read from the file below.
filename = 'Input Table.csv'

# zi_areaL, zi_areaU: lower, upper thresholds for detecting ZI.
# path_areaL, path_areaU: lower, upper thresholds for following the path. 
# shed_areaL, shed_areaU: lower, upper thresholds for detecting shed.
zi_areaL,zi_areaU,path_areaL,path_areaU,shed_areaL,shed_areaU=17000,22000,4000,17000,40000,50000 

# zi_count: number of colour markers detected
# ipflag, shed_flag: to set status of IP and Shed.
zi_count,ip_flag,shed_flag=0,0,0

# cm_color, cm_shape, cm_count: lists for storing color,shape and count of color markers
cm_color,cm_shape,cm_count=[],[],[]

# centroid of frame initialised globally to be used later for differential turns
cx=0
# zi_read_once: flag to prevent multiple ZI calls for same ZI  
zi_read_once=0

# RPi settings
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

# declaration of motor pins
# l1, l2: GPIO Pins of RPi to which M+ and M- Pins of left motor are connected.
# r1, r2: GPIO Pins of RPi to which M+ and M- Pins of right motor are connected.
# le: GPIO Pin of RPi to which enable pin of left motor is connected.
# re: GPIO Pin of RPi to which enable pin of right motor is connected.
l1,l2,le,r1,r2,re=33,35,37,36,38,40
GPIO.setup(l1,GPIO.OUT)
GPIO.setup(l2,GPIO.OUT)
GPIO.setup(le,GPIO.OUT)
GPIO.setup(r1,GPIO.OUT)
GPIO.setup(r2,GPIO.OUT)
GPIO.setup(re,GPIO.OUT)

# Applying PWM on M+ and M- pins of both the motors
pl1 = GPIO.PWM(l1, 100)
pr1 = GPIO.PWM(r1, 100)
pl2 = GPIO.PWM(l2, 100)
pr2 = GPIO.PWM(r2, 100)

# set the dutycycle to zero initially
pl1.start(0)
pr1.start(0)
pl2.start(0)
pr2.start(0)

# Declaring the pins for RGB LED.
R = 11
G = 13
B = 15
GPIO.setup(R,GPIO.OUT)
GPIO.setup(G,GPIO.OUT)
GPIO.setup(B,GPIO.OUT)

# Setting the speeds for the left and right motors.
# lmspeed, rmspeed: duty cycle of left motor, duty cycle of right motor.
# In the entire code slightly different duty cycles are given to both the motors because they vary slightly in their speed of operation
lmspeed, rmspeed = 65,61

# rw, rh: width of resolution, height of resolution.
rw,rh=320,240

# background plantation image
background_img = cv2.imread("/home/pi/Plantation/Plantation.png")

# PiCam settings
# initialize a PiCam object
cam = PiCamera()
# set the resolution of the video to be captured
cam.resolution = (rw,rh)
# set the framerate
cam.framerate = 70
# create a RGB Array of PiCam storage type
raw_cap = PiRGBArray(cam,(rw,rh))


'''
* Function Name: main()
* Input: nil
* Output: nil
* Logic: It calls process() which handles image processing while PB is traversing the arena
	 On the completion of the run, the endrun() is called to blink the RGB LED as specified.
* Example Call: main()
'''
def main():
    # set the enable pins of both the motors
    GPIO.output(le,GPIO.HIGH)
    GPIO.output(re,GPIO.HIGH)
    
    ''' #When our bot is placed in R3, it spans till Q3. therefore, we have'nt used this part in the demoand bonus video. 
    # To check nursery before starting, do uncomment this piece of the code.    
    cam.start_preview()
    sleep(0.1)# camera warmup time
    cam.capture("start.jpg")# capture first frame
    cam.stop_preview()
    img=cv2.imread('start.jpg')
    u=75 # u:upper limit value for masking
    l0=np.array([0,0,0]) # lower range for masking
    u0=np.array([u,u,u]) # upper range for masking 
    mask=cv2.inRange(img,l0, u0) # masking done in order to avoid shadows as much as possible
    ret2,thresh = cv2.threshold(mask,127,255,0) # thresholding the masked frame
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) # detecting contours
    for cnt in contours:
           if cnt is not None:
              area = cv2.contourArea(cnt) # find the area of contour
              
              # checking if area satisfies minimum nursery area condition. The area must be tested depending on position of bot and PiCam placed on bot. 
              if (area>=10000): # call process(), stop(), endrun(), print statements within the if condition'''
    # call process() to start capturing video and processing frames
    process()   

    # stop the PB after processing gets completed and return after shed_flag is set  
    stop()

    # call endrun() to blink RGB LED
    endrun(cm_color,zi_count)

    # print the final list of CMs detected in each zone
    print cm_color,cm_shape,cm_count

    # make the enable pins of both the motors low after the whole arena is traversed
    GPIO.output(le,GPIO.LOW)
    GPIO.output(re,GPIO.LOW)

    # end connections to GPIOs, PiCam 
    GPIO.cleanup()
    cam.close()

    # for video purpose, to show overlay image clearly at the end of run
    sleep(20) 

    #close all open windows 
    cv2.destroyAllWindows()


'''  
* Function Name: process()
* Input: Nil 
* Output: Nil
* Logic: It captures continuous frames from the path. 
         The range for masking the path is defined and the captured image is masked.
         The contours present in the frame captured is used as a criterion to differentiate between smooth and sharp turns.
         The centroid of the path is detected and followed by the PB.
         If a zi is detected the control is passed on to ZI().
* Example Call: process ()
'''
def process():
    # zi_flag: This variable is checked every time to continue capturing frames. In case a zi is detected, zi_flag is set which ends the current video loop and then calls ZI() function.
    zi_flag=0
    # global variables used in this function
    global zi_read_once,cx

    # capturing video
    for frame1 in cam.capture_continuous(raw_cap,format="rgb",use_video_port=True,splitter_port=2,resize=(rw,rh)):
        # extract opencv bgr array of color frame
        frame = frame1.array
    
        # displays plantation background
        cv2.imshow('Overlay',background_img)  

        # Masking the frame to detect black color path and ignore shadows
        u=75 # u:upper limit value for masking
        l0=np.array([0,0,0]) # lower range for masking
        u0=np.array([u,u,u]) # upper range for masking 
        mask=cv2.inRange(frame,l0, u0) # masking done in order to avoid shadows as much as possible
        ret2,thresh = cv2.threshold(mask,127,255,0) # thresholding the masked frame
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) # detecting contours

        l=len(contours) # l:stores the number of contours

        # if no contours present in the present frame i.e,if a sharp turn has occured then take left or right turns depending on the position of centroid in previous frame
        # to take sharp left and right turns, differentially 
        if l == 0:  
           if  cx>180 :  
              right(lmspeed,rmspeed)
           elif cx<140:
              left(lmspeed,rmspeed)
          
        # find the area of the contours
        # differentiate between ZI and path based on area
        for cnt in contours:
           if cnt is not None:
              area = cv2.contourArea(cnt) # find the area of contour
              
              # checking if area satisfies zone indicator condition  
              if (area>=zi_areaL and area<=zi_areaU and zi_read_once==0): 
                    forward(lmspeed,rmspeed) # moving forward so as to stop and capture a perfect frame consisting of CMs
                    sleep(0.2) 
                    print "ZI" 
                    zi_flag=1 # setting zi_flag to end outer video for loop and shift the control to ZI()
                    break # breaks from inner contour processing loop
                                              
              # right angle turn correction if the bot is not moving exactly straight
              # area of those contours is minimum and only one small contour is detected at that sharp turning 
              elif (area<=1500 and l==1):
                   if  cx>180 :  
                       right(lmspeed,rmspeed)
                       
                   elif cx<140:
                       left(lmspeed,rmspeed)
                       
              # checking for path area condition      
              elif (area>=path_areaL and area<path_areaU): 
                    M = cv2.moments(cnt) # find moment
        
                    if(M['m00']!=0): # to avoid Divide by zero error
                        # calculate the centroid of the path
                        cx = int(M['m10']/M['m00'])
                        cy = int(M['m01']/M['m00'])
                                    	
                        # To take smooth left and right turns based on centroid of the path. Respective motor functions are called.
                        if (120<cx<200 ): 
                            forward(lmspeed,rmspeed)
                        elif (cx > 200): 
                            right1(lmspeed)                
                        elif (cx < 120): 
                            left1(rmspeed)
        #clear stream to capture next frame
        raw_cap.truncate(0)

        # if zi_flag is set,break from video loop releasing cam object
        # waitkey is used here to stop the bot on pressing 'q' only during testing(to avoid damage)
        # in the demo video this holds the plantation background throughout the run 
        if(zi_flag==1 or ((cv2.waitKey(1) & 0xFF) == ord('q')) ):
            break
        # decrementing zi_read_once count when enough frames have been captured to prevent recognising same ZI more than once 
        if zi_read_once!=0:   
          zi_read_once-=1
    
    if zi_flag==1 : #calls ZI() function
        ZI()

    return

'''.
* Function Name: IP()
* Input: Nil
* Output: Nil
* Logic: A new video loop is started in which images are captured, converted to grayscale, blured, thresholded to remove noise, and pixels are reinverted.
         The area of all the contours in the frame is calculated; Using this, the shed, the path and the IP terrain are differentiated.
    	 Motor functions are called to traverse the IP.
* Example Call: IP()
'''	
def IP():
    # when ip_flag is set, the PB moves forward till it encounters some path or ip distinction
    forward(lmspeed, rmspeed)
    # inv: area set to distinguish between the path in white from that in black, used for inversion
    inv=30000
    # ip_entry, ip_exit: flags are set when the PB enters, exits the IP terrain based on area conditions
    ip_entry,ip_exit=0,0
    global shed_flag
    shed_flag=0

    # capture the frames in a loop for video
    for frame in cam.capture_continuous(raw_cap,format="rgb",use_video_port=True,splitter_port=2,resize=(rw,rh)):
        image = frame.array # extract opencv bgr array of color frame
        gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY) #convert each frame to grayscale.
        blur=cv2.GaussianBlur(gray,(5,5),0) #blur the grayscale image
   	ret,th1 = cv2.threshold(blur,35,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU) #using threshold remove noise
        ret1,th2 = cv2.threshold(th1,127,255,cv2.THRESH_BINARY_INV) # invert the pixels of the image frame
        # contours are drawn on white regions,therefore pixels are inverted to detect the black line
        # if black background area is more, pixels are reinverted to restore normal and PB follows white line
        contours, hierarchy = cv2.findContours(th2,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE) #find the contours    
        # areasum: used to hold the total area of all the contours detected in the image (since multiple contours exist in the IP terrain)
        areasum=0 
        for cnt in contours: 
            area = cv2.contourArea(cnt) # find the area of contour
            areasum+=area
        if(areasum > inv):
                path_flag=0 #clear the path_flag when the PB enters the IP terrain i.e when it starts following the white line
		if(areasum>60000):
                   ip_entry=1 #set the ip_entry flag when the total area detected is greater than the set threshold of 60000
                if(ip_entry==1 and areasum<40000):
                   ip_exit=1 #set the ip_exit flag only after the PB has entered the IP terrain and the total area detected becomes less than 40000
                   inv=60000 #change the inversion threshold to 60000 so that the image is no more inverted after the PB has traversed the IP terrain 
                contours1, hierarchy1 = cv2.findContours(th1,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE) #find the contours
		f_contours=contours1 # copy the contours1 detected to a final contour array 
        else :
		path_flag=1 #set the path flag when the thresholding condition for inversion is not satisfied and PB is following black line
		f_contours=contours # copy the contours detected to a final contour array

        l=len(f_contours) #l: variable to store number of contours detected

        for cnt in f_contours:
         if cnt is not None:
	  area = cv2.contourArea(cnt) # calculate the area of the individual contours detected 
          if(path_flag==1 and ip_exit==1  and area>=shed_areaL and area<=shed_areaU):
                        shed_flag=1
                        # shed_flag is set only when the PB has exit from the IP terrain and entered the small black path present after that.
                        # It also checks if the area of the contour falls within the range of the shed area
          if(shed_flag==1 and areasum<=25000):
                # stop() is called when the shed_flag is set i.e. the shed is detected, and the area detected is less than the total shed area
                # as the PB has to move certain distance to cover the gap between the PiCam and the wheel
                stop()
		return 
                # pass the control to ZI(), which passes it to process() from where it returns to main()
            
          # when the area of the contour is greater than the lower threshold for path area, the centroid processing technique is used to follow the path both in IP and small patch of normal path present before and after the IP terrain  
          if(area>=path_areaL) :
               M = cv2.moments(cnt)
               if(M['m00']!=0): #to avoid Divide by zero error
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])          
                    if (120<cx<200):
                         forward(lmspeed,rmspeed)
                    elif (cx > 200):
                        right1(lmspeed)
                    elif (cx < 120):
                        left1(rmspeed)

        raw_cap.truncate(0)

        #waitkey is used here to stop the bot on pressing 'q' only during testing(to avoid damage)
        #in the demo video this holds the plantation background throughout the run 
        if((cv2.waitKey(1) & 0xFF) == ord('q')):
            return

'''
* Function Name: ZI()
* Input: nil
* Output: nil
* Logic: The PB stops at the ZI and checks if it has stopped correctly so that it can capture a proper picture in which all 4 color markers can be covered(if 4 are present).
         If not, then it starts capturing frames and corrects its position by comparing its centroid position and taking differential turn at a slower speed
         Once its position is correct,a new image is captured and processed.
         Ignoring the other contours only the CMs are identified.
	 If no CMs are detected, the ip_flag is set. 
	 Else, the function to blink the rgb LEDs, i.e. rgb() and the function overlay() are called.
* Example Call: ZI()
'''
def ZI():
    stop() # stopping the planter bot when ZI is detected
    # global variables used in this function
    global  zi_read_once,zi_count,cx

    # cm_ctr : local variable to store number of color markers in this zone
    # imp : local variable to store index of first color marker detected
    cm_ctr,imp=0,0

    # end correction if bot position is incorrect to capture image
    if cx>170 or cx<100:
            cd=0 # cd : flag to indicate correction done
            for frame in cam.capture_continuous(raw_cap,format="rgb",use_video_port=True,splitter_port=2,resize=(rw,rh)):
                    cor = frame.array # cor : image processed for applying correction
                    gray=cv2.cvtColor(cor,cv2.COLOR_BGR2GRAY) #convert each frame to grayscale.
                    blur=cv2.GaussianBlur(gray,(5,5),0) #blur the grayscale image
                    ret,th1 = cv2.threshold(blur,35,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU) #using threshold remove noise
                    ret1,th2 = cv2.threshold(th1,127,255,cv2.THRESH_BINARY_INV) # invert the pixels of the image frame
                    contours, hierarchy = cv2.findContours(th2,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE) #find the contours    
                    for cnt in contours:
                      if cnt is not None:
                        M = cv2.moments(cnt)
                        if(M['m00']!=0): # to avoid Divide by zero error
                             # calculate the centroid of the path
                             cx = int(M['m10']/M['m00'])
                             if(cx>190):
                                right(15,15)
                             elif cx<90: 
                                left(15,15)
                             if(cx>155 and cx<170):
                               stop()
                               cd=1  #correction done variable is set
                               break;
                    raw_cap.truncate(0)
                    if(cd==1): # break out of video loop once correction is done
                       break
    # capture frame in front of ZI
    cam.start_preview()
    cam.brightness=50
    sleep(1)# camera warmup time
    cam.capture("img.jpg")# capture frame containing CM's
    cam.stop_preview()
    img=cv2.imread('img.jpg')
    img=cv2.resize(img,(0,0),fx=2,fy=2)# resizing the image for better resolution
    img1=img.copy()
    # eroding and dilating to remove noise
    kernel = np.ones((3,3), np.uint8)
    img = cv2.erode(img, kernel, iterations=10) 
    img = cv2.dilate(img, kernel, iterations=10)
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY) # converting image to grayscale image
    ret2,thresh = cv2.threshold(gray,127,255,0)
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE) # finding the contours
    i,j=0,len(contours) # j: stores length of contours
    shape,color,area1=['']*j,['']*j,['']*j  # local lists to store color,shape and area of all contours detected
    while(i<j):
        cnt = contours[i]# accessing each contour
        area1[i]=cv2.contourArea(cnt)# finding area of contour
        shape[i]=cntshape(cnt)# finding shape of each contour
        M = cv2.moments(cnt)
        if(M['m00']!=0): #to avoid Divide by zero error
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                [b, g, r] = img[cy,cx]# finding the b,g,r values of the centroid pixel and finding the greatest value among them to detect the colour
                if (g>r and g>b ) :
                  color[i]='Green'
                elif (b> r and b>g ):
                  color[i]='Blue'
                else :
                  color[i]='Red'
                if(color[i]!='' and area1[i]>500 and area1[i]<4000):# only the contours within the area range are considered as colour markers
                  if imp==0:
			imp=i #to store the index of first contour which satisified CM conditions
			# given all color markers in a particular zone are of same shape and color
		  cm_ctr+=1         
        i=i+1
    if (cm_ctr==0): # if no CMs are present when a ZI is detected, the ip_flag is set
         print 'IP'
         global ip_flag
	 ip_flag=1 # setting the ipflag
         IP() # call IP()
         return
    else:
          rgb(color[imp],cm_ctr) # blinking the rgb LED depending on the number of CMs
          # stores the color,shape and count of detected CMs in respective lists 
    	  cm_color.append(color[imp])
          cm_shape.append(shape[imp])
          cm_count.append(cm_ctr)
          # finds the corresponding overlay image from Input Table.csv
          # 0th index in these lists to corresponds to first zone(A) and index one to zone B and so on
          name=str(readcsv(cm_color[zi_count],cm_shape[zi_count]))
          # incrementing zi_count
          zi_count+=1 # zi_count is incremented here, for ip its not incremented
          # if overlay_img is not found,return
          if(name== ''):
             return
          # read the image,-1 is for alpha channel
          overlay_img=cv2.imread('/home/pi/Seedlings/'+name,-1)
          # calls overlay function
          background_img1=overlay(cm_ctr,overlay_img,zi_count,background_img)
          # displays overlaid plantation background
          cv2.imshow('Overlay',background_img1)
    zi_read_once=15 #setting minimum number of frames to be captured before next ZI is detected
    if ip_flag==0:
        process() # call process() to continue capturing frames and processing 
    return

'''
* Function Name: rgb()
* Input: color - stores the color of the CMs detected by masking the frame captured at a ZI.
         cm_ctr - holds the updated value of the number of CMs detected at a ZI.
* Output: Nil
* Logic: according to the color of the CM, the RGB LED is blinked in the respective colour for cm_ctr number of times.
* Example Call: rgb(color,cm_ctr)
'''
def rgb(color,cm_ctr):
  # initialize for() loop to blink RGB LED cm_ctr number of times
  for i in range (0,cm_ctr):
    # blinking the LED in respective colour
    if (color == 'Red'):
        GPIO.output(R, GPIO.HIGH)
        sleep(1)
        GPIO.output(R, GPIO.LOW)
	sleep(1)
    
    elif (color == 'Green'):
        GPIO.output(G, GPIO.HIGH)
        sleep(1)
        GPIO.output(G, GPIO.LOW)
	sleep(1)
    
    elif (color == 'Blue'):
        GPIO.output(B, GPIO.HIGH)
        sleep(1)
        GPIO.output(B, GPIO.LOW)
	sleep(1)
  return

'''
* Function Name: endrun()
* Input: cm_color - It is a list to store the colour of the CMs identified at various ZIs along the path.
         zi_count - It is the number of ZIs detected along the path.
* Output: Nil
* Logic: It calls the rgb() zi_count number of times to blink the RGB LED at the end of the run.
* Example Call: endrun(cm_color,zi_count)
'''
def endrun(cm_color,zi_count):
    if zi_count is None :
       return
    #at the end of the run blinking the LED in the order of the colour of the CM's detected
    for i in range(0,(zi_count)):
        rgb(cm_color[i],1)
    return        

'''
* Function Name: cntshape()
* Input: cnt - It has the contours identified on the image captured at the ZI.
* Output: shape - It holds the shape of the CM detected at a ZI.
* Logic: Using the opencv funtion, cv2.approxPolyDP(), the shape of the CM is identified.
* Example Call: shape=cntshape(cnt) 
'''     
def cntshape(cnt):
    approx = cv2.approxPolyDP(cnt,0.04*cv2.arcLength(cnt,True),True)
    if len(approx)==3:
       shape='Triangle'
    elif len(approx)==4:
       shape='Square'       
    elif len(approx)<=12:
       shape='Circle' 
    return shape

'''
* Function Name: readcsv()
* Input: color - stores the color of the CMs detected by masking the frame captured at a ZI.
         shape - It holds the shape of the CM detected at a ZI.
* Output: It returns the seedling image to be overlaid on the plantation background image.
* Logic: The csv file provided is read and the right seedling image is chosen to perform the overlay.  
* Example Call: name=str(readcsv(color,shape))
'''
def readcsv(color,shape):
    global filename
    with open(filename) as f: 
        data = f.readlines()
    i,words=0,['']*10
    for line in data:
        words[i] = line.split(",")# each word is differentiated and read individually by using comma seperator
        i=i+1
    img=filter(lambda x: shape in x, filter(lambda x: color in x, words))
    f.close()
    return img[0][2].replace('\n','').replace('\r','')
    
'''
* Function Name: forward()
* Input: rx - It is the value of duty cycle, which inturn decides the speed of the right motor.
         lx - It is the value of duty cycle, which inturn decides the speed of the left motor.
* Output: Nil
* Logic: It changes the duty cycle of the M+ and M- pins of the right motor, so as to move the PB forward at a particular speed.
         It is used to give slightly different speeds to the left and right motors so they follow a straight path.
* Example Call: forward(lmspeed,rmspeed)
'''
def forward(lx, rx):
    # introduce correction in speed of motors
    pl1.ChangeDutyCycle(0)
    pl2.ChangeDutyCycle(lx)
    pr1.ChangeDutyCycle(0)
    pr2.ChangeDutyCycle(rx)

'''
* Function Name: right()
* Input: lx - It is the value of duty cycle, which inturn decides the speed of the left motor.
         rx - It is the value of duty cycle, which inturn decides the speed of the right motor.
* Output: Nil
* Logic: It changes the duty cycle of the M+ and M- pins of the left and right motors, so as to steer the PB towards the right at a particular speed, differentially.
         It is used for taking sharp turns.
* Example Call: right(lmspeed,rmspeed) 
'''	  
def right(lx,rx):
	# turn right motor backward and left motor in forward direction
        pl1.ChangeDutyCycle(0)
	pl2.ChangeDutyCycle(lx)
	pr1.ChangeDutyCycle(rx)
	pr2.ChangeDutyCycle(0)
	
'''
* Function Name: left()
* Input: lx - It is the value of duty cycle, which inturn decides the speed of the left motor.
         rx - It is the value of duty cycle, which inturn decides the speed of the right motor.
* Output: Nil
* Logic: It changes the duty cycle of the M+ and M- pins of the left and right motors, so as to steer the PB towards the left at a particular speed, differentially.
         It is used for taking sharp turns.
* Example Call: left(lmspeed,rmspeed) 
'''	
def left(lx,rx):
        # turn left motor backward and right motor in forward direction
	pl1.ChangeDutyCycle(lx)
	pl2.ChangeDutyCycle(0)
	pr1.ChangeDutyCycle(0)
	pr2.ChangeDutyCycle(rx)

'''
* Function Name: right1()
* Input: lx - It is the value of duty cycle, which inturn decides the speed of the left motor.
* Output: Nil
* Logic: It changes the duty cycle of the M+ and M- pins of the left motor, so as to steer the PB towards the right at a particular speed.
         It is used for taking curved turns.
* Example Call:  right1(lmspeed)
'''	
def right1(lx):
        # turn left motor forward and stop the right motor.
	pl1.ChangeDutyCycle(0)
	pl2.ChangeDutyCycle(lx)
	pr1.ChangeDutyCycle(0)
	pr2.ChangeDutyCycle(0)

'''
* Function Name: left1()
* Input: rx - It is the value of duty cycle, which inturn decides the speed of the right motor.
* Output: Nil
* Logic: It changes the duty cycle of the M+ and M- pins of the right motor, so as to steer the PB towards the left at a particular speed.
         It is used for taking curved turns.
* Example Call:  left1(rmspeed)
'''	
def left1(rx):
        # turn right motor forward and stop the left motor.
	pl1.ChangeDutyCycle(0)
	pl2.ChangeDutyCycle(0)
	pr1.ChangeDutyCycle(0)
	pr2.ChangeDutyCycle(rx)

'''
* Function Name: stop()
* Input: Nil 
* Output: Nil
* Logic: It brings the duty cycle of the M+ and M- pins of the right and left motors to zero.
         It is used to stop the PB.
* Example Call: stop()
'''	
def stop():
    # stop both motors
    pl1.ChangeDutyCycle(0)
    pl2.ChangeDutyCycle(0)
    pr1.ChangeDutyCycle(0)
    pr2.ChangeDutyCycle(0)
    
'''
* Function Name: overlay()
* Input: cm_ctr - To hold the cm_count
         cm_img - It holds the image to be overlaid. 
         zone - To hold the zi_count
         pb - It holds the plantation background image.
* Output: img - It is the new overlaid image returned after the overlay is done at a ZI.
* Logic: The plantation background image is seperated into the 4 zones specified in the rulebook by drawing 4 rectangles. Each zone is divided into 4 compartments.
         The required number of seedling images corresponding to the CM detected at a ZI are resized and overlaid on that particular zone. 
* Example Call: background_img1=overlay(cm_ctr,overlay_img,zi_count,background_img)
'''
def overlay(cm_ctr,cm_img,zone,pb):
    # error handling for index out of bound
    # given maximum of 4 color markers in each zone, overlay has array of 4 coordinates
    # if cm_ctr exceeds 4, its deprecated to avoid raising errors and continue execution
    if(cm_ctr>4):
        cm_ctr=4
        
    img=pb # pb:plantation background 
    # depending on the position of the zones A,B,C,D in the plantation background and the number of colour markers detected the seedlings are overlaid

    # ax,(b1x,b2x),cx,dx: x coordinate for each beginning rectangle in respective zone
    # ay,(b1y,b2y),cy,dy: y coordinate for each beginning rectangle in respective zone
    # aw,(b1w,b2w),cw,dw: width of each rectangle in respective zone
    # ay,(b1h,b2h),ch,dh: height of each rectangle in respective zone
    ax,b1x,b2x,cx,dx=[0]*4,[0]*4,[0]*4,[0]*4,[0]*4
    ay,aw,ah=237,42,63
    b1y,b1w,b1h=186,36,38
    b2y,b2w,b2h=232,36,38
    cy,cw,ch=154,29,46
    dy,dw,dh=168,24,42 

    # since the plantation background image is fixed, we have calculated the co-ordinates of plantation zones
    for i in range(0,4):
        cx[i]= 248+33*i+4
        ax[i] = 351+46*i+4
        if(i<2):
            b1x[i] = 110+44*i+8
        else:
            b2x[i] = 71+44*(i-2)+8 
        dx[i] = 516+28*i+4
    if zone==1 :
        # overlay after first ZI is detected i.e, in zone A
        for i in range(0,cm_ctr):
            x,y,w,h=ax[i],ay,aw,ah
            overlay_image = cv2.resize(cm_img,(w,h))
            img[y:y+h,x:x+w,:] = blend_transparent(img[y:y+h,x:x+w,:], overlay_image)
    elif zone==2 :
        # overlay after second ZI is detected i.e, in zone B
        # in zone B, overlay positions are in the form of 2*2 matrix whereas other zones have 1*4 matrix configuration
        for i in range(0,cm_ctr):
            if(i<2):
               x,y,w,h=b1x[i],b1y,b1w,b1h
            else:
               x,y,w,h=b2x[i],b2y,b2w,b2h 
            overlay_image = cv2.resize(cm_img,(w,h))
            img[y:y+h,x:x+w,:] = blend_transparent(img[y:y+h,x:x+w,:], overlay_image)
    elif zone==3 :
        #overlay after third ZI is detected i.e, in zone C
        for i in range(0,cm_ctr):
            x,y,w,h=cx[i],cy,cw,ch
            overlay_image = cv2.resize(cm_img,(w,h))
            img[y:y+h,x:x+w,:] = blend_transparent(img[y:y+h,x:x+w,:], overlay_image)
    elif zone==4 :
       #overlay after fourth ZI is detected i.e, in zone D
       for i in range(0,cm_ctr):
            x,y,w,h=dx[i],dy,dw,dh
            overlay_image = cv2.resize(cm_img,(w,h))
            img[y:y+h,x:x+w,:] = blend_transparent(img[y:y+h,x:x+w,:], overlay_image)
    return img

'''
* Function Name: blend_transparent()
* Input: face_img - It is the rectangular part of the plantation image on which overlay is to be performed
         overlay_t_img - It is the seedling image to be overlaid on the plantation background
* Output: It returns the overlaid plantation background image
* Logic: It modifies only that part of the plantation background image on which the seedling is to be overlaid
* Example Call:  img[y:y+h,x:x+w,:] = blend_transparent(img[y:y+h,x:x+w,:], overlay_image)
'''
def blend_transparent(face_img, overlay_t_img):
    # split out the transparency mask from the colour info
    # grab the BRG planes
    overlay_img = overlay_t_img[:,:,:3]
    # and the alpha plane
    overlay_mask = overlay_t_img[:,:,3:]  

    # again calculate the inverse mask
    background_mask = 255 - overlay_mask

    # turn the masks into three channel, so we can use them as weights
    overlay_mask = cv2.cvtColor(overlay_mask, cv2.COLOR_GRAY2BGR)
    background_mask = cv2.cvtColor(background_mask, cv2.COLOR_GRAY2BGR)

    # create a masked out face image, and masked out overlay
    # we convert the images to floating point in range 0.0 - 1.0
    face_part = (face_img * (1 / 255.0)) * (background_mask * (1 / 255.0))
    overlay_part = (overlay_img * (1 / 255.0)) * (overlay_mask * (1 / 255.0))

    # and finally just add them together, and rescale it back to an 8bit integer image    
    return np.uint8(cv2.addWeighted(face_part, 255.0, overlay_part, 255.0, 0.0))

# call main() at the beginning when program is executed
if __name__ == "__main__":
    main()
