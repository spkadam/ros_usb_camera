#!/usr/bin/env python
import rospy
import cv2
import imutils
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from move_robot import MoveKobuki #trying on turtlebot3 in simulation
from pid_control import PID

class LineFollower(object):

    def __init__(self):
    	rospy.logwarn("Init sticky note line follower")
        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber("/image_raw",Image,self.camera_callback)
        self.movekobuki_object = MoveKobuki() #code to move bartbot
	# We set init values to ideal case where we detect it just ahead
        setPoint_value = 0.0
        state_value = 0.0
        self.pid_object = PID(init_setPoint_value = setPoint_value,
                        init_state = state_value)


    def camera_callback(self,data):
        
        try:
            
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
            

        height, width, channels = cv_image.shape
	# Cropping image for fast processing
#        descentre = 160
#        rows_to_watch = 20
#        crop_img = cv_image[(height)/2+descentre:(height)/2+(descentre+rows_to_watch)][1:width]
        
        # Convert from RGB to HSV
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
#        
	#defining range of greenish yellow color in HSV
	lower_color = np.array([36,80,80])
	upper_color = np.array([60,255,255])


        # Threshold the HSV image to get only required greenish yellow color
        mask = cv2.inRange(hsv, lower_color, upper_color)

	## slice the green
#	imask = mask>0
#	green = np.zeros_like(image, np.uint8)
#	green[imask] = image[imask]

	blurred = cv2.GaussianBlur(mask, (5, 5), 0)
	thresh = cv2.threshold(blurred, 23, 255, cv2.THRESH_BINARY)[1]

	# find contours in the thresholded image
	cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE)
	cnts = imutils.grab_contours(cnts)

# loop over the contours
    	for c in cnts:        
        # Calculate centroid of the blob of binary image using ImageMoments
        #m = cv2.moments(mask, False)
		M = cv2.moments(c)
        	try:
			cX, cY = M['m10']/M['m00'], M['m01']/M['m00']
        	except ZeroDivisionError:
            		cY, cX = height/2, width/2


		# draw the contour and center of the shape on the image
		cv2.drawContours(cv_image, [c], -1, (0, 255, 0), 2)
		cv2.circle(cv_image, (int(cX), int(cY)), 7, (255, 255, 255), -1)
		cv2.putText(cv_image, "center", (int(cX)- 20,int(cY)-20),
		cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

		cv2.imshow("Original", cv_image)
		cv2.imshow("HSV", hsv)
		cv2.imshow("MASK", mask)
#		
		cv2.waitKey(1)
        
        	# Move the Robot , center it in the middle of the witdth 640 => 320:
		setPoint_value = width/2
		self.pid_object.setpoint_update(value=setPoint_value)

		twist_object = Twist()
        	twist_object.linear.x = 0.1

		# Make it start turning

		self.pid_object.state_update(value=cX)
		effort_value = self.pid_object.get_control_effort()
		# Dividing the effort to map it to the normal values for angular speed in the turtlebot; didn't know how things work for bartbot
		rospy.logwarn("Set Value=="+str(setPoint_value))
		rospy.logwarn("State Value=="+str(cX))
		rospy.logwarn("Effort Value=="+str(effort_value))
		angular_effort_value = effort_value / 1000.0

		twist_object.angular.z = angular_effort_value
		rospy.logwarn("Twist =="+str(twist_object.angular.z))
		self.movekobuki_object.move_robot(twist_object)

		
    def clean_up(self):
        self.movekobuki_object.clean_class()
        cv2.destroyAllWindows()
        
        

def main():
    rospy.init_node('line_following_node', anonymous=True)
    
    
    line_follower_object = LineFollower()
   
    rate = rospy.Rate(5)
    ctrl_c = False
    def shutdownhook():
        # works better than the rospy.is_shut_down()
        line_follower_object.clean_up()
        rospy.loginfo("shutdown time!")
        ctrl_c = True
    
    rospy.on_shutdown(shutdownhook)
    
    while not ctrl_c:
        rate.sleep()

    
    
if __name__ == '__main__':
    main()
