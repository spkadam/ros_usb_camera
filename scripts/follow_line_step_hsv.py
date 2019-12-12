#!/usr/bin/env python
import rospy
import cv2
import imutils
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from move_robot import MoveKobuki #trying on turtlebot3

class LineFollower(object):

    def __init__(self):
    
        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber("/image_raw",Image,self.camera_callback)
        self.movekobuki_object = MoveKobuki() #code to move bartbot

    def camera_callback(self,data):
        
        try:
            # We select bgr8 because its the OpneCV encoding by default
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
            

        height, width, channels = cv_image.shape
#        descentre = 160
#        rows_to_watch = 20
#        crop_img = cv_image[(height)/2+descentre:(height)/2+(descentre+rows_to_watch)][1:width]
        
        # Convert from RGB to HSV
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
#        
	#defining range of greenish yellow  color in HSV
	lower_color = np.array([36,80,80])
	upper_color = np.array([60,255,255])


        # Threshold the HSV image to get only yellow colors
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
        
        
        # Bitwise-AND mask and original image
        #res = cv2.bitwise_and(crop_img,crop_img, mask= mask)
        
        # Draw the centroid in the resultut image
        # cv2.circle(img, center, radius, color[, thickness[, lineType[, shift]]]) 
        #cv2.circle(res,(int(cx), int(cy)), 10,(0,0,255),-1)


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
        
        
		error_x = cX - width / 2;
		twist_object = Twist();
		twist_object.linear.x = 0.2;
		twist_object.angular.z = -error_x / 100;
		rospy.loginfo("ANGULAR VALUE SENT===>"+str(twist_object.angular.z))
		# Make it start turning
		self.movekobuki_object.move_robot(twist_object) #twist required for bartbot
        
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
