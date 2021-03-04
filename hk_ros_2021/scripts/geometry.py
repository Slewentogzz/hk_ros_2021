#!/usr/bin/env python2

import numpy as np
import cv2
import rospy
from sensor_msgs.msg import LaserScan, Image
#from darknet_ros_msgs.msg import BoundingBoxes

import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg
from cv_bridge import CvBridge, CvBridgeError


#def callback(msg):

#class animal_pos:
#	def __init__(self):
#	    self.angle = 0
#	    self.dist = 0
 

class Callbacks:

	def __init__(self):
		self.lidar_list = []
		self.bridge = CvBridge()
		self.object = geometry_msgs.msg.TransformStamped()
		self.mask = 0
		self.object = geometry_msgs.msg.TransformStamped()
		self.lidar = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
		self.pub_tf = rospy.Publisher("/tf",tf2_msgs.msg.TFMessage,queue_size=1)
		self.imageSub = rospy.Subscriber('/raspicam_node/image', Image, self.image_callback, queue_size=10)
		self.imagePub = rospy.Publisher('/mask_image', Image, queue_size=10)
		

	def image_callback(self, image):
		#print image.width
		
		img = self.bridge.imgmsg_to_cv2(image, "bgr8")
		
		
		#cv2.imshow('Geometry Window', img)
		#img = np.asarray(image)
		#img = cv2.cvt(img)
		#img = cv2.imread('shapes.PNG')
		imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
		imgHSV = cv2.blur(imgHSV, (5,5))


		green_mask = cv2.inRange(imgHSV, (25, 100, 32), (102, 150, 160))
		blue_mask = cv2.inRange(imgHSV, (94,100,42), (126,150,160))


		mask_or = cv2.bitwise_or(green_mask, blue_mask)

		kernel = np.ones((5,5), np.uint8) 
		#mask_or_dilatated = cv2.dilate(mask_or, kernel, iterations=1	) #adds
		self.mask = cv2.erode(mask_or, kernel, iterations=1) #removes
		
		#self.mask = cv2.bitwise_and(img, img, mask=mask_or)
		contours , hierarchy = cv2.findContours(self.mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)[-2:]
		#print hierarchy
		#print contours
		
		try:	
			for contour in contours:
				#print contour
			    	approx = cv2.approxPolyDP(contour, 0.01* cv2.arcLength(contour, True), True)
				(x, y, w, h) = cv2.boundingRect(approx)
				#print w
				if w < 30 and w > 15 and h < 30 and h > 15:
					#print(x, y, w, h)
					#self.mask = cv2.rectangle(self.mask,(x,y),(x+w,y+h),(0,0,255),3)
					
					   
					#rospy.loginfo("len: "+str(len(self.lidar_list)) + "  index:"+str(int(round((i.xmin+i.xmax)*62.5/(2*638)-31.25))))
					#self.object = geometry_msgs.msg.TransformStamped()
					rospy.sleep(0.1)
				    	self.object.header.frame_id = "camera"
				    	self.object.header.stamp = rospy.Time.now()
				    	self.object.child_frame_id = "geometric_shape"
				    	self.object.transform.translation.x = x
				    	self.object.transform.translation.y = y
				    	self.object.transform.translation.z = 0.0

				    	self.object.transform.rotation.x = 0.0
				    	self.object.transform.rotation.y = 0.0
				    	
	
					angle = int(-round((x-w/2)*62.5/640-31.25))
					self.object.transform.rotation.z = angle
					smallest = self.lidar_list[angle]
					#print angle
					
	
					for i in range(-10,10):
						if self.lidar_list[angle+i] < smallest and self.lidar_list[angle+i] > 0.0:
							smallest = self.lidar_list[angle+i]
							self.object.transform.rotation.z = angle+i
							
					#print self.object.transform.rotation.z, smallest

				    	self.object.transform.rotation.w = smallest                         
					self.pub_tf.publish(tf2_msgs.msg.TFMessage([self.object]))

				
			#print "Countours"
		except:
			print "Exception"

		self.talker()




	def lidar_callback(self, lidar):
		self.lidar_list = lidar.ranges[:]

	def talker(self):
		#print self.bridge.cv2_to_imgmsg(self.mask, "8UC1")
		try: 
			#print "in talker in try"
			self.imagePub.publish(self.bridge.cv2_to_imgmsg(self.mask, "8UC1"))
		except:
			print "Exception"


if __name__ == '__main__':
	
	#cv2.namedWindow('Geometry Window', cv2.WINDOW_NORMAL)
	
	rospy.init_node('geometry')
	values = Callbacks()
	#rospy.spin()

	while not rospy.is_shutdown():
		continue

	#cv2.destroyAllWindows()


