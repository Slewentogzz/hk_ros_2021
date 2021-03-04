#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from darknet_ros_msgs.msg import BoundingBoxes

import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg


#def callback(msg):

#class animal_pos:
#	def __init__(self):
#	    self.angle = 0
#	    self.dist = 0
 

class Callbacks:

    def __init__(self):
	self.lidar_list = []
	self.object = geometry_msgs.msg.TransformStamped()
	self.lidar = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
	self.darknet = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.darknet_callback)
	self.pub_tf = rospy.Publisher("/tf",tf2_msgs.msg.TFMessage,queue_size=1)
	

    def darknet_callback(self, darknet):

	for i in darknet.bounding_boxes: 
	    if (i.Class == 'cat' or i.Class == 'dog' or i.Class == 'cow') and i.probability > 0.3: 
		#rospy.loginfo("len: "+str(len(self.lidar_list)) + "  index:"+str(int(round((i.xmin+i.xmax)*62.5/(2*638)-31.25))))
		print str(i.Class) + " prob: "+str(i.probability)#xMax: " + str(i.xmax) +"\nxmin: " + str(i.xmin)+"\npos: "+str((i.xmin+i.xmax)*62.5/(2*638)-31.25)+"\ndist: "+str(self.lidar_list[int(round((i.xmin+i.xmax)*62.5/(2*638)-31.25))])
		#self.object = geometry_msgs.msg.TransformStamped()
		rospy.sleep(0.1)
            	self.object.header.frame_id = "camera"
            	self.object.header.stamp = rospy.Time.now()
            	self.object.child_frame_id = "animal"
            	self.object.transform.translation.x = 0.0
            	self.object.transform.translation.y = 0.0
            	self.object.transform.translation.z = 0.0

            	self.object.transform.rotation.x = 0.0
            	self.object.transform.rotation.y = 0.0
            	
		
		angle = int(-round((i.xmin+i.xmax)*62.5/(2*638)-31.25))
		self.object.transform.rotation.z = angle
		smallest = self.lidar_list[angle]
		
		for i in range(-10,10):
			if self.lidar_list[angle+i] < smallest and self.lidar_list[angle+i] > 0.0:
				smallest = self.lidar_list[angle+i]
				self.object.transform.rotation.z = angle+i

            	self.object.transform.rotation.w = smallest                         
		self.pub_tf.publish(tf2_msgs.msg.TFMessage([self.object]))
	


    def lidar_callback(self, lidar):
	self.lidar_list = lidar.ranges[:]

    def talker(self):
	#pub = rospy.Publisher('animal_pos',tfMessage,queue_size = 10)
	#rospy.init_node('talker',anonymous=True)
	#rate = Rate(10) #behovs?
	rospy.loginfo("Loginfo????")
	self.pub_tf.publish(tf2_msgs.msg.TFMessage([self.object]))


if __name__ == '__main__':

    rospy.init_node('scan_values')
    values = Callbacks()
    rospy.spin()


#rospy.init_node('scan_values')
#sub = rospy.Subscriber('/scan', LaserScan, callback)

#rospy.spin()
