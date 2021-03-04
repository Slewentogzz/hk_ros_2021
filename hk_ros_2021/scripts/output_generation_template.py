#!/usr/bin/env python2

# Example how to generate the output file

import yaml
import rospkg
import rospy
import math
import tf2_ros
import tf2_geometry_msgs

from tf.msg import tfMessage
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, PointStamped

global detections
global detections_internal
x_rob_odom = 0
y_rob_odom = 0
rot_rob_odom = 0

def objectPlacement(data):
	#global x_rob_odom, y_rob_odom, rot_rob_odom
	x_obj_odom = (x_rob_odom + data.transforms[0].transform.translation.z*math.cos(rot_rob_odom*math.pi) - (-data.transforms[0].transform.translation.x)*math.sin(rot_rob_odom*math.pi))
	y_obj_odom = (y_rob_odom + data.transforms[0].transform.translation.z*math.sin(rot_rob_odom*math.pi) + (-data.transforms[0].transform.translation.x)*math.cos(rot_rob_odom*math.pi))

	return (x_obj_odom, y_obj_odom)

def animalPlacement(data):
    x_animal_odom, y_animal_odom = x_rob_odom + math.cos(rot_rob_odom*math.pi + data.transforms[0].transform.rotation.z*math.pi/180)* data.transforms[0].transform.rotation.w, y_rob_odom + math.sin(rot_rob_odom*math.pi +  data.transforms[0].transform.rotation.z*math.pi/180)* data.transforms[0].transform.rotation.w
    return x_animal_odom,y_animal_odom


# function to identify and store objects as their detected
def callbackObjects(data):
	global x_rob_odom, y_rob_odom


	#rospy.loginfo(rospy.get_caller_id() + "\nI heard \n%s", data.transforms[0].header.frame_id)
	#print(data)

	

	#if data.transforms[0].header.frame_id == "map":
		#print("Map:" + str(data.transforms[0].transform.translation.x) + "\t" + str(data.transforms[0].transform.translation.y))

	if data.transforms[0].header.frame_id == "camera" and data.transforms[0].child_frame_id != "animal" and data.transforms[0].child_frame_id != "geometric_shape":
		(x_obj_odom, y_obj_odom) = objectPlacement(data)
		#print("Apr_odom:\t" + str(x_obj_odom) + "\t" +  str(y_obj_odom))

		ID = data.transforms[0].child_frame_id

		x = round(x_obj_odom, 3)
		y = round(y_obj_odom, 3)
		
		if x<20 and y< 20:
			if ID not in lookup:
				lookup.append(ID)
				#detections.append({"obj_type": "A", "XY_pos": [0.756,3.332]})
				detections.append({"obj_type": "A", "XY_pos": [-x, -y]}) #The A needs to be replaced with a variable, when we identify animals and objects
		else:
			index = lookup.index(ID)
			detections[index] = {"obj_type": "A", "XY_pos": [-x, -y]}

	
	if data.transforms[0].header.frame_id == "camera" and data.transforms[0].child_frame_id == "animal" and data.transforms[0].transform.rotation.w != 0:
		(x_obj_odom, y_obj_odom) = animalPlacement(data)
		#print("Ani_odom:\tx: " + str(x_obj_odom) + "\t y:" +  str(y_obj_odom))

		ID = ["animal", round(x_obj_odom,3), round(y_obj_odom,3)]


		x = round(x_obj_odom, 3)
		y = round(y_obj_odom, 3)
		if x<20 and y< 20:
			

			if len(lookup) < 1:
				#print "New entry"
				lookup.append(ID)
				detections.append({"obj_type": "C", "XY_pos": [x, y]})
			else:
				for i, e in enumerate(lookup):
					print e
					if isinstance(e, list) and (abs(e[1]-x) < 1 and abs(e[2]-y) < 1):
						#print "Replaced"
						detections[i] = {"obj_type": "C", "XY_pos": [-x, -y]}
						lookup[i] = ID
						break
					elif i == len(lookup)-1 or len(lookup) == 0:
						#print "New entry"
						lookup.append(ID)
						detections.append({"obj_type": "C", "XY_pos": [-x, -y]})
	
	if data.transforms[0].header.frame_id == "camera" and data.transforms[0].child_frame_id == "geometric_shape" and data.transforms[0].transform.rotation.w > 0.7 and data.transforms[0].transform.rotation.w < 1.5:
		print "geometric shape"
		(x_obj_odom, y_obj_odom) = animalPlacement(data)
		print "Geo_odom:\t", x_obj_odom, y_obj_odom
		
		ID = ["geom", round(x_obj_odom,3), round(y_obj_odom,3)]


		x = round(x_obj_odom, 3)
		y = round(y_obj_odom, 3)
		
		if len(lookupGeo) < 1:
			print "New entry"
			lookupGeo.append(ID)
			detectionsGeo.append({"obj_type": "C", "XY_pos": [x, y]})
		else:
			for i, e in enumerate(lookupGeo):
				print e
				if isinstance(e, list) and (abs(e[1]-x) < 1 and abs(e[2]-y) < 1):
					print "Replaced"
					detectionsGeo[i] = {"obj_type": "B", "XY_pos": [-x, -y]}
					lookupGeo[i] = ID
					break
				elif i == len(lookupGeo)-1:
					print "New entry"
					lookupGeo.append(ID)
					detectionsGeo.append({"obj_type": "B", "XY_pos": [-x, -y]})

		
				
	
# function to localise the robots whereabouts
def callbackOdom(data):
	global x_rob_odom, y_rob_odom, rot_rob_odom
	x_rob_odom = data.pose.pose.position.x
	y_rob_odom = data.pose.pose.position.y
	rot_rob_odom = data.pose.pose.orientation.z
	#print(data)
	#print("Rob_odom:\t " + str(x_rob_odom) + "\t" + str(y_rob_odom) + "\t" + str(rot_rob_odom))


# function to call all publishers for information
def topicSubscriber():
	rospy.Subscriber("tf", tfMessage, callbackObjects)
	rospy.Subscriber('/odom', Odometry, callbackOdom)







if __name__ == '__main__':
	# 1 create an empty list to store the detections
	
	detections = []
	lookup = []

	detectionsGeo = []
	lookupGeo = []

	rospy.init_node('listener', anonymous=True)
	#transform = tf2_ros.Buffer.lookup_transform('turtle1', 'world', rospy.Time.now()) #tf2_ros.Buffer.lookupTransform('odom', 'base_footprint')
	topicSubscriber()



	rospy.spin()
	filename = "latest_output_file.yaml"
	filenameGeo = "latest_output_fileGeo.yaml"
	filepath = rospkg.RosPack().get_path('hk_ros_2021') + '/exported_detection_logs/' 

	with open(filepath + filename, 'w') as outfile:
    		yaml.dump_all(detections, outfile,explicit_start=True)
		outfile.close()

	with open(filepath + filenameGeo, 'w') as outfile:
    		yaml.dump_all(detectionsGeo, outfile,explicit_start=True)



#transforms: 
#  - 
#    header: 
#      seq: 0
#      stamp: 
#        secs: 1607481189
#        nsecs: 209141609
#      frame_id: "camera"
#    child_frame_id: "tag_9"
#    transform: 
#      translation: 
#        x: 1.06663620594
#        y: 0.445837117661
#        z: 1.72656781987
#      rotation: 
#        x: 0.727708084377
#        y: 0.685785767952
#       z: 0.000157267331777
#       w: -0.0117813273492

	


# 2 append detections during the run
# remember to add logic to avoid duplicates

# first dummy detection (apriltag)
#detections.append({"obj_type": "A", "XY_pos": [0.756,3.332]})

# second dummy detection (geometric shape)
#detections.append({"obj_type": "B", "XY_pos": [3.396,0.123]})

# third dummy detection (animal)
#detections.append({"obj_type": "C", "XY_pos": [6.001,2.987]}) 


# 3 save the file
#filename = "latest_output_file.yaml"
#filepath = rospkg.RosPack().get_path('hk_ros_2021') + '/exported_detection_logs/' 

#with open(filepath + filename, 'w') as outfile:
    #yaml.dump_all(detections, outfile,explicit_start=True)
