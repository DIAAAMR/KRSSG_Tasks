#!/usr/bin/env python
import rospy
from turtlesim.msg import Pose
from move_turtle.srv import *

#saves current position published to /Pose
position=Pose()

#called when request is made by another node to provide current point
def sendPosition(requested):
	print(requested)
	#if request is true 
	if requested.c==1:
		print(position)
		#current position is sent, the datatype is the the response type of newPosition and it expects all defined arguments accurately
		return newPositionResponse(position.x, position.y, position.theta, position.linear_velocity, position.angular_velocity)

#called when data is received from /Pose
def collect(data):
	global position
	#updates current position 
	position = data

def main():
	global position
	#node is initialised by roscore
	rospy.init_node('position', anonymous=True)	
	#subscribes to topic /Pose and collects latest position
	rospy.Subscriber("/turtle1/pose", Pose, collect)
	#starts a service which sends current position on response by invoking sendPosition function
	s = rospy.Service('positionService', newPosition, sendPosition)
	rospy.spin()

if __name__ == "__main__":
	main()	
