#!/usr/bin/env python

import cv2
import math
import heapq
import random
import time
import rospy
from collections import deque
from geometry_msgs.msg import Twist
from move_turtle.srv import *

#the Pose from turtlesim can't be used because of not providing accurate current position due to waiting older messages
#from turtlesim.msg import Pose
#newPosition service needs to be imported, works in place Pose of turtlesim


#basically Pose message, but part of manually defined service newPositionResponse, gets current position of turtle
position=newPositionResponse()

startTime=time.time()
#unit step
l=5
#size of image
size=[]
#sorce and destination points on image
xSource=0
ySource=0
xDestination=0
yDestination=0

#finds source and destination points
def sourceDestinationPoints(img):
	global xSource
	global ySource
	global xDestination
	global yDestination
	global size

	#finding source node
	for y in range(size[0]):
		for x in range(size[1]):
			if img.item(y,x,1)>200 and img.item(y,x,2)<100 and img.item(y,x,0)<100:
				xSource=x
				ySource=y
				break;

		if xSource==x and	ySource==y:
			break;	

	#finding destination node
	for y in range(size[0]):
		for x in range(size[1]):
			if img.item(y,x,2)>200 and img.item(y,x,1)<100 and img.item(y,x,0)<100:
				xDestination=x
				yDestination=y
				break;

		if xDestination==x and	yDestination==y:
			break;			

#checks node is inside image
def valid(x, y):
	global size

	if (x >=0 and y >= 0) and (x < size[1] and y < size[0]): 
		return 1
	else:
		return 0	

#checks closest vertex to random point
def closestVertex(xRad, yRad, listNode):

	xClosestVertex=None
	yClosestVertex=None
	distanceClosestVertex=100000

	for i in listNode:

		distanceCurrentVertex = math.sqrt( (xRad-i[1])*(xRad-i[1]) + (yRad-i[0])*(yRad-i[0]) )

		if  distanceCurrentVertex< distanceClosestVertex:
			distanceClosestVertex = distanceCurrentVertex
			xClosestVertex = i[1]
			yClosestVertex = i[0]

	#returning closest vertex and its distance
	return [yClosestVertex, xClosestVertex,distanceClosestVertex]				

#checks if point is on obstacle
def notObstacle(y, x, img):

	if int(img.item(y,x,0))<50 or int(img.item(y,x,1))<50 or int(img.item(y,x,2))<50 :
		return 1
	else:
		return 0


#checks lowest cost in vicinity
def lowestCostNode(nodeTree, xProbableNode, yProbableNode, costNode):
	global size

	xvals=[]
	yvals=[]
	nodeDistance=None
	xParentNode=None
	yParentNode=None
	lcstNodeDistance= 10000000000000000000000000000
	probableRemaps=[]

	#finding valid points to check in 15*15
	for i in range(xProbableNode-15, xProbableNode+15):
		if i>=0 and i< size[1]:
			xvals.append(i)
	for i in range(yProbableNode-15, yProbableNode+15):
		if i>=0 and i< size[0]:
			yvals.append(i)		
	
	#finding lowest cost node
	for y in yvals:

		for x in xvals:
			if x== xProbableNode and y== yProbableNode:
				continue
			if nodeTree[y][x] == 1:
				#adding nodes in region for future use, when checking if other point can be updated with new node
				probableRemaps.append([y,x])
				nodeDistance=  math.sqrt( (xProbableNode -x)*(xProbableNode-x) + (yProbableNode-y)*(yProbableNode-y) ) + costNode[y][x]
				if nodeDistance< lcstNodeDistance:
					lcstNodeDistance=nodeDistance
					xParentNode= x
					yParentNode= y

	#returning parent node and its distance, also nodes to check remapping
	return [yParentNode,xParentNode,lcstNodeDistance], probableRemaps				


#checks for any obstacles
def notBlocked(ParentNode, x, y, img):

	flag=1

	if x-ParentNode[1]>0 :
		step =1
	#corrected if the path becomes vertical
	elif x==ParentNode[1]:
		for i in range(y, ParentNode[0]):
			if not notObstacle(x, i, img):
				return 0
		return 1
	else:
		step =-1

	#checking for any obstacle in path, needed correction denominator can become zero
	for i in range (0, x-ParentNode[1], step):
		k=math.floor((i*y+(x-ParentNode[1]-i)*ParentNode[0])/(x-ParentNode[1]))
		for j in range(-1,2):
			if valid(k+j,i+ParentNode[1]):
				if notObstacle(k+j, i+ParentNode[1], img):
					continue
				else:
					flag=0
					return flag	

	return flag

#if any remapping is done, cost needs to be changed of all child, done it using bfs
def changeCost(parentNode, costNode, node):

	priorityQueue=[]
	#creating a prirority queue
	heapq.heapify(priorityQueue)
	heapq.heappush(priorityQueue,[0, node])

	#stores nuber of nodes at each priority level
	numberElementsPriority=[1]
	i=0
	#checks weather current level of node has any element
	while numberElementsPriority[i]:
		#runs for number of elements at ith level
		for j in range(0,numberElementsPriority[i]):
			#creates i+1th level
			numberElementsPriority.append(0)
			#takes out element from ith level and updates its cost
			a=heapq.heappop(priorityQueue)
			distanceParent=math.sqrt((a[1][0]-parentNode[a[1][0]][a[1][1]][0][0])*(a[1][0]-parentNode[a[1][0]][a[1][1]][0][0])+(a[1][1]-parentNode[a[1][0]][a[1][1]][0][1])*(a[1][1]-parentNode[a[1][0]][a[1][1]][0][1]))
			costNode[a[1][0]][a[1][1]]= costNode[parentNode[a[1][0]][a[1][1]][0][0]][parentNode[a[1][0]][a[1][1]][0][1]] + distanceParent
			#adds all elents in queue to i+1th level and updates their number
			for k in range(1,len(parentNode[a[1][0]][a[1][1]])):
				heapq.heappush(priorityQueue,[i+1, [parentNode[a[1][0]][a[1][1]][k][0], parentNode[a[1][0]][a[1][1]][k][1]]])
				numberElementsPriority[i+1] += 1
		#increase level by 1		
		i+=1		 

#changes colors of pixel lying on path between two nodes to blue
def changeColour(image, node1,node2):

	xDisplacement=node1[1]-node2[1]
	if xDisplacement>0 :
		step =1
	else:
		step =-1

	#changes colour while travelling in x direction
	for i in range (0, xDisplacement, step):
		k=math.floor((i*node1[0]+(xDisplacement-i)*node2[0])/(xDisplacement))
		image.itemset((k,node2[1]+i,0),255)
		image.itemset((k,node2[1]+i,1),0)
		image.itemset((k,node2[1]+i,2),0)	

	yDisplacement=node1[0]-node2[0]
	if yDisplacement>0 :
		step =1
	else:
		step =-1

	#changes colour while travelling in y direction 
	for i in range (0, yDisplacement, step):
		j=math.floor((i*node1[1]+(yDisplacement-i)*node2[1])/(yDisplacement))
		image.itemset((node2[0]+i,j,0),255)
		image.itemset((node2[0]+i,j,1),0)
		image.itemset((node2[0]+i,j,2),0)						

#provides two nodes to change colour, moves from lat node of tree to its source point through parent
def connectNodes(image, parentNode, node):
	#initialises parent node
	xNode=node[1]
	yNode=node[0]

	#provides two nodes for path colouring
	print([yNode,xNode],parentNode[yNode][xNode][0])
	while [yNode,xNode]!= parentNode[yNode][xNode][0]:
		changeColour(image,[yNode,xNode],[parentNode[yNode][xNode][0][0],parentNode[yNode][xNode][0][1]])
		[yNode,xNode]= [parentNode[yNode][xNode][0][0],parentNode[yNode][xNode][0][1]]
		print([yNode,xNode],parentNode[yNode][xNode][0])
		
#takes a step in direction and creates node on given tree
def createNode(img, nodeTree, connectionTree, parentNode, costNode, listNode):
	global size

	d=0
	#loops run till generated radom point is not on a node
	while not d: 
		xRad=random.randrange(0,size[1])
		yRad=random.randrange(0,size[0])

		ClosestVertex=closestVertex(xRad, yRad, listNode)
		#calculate the coordinates of probable node
		if ClosestVertex[2]!=0:
			xProbableNode = ClosestVertex[1] +int((xRad - ClosestVertex[1])*l/ClosestVertex[2])
			yProbableNode = ClosestVertex[0] +int((yRad - ClosestVertex[0])*l/ClosestVertex[2])
		d=	ClosestVertex[2]

	#check if node inside image	
	if valid(xProbableNode, yProbableNode):
		#check if node is not on obstacle
		if notObstacle(yProbableNode, xProbableNode, img):
			ParentNode, probableRemaps =lowestCostNode(nodeTree, xProbableNode, yProbableNode, costNode)
			#check if path between node and parent node is blocked
			if notBlocked(ParentNode, xProbableNode, yProbableNode, img):
				#update point to become node
				nodeTree[yProbableNode][xProbableNode]=1
				if [ParentNode[0],ParentNode[1]]==[yProbableNode,xProbableNode]:
					print(error)
				#update parent node of new node 	
				parentNode[yProbableNode][xProbableNode][0]=[ParentNode[0],ParentNode[1]]
				#add child to parent node
				parentNode[ParentNode[0]][ParentNode[1]].append([yProbableNode,xProbableNode])

				#append in node list of tree
				listNode.append([yProbableNode,xProbableNode])
				#update the cost
				costNode[yProbableNode][xProbableNode]=ParentNode[2]
				#remap tree
				remapTree( xProbableNode, yProbableNode, parentNode, costNode, probableRemaps)
				#connect tree
				connected, availableNodes = connectTree(xProbableNode, yProbableNode, connectionTree, img)
				return connected, availableNodes

	return 0, 0	
	
#checks for any remapping required
def remapTree( xNewNode, yNewNode, parentNode, costNode, probableRemaps):
	
	#iterate over all possible remaps
	for node in probableRemaps:
		distanceNewNode=math.sqrt((node[0]-yNewNode)*(node[0]-yNewNode) + (node[1]-xNewNode)*(node[1]-xNewNode))
		#change parent and update cost of all child if any remap found
		if costNode[yNewNode][xNewNode] + distanceNewNode < costNode[node[0]][node[1]]:
			for child in parentNode[parentNode[node[0]][node[1]][0][0]][parentNode[node[0]][node[1]][0][1]]:
				#removing remapped point from its earlier parent child
				if child[0]==node[0] and child[1]==node[1]:
					del child
					break
			#updating new parent
			parentNode[node[0]][node[1]][0]=[yNewNode, xNewNode]
			#adding child to new parent
			parentNode[yNewNode][xNewNode].append([node[0],node[1]])
			#change cost of all child	
			changeCost(parentNode, costNode, node)

#checks for minimum distance between new node and other tree, return all possible connection
def connectTree(xNewNode, yNewNode, connectionTree, img):
	global size

	availableNodes=[]
	connected=0
	nodeDistance=None
	#iterate in l*l square
	for i in range(xNewNode-l, xNewNode+l+1):
		if i>=0 and i< size[1]:
			for j in range(yNewNode-l, yNewNode+l+1):
				if j>=0 and j< size[0]:
					#checks if points are nodes
					if connectionTree[j][i]==1:
						nodeDistance=math.sqrt((xNewNode-i)*(xNewNode-i)+(yNewNode-j)*(yNewNode-j))
						#checks if node are within 1 step
						if nodeDistance<=l:
							if notBlocked([yNewNode,xNewNode],i,j,img):
								#adds any available connection
								availableNodes.append([[yNewNode, xNewNode], [j,i], nodeDistance])
								connected=1
	return  connected, availableNodes

#shows tree by changing colour
def showTree(img, parentNewNode, parentNode, costNewNode,  costNode, availableConnections):
	totalPathLength=1000000000000000000000
	joint=None
	pathLength= None
	#checks for best possible connection i.e shortest path
	for connection in availableConnections:
		pathLength=costNewNode[connection[0][0]][connection[0][1]] + costNode[connection[1][0]][connection[1][1]] + connection[2]
		if pathLength< totalPathLength:
			joint=connection
	
	print(joint)
	pathImage=img
	#prints path of tree on which new node is generated
	connectNodes(pathImage,parentNewNode, joint[0])
	#prints path of other tree
	connectNodes(pathImage,parentNode, joint[1])
	#joins both tree
	changeColour(pathImage,joint[0],joint[1])
	#changes colour of new node as it was left
	pathImage.itemset((joint[0][0],joint[0][1],0),255)
	pathImage.itemset((joint[0][0],joint[0][1],1),0)
	pathImage.itemset((joint[0][0],joint[0][1],2),0)

	#print total path length and time required to create path
	print(pathLength)
	print(time.time()-startTime)
	cv2.namedWindow("window1",cv2.WINDOW_NORMAL)
	cv2.imshow('window1',pathImage)
	cv2.waitKey(10000)
	return pathLength, joint

#translates our given properties of node into that of turtlesimulator dimensions
#points are selected and movement direction and distance error(distance from destination) are addes in queue
def path(parentSourceNode, parentDestinationNode, costSourceNode, costDestinationNode, connection, pathLength):
	global size
	#stores queue from source to destination 
	path = deque()
	#source node tree is explored from opposite direction of traversing dirction, pushing it into queue is mistake
	#first push into stack such that source point is at top the pop ans push into queue
	stack = deque()

	node=connection[0]
	stack.append(node)

	numberSourceNodes=1
	numberNodes=0
	#stack of node lying on path of source tree created
	while parentSourceNode[node[0]][node[1]][0]!=node:
		
		node=parentSourceNode[node[0]][node[1]][0]
		stack.append(node)
		numberSourceNodes+=1

	#all nodes from source tree lying on path is puhsed in queue along with angle with x-axis(+-pi/2) and error distance from destination point
	#all these properties are in environment of turtle except fo  error 

	#point on which you are
	startPoint=	stack.pop()
	for i in range(0,numberSourceNodes-1):
		#point which you want to move to
		endPoint=stack.pop()
		#only allows points hihger than certain change in x or y to be pushed, otherwise endpoint is pushed to another node
		if ((abs((size[0]-startPoint[0])*11.0/size[0]-(size[0]-endPoint[0])*11.0/size[0]))>0.25 or (abs((startPoint[1])*11.0/size[1]-(endPoint[1])*11.0/size[1]))>0.25):
			#checks if movement is in vertical direction
			if(endPoint[1]-startPoint[1]==0):
				#checks if movement is in upward direction or downward direction
				if ((-1.0)*(endPoint[0]-startPoint[0]))>0:

					theta=2*(math.atan(1))
				else:

					theta=2*(math.atan(-1.0))
			else:
				#finds angle in turtlesim dimensions
				theta=math.atan(((-1.0)*(endPoint[0]-startPoint[0])*size[0])/((endPoint[1]-startPoint[1])*size[1]))	

			error= (pathLength- costSourceNode[startPoint[0]][startPoint[1]])*11.0/600
			path.append([(size[0]-startPoint[0])*11.0/size[0],startPoint[1]*11.0/size[1],theta,error])
			numberNodes+=1
			startPoint=endPoint

	#node of source tree is connected to destination node
	endPoint=connection[1]
	if(endPoint[1]-startPoint[1]==0):

				if ((-1.0)*(endPoint[0]-startPoint[0]))>0:

					theta=2*(math.atan(1))
				else:

					theta=2*(math.atan(-1.0))
	else:

		theta=math.atan(((-1.0)*(endPoint[0]-startPoint[0])*size[0])/((endPoint[1]-startPoint[1])*size[1]))

	error= (pathLength- costSourceNode[startPoint[0]][startPoint[1]])*11.0/600
	path.append([(size[0]-startPoint[0])*11.0/size[0],startPoint[1]*11.0/size[1],theta,error])	
	numberNodes+=1
	startPoint=endPoint
	currentNode=endPoint
	#nodes of destination tree are pushed
	while parentDestinationNode[currentNode[0]][currentNode[1]][0]!=currentNode:

		endPoint=parentDestinationNode[currentNode[0]][currentNode[1]][0]
		if ((abs((size[0]-startPoint[0])*11.0/size[0]-(size[0]-endPoint[0])*11.0/size[0]))>0.25 or (abs((startPoint[1])*11.0/size[1]-(endPoint[1])*11.0/size[1]))>0.25):
			
			if(endPoint[1]-startPoint[1]==0):

				if ((-1.0)*(endPoint[0]-startPoint[0]))>0:

					theta=2*(math.atan(1))
				else:

					theta=2*(math.atan(-1.0))
			else:

				theta=math.atan(((-1.0)*(endPoint[0]-startPoint[0])*size[0])/((endPoint[1]-startPoint[1])*size[1]))

			error= (costDestinationNode[startPoint[0]][startPoint[1]])*11.0/600
			path.append([(size[0]-startPoint[0])*11.0/size[0],startPoint[1]*11.0/size[1],theta,error])
			numberNodes+=1
			startPoint=endPoint	
		currentNode=endPoint

	#last remaining nofr(if any) and destination points are pushed
	if endPoint==startPoint:

		path.append([(size[0]-startPoint[0])*11.0/size[0],startPoint[1]*11.0/size[1],0,0])
	else:

		if(endPoint[1]-startPoint[1]==0):

			if ((-1.0)*(endPoint[0]-startPoint[0]))>0:

				theta=2*(math.atan(1))
			else:

				theta=2*(math.atan(-1.0))
		else:

			theta=math.atan(((-1.0)*(endPoint[0]-startPoint[0])*size[0])/((endPoint[1]-startPoint[1])*size[1]))	
		error= (costDestinationNode[startPoint[0]][startPoint[1]])*11.0/600
		path.append([(size[0]-startPoint[0])*11.0/size[0],startPoint[1]*11.0/size[1],theta,error])
		numberNodes+=1
		startPoint=endPoint
		path.append([(size[0]-startPoint[0])*11.0/size[0],startPoint[1]*11.0/size[1],0,0])

	numberNodes+=1
	turtleMove(path, numberNodes)

def turtleMove(path, numberNodes):
	global position
	#creade a node using roscre
	rospy.init_node('moveTurtle', anonymous=True)
	#created a function which publishes to /turtle1/cmd_vel for controlling turtle velocity
	velocityPublisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

	startPoint=path.popleft()

	#waits for service, it teleports turtle to starting point
	rospy.wait_for_service('/turtle1/teleport_absolute')
	#becomes client to service TeleportAbsolute
	teleport = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)
	teleport(startPoint[1],startPoint[0],0)
	#becomes client to positionService, used defined, gives position accurately
	getPosition=rospy.ServiceProxy('positionService', newPosition)

	velocity= Twist()
	#getting position by sending 1(true) message
	position=getPosition(1)
	angle=None

	print(startPoint)

	for i in range(0,numberNodes-1):
		#the point to which turtle will go
		endPoint=path.popleft()
		print(endPoint)
		velocity.linear.x=0
		velocity.linear.y=0
		velocity.linear.z=0
		velocity.angular.x=0
		velocity.angular.y=0
		#absolute angle is measured (0, 2*pi), gives direction in which movement will occur
		if (endPoint[1]-startPoint[1])>=0 :

			if startPoint[2]>=0:
				angle=startPoint[2]
			else:
				angle=startPoint[2]+8*(math.atan(1))
		else:
			angle=startPoint[2]+4*(math.atan(1))

		print(angle)
		position=getPosition(1)
		#one extra while loop added to confirm that angle of turtle is correct
		#turtle is rorated with angular speed which is proportional to error in angle
		while abs(angle-position.theta)>0.00001:
			position=getPosition(1)
			while abs(angle-position.theta)>0.0001:
				#angular velocity is constantly updated, sloeing down as errore approaches 0
				velocity.angular.z=(angle-position.theta)*5
				velocityPublisher.publish(velocity)
				position=getPosition(1)
			time.sleep(0.2)	
			position=getPosition(1)	

		velocity.angular.z=0	
		velocityPublisher.publish(velocity)	

		#changes linear velocity, proprtional to error from destination
		velocity.linear.x=startPoint[3]/10
			
		while abs(position.x-endPoint[1])>0.01 or abs(position.y-endPoint[0])>0.01:
			#if turtle overshoots the end point by mistake, then the loop is terminated 
			if abs(position.x-startPoint[1])>abs(startPoint[1]-endPoint[1]) and abs(position.y-startPoint[0])>abs(startPoint[0]-endPoint[0]):
				break;
			position=getPosition(1)
			velocityPublisher.publish(velocity)
	
		print([position.y,position.x])
		velocity.linear.x=0	
		velocityPublisher.publish(velocity)
		#shifts starting point to current point
		startPoint=endPoint	
	print(time.time()-startTime)

def main():
	global size
	global xSource
	global ySource
	global xDestination
	global yDestination

	img= cv2.imread("obstacle.png",1)
	size.append(img.shape[0])
	size.append(img.shape[1])

	#keeps tracks of points which are node
	nodeSourceTree=[[0 for _ in range(size[1])] for _ in range(size[0])]
	nodeDestinationTree=[[0 for _ in range(size[1])] for _ in range(size[0])]

	#create lst of nodes, increases efficiency when generating nodes
	listSourceNode=[]
	listDestinationNode=[]


	# stores parent and child of a node on matrix of source and destination 
	# first element is parent, and others are child, helpul in updatding all childs cost when remapping is done
	parentSourceNode=[[[[y, x]] for x in range(size[1])] for y in range(size[0])]
	parentDestinationNode=[[[[y, x]] for x in range(size[1])] for y in range(size[0])]

	#cost array for source and destination tree nodes
	costSourceNode=[[100000 for _ in range(size[1])] for _ in range(size[0])]
	costDestinationNode=[[100000 for _ in range(size[1])] for _ in range(size[0])]

	#finding source points
	sourceDestinationPoints(img)
	sourcePoint=[ySource,xSource]
	destinationPoint=[yDestination,xDestination]

	#initialises parent of source and node tree
	nodeSourceTree[sourcePoint[0]][sourcePoint[1]]=1
	nodeDestinationTree[destinationPoint[0]][destinationPoint[1]]=1

	#adding nodes to list
	listSourceNode.append([sourcePoint[0],sourcePoint[1]])
	listDestinationNode.append([destinationPoint[0],destinationPoint[1]])

	#cost of source nd destination are zero
	costSourceNode[sourcePoint[0]][sourcePoint[1]]=0
	costDestinationNode[destinationPoint[0]][destinationPoint[1]]=0

	# flag for tree are connected or not
	connected=0

	#stores all the ways in which both tree can be connected
	availableConnections=None

	#stores total path length
	pathLength=None
	while not connected:
		#creating node on source tree
		connected, availableConnections=createNode(img, nodeSourceTree, nodeDestinationTree, parentSourceNode, costSourceNode, listSourceNode)
		if connected:
			#showing path if both trees are connected
			pathLength, connection=showTree(img, parentDestinationNode, parentSourceNode, costDestinationNode,  costSourceNode, availableConnections)
			break

		#creating node on destination tree
		connected, availableConnections=createNode(img, nodeDestinationTree, nodeSourceTree, parentDestinationNode, costDestinationNode, listDestinationNode)
		if connected:
			#showing path if both trees are connected
			pathLength, connection=showTree(img, parentDestinationNode, parentSourceNode, costDestinationNode,  costSourceNode, availableConnections)
			node= connection[0]
			connection[0]= connection[1]
			connection[1]= node
			break
	path(parentSourceNode, parentDestinationNode, costSourceNode, costDestinationNode, connection, pathLength)		
	

if __name__ == "__main__":			
	
	main()
