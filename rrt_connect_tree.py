import cv2
import math
import heapq
import random
import time
from collections import deque

startTime=time.time()
l=5
size=[]
xSource=0
ySource=0
xDestination=0
yDestination=0
counter=0
blocked=0

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

#same as change colour, just schanges colour fro all tree branches
def treeColour(image, node1,node2):
	xDisplacement=node1[1]-node2[1]
	if xDisplacement>0 :
		step =1
	else:
		step =-1

	for i in range (0, xDisplacement, step):
		k=math.floor((i*node1[0]+(xDisplacement-i)*node2[0])/(xDisplacement))
		image.itemset((k,node2[1]+i,0),150)
		image.itemset((k,node2[1]+i,1),150)
		image.itemset((k,node2[1]+i,2),200)	

	yDisplacement=node1[0]-node2[0]
	if yDisplacement>0 :
		step =1
	else:
		step =-1

	for i in range (0, yDisplacement, step):
		j=math.floor((i*node1[1]+(yDisplacement-i)*node2[1])/(yDisplacement))
		image.itemset((node2[0]+i,j,0),150)
		image.itemset((node2[0]+i,j,1),150)
		image.itemset((node2[0]+i,j,2),200)

#explores all the nodes of given tree using bfs and changes colour, similar implementation as updating cost of all childs
def exploreTree(image, parentNode, node):

	priorityQueue=[]
	heapq.heapify(priorityQueue)
	heapq.heappush(priorityQueue,[0, node])
	numberElementsPriority=[1]
	i=0
	while numberElementsPriority[i]:
		for j in range(0,numberElementsPriority[i]):
			numberElementsPriority.append(0)
			a=heapq.heappop(priorityQueue)
			#pushes child into queue and changes colour off braanch
			for k in range(1,len(parentNode[a[1][0]][a[1][1]])):
				heapq.heappush(priorityQueue,[i+1, [parentNode[a[1][0]][a[1][1]][k][0], parentNode[a[1][0]][a[1][1]][k][1]]])
				treeColour(image,a[1], [parentNode[a[1][0]][a[1][1]][k][0], parentNode[a[1][0]][a[1][1]][k][1]] )
				numberElementsPriority[i+1] += 1
		i+=1

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
	global counter
	global blocked

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
				#counts total number of nodes created	
				counter+=1	
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
			else:
				blocked+=1
		else:
			blocked+=1
	else:
		blocked+=1							

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
	global counter
	global blocked

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
def showTree(img, parentNewNode, parentNode, costNewNode,  costNode, availableConnections,startNewPoint, startPoint):
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

	#explores both tree
	exploreTree(pathImage, parentNewNode, startNewPoint)
	exploreTree(pathImage, parentNode, startPoint)
	#changes colour of portion through which trees connects 
	treeColour(pathImage,joint[0],joint[1])

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
	print 'total path length =', pathLength
	print 'total number of nodes on both trees=', counter
	print 'total number of rejected probableNodes on both trees=', blocked
	print 'time in which computation was performed=', time.time()-startTime
	cv2.namedWindow("window1",cv2.WINDOW_NORMAL)
	cv2.imshow('window1',pathImage)
	cv2.waitKey(0)


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
			showTree(img, parentSourceNode, parentDestinationNode, costSourceNode,  costDestinationNode, availableConnections,sourcePoint,destinationPoint)
			break

		#creating node on destination tree
		connected, availableConnections=createNode(img, nodeDestinationTree, nodeSourceTree, parentDestinationNode, costDestinationNode, listDestinationNode)
		if connected:
			#showing path if both trees are connected
			showTree(img, parentDestinationNode, parentSourceNode, costDestinationNode,  costSourceNode, availableConnections,destinationPoint,sourcePoint)
			break
	

if __name__ == "__main__":			
	
	main()