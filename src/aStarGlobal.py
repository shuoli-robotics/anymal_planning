import sys
import os
import numpy as np
import copy
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d 
import math
import anyterrain as at
import time
from mayavi import mlab

class Point:
    def __init__(self,z,sigma):
        self.z = z
        self.sigma = sigma
        

class AnymalStateNode:
    def __init__(self,parentIndex,g,h):
        self.parent = parentIndex
        self.g = g
        self.h = h
        self.f = g + h



class AnymalAStarGlobal:
    def __init__(self,start,goal,terrain):
        self.terrain = terrain
        self.pointCloud = self.generatePointCloud()
        
        if start in self.pointCloud.keys():
            self.start = (start[0],start[1],self.pointCloud[start].z,0.0,0)
        else:
            print("Start point is invalid")
            
        if goal in self.pointCloud.keys():
            self.goal = (goal[0],goal[1],self.pointCloud[goal].z,0.0,0)
        else:
            print("Goal point is invalid")
        
        
        
        self.k_d_false = 1.0
        self.k_d_true = 2.0 #0.5
        self.k_h = 2.0
        self.k_sigma_true = 0.1
        self.k_sigma_false = 0.9 # 0.1
        
        self.openList = {}
        self.closedList = {}
        # print("The start node is {}".format(self.start))
        # print("The goal node is {}".format(self.goal))
        
        
    def generatePointCloud(self):
        X = np.arange(0,10,0.1)
        Y = np.arange(0,20,0.1)
        X = np.append(X,10)
        Y = np.append(Y,20)
        pointCloud = {}
        
        for x in X:
            for y in Y:
                x = round(x,1)
                y = round(y,1)
                for i in range(self.terrain.terrainPlanes.shape[0]):
                    if self.terrain.terrainPlanes[i,4] <= x < self.terrain.terrainPlanes[i,5] and self.terrain.terrainPlanes[i,6] <= y < self.terrain.terrainPlanes[i,7]:
                        pointCloud[(x,y)] = Point((-self.terrain.terrainPlanes[i,3]-self.terrain.terrainPlanes[i,0]*x-self.terrain.terrainPlanes[i,1]*y)/self.terrain.terrainPlanes[i,2],self.terrain.terrainPlanes[i,8])           
                        break
        return pointCloud
        
    def run(self):
        # add the start point to the open list. Its g = 0, h is an estimate distance towards the goal
        self.openList[self.start] = AnymalStateNode(self.start,0,self.getH(self.start))
        
        numSearchTimes = 0
        
        while len(self.openList) > 0:
            # find the minimum element and stand on it. Then, search its children          
            currentNode = self.moveToMinNode()
            numSearchTimes = numSearchTimes + 1
            if self.searchChildren(currentNode):
                # print("A* found the path")
                break
        #print(self.closedList)
    
    # Based on current node, we search its possible children (feasible nodes) and add valid children to
    # the open list and do the loop until we reach the goal
    def searchChildren(self,currentNode):
        #print("searchChildren is running")
        numOfValidChildren = 0
        actions = [(deltaX,deltaY,gamma) for deltaX in [-0.1,0,0.1] for deltaY in [0,0.1] for gamma in [0,1]]
        
        for act in actions:
            actionValid,child = self.checkAction(currentNode,act)
            
            if actionValid == False:
                continue
            # Here we have found a valid child of the current node. We need to check if this child is 
            # already in the open list or closed list
            if child in self.closedList:
                continue
            elif child in self.openList:
                # Check if we need to change the child's parent
                if self.openList[child].g > self.getG(currentNode,child,act):
                    self.openList[child].parent = currentNode
                    self.openList[child].g = self.getG(currentNode,child,act)
                    self.openList[child].f = self.openList[child].g + self.openList[child].h
            else:
                self.openList[child] = AnymalStateNode(currentNode,self.getG(currentNode,child,act),self.getH(child))
                
            if child == self.goal:
                # print("Found the path")
                # print(child)
                self.closedList[child] = copy.deepcopy(self.openList[child])
                self.openList.pop(child)
                return True
        return False
        
        
        
    # This is the the heuristic of the A*. In this situation, it is the estmation distance between
    # the node and the goal. In theory, it should be less than the real distance which can help A* 
    # converge to the optimal result
    def getH(self,node):
        return math.sqrt((node[0] - self.goal[0])**2 + (node[1] - self.goal[1])**2) * self.k_h
    
    
    # This function finds the node with minumum score in the open list
    # Once it is found, it will be moved to the closed list, which means we stand on this node
    def moveToMinNode(self):
        # NOTE: use deepcopy to move elements from dict1 to dict2!! Otherwise, it is just a link
        minIndex = min(self.openList, key=lambda p: self.openList[p].f)
        self.closedList[minIndex] = copy.deepcopy(self.openList[minIndex])
        self.openList.pop(minIndex)
        return minIndex
    
    
    # This function checks if the action can guide the robot from current node to a feasible node in the map
    def checkAction(self,currentNode,action):
        valid = False
        child = ('nan','nan','nan','nan','nan')
        key = (round(currentNode[0]+action[0],2),round(currentNode[1]+action[1],2))
        if key in self.pointCloud.keys():
            if abs(currentNode[2] - self.pointCloud[key].z)<0.3:
                valid = True
                child = (key[0],key[1],self.pointCloud[key].z,0.0,action[2])          
        return valid, child
    
    
    # This function calculate the g value of the child when there is a movement from the parent to the child
    # When the action is once-touch, the score is just distance + covariance
    # If it is a two-touch action, we penalize the distance by 1.5 and decrease covariance score by 0.1
    # So it is a balance of fast touch and the risk
    def getG(self,parent,child,action):
        key = (child[0],child[1])
        distance = math.sqrt((parent[0] - child[0])**2 + (parent[1] - child[1])**2)
        if action[2] == 0:
            value = distance * self.k_d_false + self.k_sigma_false * self.pointCloud[key].sigma
        elif action[2] == 1:
            value = distance * self.k_d_true + self.k_sigma_true * self.pointCloud[key].sigma
        else:
            print("Warning!! invalid action")
        return self.closedList[parent].g + value
    
    # This function returns the optimal path from the start to the goal. Also, It returns the point of the path.
    # localTargetRatio: The position of the wanted point.
    #  1 --- start 
    #  0 --- goal  
    def getOptimalPath(self,localTargetRatio):
        parent = self.goal
        optimalPath = [parent]
        while parent != self.start:
            parent = self.closedList[parent].parent
            optimalPath.append(parent)
        localTarget = optimalPath[round(len(optimalPath)*localTargetRatio)]
        return optimalPath,localTarget
    
    
    # This function checks if the local target is too close to the edge
    def isCloseToEdge(self,node):
        X = np.arange(-0.5,0.5,0.1)
        Y = np.arange(-0.5,0.5,0.1)
        
        risk = 0.0
        counter0 = 0
        counter1 = 0
        counter2 = 0
        counter = 0
        
        for x in X:
            for y in Y:
                if (round(node[0]+x,1),round(node[1]+y,1)) in self.pointCloud.keys():
                    counter2 = counter2 + 1
                    if abs(self.pointCloud[round(node[0]+x,1),round(node[1]+y,1)].z - node[2]) > 0.5:
                        distance = math.sqrt(x**2 + y**2)
                        risk = risk + math.exp(-distance)
                        counter0 = counter0 + 1
                else:
                    # print(round(node[0]+x,1),round(node[1]+y,1))
                    distance = math.sqrt(x**2 + y**2)
                    risk = risk + math.exp(-distance)
                    counter1 = counter1 + 1
                counter = counter + 1
        return risk/counter
    
    def refineTarget(self,node):
        nodeRisk = self.isCloseToEdge(node)
        if nodeRisk > 0.15:
            refinedTarget = node
            minRisk = nodeRisk
            X = np.arange(-0.5,0.5,0.1)
            Y = np.arange(-0.5,0.5,0.1)
            for x in X:
                for y in Y:
                    possiblePoint = (round(node[0]+x,1),round(node[1]+y,1))
                    if possiblePoint in self.pointCloud.keys():
                        possibleNode = (possiblePoint[0],possiblePoint[1],self.pointCloud[possiblePoint].z,0.0,node[3]) 
                        if self.isCloseToEdge(possibleNode) < minRisk:
                            refinedTarget = possibleNode
                            minRisk = self.isCloseToEdge(possibleNode)
        else:
            refinedTarget = node
        return refinedTarget
                        
    
    
    def plotOptimalPath(self,fig):
        optimalPath,localTarget = self.getOptimalPath(0.8)
        optimalPathOneTouchArray = np.zeros((3,len(optimalPath)))
        optimalPathDoubleTouchArray = np.zeros((3,len(optimalPath)))
        pointerOneTouch = 0
        pointerDoubleTouch = 0
        for i, node in enumerate(optimalPath):
            if node[4] == 0:
                optimalPathOneTouchArray[0,pointerOneTouch] = node[0]
                optimalPathOneTouchArray[1,pointerOneTouch] = node[1]
                optimalPathOneTouchArray[2,pointerOneTouch] = node[2]
                pointerOneTouch = pointerOneTouch + 1
            elif node[4] == 1:
                optimalPathDoubleTouchArray[0,pointerDoubleTouch] = node[0]
                optimalPathDoubleTouchArray[1,pointerDoubleTouch] = node[1]
                optimalPathDoubleTouchArray[2,pointerDoubleTouch] = node[2]
                pointerDoubleTouch = pointerDoubleTouch + 1
        pointerOneTouch = pointerOneTouch -1
        pointerDoubleTouch = pointerDoubleTouch -1

        # plt.rcParams["figure.figsize"]=20,20
        # # fig = plt.figure(figNum)
        # # ax = fig.gca(projection='3d')

        self.terrain.plotPlanes(fig)

        mlab.points3d(optimalPathOneTouchArray[0,0:pointerOneTouch],optimalPathOneTouchArray[1,0:pointerOneTouch],optimalPathOneTouchArray[2,0:pointerOneTouch],scale_factor=0.1,color = (1,0,0),figure = fig)
        mlab.points3d(optimalPathDoubleTouchArray[0,0:pointerDoubleTouch],optimalPathDoubleTouchArray[1,0:pointerDoubleTouch],optimalPathDoubleTouchArray[2,0:pointerDoubleTouch],scale_factor=0.1,color = (0,1,0),figure = fig)


        # ax.set_xlim([0,10])
        # ax.set_ylim([0,20])
        # ax.set_zlim([0,5])


if __name__ == "__main__":
    start = (5.0,1.0)
    goal = (5.,18.)
    terrain = at.Terrain(0)
    anyAStar = AnymalAStarGlobal(start,goal,terrain)
    
    # testX = 1.0
    # testY = 1.5
    # print("The z of point (",testX,",",testY,") is ",anyAStar.pointCloud[(testX,testY)].z)
    
    time_start=time.time()
    success = anyAStar.run()
    time_end=time.time()
    print('A* Time:',time_end-time_start,'s')
    
    optimalPath, localTarget = anyAStar.getOptimalPath(0.7)
    print("The local target is", localTarget)
    
    # testTarget = (2.9, 6.0, 0.75, 0.0, 1)
    # risk = anyAStar.isCloseToEdge(testTarget)
    # print("The risk before refinement is ",risk)
    
    # refinedTarget = anyAStar.refineTarget(localTarget)
    # print("The target before refinement is ",testTarget,"The target after refinement is ",refinedTarget)
    
    # fig = plt.figure(1)
    # ax = fig.gca(projection='3d')
    # plt.rcParams["figure.figsize"]=20,20
    
    fig = mlab.figure(1)
    anyAStar.plotOptimalPath(fig)
    mlab.show()
    # terrain.plotPlanes(ax)
    # plt.show()