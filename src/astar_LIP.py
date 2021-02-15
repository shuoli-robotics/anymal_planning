import sys
import os
sys.path.insert(0, os.path.abspath('../src'))
import numpy as np
import copy
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d
import math
import anyterrain as at
import point_cloud as pc







# def generatePointCloud(terrainPlanes):
#     X = np.arange(0, 10, 0.1)
#     Y = np.arange(0, 20, 0.1)
#     X = np.append(X, 10)
#     Y = np.append(Y, 20)
#     pointCloud = {}
#
#     for x in X:
#         for y in Y:
#             x = round(x, 2)
#             y = round(y, 2)
#             for i in range(terrainPlanes.shape[0]):
#                 if terrainPlanes[i, 4] <= x < terrainPlanes[i, 5] and terrainPlanes[i, 6] <= y < terrainPlanes[i, 7]:
#                     pointCloud[(x, y)] = Point(
#                         (-terrainPlanes[i, 3] - terrainPlanes[i, 0] * x - terrainPlanes[i, 1] * y) / terrainPlanes[
#                             i, 2], terrainPlanes[i, 8])
#                     break
#     return pointCloud



pointCloud = pc.PointCloud(0)




# pointCloud = generatePointCloud(terrain.terrainPlanes)

class AnymalStateNode:
    def __init__(self,parentIndex,g,h):
        self.parent = parentIndex
        self.g = g
        self.h = h
        self.f = g + h


class AnymalAStar:
    def __init__(self, pointCloud, start, goal):
        self.pointCloud = pointCloud
        self.start = start
        self.goal = goal
        self.openList = {}
        self.closedList = {}
        print("The start node is {}".format(self.start))
        print("The goal node is {}".format(self.goal))

    def run(self):
        # add the start point to the open list. Its g = 0, h is an estimate distance towards the goal
        self.openList[self.start] = AnymalStateNode(self.start, 0, self.getH(self.start))

        numSearchTimes = 0

        while len(self.openList) > 0:
            # find the minimum element and stand on it. Then, search its children
            currentNode = self.moveToMinNode()
            numSearchTimes = numSearchTimes + 1
            if self.searchChildren(currentNode):
                print("A* found the path")
                break
        # print(self.closedList)

    # Based on current node, we search its possible children (feasible nodes) and add valid children to
    # the open list and do the loop until we reach the goal
    def searchChildren(self, currentNode):
        # print("searchChildren is running")
        numOfValidChildren = 0
        actions = [(deltaX, deltaY, gamma) for deltaX in [-0.1, 0, 0.1] for deltaY in [0.1, 0.2] for gamma in [0, 1]]

        for act in actions:
            actionValid, child = self.checkAction(currentNode, act)

            if actionValid == False:
                continue
            # Here we have found a valid child of the current node. We need to check if this child is
            # already in the open list or closed list
            if child in self.closedList:
                continue
            elif child in self.openList:
                # Check if we need to change the child's parent
                if self.openList[child].g > self.getG(currentNode, child, act):
                    self.openList[child].parent = currentNode
                    self.openList[child].g = self.getG(currentNode, child, act)
                    self.openList[child].f = self.openList[child].g + self.openList[child].h
            else:
                self.openList[child] = AnymalStateNode(currentNode, self.getG(currentNode, child, act),
                                                       self.getH(child))

            if child == self.goal:
                print("Found the path")
                print(child)
                self.closedList[child] = copy.deepcopy(self.openList[child])
                self.openList.pop(child)
                return True
        return False

    # This is the the heuristic of the A*. In this situation, it is the estmation distance between
    # the node and the goal. In theory, it should be less than the real distance which can help A*
    # converge to the optimal result
    def getH(self, node):
        return math.sqrt((node[0] - self.goal[0]) ** 2 + (node[1] - self.goal[1]) ** 2)

    # This function finds the node with minumum score in the open list
    # Once it is found, it will be moved to the closed list, which means we stand on this node
    def moveToMinNode(self):
        # NOTE: use deepcopy to move elements from dict1 to dict2!! Otherwise, it is just a link
        minIndex = min(self.openList, key=lambda p: self.openList[p].f)
        self.closedList[minIndex] = copy.deepcopy(self.openList[minIndex])
        self.openList.pop(minIndex)
        return minIndex

    # This function checks if the action can guide the robot from current node to a feasible node in the map
    def checkAction(self, currentNode, action):
        valid = False
        child = ('nan', 'nan', 'nan', 'nan', 'nan')
        key = (round(currentNode[0] + action[0], 2), round(currentNode[1] + action[1], 2))
        if key in self.pointCloud.keys():
            if abs(currentNode[2] - self.pointCloud[key].z) < 0.3:
                valid = True
                child = (key[0], key[1], self.pointCloud[key].z, 0.0, action[2])
        return valid, child

    # This function calculate the g value of the child when there is a movement from the parent to the child
    # When the action is once-touch, the score is just distance + covariance
    # If it is a two-touch action, we penalize the distance by 1.5 and decrease covariance score by 0.1
    # So it is a balance of fast touch and the risk
    def getG(self, parent, child, action):
        key = (child[0], child[1])
        distance = math.sqrt((parent[0] - child[0]) ** 2 + (parent[1] - child[1]) ** 2)
        if action[2] == 0:
            value = distance + self.pointCloud[key].sigma
        elif action[2] == 1:
            value = distance * 1.5 + self.pointCloud[key].sigma * 0.1
        else:
            print("Warning!! invalid action")
        return self.closedList[parent].f + value

    def getOptimalPath(self):
        parent = self.goal
        optimalPath = [parent]
        while parent != self.start:
            parent = self.closedList[parent].parent
            optimalPath.append(parent)
        return optimalPath

start = (0.,0.,0.,0.,0.)
goal = (2.,18.,3.,0.,0.)
anyAStar = AnymalAStar(pointCloud,start,goal)
anyAStar.run()
optimalPath = anyAStar.getOptimalPath()

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

plt.rcParams["figure.figsize"]=20,20
fig = plt.figure(2)
ax = fig.gca(projection='3d')

terrain.plotPlanes(ax,terrain.terrainPlanes)

ax.scatter(optimalPathOneTouchArray[0,0:pointerOneTouch],optimalPathOneTouchArray[1,0:pointerOneTouch],optimalPathOneTouchArray[2,0:pointerOneTouch],color = 'red',s=40)
ax.scatter(optimalPathDoubleTouchArray[0,0:pointerDoubleTouch],optimalPathDoubleTouchArray[1,0:pointerDoubleTouch],optimalPathDoubleTouchArray[2,0:pointerDoubleTouch],color = 'green',s=40)


ax.set_xlim([0,10])
ax.set_ylim([0,20])
ax.set_zlim([0,5])
plt.show()