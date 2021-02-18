import sys
import os
sys.path.insert(0, os.path.abspath('../src'))
import numpy as np
import copy
import matplotlib.pyplot as plt
import math
import point_cloud as pc
from mayavi import mlab


class AnymalStateNode:
    def __init__(self,parentIndex,g,h,states):
        self.parent = parentIndex
        self.g = g
        self.h = h
        self.f = g + h

        self.x = states[0]
        self.vx = states[1]
        self.y = states[2]
        self.vy = states[3]


class ActionsBase:
    def __init__(self):
        self.actions = []
        for deltaX in [-0.2,-0.1,0.0,0.1,0.2]:
            for deltaY in [-0.2,-0.1,0.0,0.1,0.2]:
                if deltaX ==0 and deltaY == 0:
                    continue
                else:
                    self.actions.append((deltaX,deltaY))


    def getPossibleActions(self):
        return self.actions


class AnymalAStar:
    def __init__(self, start, goal):
        self.pointCloud = pc.CentralLake().pc
        self.start = start
        self.goal = goal
        self.initialState = (start[0],0.,start[1],0.)
        self.openList = {}
        self.closedList = {}
        self.actions = ActionsBase()
        self.phaseTime = 0.5
        self.z = 0.43
        self.g = 9.81
        self.desired_vel = 0.5
        print("The start node is {}".format(self.start))
        print("The goal node is {}".format(self.goal))

    def run(self):
        # add the start point to the open list. Its g = 0, h is an estimate distance towards the goal
        self.openList[self.start] = AnymalStateNode(self.start, 0, self.getH(self.start),self.initialState)

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

        for act in self.actions.getPossibleActions():
            actionValid, child = self.checkAction(currentNode, act)

            if not actionValid:
                continue
            # Here we have found a valid child of the current node. We need to check if this child is
            # already in the open list or closed list
            if child in self.closedList:
                continue
            elif child in self.openList:
                # Check if we need to change the child's parent
                child_g,child_states = self.getG(currentNode, child, act)
                if self.openList[child].g > child_g:
                    self.openList[child].parent = currentNode
                    self.openList[child].g = child_g
                    self.openList[child].f = self.openList[child].g + self.openList[child].h
                    self.openList[child].x = child_states[0]
                    self.openList[child].vx = child_states[1]
                    self.openList[child].y = child_states[2]
                    self.openList[child].vy = child_states[3]


            else:
                child_g, child_states = self.getG(currentNode, child, act)
                self.openList[child] = AnymalStateNode(currentNode, child_g,self.getH(child),child_states)

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
        child = ('nan','nan','nan')
        key = (round(currentNode[0] + action[0], 2), round(currentNode[1] + action[1], 2))
        if key in self.pointCloud.keys():
            if abs(currentNode[2] - self.pointCloud[key].z) < 0.2:
                valid = True
                child = (key[0], key[1], self.pointCloud[key].z)
        return valid, child

    # This function calculate the g value of the child when there is a movement from the parent to the child
    # When the action is once-touch, the score is just distance + covariance
    # If it is a two-touch action, we penalize the distance by 1.5 and decrease covariance score by 0.1
    # So it is a balance of fast touch and the risk
    def getG(self, parent, child, action):
        self.g = 9.81
        self.z = 0.43
        p_x = child[0]
        p_y = child[1]
        omega = math.sqrt(self.g / self.z)
        A = np.array([[np.cosh(omega * self.phaseTime), 1 / omega * np.sinh(omega * self.phaseTime)],
                      [omega * np.sinh(omega * self.phaseTime), np.cosh(omega * self.phaseTime)]])
        B = np.array([1 - np.cosh(omega * self.phaseTime), -omega * np.sinh(omega * self.phaseTime)])

        x0 = np.array([self.closedList[parent].x, self.closedList[parent].vx])
        y0 = np.array([self.closedList[parent].y, self.closedList[parent].vy])
        states_x = A.dot(x0) + B * p_x
        states_y = A.dot(y0) + B * p_y

        distance = math.sqrt((self.closedList[parent].x - states_x[0]) ** 2 + (self.closedList[parent].y - states_y[0]) ** 2)

        error_v0 =  abs(math.sqrt(self.closedList[parent].vx**2 + self.closedList[parent].vy**2) - self.desired_vel)
        error_vf = abs(math.sqrt(states_x[1] ** 2 + states_y[1] ** 2) - self.desired_vel)
        vel_cost = (error_v0 + error_vf)*self.phaseTime/2.0

        value = distance

        return self.closedList[parent].f + value, (states_x[0],states_x[1],states_y[0],states_y[1])

    def getOptimalPath(self):
        parent = self.goal
        optimalPath = [parent]
        while parent != self.start:
            parent = self.closedList[parent].parent
            optimalPath.append(parent)
        return optimalPath

    def plot_result(self):
        fig = mlab.figure(0)
        self.pointCloud.show_point_cloud(fig)

start = (2.,1.,0.)
goal = (2.,2.,0.)
anyAStar = AnymalAStar(start,goal)
anyAStar.run()


optimalPath = anyAStar.getOptimalPath()

temp = 1
#
# optimalPathOneTouchArray = np.zeros((3,len(optimalPath)))
# optimalPathDoubleTouchArray = np.zeros((3,len(optimalPath)))
# pointerOneTouch = 0
# pointerDoubleTouch = 0
# for i, node in enumerate(optimalPath):
#     if node[4] == 0:
#         optimalPathOneTouchArray[0,pointerOneTouch] = node[0]
#         optimalPathOneTouchArray[1,pointerOneTouch] = node[1]
#         optimalPathOneTouchArray[2,pointerOneTouch] = node[2]
#         pointerOneTouch = pointerOneTouch + 1
#     elif node[4] == 1:
#         optimalPathDoubleTouchArray[0,pointerDoubleTouch] = node[0]
#         optimalPathDoubleTouchArray[1,pointerDoubleTouch] = node[1]
#         optimalPathDoubleTouchArray[2,pointerDoubleTouch] = node[2]
#         pointerDoubleTouch = pointerDoubleTouch + 1
# pointerOneTouch = pointerOneTouch -1
# pointerDoubleTouch = pointerDoubleTouch -1
