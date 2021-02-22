import sys
import os
sys.path.insert(0, os.path.abspath('../src'))
import numpy as np
import copy
import matplotlib.pyplot as plt
import math
import point_cloud as pc
from mayavi import mlab
import logging


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
        for deltaX in [-0.15,-0.1,0.0,0.1,0.15]:
            for deltaY in [-0.15,-0.1,0.0,0.1,0.15]:
                if deltaX ==0 and deltaY == 0:
                    continue
                else:
                    self.actions.append((deltaX,deltaY))


    def getPossibleActions(self):
        return self.actions


class AnymalAStar:
    def __init__(self, start, goal):
        self.pointCloud = pc.CentralLake()
        self.start = start
        self.goal = goal
        self.initialState = (start[0],0.,start[1],0.)
        self.openList = {}
        self.closedList = {}
        self.actions = ActionsBase()
        self.phaseTime = 0.2
        self.z = 0.43
        self.g = 9.81
        self.desired_vel = 0.5

        self.numSearchTimes = 0

        self.logger = logging.getLogger('debug')
        self.logger.setLevel(logging.INFO)
        fh = logging.FileHandler('anyplan.log')
        fh.setLevel(logging.INFO)
        formatter = logging.Formatter('%(message)s')
        fh.setFormatter(formatter)
        self.logger.addHandler(fh)
        self.logTimes = 2000

        print("The start node is {}".format(self.start))
        print("The goal node is {}".format(self.goal))

    def run(self):
        # add the start point to the open list. Its g = 0, h is the estimate distance towards the goal
        self.openList[self.start] = AnymalStateNode(self.start, 0, self.getH(self.start),self.initialState)

        numSearchTimes = 0

        while len(self.openList) > 0:
            # find the minimum element and stand on it. Then, search its children
            currentNode = self.moveToMinNode()
            self.numSearchTimes = self.numSearchTimes + 1
            if self.searchChildren(currentNode):
                print("A* found the path")
                break
        # print(self.closedList)

    # Based on current node, we search its possible children (feasible nodes) and add valid children to
    # the open list and do the loop until we reach the goal
    def searchChildren(self, currentNode):
        # print("searchChildren is running")
        child_num = -1

        if self.numSearchTimes < self.logTimes:
            self.logger.info("")
            self.logger.info("The parent is ({} from {})  g = {}, \
                    h = {}, f = {}".format(currentNode, \
                                           self.closedList[currentNode].parent, \
                                           self.closedList[currentNode].g, self.closedList[currentNode].h, \
                                           self.closedList[currentNode].f))

        for act in self.actions.getPossibleActions():
            child_num += 1

            actionValid, child = self.checkAction(currentNode, act)

            if not actionValid:
                if self.numSearchTimes < self.logTimes:
                    self.logger.info("The child {} ({}-{}) is invalid.) ".format(child,self.numSearchTimes,child_num))
                continue
            # Here we have found a valid child of the current node. We need to check if this child is
            # already in the open list or closed list
            if child in self.closedList:
                if self.numSearchTimes < self.logTimes:
                    self.logger.info("The child {} ({}-{}) is in closed list.) ".format(child,self.numSearchTimes,child_num))
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

                    if self.numSearchTimes < self.logTimes:
                        self.logger.info("The child is {} ({}-{}) . x_m, y_m =({},{}). v_x, v_y = ({},{})) \
                                            g = {} h = {} f = {}".format(child,self.numSearchTimes,child_num,\
                                                                         round(self.openList[child].x,2), round(self.openList[child].y,2), \
                                                                         round(self.openList[child].vx, 2),round(self.openList[child].vy, 2), \
                                                                         round(self.openList[child].g,2), \
                                                                         round(self.openList[child].h,2), \
                                                                         round(self.openList[child].f,2)))
                else:
                    if self.numSearchTimes < self.logTimes:
                        self.logger.info("The child {} ({}-{}). x_m, y_m =({},{}) v_x,v_y = ({},{}) has larger g = {}.) ".format(child, self.numSearchTimes,child_num, \
                                                                                                               round(self.openList[child].x,2),\
                                                                                                               round(self.openList[child].y,2),\
                                                                                                               round(self.openList[child].vx, 2) ,\
                                                                                                               round(self.openList[child].vy, 2) ,\
                                                                                                               round(child_g,2)))
            else:
                child_g, child_states = self.getG(currentNode, child, act)
                child_h = self.getH(child_states)
                self.openList[child] = AnymalStateNode(currentNode, child_g,child_h,child_states)

                if self.numSearchTimes < self.logTimes:
                        self.logger.info("The child is {} ({}-{}) . x_m, y_m =({},{}). v_x, v_y = ({},{})) \
                                            g = {} h = {} f = {}".format(child,self.numSearchTimes,child_num,\
                                                                         round(self.openList[child].x,2), round(self.openList[child].y,2), \
                                                                         round(self.openList[child].vx, 2),round(self.openList[child].vy, 2), \
                                                                         round(self.openList[child].g,2), \
                                                                         round(self.openList[child].h,2), \
                                                                         round(self.openList[child].f,2)))

            if self.isDone(child):
                print("Found the path")
                self.finalStep = child
                self.closedList[child] = copy.deepcopy(self.openList[child])
                self.openList.pop(child)
                return True

        return False

    # This is the the heuristic of the A*. In this situation, it is the estmation distance between
    # the node and the goal. In theory, it should be less than the real distance which can help A*
    # converge to the optimal result
    def getH(self, node_states):
        # return math.sqrt((node[0] - self.goal[0]) ** 2 + (node[1] - self.goal[1]) ** 2)
        return math.sqrt((node_states[0] - self.goal[0]) ** 2 + (node_states[2] - self.goal[1]) ** 2)

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
        child = (currentNode[0]+action[0],currentNode[1]+action[1],0)
        key = (round(currentNode[0] + action[0], 2), round(currentNode[1] + action[1], 2))
        if key in self.pointCloud.pc.keys():
            if abs(currentNode[2] - self.pointCloud.pc[key].z) < 0.2:
                valid = True
                child = (key[0], key[1], self.pointCloud.pc[key].z)
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

        value = distance + vel_cost

        return self.closedList[parent].f + value, (states_x[0],states_x[1],states_y[0],states_y[1])

    def isDone(self,child):
        if math.sqrt((self.openList[child].x - self.goal[0])**2 + (self.openList[child].y - self.goal[1])**2) < 0.5:
            return True
        else:
            return False

    def getOptimalPath(self):
        parent = self.finalStep
        optimalPath = [parent]
        while parent != self.start:
            parent = self.closedList[parent].parent
            optimalPath.append(parent)
        return optimalPath

    def plot_result(self):
        fig = mlab.figure(0)
        self.pointCloud.show_point_cloud(fig)
        self.optimalPath = self.getOptimalPath()

        for node in self.optimalPath:
            x = self.closedList[node].x
            y = self.closedList[node].y
            z = self.z
            # mlab.points3d(node[0],node[1],node[2],figure=fig)
            # mlab.points3d(x,y,z,figure=fig)
            x_line = [node[0],x]
            y_line = [node[1],y]
            z_line = [node[2],z]
            mlab.plot3d(x_line,y_line,z_line,figure=fig)
        mlab.show()


if __name__ == "__main__":
    start = (2.,1.0,0.)
    goal = (2.,5.0,0)
    anyAStar = AnymalAStar(start,goal)
    anyAStar.run()
    optimalPath = anyAStar.getOptimalPath()
    anyAStar.plot_result()

    temp = 1

