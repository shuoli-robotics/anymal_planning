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
    def __init__(self,parentIndex,g,h,time):
        self.parent = parentIndex
        self.g = g
        self.h = h
        self.f = g + h
        self.time = time


class ActionsBase:
    def __init__(self):
        self.actions = []
        for deltaX in [0]:
            for deltaY in [-0.3,-0.25,-0.2,-0.15,-0.1,-0.05,0.0,0.05,0.1,0.15,0.2,0.25,0.3]:
                if deltaX ==0 and deltaY == 0:
                    continue
                else:
                    self.actions.append((deltaX,deltaY))


    def getPossibleActions(self):
        return self.actions


class AnymalAStar:
    def __init__(self, zmp_0, zmp_f):
        self.pointCloud = pc.CentralLake()
        self.start = zmp_0+(round(zmp_0[0],1),round(0.,1),round(zmp_0[1],1),round(0.,1))
        self.goal = zmp_f
        self.openList = {}
        self.closedList = {}
        self.actions = ActionsBase()
        self.phaseTime = 0.2
        self.z = 0.43
        self.g = 9.81
        self.desired_vel = 0.5

        self.numSearchTimes = 0

        # A* weights
        self.omega_g_distance = 1.0
        self.omega_g_speed = 1.0
        self.omega_h_distance = 1.0
        self.omega_h_time = 1.0

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
        self.openList[self.start] = AnymalStateNode(parentIndex=self.start, g=0, h=self.getH(self.start),time=0.0)

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

            action_valid, child, child_g = self.checkAction(currentNode, act)

            if not action_valid:
                if self.numSearchTimes < self.logTimes:
                    self.logger.info("The child {} ({}-{}) is invalid.) ".format(child,self.numSearchTimes,child_num))
            else:
                # Here we have found a valid child of the current node. We need to check if this child is
                # already in the open list or closed list
                if child in self.closedList:
                    if self.numSearchTimes < self.logTimes:
                        self.logger.info("The child {} ({}-{}) is in closed list.) ".format(child,self.numSearchTimes,child_num))
                    continue
                elif child in self.openList:
                    # Check if we need to change the child's parent
                    if self.openList[child].g > child_g:
                        self.openList[child].parent = currentNode
                        self.openList[child].g = child_g
                        self.openList[child].f = self.openList[child].g + self.openList[child].h
                        self.openList[child].time = self.closedList[currentNode].time + self.phaseTime

                        if self.numSearchTimes < self.logTimes:
                            self.logger.info("The child is {} ({}-{}) . x_m, y_m =({},{}). v_x, v_y = ({},{})) \
                                                g = {} h = {} f = {}".format(child,self.numSearchTimes,child_num,\
                                                                             round(child[3],2), round(child[5],2), \
                                                                             round(child[4], 2),round(child[6], 2), \
                                                                             round(self.openList[child].g,2), \
                                                                             round(self.openList[child].h,2), \
                                                                             round(self.openList[child].f,2)))
                    else:
                        if self.numSearchTimes < self.logTimes:
                            self.logger.info("The child is {} ({}-{}) . x_m, y_m =({},{}). v_x, v_y = ({},{})) \
                                                g = {} h = {} f = {}".format(child,self.numSearchTimes,child_num,\
                                                                             round(child[3],2), round(child[5],2), \
                                                                             round(child[4], 2),round(child[6], 2), \
                                                                             round(self.openList[child].g,2), \
                                                                             round(self.openList[child].h,2), \
                                                                             round(self.openList[child].f,2)))
                else:
                    child_h = self.getH(child)
                    time = self.closedList[currentNode].time + self.phaseTime
                    self.openList[child] = AnymalStateNode(parentIndex=currentNode, g = child_g,h = child_h,time = time)

                    if self.numSearchTimes < self.logTimes:
                            self.logger.info("The child is {} ({}-{}) . x_m, y_m =({},{}). v_x, v_y = ({},{})) \
                                                g = {} h = {} f = {}".format(child,self.numSearchTimes,child_num,\
                                                                             round(child[3],2), round(child[5],2), \
                                                                             round(child[4], 2),round(child[6], 2), \
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
    def getH(self, node):
        # node structure
        # 0         1       2     3   4   5   6
        # (zmp_x  zmp_y   zmp_z   x  v_x  y  v_y)

        remaining_distance = math.sqrt((node[3] - self.goal[0]) ** 2 + (node[5] - self.goal[1]) ** 2)

        velocity = math.sqrt(node[4]**2 + node[6]**2)
        if abs(velocity) < 0.01:
            velocity = 0.01

        remaining_time = remaining_distance / velocity

        # todo: remaining time is wrong.

        return self.omega_h_distance * remaining_distance + self.omega_h_time * remaining_time

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
        # node structure
        # 0         1       2     3   4   5   6
        # (zmp_x  zmp_y   zmp_z   x  v_x  y  v_y)

        valid = False
        child = ()
        child_g = 9999
        key = (round(currentNode[0] + action[0], 2), round(currentNode[1] + action[1], 2))
        if key in self.pointCloud.pc.keys() and abs(currentNode[2] - self.pointCloud.pc[key].z) < 0.2:
            next_zmp = (key[0], key[1],self.pointCloud.pc[key].z)
            child_g,child,valid = self.calc_mass_point_state(currentNode,next_zmp)
        return valid, child, child_g

    def calc_mass_point_state(self,parent,next_zmp):
        # next zmp
        #   0       1       2
        # zmp_x   zmp_y   zmp_z

        self.g = 9.81
        self.z = 0.43
        p_x = next_zmp[0]
        p_y = next_zmp[1]
        omega = math.sqrt(self.g / self.z)
        A = np.array([[np.cosh(omega * self.phaseTime), 1 / omega * np.sinh(omega * self.phaseTime)],
                      [omega * np.sinh(omega * self.phaseTime), np.cosh(omega * self.phaseTime)]])
        B = np.array([1 - np.cosh(omega * self.phaseTime), -omega * np.sinh(omega * self.phaseTime)])

        x0 = np.array([parent[3], parent[4]])
        y0 = np.array([parent[5], parent[6]])
        states_x = A.dot(x0) + B * p_x
        states_y = A.dot(y0) + B * p_y

        child = next_zmp + (round(states_x[0],1), round(states_x[1],1), round(states_y[0],1), round(states_y[1],1))

        distance = math.sqrt(
            (parent[3] - child[3]) ** 2 + (parent[5] - child[5]) ** 2)

        error_vf = abs(math.sqrt(child[4] ** 2 + child[6] ** 2) - self.desired_vel)

        delta_g = self.omega_g_distance * distance + self.omega_g_speed * error_vf

        pole_length_0 = math.sqrt((parent[3] - child[0])**2 + (parent[5] - child[1])**2)
        pole_length_f = math.sqrt((child[3] - child[0]) ** 2 + (child[5] - child[1]) ** 2)



        if max(pole_length_0,pole_length_f) > 3 or max(abs(child[4]),abs(child[6])) > 1.5:
            valid  = False
        else:
            valid = True

        return self.closedList[parent].f + delta_g, child, valid


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
        if math.sqrt((child[3] - self.goal[0])**2 + (child[5] - self.goal[1])**2) < 0.5:
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
            x = node[3]
            y = node[5]
            z = self.z
            # mlab.points3d(node[0],node[1],node[2],figure=fig)
            # mlab.points3d(x,y,z,figure=fig)
            x_line = [node[0],x]
            y_line = [node[1],y]
            z_line = [node[2],z]
            mlab.plot3d(x_line,y_line,z_line,figure=fig)
        mlab.show()


if __name__ == "__main__":
    zmp_0 = (2.,1.0,0.)
    zmp_f = (2.,5.0,0)
    anyAStar = AnymalAStar(zmp_0,zmp_f)
    anyAStar.run()
    optimalPath = anyAStar.getOptimalPath()
    anyAStar.plot_result()

    temp = 1

