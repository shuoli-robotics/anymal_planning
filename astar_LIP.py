import sys
import os
print(sys.path)
sys.path.insert(0, os.path.abspath('../src'))
import numpy as np
import copy
import matplotlib.pyplot as plt
import math
import point_cloud as pc
from mayavi import mlab
import logging


class AnymalStateNode:
    def __init__(self,parentIndex,g,h,index):
        self.parent = parentIndex
        self.g = g
        self.h = h
        self.f = g + h
        self.index = index

class StanceStatus:
    def __init__(self):
        # Indicate how Anymal arrives at current status
        # 0 ----- stand still
        # 1 ----- LF and RH swing to current position, RF and LH keep static
        # 2 ----- LF and RH keep static, RF and LH swing to current position
        self.leg_status = 0

        # Positions of each end effector
        self.LF_pos = []
        self.RF_pos = []
        self.LH_pos = []
        self.RH_pos = []

        # Trajectories' coefficients from current phase to the next phase
        self.LF_coef = []
        self.RF_coef = []
        self.LH_coef = []
        self.RH_coef = []

        self.heading = 0.0


class ActionsBase:
    def __init__(self):
        self.actions = []
        for deltaX in [-0.3,-0.2,-0.15,-0.1,-0.05,0.0,0.05,0.1,0.15,0.2,0.3]:
            for deltaY in [-0.3,-0.2,-0.15,-0.1,-0.05,0.0,0.05,0.1,0.15,0.2,0.3]:
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
        self.trajectories = {}
        self.phaseTime = 0.3
        self.z = 0.43
        self.g = 9.81
        self.anymal_width = 0.45
        self.anymal_length = 0.70
        self.desired_vel = 0.4
        self.heading = 0.0

        self.numSearchTimes = 0
        self.child_num = -1

        # A* weights
        self.omega_distance = 1.0
        self.omega_h_distance = 4.0
        self.omega_speed = 0
        self.omega_time = 0

        self.logger = logging.getLogger('debug')
        self.logger.setLevel(logging.INFO)
        fh = logging.FileHandler('anyplan.log')
        fh.setLevel(logging.INFO)
        formatter = logging.Formatter('%(message)s')
        fh.setFormatter(formatter)
        self.logger.addHandler(fh)
        self.logTimes = 2000
        self.log_flag = False

        print("The start node is {}".format(self.start))
        print("The goal node is {}".format(self.goal))

    def run(self):
        # add the start point to the open list. Its g = 0, h is the estimate distance towards the goal
        self.openList[self.start] = AnymalStateNode(parentIndex=self.start, g=0, h=self.getH(self.start),index="0-0")
        print("Astar.run enter")
        numSearchTimes = 0

        while len(self.openList) > 0:
            # find the minimum element and stand on it. Then, search its children
            currentNode = self.moveToMinNode()

            if self.isDone(currentNode):
                print("Found the path")
                self.finalStep = currentNode
                break
            else:
                self.numSearchTimes = self.numSearchTimes + 1
                print("A* is running. searching ID is {}".format(self.numSearchTimes))
                self.searchChildren(currentNode)


        # print(self.closedList)

    # Based on current node, we search its possible children (feasible nodes) and add valid children to
    # the open list and do the loop until we reach the goal
    def searchChildren(self, currentNode):
        # print("searchChildren is running")
        self.child_num = -1



        if self.numSearchTimes < self.logTimes and self.log_flag:
            self.logger.info("")
            self.logger.info("The current node is {} from {}            g = {}, \
                    h = {}, f = {}".format(self.closedList[currentNode].index, \
                                           self.closedList[self.closedList[currentNode].parent].index, \
                                           round(self.closedList[currentNode].g,2), round(self.closedList[currentNode].h,2), \
                                           round(self.closedList[currentNode].f,2)))

        for act in self.actions.getPossibleActions():
            self.child_num += 1

            if self.numSearchTimes == 2 and self.child_num == 79:
                temp = 1

            action_valid, child, child_g = self.checkAction(currentNode, act)

            if not action_valid:
                if self.numSearchTimes < self.logTimes and self.log_flag:
                    self.logger.info("The child {}-{} is invalid.".format(self.numSearchTimes,self.child_num))
            else:
                # Here we have found a valid child of the current node. We need to check if this child is
                # already in the open list or closed list
                if child in self.closedList:
                    if self.numSearchTimes < self.logTimes and self.log_flag:
                        self.logger.info("The child {}-{} is in closed list.".format(self.numSearchTimes,self.child_num))
                    continue
                elif child in self.openList:
                    # Check if we need to change the child's parent
                    if self.openList[child].g > child_g:
                        self.openList[child].parent = currentNode
                        self.openList[child].g = child_g
                        self.openList[child].f = self.openList[child].g + self.openList[child].h
                        self.openList[child].index = str(self.numSearchTimes)+'-'+str(self.child_num)

                        if self.numSearchTimes < self.logTimes:
                            self.logger.info("The child {} has smaller g. x_zmp,y_zmp = ({},{}). x_m, y_m =({},{}). v_x, v_y = ({},{})) \
                                                g = {} h = {} f = {}".format(self.openList[child].index,\
                                                                             child[0],child[1],\
                                                                             round(child[3],2), round(child[5],2), \
                                                                             round(child[4], 2),round(child[6], 2), \
                                                                             round(self.openList[child].g,2), \
                                                                             round(self.openList[child].h,2), \
                                                                             round(self.openList[child].f,2)))
                    else:
                        if self.numSearchTimes < self.logTimes and self.log_flag:
                            self.logger.info("The child {}-{} has larger g".format(self.numSearchTimes,self.child_num))
                else:
                    child_h = self.getH(child)
                    index = str(self.numSearchTimes)+'-'+str(self.child_num)
                    self.openList[child] = AnymalStateNode(parentIndex=currentNode, g = child_g,h = child_h,index = index)

                    if self.numSearchTimes < self.logTimes and self.log_flag:
                            self.logger.info("The child is {}. x_zmp,y_zmp = ({},{}). x_m, y_m =({},{}). v_x, v_y = ({},{})) \
                                                g = {} h = {} f = {}".format(self.openList[child].index, \
                                                                             child[0], child[1], \
                                                                             round(child[3],2), round(child[5],2), \
                                                                             round(child[4], 2),round(child[6], 2), \
                                                                             round(self.openList[child].g,2), \
                                                                             round(self.openList[child].h,2), \
                                                                             round(self.openList[child].f,2)))

                # if self.isDone(child):
                #     print("Found the path")
                #     self.finalStep = child
                #     self.closedList[child] = copy.deepcopy(self.openList[child])
                #     self.openList.pop(child)
                #     return True

        return False

    # This is the the heuristic of the A*. In this situation, it is the estmation distance between
    # the node and the goal. In theory, it should be less than the real distance which can help A*
    # converge to the optimal result
    def getH(self, node):
        # node structure
        # 0         1       2     3   4   5   6
        # (zmp_x  zmp_y   zmp_z   x  v_x  y  v_y)

        remaining_distance = math.sqrt((node[3] - self.goal[0]) ** 2 + (node[5] - self.goal[1]) ** 2)

        return self.omega_h_distance * remaining_distance

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

        distance_mass_point = math.sqrt(
            (parent[3] - child[3]) ** 2 + (parent[5] - child[5]) ** 2)

        distance_zmp = math.sqrt((parent[0] - child[0]) ** 2 + (parent[1] - child[1]) ** 2)

        error_v0 = abs(math.sqrt(child[3] ** 2 + child[5] ** 2) - self.desired_vel)
        error_vf = abs(math.sqrt(child[4] ** 2 + child[6] ** 2) - self.desired_vel)
        sum_error_v = (error_v0 + error_vf) * self.phaseTime / 2.0

        delta_g = self.omega_distance * (distance_mass_point + distance_zmp) + self.omega_speed * sum_error_v
        # delta_g = self.omega_distance * distance

        pole_length_0 = math.sqrt((parent[3] - child[0])**2 + (parent[5] - child[1])**2)
        pole_length_f = math.sqrt((child[3] - child[0]) ** 2 + (child[5] - child[1]) ** 2)

        if self.numSearchTimes == 2 and self.child_num == 77:
            temp = 1


        if max(pole_length_0,pole_length_f) > 1.0 or max(abs(child[4]),abs(child[6])) > 0.6:
            valid  = False
        else:
            valid = True

        return self.closedList[parent].g + delta_g, child, valid


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
        if math.sqrt((child[0] - self.goal[0])**2 + (child[1] - self.goal[1])**2) < 0.2 and \
                math.sqrt((child[3] - self.goal[0])**2 + (child[5] - self.goal[1])**2) < 0.2 \
                and math.sqrt(child[4] **2 + child[6] **2) < 0.1:
            return True
        else:
            return False

    def getOptimalPath(self):
        parent = self.finalStep
        optimalPath = [parent]
        while parent != self.start:
            parent = self.closedList[parent].parent
            optimalPath.append(parent)

        for i, node in reversed(list(enumerate(optimalPath))):
            print(i)  # len - i -1
            print(node)
        return optimalPath

    def generate_EE_trajectory(self):

        optimalPath = self.getOptimalPath()
        n = len(optimalPath)
        for i, node in reversed(list(enumerate(optimalPath))):
            index = n-i-1
            self.trajectories[index] = StanceStatus()
            if index == 0:


            else:
                if self.trajectories[index-1].leg_status == 0:
                    self.trajectories[index - 1].leg_status = 1
                elif self.trajectories[index-1].leg_status == 1:
                    self.trajectories[index - 1].leg_status = 2
                elif self.trajectories[index-1].leg_status == 2:
                    self.trajectories[index - 1].leg_status = 1

    def generate_footholds_for_all_EEs(self):
        self.trajectories[index].leg_status = 0
        R_E_B = np.array(
            [[math.cos(self.heading), math.sin(self.heading)], [-math.sin(self.heading), math.cos(self.heading)]])
        LF_pos_xy = R_E_B.dot(np.array([self.anymal_length / 2.0, self.anymal_width / 2.0]))
        LF_pos_xy = LF_pos_xy + np.array([node[0], node[1]])
        LF_pos_xy = np.around(LF_pos_xy, 1)
        self.trajectories[index].LF_pos = np.concatenate(
            (LF_pos_xy, np.array([self.pointCloud.pc[(LF_pos_xy[0], LF_pos_xy[1])].z])))

        RF_pos_xy = np.around(R_E_B.dot(np.array([self.anymal_length / 2.0, -self.anymal_width / 2.0])), 1)
        RF_pos_xy = RF_pos_xy + np.array([node[0], node[1]])
        self.trajectories[index].RF_pos = np.concatenate(
            (RF_pos_xy, np.array([self.pointCloud.pc[(RF_pos_xy[0], RF_pos_xy[1])].z])))

        LH_pos_xy = np.around(R_E_B.dot(np.array([-self.anymal_length / 2.0, self.anymal_width / 2.0])), 1)
        LH_pos_xy = LH_pos_xy + np.array([node[0], node[1]])
        self.trajectories[index].LH_pos = np.concatenate(
            (LH_pos_xy, np.array([self.pointCloud.pc[(LH_pos_xy[0], LH_pos_xy[1])].z])))

        RH_pos_xy = np.around(R_E_B.dot(np.array([-self.anymal_length / 2.0, -self.anymal_width / 2.0])), 1)
        RH_pos_xy = RH_pos_xy + np.array([node[0], node[1]])
        self.trajectories[index].RH_pos = np.concatenate(
            (RH_pos_xy, np.array([self.pointCloud.pc[(RH_pos_xy[0], RH_pos_xy[1])].z])))


                test = 1



    def plot_result(self):
        fig1, ax = plt.subplots(2,2)
        fig2, ax2 = plt.subplots(1, 2)

        fig = mlab.figure(0)
        self.pointCloud.show_point_cloud(fig)
        self.optimalPath = self.getOptimalPath()

        zmp_x = np.zeros(len(self.optimalPath))
        zmp_y = np.zeros(len(self.optimalPath))
        mass_point_x = np.zeros(len(self.optimalPath))
        mass_point_y = np.zeros(len(self.optimalPath))
        mass_point_vx = np.zeros(len(self.optimalPath))
        mass_point_vy = np.zeros(len(self.optimalPath))

        for i,node in enumerate(self.optimalPath):
            x = node[3]
            y = node[5]
            z = self.z
            # mlab.points3d(node[0],node[1],node[2],figure=fig)
            # mlab.points3d(x,y,z,figure=fig)
            x_line = [node[0],x]
            y_line = [node[1],y]
            z_line = [node[2],z]

            mass_point_x[len(self.optimalPath)-1-i] = x
            mass_point_y[len(self.optimalPath)-1-i] = y
            mass_point_vx[len(self.optimalPath)-1-i] = node[4]
            mass_point_vy[len(self.optimalPath)-1-i] = node[6]
            zmp_x[len(self.optimalPath)-1-i] = node[0]
            zmp_y[len(self.optimalPath)-1-i] = node[1]
            mlab.plot3d(x_line,y_line,z_line,figure=fig)

        zmp_x_open_list = np.zeros(len(self.openList))
        zmp_y_open_list = np.zeros(len(self.openList))
        zmp_x_closed_list = np.zeros(len(self.closedList))
        zmp_y_closed_list = np.zeros(len(self.closedList))
        for i,node in enumerate(self.openList):
            zmp_x_open_list[i] = node[0]
            zmp_y_open_list[i] = node[1]

        for i,node in enumerate(self.closedList):
            zmp_x_closed_list[i] = node[0]
            zmp_y_closed_list[i] = node[1]

        ax[0,0].plot(mass_point_x,label='mass point')
        ax[0,0].plot(zmp_x,'^',label='zmp')
        ax[0,0].set_ylabel('x[m]')

        ax[0, 1].plot(mass_point_y)
        ax[0, 1].plot(zmp_y, '^')
        ax[0, 1].set_ylabel('y[m]')

        ax[1,0].plot(mass_point_vx,label='mass point')
        ax[1,0].set_ylabel('v_x[m/s]')
        ax[1,0].set_xlabel('# of step')

        ax[1,1].plot(mass_point_vy,label='mass point')
        ax[1,1].set_ylabel('v_y[m/s]')
        ax[1,1].set_xlabel('# of step')
        ax[0,0].legend(loc='best')

        ax2[0].plot(zmp_x_open_list,zmp_y_open_list,'gs')
        ax2[1].plot(zmp_x_closed_list, zmp_y_closed_list, 'rs')
        ax2[0].set_title('open list')
        ax2[0].set_xlabel('x[m]')
        ax2[0].set_ylabel('y[m]')
        ax2[1].set_title('closed list')
        ax2[1].set_xlabel('x[m]')
        plt.show()
        mlab.show()


if __name__ == "__main__":
    zmp_0 = (2.0,1.0,0.)
    zmp_f = (9.0,9,0)
    anyAStar = AnymalAStar(zmp_0,zmp_f)
    anyAStar.run()
    optimalPath = anyAStar.getOptimalPath()
    anyAStar.generate_EE_trajectory()
    anyAStar.plot_result()

