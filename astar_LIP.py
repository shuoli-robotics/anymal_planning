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
from enum import Enum
from scipy.interpolate import CubicHermiteSpline


class AnymalStateNode:
    def __init__(self,parentIndex,g,h,index):
        self.parent = parentIndex
        self.g = g
        self.h = h
        self.f = g + h
        self.index = index

class LegStatus(Enum):
    STAND_STILL = 1   # stand still
    MAJOR_DIAGONAL = 2 # LF and RH swing to current position, RF and LH keep static
    MINOR_DIAGONAL = 3 # LF and RH keep static, RF and LH swing to current position

class FootholdStatus:
    def __init__(self):
        self.leg_status = LegStatus.STAND_STILL

        # Positions of each end effector
        self.LF_pos = []
        self.RF_pos = []
        self.LH_pos = []
        self.RH_pos = []

        self.heading = 0.0

        self.lip_states = []



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
        self.footholds = {}
        self.phaseTime = 0.3
        self.z = 0.43
        self.g = 9.81

        self.anymal_length = 0.70
        self.anymal_width = 0.45
        self.LF_b = np.array([self.anymal_length/2, self.anymal_width/2])
        self.RF_b = np.array([self.anymal_length/2, -self.anymal_width/2])
        self.LH_b = np.array([-self.anymal_length / 2, self.anymal_width / 2])
        self.RH_b = np.array([-self.anymal_length / 2, -self.anymal_width / 2])


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

        # for i, node in reversed(list(enumerate(optimalPath))):
        #     print(i)  # len - i -1
        #     print(node)
        return optimalPath

    def generate_EE_trajectory(self):

        optimalPath = self.getOptimalPath()
        n = len(optimalPath)

        # generate footholds on terrain considering gaits
        for i, node in reversed(list(enumerate(optimalPath))):
            index = n-i-1
            self.footholds[index] = FootholdStatus()
            self.footholds[index].lip_states = node
            if index == 0:
                self.footholds[index].LF_pos , self.footholds[index].RH_pos  = self.generate_footholds_for_major_diagonal_EEs(node)
                self.footholds[index].RF_pos , self.footholds[index].LH_pos = self.generate_footholds_for_minor_diagonal_EEs(node)

            else:
                # pass
                if self.footholds[index-1].leg_status == LegStatus.STAND_STILL:
                    self.footholds[index].leg_status = LegStatus.MAJOR_DIAGONAL
                    self.footholds[index].LF_pos , self.footholds[index].RH_pos = self.generate_footholds_for_major_diagonal_EEs(node)
                    self.footholds[index].RF_pos = self.footholds[index-1].RF_pos
                    self.footholds[index].LH_pos = self.footholds[index - 1].LH_pos

                elif self.footholds[index-1].leg_status == LegStatus.MAJOR_DIAGONAL:
                    self.footholds[index].leg_status = LegStatus.MINOR_DIAGONAL
                    self.footholds[index].RF_pos , self.footholds[index].LH_pos = self.generate_footholds_for_minor_diagonal_EEs(node)
                    self.footholds[index].LF_pos = self.footholds[index-1].LF_pos
                    self.footholds[index].RH_pos = self.footholds[index - 1].RH_pos

                elif self.footholds[index-1].leg_status == LegStatus.MINOR_DIAGONAL:
                    self.footholds[index].leg_status = LegStatus.MAJOR_DIAGONAL
                    self.footholds[index].LF_pos , self.footholds[index].RH_pos = self.generate_footholds_for_major_diagonal_EEs(node)
                    self.footholds[index].RF_pos = self.footholds[index-1].RF_pos
                    self.footholds[index].LH_pos = self.footholds[index - 1].LH_pos




        # generate trajectories with the position constraints (footholds) above

        mass_center_x_const = np.zeros(len(self.footholds))
        mass_center_vx_const = np.zeros(mass_center_x_const.shape)
        mass_center_y_const = np.zeros(mass_center_x_const.shape)
        mass_center_vy_const = np.zeros(mass_center_x_const.shape)
        mass_center_z_const = np.zeros(mass_center_x_const.shape)
        mass_center_vz_const = np.zeros(mass_center_x_const.shape)

        lf_x_const = np.zeros(len(self.footholds))
        lf_y_const = np.zeros(lf_x_const.shape)
        lf_z_const = np.zeros(len(self.footholds)*2-1)

        rf_x_const = np.zeros(lf_x_const.shape)
        rf_y_const = np.zeros(lf_x_const.shape)
        rf_z_const = np.zeros(2*len(self.footholds)-1)

        lh_x_const = np.zeros(lf_x_const.shape)
        lh_y_const = np.zeros(lf_x_const.shape)
        lh_z_const = np.zeros(2*len(self.footholds)-1)

        rh_x_const = np.zeros(lf_x_const.shape)
        rh_y_const = np.zeros(lf_x_const.shape)
        rh_z_const = np.zeros(2*len(self.footholds)-1)

        time = np.zeros(lf_x_const.shape)
        time_z = np.zeros(len(self.footholds)*2-1)

        for i in self.footholds:
            time[i] = i*self.phaseTime

            mass_center_x_const[i] = self.footholds[i].lip_states[3]
            mass_center_vx_const[i] = self.footholds[i].lip_states[4]
            mass_center_y_const[i] = self.footholds[i].lip_states[5]
            mass_center_vy_const[i] = self.footholds[i].lip_states[6]
            mass_center_z_const[i] = self.z


            lf_x_const[i] = self.footholds[i].LF_pos[0]
            lf_y_const[i] = self.footholds[i].LF_pos[1]
            rf_x_const[i] = self.footholds[i].RF_pos[0]
            rf_y_const[i] = self.footholds[i].RF_pos[1]
            lh_x_const[i] = self.footholds[i].LH_pos[0]
            lh_y_const[i] = self.footholds[i].LH_pos[1]
            rh_x_const[i] = self.footholds[i].RH_pos[0]
            rh_y_const[i] = self.footholds[i].RH_pos[1]

            # assign values for z direction
            time_z[i * 2] = i * self.phaseTime
            lf_z_const[i * 2] = self.footholds[i].LF_pos[2]
            rf_z_const[i * 2] = self.footholds[i].RF_pos[2]
            lh_z_const[i * 2] = self.footholds[i].LH_pos[2]
            rh_z_const[i * 2] = self.footholds[i].RH_pos[2]
            if i != 0:
                time_z[i * 2-1] = i * self.phaseTime - 0.5 * self.phaseTime
                if self.footholds[i].leg_status == LegStatus.MAJOR_DIAGONAL:
                    lf_z_const[i * 2 -1] = lf_z_const[i * 2] + 0.08
                    rf_z_const[i * 2 -1] = rf_z_const[i * 2]
                    lh_z_const[i * 2 -1] = lh_z_const[i * 2] + 0.08
                    rh_z_const[i * 2 -1] = rh_z_const[i * 2]
                elif self.footholds[i].leg_status == LegStatus.MINOR_DIAGONAL:
                    lf_z_const[i * 2 -1] = lf_z_const[i * 2]
                    rf_z_const[i * 2 -1] = rf_z_const[i * 2] + 0.08
                    lh_z_const[i * 2 -1] = lh_z_const[i * 2]
                    rh_z_const[i * 2 -1] = rh_z_const[i * 2] + 0.08
                elif self.footholds[i].leg_status == LegStatus.STAND_STILL:
                    lf_z_const[i * 2 -1] = lf_z_const[i * 2]
                    rf_z_const[i * 2 -1] = rf_z_const[i * 2]
                    lh_z_const[i * 2 -1] = lh_z_const[i * 2]
                    rh_z_const[i * 2 -1] = rh_z_const[i * 2]


        ee_velocity_const = np.zeros(lf_x_const.shape)
        ee_velocity_z_const = np.zeros(2*len(self.footholds)-1)
        self.lf_x_trajectory = CubicHermiteSpline(time, lf_x_const,ee_velocity_const)
        self.lf_y_trajectory = CubicHermiteSpline(time, lf_y_const, ee_velocity_const)
        self.lf_z_trajectory = CubicHermiteSpline(time_z, lf_z_const, ee_velocity_z_const)
        self.rf_x_trajectory = CubicHermiteSpline(time, rf_x_const, ee_velocity_const)
        self.rf_y_trajectory = CubicHermiteSpline(time, rf_y_const, ee_velocity_const)
        self.rf_z_trajectory = CubicHermiteSpline(time_z, rf_z_const, ee_velocity_z_const)
        self.lh_x_trajectory = CubicHermiteSpline(time, lh_x_const, ee_velocity_const)
        self.lh_y_trajectory = CubicHermiteSpline(time, lh_y_const, ee_velocity_const)
        self.lh_z_trajectory = CubicHermiteSpline(time_z, lh_z_const, ee_velocity_z_const)
        self.rh_x_trajectory = CubicHermiteSpline(time, rh_x_const, ee_velocity_const)
        self.rh_y_trajectory = CubicHermiteSpline(time, rh_y_const, ee_velocity_const)
        self.rh_z_trajectory = CubicHermiteSpline(time_z, rh_z_const, ee_velocity_z_const)

        self.center_point_x_trajectory = CubicHermiteSpline(time, mass_center_x_const,mass_center_vx_const)
        self.center_point_y_trajectory = CubicHermiteSpline(time, mass_center_y_const, mass_center_vy_const)
        self.center_point_z_trajectory = CubicHermiteSpline(time, mass_center_z_const, mass_center_vz_const)

        self.planning_time = time




        temp = 1

    def generate_footholds_for_major_diagonal_EEs(self,node):
        R_E_B = np.array(
            [[math.cos(self.heading), math.sin(self.heading)], [-math.sin(self.heading), math.cos(self.heading)]])

        lf_pos_h_e = np.around(R_E_B.transpose().dot(self.LF_b) + np.asarray(node[0:2]),   1)

        lf_pos_v_e = self.pointCloud.pc[(lf_pos_h_e[0],lf_pos_h_e[1])].z

        rh_pos_h_e = np.around(R_E_B.transpose().dot(self.RH_b) + np.asarray(node[0:2]), 1)
        rh_pos_v_e = self.pointCloud.pc[(rh_pos_h_e[0],rh_pos_h_e[1])].z

        return (np.append(lf_pos_h_e,lf_pos_v_e), np.append(rh_pos_h_e,rh_pos_v_e))


    def generate_footholds_for_minor_diagonal_EEs(self,node):
        R_E_B = np.array(
            [[math.cos(self.heading), math.sin(self.heading)], [-math.sin(self.heading), math.cos(self.heading)]])

        rf_pos_h_e = np.around(R_E_B.transpose().dot(self.RF_b) + np.asarray(node[0:2]),   1)

        rf_pos_v_e = self.pointCloud.pc[(rf_pos_h_e[0],rf_pos_h_e[1])].z

        lh_pos_h_e = np.around(R_E_B.transpose().dot(self.LH_b) + np.asarray(node[0:2]), 1)
        lh_pos_v_e = self.pointCloud.pc[(lh_pos_h_e[0],lh_pos_h_e[1])].z

        return (np.append(rf_pos_h_e,rf_pos_v_e), np.append(lh_pos_h_e,lh_pos_v_e))




    def plot_result(self):
        fig1, ax = plt.subplots(2,2)
        fig2, ax2 = plt.subplots(1, 2)
        fig3, ax3 = plt.subplots(1, 3)

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

        footholds_lf = np.empty(shape=[0, 3])
        footholds_rf = np.empty(shape=[0, 3])
        footholds_lh = np.empty(shape=[0, 3])
        footholds_rh = np.empty(shape=[0, 3])
        for i in self.footholds:
            footholds_lf = np.vstack((footholds_lf,self.footholds[i].LF_pos))
            footholds_rf = np.vstack((footholds_rf, self.footholds[i].RF_pos))
            footholds_lh = np.vstack((footholds_lh, self.footholds[i].LH_pos))
            footholds_rh = np.vstack((footholds_rh, self.footholds[i].RH_pos))

        # mlab.points3d(footholds_lf[:,0], footholds_lf[:,1], footholds_lf[:,2], figure=fig,scale_factor=.05,color=(1,0,0))
        # mlab.points3d(footholds_rf[:,0], footholds_rf[:,1], footholds_rf[:,2], figure=fig,scale_factor=.05,color=(1,1,0))
        # mlab.points3d(footholds_lh[:, 0], footholds_lf[:, 1], footholds_lf[:, 2], figure=fig, scale_factor=.05,
        #               color=(0, 0, 1))
        # mlab.points3d(footholds_rh[:, 0], footholds_rf[:, 1], footholds_rf[:, 2], figure=fig, scale_factor=.05,
        #               color=(0, 1, 0))

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

        sample_number = 200
        time = np.linspace(0,self.planning_time[-1],sample_number)
        lf_x = np.zeros(sample_number)
        lf_y = np.zeros(sample_number)
        lf_z = np.zeros(sample_number)
        rf_x = np.zeros(sample_number)
        rf_y = np.zeros(sample_number)
        rf_z = np.zeros(sample_number)
        lh_x = np.zeros(sample_number)
        lh_y = np.zeros(sample_number)
        lh_z = np.zeros(sample_number)
        rh_x = np.zeros(sample_number)
        rh_y = np.zeros(sample_number)
        rh_z = np.zeros(sample_number)

        center_point_x = np.zeros(sample_number)
        center_point_y = np.zeros(sample_number)
        center_point_z = np.zeros(sample_number)


        for i,t in enumerate(time):
            lf_x[i] = self.lf_x_trajectory.__call__(t)
            lf_y[i] = self.lf_y_trajectory.__call__(t)
            lf_z[i] = self.lf_z_trajectory.__call__(t)
            rf_x[i] = self.rf_x_trajectory.__call__(t)
            rf_y[i] = self.rf_y_trajectory.__call__(t)
            rf_z[i] = self.rf_z_trajectory.__call__(t)
            lh_x[i] = self.lh_x_trajectory.__call__(t)
            lh_y[i] = self.lh_y_trajectory.__call__(t)
            lh_z[i] = self.lh_z_trajectory.__call__(t)
            rh_x[i] = self.rh_x_trajectory.__call__(t)
            rh_y[i] = self.rh_y_trajectory.__call__(t)
            rh_z[i] = self.rh_z_trajectory.__call__(t)

            center_point_x[i] = self.center_point_x_trajectory.__call__(t)
            center_point_y[i] = self.center_point_y_trajectory.__call__(t)
            center_point_z[i] = self.center_point_z_trajectory.__call__(t)

        ax3[0].plot(time,lf_x,'r',time,rf_x,'y',time,lh_x,'b',time,rh_x,'g',time,center_point_x,'k')
        ax3[1].plot(time, lf_y, 'r', time, rf_y, 'y', time, lh_y, 'b', time, rh_y, 'g',time,center_point_y,'k')
        ax3[2].plot(time, lf_z,'r')

        mlab.plot3d(lf_x, lf_y, lf_z, figure=fig, line_width = 0.1, color=(1,0,0))
        mlab.plot3d(center_point_x, center_point_y, center_point_z, figure=fig, line_width=0.1, color=(0, 0, 0))
        plt.show()
        mlab.show()


if __name__ == "__main__":
    zmp_0 = (2.0,1.0,0.)
    zmp_f = (3.0,1,0)
    anyAStar = AnymalAStar(zmp_0,zmp_f)
    anyAStar.run()
    optimalPath = anyAStar.getOptimalPath()
    anyAStar.generate_EE_trajectory()
    anyAStar.plot_result()

