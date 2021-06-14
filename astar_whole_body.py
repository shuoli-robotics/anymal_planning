from astar_LIP import AnymalAStar
import numpy as np
from mayavi import mlab
import matplotlib.pyplot as plt
from enum import Enum
from scipy.interpolate import CubicHermiteSpline
from datetime import datetime
import math

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

class AnymalAStarWholeBody(AnymalAStar):
    def __init__(self,zmp_0, zmp_f):
        AnymalAStar.__init__(self,zmp_0, zmp_f)

        # These variables are used to store Anymal's current position and velocity
        # when ros topics are used, they will be rewritten by Anymal's current measured states
        self.zmp = np.array([zmp_0[0],zmp_0[1],zmp_0[2]])
        self.mass_point_pos = np.array([zmp_0[0],zmp_0[1],self.z])
        self.mass_point_vel = np.zeros((1, 3))
        self.on_ground = [True,True,True,True] # LF, RF,LH, RH
        self.trajectory_ready = False
        self.position_EE = np.zeros((4,3))
        self.position_EE[0], self.position_EE[3] = self.generate_footholds_for_major_diagonal_EEs(self.start)
        self.position_EE[1], self.position_EE[2] = self.generate_footholds_for_minor_diagonal_EEs(self.start)

        self.next_support_status = LegStatus.STAND_STILL

        # This variable is used to indicate if ROS is used. If False, the first planned footholds will be generated
        # by default standing setup. Otherwise, the first footholds is the Anymal's current standing footholds
        self.use_ros = True

        self.trajectory_ready = False

    def calc_zmp(self):
        contact_feet_position = []
        for i,foot in enumerate(self.on_ground):
            if foot:
                contact_feet_position.append(self.position_EE[i])
        self.zmp = np.mean(contact_feet_position,axis=0)
        return self.zmp

    def calc_surporting_legs(self):
        if self.on_ground[0] and self.on_ground[1] and self.on_ground[1] and self.on_ground[1]:
            self.current_support_status = LegStatus.STAND_STILL
            self.next_support_status = LegStatus.MAJOR_DIAGONAL
        elif (self.on_ground[0] and self.on_ground[3]) and not (self.on_ground[1] or self.on_ground[2]):
            self.current_support_status = LegStatus.MAJOR_DIAGONAL
            self.next_support_status = LegStatus.MINOR_DIAGONAL
        elif not (self.on_ground[0] or self.on_ground[3]) and (self.on_ground[1] or self.on_ground[2]):
            self.current_support_status = LegStatus.MINOR_DIAGONAL
            self.next_support_status = LegStatus.MAJOR_DIAGONAL
        else:
            print("[Error] [astar_whole_body::calc_suporting_legs] Next supporting legs are wrong! ")


    def generate_EE_trajectory(self):
        # calculate current ZMP and what the next supporting leg group should be
        self.calc_zmp()
        self.calc_surporting_legs()

        optimalPath = self.getOptimalPath()
        n = len(optimalPath)
        self.traj_time = n * self.phaseTime

        print("[astar] Start generating Footholds...")
        # generate footholds on terrain considering gaits

        

        for i, node in reversed(list(enumerate(optimalPath))):
            index = n-i-1
            self.footholds[index] = FootholdStatus()
            self.footholds[index].lip_states = node
            if index == 0:
                if self.use_ros == False:
                    self.footholds[index].LF_pos, self.footholds[index].RH_pos = self.generate_footholds_for_major_diagonal_EEs(node)
                    self.footholds[index].RF_pos, self.footholds[index].LH_pos = self.generate_footholds_for_minor_diagonal_EEs(node)
                else:
                    print("[astar] Use measured footholds for generating trajectories")
                    self.footholds[index].LF_pos = self.position_EE[0]
                    self.footholds[index].RF_pos = self.position_EE[1]
                    self.footholds[index].LH_pos = self.position_EE[2]
                    self.footholds[index].RH_pos = self.position_EE[3]
                    if self.on_ground[0] == True and self.on_ground[1] == False:
                        self.footholds[index].leg_status = LegStatus.MAJOR_DIAGONAL
                    elif self.on_ground[1] == True and self.on_ground[0] == False:
                        self.footholds[index].leg_status = LegStatus.MINOR_DIAGONAL
                    elif self.on_ground[1] == True and self.on_ground[0] == True:
                        self.footholds[index].leg_status = LegStatus.STAND_STILL
                    else:
                        print("[Error] [astar_whole_body::generate_EE_trajectory] Current standing status is wrong! ")

            else:
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

        print("[astar] Footholds are generated. Start generating trajectory...")
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
                    lh_z_const[i * 2 -1] = lh_z_const[i * 2]
                    rh_z_const[i * 2 -1] = rh_z_const[i * 2]+ 0.08
                elif self.footholds[i].leg_status == LegStatus.MINOR_DIAGONAL:
                    lf_z_const[i * 2 -1] = lf_z_const[i * 2]
                    rf_z_const[i * 2 -1] = rf_z_const[i * 2] + 0.08
                    lh_z_const[i * 2 -1] = lh_z_const[i * 2] + 0.08
                    rh_z_const[i * 2 -1] = rh_z_const[i * 2]
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

        self.lf_vx_trajectory = self.lf_x_trajectory.derivative()
        self.lf_vy_trajectory = self.lf_y_trajectory.derivative()
        self.lf_vz_trajectory = self.lf_z_trajectory.derivative()
        self.rf_vx_trajectory = self.rf_x_trajectory.derivative()
        self.rf_vy_trajectory = self.rf_y_trajectory.derivative()
        self.rf_vz_trajectory = self.rf_z_trajectory.derivative()
        self.lh_vx_trajectory = self.lh_x_trajectory.derivative()
        self.lh_vy_trajectory = self.lh_y_trajectory.derivative()
        self.lh_vz_trajectory = self.lh_z_trajectory.derivative()
        self.rh_vx_trajectory = self.rh_x_trajectory.derivative()
        self.rh_vy_trajectory = self.rh_y_trajectory.derivative()
        self.rh_vz_trajectory = self.rh_z_trajectory.derivative()

        self.mass_center_x_trajectory = CubicHermiteSpline(time, mass_center_x_const,mass_center_vx_const)
        self.mass_center_y_trajectory = CubicHermiteSpline(time, mass_center_y_const, mass_center_vy_const)
        self.mass_center_z_trajectory = CubicHermiteSpline(time, mass_center_z_const, mass_center_vz_const)

        self.planning_time = time
        self.generated_trajectory_timestamp = datetime.now()
        print("[astar] Trajectories are generated")

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

    def run(self):
        self.closedList = {}
        self.openList = {}
        super().run()
        # self.generate_EE_trajectory()

    def set_start(self,start_node):
        self.start = (round(start_node[0],1),round(start_node[1],1),round(start_node[2],1),\
                      round(start_node[3],1),round(start_node[4],1),round(start_node[5],1),round(start_node[6],1))
        test = 1

    def set_target(self,target):
        self.goal = (round(target[0],1),round(target[1],1),round(target[2],1))


    def plot_result(self):
        fig = super().plot_result()

        fig3, ax3 = plt.subplots(1, 3)
        footholds_lf = np.empty(shape=[0, 3])
        footholds_rf = np.empty(shape=[0, 3])
        footholds_lh = np.empty(shape=[0, 3])
        footholds_rh = np.empty(shape=[0, 3])
        for i in self.footholds:
            footholds_lf = np.vstack((footholds_lf, self.footholds[i].LF_pos))
            footholds_rf = np.vstack((footholds_rf, self.footholds[i].RF_pos))
            footholds_lh = np.vstack((footholds_lh, self.footholds[i].LH_pos))
            footholds_rh = np.vstack((footholds_rh, self.footholds[i].RH_pos))

        sample_number = 200
        time = np.linspace(0, self.planning_time[-1], sample_number)
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

        for i, t in enumerate(time):
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

            center_point_x[i] = self.mass_center_x_trajectory.__call__(t)
            center_point_y[i] = self.mass_center_y_trajectory.__call__(t)
            center_point_z[i] = self.mass_center_z_trajectory.__call__(t)

        ax3[0].plot(time, lf_x, 'r', time, rf_x, 'y', time, lh_x, 'b', time, rh_x, 'g', time, center_point_x, 'k')
        ax3[1].plot(time, lf_y, 'r', time, rf_y, 'y', time, lh_y, 'b', time, rh_y, 'g', time, center_point_y, 'k')
        ax3[2].plot(time, lf_z, 'r')

        mlab.plot3d(lf_x, lf_y, lf_z, figure=fig, line_width=0.1, color=(1, 0, 0))
        mlab.plot3d(center_point_x, center_point_y, center_point_z, figure=fig, line_width=0.1, color=(0, 0, 0))

if __name__ == "__main__":
    zmp_0 = (0.0,0.0,0.)
    zmp_f = (8.0,0,0)
    anyAStar = AnymalAStarWholeBody(zmp_0,zmp_f)
    anyAStar.run()
    anyAStar.generate_EE_trajectory()

    anyAStar.set_start((2.0,2.32,0.0, 2.05,0.41,2.31,0.10))

    time_start = datetime.now()
    anyAStar.run()
    anyAStar.generate_EE_trajectory()
    print("The time of A* is {}".format((datetime.now()-time_start).total_seconds()))

    optimalPath = anyAStar.getOptimalPath()
    anyAStar.plot_result()
    plt.show()
    mlab.show()