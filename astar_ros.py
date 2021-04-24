from astar_whole_body import AnymalAStarWholeBody
import numpy as np
from scipy.interpolate import CubicHermiteSpline
from datetime import datetime
from astar_whole_body import LegStatus
from astar_whole_body import FootholdStatus

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from anymal_ctrl_edin_gaits.msg import whole_body_control_msg

from std_msgs.msg import String
from geometry_msgs.msg import TwistWithCovarianceStamped
from signal_logger_msgs.msg import BoolStamped
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Bool
from geometry_msgs.msg import QuaternionStamped
from anymal_msgs.msg import AnymalState
import math




class AnymalAStarRos(AnymalAStarWholeBody):
    def __init__(self,zmp_0, zmp_f):
        super().__init__(zmp_0, zmp_f)
        # LIP's current states
        self.zmp = np.zeros((1,3))
        self.mass_point_pos = np.zeros((1,3))
        self.mass_point_vel = np.zeros((1, 3))
        # Anymal's feet states
        self.on_ground = [True,True,True,True] # LF, RF,LH, RH

        self.trajectory_ready = False

        self.position_EE = np.zeros((4,3))

        self.lf_is_contact = True
        self.rf_is_contact = True
        self.lh_is_contact = True
        self.rh_is_contact = True


        rospy.Subscriber("/log/loco/whole_body/positionWorldToComInWorldFrame", Vector3Stamped, self.set_center_of_mass_callback)
        rospy.Subscriber("/log/loco/whole_body/linearVelocityComInWorldFrame", Vector3Stamped, self.set_velocity_of_mass_callback)
        rospy.Subscriber("/log/loco/torso/measured/orientationWorldToControl", QuaternionStamped, self.set_orientation_callback)
        rospy.Subscriber("/log/loco/leftFore/isGrounded", BoolStamped, self.set_on_ground_LF_callback)
        rospy.Subscriber("/log/loco/rightFore/isGrounded", BoolStamped, self.set_on_ground_RF_callback)
        rospy.Subscriber("/log/loco/leftHind/isGrounded", BoolStamped, self.set_on_ground_LH_callback)
        rospy.Subscriber("/log/loco/rightHind/isGrounded", BoolStamped, self.set_on_ground_RH_callback)
        rospy.Subscriber("/log/loco/leftFore/positionWorldToEEOriginInWorldFrame", Vector3Stamped, self.set_EE_LF_callback)
        rospy.Subscriber("/log/loco/rightFore/positionWorldToEEOriginInWorldFrame", Vector3Stamped, self.set_EE_RF_callback)
        rospy.Subscriber("/log/loco/leftHind/positionWorldToEEOriginInWorldFrame", Vector3Stamped, self.set_EE_LH_callback)
        rospy.Subscriber("/log/loco/rightHind/positionWorldToEEOriginInWorldFrame", Vector3Stamped, self.set_EE_RH_callback)
        rospy.Subscriber("calc_zmp_cmd", Bool, self.calc_zmp_callback)

        rospy.init_node('AnymalAStar')
        self.pub_LIP = rospy.Publisher("LIP", Marker, queue_size=10)
        self.pub_path = rospy.Publisher("optimal_path", Marker, queue_size=30)
        self.pub_whole_body_ref = rospy.Publisher("/whole_body_control_reference", whole_body_control_msg, queue_size=30)

    def calc_zmp_callback(self,data):
        if data:
            self.calc_zmp()
            m = Marker()
            m.header.frame_id = 'odom'
            m.header.stamp = rospy.Time.now()
            m.ns = 'anyplan_listener'
            m.pose.orientation.w = 1.0
            m.action = Marker.ADD
            m.id = 0
            m.type = Marker.LINE_LIST
            m.color.r = 0.4
            m.color.g = 0.4
            m.color.b = 0.4
            m.color.a = 1.0
            m.scale.x = 0.1
            p0 = Point(self.zmp[0],self.zmp[1],self.zmp[2])
            p1 = Point(self.mass_point_pos[0],self.mass_point_pos[1],self.mass_point_pos[2])
            m.points.append(p0)
            m.points.append(p1)
            self.pub_LIP.publish(m)

    def set_on_ground_LF_callback(self,data):
        self.on_ground[0] = data.value

    def set_on_ground_RF_callback(self,data):
        self.on_ground[1] = data.value

    def set_on_ground_LH_callback(self,data):
        self.on_ground[2] = data.value

    def set_on_ground_RH_callback(self,data):
        self.on_ground[3] = data.value

    def set_EE_LF_callback(self,data):
        self.position_EE[0] = [data.vector.x,data.vector.y,data.vector.z]

    def set_EE_RF_callback(self,data):
        self.position_EE[1] = [data.vector.x,data.vector.y,data.vector.z]

    def set_EE_LH_callback(self,data):
        self.position_EE[2] = [data.vector.x,data.vector.y,data.vector.z]

    def set_EE_RH_callback(self,data):
        self.position_EE[3] = [data.vector.x,data.vector.y,data.vector.z]

    def set_center_of_mass_callback(self,data):
        self.mass_point_pos = [data.vector.x,data.vector.y,data.vector.z]

    def set_velocity_of_mass_callback(self,data):
        self.mass_point_vel = [data.vector.x,data.vector.y,data.vector.z]

    def set_orientation_callback(self,data):
        self.torso_orientation = [data.quaternion.x,data.quaternion.y,data.quaternion.z,data.quaternion.w]

    def calc_zmp(self):
        contact_feet_position = []
        for i,foot in enumerate(self.on_ground):
            if foot:
                contact_feet_position.append(self.position_EE[i])
        self.zmp = np.mean(contact_feet_position,axis=0)
        return self.zmp

    def run(self):
        self.set_start(tuple(self.calc_zmp()) + (self.mass_point_pos[0], self.mass_point_vel[0], self.mass_point_pos[1], self.mass_point_vel[1]))
        print("[astar_ros] astar.run() is called")
        print("The start node is {}".format(self.start))
        print("The goal node is {}".format(self.goal))
        self.closedList = {}
        self.openList = {}
        super().run()


    def test_calc_lf_ee_traj_callback(self):
        self.traj_time = 10.0
        time = [0,self.traj_time/4,self.traj_time/4*3,self.traj_time]

        lf_x_const = [self.position_EE[0,0],self.position_EE[0,0],self.position_EE[0,0],self.position_EE[0,0]]
        lf_y_const = [self.position_EE[0, 1], self.position_EE[0, 1], self.position_EE[0, 1],self.position_EE[0, 1]]
        lf_z_const = [self.position_EE[0, 2],self.position_EE[0, 2],self.position_EE[0, 2]+0.1,self.position_EE[0, 2]]
        lf_vel_const = [.0,.0,.0,.0]

        rf_x_const = [self.position_EE[1,0],self.position_EE[1,0],self.position_EE[1,0],self.position_EE[1,0]]
        rf_y_const = [self.position_EE[1, 1], self.position_EE[1, 1], self.position_EE[1, 1],self.position_EE[1, 1]]
        rf_z_const = [self.position_EE[1, 2],self.position_EE[1, 2],self.position_EE[1, 2],self.position_EE[1, 2]]
        rf_vel_const = [.0,.0,.0,.0]

        lh_x_const = [self.position_EE[2,0],self.position_EE[2,0],self.position_EE[2,0],self.position_EE[2,0]]
        lh_y_const = [self.position_EE[2, 1], self.position_EE[2, 1], self.position_EE[2, 1],self.position_EE[2, 1]]
        lh_z_const = [self.position_EE[2, 2],self.position_EE[2, 2],self.position_EE[2, 2],self.position_EE[2, 2]]
        lh_vel_const = [.0,.0,.0,.0]

        rh_x_const = [self.position_EE[3,0],self.position_EE[3,0],self.position_EE[3,0],self.position_EE[3,0]]
        rh_y_const = [self.position_EE[3, 1], self.position_EE[3, 1], self.position_EE[3, 1],self.position_EE[3, 1]]
        rh_z_const = [self.position_EE[3, 2],self.position_EE[3, 2],self.position_EE[3, 2],self.position_EE[3, 2]]
        rh_vel_const = [.0,.0,.0,.0]

        mass_center_x_const = [self.mass_point_pos[0],self.mass_point_pos[0]-0.05,self.mass_point_pos[0]-0.05,self.mass_point_pos[0]-0.05]
        mass_center_y_const = [self.mass_point_pos[1], self.mass_point_pos[1]-0.05, self.mass_point_pos[1]-0.05,self.mass_point_pos[1]-0.05]
        mass_center_z_const = [self.mass_point_pos[2], self.mass_point_pos[2], self.mass_point_pos[2],self.mass_point_pos[2]]
        mass_center_vel_const = [.0, .0, .0,.0]

        orientation_x_const = [self.torso_orientation[0],self.torso_orientation[0],self.torso_orientation[0],self.torso_orientation[0]]
        orientation_y_const = [self.torso_orientation[1], self.torso_orientation[1], self.torso_orientation[1], self.torso_orientation[1]]
        orientation_z_const = [self.torso_orientation[2], self.torso_orientation[2], self.torso_orientation[2], self.torso_orientation[2]]
        orientation_w_const = [self.torso_orientation[3], self.torso_orientation[3], self.torso_orientation[3], self.torso_orientation[3]]
        orientation_vel_const = [.0, .0, .0,.0]

        self.lf_x_trajectory = CubicHermiteSpline(time, lf_x_const, lf_vel_const)
        self.lf_y_trajectory = CubicHermiteSpline(time, lf_y_const, lf_vel_const)
        self.lf_z_trajectory = CubicHermiteSpline(time, lf_z_const, lf_vel_const)
        self.lf_vx_trajectory = self.lf_x_trajectory.derivative()
        self.lf_vy_trajectory = self.lf_y_trajectory.derivative()
        self.lf_vz_trajectory = self.lf_z_trajectory.derivative()


        self.rf_x_trajectory = CubicHermiteSpline(time, rf_x_const, rf_vel_const)
        self.rf_y_trajectory = CubicHermiteSpline(time, rf_y_const, rf_vel_const)
        self.rf_z_trajectory = CubicHermiteSpline(time, rf_z_const, rf_vel_const)
        self.rf_vx_trajectory = self.rf_x_trajectory.derivative()
        self.rf_vy_trajectory = self.rf_y_trajectory.derivative()
        self.rf_vz_trajectory = self.rf_z_trajectory.derivative()

        self.lh_x_trajectory = CubicHermiteSpline(time, lh_x_const, lh_vel_const)
        self.lh_y_trajectory = CubicHermiteSpline(time, lh_y_const, lh_vel_const)
        self.lh_z_trajectory = CubicHermiteSpline(time, lh_z_const, lh_vel_const)
        self.lh_vx_trajectory = self.lh_x_trajectory.derivative()
        self.lh_vy_trajectory = self.lh_y_trajectory.derivative()
        self.lh_vz_trajectory = self.lh_z_trajectory.derivative()

        self.rh_x_trajectory = CubicHermiteSpline(time, rh_x_const, rh_vel_const)
        self.rh_y_trajectory = CubicHermiteSpline(time, rh_y_const, rh_vel_const)
        self.rh_z_trajectory = CubicHermiteSpline(time, rh_z_const, rh_vel_const)
        self.rh_vx_trajectory = self.rh_x_trajectory.derivative()
        self.rh_vy_trajectory = self.rh_y_trajectory.derivative()
        self.rh_vz_trajectory = self.rh_z_trajectory.derivative()

        self.mass_center_x_trajectory = CubicHermiteSpline(time, mass_center_x_const, mass_center_vel_const)
        self.mass_center_y_trajectory = CubicHermiteSpline(time, mass_center_y_const, mass_center_vel_const)
        self.mass_center_z_trajectory = CubicHermiteSpline(time, mass_center_z_const, mass_center_vel_const)

        self.orientation_x_trajectory = CubicHermiteSpline(time, orientation_x_const, orientation_vel_const)
        self.orientation_y_trajectory = CubicHermiteSpline(time, orientation_y_const, orientation_vel_const)
        self.orientation_z_trajectory = CubicHermiteSpline(time, orientation_z_const, orientation_vel_const)
        self.orientation_w_trajectory = CubicHermiteSpline(time, orientation_w_const, orientation_vel_const)


        self.trajectory_ready = True
        self.generated_trajectory_timestamp = datetime.now()

    def publish_whole_body_trajectory_reference(self):

        t = (datetime.now()-self.generated_trajectory_timestamp).total_seconds()
        print("current time is {}".format(t))
        print("planned time is {}".format(self.traj_time))
        if t > self.traj_time:
            print("publishing function returns")
            return False

        phase_counter = 0
        while True:
            phase_counter += 1
            if t > (phase_counter-1) * self.phaseTime and t < phase_counter * self.phaseTime:
                t_in_phase = t - (phase_counter-1) * self.phaseTime
                break
        print("The Anymal is now at phase {}".format(phase_counter))

        if self.footholds[phase_counter].leg_status == LegStatus.MAJOR_DIAGONAL:
            self.lf_is_contact = False
            self.rh_is_contact = False
            self.rf_is_contact = True
            self.lh_is_contact = True
        elif self.footholds[phase_counter].leg_status == LegStatus.MINOR_DIAGONAL:
            self.lf_is_contact = True
            self.rh_is_contact = True
            self.rf_is_contact = False
            self.lh_is_contact = False

        print("Contact:lf={},rf={},lh={},rh={}".format(self.lf_is_contact,self.rf_is_contact,self.lh_is_contact,self.rh_is_contact))

        traj_ref = whole_body_control_msg()

        traj_ref.header.stamp = rospy.Time.now()

        # calculate CoM trajectory
        p_x = self.footholds[phase_counter].lip_states[0]
        p_y = self.footholds[phase_counter].lip_states[1]
        omega = math.sqrt(self.g / self.z)
        A = np.array([[np.cosh(omega * t_in_phase), 1 / omega * np.sinh(omega * t_in_phase)],
                      [omega * np.sinh(omega * t_in_phase), np.cosh(omega * t_in_phase)]])
        B = np.array([1 - np.cosh(omega * t_in_phase), -omega * np.sinh(omega * t_in_phase)])

        x0 = np.array([self.footholds[phase_counter-1].lip_states[3], self.footholds[phase_counter-1].lip_states[4]])
        y0 = np.array([self.footholds[phase_counter-1].lip_states[5], self.footholds[phase_counter-1].lip_states[6]])
        states_x = A.dot(x0) + B * p_x
        states_y = A.dot(y0) + B * p_y

        traj_ref.com_position.x = states_x[0]
        traj_ref.com_position.y = states_y[0]
        traj_ref.com_position.z = self.mass_center_z_trajectory.__call__(t)

        # CoM trajectory is done



        traj_ref.base_angular_velocity.x = states_x[1]
        traj_ref.base_angular_velocity.y = states_y[1]
        traj_ref.base_angular_velocity.z = .0

        # traj_ref.base_orientation.x = self.orientation_x_trajectory.__call__(t)
        # traj_ref.base_orientation.y = self.orientation_y_trajectory.__call__(t)
        # traj_ref.base_orientation.z = self.orientation_z_trajectory.__call__(t)
        # traj_ref.base_orientation.w = self.orientation_w_trajectory.__call__(t)


        traj_ref.base_orientation.x = .0
        traj_ref.base_orientation.y = .0
        traj_ref.base_orientation.z = .0
        traj_ref.base_orientation.w = .1

        traj_ref.foot1_position.x = self.lf_x_trajectory.__call__(t)
        traj_ref.foot1_position.y = self.lf_y_trajectory.__call__(t)
        traj_ref.foot1_position.z = self.lf_z_trajectory.__call__(t)

        traj_ref.foot1_velocity.x = self.lf_vx_trajectory.__call__(t)
        traj_ref.foot1_velocity.y = self.lf_vy_trajectory.__call__(t)
        traj_ref.foot1_velocity.z = self.lf_vz_trajectory.__call__(t)


        traj_ref.foot1_isContact = self.lf_is_contact

        traj_ref.foot2_position.x = self.rf_x_trajectory.__call__(t)
        traj_ref.foot2_position.y = self.rf_y_trajectory.__call__(t)
        traj_ref.foot2_position.z = self.rf_z_trajectory.__call__(t)


        traj_ref.foot2_velocity.x = self.rf_vx_trajectory.__call__(t)
        traj_ref.foot2_velocity.y = self.rf_vy_trajectory.__call__(t)
        traj_ref.foot2_velocity.z = self.rf_vz_trajectory.__call__(t)

        traj_ref.foot2_isContact = self.rf_is_contact

        traj_ref.foot3_position.x = self.lh_x_trajectory.__call__(t)
        traj_ref.foot3_position.y = self.lh_y_trajectory.__call__(t)
        traj_ref.foot3_position.z = self.lh_z_trajectory.__call__(t)

        traj_ref.foot3_velocity.x = self.lh_vx_trajectory.__call__(t)
        traj_ref.foot3_velocity.y = self.lh_vy_trajectory.__call__(t)
        traj_ref.foot3_velocity.z = self.lh_vz_trajectory.__call__(t)

        traj_ref.foot3_isContact = self.lh_is_contact

        traj_ref.foot4_position.x = self.rh_x_trajectory.__call__(t)
        traj_ref.foot4_position.y = self.rh_y_trajectory.__call__(t)
        traj_ref.foot4_position.z = self.rh_z_trajectory.__call__(t)

        traj_ref.foot4_velocity.x = self.rh_vx_trajectory.__call__(t)
        traj_ref.foot4_velocity.y = self.rh_vy_trajectory.__call__(t)
        traj_ref.foot4_velocity.z = self.rh_vz_trajectory.__call__(t)

        traj_ref.foot4_isContact = self.rh_is_contact

        print("Planner is publishing! mass center is({},{},{})".format(traj_ref.com_position.x,traj_ref.com_position.y,traj_ref.com_position.z))

        self.pub_whole_body_ref.publish(traj_ref)



if __name__ == "__main__":
    zmp_0 = (0.0,0.0,0.)
    zmp_f = (8.0,0,0)
    astar = AnymalAStarRos(zmp_0,zmp_f)
    wait_time = 1
    r_wait_rate = rospy.Rate(1/wait_time)
    r_wait_rate.sleep()
    astar.run()

    r = rospy.Rate(400)

    while not rospy.is_shutdown():
        astar.publish_whole_body_trajectory_reference()
        r.sleep()












