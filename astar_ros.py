from astar_LIP import AnymalAStar
import numpy as np
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from whole_body_control_msg.msg import whole_body_control_msg

class AnymalAStarRos(AnymalAStar):
    def __init__(self,zmp_0, zmp_f):
        AnymalAStar.__init__(self,zmp_0, zmp_f)
        # LIP's current states
        self.zmp = np.zeros((1,3))
        self.mass_point_pos = np.zeros((1,3))
        self.mass_point_vel = np.zeros((1, 3))
        # Anymal's feet states
        self.on_ground = [True,True,True,True] # LF, RF,LH, RH

        self.position_EE = np.zeros((4,3))

        # rospy.init_node('AnymalAStar')
        self.pub_LIP = rospy.Publisher("LIP", Marker, queue_size=10)
        self.pub_path = rospy.Publisher("optimal_path", Marker, queue_size=30)

    def calc_zmp(self):
        contact_feet_position = []
        for i,foot in enumerate(self.on_ground):
            if foot:
                contact_feet_position.append(self.position_EE[i])
        self.zmp = np.mean(contact_feet_position,axis=0)
        print(self.zmp)


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

    def set_goal_callback(self,data):
        self.goal = [data.vector.x,data.vector.y,data.vector.z]

    def set_orientation_callback(self,data):
        self.torso_orientation = [data.x,data.y,data.z,data.w]

    def run_callback(self,data):

        self.calc_zmp()
        self.zmp[0] = round(self.zmp[0],1)
        self.zmp[1] = round(self.zmp[1], 1)
        self.zmp[2] = round(self.zmp[2], 1)
        self.start = tuple(self.zmp) + (round(self.mass_point_pos[0],1),round(self.mass_point_vel[0],1),round(self.mass_point_pos[1],1),round(self.mass_point_vel[1],1))
        self.goal = (data.vector.x, data.vector.y, data.vector.z)
        print("[astar_ros] astar.run() is called")
        print("The start node is {}".format(self.start))
        print("The goal node is {}".format(self.goal))
        self.closedList = {}
        self.openList = {}
        self.run()

    def display_path_callback(self,data):
        print("[astar_ros] display_path_callback")
        op = self.getOptimalPath()
        zmp_x = np.zeros(len(op))
        zmp_y = np.zeros(len(op))
        mass_point_x = np.zeros(len(op))
        mass_point_y = np.zeros(len(op))
        mass_point_vx = np.zeros(len(op))
        mass_point_vy = np.zeros(len(op))

        for i, node in enumerate(op):
            x = node[3]
            y = node[5]
            z = self.z

            m = Marker()
            m.header.frame_id = 'odom'
            m.header.stamp = rospy.Time.now()
            m.ns = 'anyplan_listener'
            # m.pose.orientation.w = 1.0
            m.action = Marker.ADD
            m.id = i
            m.type = Marker.LINE_LIST
            m.color.r = 0.4
            m.color.g = 0
            m.color.b = 0
            m.color.a = 1.0
            m.scale.x = 0.01
            m.lifetime = rospy.Duration.from_sec(100)
            p0 = Point(node[0], node[1], node[2])
            p1 = Point(node[3], node[5], self.z)
            m.points.append(p0)
            m.points.append(p1)
            # print("[astar_ros] display_path_callback p0 = {},p1 = {}".format(p0,p1))
            self.pub_path.publish(m)

    def move_lf_ee_callback(self):
        traj_ref = whole_body_control_msg()
        traj_ref.com_position.x = self.mass_point_pos[0]
        traj_ref.com_position.y = self.mass_point_pos[1]
        traj_ref.com_position.z = self.mass_point_pos[2]

        traj_ref.base_angular_velocity.x = .0
        traj_ref.base_angular_velocity.y = .0
        traj_ref.base_angular_velocity.z = .0

        traj_ref.base_orientation.x = 0.0
        traj_ref.base_orientation.y = 0.0
        traj_ref.base_orientation.z = 0.0
        traj_ref.base_orientation.w = 1.0

        traj_ref.foot1_position.x = self.position_EE[0, 0]
        traj_ref.foot1_position.y = self.position_EE[0, 1]
        traj_ref.foot1_position.z = self.position_EE[0, 2]

        traj_ref.foot1_velocity.x = 0
        traj_ref.foot1_velocity.y = 0
        traj_ref.foot1_velocity.z = 0

        traj_ref.foot1_isContact = False

        traj_ref.foot2_position.x = self.position_EE[1, 0]
        traj_ref.foot2_position.y = self.position_EE[1, 1]
        traj_ref.foot2_position.z = self.position_EE[1, 2]

        traj_ref.foot2_velocity.x = 0
        traj_ref.foot2_velocity.y = 0
        traj_ref.foot2_velocity.z = 0

        traj_ref.foot2_isContact = True

        traj_ref.foot3_position.x = self.position_EE[2, 0]
        traj_ref.foot3_position.y = self.position_EE[2, 1]
        traj_ref.foot3_position.z = self.position_EE[2, 2]

        traj_ref.foot3_velocity.x = 0
        traj_ref.foot3_velocity.y = 0
        traj_ref.foot3_velocity.z = 0

        traj_ref.foot3_isContact = True

        traj_ref.foot4_position.x = self.position_EE[2, 0]
        traj_ref.foot4_position.y = self.position_EE[2, 1]
        traj_ref.foot4_position.z = self.position_EE[2, 2]

        traj_ref.foot4_velocity.x = 0
        traj_ref.foot4_velocity.y = 0
        traj_ref.foot4_velocity.z = 0

        traj_ref.foot4_isContact = True









