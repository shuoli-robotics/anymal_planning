from astar_LIP import AnymalAStar
import numpy as np
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

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