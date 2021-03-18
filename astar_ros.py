from astar_LIP import AnymalAStar
import numpy as np

class AnymalAStarRos(AnymalAStar):
    def __init__(self,zmp_0, zmp_f):
        AnymalAStar.__init__(self,zmp_0, zmp_f)
        # LIP's current states
        self.zmp = np.zeros((1,3))
        self.mass_point = np.zeros((1,3))

        # Anymal's feet states
        self.on_ground = [True,True,True,True] # LF, RF,LH, RH

        self.position_EE = np.zeros((4,3))

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