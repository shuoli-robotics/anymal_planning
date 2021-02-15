import matplotlib.pyplot as plt
import numpy as np
import math
from mayavi import mlab

class Point:
    def __init__(self, z, sigma):
        self.z = z
        self.sigma = sigma

class PointCloudBase:
    def __init__(self):
        self.pc = {}
        self.mapDigiti = 2
        self.length = 10
        self.center = (self.length/2,self.length/2)
        self.X = np.arange(0, self.length, 0.1)
        self.Y = np.arange(0, self.length, 0.1)
        self.X = np.append(self.X, 10)
        self.Y = np.append(self.Y, 10)
        self.Z = np.zeros((len(self.X),len(self.Y)))
        self.Sigma = np.zeros(self.Z.shape)
        for i,x in enumerate(self.X):
            for j,y in enumerate(self.Y):
                self.Z[i,j] = 0
                self.Sigma[i,j] = 0.1
                x = round(x, self.mapDigiti)
                y = round(y, self.mapDigiti)
                self.pc[(x,y)] = Point(self.Z[i,j],self.Sigma[i,j])

        self.revise_terrain()

    def revise_terrain(self):
        pass



    def show_point_cloud(self,fig):
        X,Y = np.meshgrid(self.X,self.Y)
        mesh = mlab.mesh(X, Y, self.Z,figure = fig,color=(0.3, 0.3, 0.3))
        mlab.show()


class CentralLake(PointCloudBase):
    def revise_terrain(self):
        for i,x in enumerate(self.X):
            for j,y in enumerate(self.Y):
                x = round(x, self.mapDigiti)
                y = round(y, self.mapDigiti)
                if math.sqrt((x-self.center[0])**2 + (y-self.center[1])**2) < 2:
                    self.Z[i,j] = -0.1
                    self.Sigma[i,j] = 1.0
                    self.pc[(x, y)].z = self.Z[i, j]
                    self.pc[(x, y)].sigma = self.Sigma[i, j]





if __name__ == "__main__":
    pc = CentralLake()
    fig = mlab.figure(1)
    # ax = fig.gca(projection='3d')
    pc.show_point_cloud(fig)

    temp = 1
