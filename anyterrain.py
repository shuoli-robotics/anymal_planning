import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d 
import math

class Terrain:
    def __init__(self,figNum):
        
        
        self.groundLength = 10.0
        self.secondGroundHeight = 3.0
        self.horizontalPlanes = np.array([[0.0,  0.0,  1.0,  0.0,                         0.0,                  self.groundLength,     0.0,                          self.groundLength/2],
                                       [0.0,  0.0,  1.0,  -self.secondGroundHeight,        0.0,                  self.groundLength,     self.groundLength,               self.groundLength*2]]) 
        
        self.firstStepPoint = [1.0,5.0,0.0]
        self.stepsNum = 15
        self.stepHight = self.secondGroundHeight/self.stepsNum
        self.stepLength = 2
        self.stepWidth = (self.groundLength - self.firstStepPoint[1])/self.stepsNum   
        self.levelSteps = np.zeros((self.stepsNum,8))
        for i in range(self.stepsNum-1):
            self.levelSteps[i,:] = np.array([0.0,  0.0,  1.0, -(i+1)*self.stepHight,   self.firstStepPoint[0],   self.firstStepPoint[0]+self.stepLength,  self.firstStepPoint[1]+i*self.stepWidth, self.firstStepPoint[1]+(i+1)*self.stepWidth]) 
        self.horizontalPlanes = np.concatenate((self.horizontalPlanes, self.levelSteps), axis=0) 
        
        self.firstSlopePoint = [7.0,5.0,0.0]
        self.slopeControlPoints = np.array([self.firstSlopePoint,
                                            [self.firstSlopePoint[0]+self.stepLength,self.firstSlopePoint[1],0],
                                            [self.firstSlopePoint[0],self.groundLength,self.secondGroundHeight],
                                            [self.firstSlopePoint[0]+self.stepLength,self.groundLength,self.secondGroundHeight]])
        #print(self.slopeControlPoints)
    
        self.slopeCoefficients = self.calculatePlaneCoefficient(self.slopeControlPoints)
        slopeLimit = np.array([self.firstSlopePoint[0],self.firstSlopePoint[0]+self.stepLength,self.firstSlopePoint[1],self.groundLength])
        # print(self.slopeCoefficients)
        # print(slopeLimit)
        self.slopeCoefficients = np.append(self.slopeCoefficients,slopeLimit)
        self.slopeCoefficients = np.array([self.slopeCoefficients])
        
        print(self.horizontalPlanes.shape)
        print(self.slopeCoefficients.shape)
        self.terrainPlanes = np.concatenate((self.horizontalPlanes,self.slopeCoefficients))
        
        print(self.terrainPlanes)
        
        
        self.fig = plt.figure(figNum)
        self.ax = self.fig.gca(projection='3d')
        self.ax.axis('auto')
        self.plotPlanes(self.ax,self.horizontalPlanes)
        self.plotPlanes(self.ax,self.slopeCoefficients)
        self.set_axes_equal(self.ax)
        #plt.show()


        
    def calculatePlaneCoefficient(self,controlPoints):
        A = controlPoints[0,:]
        B = controlPoints[1,:]
        C = controlPoints[2,:]
        
        n = np.cross(B-A,C-B)
        d = -n.dot(A)
        coefficient = np.append(n,d)
        return coefficient
    
    def plotPlanes(self,ax,coefficients):
        for plane in range(coefficients.shape[0]):
            x = np.linspace(coefficients[plane,4],coefficients[plane,5],100)
            y = np.linspace(coefficients[plane,6],coefficients[plane,7],100)
            X,Y = np.meshgrid(x,y)
            Z = ((-coefficients[plane,0]*X-coefficients[plane,1]*Y)-coefficients[plane,3])/coefficients[plane,2]
            surf = ax.plot_surface(X, Y, Z,color = [0.3,0.3,0.3])
    
    def getPlanesCoefficients(self):
         return self.terrainPlanes
            
    def set_axes_equal(self,ax):
        '''Make axes of 3D plot have equal scale so that spheres appear as spheres,
        cubes as cubes, etc..  This is one possible solution to Matplotlib's
        ax.set_aspect('equal') and ax.axis('equal') not working for 3D.

        Input
        ax: a matplotlib axis, e.g., as output from plt.gca().
        '''

        x_limits = ax.get_xlim3d()
        y_limits = ax.get_ylim3d()
        z_limits = ax.get_zlim3d()

        x_range = abs(x_limits[1] - x_limits[0])
        x_middle = np.mean(x_limits)
        y_range = abs(y_limits[1] - y_limits[0])
        y_middle = np.mean(y_limits)
        z_range = abs(z_limits[1] - z_limits[0])
        z_middle = np.mean(z_limits)

        # The plot bounding box is a sphere in the sense of the infinity
        # norm, hence I call half the max range the plot radius.
        plot_radius = 0.5*max([x_range, y_range, z_range])

        ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
        ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
        ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])
    
            

if __name__ == "__main__":
    tr = Terrain(figNum = 1)  