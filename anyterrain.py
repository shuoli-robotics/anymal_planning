import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d 
import math

class Terrain:
    def __init__(self):
        
        
        self.groundLength = 10.0
        self.secondGroundHeight = 3.0
        self.horizontalPlanes = np.array([[0.0,  0.0,  1.0,  0.0,                         0.0,                  self.groundLength,     0.0,                          self.groundLength/2],
                                       [0.0,  0.0,  1.0,  -self.secondGroundHeight,        0.0,                  self.groundLength,     self.groundLength,               self.groundLength*2]]) 
        
        self.firstStepPoint = [1.0,5.0,0.0]
        self.stepsNum = 10
        self.stepHight = self.secondGroundHeight/self.stepsNum
        self.stepLength = 2
        self.stepWidth = (self.groundLength - self.firstStepPoint[0])/self.stepsNum   
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
        
        fig = plt.figure(1)
        ax = fig.gca(projection='3d')
        ax.axis('auto')
        self.plotPlanes(ax,self.horizontalPlanes)
        self.plotPlanes(ax,self.slopeCoefficients)
        plt.show()


        
    def calculatePlaneCoefficient(self,controlPoints):
        A = controlPoints[0,:]
        B = controlPoints[1,:]
        C = controlPoints[2,:]
        
        n = np.cross(B-A,C-B)
        d = -n.dot(A)
        coefficient = np.append(n,d)
        #print(coefficient)
        return coefficient
    
    def plotPlanes(self,ax,coefficients):
        print(coefficients)
        for plane in range(coefficients.shape[0]):
            x = np.linspace(coefficients[plane,4],coefficients[plane,5],100)
            y = np.linspace(coefficients[plane,6],coefficients[plane,7],100)
            X,Y = np.meshgrid(x,y)
            Z = ((-coefficients[plane,0]*X-coefficients[plane,1]*Y)-coefficients[plane,3])/coefficients[plane,2]
            surf = ax.plot_surface(X, Y, Z,color = [0.3,0.3,0.3]) 
            print(plane)
    
            

if __name__ == "__main__":
    tr = Terrain()  