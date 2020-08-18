import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d 
import math

class Terrain:
    def __init__(self,ax,terrainNum):
        self.terrainPlanes = self.createTerrian(ax,terrainNum)
        
        
    def createTerrian(self,ax,terrainNum):
        if terrainNum == 0:
            groundLength = 10.0
            secondGroundHeight = 3.0
            lowCovariance = 0.1
            highCovariance = 1.0
            horizontalPlanes = np.array([[0.0,  0.0,  1.0,  0.0,                         0.0,                  groundLength,     0.0,                          groundLength/2,   lowCovariance],
                                        [0.0,  0.0,  1.0,  -secondGroundHeight,        0.0,                  groundLength,     groundLength,               groundLength*2,       lowCovariance]]) 
            
            firstStepPoint = [1.0,5.0,0.0]
            stepsNum = 15
            stepHight = secondGroundHeight/stepsNum
            stepLength = 2
            stepWidth = (groundLength - firstStepPoint[1])/stepsNum   
            levelSteps = np.zeros((stepsNum,9))
            for i in range(stepsNum-1):
                levelSteps[i,:] = np.array([0.0,  0.0,  1.0, -(i+1)*stepHight,   firstStepPoint[0],   firstStepPoint[0]+stepLength,  firstStepPoint[1]+i*stepWidth, firstStepPoint[1]+(i+1)*stepWidth, highCovariance]) 
            horizontalPlanes = np.concatenate((horizontalPlanes, levelSteps), axis=0) 
            
            firstSlopePoint = [7.0,5.0,0.0]
            slopeControlPoints = np.array([firstSlopePoint,
                                                [firstSlopePoint[0]+stepLength,firstSlopePoint[1],0],
                                                [firstSlopePoint[0],groundLength,secondGroundHeight],
                                                [firstSlopePoint[0]+stepLength,groundLength,secondGroundHeight]])
            #print(slopeControlPoints)
        
            slopeCoefficients = self.calculatePlaneCoefficient(slopeControlPoints)
            slopeLimit = np.array([firstSlopePoint[0],firstSlopePoint[0]+stepLength,firstSlopePoint[1],groundLength])
            # print(slopeLimit)
            slopeCoefficients = np.append(slopeCoefficients,slopeLimit)
            slopeCoefficients = np.append(slopeCoefficients,np.array([lowCovariance]))
            #print(slopeCoefficients)
            slopeCoefficients = np.array([slopeCoefficients])
            
            terrainPlanes = np.concatenate((horizontalPlanes,slopeCoefficients))
            
            self.plotPlanes(ax,terrainPlanes)
            
            return(terrainPlanes)
        else:
            print("Establish Terrain Failed")
            
               
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
    
            

    
            

if __name__ == "__main__":
    fig = plt.figure(1)
    ax = fig.gca(projection='3d')
    tr = Terrain(ax,0)  
    plt.show()
