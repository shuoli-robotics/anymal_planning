import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d 
import math
import anyterrain as at
from gurobipy import *

class AnyPlanner:
    def __init__(self):
        
        self.fig = plt.figure(1)
        self.ax = self.fig.gca(projection='3d')
        self.terrain = at.Terrain(self.ax,0)
        self.initialPoint = [0.0,0.0]
        self.targetPoint = [2.0,18.0]
        
    def planFootsteps(self):
        planesCoef = self.terrain.terrainPlanes
        print(planesCoef)
        R = planesCoef.shape[0]
        planes = np.array(range(R))

        model = Model('FootStep Planning')
        N = 50
        steps = [i for i in range(N)]
        footIds = ['LF','RF','LH','RH']
        states = ["x","y","z","theta"]
        footstepStatesUpperBounds = {}
        footstepStatesLowerBounds = {}

        footstepStates = model.addVars(steps,footIds,states,name="footstepStates")

        footstepAssignment = model.addVars(steps,footIds,planes,name="footstepAssignment",vtype=GRB.BINARY)


        # Assign each foothold on one plane
        constOnePlane = model.addConstrs((quicksum(footstepAssignment[step,footId,plane] for plane in planes) == 1 for step in steps for footId in footIds),name='constOnePlane')
        
        constOnThePlane0 = model.addConstrs(( (footstepAssignment[step,footId,plane]==1) >>(planesCoef[plane,0]*footstepStates[step,footId,'x']+planesCoef[plane,1]*footstepStates[step,footId,'y']+planesCoef[plane,2]*footstepStates[step,footId,'z']+planesCoef[plane,3] == 0) 
                                     for step in steps for footId in footIds for plane in planes ) ,name='constOnThePlane')
        constOnThePlane1 = model.addConstrs((footstepAssignment[step,footId,plane]==1) >> (footstepStates[step,footId,"x"] <= planesCoef[plane,5]) for step in steps for footId in footIds for plane in planes)
        constOnThePlane2 = model.addConstrs((footstepAssignment[step,footId,plane]==1) >> (footstepStates[step,footId,"x"] >= planesCoef[plane,4]) for step in steps for footId in footIds for plane in planes)
        constOnThePlane3 = model.addConstrs((footstepAssignment[step,footId,plane]==1) >> (footstepStates[step,footId,"y"] <= planesCoef[plane,7]) for step in steps for footId in footIds for plane in planes)
        constOnThePlane4 = model.addConstrs((footstepAssignment[step,footId,plane]==1) >> (footstepStates[step,footId,"y"] >= planesCoef[plane,6]) for step in steps for footId in footIds for plane in planes)
        
        # Limit the distance between each step for each leg
        maxDistanceBetweenSteps = 0.4
        model.addConstrs((footstepStates[step,footId,state]-footstepStates[step-1,footId,state] <= maxDistanceBetweenSteps for step in steps for footId in footIds for state in states if step !=steps[0]),name="stepLimit")
        model.addConstrs((footstepStates[step,footId,state]-footstepStates[step-1,footId,state] >= -maxDistanceBetweenSteps for step in steps for footId in footIds for state in states if step !=steps[0]),name="stepLimit")

        # Limit the distance between the legs for each step
        minDistanceBetweenLeftAndRight = 0.1
        maxDistanceBetweenLeftAndRight = 0.2
        minDistanceBetweenFrontAndHind= 0.2
        maxDistanceBetweenFrontAndHind= 0.6
        model.addConstrs((footstepStates[step,footIds[1],"x"] - footstepStates[step,footIds[0],"x"] >= minDistanceBetweenLeftAndRight for step in steps), name = "RF limit")
        model.addConstrs((footstepStates[step,footIds[1],"x"] - footstepStates[step,footIds[0],"x"] <=  maxDistanceBetweenLeftAndRight for step in steps), name = "RF limit")
        model.addConstrs((footstepStates[step,footIds[1],"y"] - footstepStates[step,footIds[0],"y"] >= -maxDistanceBetweenLeftAndRight for step in steps), name = "RF limit")
        model.addConstrs((footstepStates[step,footIds[1],"y"] - footstepStates[step,footIds[0],"y"] <=  maxDistanceBetweenLeftAndRight for step in steps), name = "RF limit")

        model.addConstrs((footstepStates[step,footIds[2],"x"] - footstepStates[step,footIds[0],"x"] <=  maxDistanceBetweenLeftAndRight for step in steps), name = "LH limit")
        model.addConstrs((footstepStates[step,footIds[2],"x"] - footstepStates[step,footIds[0],"x"] >=  -maxDistanceBetweenLeftAndRight for step in steps), name = "LH limit")
        model.addConstrs((footstepStates[step,footIds[0],"y"] - footstepStates[step,footIds[2],"y"] <=  maxDistanceBetweenFrontAndHind for step in steps), name = "LH limit")
        model.addConstrs((footstepStates[step,footIds[0],"y"] - footstepStates[step,footIds[2],"y"] >=  minDistanceBetweenFrontAndHind for step in steps), name = "LH limit")


        model.addConstrs((footstepStates[step,footIds[3],"x"] - footstepStates[step,footIds[1],"x"] <=  maxDistanceBetweenLeftAndRight for step in steps), name = "RH limit")
        model.addConstrs((footstepStates[step,footIds[3],"x"] - footstepStates[step,footIds[1],"x"] >=  -maxDistanceBetweenLeftAndRight for step in steps), name = "RH limit")
        model.addConstrs((footstepStates[step,footIds[1],"y"] - footstepStates[step,footIds[3],"y"] <=  maxDistanceBetweenFrontAndHind for step in steps), name = "RH limit")
        model.addConstrs((footstepStates[step,footIds[1],"y"] - footstepStates[step,footIds[3],"y"] >=  minDistanceBetweenFrontAndHind for step in steps), name = "RH limit")


        # Initial states and final states
        model.addConstrs(footstepStates[steps[0],footId,state] <= maxDistanceBetweenSteps for state in states for footId in footIds if state != states[-1])
        model.addConstrs(footstepStates[steps[-1],footId,states[0]]-self.targetPoint[0] <= maxDistanceBetweenSteps for footId in footIds)
        model.addConstrs(footstepStates[steps[-1],footId,states[0]]-self.targetPoint[0] >= -maxDistanceBetweenSteps for footId in footIds)
        model.addConstrs(footstepStates[steps[-1],footId,states[1]]-self.targetPoint[1] <= maxDistanceBetweenSteps for footId in footIds)
        model.addConstrs(footstepStates[steps[-1],footId,states[1]]-self.targetPoint[1] >= -maxDistanceBetweenSteps for footId in footIds)
        model.addConstrs(footstepStates[steps[0],footId,states[-1]] == 0.0 for footId in footIds)
        model.addConstrs(footstepStates[steps[-1],footId,states[-1]] == 0.0 for footId in footIds)
        
        obj = quicksum((footstepStates[step,footId,states[0]]-footstepStates[step-1,footId,states[0]])*(footstepStates[step,footId,states[0]]-footstepStates[step-1,footId,states[0]])+(footstepStates[step,footId,states[1]]-footstepStates[step-1,footId,states[1]])*(footstepStates[step,footId,states[1]]-footstepStates[step-1,footId,states[1]])
              for step in steps for footId in footIds if step != steps[0])
        model.setObjective(obj, GRB.MINIMIZE)
        model.optimize()
        
        optimalFootstepLF = np.zeros((3,N))
        optimalFootstepRF = np.zeros((3,N))
        optimalFootstepLH = np.zeros((3,N))
        optimalFootstepRH = np.zeros((3,N))
        solutions = model.getAttr('x', footstepStates)

        print(solutions)
        for solu in solutions:
            if solu[1] == 'LF':
                if solu[2] == 'x':
                    optimalFootstepLF[0,solu[0]] = solutions[solu]
                elif solu[2] == 'y':
                    optimalFootstepLF[1,solu[0]] = solutions[solu]
                elif solu[2] == 'z':
                    optimalFootstepLF[2,solu[0]] = solutions[solu]
            elif solu[1] == 'RF':
                if solu[2] == 'x':
                    optimalFootstepRF[0,solu[0]] = solutions[solu]
                elif solu[2] == 'y':
                    optimalFootstepRF[1,solu[0]] = solutions[solu]
                elif solu[2] == 'z':
                    optimalFootstepRF[2,solu[0]] = solutions[solu]
            elif solu[1] == 'LH':
                if solu[2] == 'x':
                    optimalFootstepLH[0,solu[0]] = solutions[solu]
                elif solu[2] == 'y':
                    optimalFootstepLH[1,solu[0]] = solutions[solu]
                elif solu[2] == 'z':
                    optimalFootstepLH[2,solu[0]] = solutions[solu]
            elif solu[1] == 'RH':
                if solu[2] == 'x':
                    optimalFootstepRH[0,solu[0]] = solutions[solu]
                elif solu[2] == 'y':
                    optimalFootstepRH[1,solu[0]] = solutions[solu]
                elif solu[2] == 'z':
                    optimalFootstepRH[2,solu[0]] = solutions[solu]

        print(optimalFootstepLF[:,0:5])
        print(optimalFootstepRF[:,0:5])
        print(optimalFootstepLH[:,0:5])
        print(optimalFootstepRH[:,0:5])

        self.ax.scatter(optimalFootstepLF[0,:],optimalFootstepLF[1,:],optimalFootstepLF[2,:],color = 'white')
        self.ax.scatter(optimalFootstepRF[0,:],optimalFootstepRF[1,:],optimalFootstepRF[2,:],color = 'red')
        self.ax.scatter(optimalFootstepLH[0,:],optimalFootstepLH[1,:],optimalFootstepLH[2,:],color = 'yellow')
        self.ax.scatter(optimalFootstepRH[0,:],optimalFootstepRH[1,:],optimalFootstepRH[2,:],color = 'green')

        self.ax.set_xlim([0,10])
        self.ax.set_ylim([0,20])
        self.ax.set_zlim([0,5])
        #self.set_axes_equal()
        
    def set_axes_equal(self):
        x_limits = self.ax.get_xlim3d()
        y_limits = self.ax.get_ylim3d()
        z_limits = self.ax.get_zlim3d()

        x_range = abs(x_limits[1] - x_limits[0])
        x_middle = np.mean(x_limits)
        y_range = abs(y_limits[1] - y_limits[0])
        y_middle = np.mean(y_limits)
        z_range = abs(z_limits[1] - z_limits[0])
        z_middle = np.mean(z_limits)
        plot_radius = 0.5*max([x_range, y_range, z_range])

        self.ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
        self.ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
        self.ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])
        plt.show()
        


if __name__ == "__main__":
    anyPlanner = AnyPlanner()  
    anyPlanner.planFootsteps()
    anyPlanner.set_axes_equal()
        
