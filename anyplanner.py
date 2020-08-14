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
        footstepStates = model.addVars(steps,states,name="footstepStates")

        footstepAssignment = model.addVars(steps,planes,name="footstepAssignment",vtype=GRB.BINARY)

        constOnePlane = model.addConstrs((quicksum(footstepAssignment[step,plane] for plane in planes) == 1 for step in steps),name='constOnePlane')
        
        constOnThePlane0 = model.addConstrs(( (footstepAssignment[step,plane]==1) >>(planesCoef[plane,0]*footstepStates[step,'x']+planesCoef[plane,1]*footstepStates[step,'y']+planesCoef[plane,2]*footstepStates[step,'z']+planesCoef[plane,3] == 0) 
                                     for step in steps for plane in planes) ,name='constOnThePlane')
        constOnThePlane1 = model.addConstrs((footstepAssignment[step,plane]==1) >> (footstepStates[step,"x"] <= planesCoef[plane,5]) for step in steps for plane in planes)
        constOnThePlane2 = model.addConstrs((footstepAssignment[step,plane]==1) >> (footstepStates[step,"x"] >= planesCoef[plane,4]) for step in steps for plane in planes)
        constOnThePlane3 = model.addConstrs((footstepAssignment[step,plane]==1) >> (footstepStates[step,"y"] <= planesCoef[plane,7]) for step in steps for plane in planes)
        constOnThePlane4 = model.addConstrs((footstepAssignment[step,plane]==1) >> (footstepStates[step,"y"] >= planesCoef[plane,6]) for step in steps for plane in planes)
        
        maxDistanceBetweenSteps = 0.4
        model.addConstrs((footstepStates[step,state]-footstepStates[step-1,state] <= maxDistanceBetweenSteps for step in steps for state in states if step !=steps[0]),name="stepLimit")
        model.addConstrs((footstepStates[step,state]-footstepStates[step-1,state] >= -maxDistanceBetweenSteps for step in steps for state in states if step !=steps[0]),name="stepLimit")
        
        model.addConstrs(footstepStates[steps[0],state] <= maxDistanceBetweenSteps for state in states if state != states[-1])
        model.addConstr(footstepStates[steps[-1],states[0]]-self.targetPoint[0] <= maxDistanceBetweenSteps)
        model.addConstr(footstepStates[steps[-1],states[0]]-self.targetPoint[0] >= -maxDistanceBetweenSteps)
        model.addConstr(footstepStates[steps[-1],states[1]]-self.targetPoint[1] <= maxDistanceBetweenSteps)
        model.addConstr(footstepStates[steps[-1],states[1]]-self.targetPoint[1] >= -maxDistanceBetweenSteps)
    
        model.addConstr(footstepStates[steps[0],states[-1]] == 0.0)
        model.addConstr(footstepStates[steps[-1],states[-1]] == 0.0)
        
        obj = quicksum((footstepStates[step,states[0]]-footstepStates[step-1,states[0]])*(footstepStates[step,states[0]]-footstepStates[step-1,states[0]])+(footstepStates[step,states[1]]-footstepStates[step-1,states[1]])*(footstepStates[step,states[1]]-footstepStates[step-1,states[1]])
              for step in steps if step != steps[0])
        model.setObjective(obj, GRB.MINIMIZE)
        model.optimize()
        
        optimalFootstep = np.zeros((3,N))
        solutions = model.getAttr('x', footstepStates)
        #print(solutions)
        for solu in solutions:
            if solu[1] == 'x':
                optimalFootstep[0,solu[0]] = solutions[solu]
            elif solu[1] == 'y':
                optimalFootstep[1,solu[0]] = solutions[solu]
            elif solu[1] == 'z':
                optimalFootstep[2,solu[0]] = solutions[solu]
        self.ax.scatter(optimalFootstep[0,:],optimalFootstep[1,:],optimalFootstep[2,:])
        self.set_axes_equal()
        
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
        