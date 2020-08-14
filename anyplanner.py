import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d 
import math
import anyterrain as at
from gurobipy import *

class AnyPlanner:
    def __init__(self):
        self.terrain = at.Terrain(1)
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
        self.terrain.ax.scatter(optimalFootstep[0,:],optimalFootstep[1,:],optimalFootstep[2,:])
        
        plt.show()
        
        


if __name__ == "__main__":
    anyPlanner = AnyPlanner()  
    anyPlanner.planFootsteps()
        