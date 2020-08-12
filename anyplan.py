import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d 
import math
from gurobipy import *

planesCoef = np.array([[0,0,1,0,0,5,0,5]])

model = Model('FootStep Planning')
N = 100
steps = [i for i in range(N)]
footIds = ['LF','RF','LH','RH']
states = ["x","y","z","theta"]
footstepStatesUpperBounds = {}
footstepStatesLowerBounds = {}
footstepStates = model.addVars(steps,states,name="footstepStates")

constr0 = model.addConstrs(planesCoef[0,0]*footstepStates[step,"x"]+planesCoef[0,1]*footstepStates[step,"y"]+planesCoef[0,2]*footstepStates[step,"z"]+planesCoef[0,3] == 0 for step in steps)

maxDistanceBetweenSteps = 0.2
model.addConstrs((footstepStates[step,state]-footstepStates[step-1,state] <= maxDistanceBetweenSteps for step in steps for state in states if step !=steps[0]),name="stepLimit")
model.addConstrs((footstepStates[step,state]-footstepStates[step-1,state] >= -maxDistanceBetweenSteps for step in steps for state in states if step !=steps[0]),name="stepLimit")

model.addConstrs(footstepStates[steps[0],state] <= maxDistanceBetweenSteps for state in states if state != states[-1])
model.addConstrs(footstepStates[steps[-1],state]-5.0 <= maxDistanceBetweenSteps for state in states if state != states[-1] and state != states[-2])
model.addConstrs(footstepStates[steps[-1],state]-5.0 >= -maxDistanceBetweenSteps for state in states if state != states[-1] and state != states[-2])
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
#print(optimalFootstep)

fig = plt.figure()
ax = fig.gca(projection='3d')
ax.axis('auto')
for plane in range(planesCoef.shape[0]):
    x = np.linspace(planesCoef[plane,4],planesCoef[plane,5],100)
    y = np.linspace(planesCoef[plane,6],planesCoef[plane,7],100)
    X,Y = np.meshgrid(x,y)
    Z = ((-planesCoef[plane,0]*X-planesCoef[plane,1]*Y)-planesCoef[plane,3])/planesCoef[plane,2]
    surf = ax.plot_surface(X, Y, Z)
ax.scatter(optimalFootstep[0,:],optimalFootstep[1,:],optimalFootstep[2,:])
plt.show()