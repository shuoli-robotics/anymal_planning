import sys
import os
sys.path.insert(0, os.path.abspath('../src'))
import numpy as np
import copy
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d 
import math
import anyterrain as at
import time
import aStarLocal
import aStarGlobal
from mayavi import mlab


terrain = at.Terrain(0)
fig = mlab.figure(1)
terrain.plotPlanes(fig)


startGlobal = (5.0,1.0)
goalGlobal = (5.0,18.0)
startLocal = (startGlobal[0],startGlobal[1],3.14/2)

flagArrivedTheGoal = False

counter = 0

while True:
    globalPlanner = aStarGlobal.AnymalAStarGlobal(startGlobal,goalGlobal,terrain)
    globalPlanner.run()
    optimalPath, localTarget = globalPlanner.getOptimalPath(0.7)
    refinedLocalTarget = globalPlanner.refineTarget(localTarget)
    
    localTarget = (refinedLocalTarget[0],refinedLocalTarget[1],3.14/2)
    
    # print("localTarget =",localTarget,"startLocal = ",startLocal)
    
    localPlanner = aStarLocal.AnymalAStarLocal(startLocal,localTarget,terrain)
    localPlanner.run()
    localPlanner.plotOptimalPath(fig)
    
    startGlobal = (localTarget[0],localTarget[1])
    startLocal = localTarget
    
    counter = counter + 1
    if counter == 10:
        break
    
    
mlab.show()    