import numpy as np
import matplotlib.pyplot as plt
import math

g = 9.8
z = 0.43
p_x = -0.1
def lip_model(state):
    A = np.array([[0,1],
                  [g/z,0]])
    B = np.array([0,-g*p_x/z])

    dx = A.dot(state)+B
    return dx



sim_time = 0.3
sim_step = 0.001
x0 = 0.0
v0 = 0

time = np.zeros(int(sim_time/sim_step))
states = np.zeros((len(time),2))
states_analytic = np.zeros(states.shape)

states[0] = np.array([x0,v0])
states_analytic[0] = np.array([x0,v0])

for i,state in enumerate(states):
    time[i] = i*sim_step
    omega = math.sqrt(g / z)
    A = np.array([[np.cosh(omega * time[i]), 1 / omega * np.sinh(omega * time[i])],
                  [omega * np.sinh(omega * time[i]), np.cosh(omega * time[i])]])
    B = np.array([1 - np.cosh(omega * time[i]), -omega * np.sinh(omega * time[i])])
    states_analytic[i] = A.dot(states_analytic[0]) + B * p_x

    if i == states.shape[0]-1:
        break
    states[i+1] = states[i] + sim_step * lip_model(states[i])




fig = plt.figure(0)
ax1 = plt.subplot(211)
ax1.plot(time,states[:,0])
ax1.plot(time,states_analytic[:,0])
ax2 = plt.subplot(212)
ax2.plot(time,states[:,1])
ax2.plot(time,states_analytic[:,1])
plt.show()

temp = 1