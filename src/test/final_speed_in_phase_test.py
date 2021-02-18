import numpy as np
import matplotlib.pyplot as plt
import math

class PhaseFinalSpeed:
    def __init__(self,step_dist):
        self.step_dist = step_dist
        self.time_range = np.linspace(0.1, 1, 200, endpoint=True)
        self.speed_range = np.linspace(0.1, 1, 200, endpoint=True)
        self.speed_diff = np.zeros((len(self.speed_range),len(self.time_range)))

    def calc_vel_diff(self):
        for i,t in enumerate(self.time_range):
            for j,v0 in enumerate(self.speed_range):
                g = 9.81
                z = 0.43
                p_x = self.step_dist
                omega = math.sqrt(g / z)
                A = np.array([[np.cosh(omega * t), 1 / omega * np.sinh(omega * t)],
                              [omega * np.sinh(omega * t), np.cosh(omega * t)]])
                B = np.array([1 - np.cosh(omega * t), -omega * np.sinh(omega * t)])

                x0 = np.array([0,v0])
                states_analytic = A.dot(x0) + B * p_x
                terminal_speed = states_analytic[1]
                self.speed_diff[i,j] = terminal_speed - v0

    def plot_speed_diff(self,ax):
        xx,yy = np.meshgrid(self.time_range, self.speed_range, sparse=True)
        # h = ax.contourf(self.time_range, self.speed_range, self.speed_diff)
        axis_label = [self.time_range[0], self.time_range[-1], self.speed_range[0], self.speed_range[-1]]
        h = ax.imshow(self.speed_diff,extent=axis_label,vmin = -2,vmax=2,aspect="auto")
        return h



if __name__ == "__main__":
    speed_diff_1 = PhaseFinalSpeed(0.1)
    speed_diff_1.calc_vel_diff()

    speed_diff_2 = PhaseFinalSpeed(0.2)
    speed_diff_2.calc_vel_diff()

    fig, (ax1,ax2) = plt.subplots(1,2)
    h1 = speed_diff_1.plot_speed_diff(ax1)
    # fig.colorbar(h1)

    h2 = speed_diff_2.plot_speed_diff(ax2)
    fig.colorbar(h2)
    plt.show()

    temp = 1