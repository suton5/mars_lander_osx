#!/usr/bin/python

import numpy as np
import matplotlib.pyplot as plt

# mass, spring constant, initial position and velocity
m = 1
k = 1
x = 0
v = 1

# simulation time, timestep and time
t_max = 100
dt = [1, 0.1, 0.01]

for interval in dt:
    
    t_array = np.arange(0, t_max, interval)

    # initialise empty lists to record trajectories
    x_list = []
    v_list = []

    # Euler integration
    for t in t_array:

        # append current state to trajectories
        x_list.append(x)
        v_list.append(v)

        # calculate new position and velocity
        a = -k * x / m
        x = x + interval * v
        v = v + interval * a

    # convert trajectory lists into arrays, so they can be indexed more easily
    x_array = np.array(x_list)
    v_array = np.array(v_list)
 
    # plot the position-time graph
    plt.figure(1)
    plt.clf()
    plt.xlabel('time (s)')
    plt.title('For dt=' + str(interval))
    plt.grid()
    plt.plot(t_array, x_array, label='x (m)')
    plt.plot(t_array, v_array, label='v (m/s)')
    plt.legend()
    plt.show()
