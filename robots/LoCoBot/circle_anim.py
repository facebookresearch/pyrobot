import numpy as np
from matplotlib import pyplot as plt
import time


# Running script in conjuction with command to move robot
# Will have to subscribe to something that tells me when the robot *actually* starts to move

robot_started = True
velocity = 0.2
distance = 1.8
time_to_goal = distance/velocity
init_time = int(time_to_goal)
fig, ax = plt.subplots()

if robot_started:
    start_time = time.time()

    # Initial plot
    an = np.linspace(0, 2 * np.pi, 100)
    ax.plot(3 * np.cos(an), 3 * np.sin(an), linewidth=6)
    ann=plt.annotate(xy=[-.25,0], s=init_time, size=30)
    ax.axis('equal')

    plt.ion() #set the interactive mode
    plt.show() # opens the interactive window and displays initial plot

    time_left = int(time_to_goal)


    while time.time() < (start_time + time_to_goal):
        plt.pause(1)
        ann.remove()
        time_left = time_left - 1
        ax.plot(3 * np.cos(an), 3 * np.sin(an), linewidth=6)
        ann=plt.annotate(xy=[-.25,0], s=time_left, size=30)
        plt.gcf().canvas.draw() #updates the altered figure



    ann.remove()
    ax.plot(3 * np.cos(an), 3 * np.sin(an), linewidth=6, color='green')
    ann = plt.annotate(xy=[-0.9,0], s="Goal", size=25)
    plt.gcf().canvas.draw()
    plt.pause(1)

