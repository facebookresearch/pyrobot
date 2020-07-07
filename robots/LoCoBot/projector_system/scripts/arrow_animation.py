#!/usr/bin/env python


import numpy as np
import matplotlib.pyplot as plt


fig, ax = plt.subplots()
an = np.linspace(0, 2 * np.pi, 100)
ax.plot(3 * np.cos(an), 3 * np.sin(an), linewidth=6)

ann = plt.annotate('', xytext = (0, 0),xy = (0, 2.8), arrowprops=dict(facecolor='black', shrink=0.05))


ax.axis('equal')

plt.ion() #set the interactive mode
plt.show() # opens the interactive window and displays initial plot


#circle_poses = [(-.93, 2.64), (-1.86, 1.88), (-2.64)

plt.pause(1)
ann.remove()
ax.plot(3 * np.cos(an), 3 * np.sin(an), linewidth=6)

ann = plt.annotate('', xytext = (0, 0),xy = (-.75, 2.5), arrowprops=dict(facecolor='black', shrink=0.05))

plt.gcf().canvas.draw() #updates the altered figure

plt.pause(1)

