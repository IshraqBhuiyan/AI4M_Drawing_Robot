#!/usr/bin/env python3
from matplotlib import pyplot as plt
import numpy as np
import time

plt.rcParams["figure.figsize"] = [8.00, 4.80]
#plt.rcParams["figure.autolayout"] = True
plt.rcParams["toolbar"]= 'None'
#print(plt.rcParams['backend'])
img = np.zeros((24, 40), dtype=int)
fig = plt.figure()

#plt.plot([0,1],[0,2])
manager = plt.get_current_fig_manager()
window = manager.window
width = window.winfo_screenwidth()
height = window.winfo_screenheight()
manager.resize(window.winfo_screenwidth(), window.winfo_screenheight())


plt.ion()
plt.show()
ax = fig.add_axes((0,0, 1, 1))
plt.axis('off')
plt.box(False)
ax.imshow(img)
plt.draw()
plt.pause(10)

#for x in range(img.shape[1]):
#    for y in range(img.shape[0]):
#        ax.clear()
        #plt.cla or clf
#        img[y,x] = 1
        #ax = fig.add_axes((0,0, 1, 1))
        #plt.axis('off')
        #plt.box(False)
#        ax.imshow(img)
        #plt.draw()
#        plt.draw()
#        plt.pause(0.001)

#print(width)
#print(height)
#plt.show()