#!/usr/bin/env python3
from matplotlib import pyplot as plt
import numpy as np
import time
x = 0
y=0
def mouse_move(event):
    global x,y
    x,y = event.xdata, event.ydata
    print(x,y)

def key_event(event):
    global pressed
    pressed=True
    print("Pressed")

#plt.ion()
#plt.rcParams["figure.figsize"] = [8.00, 4.80]
#plt.rcParams["figure.autolayout"] = True
plt.rcParams["toolbar"]= 'None'
#print(plt.rcParams['backend'])
img = np.zeros((24, 40), dtype=int)
img[0,:] = 1
img[:,0] = 1
img[-1,:] = 1
img[:,-1] = 1
fig = plt.figure()
ax = fig.add_axes((0,0, 1, 1))
#plt.plot([0,1],[0,2])
manager = plt.get_current_fig_manager()
window = manager.window
width = window.winfo_screenwidth()
height = window.winfo_screenheight()
manager.resize(window.winfo_screenwidth(), window.winfo_screenheight())
plt.axis('off')
plt.box(False)
#plt.margins(0,0)
#plt.autoscale(tight=True)
ax.imshow(img)
#plt.xlim([0,1])
#plt.ylim([0,2])
print(width)
print(height)
plt.connect('motion_notify_event', mouse_move)
plt.connect('key_press_event', key_event)
plt.ion()
plt.show()
#plt.pause(10)

pressed = False
while(not pressed):
    plt.pause(0.01)

#plt.pause(10)
#print("Mouse event %f %f" %(x,y))

#input()