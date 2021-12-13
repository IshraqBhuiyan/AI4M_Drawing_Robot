#!/usr/bin/env python3

from matplotlib import pyplot as plt
from matplotlib import colors
import numpy as np
import time
import serial

width_size = 40
height_size = 24

grid_width = 109 / width_size
grid_height = 65 / height_size

def mouse_move(event):
    global m_x, m_y
    m_x,m_y = event.xdata, event.ydata
    print(m_x,m_y)

def key_event(event):
    global pressed
    global done
    pressed=True
    print("keypress")
    if(event.key=='d'):
        done = True

cmap = colors.ListedColormap(['b', 'w', 'g', 'r'])
#plt.rcParams["figure.figsize"] = [8.00, 4.80]
#plt.rcParams["figure.autolayout"] = True
plt.rcParams["toolbar"]= 'None'
#print(plt.rcParams['backend'])
img = np.zeros((height_size, width_size), dtype=int)
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
ax.imshow(img, cmap=cmap, vmin=0, vmax=3)
plt.connect('motion_notify_event', mouse_move)
plt.connect('key_press_event', key_event)
plt.ion()
plt.show()
#plt.draw()
plt.pause(0.01)

ser = serial.Serial('/dev/ttyUSB0')

pressed=False
print("Robot home")
while(not pressed):
    plt.pause(0.001)
    pass
time.sleep(4)
while(ser.in_waiting > 0):
    robotpos = ser.readline()

robotpos = robotpos.split()
robot_home_x = float(robotpos[0])
robot_home_y = float(robotpos[1])
robot_home_z = float(robotpos[2])

mouse_home_x = m_x
mouse_home_y = m_y

#print("Drawing")
#pressed=False
#while(not pressed):
#    plt.pause(0.001)

max_dist_x = 22 - robot_home_x
end_x = mouse_home_x + max_dist_x/grid_width
end_x = int(round(end_x))
start_x = int(round(mouse_home_x))
start_y = int(round(mouse_home_y))
end_y = start_y

img[start_y, start_x:end_x] = 1

ax.cla()
ax.imshow(img, cmap=cmap, vmin=0, vmax=3)
plt.show()
plt.pause(0.001)

done=False
print("Start drawing!")
last_draw = time.time()
while(np.any(img==1)):
    if(done):
        break
    plt.pause(0.001)
    if(ser.in_waiting>0):
        i=0
        buffer = ser.read(ser.in_waiting)
        buffer = buffer.split(b'\n')
        robotpos = buffer[-2]
        #while(ser.in_waiting > 0):
        #    plt.pause(0.001)
            #if(i>0):
            #    print("This many readlines missed %d" %(i))
        #    i=i+1
        #    robotpos = ser.readline()
        #    print("reading")
        print("This many lines missed %d" %(len(buffer)-1))
        print(buffer[-1])
        robotpos = robotpos.split()
        r_x = float(robotpos[0])
        r_y = float(robotpos[1])
        r_z = float(robotpos[2])
        print(r_x, r_y, r_z)
        draw_x = int(round(m_x))
        draw_y = int(round(m_y))
        # Maybe add a cursor for where the brush thinks it is
        if(img[draw_y, draw_x] < 2):
            if(img[draw_y, draw_x]==1):
                print("plot correct")
                img[draw_y, draw_x] = 2
            else:
                print("plot Incorrect")
                img[draw_y, draw_x] = 3
        if(time.time() - last_draw > 0.5):
            print("Plotting")
            ax.cla()
            plt.axis('off')
            ax.imshow(img, cmap=cmap, vmin=0, vmax=3)
            plt.show()
            plt.pause(0.001)
            print("Done plot")
            last_draw = time.time()

print("Done!")
print("Plotting and saving")
ax.cla()
plt.axis('off')
ax.imshow(img, cmap=cmap, vmin=0, vmax=3)
plt.savefig('Screen.png')
pressed=False
while(not pressed):
    plt.pause(0.001)