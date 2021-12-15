import paho.mqtt.client as mqtt
import numpy as np
from scipy import stats
import time

last_robot_pos = np.zeros(3)
fdown_array = np.empty((0,2))
robot_pos_array = np.empty((0,3))

def on_message(client, userdata, message):
    if(message.topic=="robotpos"):
        on_robot(client, userdata, message)
    elif(message.topic=="fingerdown"):
        on_fdown(client, userdata, message)

def on_fdown(client, userdata, message):
    #print("received fdown message ", str(message.payload))
    global fdown_array, last_robot_pos, robot_pos_array
    payload = message.payload
    payload = payload.split()
    t_x = float(payload[0])
    t_y = float(payload[1])
    fdown_array = np.vstack([fdown_array, [t_x, t_y]])
    robot_pos_array = np.vstack([robot_pos_array, last_robot_pos])
    print("fdown")
    print(t_x, t_y)

def on_robot(client, userdata, message):
    #print("received robot message", str(message.payload))
    global fdown_array, last_robot_pos, robot_pos_array
    payload = message.payload
    payload = payload.split()
    r_x = float(payload[0])
    r_y = float(payload[1])
    r_z = float(payload[2])
    print("robot")
    print(r_x, r_y, r_z)
    last_robot_pos[:] = [r_x, r_y, r_z]

mqttBroker = "127.0.0.1"

client=mqtt.Client("calibration")
client.connect(mqttBroker, keepalive=300)

client.loop_start()
client.subscribe([("robotpos",0), ("fingerdown",0)])
client.on_message = on_message

#client.loop_forever()
input("Press Enter when done collecting")
client.disconnect()
np.savez("calibration_data4.npz", fdown_array=fdown_array, robot_pos_array=robot_pos_array)

r_x = robot_pos_array[:,0]
r_y = robot_pos_array[:,1]
r_z = robot_pos_array[:,2]

t_x = fdown_array[:,0]
t_y = fdown_array[:,1]

A = np.vstack([r_x, r_x*r_x, r_z, r_z*r_z, np.ones_like(r_x)]).T
y = np.vstack([t_x, t_y]).T

soln = np.linalg.lstsq(A, y, rcond=None)

pixel_x = soln[0][:,0]
pixel_y = soln[0][:,1]

res_x = soln[1][0]
res_y = soln[1][1]

r2_x = 1 - res_x / (y.shape[0] * y[:,0].var())
r2_y = 1 - res_y / (y.shape[0] * y[:,1].var())

print("r2_x ", r2_x)
print("r2_y ", r2_y)

residuals_x = A @ pixel_x - y[:,0]
residuals_y = A @ pixel_y - y[:,1]

mean_x = np.mean(np.abs(residuals_x))
mean_y = np.mean(np.abs(residuals_y))

std_x = np.std(np.abs(residuals_x))
std_y = np.std(np.abs(residuals_y))

print("mean_x ", mean_x)
print("mean_y ", mean_y)

print("std x ", std_x)
print("std y ", std_y)

soln = np.array(soln, dtype=object)
np.savez("calibration_soln4.npz", soln=soln, pixel_x=pixel_x, pixel_y = pixel_y, res_x = res_x, res_y=res_y, r2_x=r2_x, r2_y=r2_y, mean_x=mean_x, mean_y=mean_y, t_x=t_x, t_y=t_y, r_x=r_x, r_y=r_y, r_z=r_z, std_x=std_x, std_y=std_y)