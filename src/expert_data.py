import numpy as np
import paho.mqtt.client as mqtt
import time

calib_file = np.load("calibration_soln4.npz", allow_pickle=True)
inv_file = np.load("inv_soln4.npz", allow_pickle=True)

inv_x = inv_file['inv_x'] # Maps Pixel to Robot X
inv_z = inv_file['inv_z'] # Maps Pixel to Robot Z

forward_x = calib_file['pixel_x'] # Maps Robot to Pixel X
forward_y = calib_file['pixel_y'] # Maps Robot to Pixel Y

touching = False
last_touch = np.zeros(2)
last_robot_pos = np.zeros(3)
robot_traj = np.empty((0,3))
finger_traj = np.empty((0,2))
commands = np.array([], dtype=str)
touches = np.array([])
curr_goal = np.empty((0,2))
line_traj = None
full_line_traj = None
r_start_x = 0
r_start_z = 0
last_goal = np.array([0,0])

thresh = 20

def on_message(client, userdata, message):
    global line_traj, touching, last_touch, last_robot_pos, robot_traj, finger_traj, commands, curr_goal, last_goal, touches
    if(message.topic=="fup"):
        touching = False
    if(message.topic=="robocomm" and line_traj.shape[0]>0):
        payload = message.payload
        payload = str(payload.decode())
        payload = payload.split()
        oldx = float(payload[1])
        oldy = float(payload[2])
        oldz = float(payload[3])
        robot_traj = np.vstack([robot_traj, [oldx, oldy, oldz]])
        if(touching):
            finger_traj = np.vstack([finger_traj, last_touch])
        else:
            A_r = np.array([oldx, oldx*oldx, oldz, oldz*oldz, 1])
            p_x = A_r @ forward_x
            p_y = A_r @ forward_y
            last_touch[0] = p_x
            last_touch[1] = p_y
            finger_traj = np.vstack([finger_traj, last_touch])
        commands = np.append(commands, payload[0])
        curr_goal = np.vstack([curr_goal, line_traj[0,:]])
        touches = np.append(touches, touching)
        if(False and payload[0] == '+Y'): #Ignore the resetting
            pos_pix_x = last_goal[0]
            pos_pix_y = last_goal[1]
            A = np.array([pos_pix_x, pos_pix_x*pos_pix_x, pos_pix_y, pos_pix_y*pos_pix_y, 1])
            r_x = A @ inv_x
            r_z = A @ inv_z
            r_y = 0
            sendstr = ""
            sendstr+= str(r_x) + " " + str(r_y) + " " + str(r_z)
            client.publish("goto", sendstr)
            time.sleep(0.5)
            r_y = -3
            sendstr = ""
            sendstr += str(r_x) + " " + str(-3) + " " + str(r_z)
            client.publish("goto", sendstr)
            time.sleep(0.5)
    elif(message.topic=='fdraw'):
        touching=True
        payload = message.payload
        payload = str(message.payload.decode())
        payload = payload.split()
        f_x = float(payload[0])
        f_y = float(payload[1])
        last_touch[0] = f_x
        last_touch[1] = f_y
        #print("drawn")
        if(line_traj.shape[0]>0):
            curr_traj_x = line_traj[0,0]
            curr_traj_y = line_traj[0,1]
            print(curr_traj_x, curr_traj_y)
            print("last touch ", f_x, f_y)
            if(abs(f_x - curr_traj_x)<=thresh and abs(f_y - curr_traj_y)<=thresh):
                if(line_traj.shape[0]>1):
                    last_goal = line_traj[0,:]
                    line_traj = line_traj[1:,:]
                else:
                    line_traj = np.empty((0,2))
                    print("Trajectory Completed!")
                print("trajectory length ", line_traj.shape[0])

r_start_x = 6
r_start_z = -42
line_length = 160 #pixels wide

A_r = np.array([r_start_x, r_start_x*r_start_x, r_start_z, r_start_z*r_start_z, 1])
p_start_x = A_r @ forward_x
p_start_y = A_r @ forward_y

p_start_x = int(p_start_x)
p_start_y = int(p_start_y)
p_end_x = p_start_x + line_length
p_end_y = p_start_y

spacing = 10 #pixels

line_traj_x = np.arange(p_start_x, p_end_x, spacing)
line_traj_x = np.append(line_traj_x, p_end_x)

line_traj_full = np.vstack([line_traj_x, p_start_y * np.ones_like(line_traj_x)]).T

line_traj = line_traj_full
line_traj = line_traj[1:,:]
last_goal = line_traj_full[0,:]

mqttBroker = "127.0.0.1"
client = mqtt.Client("expert_data")
client.connect(mqttBroker, keepalive=300)

client.loop_start()
client.subscribe([("robotpos", 0), ("fingerdown", 0), ("fdraw",0), ("fup", 0), ('robocomm', 0)])
client.on_message = on_message

cont = True
n = 0
while(cont):
    c = input("enter c to continue or anything else to end")
    if(c=='c'):
        if(n>0):
            fname = "expert" + str(n) + ".tga"
            client.publish("save", fname)
        sendstr = ""
        sendstr+= str(r_start_x) + " " + str(0) + " " + str(r_start_z)
        client.publish("goto", sendstr)
        time.sleep(0.5)
        #sendstr = ""
        #sendstr+= str(r_start_x) + " " + str(-3) + " " + str(r_start_z)
        #client.publish("goto", sendstr)
        line_traj = line_traj_full[1:,:]
        print("curr goal ", line_traj[0,:])
        client.publish("clear", " ")
        sendstr = ""
        sendstr += str(p_start_x) + " " + str(p_start_y) + " " + str(p_end_x) + " " + str(p_end_y)
        client.publish("drawline", sendstr)
        last_touch[0] = p_start_x
        last_touch[1] = p_start_y
        n+=1
    else:
        cont=False
        client.disconnect()
        break

np.savez("expert.npz", robot_traj = robot_traj, finger_traj=finger_traj, commands=commands, curr_goal=curr_goal, full_line_traj=full_line_traj, touches=touches)
