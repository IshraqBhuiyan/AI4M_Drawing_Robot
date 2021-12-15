import torch
from torch.functional import Tensor
from torch.utils.data import TensorDataset, Dataset, DataLoader
import paho.mqtt.client as mqtt
import numpy as np
from net import NeuralNet, make_model
import time

model_path = "models/128_128_128_notouch_400ep_8774.pt"
#model_path = "models/128_128_128_touch_400ep_8746.pt"

calib_file = np.load("../calibration_soln4.npz", allow_pickle=True)
inv_file = np.load("../inv_soln4.npz", allow_pickle=True)

inv_x = inv_file['inv_x'] # Maps Pixel to Robot X
inv_z = inv_file['inv_z'] # Maps Pixel to Robot Z

forward_x = calib_file['pixel_x'] # Maps Robot to Pixel X
forward_y = calib_file['pixel_y'] # Maps Robot to Pixel Y

#model = net.make_model(input_dim=5, layers=[128,128,64])
#model.load_state_dict(torch.load(model_path))
model = torch.load(model_path)
model.eval()

curr_x = 0
curr_y = 0
curr_z = 0
touching=False
last_goal = np.array([0,0])
last_touch = np.zeros(2)
thresh = 20
line_traj = None
full_line_traj = None

send_command = False

end_traj = False

def on_message(client, userdata, message):
    global curr_x, curr_y, curr_z, touching, last_goal, send_command, last_touch, line_traj, end_traj
    if(message.topic == "end_test"):
        end_traj = True
    if(message.topic=="fup"):
        touching=False
        A_r = np.array([curr_x, curr_x*curr_x, curr_z, curr_z*curr_z, 1])
        p_x = A_r @ forward_x
        p_y = A_r @ forward_y
        last_touch[0] = p_x
        last_touch[1] = p_y
    if(message.topic=="robocomm" and line_traj.shape[0]>0):
        payload = message.payload
        payload = payload.decode()
        payload = payload.split()
        oldx = float(payload[1])
        oldy = float(payload[2])
        oldz = float(payload[3])
        curr_x = float(payload[4])
        curr_y = float(payload[5])
        curr_z = float(payload[6])
        send_command=False
    elif(message.topic == 'fdraw'):
        touching = True
        payload = message.payload
        payload = payload.decode()
        payload = payload.split()
        f_x = float(payload[0])
        f_y = float(payload[1])
        last_touch[0] = f_x
        last_touch[1] = f_y
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

mqttBroker = "127.0.0.1"
client = mqtt.Client("test_model")
client.connect(mqttBroker, keepalive=300)

r_start_x = 6 #6
r_start_z = -42 #42
line_length = 160

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

client.loop_start()
client.subscribe([("robotpos", 0), ("fingerdown", 0), ("fdraw",0), ("fup", 0), ('robocomm', 0), ("end_test", 0)])
client.on_message = on_message

cont = True
n=0

command_list = ["+X", "-X", "+Z", "-Z", "+Y", "-Y"]

while(cont):
    c = input("enter c to continue or Enter to end")
    if(c=='c'):
        sendstr = ""
        sendstr+= str(r_start_x) + " " + str(0) + " " + str(r_start_z)
        curr_x = r_start_x
        curr_y = 0
        curr_z = r_start_z
        client.publish("goto", sendstr)
        time.sleep(0.5)
        sendstr = ""
        sendstr+= str(r_start_x) + " " + str(-2) + " " + str(r_start_z)
        client.publish("goto", sendstr)
        curr_y = -2
        time.sleep(1)
        line_traj = line_traj_full[1:,:] # Maybe change this
        print("curr goal ", line_traj[0,:])
        client.publish("clear", " ")
        sendstr = ""
        sendstr += str(p_start_x) + " " + str(p_start_y) + " " + str(p_end_x) + " " + str(p_end_y)
        client.publish("drawline", sendstr)
        last_touch[0] = p_start_x
        last_touch[1] = p_start_y
        input("Press Enter to start test traj")
        i=0
        while(line_traj.shape[0]>0):
            if(end_traj):
                end_traj = False
                break
            while(send_command):
                pass
            curr_goal = line_traj[0,:]
            A_r = np.array([curr_x, curr_x*curr_x, curr_z, curr_z*curr_z, 1])
            r_px = A_r @ forward_x
            r_py = A_r @ forward_y

            dx_touch = r_px - last_touch[0]
            dy_touch = r_py - last_touch[1]

            dx_goal = curr_goal[0] - r_px
            dy_goal = curr_goal[1] - r_py

            touching_int = 1 if touching else -1

            input_x = np.array([dx_goal, dy_goal, dx_touch, dy_touch, curr_y])
            input_x = torch.Tensor(input_x)

            infer = model(input_x)
            i+=1
            probs = infer.softmax(dim=0)
            action = torch.distributions.Categorical(probs).sample().item()
            action_str = command_list[int(action)]
            print("Taking Action ", action_str)
            send_command = True
            client.publish("robostep", action_str)
            time.sleep(0.5)
        fname = "notouch_" + "test_" + str(n) + "_num_steps_" + str(i) +"_start_" + str(r_start_x) + "_" + str(-r_start_z)+ ".tga"
        client.publish("save", fname)
        n+=1
    else:
        break