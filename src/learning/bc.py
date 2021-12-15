import torch
from torch.functional import Tensor
from torch.utils.data import TensorDataset, Dataset, DataLoader
import numpy as np
import net

expert1 = np.load("../experts/expert1.npz")
expert2 = np.load("../experts/expert2.npz")

calib_file = np.load("../calibration_soln4.npz", allow_pickle=True)
inv_file = np.load("../inv_soln4.npz", allow_pickle=True)

inv_x = inv_file['inv_x'] # Maps Pixel to Robot X
inv_z = inv_file['inv_z'] # Maps Pixel to Robot Z

forward_x = calib_file['pixel_x'] # Maps Robot to Pixel X
forward_y = calib_file['pixel_y'] # Maps Robot to Pixel Y

robot_traj1 = expert1['robot_traj']
robot_traj2 = expert2['robot_traj']
robot_traj = np.vstack([robot_traj1, robot_traj2])

finger_traj1 = expert1['finger_traj']
finger_traj2 = expert2['finger_traj']
finger_traj = np.vstack([finger_traj1, finger_traj2])

commands1 = expert1['commands']
commands2 = expert2['commands']
commands = np.hstack([commands1, commands2])

curr_goal1 = expert1['curr_goal']
curr_goal2 = expert2['curr_goal']
curr_goal = np.vstack([curr_goal1, curr_goal2])

touches1 = expert1['touches']
touches2 = expert2['touches']
touches = np.hstack([touches1, touches2])

command_map = {'+X':0, '-X':1, '+Z':2, '-Z':3, '+Y':4, '-Y':5}

commands_int = np.array(list(map(lambda x:command_map[x], commands)))
touches_int = np.array(list(map(lambda x: 1 if x else -1, touches)))

r_x = robot_traj[:,0]
r_y = robot_traj[:,1]
r_z = robot_traj[:,2]

# Computer Forward projected pixel location of stylus
A_r = np.vstack([r_x, r_x*r_x, r_z, r_z*r_z, np.ones_like(r_x)]).T
r_px = A_r @ forward_x
r_py = A_r @ forward_y

dx_touch = r_px - finger_traj[:,0]
dy_touch = r_py - finger_traj[:,1]

dx_goal = curr_goal[:,0] - r_px
dy_goal = curr_goal[:,1] - r_py

train_X = np.vstack([dx_goal, dy_goal, dx_touch, dy_touch, r_y]).T
train_Y = commands_int

train_X = torch.Tensor(train_X)
train_Y = torch.Tensor(train_Y).type(torch.long)

train_set = TensorDataset(train_X, train_Y)
train_loader = DataLoader(dataset=train_set, batch_size=64, shuffle=True)

model = net.make_model(input_dim=5)
criterion = torch.nn.CrossEntropyLoss()
optimizer = torch.optim.Adam(model.parameters())

num_epochs = 800
running_loss = 0
correct = 0
device = torch.device('cpu')


print("Start Training")
for epoch in range(num_epochs):
    curr_correct = 0
    curr_loss = 0
    for i,data in enumerate(train_loader, 0):
        x_batch, y_batch = data
        x_batch, y_batch = x_batch.to(device), y_batch.to(device)
        optimizer.zero_grad()
        yhat = model(x_batch)
        probs = yhat.softmax(dim=1)
        loss = criterion(yhat, y_batch)
        loss.backward()
        optimizer.step()
        curr_correct += (torch.argmax(probs, dim=1) == y_batch).cpu().float().sum()
        correct += (torch.argmax(probs, dim=1) == y_batch).cpu().float().sum()
        #print(loss)
        curr_loss +=loss.cpu().item()
        running_loss += loss.cpu().item()
    acc = curr_correct / (len(train_set))
    loss = curr_loss / (len(train_loader))
    print("Epoch %d accuracy %f running loss %f actual loss %f train set %d" %(epoch, acc, running_loss, loss, len(train_set)))

#from IPython import embed;
#embed()

fname = input("type name to save")
if(fname!=""):
    fname = fname+".pt"
    torch.save(model, fname)