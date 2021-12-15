import torch
from torch.utils.data import TensorDataset, Dataset, DataLoader
import numpy as np

class NeuralNet(torch.nn.Module):
    # Change layers at make_model
    def __init__(self, input_size, output_size, layers=[128,128,128]):
        super().__init__()

        self.linear1 = torch.nn.Linear(input_size, layers[0])
        self.activation1 = torch.nn.Tanh()
        self.linear2 = torch.nn.Linear(layers[0], layers[1])
        self.activation2 = torch.nn.Tanh()
        self.linear3 = torch.nn.Linear(layers[1], layers[2])
        self.activation3 = torch.nn.Tanh()

        self.output_layer = torch.nn.Linear(layers[2], output_size)
        #self.output_activation = activation

        #initialize weights, following 'fan_avg' approach
        torch.nn.init.xavier_normal_(self.linear1.weight)
        torch.nn.init.xavier_normal_(self.linear2.weight)
        torch.nn.init.xavier_normal_(self.linear3.weight)
        torch.nn.init.xavier_normal_(self.output_layer.weight)

    def forward(self, inputs):
        x = self.activation1(self.linear1(inputs))
        x = self.activation2(self.linear2(x))
        x = self.activation3(self.linear3(x))
        #x = self.output_activation(self.output_layer(x))
        x = self.output_layer(x)
        return x

def make_model(input_dim=5, output_dim=6, layers=[128,128,128]):
	model = NeuralNet(input_dim, output_dim, layers)

	#model = model.cuda()
	device=torch.device("cpu")
	model.to(device)
	return model

