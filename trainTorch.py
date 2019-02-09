import json
import logging
import sys, os
import shutil
import argparse
import torch
import torch.nn as nn
import torch.optim
import torch.utils.data
import matplotlib.pyplot as plt
import numpy as np
import scipy.io as sci
from torch.utils.data import Dataset, DataLoader
from pdb import set_trace as bp

###########################################################
# Parse Arguments
###########################################################
parser = argparse.ArgumentParser(description='Train Classifier')
parser.add_argument('--epoch', type=int, default=10, metavar='N', help='number of epochs (default: 10)')
parser.add_argument('--ratio', type=float, default=0.8, metavar='N', help='train to test ratio (default: 0.8)')
parser.add_argument('--batch', type=int, default=2048, metavar='N', help='batch size (default: 2048)')
parser.add_argument('--shuffle', type=bool, default=True, metavar='N', help='shuffle the dataset (default: True)')
parser.add_argument('--lr', type=float, default=0.0001, metavar='N', help='learning rate (default: 0.0001)')
parser.add_argument('--weight_decay', type=float, default=1e-5, metavar='N', help='weight decay (default: 1e-5)')
parser.add_argument('--save', type=bool, default=False, metavar='N', help='save model (default: False)')

args = parser.parse_args()

#######################################################
# Define Deep Neural Network
#######################################################
device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

class DNN(nn.Module):
    def __init__(self):
        super(DNN, self).__init__()

        self.autoenc = nn.Sequential(
            nn.Linear(2, 10),
            nn.ReLU(),
            nn.Linear(10, 20),
            nn.ReLU(),
            nn.Linear(20, 50),
            nn.ReLU(),
            nn.Linear(50, 100),
            nn.ReLU(),
            nn.Linear(100, 50),
            nn.ReLU(),
            nn.Linear(50, 20),
            nn.ReLU(),
            nn.Linear(20, 10),
            nn.ReLU(),
            nn.Linear(10, 2),
            nn.Softmax(dim=-1),
        )

    def forward(self, x):
        res = self.autoenc(x)
        return res

#######################################################
# Make Data Loader Class
#######################################################
class PylayersDataset(Dataset):
    def __init__(self, mat="assets/data/trainLarge.mat", transform=None):
        self.mat = sci.loadmat(mat)
        self.data = []
        self.label = []
        self.size = 0
        self.getData(self.mat['rssi'][:], self.mat['euclid'][:], self.mat['labels'][:])
        self.transform = transform

    def getData(self, r, e, l):
        self.size = len(r)
        for i in range(len(r)):
            self.data.append([[r[i][0],e[i][0]]])
            if l[i][0]==0: self.label.append([1,0])
            else: self.label.append([0,1])

        self.data = torch.tensor(self.data, dtype=torch.float64)
        self.label = torch.tensor(self.label, dtype=torch.float64)

    def __len__(self):
        return self.size

    def __getitem__(self, idx):
        return {'data':self.data[idx], 'label':self.label[idx]}


data = PylayersDataset()

MAX_EPOCH = args.epoch
BATCH_SIZE = args.batch
shuffle = args.shuffle

total = len(data)

dataloader = DataLoader(data, batch_size=BATCH_SIZE, shuffle=shuffle, num_workers=4)

#######################################################
# Make Model Object
#######################################################
model = DNN().to(device)
optimizer = torch.optim.Adam(model.parameters(), lr=args.lr, weight_decay=args.weight_decay)
loss_func = nn.MSELoss()

# Start Shuffling the Data
for epoch in range(MAX_EPOCH):
    for i, data in enumerate(dataloader):
        optimizer.zero_grad()
        output = model(data['data'].float().to(device))
        loss = loss_func(output, data['label'].float().to(device))
        loss.backward()
        optimizer.step()
        print('epoch [{}/{}]: completion: {:.1f} %   |   loss:{:.4f}'.format(epoch+1, MAX_EPOCH, i*BATCH_SIZE*100/total, loss.data), end="\r")

# example usage for classifier
input = torch.tensor([100,100])
input = input.to(device)
output = model(input.float())
print(output)

#######################################################
# Save Model
#######################################################
def save_checkpoint(state, is_best, checkpoint):
    filepath = os.path.join(checkpoint, 'stacked.pth')
    if not os.path.exists(checkpoint):
        print("Checkpoint Directory does not exist! Making directory {}".format(checkpoint))
        os.mkdir(checkpoint)
    else:
        print("Checkpoint Directory exists! ")
    torch.save(state, filepath)
    if is_best:
        shutil.copyfile(filepath, os.path.join(checkpoint, 'best.pth'))

def load_checkpoint(checkpoint, model, optimizer=None):
    if not os.path.exists(checkpoint):
        raise("File doesn't exist {}".format(checkpoint))
    checkpoint = torch.load(checkpoint)
    model.load_state_dict(checkpoint['state_dict'])

    if optimizer:
        optimizer.load_state_dict(checkpoint['optim_dict'])
    return checkpoint

if args.save:
    state = {'epoch': epoch + 1, 'state_dict': model.state_dict(), 'optim_dict' : optimizer.state_dict()}
    save_checkpoint(state, is_best=False, checkpoint="./models/")
