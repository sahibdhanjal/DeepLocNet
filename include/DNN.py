import torch
import torch.nn as nn
import torch.optim

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
            nn.Linear(100, 200),
            nn.ReLU(),
            nn.Linear(200, 1000),
            nn.ReLU(),
            nn.Linear(1000, 200),
            nn.ReLU(),
            nn.Linear(200, 100),
            nn.ReLU(),
            nn.Linear(100, 50),
            nn.ReLU(),
            nn.Linear(50, 20),
            nn.ReLU(),
            nn.Linear(20, 10),
            nn.ReLU(),
            nn.Linear(10, 2),
            nn.ReLU(),
            nn.Softmax(dim=-1),
        )

    def forward(self, x):
        res = self.autoenc(x)
        return res


class DNN2(nn.Module):
    def __init__(self):
        super(DNN2, self).__init__()

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
            nn.ReLU(),
            nn.Softmax(dim=-1),
        )

    def forward(self, x):
        res = self.autoenc(x)
        return res


class DNN3(nn.Module):
    def __init__(self):
        super(DNN3, self).__init__()

        self.autoenc = nn.Sequential(
            nn.Linear(2, 10),
            nn.ReLU(),
            nn.Linear(10, 20),
            nn.ReLU(),
            nn.Linear(20, 50),
            nn.ReLU(),
            nn.Linear(50, 100),
            nn.ReLU(),
            nn.Linear(100, 2),
            nn.ReLU(),
            nn.Softmax(dim=-1),
        )

    def forward(self, x):
        res = self.autoenc(x)
        return res