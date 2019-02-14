import torch, pdb
import pandas as pd
import numpy as np
from torch.utils.data import Dataset, DataLoader
from torch.utils.data.sampler import SubsetRandomSampler

class PylayersDataset(Dataset):
    def __init__(self, csv_path = "assets/data/train.csv", transform = None):
        # import csv file
        self.csv = pd.read_csv(csv_path)
        self.size = len(self.csv.ix[:,0])
        # one hot encode labels
        self.label = np.zeros((self.size, 2))
        self.label[np.arange(self.size), self.csv.ix[:,0]] = 1
        self.data = self.csv.ix[:,1:3].values
        self.rssival = self.csv.ix[:,3].values
        # convert data/labels to torch tensors
        self.data = torch.tensor(self.data.tolist(), dtype=torch.float64)
        self.label = torch.tensor(self.label.tolist(), dtype=torch.float64)
        # add transform if required
        self.transform = transform

    def __len__(self):
        return self.size

    def __getitem__(self, idx):
        return {'data':self.data[idx], 'label':self.label[idx]}

class loadData:
    def __init__(self, batch = 1024, val = 0.2, shuffle = True):
        self.batch_size = batch
        self.validation_split = val
        self.shuffle = shuffle
        self.dataset = PylayersDataset()
        self.size = len(self.dataset)
    
    def process(self):
        idxs = list(range(self.size))
        split = int(np.floor(self.validation_split * self.size))
        if self.shuffle: np.random.shuffle(idxs)
        train_idxs, val_idxs = idxs[split:], idxs[:split]
        
        train_sampler = SubsetRandomSampler(train_idxs)
        val_sampler = SubsetRandomSampler(val_idxs)

        train_loader = DataLoader(self.dataset, batch_size=self.batch_size, sampler=train_sampler, num_workers=4)
        valid_loader = DataLoader(self.dataset, batch_size=self.batch_size, sampler=val_sampler, num_workers=4)

        return len(train_idxs), train_loader, valid_loader