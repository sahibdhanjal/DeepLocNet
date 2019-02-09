import sys, argparse
import scipy.io as sci
import numpy as np
from autoEncoder import autoEncoder
from pdb import set_trace as bp

###########################################################
# Parse Arguments
###########################################################
parser = argparse.ArgumentParser(description='Train Classifier')
parser.add_argument('--epoch', type=int, default=3, metavar='N', help='number of epochs (default: 3), ie. 10**3')
parser.add_argument('--batch', type=int, default=8096, metavar='N', help='batch size (default: 8096)')
parser.add_argument('--ratio', type=float, default=0.8, metavar='N', help='train-test ration (default: 0.8)')
args = parser.parse_args()

###########################################################
# Load Data from .mat file
###########################################################
mat = sci.loadmat("../assets/data/trainLarge.mat")
euclid = mat['euclid'][:].tolist()
rssi = mat['rssi'][:].tolist()
labels = mat['labels'][:].tolist()

###########################################################
# Segregate Data to train/test
###########################################################
data = np.array([euclid, rssi]) ; nTotal = len(euclid) ; data = data.reshape(nTotal, 2)
nTrain = round(nTotal*args.ratio) ; nTest = nTotal - nTrain
xTrain = data[0:nTrain, :] ; xTest = data[nTrain+1:, :]
yTrain = np.array(labels[0:nTrain]) ; yTest = np.array(labels[nTrain+1:])

###########################################################
# Train and Save the Network
###########################################################
encoder = autoEncoder(xTrain, xTest, epochs = (10**args.epoch), batchSize = args.batch )
encoder.train(xTrain, xTest)
encoder.save()
