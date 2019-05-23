# basic includes
import sys
import argparse
import numpy as np
import matplotlib.pyplot as plt
from random import random, randint
from numpy.random import randint as ri
from pdb import set_trace as bp
import scipy.io as sio

from include.calculateDists import calculateDistExp
from include.localize import localizeExp

##############################################################
# Parse Arguments
##############################################################
parser = argparse.ArgumentParser(description='Radio-Inertial Localization')
parser.add_argument('--dim', type=int, default=2, metavar='dimension for localization (default: 2)')
parser.add_argument('--su', type=float, default=0.3, metavar='motion model noise (default: 0.2)')
parser.add_argument('--np', type=int, default=5000, metavar='number of particles for localization (default: 3000)')
parser.add_argument('--useClas', type=int, default=0, metavar='use classifier or not (default: 0)')
parser.add_argument('--hard', type=int, default=0, metavar='use hard or soft classification (default: 0 (soft))')
parser.add_argument('--savemat', type=int, default=0, metavar='save mat file for plotting or not (default : 0 (no))')

args = parser.parse_args()

##############################################################
# Pylayers map
##############################################################
# Parameters for RSSI Generation
map = calculateDistExp()

##############################################################
# Run Particle Filter / Fast SLAM v1 depending on choice
##############################################################
# Parameters for localization
numP        = args.np                                       # number of particles used
sigU        = [args.su, args.su, 0]                         # motion model noise - typically a fraction of the step size
sigZ        = ri(50,100, size=map.numAPs)                   # measure model noise
useClas     = args.useClas                                  # use of classifier or not
hardClas    = args.hard                                     # use soft vs hard classification

# Run the localization algorithms
localizer = localizeExp(numP, sigU, sigZ, map, useClas, hardClas)
localizer.FastSLAM()
print("The MSE in the localized path is:", localizer.MSE())
print("The point-wise error in localization is: ",localizer.getCDF())
print(localizer.path)

if args.savemat:
    dic = {}
    dic['mse'] = localizer.MSE()
    dic['cdf'] = localizer.getCDF()
    dic['waypts'] = localizer.wayPts
    dic['path'] = localizer.path
    dic['sigu'] = sigU
    dic['sigz'] = sigZ
    dic['TXName'] = localizer.TXName
    dic['TX'] = localizer.APLocs
    sio.savemat('trial.mat', dic)