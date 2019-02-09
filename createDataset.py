# basic includes
import sys
import argparse
from numpy.random import randint as ri
from pdb import set_trace as bp

from include.createMap import createMap
from include.rrtPlanner import calculatePath
from include.calculateDists import calculateDist

##############################################################
# Parse Arguments
##############################################################
parser = argparse.ArgumentParser(description='Radio-Inertial Localization')
parser.add_argument('--z', type=int, default=2, metavar='maxZ', help='maximum height of workspace (default: 2)')
parser.add_argument('--maps', type=int, default=10, metavar='map', help='number of times maps are to be selected (default: 10)')
parser.add_argument('--trials', type=int, default=10, metavar='trials', help='number of trials per selected map (default: 10)')
parser.add_argument('--step', type=int, default=3, metavar='step', help='step size for RRT (default: 3)')
args = parser.parse_args()

##############################################################
# Global Parameters
##############################################################
# All the Maps
inis        = ["defstr.ini", "office.ini", "11Dbibli.ini", "DLR.ini", "DLR2.ini", "Luebbers.ini", "TC2_METIS.ini", "TC1_METIS.ini", "W2PTIN.ini", "defdiff.ini", "defsthdiff.ini", "homeK_vf.ini", "klepal.ini", "testair0.ini"]
# Start Positions realtive to maps
starts      = [[20,20,0.2], [25,90,0.4], [5,5,0.8], [20,15,1.0], [20,15,1.2], [2,2,1.4], [80,50,1.8], [15,25,2.0], [80,20,1.7], [10,10,1.5], [5,5,1.1], [35,10,0.5], [5,40,0.3], [10,10,0.1]]
# Goal Positions realtive to maps
goals       = [[60,110,0.1], [18,15,0.3], [10,135,0.7], [20,55,0.5], [20,55,1.5], [50,50,1.3], [5,65,1.6], [65,125,1.8], [30,75,1.2], [24,53,1.4], [5,60,0.6], [25,45,0.2], [20,80,1.8], [28,52,0.8]]

##############################################################
# Pylayers map
##############################################################
for i in range(args.maps):
    mapID = ri(0,len(inis)) ; dim = 2 ; layout = inis[mapID] ; maxZ = args.z
    start = starts[mapID] ; goal  = goals[mapID] ; step  = args.step
    print("Map ", i+1,": ", inis[mapID])

    mat = createMap(layout, dim, maxZ)
    mat.cover()

    for j in range(args.trials):
        ##############################################################
        # Run RRT on Map
        ##############################################################
        RRT = calculatePath(start, goal, mat, step, dim, 2000)
        wayPts = RRT.RRTSearch()

        ##############################################################
        # Calculate RSSI Value, RSSI Distances and Actual Distances
        ##############################################################
        distMap = calculateDist(wayPts, mat, dim, True)
        dists = distMap.readDistances()