import sys, h5py
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from pdb import set_trace as bp

class readMat:
    def __init__(self, file):
        self.mat = h5py.File(file, 'r')
        self.StrengthMap = self.mat["StrengthMap"][:]
        self.Tx = self.mat["Tx"][:]
        self.Tx = self.Tx.T.tolist()
        self.map = self.mat["map"][:]
        self.pathUnit = float(self.mat["pathUnit"][:])
        self.numAPs = int(self.mat["numAPs"][:])
        self.freq = float(self.mat["freq"][:])

    def visualize(self, wayPts=None, Tx=None, path=None, TX=None, ID=None):
        print("Displaying Floor Plan.")
        plt.imshow(self.map, cmap="gray")

        # display the waypoints by RRT
        if wayPts!=None:
            rows = []; cols = []
            for x,y in wayPts:
                rows.append(x); cols.append(y)
            plt.plot(cols, rows, 'bo-')

        # display the actual AP locations
        if Tx!=None:
            rows = []; cols = []
            ctr = 1
            for x,y in Tx:
                rows.append(int(x)); cols.append(int(y))
                plt.text(y,x,"  AP "+str(ctr), color='black')
                ctr += 1

            plt.plot(cols, rows, 'kx')

        # display the localized path
        if path!=None:
            rows = []; cols = []
            for x,y in path:
                rows.append(x); cols.append(y)
            plt.plot(cols, rows, 'go-')

        # display the estimated AP locations
        if TX!=None and ID!=None:
            rows = []; cols = []
            ctr = 1
            for x,y in TX:
                rows.append(int(x)); cols.append(int(y))
                plt.text(y,x,"  AP "+str(ID[ctr-1]+1), color='red')
                ctr += 1

            plt.plot(cols, rows, 'rx')

        plt.show()

