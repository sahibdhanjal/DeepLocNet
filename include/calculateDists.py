import math, csv
from pdb import set_trace as bp
import numpy as np
from include.dataStructures.distance import distanceMap

class calculateDist:
    def __init__(self, wayPts, mat, dim, savecsv = False):
        self.wayPts = wayPts
        self.C = mat
        self.dim = dim
        self.Smap = mat.StrengthMap
        self.map = mat.map
        self.Tx = mat.Tx
        self.unit = mat.pathUnit
        self.numPts = len(wayPts)
        self.numAPs = mat.numAPs
        self.res = mat.resolution
        self.maxZ = mat.maxZ
        self.heights = []
        self.csv = savecsv
        h = 0
        while h<=self.maxZ:
            self.heights.append(h)
            h += self.res

    def bresenham2D(self, x, y):
        x1, y1 = x ; x2, y2 = y
        dx = x2 - x1 ; dy = y2 - y1
        # Determine how steep the line is
        is_steep = abs(dy) > abs(dx)
        # Rotate line
        if is_steep:
            x1, y1 = y1, x1
            x2, y2 = y2, x2

        # Swap start and end points if necessary and store swap state
        swapped = False
        if x1 > x2:
            x1, x2 = x2, x1
            y1, y2 = y2, y1
            swapped = True

        # Recalculate differentials
        dx = x2 - x1
        dy = y2 - y1

        # Calculate error
        error = int(dx / 2.0)
        ystep = 1 if y1 < y2 else -1

        # Iterate over bounding box generating points between start and end
        y = y1
        points = []
        for x in range(x1, x2 + 1):
            coord = (y, x) if is_steep else (x, y)
            points.append(coord)
            error -= abs(dy)
            if error < 0:
                y += ystep
                error += dx
        # Reverse the list if the coordinates were swapped
        if swapped:
            points.reverse()
        return points

    def getIndex(self, x):
        idx = 0 ; diff = 9999999999
        for i in range(len(self.heights)):
            d = abs(x - self.heights[i])
            if d<diff:
                diff = d
                idx = i
        return idx

    def obstruction(self, x, y):
        x = [x[0], x[1]] ; y = [y[0], y[1]]
        points = self.bresenham2D(x, y)
        for r,c in points:
            if self.map[r][c]==0:
                return 1
        return 0

    def distance(self, x, y):
        if len(x)==3 and len(y)==3:
            return math.sqrt( (x[1]-y[1])**2 + (x[0]-y[0])**2 + (x[2]-y[2])**2 )
        else:
            return math.sqrt( (x[1]-y[1])**2 + (x[0]-y[0])**2 )

    def rssi2Dist(self, rssi):
        '''
        https://stackoverflow.com/questions/11217674/how-to-calculate-distance-from-wifi-router-using-signal-strength
        http://pylayers.github.io/pylayers/notebook/2-AP/CoverageMetis.html
        '''
        if abs(rssi) > 60: exp = (abs(rssi) - 32.44)/20
        else : exp = (abs(rssi) - 12.55)/20
        val = (10**exp) / 60
        val = val if val<1e6 else 1e6
        return val

    def expConvert(self, dist):
        a = 76.95 ; b = 3.803e-41
        c = -50.55 ; d = -5.097e-40
        val = a*np.exp(b*dist) + c*np.exp(d*dist)
        return val

    def readDistances(self):
        if self.dim==3:
            raise ValueError("Use readDistances3D() instead of readDistances()")

        distMap = []
        
        if self.csv:
            f = open('data.csv', mode='a')
            fW = csv.writer(f, delimiter=',',quotechar='"', quoting=csv.QUOTE_MINIMAL)

        for i in range(self.numPts):
            pt = list(map(int,self.wayPts[i][:]))

            APMap = []
            for j in range(self.numAPs):
                tx = list(map(int,self.Tx[j][:]))
                label = self.obstruction(pt, tx)                                # 1 = NLOS, 0 = LOS
                euclid = self.distance(pt, tx)
                rssiVal = self.Smap[pt[0]][pt[1]][j]
                rssiDist = self.rssi2Dist(rssiVal)/self.unit
                APMap.append(distanceMap(rssiDist, euclid, label))
                if self.csv: fW.writerow([label, rssiDist, euclid, rssiVal])

            distMap.append(APMap)

        print("Distances mapped on Grid!")
        return distMap

    def readDistances3D(self):
        if self.dim==2:
            raise ValueError("Use readDistances() instead of readDistances3D()")

        distMap = []

        if self.csv:
            f = open('data.csv', mode='a')
            fW = csv.writer(f, delimiter=',',quotechar='"', quoting=csv.QUOTE_MINIMAL)

        for i in range(self.numPts):
            pt = list(map(float,self.wayPts[i][:]))
            idx = self.getIndex(pt[2])
            pt = list(map(int,pt))
            APMap = []

            for j in range(self.numAPs):
                tx = list(map(int,self.Tx[j][:]))
                label = self.obstruction(pt, tx)        # 1 = NLOS, 0 = LOS
                euclid = self.distance(pt, tx)
                rssiVal = self.Smap[idx][pt[0]][pt[1]][j]
                rssiDist = self.rssi2Dist(rssiVal)/self.unit
                APMap.append(distanceMap(rssiDist, euclid, label))
                if self.csv: fW.writerow([label, rssiDist, euclid, rssiVal])

            distMap.append(APMap)
        print("Distances mapped on Grid!")
        return distMap