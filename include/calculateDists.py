import math, csv
from pdb import set_trace as bp
import numpy as np
from numpy.random import normal as normrnd
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
        self.LNL = [0, 0]       # LOS, NLOS
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
        return val

    def printLNL(self):
        total = self.LNL[0] + self.LNL[1]
        return "LOS: " + str(self.LNL[0]/total*100) + "%     |       NLOS: " + str(self.LNL[1]/total*100) + "%"

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
                
                # LOS/ NLOS Labels in Path
                if label==0 : self.LNL[0] = self.LNL[0] + 1
                if label==1 : self.LNL[1] = self.LNL[1] + 1
                
                if math.isinf(rssiVal) or math.isinf(rssiDist): continue
                APMap.append(distanceMap(rssiDist, euclid, label))
                if self.csv: fW.writerow([label, rssiDist, euclid, rssiVal])

            distMap.append(APMap)

        print("All distances mapped on grid!")
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
                                
                # LOS/ NLOS Labels in Path
                if label==0 : self.LNL[0] = self.LNL[0] + 1
                if label==1 : self.LNL[1] = self.LNL[1] + 1
                
                APMap.append(distanceMap(rssiDist, euclid, label))
                if self.csv: fW.writerow([label, rssiDist, euclid, rssiVal])

            distMap.append(APMap)
        print("All distances mapped on grid!")
        return distMap


class calculateDistExp:
    def __init__(self, waypts = 'assets/data/Trials/Trial1/waypts.csv', wifi = 'assets/data/Trials/Trial1/Wifi.csv'):
        self.wFile = waypts
        self.dim = 2
        self.TX = wifi
        self.maxZ = 2.4

        self.TXName = None; self.numPts = None ; self.numAPs = None
        self.name2MAC = None ; self.MAC2Name = None ; self.name2Pos = None
        self.distMap = [] ; self.wayPts = None

        self.defineAPS()
        self.parseWaypts()
        self.readDistances()

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
        return val

    def defineAPS(self):
        # 45 APs defined
        self.TXName = [ '106', '10M', '111', '114-H', '115', '116', '117', '120', '121', '124', '125', '125-H', '129', '130', 
                        '131', '134-E', '134-W', '135', '137', '138-E', '138-W', '139', '203', '203-H', '204', '208', '209', 
                        '210', '211', '212', '213', '214', '216', '219', '220', '221', '222', '229', '232', '233', '235', 
                        '236', '242', '244-E', '244-W' ]

        self.numAPs = len(self.TXName)

        self.name2MAC = {'106'   : '2c:33:11:89:6d:c2' , '10M'   : '2c:33:11:68:49:92' , '111'   : '2c:33:11:89:68:82' , '114-H' : '2c:33:11:3c:53:12' , '115'   : '2c:33:11:9e:06:32' , 
                        '116'   : '2c:33:11:3c:53:62' , '117'   : '2c:33:11:5b:a1:92' , '120'   : 'e8:65:49:c2:d4:62' , '121'   : '2c:33:11:5b:a5:e2' , '124'   : '2c:33:11:89:42:f2' ,
                        '125'   : '2c:33:11:3c:55:12' , '125-H' : '2c:33:11:89:6a:52' , '129'   : '38:90:a5:19:87:e2' , '130'   : '2c:33:11:68:48:82' , '131'   : '2c:33:11:9e:0a:32' ,
                        '134-E' : '2c:33:11:9e:07:f2' , '134-W' : '2c:33:11:9e:0d:82' , '135'   : '2c:33:11:5b:a6:e2' , '137'   : '2c:33:11:9e:05:42' , '138-E' : '2c:33:11:9e:0a:92' ,
                        '138-W' : '2c:33:11:5b:a5:32' , '139'   : '2c:33:11:89:48:22' , '203'   : '2c:33:11:3c:52:a2' , '203-H' : '2c:33:11:5b:95:52' , '204'   : '2c:33:11:89:69:72' ,
                        '208'   : '2c:33:11:49:49:c2' , '209'   : '2c:33:11:89:63:72' , '210'   : '2c:33:11:89:6e:12' , '211'   : '2c:33:11:89:61:62' , '212'   : '2c:33:11:5b:9e:82' ,
                        '213'   : '2c:33:11:5b:9d:82' , '214'   : '2c:33:11:68:2e:32' , '216'   : '2c:33:11:68:43:52' , '219'   : '2c:33:11:9e:03:12' , '220'   : '2c:33:11:5b:a4:52' ,
                        '221'   : '2c:33:11:21:b6:22' , '222'   : '2c:33:11:9e:02:d2' , '229'   : '2c:33:11:5b:a3:72' , '232'   : '2c:33:11:68:2e:62' , '233'   : '2c:33:11:68:43:b2' ,
                        '235'   : '2c:33:11:89:63:a2' , '236'   : '2c:33:11:3c:40:a2' , '242'   : '2c:33:11:89:54:12' , '244-E' : '2c:33:11:89:69:22' , '244-W' : '2c:33:11:5b:a4:42' }

        self.MAC2Name = {'2c:33:11:89:6d:c2' : '106' ,  '2c:33:11:68:49:92' : '10M' ,  '2c:33:11:89:68:82' : '111' ,  '2c:33:11:3c:53:12' : '114-H' ,  '2c:33:11:9e:06:32' : '115' ,
                        '2c:33:11:3c:53:62' : '116' ,  '2c:33:11:5b:a1:92' : '117' ,  'e8:65:49:c2:d4:62' : '120' ,  '2c:33:11:5b:a5:e2' : '121' ,  '2c:33:11:89:42:f2' : '124' ,
                        '2c:33:11:3c:55:12' : '125' ,  '2c:33:11:89:6a:52' : '125-H' ,  '38:90:a5:19:87:e2' : '129' ,  '2c:33:11:68:48:82' : '130' ,  '2c:33:11:9e:0a:32' : '131' ,
                        '2c:33:11:9e:07:f2' : '134-E' ,  '2c:33:11:9e:0d:82' : '134-W' ,  '2c:33:11:5b:a6:e2' : '135' ,  '2c:33:11:9e:05:42' : '137' ,  '2c:33:11:9e:0a:92' : '138-E' ,
                        '2c:33:11:5b:a5:32' : '138-W' ,  '2c:33:11:89:48:22' : '139' ,  '2c:33:11:3c:52:a2' : '203' ,  '2c:33:11:5b:95:52' : '203-H' ,  '2c:33:11:89:69:72' : '204' ,
                        '2c:33:11:49:49:c2' : '208' ,  '2c:33:11:89:63:72' : '209' ,  '2c:33:11:89:6e:12' : '210' ,  '2c:33:11:89:61:62' : '211' ,  '2c:33:11:5b:9e:82' : '212' ,
                        '2c:33:11:5b:9d:82' : '213' ,  '2c:33:11:68:2e:32' : '214' ,  '2c:33:11:68:43:52' : '216' ,  '2c:33:11:9e:03:12' : '219' ,  '2c:33:11:5b:a4:52' : '220' ,
                        '2c:33:11:21:b6:22' : '221' ,  '2c:33:11:9e:02:d2' : '222' ,  '2c:33:11:5b:a3:72' : '229' ,  '2c:33:11:68:2e:62' : '232' ,  '2c:33:11:68:43:b2' : '233' ,
                        '2c:33:11:89:63:a2' : '235' ,  '2c:33:11:3c:40:a2' : '236' ,  '2c:33:11:89:54:12' : '242' ,  '2c:33:11:89:69:22' : '244-E' ,  '2c:33:11:5b:a4:42' : '244-W' }                       

        self.name2Pos = { '106'   : [ 1.2, 3.2, -0.6] , '10M'   : [ 0.8, 4.2, -0.6] , '111'   : [ 8, -4, -0.6] , '114-H' : [ 8, 4, -0.6] , '115'   : [ 17.8, -1.2, -0.6] , 
                        '116'   : [ 17.8, 1.8, -0.6] , '117'   : [ 24, -1.8, -0.6] , '120'   : [ 27, 1.8, -0.6] , '121'   : [ 30, -1.2, -0.6] , '124'   : [ 34, 1, -0.6] , 
                        '125'   : [ 38, -2.8, -0.6] , '125-H' : [ 40, 0, -0.6] , '129'   : [ 44, -2.8, -0.6] , '130'   : [ 43, 2, -0.6] , '131'   : [ 50, -0.8, -0.6] , 
                        '134-E' : [ 58, 4, -0.6] , '134-W' : [ 53, 2.4, -0.6] , '135'   : [ 56, -1, -0.6] , '137'   : [ 62, -0.8, -0.6] , '138-E' : [ 72.4, 3.2, -0.6] , 
                        '138-W' : [ 64, 1.8, -0.6] , '139'   : [ 68, -1.5, -0.6] , '203'   : [3.8, -1.5, 2.4] , '203-H' : [0, 0, 2.4] , '204'   : [0.8, 4.7, 2.4] , 
                        '208'   : [4.2, 3.2, 2.4] , '209'   : [8, -4, 2.4] , '210'   : [7.6, 4.7, 2.4] , '211'   : [15, -1.5, 2.4] , '212'   : [11, 1, 2.4] , 
                        '213'   : [18, -1, 2.4] , '214'   : [14.4, 3.2, 2.4] , '216'   : [17.8, 4.7, 2.4] , '219'   : [28, -1.5, 2.4] , '220'   : [24.6, 4, 2.4] , 
                        '221'   : [34, -1.5, 2.4] , '222'   : [28, 1.8, 2.4] , '229'   : [40.4, -1.5, 2.4] , '232'   : [44.4, 2.8, 2.4] , '233'   : [48.4, -2.5, 2.4] , 
                        '235'   : [54.4, -2.5, 2.4] , '236'   : [50.4, 5.2, 2.4] , '242'   : [62.4, 2.4, 2.4] , '244-E' : [72.4, 4.8, 2.4] , '244-W' : [66.4, -2.5, 2.4] }

    def parseWaypts(self):
        wpt_file = open(self.wFile, 'r')
        reader = csv.reader(wpt_file)
        waypts = []
        for row in reader:
            x = float(row[0]) ; y = float(row[1])
            waypts.append([x,y,1])
        self.numPts = len(waypts)
        self.wayPts = waypts

    def readDistances(self):
        wifi_file = open(self.TX, 'r')
        reader = csv.reader(wifi_file)
        measures = []
        
        for row in reader:
            rowint = [float(x) for x in row]
            measures.append(rowint)
        
        for i in range(self.numPts):
            pt = self.wayPts[i]
            APMap = []
            for j in range(self.numAPs):
                name = self.TXName[j]
                tx = self.name2Pos[name]
                rssiVal = measures[i][j]
                label = 1

                if name=='203-H' or name=='125-H': label = 0               
                euclid = self.distance(pt, tx)

                if rssiVal == 0: rssiDist = normrnd(20,3)
                else: rssiDist = self.rssi2Dist(rssiVal)
                
                APMap.append(distanceMap(rssiDist, euclid, label, name))

            self.distMap.append(APMap)

        print("All distances mapped on grid!")