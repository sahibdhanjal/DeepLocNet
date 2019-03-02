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
    def __init__(self):
        self.wayPts = None
        self.dim = 2
        
        self.TXName = None
        self.numPts = None
        self.numAPs = None
        self.maxZ = 2.4
        self.TXName = None
        self.name2MAC = None
        self.name2Pos = None
        self.measure = None
        self.distMap = []

        self.parseAll()

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

    def expConvert(self, dist):
        a = 76.95 ; b = 3.803e-41
        c = -50.55 ; d = -5.097e-40
        val = a*np.exp(b*dist) + c*np.exp(d*dist)
        return val

    def parseAPs(self):
        wp1 = {'203-H':-10, '203':-51, '204':-75, '208':-75, '212':-80, '233':-81, '219':-83 ,'213':-83, '222':-84, '232':-85}
        wp2 = {'203-H':-14, '203':-51, '204':-75, '208':-75, '212':-80, '233':-81, '219':-83, '213':-83, '222':-84, '232':-85}
        wp3 = {'203-H':-14, '203':-52, '204':-75, '208':-75, '212':-80, '233':-81, '219':-83, '213':-83, '222':-84, '232':-85}
        wp4 = {'203-H':-18, '203':-52, '204':-64, '208':-75, '212':-80, '233':-81, '219':-83, '213':-83, '222':-84, '232':-85}
        wp5 = {'203-H':-20, '203':-51, '204':-64, '208':-75, '212':-72, '233':-78, '219':-83, '213':-83, '222':-84, '232':-85}
        wp6 = {'203-H':-40, '203':-51, '204':-50, '208':-75, '212':-65, '233':-80, '219':-83, '213':-83, '222':-84, '232':-85}
        wp7 = {'203-H':-42, '203':-62, '204':-64, '208':-75, '212':-80, '233':-80, '219':-83, '213':-85, '222':-84, '232':-85, '209':-74, '211':-65, '214':-56, '210':-48 , '216':-82 }
        wp8 = {'203-H':-64, '203':-62, '204':-64, '208':-75, '212':-80, '233':-80, '219':-83, '213':-85, '222':-84, '232':-85, '209':-74, '211':-65, '214':-56, '210':-48 , '216':-82 }
        wp9 = {'203-H':-80, '203':-64, '204':-75, '208':-75, '212':-80, '233':-80, '219':-83, '213':-85, '222':-84, '232':-85, '209':-74, '211':-65, '214':-56, '210':-48 , '216':-82 }
        wp10 = {'203-H':-80, '203':-64, '204':-80, '208':-75, '212':-80, '233':-80, '219':-83, '213':-85, '222':-84, '232':-85, '209':-74, '211':-65, '214':-56, '210':-48 , '216':-82 }
        wp11 = {'203-H':-80, '203':-68, '204':-80, '208':-75, '212':-80, '233':-80, '219':-83, '213':-85, '222':-84, '232':-85, '209':-74, '211':-65, '214':-56, '210':-48 , '216':-82 }
        wp12 = {'203-H':-85, '203':-80, '204':-75, '208':-75, '212':-80, '233':-80, '219':-83, '213':-85, '222':-84, '232':-85, '209':-74, '211':-65, '214':-56, '210':-48 , '216':-82 }
        wp13 = {'204':-75, '208':-75, '212':-80, '233':-80, '219':-83, '213':-85, '222':-84, '232':-85, '209':-74, '211':-65, '214':-56, '210':-48 , '216':-82 }
        wp14 = {'203':-80, '204':-75, '208':-75, '212':-80, '233':-80, '219':-83, '213':-85, '222':-84, '232':-85, '209':-74, '211':-65, '214':-56, '210':-48 , '216':-82 }
        wp15 = {'203':-80, '204':-75, '208':-75, '212':-80, '233':-80, '219':-83, '213':-85, '222':-84, '232':-85, '209':-74, '211':-65, '214':-56, '210':-48 , '216':-82 }
        wp16 = {'203':-72, '204':-75, '208':-75, '212':-80, '233':-80, '219':-83, '213':-85, '222':-84, '232':-85, '209':-74, '211':-65, '214':-56, '210':-48 , '216':-82 }
        wp17 = {'203-H':-80, '203':-61, '204':-75, '208':-75, '212':-80, '233':-80, '219':-83, '213':-85, '222':-84, '232':-85, '209':-74, '211':-65, '214':-56, '210':-48 , '216':-82 }
        wp18 = {'203-H':-64, '203':-55, '204':-75, '208':-75, '212':-80, '233':-80, '219':-83, '213':-85, '222':-84, '232':-85, '209':-74, '211':-65, '214':-56, '210':-48 , '216':-82 }
        wp19 = {'203-H':-42, '203':-51, '204':-75, '208':-75, '212':-80, '233':-80, '219':-83, '213':-85, '222':-84, '232':-85, '209':-74, '211':-65, '214':-56, '210':-48 , '216':-82 }
        wp20 = {'203-H':-20, '203':-51, '204':-75, '208':-75, '212':-80, '233':-80, '219':-83, '213':-85, '222':-84, '232':-85, '209':-74, '211':-65, '214':-56, '210':-48 , '216':-82 }
        wp21 = {'203-H':-14, '203':-51, '204':-75, '208':-75, '212':-80, '233':-80, '219':-83, '213':-85, '222':-84, '232':-85, '209':-74, '211':-65, '214':-56, '210':-48 , '216':-82 }
        wp22 = {'203-H':-14, '203':-51, '204':-75, '208':-75, '212':-80, '233':-80, '219':-83, '213':-85, '222':-84, '232':-85, '209':-74, '211':-65, '214':-56, '210':-48 , '216':-82 }
        self.measure = [wp1, wp2, wp3, wp4, wp5, wp6, wp7, wp8, wp9, wp10, wp11, wp12,wp13, wp14, wp15, wp16, wp17, wp18, wp19, wp20, wp21, wp22]

    def parseMap(self):
        self.TXName = ['203-H', '204', '203', '208', '210', '209', '212', '214', '211', '216', '213', '220', '222', '219', '221', '229', '232', '233', '236', '235', '242']
        self.name2MAC = {'203-H':'2c:33:11:5b:95:52',
                        '204':'2c:33:11:89:69:72',
                        '203':'2c:33:11:3c:52:a2',
                        '208':'2c:33:11:49:49:c2',
                        '210':'2c:33:11:89:6e:12',
                        '209':'2c:33:11:89:63:72',
                        '213':'2c:33:11:5b:9d:82',
                        '214':'2c:33:11:68:2e:32',
                        '211':'2c:33:11:89:61:62',
                        '216':'2c:33:11:68:43:52',
                        '212':'2c:33:11:5b:9e:82',
                        '220':'2c:33:11:5b:a4:52',
                        '222':'2c:33:11:9e:02:d2',
                        '219':'2c:33:11:9e:03:12',
                        '221':'2c:33:11:21:b6:22',
                        '229':'2c:33:11:5b:a3:72',
                        '232':'2c:33:11:68:2e:62',
                        '233':'2c:33:11:68:43:b2',
                        '236':'2c:33:11:3c:40:a2',
                        '235':'2c:33:11:89:63:a2',
                        '242':'2c:33:11:89:54:12'}
        self.numAPs = len(self.TXName)
        self.name2Pos = {'203-H':[0, 0, self.maxZ],
                        '204':[0.8, 4.7, self.maxZ],
                        '203':[3.8, -1.5, self.maxZ],
                        '208':[4.2, 3.2, self.maxZ],
                        '210':[7.6, 4.7, self.maxZ],
                        '209':[8.0, -4.0, self.maxZ],
                        '212':[11.0, 1.0, self.maxZ],
                        '214':[14.4, 3.2, self.maxZ],
                        '211':[15.0, -1.5, self.maxZ],
                        '216':[17.8, 4.7, self.maxZ],
                        '213':[18.0, -1.0, self.maxZ],
                        '220':[24.6, 4.0, self.maxZ],
                        '222':[28.0, 1.8, self.maxZ],
                        '219':[28.0, -1.5, self.maxZ],
                        '221':[35.0, -1.5, self.maxZ],
                        '229':[42.4, -1.5, self.maxZ],
                        '232':[46.4, 2.8, self.maxZ],
                        '233':[54.4, -2.5, self.maxZ],
                        '236':[56.4, 5.2, self.maxZ],
                        '235':[62.4, -2.5, self.maxZ],
                        '242':[70.4, 1.5, self.maxZ]}

    def parseWayPts(self):
        csvFile = open('assets/data/Experiment/waypts-2rot.csv')
        reader = csv.reader(csvFile)
        wayPts = []
        for row in reader:
            x = float(row[0]); y = float(row[1])
            wayPts.append([x,y])
        self.numPts = len(wayPts)
        self.wayPts = wayPts
    
    def parseAll(self):
        self.parseAPs()
        self.parseMap()
        self.parseWayPts()
        # self.readDistances()

    def readDistances(self):
        for i in range(self.numPts):
            pt = self.wayPts[i]
            APMap = []
            for j in range(len(self.name2Pos)):
                name = self.TXName[j]
                tx = self.name2Pos[name]
                if name in self.measure[j]: 
                    if name=='203-H': label = 0
                    else : label = 1
                    euclid = self.distance(pt, tx)
                    rssiVal = self.measure[j][name]
                    rssiDist = self.rssi2Dist(rssiVal)
    
                    # LOS/ NLOS Labels in Path
                    # if label==0 : self.LNL[0] = self.LNL[0] + 1
                    # if label==1 : self.LNL[1] = self.LNL[1] + 1
                    if math.isinf(rssiVal) or math.isinf(rssiDist): continue
                    APMap.append(distanceMap(rssiDist, euclid, label, name))
            self.distMap.append(APMap)
        print("All distances mapped on grid!")