import math
from random import random, randint
from pdb import set_trace as bp
from include.dataStructures.point import Point
import numpy as np

class calculatePath:
    def __init__(self, start, goal, mat, step, dim = 2, maxIter = 1e4, viz = False):
        self.dim = dim
        self.start = start[0:2]
        self.goal = goal[0:2]
        self.map = mat.map
        self.ap = mat.numAPs
        self.step = step
        self.maxZ = mat.maxZ
        self.maxIter = int(maxIter)
        self.rows = mat.map.shape[0]
        self.cols = mat.map.shape[1]
        self.visited = np.zeros(mat.map.shape)
        self.goalFound = False
        self.hTresh = 200
        self.randH = 1
        self.viz = viz
        self.wayPts = []
        if self.dim==3: self.zdim = [start[2], goal[2]]

    def isValid(self, x):
        return (x[0]>=0 and x[0]<self.rows and x[1]>=0 and x[1]<self.cols and self.map[x[0]][x[1]]==1)

    def isGoal(self, x):
        return (x[0]==self.goal[0] and x[1]==self.goal[1])

    def angle(self, x, y):
        return math.atan2( (y[1]-x[1]) , (y[0]-x[0]) )

    def distance(self, x, y):
        if len(x)==3 and len(y)==3:
            return math.sqrt( (x[1]-y[1])**2 + (x[0]-y[0])**2 + (x[2]-y[2])**2 )
        else:
            return math.sqrt( (x[1]-y[1])**2 + (x[0]-y[0])**2 )

    def nearGoal(self, x):
        return self.distance(x, self.goal) <=self.step

    def nearestToGoal(self):
        minDist = 999999999999

        for i in self.wayPts:
            dist = self.distance(self.goal, [i.x, i.y])
            if dist<minDist:
                minDist = dist
                point = [i.x, i.y]

        return point

    def findClosest(self, point):
        minDist = 99999999999999
        closest = [0, 0]

        for row in range(self.rows):
            for col in range(self.cols):
                if self.visited[row][col]!=0:
                    dist = self.distance( point, [row,col])
                    if dist<minDist:
                        minDist = dist
                        closest = [row, col]
        return closest, minDist

    def inPath(self, curr, boundary = 10):
        nbrsX = [1, 1, -1, -1]
        nbrsY = [1, -1, 1, -1]
        
        for i in range(0, boundary):
            for j in range(4):
                x = curr[0] + i*nbrsX[j]
                y = curr[1] + i*nbrsY[j]

                if x>=0 and x<self.rows and y>=0 and y<self.cols:
                    if self.visited[x][y] > 0:
                        return True
        return False

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

    def bresenham3D(self, x, y):
        # https://www.geeksforgeeks.org/bresenhams-algorithm-for-3-d-line-drawing/
        x1, y1, z1 = x
        x2, y2, z2 = y
        points = []
        points.append((x1, y1, z1))
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        dz = abs(z2 - z1)
        if (x2 > x1):
            xs = 1
        else:
            xs = -1
        if (y2 > y1):
            ys = 1
        else:
            ys = -1
        if (z2 > z1):
            zs = 1
        else:
            zs = -1

        if (dx >= dy and dx >= dz):
            p1 = 2 * dy - dx
            p2 = 2 * dz - dx
            while (x1 != x2):
                x1 += xs
                if (p1 >= 0):
                    y1 += ys
                    p1 -= 2 * dx
                if (p2 >= 0):
                    z1 += zs
                    p2 -= 2 * dx
                p1 += 2 * dy
                p2 += 2 * dz
                points.append((x1, y1, z1))

        elif (dy >= dx and dy >= dz):
            p1 = 2 * dx - dy
            p2 = 2 * dz - dy
            while (y1 != y2):
                y1 += ys
                if (p1 >= 0):
                    x1 += xs
                    p1 -= 2 * dy
                if (p2 >= 0):
                    z1 += zs
                    p2 -= 2 * dy
                p1 += 2 * dx
                p2 += 2 * dz
                points.append((x1, y1, z1))

        else:
            p1 = 2 * dy - dz
            p2 = 2 * dx - dz
            while (z1 != z2):
                z1 += zs
                if (p1 >= 0):
                    y1 += ys
                    p1 -= 2 * dz
                if (p2 >= 0):
                    x1 += xs
                    p2 -= 2 * dz
                p1 += 2 * dy
                p2 += 2 * dx
                points.append((x1, y1, z1))
        return points

    def obstruction(self, x, y):
        if len(x)==3: x, y = x[:2], y[:2]
        points = self.bresenham2D(x, y)
        for r,c in points:
            if self.map[r][c]==0:
                return True
        return False

    def findParent(self, point):
        for i in self.wayPts:
            if i.x==point[0] and i.y==point[1]:
                return [i.px, i.py]

    def getPath(self):
        path = []
        if self.wayPts[-1].x == self.goal[0] and self.wayPts[-1].y == self.goal[1]:
            last = self.goal
        else:
            last = self.nearestToGoal()
        path.append(last)

        while last[0]!=None and last[1]!=None:
            parent = self.findParent(last)
            if parent[0]!=None and parent[1]!=None:
                path.append(parent)
            last = parent
        path.reverse()

        if self.dim==3:
            path[0].append(self.zdim[0])
            for i in range(1, len(path)): path[i].append(max(0,self.zdim[0] + random()*abs(self.zdim[1] - self.zdim[0])))
        
        return path

    def RRTSearch(self):
        goalBias = 100
        ctr = 1
        self.visited[self.start[0]][self.start[1]] = ctr
        pt = Point(self.start[0], self.start[1])
        self.wayPts.append(pt)

        print("Starting RRT Search..")

        # check if start and goal is valid and only then proceed
        if self.isValid(self.start) and self.isValid(self.goal):
            # iterate till max iterations
            for it in range(self.maxIter):

                if self.goalFound:
                    print("Goal Found! Constructing Path....")
                    break

                print("Number of Iterations: {:4}".format(it+1), end="\r")

                if self.randH == 2:
                    point = [ (self.goal[0] + goalBias - 2*goalBias*random()) , (self.goal[1] + goalBias - 2*goalBias*random())]

                else:
                    point = [self.rows*random(), self.cols*random()]

                curr,_ = self.findClosest(point)
                ang = self.angle(curr, point)
                temp = [ (curr[0] + round(self.step*math.cos(ang))) , (curr[1] + round(self.step*math.sin(ang))) ]
                
                if self.isValid(temp) and not self.inPath(temp) and not self.obstruction(temp, curr):
                    ctr += 1
                    # add point to waypoint array
                    pt = Point(temp[0], temp[1], None, curr[0], curr[1], None)
                    self.wayPts.append(pt)

                    if self.isGoal(temp) or self.nearGoal(temp):
                        self.goalFound = 1
                        pt = Point(self.goal[0], self.goal[1], None, temp[0], temp[1])
                        self.wayPts.append(pt)

                    curr = temp
                    self.visited[curr[0]][curr[1]] = ctr

                    if self.distance(curr, self.goal) < self.hTresh:
                        self.randH = 2
        else:
            raise ValueError("Invalid Start or Goal point! Reinitialize")

        return self.getPath()
