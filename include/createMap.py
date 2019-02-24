import numpy as np
from pylayers.gis.layout import Layout
from pylayers.antprop.coverage import *
from pylayers.simul.link import *
from mayavi import mlab
from pdb import set_trace as bp
from math import sqrt, floor, ceil
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class parseMap:
    def __init__(self, L, rows, cols):
        self.map = np.ones((rows,cols), dtype=int).T
        self.unit = 0
        self.factor = None
        self.min = None
        self.parse(L, rows, cols)

    def parse(self, L, rows, cols):
        G = L.Gs
        minX, maxX, minY, maxY = L.ax
        dx = maxX-minX; dy = maxY-minY
        factX = rows/dx ; factY = cols/dy
        all = np.array(G.nodes())
        
        self.unit = sqrt((dx**2 + dy**2)/(rows**2 + cols**2))
        self.factor = [factX, factY]
        self.min = [minX, minY]

        # nodes = [ x for x in all if  x<0 ]
        edges = [ x for x in all if  x>0 ]
        # print("grid distance:",rows, cols, " |   map distance: ",dx, dy, " |   factors :",factX, factY, " |   origin shift by: ", minX, minY)

        for i in edges:
            if 'AIR' in G.node[i]['name'] or 'WOOD' in G.node[i]['name'] or 'PILLLAR' in G.node[i]['name'] or 'GLASS' in G.node[i]['name'] or 'WINDOW' in G.node[i]['name']:
                continue
            v1, v2 = G.node[i]['connect']
            v1x, v1y = G.pos[v1]
            v2x, v2y = G.pos[v2]
            
            # shift origin
            v1x -= minX ; v1y -= minY
            v2x -= minX ; v2y -= minY
            
            # transform factor
            v1x *= factX ; v1y *= factY
            v2x *= factX ; v2y *= factY
            
            if v1x==0 or v2x==0 or v1y ==0 or v2y == 0:
                continue

            # check cases when vertices are the upper boundaries of the image
            if v1x==rows or v2x==rows or v1y == cols or v2y == cols:
                continue

            if abs(v1x-v2x)<=1:
                rmin = int(min(v1y, v2y))
                rmax = int(max(v1y, v2y))
                x = int(v1x)

                for i in range(rmin, rmax+1):
                    self.map[i][x] = 0

            if abs(v1y-v2y)<=1:
                rmin = int(min(v1x, v2x))
                rmax = int(max(v1x, v2x))
                y = int(v1y)

                for i in range(rmin, rmax+1):
                    self.map[y][i] = 0

class createMap:
    def __init__(self, name = 'test.ini', dim = 2, maxZ = 2):
        self.name = name                                # name of file
        self.map = None                                 # 2D grid map with 0 as wall, 1 as free cell
        self.StrengthMap = None                         # map with nAP depth of dBM
        self.Tx = []                                    # 3D position of APs
        self.pathUnit = 0                               # pixel/m
        self.numAPs = 0                                 # number of APs
        self.C = Coverage(self.name)                    # coverage object
        self.maxZ = maxZ                                # max height of blueprint
        self.dim = dim                                  # dimension for localization
        self.range = [self.C.nx, self.C.ny]             # range of layout (xmin, xmax, ymin, ymax)
        self.resolution = 0.5                           # distances at which each map is created
        self.factor = None                              # X conversion of map to grid coordinates
        self.min = None                                 # minX factor to be added to AP Loc
        self.DL = DLink(L=self.C.L)                     # DLink for 3D plotting
        self.getMap()
        self.parseAPs()

    '''
    Parses number and locations of APs
    '''
    def parseAPs(self):
        self.numAPs = len(self.C.dap)
        for i in range(self.numAPs):
            y,x,z = self.C.dap[i]['p']
            x -= self.min[0] ; y -= self.min[1]
            x *= self.factor[0] ; y *= self.factor[1]
            x = int(x) ; y = int(y)
            if self.dim==2 : self.Tx.append((x,y))
            else : self.Tx.append((x,y,z))

    '''
    Parses the map and other relevant parameters
    through the parser class
    '''
    def getMap(self):
        parser = parseMap(self.C.L, self.C.nx, self.C.ny)
        self.map = parser.map
        self.pathUnit = parser.unit
        self.min = parser.min
        self.factor = parser.factor

    '''
    Refer coverage.py - lines 843 onwards for details

    set ap to -1 to visualize all access points
    otherwise set it to the AP index - [0, nAP]
    '''
    def cover(self):
        if self.dim == 3:
            raise TypeError("Use cover3D() instead of cover()")

        self.C.zgrid = self.maxZ
        self.C.cover()
        V = self.C.CmWp[0,:,:]
        U = self.reshapeMap(V)
        self.StrengthMap = 10*np.log10(U)
        
    
    '''
    Refer coverage.py - lines 843 onwards for details

    set ap to -1 to visualize all access points
    otherwise set it to the AP index - [0, nAP]
    '''
    def cover3D(self):
        if self.dim == 2:
            raise TypeError("Use cover() instead of cover3D()")

        h = 0
        height = []
        while h<=self.maxZ:
            height.append(h)
            h += self.resolution
        
        self.StrengthMap = []
        for h in height:
            self.C.zgrid = h
            self.C.cover()
            V = self.C.CmWp[0,:,:]
            U = self.reshapeMap(V)
            self.StrengthMap.append(10*np.log10(U))
            print('Strength Calculated at level: {:.1f}'.format(h), end="\r")
        self.StrengthMap = np.array(self.StrengthMap)
        print()


    '''
    Reshapes from (x*y x 1 x nAP) to (x x y x nAP)
    '''
    def reshapeMap(self, M):
        _, nAP = M.shape
        x, y = self.C.nx, self.C.ny
        sMap = np.zeros((y,x,nAP))
        
        for i in range(nAP):
            sMap[:,:,i] = M[:,i].reshape((x,y)).T
        return sMap

    '''
    print AP locations
    '''
    def printAPLocs(self):
        for i in range(self.numAPs):
            print("AP ",i,": [",self.Tx[i][0],", ", self.Tx[i][1],", ", self.Tx[i][2], "]")

    '''
    Helper function to visualize all waypoints, access points,
    the ground truth and localized path and the localized APs
    '''
    def visualize(self, start=None, goal=None, wayPts=None, path=None, TX=None, ID=None):
        if self.dim == 3: raise ValueError("Use visualize3D() instead of visualize() for 3 dimensions")
        print("Displaying Floor Plan.")
        plt.imshow(self.map, cmap="gray")
        Tx = self.Tx

        # display the waypoints by RRT
        if wayPts!=None:
            rows = []; cols = []
            for x,y in wayPts:
                rows.append(x); cols.append(y)
            plt.plot(cols, rows, 'b.-')

        # display the actual AP locations
        if Tx!=None:
            rows = []; cols = []
            ctr = 1
            for i in Tx:
                rows.append(int(i[0])); cols.append(int(i[1]))
                plt.text(i[1],i[0],"  AP "+str(ctr), color='black')
                ctr += 1

            plt.plot(cols, rows, 'kx')

        # display the localized path
        if path!=None:
            rows = []; cols = []
            for i in path:
                rows.append(i[0]); cols.append(i[1])
            plt.plot(cols, rows, 'c.-')

        # display the estimated AP locations
        if TX!=None and ID!=None:
            rows = []; cols = []
            ctr = 1
            for i in TX:
                rows.append(int(i[0])); cols.append(int(i[1]))
                plt.text(i[1],i[0],"  AP "+str(ID[ctr-1]+1), color='red')
                ctr += 1

            plt.plot(cols, rows, 'rx')

        if start : plt.plot(start[1], start[0], 'gs', markersize=6)
        if goal : plt.plot(goal[1], goal[0], 'rs', markersize=6)

        plt.show()

    def visualize3D(self, start=None, goal=None, wayPts=None, path=None, TX=None, ID=None):
        if self.dim == 2: raise ValueError("Use visualize() instead of visualize3D() for 2 dimensions")
        print("Displaying Floor Plan.")
        
        [factX, factY] = self.factor
        [minX, minY] = self.min
        Tx = self.Tx
        
        # display the actual AP locations
        x = []; y = [] ; z = []
        for i in Tx: x.append(i[0]/factX + minX); y.append(i[1]/factY + minY) ; z.append(i[2])
        mlab.points3d(y, x, z, scale_factor=0.2, color=(1.0, 0.0, 0.0))

        # display the estimated AP locations
        if TX!=None and ID!=None:
            if len(TX)==0: 
                print("No AP detected")
                pass
            else:
                x = []; y = [] ; z = []
                for i in Tx: x.append(i[0]/factX + minX); y.append(i[1]/factY + minY) ; z.append(i[2])
                mlab.points3d(y, x, z, scale_factor=0.2, color=(0.0, 1.0, 0.0))
        
        # display the waypoints by RRT
        if wayPts!=None:
            x = []; y = [] ; z = []
            for i in wayPts: x.append(i[0]/factX + minX); y.append(i[1]/factY + minY) ; z.append(i[2])
            mlab.plot3d(y, x, z, tube_radius=0.025, color=(1.0,1.0,1.0))

        # display the localized path
        if path!=None:
            x = []; y = [] ; z = []
            for i in path: x.append(i[0]/factX + minX); y.append(i[1]/factY + minY) ; z.append(i[2])
            mlab.plot3d(y, x, z, tube_radius=0.025, color=(0.0,0.0,0.0))

        self.DL._show3(ant=False)

    '''
    visualize only the strength map for one or all
    APs. For combined strength map, use a=-1, else
    use the ID of one of the APs
    '''
    def visualizeSMap(self, ap=-1):
        self.C.show(a=ap, figsize=(16,9))
        plt.show()


