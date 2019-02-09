import math
import numpy as np
from numpy.random import normal as normrnd
from scipy.stats import multivariate_normal, norm
from scipy.linalg import sqrtm, expm
from pdb import set_trace as bp
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from include.dataStructures.particle import Particle


class localize:
    def __init__(self, numP, su, sz, distMap, mat, wayPts, R, start, dim, useClas, hardClas):
        self.np = numP
        self.sz = sz
        self.dists = distMap
        self.dim = dim
        self.wayPts = wayPts
        self.pts = self.convert(wayPts)
        self.nAP = mat.numAPs
        self.tx = mat.Tx
        self.R = R
        self.start = start
        self.su = su
        self.path = []
        self.APLocs = []
        self.IDs = []
        self.use = useClas
        self.hard = hardClas
        if self.dim == 2: self.su = su[0:2]
        if self.dim == 2: self.start = start[0:2]

    def print(self, samples):
        for i in range(self.np):
            print("pose: ", samples[i].pose, " | weight: ", samples[i].w)

    def distance(self, x, y):
        if len(x)==3 and len(y)==3:
            return math.sqrt( (x[1]-y[1])**2 + (x[0]-y[0])**2 + (x[2]-y[2])**2 )
        else:
            return math.sqrt( (x[1]-y[1])**2 + (x[0]-y[0])**2 )

    def MSE(self):
        mse = 0
        for i in range(len(self.pts)):
            mse += self.distance(self.wayPts[i], self.path[i])
        return mse

    def distrib(self):
        samples = []
        if self.dim == 2: start = [self.start[0], self.start[1]]
        if self.dim == 3: start = self.start
        for _ in range(self.np):
            samples.append(Particle(start, 1/self.np))
        return samples

    def convert(self, pts):
        n = len(pts)
        rtPts = []
        for i in range(1, n):
            dx = pts[i][0] - pts[i-1][0]
            dy = pts[i][1] - pts[i-1][1]
            if self.dim==2: rtPts.append([dx, dy])
            if self.dim==3: dz = pts[i][2] - pts[i-1][2] ; rtPts.append([dx, dy, dz])
        return rtPts
 
    '''
    weighting using the normpdf subroutine
    '''
    def getWeight(self, dz):
        norpdf = 1
        for i in range(len(dz)):
            if dz[i]!=0:
                norpdf *= norm.pdf(dz[i], 0, self.sz[i])
        return norpdf
        
    '''
    weighting using the mvnpdf subroutine
    '''
    def getMultiWeight(self, dz):
        idx = [i for i, e in enumerate(dz) if e != 0]
        val = [] ; sig = []
        if len(idx)==0:
            return 1/self.np
        for i in idx:
            val.append(dz[i])
            sig.append(self.sz[i])
        
        mvn = multivariate_normal([0]*len(idx), np.diag(sig))
        return mvn.pdf(val)

    '''
    return is not required as python works on
    pass-by-reference and there is no way of
    emulating pass-by-value
    '''
    def motion_model(self, samples, point, su):
        for i in range(self.np):
            dx = point[0] - normrnd(0, su[0])
            dy = point[1] - normrnd(0, su[1])
            if self.dim == 2: pose = [samples[i].pose[0] + dx, samples[i].pose[1] + dy]
            if self.dim == 3: dz = point[2] - normrnd(0, su[2]) ; pose = [samples[i].pose[0] + dx, samples[i].pose[1] + dy, samples[i].pose[2] + dz]
            samples[i].pose = pose

    '''
    measurement model for the particle filter
    label for dMap = 1 : NLOS   ,   0 : LOS
    '''
    def measure_model(self, samples, z):
        totalWt = 0 ; nAP = len(z)
        for i in range(self.np):
            dz = [0 for x in range(nAP)]
            for j in range(nAP):
                tx = self.tx[j] ; pos = samples[i].pose
                d = self.distance(tx, pos)
                if d <= self.R:
                    dz[j] = abs(z[j].rssi-d)
                    '''
                    if self.use:
                        label = self.classify(z[j].rssi, d)
                        if label==0:
                            if self.hard:
                                dz[j] = abs(z[j].rssi-d)
                            else:
                                dz[j] = abs(z[j].rssi-d)

                    else:
                        dz[j] = abs(z[j].rssi-d)
                    '''

            wt = self.getWeight(dz)
            samples[i].w *= wt
            totalWt += wt
        
        if totalWt!=0:
            for i in range(self.np):
                samples[i].w = samples[i].w / totalWt
        else:
            for i in range(self.np):
                samples[i].w = 1/self.np

    '''
    measurement model for fast slam v1
    label for dMap = 1 : NLOS   ,   0 : LOS
    '''
    def fast_measure_model(self, samples, z):
        if self.dim == 2: Qt = np.diag([10,10])
        if self.dim == 3: Qt = np.diag([10,10,10])
        Qt = Qt.tolist() ; nAP = len(z) ;  totWt = 0

        for i in range(self.np):
            for j in range(nAP):
                tx = np.array(self.tx[j]) ; pos = np.array(samples[i].pose)
                d = self.distance(tx, pos)
                if d <= self.R:
                    # initialize particle map
                    if j not in samples[i].mapID:
                        samples[i].mapMu.append(tx)
                        samples[i].mapSigma.append(Qt)
                        samples[i].mapID.append(j)
                        samples[i].hashMap[j] = len(samples[i].mapID) - 1
                        samples[i].w = 1/self.np

                    # update particle map
                    else:
                        ID = samples[i].hashMap[j]

                        # prediction step
                        muHat = samples[i].mapMu[ID]
                        sigHat = np.array(samples[i].mapSigma[ID])

                        # update step
                        dHat = self.distance(pos, muHat)
                        innov = abs(z[j].rssi-dHat)
                        dx = muHat[0] - pos[0] ; dy = muHat[1] - pos[1]
                        den = math.sqrt(dx**2 + dy**2)
                        H = np.array([dx/den, dy/den])
                        
                        if self.dim==3:
                            dz = muHat[2] - pos[2]
                            den = math.sqrt(dx**2 + dy**2 + dz**2)
                            H = np.array([dx/den, dy/den, dz/den])

                        try:
                            Q = np.matmul(np.matmul(H, sigHat), H) + self.sz[j]
                        except:
                            bp()

                        # Kalman Gain
                        K = np.matmul(sigHat, H)/Q

                        # update pose/ covar
                        mu = muHat + innov*K
                        K = K.reshape((self.dim,1))
                        sig = (np.identity(self.dim) - K*H)*sigHat
                        samples[i].mapMu[ID] = mu.reshape((self.dim,))
                        samples[i].mapSigma[ID] = sig.tolist()
                        samples[i].w = max(samples[i].w, math.sqrt(2*math.pi*Q)*math.exp(-0.5*(innov**2)/Q))
                        totWt += samples[i].w

        # normalize the weights
        if totWt==0:
            for i in range(self.np):
                samples[i].w = 1/self.np
        else:
            for i in range(self.np):
                samples[i].w = samples[i].w/totWt

    '''
    resampling algorithm applicable to both
    particle filter and fast slam because of
    common structure of particle
    '''
    def resample(self, samples):
        idx = [0]*self.np ; Q = [0]*self.np ; Q[0] = samples[0].w

        for i in range(1, self.np):
            Q[i] = samples[i].w + Q[i-1]

        t = np.random.rand(self.np+1, 1)
        T = np.sort(t, axis=0)
        T[self.np] = 1 ; i,j = 0,0

        while i<self.np and j<self.np:
            if T[i] < Q[j]:
                idx[i] = j
                i += 1
            else:
                j += 1

        if len(set(idx))>0.2*self.np:
            for i in range(self.np):
                samples[i].pose = samples[idx[i]].pose
                samples[i].w = 1/self.np
                samples[i].mapMu = samples[idx[i]].mapMu
                samples[i].mapID = samples[idx[i]].mapID
                samples[i].mapSigma = samples[idx[i]].mapSigma
                samples[i].hashMap = samples[idx[i]].hashMap

    '''
    Calculates the effective number of particles
    in the sampled distribution.
    '''
    def neff(self, samples):
        wghts = [0]*self.np ; totWt = 0
        for i in range(self.np):
            wghts[i] = samples[i].w
            totWt += samples[i].w
        den = 0
        for i in range(self.np):
            wghts[i] = (wghts[i]/totWt)**2
            den += wghts[i]
        return 1/den


    '''
    Calculates weighted mean and variance of the
    sample distribution
    '''
    def meanVar(self, samples):
        totWt = 0 ; mu = [0 for _ in range(self.dim)] ; sig = np.zeros((self.dim,self.dim))
        for i in range(self.np):
            mu[0] += samples[i].pose[0]
            mu[1] += samples[i].pose[1]
            if self.dim==3: mu[2] += samples[i].pose[2]
            totWt += samples[i].w

        if self.dim==2: mu = [mu[0]/self.np, mu[1]/self.np]
        if self.dim==3: mu = [mu[0]/self.np, mu[1]/self.np, mu[2]/self.np]

        for i in range(self.np):
            if self.dim==2: x = np.array([ samples[i].pose[0]-mu[0] , samples[i].pose[1]-mu[1] ])
            if self.dim==3: x = np.array([ samples[i].pose[0]-mu[0] , samples[i].pose[1]-mu[1] , samples[i].pose[2]-mu[2] ])

            sig += np.matmul(x.reshape((self.dim,1)),x.reshape((1,self.dim)))
        sig = sig/self.np
        return mu, sig

    '''
    Calculates weighted mean and variance of the
    sample distribution
    '''
    def weightedMeanVar(self, samples):
        totWt = 0 ; mu = [0 for _ in range(self.dim)] ; sig = np.zeros((self.dim,self.dim))
        for i in range(self.np):
            mu[0] += samples[i].w*samples[i].pose[0]
            mu[1] += samples[i].w*samples[i].pose[1]
            if self.dim==3: mu[2] += samples[i].w*samples[i].pose[2]
            totWt += samples[i].w

        if self.dim==2: mu = [mu[0]/totWt, mu[1]/totWt]
        if self.dim==3: mu = [mu[0]/totWt, mu[1]/totWt, mu[2]/totWt]

        for i in range(self.np):
            if self.dim==2: x = np.array([ samples[i].pose[0]-mu[0] , samples[i].pose[1]-mu[1] ])
            if self.dim==3: x = np.array([ samples[i].pose[0]-mu[0] , samples[i].pose[1]-mu[1] , samples[i].pose[2]-mu[2] ])

            sig += samples[i].w*np.matmul(x.reshape((self.dim,1)),x.reshape((1,self.dim)))
        sig = sig/totWt
        return mu, sig

    '''
    Get the maximum weighted particle and use it
    to calculate the IDs of the APs discovered &
    the locations of the discovered APs
    '''
    def getAPLocs(self, samples):
        maxWeight = -9999999999 ; idx = 0

        for i in range(self.np):
            if samples[i].w > maxWeight:
                maxWeight = samples[i].w
                idx = i

        self.APLocs = samples[idx].mapMu
        self.IDs = samples[idx].mapID

    '''
    Plot the particle poses for each particle. Can
    only be used for debugging as of now as animation
    support is yet to be added
    '''
    def plot(self, samples):
        x = [] ; y = []
        for i in range(self.np):
            x.append(samples[i].pose[0])
            y.append(samples[i].pose[1])
        plt.plot(x,y,'c.')
        
        mXY,_ = self.meanVar(samples)
        wmXY,_ = self.weightedMeanVar(samples)

        plt.plot(mXY[0],mXY[1],'ro')
        plt.plot(wmXY[0],wmXY[1],'bo')

        plt.xlim([-100,300])
        plt.ylim([-100,300])
        plt.show()

    '''
    The main Particle filter class
    '''
    def particleFilter(self):
        self.path.append(self.start)
        samples = self.distrib()
        for i in range(len(self.pts)):
            # provide action update
            self.motion_model(samples, self.pts[i], self.su)
            # provide measurement update
            self.measure_model(samples, self.dists[i])
            # resample only when number of effective particle drops
            if self.neff(samples) <= 1/3*self.np:
                self.resample(samples)
            
            mXY, _ = self.weightedMeanVar(samples)
            self.path.append(mXY)

        print("Particle Filter has finished running....")

    '''
    The main Fast SLAM v1 class
    '''
    def FastSLAM(self):
        self.path.append(self.start)
        samples = self.distrib()

        for i in range(len(self.pts)):
            # provide action update
            self.motion_model(samples, self.pts[i], self.su)
            # provide measurement update
            self.fast_measure_model(samples, self.dists[i])
            # resample only when number of effective particle drops
            if self.neff(samples) <= 1/3*self.np:
                self.resample(samples)

            mXY, _ = self.weightedMeanVar(samples)
            self.path.append(mXY)

        self.getAPLocs(samples)

        print("FastSLAM has finished running....")
