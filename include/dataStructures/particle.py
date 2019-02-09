class Particle:
    def __init__(self, pose, weight):
        self.pose = pose
        self.w = weight
        self.mapMu = []
        self.mapID = []
        self.mapSigma = []
        self.hashMap = {}

    def print(self):
        print("pose: ", self.pose, " weight: ", self.w)

    def printMap(self):
        print("mu: ", self.mapMu)
        print("IDs: ", self.mapID)
        print("hash table: ", self.hashMap)
