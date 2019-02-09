class distanceMap:
    def __init__(self, rssi, euclid, label=None):
        self.rssi = rssi
        self.euclid = euclid
        self.label = label

    def print(self):
        print("rssi: ", self.rssi, " , euclid: ", self.euclid, " , label: ", self.label)
