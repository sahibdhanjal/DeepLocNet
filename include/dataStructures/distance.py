class distanceMap:
    def __init__(self, rssi, euclid, label=None, name = None):
        self.rssi = rssi
        self.euclid = euclid
        self.label = label
        self.name = name

    def print(self):
        print("rssi: ", self.rssi, " , euclid: ", self.euclid, " , label: ", self.label)
