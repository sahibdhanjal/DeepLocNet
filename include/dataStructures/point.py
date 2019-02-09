class Point:
    def __init__(self, x, y, z = None, px = None, py = None, pz = None):
        self.x = x
        self.y = y
        self.z = z
        self.px = px
        self.py = py
        self.pz = pz

    def print(self):
        print("row: ", self.x, "col: ", self.y, "depth: ", self.z)
