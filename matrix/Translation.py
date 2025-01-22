
class Translation:

    def __init__(self, d):
        self.d = d
        self.matrix = [[1.0, 0.0, 0.0, self.d[0]],
                       [0.0, 1.0, 0.0, self.d[1]],
                       [0.0, 0.0, 1.0, self.d[2]],
                       [0.0, 0.0, 0.0, 1.0]]

    def getX(self):
        return self.d[0]

    def getY(self):
        return self.d[1]

    def getZ(self):
        return self.d[2]
