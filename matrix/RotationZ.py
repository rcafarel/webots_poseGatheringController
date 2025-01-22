import numpy


class RotationZ:

    def __init__(self, theta):  # radians
        self.theta = theta
        self.matrix = []
        self.updateMatrix()

    def updateMatrix(self):
        self.matrix = [[numpy.cos(self.theta), -1.0 * numpy.sin(self.theta), 0.0, 0.0],
                       [numpy.sin(self.theta), numpy.cos(self.theta), 0.0, 0.0],
                       [0.0, 0.0, 1.0, 0.0],
                       [0.0, 0.0, 0.0, 1.0]]

    def setTheta(self, theta):
        self.theta = theta
        self.updateMatrix()
