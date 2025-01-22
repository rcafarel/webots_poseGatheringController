import math

# import transformations

import numpy

class Quaternion:

    def __init__(self, q, x=0, y=0, z=0):
        if q is not None:
            self.q = q
            self.matrix = [[(self.q[0] * self.q[0] + self.q[1] * self.q[1]) - (self.q[2] * self.q[2] + self.q[3] * self.q[3]),
                            2.0 * (self.q[1] * self.q[2] - self.q[0] * self.q[3]),
                            2.0 * (self.q[0] * self.q[2] + self.q[1] * self.q[3]), 0.0],
                           [2.0 * (self.q[1] * self.q[2] + self.q[0] * self.q[3]),
                            (self.q[0] * self.q[0] + self.q[2] * self.q[2]) - (self.q[1] * self.q[1] + self.q[3] * self.q[3]),
                            2.0 * (self.q[2] * self.q[3] - self.q[0] * self.q[1]), 0.0],
                           [2.0 * (self.q[1] * self.q[3] - self.q[0] * self.q[2]),
                            2.0 * (self.q[0] * self.q[1] + self.q[2] * self.q[3]),
                            (self.q[0] * self.q[0] + self.q[3] * self.q[3]) - (self.q[1] * self.q[1] + self.q[2] * self.q[2]), 0.0],
                           [0.0, 0.0, 0.0, 1.0]]
        else:
            d = math.sqrt(x*x + y*y + z*z)
            I3 = [[1.0, 0, 0], [0, 1, 0], [0, 0, 1]]
            k = [[0, -1.0*z, y], [z, 0, -1.0*x], [-1.0*y, x, 0]]
            R3 = I3 + numpy.dot(math.sin(d), k) + numpy.dot((1.0-math.cos(d)), numpy.matmul(k, k))
            self.matrix = [[R3[0][0], R3[0][1], R3[0][2], 0.0],
                           [R3[1][0], R3[1][1], R3[1][2], 0.0],
                           [R3[2][0], R3[2][1], R3[2][2], 0.0],
                           [0.0, 0.0, 0.0, 1.0]]
            
            # self.q = transformations.quaternion_from_matrix(self.matrix)

    def getQ0(self):
        return self.q[0]

    def getQ1(self):
        return self.q[1]

    def getQ2(self):
        return self.q[2]

    def getQ3(self):
        return self.q[3]

    def getYaw(self):
        return math.atan2(2.0*(self.q[2]*self.q[3] + self.q[0]*self.q[1]),
                          self.q[0]*self.q[0] - self.q[1]*self.q[1] - self.q[2]*self.q[2] + self.q[3]*self.q[3])

    def getPitch(self):
        return math.asin(-2.0*(self.q[1]*self.q[3] - self.q[0]*self.q[2]));

    def getRoll(self):
        return math.atan2(2.0*(self.q[1]*self.q[2] + self.q[0]*self.q[3]),
                          self.q[0]*self.q[0] + self.q[1]*self.q[1] - self.q[2]*self.q[2] - self.q[3]*self.q[3])
    
    def getRPY(self):
        return [self.getRoll(), self.getPitch(), self.getYaw()]

    def getR(self):
        return [[self.matrix[0][0], self.matrix[0][1], self.matrix[0][2]],
               [self.matrix[1][0], self.matrix[1][1], self.matrix[1][2]],
               [self.matrix[2][0], self.matrix[2][1], self.matrix[2][2]]]
