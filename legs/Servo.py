import math

from matrix.RotationZ import RotationZ


class Servo:

    def __init__(self, id):
        self.id = id
        self.positionTicks = 500  # range: 0 - 1000
        self.rotationZ = RotationZ(0)

    def positionRadians(self):
        return positionDegrees(self.positionTicks) * math.pi / 180.0

    def goToTicks(self, ticks, timeForMove):
        self.positionTicks = ticks

    def goToTicksSyncronized(self, ticks, timeForMove):
        self.positionTicks = ticks

    def readServo(self):
        return self.positionTicks

def positionDegrees(positionTicks):
    return positionTicks / 1000.0 * 240.0 - 120.0

def positionTicks(positionDegrees):
    return positionDegrees * 1000.0 / 240.0 + 500.0

def positionTicksFromRadians(positionRadians):
    return positionTicks(positionRadians * 180.0 / math.pi)

def positionRadiansFromTicks(positionTicks):
    return positionDegrees(positionTicks) * math.pi / 180.0
