import time

from legs.BackLeftLeg import BackLeftLeg
from legs.BackRightLeg import BackRightLeg
from legs.FrontLeftLeg import FrontLeftLeg
from legs.FrontRightLeg import FrontRightLeg
from legs.MiddleLeftLeg import MiddleLeftLeg
from legs.MiddleRightLeg import MiddleRightLeg


class SimpleRobotForFindingPoses:

    def __init__(self):
        # print("Initialize left legs")
        self.backLeftLeg = BackLeftLeg()
        self.middleLeftLeg = MiddleLeftLeg()
        self.frontLeftLeg = FrontLeftLeg()

        # print("Initialize right legs")
        self.backRightLeg = BackRightLeg()
        self.middleRightLeg = MiddleRightLeg()
        self.frontRightLeg = FrontRightLeg()


    def updateRobotOrientationAndEEs(self, orientation):
        self.calculateIndependentFootPositions()
        self.calculateFootPositionsWRTRobotOrientation(orientation)

    def calculateIndependentFootPositions(self):
        self.backLeftLeg.calculateFootPosition()
        self.middleLeftLeg.calculateFootPosition()
        self.frontLeftLeg.calculateFootPosition()
        self.backRightLeg.calculateFootPosition()
        self.middleRightLeg.calculateFootPosition()
        self.frontRightLeg.calculateFootPosition()

    def calculateFootPositionsWRTRobotOrientation(self, orientation):
        self.backLeftLeg.calculateFootPositionWRTRobotOrientation(orientation)
        self.middleLeftLeg.calculateFootPositionWRTRobotOrientation(orientation)
        self.frontLeftLeg.calculateFootPositionWRTRobotOrientation(orientation)
        self.backRightLeg.calculateFootPositionWRTRobotOrientation(orientation)
        self.middleRightLeg.calculateFootPositionWRTRobotOrientation(orientation)
        self.frontRightLeg.calculateFootPositionWRTRobotOrientation(orientation)

    def getLegs(self):
        return [self.backLeftLeg, self.middleLeftLeg, self.frontLeftLeg, self.backRightLeg, self.middleRightLeg, self.frontRightLeg]

    def getFirstTripodLegs(self):
        return [self.backLeftLeg, self.frontLeftLeg, self.middleRightLeg]

    def getSecondTripodLegs(self):
        return [self.middleLeftLeg, self.backRightLeg, self.frontRightLeg]

    def printServoPositions(self):
        self.backLeftLeg.printServoPositions()
        self.middleLeftLeg.printServoPositions()
        self.frontLeftLeg.printServoPositions()
        self.backRightLeg.printServoPositions()
        self.middleRightLeg.printServoPositions()
        self.frontRightLeg.printServoPositions()