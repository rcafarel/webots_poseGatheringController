import numpy as np

from kinematics import RobotChain
from legs.Servo import positionTicksFromRadians
from matrix.Quaternion import Quaternion
from matrix.Translation import Translation


class SimpleLegForFindingPoses:

    def calculateFootPosition(self):
        eeH = self.inverseKinematicChain.forward_kinematics(self.currentJointAngles)
        self.footPosition = [eeH[0][3], eeH[1][3], eeH[2][3]]
        # print("Leg:", self.id, "Foot position:", eeH[0][3], eeH[1][3], eeH[2][3])


    def calculateFootPositionWRTRobotOrientation(self, q):
        Q = Quaternion(q)
        self.footPositionWRTRobotOrientation = np.matmul(Q.getR(), self.footPosition)
        if self.id == "frontRightLeg":
            T = Translation([self.x, self.y, 0]).matrix
            self.hipPositionWRTRobotOrientation = np.matmul(Q.getR(), [T[0][3], T[1][3], T[2][3]])


    def setServoPositionForCalculations(self, index, position):
        self.currentJointAngles[2 + 3 * index] = position


    def readServos(self):
        for i in range(self.numberOfServos):
            self.servoArray[i].readServo()
            self.currentJointAngles[5 + i*3] = self.servoArray[i].positionRadians()

    def getServoPositions(self):
        servoPositions = [0]*self.numberOfServos
        for i in range(self.numberOfServos):
            self.servoArray[i].readServo()
            servoPositions[i] = self.servoArray[i].positionRadians()
        return servoPositions

    def goToZ(self, z, t):  # maintain x and y position of foot
        self.goToPosition(self.footPosition[0], self.footPosition[1], z, t)

    def goToPosition(self, x, y, z, t):  # maintain x and y position of foot
        updatedJointAngles = self.getIKJointAngles(x, y, z, self.currentJointAngles)

        for i in range(self.numberOfServos):
            self.servoArray[i].goToTicksSyncronized(positionTicksFromRadians(updatedJointAngles[5 + i*3]), t)

        self.currentJointAngles = updatedJointAngles

    def getIKJointAngles(self, x, y, z, curJointAngles):
        targetFrame = [[1.0, 0.0, 0.0, x],
                       [0.0, 1.0, 0.0, y],
                       [0.0, 0.0, 1.0, z],
                       [0.0, 0.0, 0.0, 1.0]]

        return self.inverseKinematicChain.inverse_kinematics_frame(targetFrame, initial_position=curJointAngles,
                                                                   orientation_mode=None, no_position=False)

    def getModifiedZ(self, q):
        return np.matmul(Translation(self.footPosition).matrix, q.matrix)[2][3]

    def getChainHipToOrigin(self, z, ox, oy, oz):
        return RobotChain.getChainHipToOrigin(self.x, self.y, z, ox, oy, oz)

    def getChainOriginToHip(self, h, ox, oy, oz):
        return RobotChain.getChainOriginToHip(self.x, self.y, h, ox, oy, oz)



    def printServoPositions(self):
        self.readServos()
        for i in range(self.numberOfServos):
            print("Servo:", self.servoArray[i].id, "Ticks:", self.servoArray[i].positionTicks, "Radians:", self.servoArray[i].positionRadians())

