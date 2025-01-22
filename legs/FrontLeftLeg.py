import math

from matrix.Quaternion import Quaternion
from legs.SimpleLegForFindingPoses import SimpleLegForFindingPoses
from legs.Servo import Servo
from kinematics import RobotChain


class FrontLeftLeg(SimpleLegForFindingPoses):

    def __init__(self):
        self.id = "frontLeftLeg"
        self.xWalkingCentroid = -200.0
        self.yWalkingCentroid = 180.0

        self.numberOfServos = 3

        self.servoArray = [Servo(7), Servo(8), Servo(9)]
        self.currentJointAngles = [0]*13
        self.readServos()

        self.legSegmentMatrix = [[0.0, 27.0, 0.0], [0.0, 44.0, 0.0], [0.0, 75.0, 0.0]]
        self.lowerLegSegmentArray = [101.0, 91.0, 0.0]
        self.servoQMatrix = [Quaternion([1.0, 0.0, 0.0, 0.0]), Quaternion([0.7071, 0.0, -0.7071, 0.0]), Quaternion([0.0, 0.0, 1.0, 0.0])]
        self.servoOrientationMatrix = [self.servoQMatrix[0].getRPY(), self.servoQMatrix[1].getRPY(), self.servoQMatrix[2].getRPY()]
        self.inverseKinematicChain = RobotChain.createChain(-39.5, 100.0, math.pi / 4.0, self.legSegmentMatrix, self.servoOrientationMatrix, self.lowerLegSegmentArray)

        self.footPosition = []
        self.footPositionWRTRobotOrientation = []
        self.calculateFootPosition()

