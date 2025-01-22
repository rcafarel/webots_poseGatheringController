import math

from matrix.Quaternion import Quaternion
from legs.SimpleLegForFindingPoses import SimpleLegForFindingPoses
from legs.Servo import Servo
from kinematics import RobotChain


class MiddleLeftLeg(SimpleLegForFindingPoses):

    def __init__(self):
        self.id = "middleLeftLeg"
        
        self.x = -64.5
        self.y = 0.0
        self.theta = math.pi / 2.0
        
        self.xWalkingCentroid = -250.0
        self.yWalkingCentroid = 0.0

        self.numberOfServos = 3

        self.servoArray = [Servo(4), Servo(5), Servo(6)]
        self.currentJointAngles = [0]*13
        self.readServos()

        self.legSegmentMatrix = [[0.0, 27.0, 0.0], [0.0, 44.0, 0.0], [0.0, 75.0, 0.0]]
        self.lowerLegSegmentArray = [101.0, 91.0, 0.0]
        self.servoQMatrix = [Quaternion([1.0, 0.0, 0.0, 0.0]), Quaternion([0.7071, 0.0, -0.7071, 0.0]), Quaternion([0.0, 0.0, 1.0, 0.0])]
        self.servoOrientationMatrix = [self.servoQMatrix[0].getRPY(), self.servoQMatrix[1].getRPY(), self.servoQMatrix[2].getRPY()]
        self.inverseKinematicChain = RobotChain.createChain(self.x, self.y, self.theta, self.legSegmentMatrix, self.servoOrientationMatrix, self.lowerLegSegmentArray)

        self.footPosition = []
        self.footPositionWRTRobotOrientation = []
        self.calculateFootPosition()

    def getModifiedChainFromOrientation(self, ox, oy, oz):
        return RobotChain.createChainWithOrientation(self.x, self.y, self.theta, self.legSegmentMatrix, self.servoOrientationMatrix, self.lowerLegSegmentArray, ox, oy, oz)

