import math
import time

import numpy as np

from legs.Servo import positionRadiansFromTicks
from matrix.MatrixManip import matrix2quaternion, getArray, getAxisOfRotation, getMatrix4
from matrix.Quaternion import Quaternion


class PoseDefinition:

    def __init__(self, supervisor, controllerRobot, imu):
        self.controllerRobot = controllerRobot
        self.imu = imu
        self.poseCounter = 0

        self.supervisor = supervisor

        self.robot_node = self.supervisor.getFromDef("robot")
        self.ee_frontRightLeg = self.supervisor.getFromDef("ee_frontRightLeg")
        self.ee_middleLeftLeg = self.supervisor.getFromDef("ee_middleLeftLeg")
        self.ee_backRightLeg = self.supervisor.getFromDef("ee_backRightLeg")

        self.r_frontRightLeg = self.supervisor.getFromDef("r_frontRightLeg")

        self.rInitO_inverse = None

        self.frPos = []
        self.rPos = []
        self.frH_Pos = []
        self.frH_O = []
        self.imu_O = []
        self.slopeIMU = []
        self.p_imuO = None

        self.output_h = []
        self.output_q0 = []
        self.output_q1 = []
        self.output_q2 = []
        self.output_q3 = []
        self.output_s1 = []
        self.output_s2 = []
        self.output_s3 = []
        self.output_eex = []
        self.output_eey = []
        self.output_eez = []

        self.output_eex_actual = []
        self.output_eey_actual = []
        self.output_eez_actual = []

        self.pRoll = None
        self.pPitch = None

        self.useRollAndPitch = False

        self.inputArray = []
        self.outputArray = []

        self.p_fr = None
        self.p_ml = None
        self.p_br = None

    def initializeRobotOrientation(self):
        # read the initial orientation of the robot to make subsequent readings indifferent of initial orientation
        imuq = self.imu.getQuaternion()
        self.rInitO_inverse = np.transpose(Quaternion([imuq[3], imuq[0], imuq[1], imuq[2]]).getR())
        if self.useRollAndPitch:
            self.rInitO_inverse = np.transpose(np.eye(3))
        # print("IMU initial O:", self.rInitO_inverse)

    def readPosition(self, subCommand):
        # this is the true orientation of the robot which is recorded prior to calculating the axis of rotation for each pose
        # this is used to calculate the forward kinematics of the robot
        self.imu_O = self.imu.getQuaternion()
        imuRPY = self.imu.getRollPitchYaw()

        # on the physical system frH_Pos will be calculated
        self.frH_Pos = self.r_frontRightLeg.getCenterOfMass()
        # frH_O is calculated to remove the affect of the initial orientation of the robot, which should be level
        # print(Quaternion([self.imu_O[3], self.imu_O[0], self.imu_O[1], self.imu_O[2]]).getR())
        # print(self.rInitO_inverse)
        self.frH_O = np.matmul(Quaternion([self.imu_O[3], self.imu_O[0], self.imu_O[1], self.imu_O[2]]).getR(), self.rInitO_inverse)

        if self.useRollAndPitch:
            q = [math.cos(imuRPY[0]/2)*math.cos(imuRPY[1]/2),
                 math.sin(imuRPY[0]/2)*math.cos(imuRPY[1]/2),
                 math.cos(imuRPY[0]/2)*math.sin(imuRPY[1]/2),
                 math.sin(imuRPY[0]/2)*math.sin(imuRPY[1]/2)]
            self.frH_O = np.matmul(Quaternion(q).getR(), self.rInitO_inverse)

        # this reading is only for the simulation, it is used to validate the end effector calculations against actual position
        self.frPos = self.ee_frontRightLeg.getCenterOfMass()
        self.rPos = self.robot_node.getCenterOfMass()

        props = subCommand[1].split(",")
        s1 = positionRadiansFromTicks(int(props[0].split(":")[1].strip()))
        s2 = positionRadiansFromTicks(int(props[1].split(":")[1].strip()))
        s3 = positionRadiansFromTicks(int(props[2].split(":")[1].strip()))

        self.p_fr = self.getPositionFromRobotNode(self.ee_frontRightLeg)
        self.p_ml = self.getPositionFromRobotNode(self.ee_middleLeftLeg)
        self.p_br = self.getPositionFromRobotNode(self.ee_backRightLeg)
        print("Pose FR", self.p_fr[0], self.p_fr[1], self.p_fr[2], s1, s2, s3)
        print("Pose ML", self.p_ml[0], self.p_ml[1], self.p_ml[2], s1, s2, s3)
        print("Pose BR", self.p_br[0], self.p_br[1], self.p_br[2], s1, s2, s3)

    def readOrientation(self):
        imuq = self.imu.getQuaternion()
        imuRPY = self.imu.getRollPitchYaw()
        imuO = Quaternion([imuq[3], imuq[0], imuq[1], imuq[2]]).getR()

        if self.useRollAndPitch:
            q = [math.cos(imuRPY[0]/2)*math.cos(imuRPY[1]/2),
                 math.sin(imuRPY[0]/2)*math.cos(imuRPY[1]/2),
                 math.cos(imuRPY[0]/2)*math.sin(imuRPY[1]/2),
                 math.sin(imuRPY[0]/2)*math.sin(imuRPY[1]/2)]
            imuO = Quaternion(q).getR()

        if self.p_imuO is not None:
            imu_R = np.matmul(np.transpose(self.p_imuO), imuO)
            iR = getAxisOfRotation(imu_R)
            slope_i = iR[1]/iR[0]

            if self.useRollAndPitch:
                slope_i = (self.pPitch - imuRPY[1]) / (self.pRoll - imuRPY[0])

            self.slopeIMU.append(slope_i)
            # print("IMU slope: ", slope_i)
        self.p_imuO = imuO
        self.pRoll = imuRPY[0]
        self.pPitch = imuRPY[1]

    def calculatePoseAndInitializeNextPose(self, subCommand):
        # before resetting variables for the next pose, calculate and record pose details
        # print("Pose:", self.poseCounter, subCommand[1])
        self.poseCounter += 1

        props = subCommand[1].split(",")
        s1 = positionRadiansFromTicks(int(props[2].split(":")[1].strip()))
        s2 = positionRadiansFromTicks(int(props[3].split(":")[1].strip()))
        s3 = positionRadiansFromTicks(int(props[4].split(":")[1].strip()))
        self.output_s1.append(s1)
        self.output_s2.append(s2)
        self.output_s3.append(s3)

        imuR = np.matmul(Quaternion([self.imu_O[3], self.imu_O[0], self.imu_O[1], self.imu_O[2]]).getR(), self.rInitO_inverse)
        imuq = matrix2quaternion(getArray(imuR))
        # print("imuq:", imuq)
        self.controllerRobot.updateRobotOrientationAndEEs(imuq)

        self.printPoseDetails()
        self.slopeIMU = []

    def printPoseDetails(self):
        h = 1000*self.frH_Pos[2]
        self.output_h.append(h)
        # print("fr_h:", 1000*self.frH_Pos[0], 1000*self.frH_Pos[1], h)

        q = matrix2quaternion(getArray(self.frH_O))
        self.output_q0.append(float(q[0]))
        self.output_q1.append(float(q[1]))
        self.output_q2.append(float(q[2]))
        self.output_q3.append(float(q[3]))
        # print("fr_O:", q)
        # print("imu_O:", self.imu_O)

        # calculate the average axis of rotation between consecutive readings
        mlSlopeIMU = (self.slopeIMU[6] + self.slopeIMU[7] + self.slopeIMU[8]) / 3
        brSlopeIMU = (self.slopeIMU[1] + self.slopeIMU[2] + self.slopeIMU[3]) / 3

        print("mlSlopeIMU:", mlSlopeIMU)
        print("brSlopeIMU:", brSlopeIMU)

        # get the positions of the middle left and back right end effectors to calculate the intersection of the axis of rotation
        middleLeftLeg = self.controllerRobot.middleLeftLeg
        backRightLeg = self.controllerRobot.backRightLeg
        frontRightLeg = self.controllerRobot.frontRightLeg

        mlX = middleLeftLeg.footPositionWRTRobotOrientation[0] / 1000.0
        mlY = middleLeftLeg.footPositionWRTRobotOrientation[1] / 1000.0
        mlZ = middleLeftLeg.footPositionWRTRobotOrientation[2] / 1000.0
        brX = backRightLeg.footPositionWRTRobotOrientation[0] / 1000.0
        brY = backRightLeg.footPositionWRTRobotOrientation[1] / 1000.0
        brZ = backRightLeg.footPositionWRTRobotOrientation[2] / 1000.0

        # print("rPos", 1000*self.rPos[0], 1000*self.rPos[1], 1000*self.rPos[2])

        RT = np.transpose(Quaternion(None, self.pRoll, self.pPitch, 0).getR())

        print("ml fk:", middleLeftLeg.footPositionWRTRobotOrientation[0], middleLeftLeg.footPositionWRTRobotOrientation[1], middleLeftLeg.footPositionWRTRobotOrientation[2])
        print("ml rel", 1000*self.p_ml[0], 1000*self.p_ml[1], 1000*self.p_ml[2])
        print(np.matmul(RT, [1000*self.p_ml[0], 1000*self.p_ml[1], 1000*self.p_ml[2]]))
        print("br fk:", backRightLeg.footPositionWRTRobotOrientation[0], backRightLeg.footPositionWRTRobotOrientation[1], backRightLeg.footPositionWRTRobotOrientation[2])
        print("br rel", 1000*self.p_br[0], 1000*self.p_br[1], 1000*self.p_br[2])
        print(np.matmul(RT, [1000*self.p_br[0], 1000*self.p_br[1], 1000*self.p_br[2]]))
        print("frPos:", 1000*(self.frPos[0] - self.rPos[0]), 1000*(self.frPos[1] - self.rPos[1]), 1000*(self.frPos[2]))
        print("fr rel", 1000*self.p_fr[0], 1000*self.p_fr[1], 1000*self.p_fr[2])
        print(np.matmul(RT, [1000*self.p_fr[0], 1000*self.p_fr[1], 1000*self.p_fr[2]]))



        # calculate intersection point
        num2 = mlSlopeIMU * mlX - mlY - brSlopeIMU * brX + brY
        den2 = mlSlopeIMU - brSlopeIMU
        frX2 = num2 / den2
        frY2 = mlSlopeIMU * frX2 - mlSlopeIMU * mlX + mlY

        # we are determining the proprioception of the leg so we offset the end effector position to where the leg connects to the body (a known poistion)
        hipH = frontRightLeg.hipPositionWRTRobotOrientation
        # print("hipH:", hipH)

        x2 = 1000 * frX2 - hipH[0]
        y2 = 1000 * frY2 - hipH[1]
        z2 = 1000 * (brZ + mlZ)/2.0

        self.output_eex.append(float(x2))
        self.output_eey.append(float(y2))
        self.output_eez.append(float(z2))

        actualX = float(1000*(self.frPos[0]-self.frH_Pos[0]))
        actualY = float(1000*(self.frPos[1]-self.frH_Pos[1]))
        actualZ = float(1000*(self.frPos[2]))

        self.output_eex_actual.append(actualX)
        self.output_eey_actual.append(actualY)
        self.output_eez_actual.append(actualZ)

        # print("frPos_Calculated:", x2, y2, z2)
        # print("frPos_Actual:", actualX, actualY, actualZ)
        # print("frPos_Diff:", x2 - actualX, y2 - actualY, z2 - actualZ)

    def getPositionFromRobotNode(self, node):
        H = node.getPose(self.robot_node)  # this is a flattened homogeneous transformation
        x = H[3]
        y = H[7]
        z = H[11]
        return [x, y, z]

    def getPositionFromRobotNode_relative(self, node):
        imu_O = Quaternion(self.imu.getQuaternion())
        H = node.getPose(self.robot_node)  # this is a flattened homogeneous
        H = np.matmul(np.transpose(imu_O.matrix), [H[3], H[7], H[11], 1])
        x = H[0]
        y = H[1]
        z = H[2]

        return [x, y, z]

    def outputPosesForSimulatedAnnealingAlgorithm(self):
        # after all poses have been calculated output the data to be consumed by the simulated annealing algorithm
        print("H = ", self.output_h)
        print("q0 = ", self.output_q0)
        print("q1 = ", self.output_q1)
        print("q2 = ", self.output_q2)
        print("q3 = ", self.output_q3)
        print("s1 = ", self.output_s1)
        print("s2 = ", self.output_s2)
        print("s3 = ", self.output_s3)
        print("x = ", self.output_eex)
        print("y = ", self.output_eey)
        print("z = ", self.output_eez)

        print("x = ", self.output_eex)
        print("y = ", self.output_eey)
        print("z = ", self.output_eez)

        print("")
        print("")
        print("")

        print("x_actual = ", self.output_eex_actual)
        print("y_actual = ", self.output_eey_actual)
        print("z_actual = ", self.output_eez_actual)

        print(time.strftime("%H:%M:%S", time.localtime()))

        print("INPUT ARRAY")
        print(self.inputArray)
        print("OUTPUT ARRAY")
        print(self.outputArray)

