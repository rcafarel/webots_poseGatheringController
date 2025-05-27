import math

import numpy

from SimpleRobotForFindingPoses import SimpleRobotForFindingPoses

robotGlobalPosition = [44.17636039, -1.306921488, 101.2701326]

robotIMU_RPY = [-0.043114864, -0.027476663, 0.009271837]

# this is the forward kinematics for each end-effector starting from the robot position and orientation
mlEE_wrtRobot = [-250.4309446, -1.229462288, -80.01276642]
brEE_wrtRobot = [200.1494859, -179.8404623, -99.99581721]
frEE_wrtRobot = [215.9582658, 258.0834609, -81.47878469]

eeRadius = 0.005 * 1000

servoPositions = [-0.5584903781083155, -0.904846077584462, -0.8488403772423576, -0.003126129573173333, -0.637791196478492, -0.9607286358007358, 0.556187775639497, -0.9048460828838977, -0.8488403830876045, 0.3827104640642653, 0.4056981460310249, 0.8322857057739854, 0.0010038648240746433, 0.6818842017375893, 0.5056892270122704, -0.20943951023931953, 0.20943951023931953, 0.20943951023931953]

mlServos = [4, 5, 6]
brServos = [10, 11, 12]
frServos = [16, 17, 18]

robot = SimpleRobotForFindingPoses()
mlLeg = robot.middleLeftLeg
brLeg = robot.backRightLeg
frLeg = robot.frontRightLeg

mlLeg.setServoPositionForCalculations(1, servoPositions[mlServos[0]-1])
mlLeg.setServoPositionForCalculations(2, servoPositions[mlServos[1]-1])
mlLeg.setServoPositionForCalculations(3, servoPositions[mlServos[2]-1])
mlLeg.calculateFootPosition()

brLeg.setServoPositionForCalculations(1, servoPositions[brServos[0]-1])
brLeg.setServoPositionForCalculations(2, servoPositions[brServos[1]-1])
brLeg.setServoPositionForCalculations(3, servoPositions[brServos[2]-1])
brLeg.calculateFootPosition()

frLeg.setServoPositionForCalculations(1, servoPositions[frServos[0]-1])
frLeg.setServoPositionForCalculations(2, servoPositions[frServos[1]-1])
frLeg.setServoPositionForCalculations(3, servoPositions[frServos[2]-1])
frLeg.calculateFootPosition()

print("ML_EE", mlLeg.footPosition)
print("BR_EE", brLeg.footPosition)
print("FR_EE", frLeg.footPosition)
print("FR_EE", frEE_wrtRobot)

print("")

def euler_to_rotation_matrix(roll, pitch, yaw):

    # Rotation matrices around the X, Y, and Z axes
    rotation_x = numpy.array([
        [1, 0, 0],
        [0, math.cos(roll), -math.sin(roll)],
        [0, math.sin(roll), math.cos(roll)]
    ])

    rotation_y = numpy.array([
        [math.cos(pitch), 0, math.sin(pitch)],
        [0, 1, 0],
        [-math.sin(pitch), 0, math.cos(pitch)]
    ])

    rotation_z = numpy.array([
        [math.cos(yaw), -math.sin(yaw), 0],
        [math.sin(yaw), math.cos(yaw), 0],
        [0, 0, 1]
    ])

    # Combined rotation matrix
    rotation_matrix = numpy.matmul(rotation_z, numpy.matmul(rotation_y, rotation_x))

    return rotation_matrix



IMU_R = euler_to_rotation_matrix(robotIMU_RPY[0], robotIMU_RPY[1], robotIMU_RPY[2])
IMU_R_T = numpy.transpose(IMU_R)

mlGlobal = numpy.matmul(mlEE_wrtRobot, IMU_R_T)
brGlobal = numpy.matmul(brEE_wrtRobot, IMU_R_T)
frGlobal = numpy.matmul(frEE_wrtRobot, IMU_R_T)

# print("ML*IMU", numpy.matmul(mlEE_wrtRobot, IMU_R))
# print("BR*IMU", numpy.matmul(brEE_wrtRobot, IMU_R))
# print("FR*IMU", numpy.matmul(frEE_wrtRobot, IMU_R))

print("ML*IMU_T", mlGlobal)
print("BR*IMU_T", brGlobal)
print("FR*IMU_T", frGlobal)

print("")

print("IMU*ML", numpy.matmul(IMU_R, mlEE_wrtRobot))
print("IMU*BR", numpy.matmul(IMU_R, brEE_wrtRobot))
print("IMU*FR", numpy.matmul(IMU_R, frEE_wrtRobot))

# print("IMU_T*ML", numpy.matmul(IMU_R_T, mlEE_wrtRobot))
# print("IMU_T*BR", numpy.matmul(IMU_R_T, brEE_wrtRobot))
# print("IMU_T*FR", numpy.matmul(IMU_R_T, frEE_wrtRobot))

print("")

local_m2 = (frEE_wrtRobot[1] - mlEE_wrtRobot[1])/(frEE_wrtRobot[0] - mlEE_wrtRobot[0])
local_m1 = (frEE_wrtRobot[1] - brEE_wrtRobot[1])/(frEE_wrtRobot[0] - brEE_wrtRobot[0])
print("M1_local", local_m1)
print("M2_local", local_m2)

print("")

global_m2 = (frGlobal[1] - mlGlobal[1])/(frGlobal[0] - mlGlobal[0])
global_m1 = (frGlobal[1] - brGlobal[1])/(frGlobal[0] - brGlobal[0])
print("M1_global", global_m2)
print("M2_global", global_m1)

print("")

print("ML slope", 0.555828667)
print("BR slope", 28.33230515)


num2 = local_m2 * mlEE_wrtRobot[0] - mlEE_wrtRobot[1] - local_m1 * brEE_wrtRobot[0] + brEE_wrtRobot[1]
den2 = local_m2 - local_m1
frX2 = num2 / den2
frY2 = local_m2 * frX2 - local_m2 * mlEE_wrtRobot[0] + mlEE_wrtRobot[1]

print("Local FR", frX2, frY2)

num2 = 0.555828667 * mlEE_wrtRobot[0] - mlEE_wrtRobot[1] - 28.33230515 * brEE_wrtRobot[0] + brEE_wrtRobot[1]
den2 = 0.555828667 - 28.33230515
frX2 = num2 / den2
frY2 = 0.555828667 * frX2 - 0.555828667 * mlEE_wrtRobot[0] + mlEE_wrtRobot[1]

print("Local FR", frX2, frY2)
print("")


num2 = global_m2 * mlGlobal[0] - mlGlobal[1] - global_m1 * brGlobal[0] + brGlobal[1]
den2 = global_m2 - global_m1
frX2 = num2 / den2
frY2 = global_m2 * frX2 - global_m2 * mlGlobal[0] + mlGlobal[1]

print("Global FR", frX2, frY2)


num2 = 0.555828667 * mlGlobal[0] - mlGlobal[1] - 28.33230515 * brGlobal[0] + brGlobal[1]
den2 = 0.555828667 - 28.33230515
frX2 = num2 / den2
frY2 = 0.555828667 * frX2 - 0.555828667 * mlGlobal[0] + mlGlobal[1]

print("Global FR", frX2, frY2)


print("Local FR", numpy.matmul(IMU_R_T, [frX2, frY2, -86.55886152]))
# print("Local FR", numpy.matmul(IMU_R, [frX2, frY2, -86.55886152]))
print("Local FR", numpy.matmul([frX2, frY2, -86.55886152], IMU_R))
# print("Local FR", numpy.matmul([frX2, frY2, -86.55886152], IMU_R_T))
