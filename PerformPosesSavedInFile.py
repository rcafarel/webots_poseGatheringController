import json
import csv
from controller import Robot, Supervisor

filename = 'C:\\Projects\\robot\\webots\\SingleLeg\\controllers\\poseGatheringController\\frontRightLegInjury_FindPoses_subCommands.txt'
subCommands = []
with open(filename, 'r') as file:
    for line in file:
        subCommands.append(line.strip())

print(len(subCommands))
print(subCommands[0])
print(subCommands[1])
print(subCommands[2])

# create the Robot instance.
robot = Robot()
servos = {}
for i in range(18):
    servos[i+1] = robot.getDevice('servo' + str(i+1) + 'Motor')

commandCounter = 0
timestep = int(robot.getBasicTimeStep())

while robot.step(timestep) != -1 and commandCounter < len(subCommands):
    subCommand = subCommands[commandCounter]
    commandCounter += 1

    if subCommand[0] == 'nextPose':
        # before resetting variables for the next pose, calculate and record pose details
        # poseDefinition.calculatePoseAndInitializeNextPose(subCommand)
        print('nextPose')

    elif subCommand[0] == 'getInitialOrientation':
        # read the initial orientation of the robot to make subsequent readings indifferent of initial orientation
        # poseDefinition.initializeRobotOrientation()
        print('getInitialOrientation')

    elif subCommand[0] == 'readPosition':
        # poseDefinition.readPosition(subCommand)
        print('readPosition')

    elif subCommand[0] == 'readGyro':
        # poseDefinition.readOrientation()
        print('readGyro')

    else:  # go to position
        nextPositions = []
        for i in range(18):
            position = subCommand[i]
            if type(position) == 'float':
                if servos[i+1] is not None:
                    servos[i+1].setPosition(position)
                    nextPositions.append(position)
            else:
                if servos[i+1] is not None:
                    servos[i+1].setPosition(float(position))
                    nextPositions.append(float(position))

        robot.step(timestep)
        # nextState = poseDefinition.imu.getRollPitchYaw() + poseDefinition.robot_node.getCenterOfMass() + poseDefinition.ee_frontRightLeg.getCenterOfMass()
        # if previousPositions is not None:
        #     action = []
        #     for i in range(18):
        #         action.append(previousPositions[i] - nextPositions[i])
        #
        #     poseDefinition.inputArray.append(previousState + previousPositions + action)
        #     poseDefinition.outputArray.append(nextState + nextPositions)
        #
        # previousPositions = nextPositions
        # previousState = nextState


# poseDefinition.outputPosesForSimulatedAnnealingAlgorithm()