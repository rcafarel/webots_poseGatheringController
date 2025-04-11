"""poseGatheringController controller."""
import math
import time

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Supervisor

from FindPoses import FindPoses
from PoseDefinition import PoseDefinition
from SimpleRobotForFindingPoses import SimpleRobotForFindingPoses

print(time.strftime("%H:%M:%S", time.localtime()))

# create the Robot instance.
robot = Robot()

imu = robot.getDevice("imu")
imu.enable(20) # 20ms collection rate

controllerRobot = SimpleRobotForFindingPoses()
injuredLeg = controllerRobot.frontRightLeg

# define each servo which will be used with the position controller
servos = {}
for i in range(18):
    servos[i+1] = robot.getDevice('servo' + str(i+1) + 'Motor')

frontRightLeg_segment3_twistAngle = 90.0 * math.pi / 180.0
supervisor = Supervisor()

# injuredLegSegment = supervisor.getFromDef("s3")
# print(injuredLegSegment.getRotation())
# injuredLegSegment.setRotation(0, frontRightLeg_segment3_twistAngle, 0)

findPoses = FindPoses(controllerRobot, injuredLeg)
subCommands = findPoses.subCommands

commandCounter = 0

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

poseDefinition = PoseDefinition(supervisor, controllerRobot, imu)

previousPositions = None
previousState = None


# Main loop:
# - perform simulation steps until Webots is stopping the controller
# - or we reach the end of our subcommands
# print(subCommands)
while robot.step(timestep) != -1 and commandCounter < len(subCommands):
    subCommand = subCommands[commandCounter]
    commandCounter += 1

    if subCommand[0] == 'nextPose':
        # before resetting variables for the next pose, calculate and record pose details
        poseDefinition.calculatePoseAndInitializeNextPose(subCommand)

    elif subCommand[0] == 'getInitialOrientation':
        # read the initial orientation of the robot to make subsequent readings indifferent of initial orientation
        poseDefinition.initializeRobotOrientation()

    elif subCommand[0] == 'readPosition':
        poseDefinition.readPosition(subCommand)

    elif subCommand[0] == 'readGyro':
        poseDefinition.readOrientation()

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
        nextState = poseDefinition.imu.getRollPitchYaw() + poseDefinition.robot_node.getCenterOfMass() + poseDefinition.ee_frontRightLeg.getCenterOfMass()
        if previousPositions is not None:
            action = []
            for i in range(18):
                action.append(previousPositions[i] - nextPositions[i])

            poseDefinition.inputArray.append(previousState + previousPositions + action)
            poseDefinition.outputArray.append(nextState + nextPositions)

        previousPositions = nextPositions
        previousState = nextState


poseDefinition.outputPosesForSimulatedAnnealingAlgorithm()

