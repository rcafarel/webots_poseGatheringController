import time


class FindPoses:

    def __init__(self, robot, injuredLeg):
        print("Start Gathering Poses")
        self.printTime()

        self.robot = robot
        self.injuredLeg = injuredLeg
        self.otherTripodLegs = []
        self.otherInitialSupportingLegs = []

        self.commands = []
        self.subCommands = []
        self.servoPositionsArray = []
        self.timeArray = []

        self.defineLegRoles()

        self.defineInstructionsForController()

        self.breakDownCommandsIntoSubCommands()

        print("End Gathering Poses")
        self.printTime()

    def breakDownCommandsIntoSubCommands(self):
        previousCommand = None
        for command in self.commands:
            if command['command'] == 'readGyro':
                command['subCommands'] = [["readGyro"]]
            elif command['command'] == 'readPosition':
                command['subCommands'] = [["readPosition"]]
            elif command['command'] == 'nextPose':
                command['subCommands'] = [["nextPose", command['message']]]
            elif command['command'] == 'getInitialOrientation':
                command['subCommands'] = [["getInitialOrientation"]]
            else:
                command['subCommands'] = self.makeSubCommands(command, previousCommand)
                previousCommand = command

        # map the subcommands from each command from the digital twin into an array of subcommands
        for i in range(len(self.commands)):
            command = self.commands[i]
            for j in range(len(command['subCommands'])):
                self.subCommands.append(command['subCommands'][j])

    def makeSubCommands(self, command, previousCommand):

        if previousCommand == None:
            positions = []
            for i in range(18):
                positions.append(command['servoPositions'][i+1])
            return [positions]

        previousPositions = []
        positionDiffs = []
        numberOfIntervals = command['time'] / 0.05
        for i in range(18):
            previousPositions.append(previousCommand['servoPositions'][i+1])
            positionDiffs.append((command['servoPositions'][i+1] - previousCommand['servoPositions'][i+1]) / numberOfIntervals)

        subCommands = [previousPositions]

        counter = 0
        while counter <= numberOfIntervals:
            currentPositions = []
            for i in range(18):
                currentPositions.append(previousPositions[i] + positionDiffs[i])
            counter += 1
            subCommands.append(currentPositions)
            previousPositions = currentPositions

        return subCommands

    def printTime(self):
        print(time.strftime("%H:%M:%S", time.localtime()))

    def printServoPositionsAndTime(self, t):
        servoPositions = {}
        legServoPositions = self.robot.backLeftLeg.getServoPositions()
        servoPositions[1] = legServoPositions[0]
        servoPositions[2] = legServoPositions[1]
        servoPositions[3] = legServoPositions[2]
        legServoPositions = self.robot.middleLeftLeg.getServoPositions()
        servoPositions[4] = legServoPositions[0]
        servoPositions[5] = legServoPositions[1]
        servoPositions[6] = legServoPositions[2]
        legServoPositions = self.robot.frontLeftLeg.getServoPositions()
        servoPositions[7] = legServoPositions[0]
        servoPositions[8] = legServoPositions[1]
        servoPositions[9] = legServoPositions[2]
        legServoPositions = self.robot.backRightLeg.getServoPositions()
        servoPositions[10] = legServoPositions[0]
        servoPositions[11] = legServoPositions[1]
        servoPositions[12] = legServoPositions[2]
        legServoPositions = self.robot.middleRightLeg.getServoPositions()
        servoPositions[13] = legServoPositions[0]
        servoPositions[14] = legServoPositions[1]
        servoPositions[15] = legServoPositions[2]
        legServoPositions = self.robot.frontRightLeg.getServoPositions()
        servoPositions[16] = legServoPositions[0]
        servoPositions[17] = legServoPositions[1]
        servoPositions[18] = legServoPositions[2]

        self.servoPositionsArray.append(servoPositions)
        self.timeArray.append(t)
        self.commands.append({"command": "goToPosition", "servoPositions": servoPositions, "time": t})


    def defineInstructionsForController(self):
        # print("Gather poses")
        leg0 = self.otherTripodLegs[0]
        leg1 = self.otherTripodLegs[1]
        iLeg = self.injuredLeg
        currentServo = iLeg.servoArray[0]
        currentServo2 = iLeg.servoArray[1]
        currentServo3 = iLeg.servoArray[2]

        self.updateOtherInitialSupportingLegs()
        self.lowerOtherInitialSupportingLegs()

        self.commands.append({"command": "getInitialOrientation", "servoPositions": [], "time": 0})

        for ticks in [450, 550, 650]:
            currentServo.goToTicks(ticks, 0.5)

            for ticks2 in [550, 650, 750]:
                currentServo2.goToTicks(ticks2, 0.5)

                for ticks3 in [ticks2, ticks2+50, ticks2+100]:
                    currentServo3.goToTicks(ticks3, 0.5)

                    for hs in [{'h0': -80, 'h1': -100}, {'h0': -100, 'h1': -80}]:

                        h0Dev = [hs['h0'] + 10, hs['h0'] + 5, hs['h0'], hs['h0'] - 5, hs['h0'] - 10]
                        h1Dev = [hs['h1'] + 10, hs['h1'] + 5, hs['h1'], hs['h1'] - 5, hs['h1'] - 10]

                        self.moveLeg(leg0, hs['h0'])
                        self.moveLeg(leg1, hs['h1'])

                        self.raiseOtherInitialSupportingLegs()
                        self.commands.append({"command": "readPosition", "servoPositions": {}, "time": 0})

                        for h0 in h0Dev:
                            self.moveLeg(leg0, h0)
                            self.commands.append({"command": "readGyro", "servoPositions": {}, "time": 0})

                        self.moveLeg(leg0, hs['h0'])
                        self.moveLeg(leg1, hs['h1'])

                        for h1 in h1Dev:
                            self.moveLeg(leg1, h1)
                            self.commands.append({"command": "readGyro", "servoPositions": {}, "time": 0})

                        self.lowerOtherInitialSupportingLegs()

                        self.commands.append({"command": "nextPose", "servoPositions": {}, "time": 0, "message": "BR: " + str(hs['h1']) + ", ML: " + str(hs['h0']) + ", S1: " + str(ticks) + ", S2: " + str(ticks2) + ", S3: " + str(ticks3)})


    def moveLeg(self, leg, h):
        leg.goToPosition(leg.xWalkingCentroid, leg.yWalkingCentroid, h, 0.5)
        self.printServoPositionsAndTime(0.5)


    def raiseOtherInitialSupportingLegs(self):
        for leg in self.otherInitialSupportingLegs:
            leg.goToPosition(1.5*leg.xWalkingCentroid, leg.yWalkingCentroid, -40.0, 2.0)

        self.printServoPositionsAndTime(2.0)

    def lowerOtherInitialSupportingLegs(self):
        for leg in self.otherInitialSupportingLegs:
            leg.goToPosition(leg.xWalkingCentroid, leg.yWalkingCentroid, -150.0, 0.75)

        self.printServoPositionsAndTime(0.75)

    def updateOtherInitialSupportingLegs(self):
        for leg in self.otherInitialSupportingLegs:
            leg.xWalkingCentroid = leg.xWalkingCentroid * 0.8
            leg.yWalkingCentroid = leg.yWalkingCentroid * 0.9



    # we want to ensure static stability while searching for the initial foothold pose for the injured leg
    # we will use the otherTripodLegs and otherInitialSupportingLegs to create initial static stability
    def defineLegRoles(self):
        if self.injuredLeg == self.robot.backLeftLeg:
            self.otherTripodLegs.append(self.robot.frontLeftLeg)
            self.otherTripodLegs.append(self.robot.middleRightLeg)
            self.otherInitialSupportingLegs.append(self.robot.backRightLeg)
            self.otherInitialSupportingLegs.append(self.robot.middleLeftLeg)
            self.otherInitialSupportingLegs.append(self.robot.frontRightLeg)
        elif self.injuredLeg == self.robot.backRightLeg:
            self.otherTripodLegs.append(self.robot.frontRightLeg)
            self.otherTripodLegs.append(self.robot.middleLeftLeg)
            self.otherInitialSupportingLegs.append(self.robot.backLeftLeg)
            self.otherInitialSupportingLegs.append(self.robot.middleRightLeg)
            self.otherInitialSupportingLegs.append(self.robot.frontLeftLeg)
        elif self.injuredLeg == self.robot.middleLeftLeg:
            self.otherTripodLegs.append(self.robot.frontRightLeg)
            self.otherTripodLegs.append(self.robot.backRightLeg)
            self.otherInitialSupportingLegs.append(self.robot.frontLeftLeg)
            self.otherInitialSupportingLegs.append(self.robot.middleRightLeg)
            self.otherInitialSupportingLegs.append(self.robot.backLeftLeg)
        elif self.injuredLeg == self.robot.middleRightLeg:
            self.otherTripodLegs.append(self.robot.frontLeftLeg)
            self.otherTripodLegs.append(self.robot.backLeftLeg)
            self.otherInitialSupportingLegs.append(self.robot.frontRightLeg)
            self.otherInitialSupportingLegs.append(self.robot.middleLeftLeg)
            self.otherInitialSupportingLegs.append(self.robot.backRightLeg)
        elif self.injuredLeg == self.robot.frontLeftLeg:
            self.otherTripodLegs.append(self.robot.middleRightLeg)
            self.otherTripodLegs.append(self.robot.backLeftLeg)
            self.otherInitialSupportingLegs.append(self.robot.frontRightLeg)
            self.otherInitialSupportingLegs.append(self.robot.middleLeftLeg)
            self.otherInitialSupportingLegs.append(self.robot.backRightLeg)
        elif self.injuredLeg == self.robot.frontRightLeg:
            self.otherTripodLegs.append(self.robot.middleLeftLeg)
            self.otherTripodLegs.append(self.robot.backRightLeg)
            self.otherInitialSupportingLegs.append(self.robot.frontLeftLeg)
            self.otherInitialSupportingLegs.append(self.robot.middleRightLeg)
            self.otherInitialSupportingLegs.append(self.robot.backLeftLeg)

