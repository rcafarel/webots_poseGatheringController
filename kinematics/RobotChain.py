import math

from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink, DHLink
import warnings

def getChain(leg):
    warnings.filterwarnings("ignore", message="Link")
    links = []
    links.append(OriginLink())
    links.append(URDFLink(name="toLegTranslate",  joint_type='fixed',
                          origin_orientation=[leg.robotQ.getRoll(), leg.robotQ.getPitch(), 0.0],
                          origin_translation=[leg.robotPosition.getX() + leg.positionX, leg.robotPosition.getY() + leg.positionY, leg.robotPosition.getZ()]))
    links.append(URDFLink(name="toLegRotate",  joint_type='fixed',
                          origin_translation=[0, 0, 0],
                          origin_orientation=[0, 0, leg.orientationZ.theta]))

    for i in range(leg.numberOfServos):
        links.append(URDFLink(name="translate" + str(i), joint_type='fixed',
                              origin_orientation=[0, 0, 0],
                              origin_translation=[leg.legSegmentArray[i].getX(),
                                                  leg.legSegmentArray[i].getY(),
                                                  leg.legSegmentArray[i].getZ()]))
        links.append(URDFLink(name="rotate" + str(i), joint_type='fixed',
                              origin_translation=[0, 0, 0],
                              origin_orientation=[leg.servoOrientationArray[i].getRoll(),
                                                  leg.servoOrientationArray[i].getPitch(),
                                                  leg.servoOrientationArray[i].getYaw()]))
        links.append(URDFLink(name="servo" + str(i),
                              origin_translation=[0, 0, 0],
                              origin_orientation=[0, 0, 0],
                              rotation=[0, 0, 1]))

    links.append(URDFLink(name="lowerLeg",  joint_type='fixed',
                          origin_orientation=[0, 0, 0],
                          origin_translation=[leg.lowerLegSegment.getX(),
                                              leg.lowerLegSegment.getY(),
                                              leg.lowerLegSegment.getZ()]))

    return Chain(name='defaultLeg', links=links)

def createChain(x, y, theta, legSegmentMatrix, servoOrientationMatrix, lowerLegSegmentArray):
    warnings.filterwarnings("ignore", message="Link")
    links = []
    links.append(OriginLink())
    links.append(URDFLink(name="toLegTranslate",  joint_type='fixed',
                          origin_orientation=[0.0, 0.0, 0.0],
                          origin_translation=[x, y, 0.0]))
    links.append(URDFLink(name="toLegRotate",  joint_type='fixed',
                          origin_translation=[0.0, 0.0, 0.0],
                          origin_orientation=[0.0, 0.0, theta]))

    for i in range(len(legSegmentMatrix)):
        legSegmentArray = legSegmentMatrix[i]
        servoOrientationArray = servoOrientationMatrix[i]
        links.append(URDFLink(name="translate" + str(i), joint_type='fixed',
                              origin_orientation=[0.0, 0.0, 0.0],
                              origin_translation=[legSegmentArray[0], legSegmentArray[1], legSegmentArray[2]]))
        links.append(URDFLink(name="rotate" + str(i), joint_type='fixed',
                              origin_translation=[0.0, 0.0, 0.0],
                              origin_orientation=[servoOrientationArray[0], servoOrientationArray[1], servoOrientationArray[2]]))
        links.append(URDFLink(name="servo" + str(i),
                              origin_translation=[0.0, 0.0, 0.0],
                              origin_orientation=[0.0, 0.0, 0.0],
                              rotation=[0.0, 0.0, 1.0]))

    links.append(URDFLink(name="lowerLeg",  joint_type='fixed',
                          origin_orientation=[0.0, 0.0, 0.0],
                          origin_translation=[lowerLegSegmentArray[0], lowerLegSegmentArray[1], lowerLegSegmentArray[2]]))

    return Chain(name='defaultLeg', links=links)


def createDHChain(x, y, theta, dhSegments):
    warnings.filterwarnings("ignore", message="Link")
    links = []
    links.append(OriginLink())
    links.append(URDFLink(name="toLegTranslate",  joint_type='fixed',
                          origin_orientation=[0.0, 0.0, 0.0],
                          origin_translation=[x, y, 0.0]))
    links.append(URDFLink(name="toLegRotate",  joint_type='fixed',
                          origin_translation=[0.0, 0.0, 0.0],
                          origin_orientation=[0.0, 0.0, theta]))
    for i in range(len(dhSegments)):
        dhSegment = dhSegments[i]
        linkLength = math.sqrt(dhSegment.d*dhSegment.d + dhSegment.a*dhSegment.a)
        dhLink = DHLink(name="dh" + str(i), d=dhSegment.d, a=dhSegment.a, alpha=dhSegment.alpha, theta=dhSegment.theta) #, length=linkLength)
        dhLink.joint_type = 'fixed'
        links.append(dhLink)

        if i < len(dhSegments)-1:
            links.append(URDFLink(name="spacer" + str(i), joint_type='fixed',
                                  origin_orientation=[0.0, 0.0, 0.0],
                                  origin_translation=[0.0, 0.0, 0.0]))  # this link is here to make the servos indicies match the old style legs
            links.append(URDFLink(name="servo" + str(i),
                              origin_translation=[0.0, 0.0, 0.0],
                              origin_orientation=[0.0, 0.0, 0.0],
                              rotation=[0.0, 0.0, 1.0])) # this link is the servo, it is not a fixed link

    return Chain(name='dhLeg', links=links)

def createChainWithOrientation(x, y, theta, legSegmentMatrix, servoOrientationMatrix, lowerLegSegmentArray, ox, oy, oz):
    warnings.filterwarnings("ignore", message="Link")
    links = []
    links.append(OriginLink())
    links.append(URDFLink(name="toLegTranslate",  joint_type='fixed',
                          origin_orientation=[ox, oy, oz],
                          origin_translation=[x, y, 0.0]))
    links.append(URDFLink(name="toLegRotate",  joint_type='fixed',
                          origin_translation=[0.0, 0.0, 0.0],
                          origin_orientation=[0.0, 0.0, theta]))

    for i in range(len(legSegmentMatrix)):
        legSegmentArray = legSegmentMatrix[i]
        servoOrientationArray = servoOrientationMatrix[i]
        links.append(URDFLink(name="translate" + str(i), joint_type='fixed',
                              origin_orientation=[0.0, 0.0, 0.0],
                              origin_translation=[legSegmentArray[0], legSegmentArray[1], legSegmentArray[2]]))
        links.append(URDFLink(name="rotate" + str(i), joint_type='fixed',
                              origin_translation=[0.0, 0.0, 0.0],
                              origin_orientation=[servoOrientationArray[0], servoOrientationArray[1], servoOrientationArray[2]]))
        links.append(URDFLink(name="servo" + str(i),
                              origin_translation=[0.0, 0.0, 0.0],
                              origin_orientation=[0.0, 0.0, 0.0],
                              rotation=[0.0, 0.0, 1.0]))

    links.append(URDFLink(name="lowerLeg",  joint_type='fixed',
                          origin_orientation=[0.0, 0.0, 0.0],
                          origin_translation=[lowerLegSegmentArray[0], lowerLegSegmentArray[1], lowerLegSegmentArray[2]]))

    return Chain(name='defaultLeg', links=links)

def getChainHipToOrigin(x, y, z, ox, oy, oz):
    warnings.filterwarnings("ignore", message="Link")
    links = []
    links.append(OriginLink())
    links.append(URDFLink(name="orientWithOrigin",  joint_type='fixed',
                          origin_orientation=[ox, oy, oz],
                          origin_translation=[0.0, 0.0, z]))

    links.append(URDFLink(name="translateBackToOrigin",  joint_type='fixed',
                          origin_orientation=[0.0, 0.0, 0.0],
                          origin_translation=[x, y, 0.0]))
    
    return Chain(name='hipToOrigin', links=links)

def getChainOriginToHip(x, y, h, ox, oy, oz):
    warnings.filterwarnings("ignore", message="Link")
    links = []
    links.append(OriginLink())
    links.append(URDFLink(name="orientWithOriginAndHeight",  joint_type='fixed',
                          origin_orientation=[ox, oy, oz],
                          origin_translation=[0.0, 0.0, h]))

    links.append(URDFLink(name="translateToHip",  joint_type='fixed',
                          origin_orientation=[0.0, 0.0, 0.0],
                          origin_translation=[-1.0*x, -1.0*y, 0.0]))
    
    return Chain(name='originToHip', links=links)

# contactPositions should always be at least (but probably more) 2 more than swingPositions
# endEffectorLiftDeviation should always be positive
# assume for now even terrain and level trajectory
def getEndEffectorSequence(leg, startContactPosition, endContactPosition, endEffectorLiftDeviation):
    contactPositions = 13

    positionArray = []
    contactPositionArray = []
    xDev = (endContactPosition[0] - startContactPosition[0]) / (contactPositions - 1)
    yDev = (endContactPosition[1] - startContactPosition[1]) / (contactPositions - 1)
    zDev = (endContactPosition[2] - startContactPosition[2]) / (contactPositions - 1)
    for i in range(contactPositions):
        nextX = startContactPosition[0] + xDev * i
        nextY = startContactPosition[1] + yDev * i
        nextZ = startContactPosition[2] + zDev * i
        contactPositionArray.append([nextX, nextY, nextZ])
        positionArray.append(contactPositionArray[i])

    swingPositionArray = []
    swingPositions = 9  # intentionally odd so the middle position is the peak
    swingMidPoint = 5
    
    # lift, forward-up, forward-down, drop
    verticalDeviation = endEffectorLiftDeviation / 4

    nextZ = endContactPosition[2]

    xDev = (startContactPosition[0] - endContactPosition[0]) / (swingPositions - 1)
    yDev = (startContactPosition[1] - endContactPosition[1]) / (swingPositions - 1)
    for i in range(swingPositions):
        nextX = endContactPosition[0] + xDev * i
        nextY = endContactPosition[1] + yDev * i
        if i > 5:
            nextZ -= verticalDeviation
        elif i < 4:
            nextZ += verticalDeviation
        swingPositionArray.append([nextX, nextY, nextZ])
        positionArray.append(swingPositionArray[i])

    # this outputs first and last position as the same
    if leg.firstTripod:
        positionArray.append(contactPositionArray[0])
        #for i in range(len(positionArray)):
        #    print(positionArray[i])
        return positionArray
    else:
        offset = (contactPositions - swingPositions) / 2  # this should be an integer
        secondTripodPositionArray = []
        for i in range(int(swingPositions + offset), len(positionArray)):
            secondTripodPositionArray.append(positionArray[i])
        for i in range(int(swingPositions + offset + 1)):
            secondTripodPositionArray.append(positionArray[i])
        #for i in range(len(secondTripodPositionArray)):
        #    print(secondTripodPositionArray[i])
        return secondTripodPositionArray


def getEndEffectorSequence_shortSequence(leg, startContactPosition, endContactPosition, endEffectorLiftDeviation):
    #contactPositions = 13
    contactPositions = 7

    positionArray = []
    contactPositionArray = []
    xDev = (endContactPosition[0] - startContactPosition[0]) / (contactPositions - 1)
    yDev = (endContactPosition[1] - startContactPosition[1]) / (contactPositions - 1)
    zDev = (endContactPosition[2] - startContactPosition[2]) / (contactPositions - 1)
    for i in range(contactPositions):
        nextX = startContactPosition[0] + xDev * i
        nextY = startContactPosition[1] + yDev * i
        nextZ = startContactPosition[2] + zDev * i
        contactPositionArray.append([nextX, nextY, nextZ])
        positionArray.append(contactPositionArray[i])

    swingPositionArray = []
    #swingPositions = 9  # intentionally odd so the middle position is the peak
    swingPositions = 3
    #swingMidPoint = 5
    swingMidPoint = 2
    # lift, forward-up, forward-down, drop
    #verticalDeviation = endEffectorLiftDeviation / 4
    verticalDeviation = endEffectorLiftDeviation

    nextZ = endContactPosition[2]

    xDev = (startContactPosition[0] - endContactPosition[0]) / (swingPositions - 1)
    yDev = (startContactPosition[1] - endContactPosition[1]) / (swingPositions - 1)
    for i in range(swingPositions):
        nextX = endContactPosition[0] + xDev * i
        nextY = endContactPosition[1] + yDev * i
        #if i > 5:
        if i > 2:
            nextZ -= verticalDeviation
        #elif i < 4:
        elif i < 1:
            nextZ += verticalDeviation
        swingPositionArray.append([nextX, nextY, nextZ])
        positionArray.append(swingPositionArray[i])

    # this outputs first and last position as the same
    if leg.firstTripod:
        positionArray.append(contactPositionArray[0])
        for i in range(len(positionArray)):
            print(positionArray[i])
        return positionArray
    else:
        offset = (contactPositions - swingPositions) / 2  # this should be an integer
        secondTripodPositionArray = []
        for i in range(int(swingPositions + offset), len(positionArray)):
            secondTripodPositionArray.append(positionArray[i])
        for i in range(int(swingPositions + offset + 1)):
            secondTripodPositionArray.append(positionArray[i])
        for i in range(len(secondTripodPositionArray)):
            print(secondTripodPositionArray[i])
        return secondTripodPositionArray
    

def getEndEffectorSequence_extraShortSequence(leg, startContactPosition, endContactPosition, endEffectorLiftDeviation):
    contactPositions = 3

    positionArray = []
    contactPositionArray = []
    xDev = (endContactPosition[0] - startContactPosition[0]) / (contactPositions - 1)
    yDev = (endContactPosition[1] - startContactPosition[1]) / (contactPositions - 1)
    zDev = (endContactPosition[2] - startContactPosition[2]) / (contactPositions - 1)
    for i in range(contactPositions):
        nextX = startContactPosition[0] + xDev * i
        nextY = startContactPosition[1] + yDev * i
        nextZ = startContactPosition[2] + zDev * i
        contactPositionArray.append([nextX, nextY, nextZ])
        positionArray.append(contactPositionArray[i])

    swingPositionArray = []
    swingPositions = 1
    # lift, forward-up, forward-down, drop

    nextZ = endContactPosition[2]

    nextX = endContactPosition[0] + (startContactPosition[0] - endContactPosition[0]) / 2.0
    nextY = endContactPosition[1] + (startContactPosition[1] - endContactPosition[1]) / 2.0
    nextZ = endContactPosition[2] + endEffectorLiftDeviation
    positionArray.append([nextX, nextY, nextZ])

    # this outputs first and last position as the same
    if leg.firstTripod:
        positionArray.append(contactPositionArray[0])
        for i in range(len(positionArray)):
            print(positionArray[i])
        return positionArray
    else:
        secondTripodPositionArray = []
        secondTripodPositionArray.append(positionArray[2])
        secondTripodPositionArray.append(positionArray[3])
        secondTripodPositionArray.append(positionArray[0])
        secondTripodPositionArray.append(positionArray[1])
        secondTripodPositionArray.append(positionArray[2])
        for i in range(len(secondTripodPositionArray)):
            print(secondTripodPositionArray[i])
        return secondTripodPositionArray
