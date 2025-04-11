This repository is used to gather end-effector positions for an injured hexapod leg with three servos.
The code is written as a webots controller to work with a webots world such as SingleLegPoseGathering_15degree.wbt.
The code defines a series of motions from diverse starting points to pinpoint the end-effector position within a tripod stance.
The output of this controller is a series of arrays representing the position and orientation of the end-effector with respect to its connection point to the robot body.
This output acts as the input to the simulated annealing algorithm.
