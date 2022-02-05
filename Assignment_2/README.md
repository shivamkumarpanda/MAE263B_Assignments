## Instructions

1. For Joint Space Trajectories open and run `JointSpaceTrajectoryScript.m`. This file also includes Simulation/Animation with the robot model and end-effector position, hence it takes time to run. To check only the plots one can open and run `JointSpaceTrajectoryLiveScript.mlx`.

2. For Task Space Trajectories open and run `TaskSpaceTrajectoryScript.m`. This file also includes Simulation/Animation with the robot model and end-effector position, hence it takes time to run. To check only the plots one can open and run `TaskSpaceTrajectoryLiveScript.mlx`.

3. Optimization of positions has been done for Joint Space and Task Space in files `JointSpaceTrajectoryScriptOptimize.m` and `TaskSpaceTrajectoryScriptOptimize.m` respectively. 

## Observations

* During optimization it was observed that the overall time was reduced on changing the feeder and PCB position strategically - such that the rotations by Joint1 and Joint2 is reduced. 

* The Kinematic limitations on velocity and collision avoidance were verified. 

* The Assignment 2 Part 1 is also present in the folder with the name `HW2Part1.pdf`.

## Functions Details

* `cubicTrajectory.m` : Calculates a cubic trajectory between via points. 
* `inverseKinematicsScara.m` : Calculates inverse kinematics for the given scara robot
* `singleAxis.m` : Converts a Transformation matrix to $\chi$ matrix with equivalent Angle-Axis representation
* `totalTrajectory.m` : Calculates Joint Space trajectory with fastest time with multiple via points
* `taskSpaceTrajectory.m` : Calculates Task Space trajectory with fastest time with multiple via points   