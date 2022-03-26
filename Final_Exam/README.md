## Instructions

1. Read the Report `MAE263B_Final_Exam_Shivam_105730045_Report.pdf` for answers and references to all the questions in the final exam. It also has details to all Matlab files required to see the answers. 

2. For part 1 Kinematic Analysis, refer to:
* `MAE263B_Final_Exam_Shivam_105730045_Report.pdf`
* `ForwardKinematicsAnalysis.mlx`
* `InverseKinematicsPuma.mlx`
* `IKPuma.m`
* `InverseKinematicsTesting.mlx`
* `JacobianPuma.mlx`

3. For part 2 Dynamics, refer to:
* `MAE263B_Final_Exam_Shivam_105730045_Report.pdf`
* `DynamicsPuma.mlx`
* `DynamicsPuma2.mlx`

3. For part 3 Trajectory Generation, refer to:
* `MAE263B_Final_Exam_Shivam_105730045_Report.pdf`
* `TrajectoryGenerationJointSpace.mlx`
* `TrajectoryGenerationTaskSpace.mlx` 

## Functions Details

* `IKPuma.m`: Calculates all inverse kinematics solution given a point. 
* `FKPuma.m`: Takes all joint angles as input and ouputs the end effector transformation matrix.
* `IKPuma2.m`, `IKPuma3.m` and `IKPuma4.m` all of them gives a single inverse kinematic solution with preference to the nearest solution for $\theta_1$ and elbow-up position for $\theta_2$ & $\theta_3$. 
* `calculatePositionError.m`: Calculates the position as well as the orientation error between the required trajectory point and the actual trajectory point. 
* `plotLetter.m` : Generates via points for spline-segments of a letter.
* `plotLines.m` : Generates via points for straight-line-segments of a letter. 
* `plotScript.m`: Plots the alphabet using the generated via points.
* `plotThetas.m`: Plots all joint angles in a single plot. 
* `plotxyz.m`: Plots all x,y and z positions of the tool tip in a single plot.
* `plotEulerAngles`: Plots euler angles at required times.
* `JSTrajectory1`: Outputs joint angles based on joint space trajectory scheme. 
* `TSTrajectory1`: Outputs joint angles based on task space trajectory scheme.
* `cubicTrajectory.m` : Calculates a cubic trajectory between via points. 
* `singleAxis.m` : Converts a Transformation matrix to $\chi$ matrix with equivalent Angle-Axis representation
* `wrapToHalfPi.m`: Converts an angle to a scale within $-\pi$ to $\pi$ radians. 