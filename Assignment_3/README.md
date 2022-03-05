## Instructions

1. For finding the Jacobian through velocity propagation, force propagation and direct differentiation open `JacobianScara.mlx`. The results can already be found or you can try to run again.  

2. For finding the Singularities and the implications accordingly see towards the last in the code `JacobianScara.mlx`. The implications can be found in form of comments in the last. You can try to read that or it would be mentioned later in this document too. 

3. For the link optimisation, open `LinkOptimizationScara.mlx`. The results can already be found or you can try to run again. However on running the optimization algorithm takes a bit longer time (probably since it's using symbolic substitution). If you wish to avoid getting into running that, you can just download presaved data from the `Data` directory. The plots for this part can also be found presaved in the `Plots` directory.

## Observations

### Question 1:

* It's verified that the Jacobians obtained from all methods the same.

### Question 2: 

* From the Jacobian equation we can say that the determinant of the Jacobian is zero at $\theta_2 = 0$ or $\theta_2 = \pi$. Hence all configurations when $\theta_2$ = 0 or $\theta_2 = \pi$ are the  singular configurations. 

* Analytically $\theta_2 = 0$ is the stretched out configuration  and $\theta_2 = \pi$ is the fold-back configuration. 

* Implications:

    1. The manipulator loses 1 DOF. The end effector can only move along the tangent direction of the arm. Motion along the radial direction is not possible.
    
    2. A finite force can be applied to the end effector that produces no torque at the robot’s joints. Hence in this singular configuration, the manipulator can "lock up." 


### Question 3:

* The $\kappa_i$-plot is symmetric about Y=0 (or X-axis). This is because the the workspace is itself symmetric with respect to the manipulator about Y=0 (or X-axis). 

* The best manipulability points are at around $X=0.24 m$ (somewhere close to the middle), and worst manipulability points are at $X_{min} (X=0.2 m)$ and $X_{max} (0.34 m)$ values within the workspace. If we look more closely, we see local minimums at

    1. $(|X|_{min}, |Y|_{min})$ i.e. $(0.2, 0)$
    
    2. $(|X|_{max}, |Y|_{max})$ i.e. $(0.34, 0.5)$ and $(0.34, -0.5)$

* These are the points which are either the closest or the farthest from the manipulator Hence we see low manipulability are closer points and farther 
points within the workspace. 

* The same points mentioned above (with low manipulability) are closer to the singularity of the workspace. 

    1. $(|X|_{min}, |Y|_{min})$ i.e. $(0.2, 0)$
    
    2. $(|X|_{max}, |Y|_{max})$ i.e. $(0.34, 0.5)$ and $(0.34, -0.5)$

* From the catalogue the manipulators with $L_1=0.225$ and $L_2=0.225$ can be a good possibility. For example:
    1. RH-3FRH4515(C)
    2. RH-6FRH4520
    3. RH-6FRH4534

## Functions and Other files Details

* `taskSpaceTrajectory.m` : Calculates Task Space trajectory with fastest time with multiple via points   
* `Data`: This directory has some pre-saved data for `LinkOptimizationScara.mlx`.
* `Plots`: This directory has plots obtained from `LinkOptimizationScara.mlx`