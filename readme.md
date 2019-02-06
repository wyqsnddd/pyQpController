# Impact-friendly task space quadratic optimization robot controller

The simulation is based on the [python interface](https://github.com/sehoonha/pydart2) of the [DART simulator](https://dartsim.github.io/). 

 ### 1. Running guide: 
We can find a quick start from the  [Running guide](doc/running_guide.md)

 ### 2. Implementation: 
 
Interested readers can find the implementation details from the following python files. Basically each inequality constraint "G x <= h " would output the G and h matrices. The similar results could be found for the equality constraint (21). Each task in the folder "manipulatorTasks" outputs Q and P matrices expanded from the 2-norm with respect to the decision variables. 
    

   |Equations in the paper  | Simulation code |
    | ------ | ------ |
    | Joint position limits (6) | manipulatorConstraints/jointPositionLimits.py |
    | Joint velocity limits (7)  |  manipulatorConstraints/jointVelocityConstraints.py|
    | Joint torque limits (8)  |  manipulatorConstraints/torqueLimitConstraints.py|
    | impact joint position limits (15) |  manipulatorConstraints/robustJointPositionLimits.py |
    | impact joint velocity limits (16)  |manipulatorConstraints/robustJointVelocityConstraints.py |
    | impulse torque constraint (18) | manipulatorConstraints/impulseTorqueLimitConstraints.py |
    | force-aware torque constraint (19) |manipulatorConstraints/impulseTorqueLimitConstraints.py |
    | impact dynamics equation (21) |  manipulatorConstraints/impactConstraints.py |
    | impulsive force maximization (23) |  manipulatorTasks/maxImpactForceTask.py |
    | Generic force control task |  manipulatorTasks/admittanceTask.py |

### 3. Figure re-generation:
  
   We save the simulation data as "*.npz" files. If one wants to re-generate the figures, we can check the [plotting commands](doc/plotting_commands.md)
   





