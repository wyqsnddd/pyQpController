
# <center>Impact-friendly QP controller</center>


 
|[![Impact-friendly QP controller](https://img.youtube.com/vi/ZlPM9PbJnnY/0.jpg)](https://www.youtube.com/watch?v=ZlPM9PbJnnY)  |  [![Impact-aware QP controller](http://img.youtube.com/vi/K9ar8tsPN8s/0.jpg)](http://www.youtube.com/watch?v=K9ar8tsPN8s "Video Title")   |
| ------------- | :-------------: |
 
pyQpController is the proof of concept simulator attached to the paper: [Impact-Friendly Robust Control Design with Task-Space Quadratic Optimization](https://pdfs.semanticscholar.org/2d37/f86c6c1ff63986cc1f9d6bc7d1bfac1274ed.pdf?_ga=2.217802134.1089995681.1567288698-1926356895.1556608070) pyQpController is based on the [python interface](https://github.com/sehoonha/pydart2) of the [DART simulator](https://dartsim.github.io/). 

We explicitly introduce discrete impact dynamics model into the QP-based controllers to generate robot motions that are robust to impact-induced state jumps in the joint velocities and joint torques. Our simulations, validate that our proposed impact-friendly QP controller is robust to contact impacts, shall they be expected or not. Therefore, we can exploit it for establishing contacts with high velocities, and explicitly generate task-purpose impulsive forces.

Recently we extended the concept for floating-based robot, i.e. a HRP4 humanoid robot, where we successfully applied impact at 0.35 m/s and achieved impulsive force of 133 N without breaking the hardware or losing balance. The C++ implementation is available at [mc_impact_pusher](https://github.com/wyqsnddd/mc_impact_pusher).




 ### 1. Running guide: 
We can find a quick start from the  [Running guide](doc/running_guide.md)

 ### 2. Implementation: 
 
Interested readers can find the implementation details from the following python files. Basically each inequality constraint "G x <= h " would output the G and h matrices. 

The similar results could be found for the equality constraint (21). Each task in the folder "manipulatorTasks" outputs Q and P matrices expanded from the 2-norm with respect to the decision variables. 


|Equations in the paper  | Simulation code |
| ------------- | :-------------: |
| Joint position limits (6) | [jointPositionLimits.py]( manipulatorConstraints/jointPositionLimits.py) |
| Joint velocity limits (7)  |  [jointVelocityConstraints.py]( manipulatorConstraints/jointVelocityConstraints.py)|
| Joint torque limits (8)  |  [torqueLimitConstraints.py](manipulatorConstraints/torqueLimitConstraints.py)|
| impact joint position limits (15) |  [robustJointPositionLimits.py](manipulatorConstraints/robustJointPositionLimits.py) |
| impact joint velocity limits (16)  |[robustJointVelocityConstraints.py](manipulatorConstraints/robustJointVelocityConstraints.py)|
| impulse torque constraint (18) | [impulseTorqueLimitConstraints.py](manipulatorConstraints/impulseTorqueLimitConstraints.py) |
| force-aware torque constraint (19) |[torqueLimitConstraints.py](manipulatorConstraints/torqueLimitConstraints.py) |
| impact dynamics equation (21) |  [impactConstraints.py](manipulatorConstraints/impactConstraints.py) |
| impulsive force maximization (23) |  [maxImpactForceTask.py](manipulatorTasks/maxImpactForceTask.py) |
| Generic force control task |  [admittanceTask.py](manipulatorTasks/admittanceTask.py) |

### 3. Figure re-generation:
  
   We save the simulation data as "*.npz" files. If one wants to re-generate the figures, we can check the [plotting commands](doc/plotting_commands.md).

