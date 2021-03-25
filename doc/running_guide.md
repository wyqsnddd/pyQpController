Once the DART simulator and its python interface [pydart2](https://pydart2.readthedocs.io/en/latest/install.html) is correctly configured, we can use the following commands to run the simulations: 

  #### 1. Collide with an unknow wall:
- Enter the main containing [impact_one.py](@ref impact_one.py) and run the following: 

```sh
$ python impact_one.py
```
This will bring up the GUI. Please hit space to run. If you would like to tweak the parameters, the corresponding configuration file is "config/impact_one.json"

This task would use a position task, an orientation task and a velocity task to collide an unknown wall with the maximal speed. The current configuration runs with the proposed QP controller. If you want to try a generic QP controller, you can open the configuration file "config/impact_one.json" and toggle the following: [qpController][impactRobust] from "true" to "false". 

  #### 2. Impact stabilization
- Enter the main directory [hybrid_impact_one.py](@ref hybrid_impact_one.py) and run the following: 
```sh
$ python hybrid_impact_one.py
```
This will bring up a similar GUI. Please hit space to run. If you would like to tweak the parameters, the corresponding configuration file is "config/hybrid_impact_controller_one.json" 




The difference compared to "colliding with an unknown wall" is that we will stabilize the contact force upon the detection of the impact. Thus along the contact surface normal direction, we will replace the velocity task with a task-space force control task. 

We can tweak the post-impact force control task parameters in the configuration file under the "admittance task" block. 

  #### 3. Impulsive force maximization
  In the configuration file, either "config/impact_one.json" or "config/hybrid_impact_controller_one.json", we can toggle the parameter: [qpController][maxImpactForceTask][enabled] between true and false to observe the difference. 
  
  Then we can either run "Collide with an unknow wall" or "Impact stabilization" with the maximal impulsive force. 
  #### 4. Push box with impact:  
  - Enter the main directory [sliding_box_admittance_impact.py](@ref sliding_box_admittance_impact.py) and run the following: 
```sh
$ python sliding_box_admittance_impact.py
```

The configuration file is: "config/sliding_box_admittance_impact.json"
