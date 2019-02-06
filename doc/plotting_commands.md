* Generate figures from existing data file: 
   
```sh
        $ python utils/impact_plotter.py ./submitted-data/proposed-QP/max_contact_velocity/jointVelocityJump-data_Jan_31_2019_22-44-36.npz
```

        or

```sh
        $ python  utils/impact_plotter_no_force.py submitted-data/generic-QP/four_impacts_simulation_data.npz
```

* Generate figures of new simulations: 

```sh
       $ python utils/impact_plotter.py log/data/jointVelocityJump-data_Feb_05_2019_02-02-45.npz
```

        where we need to replace the corresponding date and time.

