# Plan Runner

This module contains scripts that are useful for running simulations and sending commands to the real robot. 

## Files
- `manipulation_station_plan_runner.py`: `ManipStationPlanRunner` is a Drake leaf system that evaluates a list of `Plan`s and sends commands to the robot. 
- `manipulation_station_simulator.py`: `ManipulationStationSimulator` constructs a Drake diagram system in which a `ManipStationPlanRunner` is connected to either a simulator ([`ManipulationStation`](https://drake.mit.edu/doxygen_cxx/classdrake_1_1examples_1_1manipulation__station_1_1_manipulation_station.html)) or an interface to the robot drivers ([`ManipulationStationHardwareInterface`](https://drake.mit.edu/doxygen_cxx/classdrake_1_1examples_1_1manipulation__station_1_1_manipulation_station_hardware_interface.html)). It can be used for simulating `Plan`s or executing `Plan`s on real hardware.
- `robot_plans.py` contains implementation of various plans. A plan is a class that contains function or data members that can be evaluated to generate position and/or torque command for a robot. 
- `open_left_door_plans.py` contains plan types specifically designed to open the left door of the cabinet in `ManipulationStation`. 
- `open_left_door.py` contains utility methods that facilitate opening the left door. 

## Running an example
`run_open_left_door.py` is an "executable" that executes different combinations of open-left-door procedures based on command line arguments. For example, 
```bash
$ python run_open_left_door.py --controller=Trajectory 
```
simulates opening the door by following trajectories, and
```bash
$ python run_open_left_door.py --controller=Impedance --hardware 
```
sends position and torque commands generated by the impedance controller to the real robot.

For more details, please refer to the definitions of command line arguments in `run_open_left_door.py`.

## Meshcat
Scripts running simulation will hang if no meshcat server is running. Run the following command in a terminal to open a meshcat server:
```bash
$ meshcat-server &
``` 