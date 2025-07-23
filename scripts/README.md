# Hydra Calibration

[![license - apache 2.0](https://img.shields.io/:license-Apache%202.0-yellowgreen.svg)](https://opensource.org/licenses/Apache-2.0)

This repository contain scripts for calibrating the
[Hydra](https://github.com/alexarbogast/hydra_ros) multi-robot system at Georgia
Tech's [Advanced Manufacturing Pilot Facility
(AMPF)](https://ampf.research.gatech.edu/).
The scripts move the robots to the positions required for data collection. 
See the
[multi_robot_calibration](https://github.com/Georgia-Tech-Manufacturing-Institute/multi_robot_calibration.git)
package for instructions on how to collect and use the probed positions to find
the base-to-base transformations between robots.

## Usage

There are two scripts: the first is used to collect data for calibration, and
the second is used to validate the calibrated parameters. A MoveIt planning
server should be running before launching both scripts. Bringup the robots and
MoveIt planning according to the instruction in
[`hydra_ros`](https://github.com/alexarbogast/hydra_ros).

#### `calibration_routine`

```
rosrun hydra_calibration calibration_routine.py
```

This script uses MoveIt to move each robot to eight identical poses in a cube.
End-effector positions should be probed at each location. The non-active arm is
positioned in a vertical configuration to avoid potential collisions.

#### `check_calibration`

```
rosrun hydra_calibration check_calibration.py
```

This script moves each robot to face one another and then incrementally
positions them closer together until the gap between the flanges is 1mm.

> [!WARNING]
> Do not run this script until the system has been calibrated
