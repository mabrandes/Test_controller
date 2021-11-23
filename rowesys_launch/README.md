# rowesys_launch

## Overview

This package contains the launch files needed to operate the robot.

### License

**Author:** Timo Schönegg <br />
**Project:** Focusproject ROWESYS, ETH Zürich <br />
**Created:** May 2020

The rowesys_launch package has been tested under ROS Melodic and Ubuntu 18.04.


## Launch files

* **manual_drive.launch:** Launches the real_robot launch file from the rowesys_base repository to control the motors. Launches the gamepad including sounds and lights for manual operation.

* **autonomous_navigation.launch:** Launches the simple state machine, the state estimator, the geese feet planner including the normal planner, the turn, the line detection and the end of row detection. This launch file launches all the additionally necessary packages for autonomous navigation.

* **autonomous_navigation_gf.launch:** Same as autonomous_navigation.launch, but it launches the geese feet state machine.

* **rowesys_all.launch:** Launches manual_drive.launch and autonomous_navigation.launch. This is the standard launch file.

* **rowesys_all_gf.launch:** Launches manual_drive.launch and autonomous_navigation_gf.launch.

* **manual_line_detection.launch:** Launches manual_drive.launch, the state estimator and the line detection. This launch file is just for testing reasons.

* **manual_state_estimator.launch:** Launches manual_drive.launch and the state estimator.