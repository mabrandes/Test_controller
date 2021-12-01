# Test_controller


## Test Clcontoller
Is just a node subscribing to the model states of the smb_gazebo model and to the pose of the state estimation from the Rosbag in the garage
```
roslaunch clcontroller clc.launch
```

## Test_Transformation
Subscribes to model states of rowesys simulation in gazebo.
```
rosrun transfrom_quad quad_to_euler.py 
```
Prerequisits: 
- run:  
``` 
roslaunch rowesys_base_gazebo gazebo.launch
roslaunch rowesys_control single_highlevel_controller.launch controller_name:='rowesys_swerve_steering_controller' controller_param_path:='$(find rowesys_control)/config/swerve_steering.yaml'
```

can be found in rowesys workspace (catkin_ws_rowesys). It turns on the simulation and the swerve controller.
with 
```
rostopic pub -r 10 /rowesys/rowesys_swerve_steering_controller/cmd_vel geometry_msgs/Twist  "{linear:  {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0, z: 0.0}}"
```
one can move the robot



# Debugging Error: 
"ImportError: dynamic module does not define module export function (PyInit__tf2)"

Solution:
```
mkdir -p ~/catkin_ws/src; cd ~/catkin_ws
catkin_make
source devel/setup.bash
wstool init
wstool set -y src/geometry2 --git https://github.com/ros/geometry2 -v 0.6.5
wstool up
rosdep install --from-paths src --ignore-src -y -r

catkin build --cmake-args \
            -DCMAKE_BUILD_TYPE=Release \
            -DPYTHON_EXECUTABLE=/usr/bin/python3 \
            -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m \
            -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
            
```
