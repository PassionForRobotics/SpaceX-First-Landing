 For simulating a single rocket mini booster

$ pwd

... catkin_workspace/

$ catkin_make

... ...
... ...
... ...

$ source source

$ roslaunch rocket force_test.launch

... gazebo should open 

... Note: allow world to have a model with a link


For single thruster

$rostopic pub  /rocket_thrust_commander_node rocket/ThrustCommand "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
thrust: 1000.0"


$ $ roslaunch rocket five_thrusters_test.launch 


... gazebo should open 

... Note: allow world to have a model with a link


For 5 thrusters 

$rostopic pub /rocket_five_thrusts_commander_node rocket/FiveThrustCommand "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
thrust_center: 200.0
thrust_side_1: 200.0
thrust_side_2: 200.0
thrust_side_3: 200.0
thrust_side_4: 200.0"


$ $ roslaunch rocket five_thrusters_imu_test.launch 

... gazebo should open 

... Note: allow world to have a model with a link


For odom

rostopic pub /rocket_stability_five_thrusts_commander_node rocket/FiveThrustCommand "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
thrust_center: 150.0
thrust_side_1: 0.0
thrust_side_2: 0.0
thrust_side_3: 0.0
thrust_side_4: 0.0" 

and 

rostopic echo /odom


Added another package from https://github.com/PassionForRobotics/ATOM_DRONE/tree/master/ros/rosatom/src/atom_esp_joy

Possible ssue: module 'libevdev' : $ sudo apt-get install libevdev-dev


