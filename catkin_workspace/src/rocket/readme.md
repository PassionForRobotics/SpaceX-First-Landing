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
