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


$rostopic pub  /rocket_thrust_commander_node rocket/ThrustCommand "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
thrust: 1000.0"
