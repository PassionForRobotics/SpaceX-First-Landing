#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ros/ros.h>
#include <rocket/FiveThrustCommand.h>
#include <atom_esp_joy/joydata.h>

#include <boost/bind.hpp>
#include <gazebo/common/common.hh>
#include <gazebo/common/PID.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/ModelStates.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <stdio.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <geometry_msgs/Vector3.h> 


using gazebo::common::PID;
using gazebo::common::Time;

/*
Only for reference

    private: void joystate_cb(const atom_esp_joy::joydata::ConstPtr& _joy_data_msg)
    {
	this->joy_data = *_joy_data_msg;
        this->addaccel_center = this->joy_data.S;
        
      this->addaccel_side_1 = this->addaccel_center - this->joy_data.X;
      this->addaccel_side_2 = this->addaccel_center + this->joy_data.X;
      this->addaccel_side_3 = this->addaccel_center - this->joy_data.Y;
      this->addaccel_side_4 = this->addaccel_center + this->joy_data.Y;   
    
	ROS_INFO_THROTTLE(5,"Joy msg received! %f %f", (float)this->joy_data.S, this->addaccel_center);
    }

*/

geometry_msgs::Vector3 g_target_pos;//(0,0,0.2);
nav_msgs::Odometry g_input_odom;
geometry_msgs::Vector3 g_pid_param; //(500,0,0);
double intError = 0; // PID interative error

void calculateReqThrust(const nav_msgs::Odometry *odom, const geometry_msgs::Vector3 *targetPos, rocket::FiveThrustCommand *reqThrust)
{
	// probably will need PID trying with P only
        static gazebo::common::PID pid(5,0,0,10, 0.1, 500, 10 );

        static rocket::FiveThrustCommand lastReqThrust;

        float center_thrust,
	right_plus_thrust, // opposite has to be applied
	right_minus_thrust,
        front_plus_thrust,
	front_minus_thrust;

        float right_plus_pos  = odom->pose.pose.position.x;
        float front_plus_pos  = odom->pose.pose.position.y;
        float center_plus_pos = odom->pose.pose.position.z;

        float right_plus_target  = targetPos->x;
        float front_plus_target  = targetPos->y;
        float center_plus_target = targetPos->z;
	//gazebo::common::Time
	static ros::Time last_time;	
	static float errorPlast;
	ros::Time curr_time = ros::Time::now();

	//static gazebo::common::Time last_time;//
        //gazebo::common::Time cur_time(curr_time.sec,curr_time.nsec);
               

        //lastReqThrust.thrust_center = pid.Update(
	// center_plus_target-center_plus_pos,
	// cur_time-last_time
        //);

	float errorP = center_plus_target - center_plus_pos;
        
	float dt = (curr_time-last_time).toSec();
	float errorPder = (errorP - errorPlast)/dt;
	
	//static double intError; 
        intError += errorP*dt; 

	lastReqThrust.thrust_center = errorP*g_pid_param.x + errorPder*g_pid_param.y + intError*g_pid_param.z;

        lastReqThrust.thrust_center = lastReqThrust.thrust_center > 0 ? lastReqThrust.thrust_center : 0 ;

	center_thrust = lastReqThrust.thrust_center; 
	
        *reqThrust = lastReqThrust;
	last_time = curr_time;
	errorPlast = errorP;

	
/*
        ros::Time curr_time = ros::Time::now();
	//nav_msgs::Odometry odom;
	//odom->header.stamp = curr_time;
	//odom->header.frame_id = "odom";

	// set the position
	odom->pose.pose.position.x, y, z
	odom->pose.pose.orientation.x , y, z, w 
	// set the velocity
	//odom->child_frame_id = this->model->GetName();
	// set the twists
	odom->twist.twist.linear.x, y, z 
        odom->twist.twist.angular.x, y, z 
*/
}

void target_pos_cb(const geometry_msgs::Vector3 _target_pos)
{
	g_target_pos = _target_pos;
        ROS_INFO("target received");
}


void pidparam_cb(const geometry_msgs::Vector3 _pid_param)
{
	g_pid_param = _pid_param;
	intError = 0;
        ROS_INFO("pid params received");
}

void odom_cb(const nav_msgs::Odometry _input_odom)
{

	g_input_odom = _input_odom;
	ROS_INFO_THROTTLE(10, "odom received");

}

int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "rocket_stabilization_node");
  ROS_INFO("stabilization control -> starting up...");

  g_target_pos.x = 0;
  g_target_pos.y = 0;
  g_target_pos.z = 0.2;

  g_pid_param.x = 500;
  g_pid_param.y = 0;
  g_pid_param.z = 0;

  ros::NodeHandle nh_;
  
  ros::Rate loop_rate(100);
 
  ros::Subscriber target_sub = nh_.subscribe("rocket_target_pos", 100, target_pos_cb); 
 
  ros::Subscriber odom_sub = nh_.subscribe("/odom", 100, odom_cb); 
  
  ros::Subscriber pidparam_sub = nh_.subscribe("pid_param", 100, pidparam_cb); 

         
  ros::Publisher rocket_5_thrusts_pub = nh_.advertise<rocket::FiveThrustCommand>("rocket_stabilization_control", 100);

  // setRightForce(center+(X))
  // setFrontForce(center+(Y))
  // setAltitudeForce(center) 
  
  while(ros::ok())
  {
    	static rocket::FiveThrustCommand five_thrusts;
        calculateReqThrust(&g_input_odom, &g_target_pos, &five_thrusts);
	rocket_5_thrusts_pub.publish(five_thrusts);

	ros::spinOnce();
	
        loop_rate.sleep();

  }

}



