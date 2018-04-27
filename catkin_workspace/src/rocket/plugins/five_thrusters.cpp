#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ros/ros.h>
#include <rocket/FiveThrustCommand.h>


#include <boost/bind.hpp>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <stdio.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

namespace gazebo
{
  class fivethrusters : public ModelPlugin
  { 
    private: void five_thrusts_command_data_cb(const rocket::FiveThrustCommand::ConstPtr& _five_thrusts_msg)
    {
      this->five_thrusts_msg = *_five_thrusts_msg;
      this->addaccel_center = this->five_thrusts_msg.thrust_center;
      this->addaccel_side_1 = this->five_thrusts_msg.thrust_side_1;
      this->addaccel_side_2 = this->five_thrusts_msg.thrust_side_2;
      this->addaccel_side_3 = this->five_thrusts_msg.thrust_side_3;
      this->addaccel_side_4 = this->five_thrusts_msg.thrust_side_4;   
      ROS_INFO("msg received!");
    }

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf_plugin_params)
    {
      // Store the pointer to the model
      this->model = _parent;

      if( // _sdf->HasElement("right_joint")
	   _sdf_plugin_params->HasElement("center_thruster_link")
	&& _sdf_plugin_params->HasElement("side_thruster_1_link")
	&& _sdf_plugin_params->HasElement("side_thruster_2_link")
	&& _sdf_plugin_params->HasElement("side_thruster_3_link")
	&& _sdf_plugin_params->HasElement("side_thruster_4_link")
        )
      {

	      this->center_link = this->model->GetLink(
	      _sdf_plugin_params->GetElement("center_thruster_link")
	      ->Get<std::string>());

	      this->side_1_link = this->model->GetLink(
	      _sdf_plugin_params->GetElement("side_thruster_1_link")
	      ->Get<std::string>());

	      this->side_2_link = this->model->GetLink(
	      _sdf_plugin_params->GetElement("side_thruster_2_link")
	      ->Get<std::string>());

	      this->side_3_link = this->model->GetLink(
	      _sdf_plugin_params->GetElement("side_thruster_3_link")
	      ->Get<std::string>());

	      this->side_4_link = this->model->GetLink(
	      _sdf_plugin_params->GetElement("side_thruster_4_link")
	      ->Get<std::string>());

              ROS_INFO("Links found!"); // also check for link variable for value null
 
      }      
      else
      {
	      ROS_ERROR("Incomplete link info provided.");	
	      return;
      }

      this->addaccel_center = 0.0;
      this->addaccel_side_1 = 0.0;
      this->addaccel_side_2 = 0.0;
      this->addaccel_side_3 = 0.0;
      this->addaccel_side_4 = 0.0;   

      int argc=0; char** argv=NULL;
      //ros::init(argc, argv, "rocket_THRUSTER_node", ros::init_options::AnonymousName); 
      //ros::init_options::AnonymousName // look for auto number gen suffix

      ROS_INFO("Ros node Starting up...");
     
      //ros::NodeHandle nh;
      
      //ROS_INFO("%s : %s", "THRUSTER", nh_.resolveName("rocket_THRUSTER_node").c_str());
      
      five_thrusts_command_sub = this->nh_.subscribe("rocket_five_thrusts_commander_node", 2000, &fivethrusters::five_thrusts_command_data_cb, this); 
      
      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&fivethrusters::OnUpdate, this));

      if(ros::ok())
      {
        ROS_INFO("5 THRUSTERS LOADED!!");
      }
      else
      {
       ROS_INFO("THRUSTER(s) NOT LOADED!!");
      }
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      ros::spinOnce();
      ROS_INFO_THROTTLE(5,"THRUSTER UPDATE!!");

      if( (NULL!=center_link) && (NULL!=side_1_link) && (NULL!=side_2_link) && (NULL!=side_3_link) && (NULL!=side_4_link) )
      {      
	ROS_DEBUG_THROTTLE(5,"CONTINUOUS THRUSTS ARE BEING APPLIED!!");
	this->center_link->AddRelativeForce(ignition::math::Vector3d(0, 0
	    , /* this->center_link->GetInertial()->GetMass()* */ addaccel_center)); 

	this->side_1_link->AddRelativeForce(ignition::math::Vector3d(0, 0
	    , /* this->side_1_link->GetInertial()->GetMass()* */ addaccel_side_1));

	this->side_2_link->AddRelativeForce(ignition::math::Vector3d(0, 0
	    , /* this->side_2_link->GetInertial()->GetMass()* */ addaccel_side_2));

	this->side_3_link->AddRelativeForce(ignition::math::Vector3d(0, 0
	    , /* this->side_3_link->GetInertial()->GetMass()* */ addaccel_side_3));

	this->side_4_link->AddRelativeForce(ignition::math::Vector3d(0, 0
	    , /* this->side_4_link->GetInertial()->GetMass()* */ addaccel_side_4));


      }
 
    }

    // Pointer to the model
    private: physics::ModelPtr model;
    private: gazebo::physics::LinkPtr center_link;
    private: gazebo::physics::LinkPtr side_1_link;
    private: gazebo::physics::LinkPtr side_2_link;
    private: gazebo::physics::LinkPtr side_3_link;
    private: gazebo::physics::LinkPtr side_4_link;

    private: double addaccel_center, addaccel_side_1, addaccel_side_2, addaccel_side_3, addaccel_side_4; 
    // value to given by external callers

    private: rocket::FiveThrustCommand five_thrusts_msg;
    private: ros::NodeHandle nh_;
    private: ros::Subscriber five_thrusts_command_sub;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(fivethrusters)
}
