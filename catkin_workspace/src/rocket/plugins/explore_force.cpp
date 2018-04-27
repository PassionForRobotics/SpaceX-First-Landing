#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ros/ros.h>
#include <rocket/ThrustCommand.h>


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
  class forceexplr : public ModelPlugin
  { 
    private: void thrust_command_data_cb(const rocket::ThrustCommand::ConstPtr& _thrust_msg)
    {
      this->thrust_msg = *_thrust_msg;
      this->addaccel   = this->thrust_msg.thrust;
      //ROS_INFO("msg received!");
    }

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;
       
      this->link = this->model->GetChildLink("mini_rocket_thruster_40cms::link_0");
             
      addaccel = 0.0; //4.9
      
      const int ros_init_type = 0 ;
      int argc=0; char** argv=NULL;
      //ros::init(argc, argv, "rocket_THRUSTER_node", ros::init_options::AnonymousName); 
      //ros::init_options::AnonymousName // look for auto number gen suffix

      ROS_INFO("Ros node Starting up...");
     
      //ros::NodeHandle nh;
      
      //ROS_INFO("%s : %s", "THRUSTER", nh_.resolveName("rocket_THRUSTER_node").c_str());
      
      thrust_command_sub = this->nh_.subscribe("rocket_thrust_commander_node", 2000, &forceexplr::thrust_command_data_cb, this); 
      
      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&forceexplr::OnUpdate, this));

      if(ros::ok())
      {
        ROS_INFO("THRUSTER LOADED!!");
      }
      else
      {
       ROS_INFO("THRUSTER NOT LOADED!!");
      }
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      ros::spinOnce();
      //ROS_INFO_THROTTLE(5,"THRUSTER UPDATE!!");

      if(NULL!=link)
      {      
	//ROS_DEBUG_THROTTLE(5,"CONTINUOUS THRUST IS BEING APPLIED!!");
        this->link->AddRelativeForce(ignition::math::Vector3d(0, 0
            , /* this->link->GetInertial()->GetMass()* */ addaccel)); // continuous 18 doesn't 20 flies
        //addaccel = 0.0; // non-lacthed  // set a param for <latch> true or false 
      }


 
    }

    // Pointer to the model
    private: physics::ModelPtr model;
    private: gazebo::physics::LinkPtr link; 
    private: double addaccel; // value to given by external callers

    private: rocket::ThrustCommand thrust_msg;
    private: ros::NodeHandle nh_;
    private: ros::Subscriber thrust_command_sub;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(forceexplr)
}
