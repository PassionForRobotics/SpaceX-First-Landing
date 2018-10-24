#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ros/ros.h>
#include <rocket/ThrustCommand.h>
#include <rocket/ThrustCmd.h>


#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TwistStamped.h>

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
  class explrthrust : public ModelPlugin
  {

    private: void multi_thrust_command_data_cb(const rocket::ThrustCmd::ConstPtr& _m_thrust_msg)
    {
      const rocket::ThrustCmd t = *_m_thrust_msg;

      std::vector<double> v_t(t.thrusts.data);


      // + [0] [1] [2]
      // - [3] [4] [5]
      //this->thrust_msg = *_thrust_msg;
      //this->addaccel   = this->thrust_msg.thrust;
      ignition::math::Vector3d thrust(t.thrusts.data[0], t.thrusts.data[1], t.thrusts.data[2]);

      this->thrust_ = thrust;



      //ROS_INFO("msg received!");
    }

  private: void thrust_command_data_cb(const rocket::ThrustCommand::ConstPtr& _thrust_msg)
    {
      this->thrust_msg = *_thrust_msg;
      this->addaccel   = this->thrust_msg.thrust;
      //ROS_INFO("msg received!");
    }

  private: void twist_command_data_cb(const geometry_msgs::TwistStamped::ConstPtr& _twi_msg)
    {

      gazebo::common::Time now = this->world_->SimTime();
      ignition::math::Vector3d a = ignition::math::Vector3d(_twi_msg->twist.angular.x, _twi_msg->twist.angular.y, _twi_msg->twist.angular.z);
      ignition::math::Vector3d l = ignition::math::Vector3d(_twi_msg->twist.linear.x, _twi_msg->twist.linear.y, _twi_msg->twist.linear.z);

      double dt = (now - this->last_time_).sec;
      this->model->SetAngularVel(a);
      this->model->SetLinearVel(l*dt);


      this->last_time_ = now;//this->world_->SimTime();

      ROS_INFO_THROTTLE(1, "rec");
    }
    private: void orientation_command_data_cb(const geometry_msgs::Vector3Stamped::ConstPtr& _ori_msg)
    {
      //this->thrust_msg = *_thrust_msg;

      //_ori_msg->vector.x
     // ignition::math::Vector3d pos(this->model->), _ori_msg->vector.y, _ori_msg->vector.z);
     // ignition::math::Vector3d pos(_ori_msg->vector.x, _ori_msg->vector.y, _ori_msg->vector.z);
      //ignition::math::Pose3d pose;

      //this->model->SetWorldPose();

      //this->addaccel   = this->thrust_msg.thrust;
      //ROS_INFO("msg received!");
    }

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

      this->world_ = this->model->GetWorld();

      this->link = this->model->GetChildLink(this->model->GetName()+"::test_thrust::link_0");

      if(NULL==this->link)
      {
        ROS_ERROR("NO LINK FOUND!");
        gazebo::physics::Link_V l_v = this->model->GetLinks();

        for(int i = 0 ; i < l_v.size(); i++)
        {
          ROS_INFO(" %s",l_v[i]->GetScopedName().c_str());
        }

      }

      addaccel = 0.0; //4.9

      const int ros_init_type = 0 ;
      int argc=0; char** argv=NULL;
      //ros::init(argc, argv, "rocket_THRUSTER_node", ros::init_options::AnonymousName);
      //ros::init_options::AnonymousName // look for auto number gen suffix

      ROS_INFO("Ros node Starting up...");

      //ros::NodeHandle nh;

      //ROS_INFO("%s : %s", "THRUSTER", nh_.resolveName("rocket_THRUSTER_node").c_str());

      //thrust_command_sub = this->nh_.subscribe("rocket_thrust_commander_node", 2000, &explrthrust::thrust_command_data_cb, this);

      //orientation_command_sub = this->nh_.subscribe("fusedOrientation", 2000, &explrthrust::orientation_command_data_cb, this);

      //twist_command_sub_ = this->nh_.subscribe("Twist", 2000, &explrthrust::twist_command_data_cb, this);


      multi_thrust_command_sub_ = this->nh_.subscribe("multi_thrust_commander_node", 2000, &explrthrust::multi_thrust_command_data_cb, this);

      odom_ = this->nh_.advertise<nav_msgs::Odometry>("odom", 2000, false);
      twist_ = this->nh_.advertise<geometry_msgs::Twist>("twist", 2000, false);
      imu_ = this->nh_.advertise<geometry_msgs::Twist>("imu", 2000, false);


      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&explrthrust::OnUpdate, this));

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
      //ros::spinOnce();
      //ROS_INFO_THROTTLE(5,"THRUSTER UPDATE!!");

      if(NULL!=link)
      {

        this->link->SetForce(this->thrust_);

        ignition::math::Pose3d p = this->model->WorldPose();
        nav_msgs::Odometry odom;
        odom.pose.pose.position.x = p.Pos().X();
        odom.pose.pose.position.y = p.Pos().Y();
        odom.pose.pose.position.z = p.Pos().Z();
        odom.pose.pose.orientation.x = p.Rot().X();
        odom.pose.pose.orientation.y = p.Rot().Y();
        odom.pose.pose.orientation.z = p.Rot().Z();
        odom.pose.pose.orientation.w = p.Rot().W();


        ignition::math::Vector3d v = this->model->WorldLinearVel();
        ignition::math::Vector3d a = this->model->WorldLinearAccel();
        geometry_msgs::Twist t;
        t.linear.x = a.X();
        t.linear.y = a.Y();
        t.linear.z = a.Z();

        this->twist_.publish(t);




        this->odom_.publish(odom);
  //ROS_DEBUG_THROTTLE(5,"CONTINUOUS THRUST IS BEING APPLIED!!");
        //this->link->AddRelativeForce(ignition::math::Vector3d(0, 0
        //    , this->link->GetInertial()->Mass()*addaccel)); // continuous 18 doesn't 20 flies
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
     private: ros::Subscriber multi_thrust_command_sub_;
  private: ros::Subscriber orientation_command_sub;
  private: ros::Subscriber twist_command_sub_;
  private: gazebo::common::Time last_time_;
  private: gazebo::physics::WorldPtr world_;
    ros::Publisher twist_, imu_, odom_;
ignition::math::Vector3d thrust_;


    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(explrthrust)
}
