#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ros/ros.h>

namespace gazebo
{
  class forceexplr : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;
       
      this->link = this->model->GetChildLink("mini_rocket_thruster_40cms::link_0");
       
      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&forceexplr::OnUpdate, this));
      
      addaccel = 0.0; //4.9
      ROS_INFO("THRUSTER LOADED!!");
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      
      if(NULL!=link)
      {      
	//ROS_DEBUG_THROTTLE(5,"CONTINUOUS THRUST IS BEING APPLIED!!");
        this->link->AddRelativeForce(ignition::math::Vector3d(0, 0
            , this->link->GetInertial()->GetMass()*addaccel)); // continuous 18 doesn't 20 flies
        addaccel = 0.0; // non-lacthed  // set a param for <latch> true or false 
      }
 
    }

    // Pointer to the model
    private: physics::ModelPtr model;
    private: gazebo::physics::LinkPtr link; 
    private: double addaccel; // value to given by external callers

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(forceexplr)
}
