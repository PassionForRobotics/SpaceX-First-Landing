#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>

#include <rocket/ThrustCmd.h>
#include <gazebo/common/PID.hh>
#include <gazebo/physics/physics.hh>


#define PID_MAX_IN (1000000)
#define PID_MIN_IN (-1000000)
#define PID_MAX_OUT (1000000)
#define PID_MIN_OUT (-1000000)

static nav_msgs::Odometry g_odom;
static geometry_msgs::Twist g_tw, g_imu;
static geometry_msgs::Point g_pid_gains, g_pid_setpoints;
static double g_error;

gazebo::common::PID g_pid;


void odom_cb(nav_msgs::Odometry _odom)
{
  g_odom = _odom;
}

void imu_cb(geometry_msgs::Twist _imu)
{
  g_imu = _imu;
}


void twist_cb(geometry_msgs::Twist _tw)
{
  g_tw = _tw;
}

void pid_gains_cb(geometry_msgs::Point _pid_g)
{
  g_pid_gains = _pid_g;
  g_pid.Reset();
  g_pid.Init(g_pid_gains.x, g_pid_gains.y, g_pid_gains.z, PID_MAX_IN, PID_MIN_IN, PID_MAX_OUT, PID_MIN_IN);

}

void pid_setpoints_cb(geometry_msgs::Point _pid_sp)
{
  g_pid_setpoints = _pid_sp;

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_thrust");
  ros::NodeHandle nh;

  double dt;
  gazebo::common::Time current_time = gazebo::common::Time( ros::Time::now().toSec() );// gazebo::physics::get_world("default")->SimTime();
  gazebo::common::Time last_time = current_time;//gazebo::physics::get_world("default")->SimTime();

  ros::Publisher p = nh.advertise<rocket::ThrustCmd>("multi_thrust_commander_node", 2, false);
  ros::Subscriber twist = nh.subscribe("twist", 10, &twist_cb);
  ros::Subscriber odom = nh.subscribe("odom", 10, &odom_cb);
  ros::Subscriber imu = nh.subscribe("imu", 10, &imu_cb);

  ros::Subscriber pid_gains = nh.subscribe("pid_gains", 10, &pid_gains_cb);
  ros::Subscriber pid_setpoints = nh.subscribe("pid_setpoints", 10, &pid_setpoints_cb);


  g_pid.Init(g_pid_gains.x, g_pid_gains.y, g_pid_gains.z, PID_MAX_IN, PID_MIN_IN, PID_MAX_OUT, PID_MIN_IN);
  g_pid.SetCmd(0.0);

  rocket::ThrustCmd t;
  t.thrusts.data.push_back(0.0);
  t.thrusts.data.push_back(0.0);
  t.thrusts.data.push_back(0.0);


  while(ros::ok())
  {
    dt =  ( gazebo::common::Time( ros::Time::now().toSec() ) - last_time ).Double();
    last_time = current_time;

    //g_pid.Update(g_odom.pose.pose.position.z-g_pid_setpoints.z, dt);
    g_pid.Update(g_tw.linear.z-g_pid_setpoints.z, dt);

    t.thrusts.data[2] = g_pid.GetCmd();
    p.publish(t);
    ros::spinOnce();

    double pe, ie, de;
    g_pid.GetErrors(pe, ie, de);
    ROS_INFO_THROTTLE(0.1, "PID %f, %f, (%f,  %f,  %f)", g_pid.GetCmd(), g_odom.pose.pose.position.z-g_pid_setpoints.z, pe, ie, de);
  }
  ROS_INFO("published");
}
