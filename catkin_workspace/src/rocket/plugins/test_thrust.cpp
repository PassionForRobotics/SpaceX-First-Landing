#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>

#include <rocket/ThrustCmd.h>
#include <atom_esp_joy/joydata.h>

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

#define X_CTRL (0)
#define Y_CTRL (1)
#define Z_CTRL (2)

gazebo::common::PID g_pid[3];


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

  g_pid[X_CTRL].Reset();
  g_pid[X_CTRL].Init(g_pid_gains.x, g_pid_gains.y, g_pid_gains.z, PID_MAX_IN, PID_MIN_IN, PID_MAX_OUT, PID_MIN_IN);
  g_pid[Y_CTRL].Reset();
  g_pid[Y_CTRL].Init(g_pid_gains.x, g_pid_gains.y, g_pid_gains.z, PID_MAX_IN, PID_MIN_IN, PID_MAX_OUT, PID_MIN_IN);
  g_pid[Z_CTRL].Reset();
  g_pid[Z_CTRL].Init(g_pid_gains.x, g_pid_gains.y, g_pid_gains.z, PID_MAX_IN, PID_MIN_IN, PID_MAX_OUT, PID_MIN_IN);

}

void pid_setpoints_cb(geometry_msgs::Point _pid_sp)
{
  g_pid_setpoints = _pid_sp;
}

void atom_joy_ctrl(atom_esp_joy::joydata joy)
{
  g_pid_setpoints.z = joy.S;
  g_pid_setpoints.x = joy.X;
  g_pid_setpoints.y = joy.Y;

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

  ros::Subscriber atom_joy = nh.subscribe("atom_joydata", 1000, &atom_joy_ctrl);

  g_pid_gains.x = 0.5;
  g_pid_gains.y = -20;


  g_pid[X_CTRL].Init(g_pid_gains.x, g_pid_gains.y, g_pid_gains.z, PID_MAX_IN, PID_MIN_IN, PID_MAX_OUT, PID_MIN_IN);
  g_pid[X_CTRL].SetCmd(0.0);

  g_pid[Y_CTRL].Init(g_pid_gains.x, g_pid_gains.y, g_pid_gains.z, PID_MAX_IN, PID_MIN_IN, PID_MAX_OUT, PID_MIN_IN);
  g_pid[Y_CTRL].SetCmd(0.0);

  g_pid[Z_CTRL].Init(g_pid_gains.x, g_pid_gains.y, g_pid_gains.z, PID_MAX_IN, PID_MIN_IN, PID_MAX_OUT, PID_MIN_IN);
  g_pid[Z_CTRL].SetCmd(0.0);

  rocket::ThrustCmd t;
  t.thrusts.data.push_back(0.0);
  t.thrusts.data.push_back(0.0);
  t.thrusts.data.push_back(0.0);

//  double LOOKAHEAD = g_pid_gains.x; //seconds
//  double SENSITIVITY =g_pid_gains.y;
//  ignition::math::Vector3d YOUR_TARGET_HERE(g_pid_setpoints.x, g_pid_setpoints.y, g_pid_setpoints.z);

//  ignition::math::Vector3d  curVel(g_tw.linear.x, g_tw.linear.y, g_tw.linear.z);
//  ignition::math::Vector3d  curPos(g_odom.pose.pose.position.x, g_odom.pose.pose.position.y, g_odom.pose.pose.position.z);
//  ignition::math::Vector3d  futurePos = curPos + LOOKAHEAD * curVel;
//  ignition::math::Vector3d  desiredPos = YOUR_TARGET_HERE;
//  ignition::math::Vector3d  applyForce = (futurePos - desiredPos) * SENSITIVITY;
  //  clampLength( applyForce, MAX_FORCE );
  //  dBodyAddForce( b, applyForce );


  while(ros::ok())
  {
    dt =  ( gazebo::common::Time( ros::Time::now().toSec() ) - last_time ).Double();
    last_time = current_time;

    g_pid[X_CTRL].Update(g_odom.pose.pose.position.x-g_pid_setpoints.x, dt);
    g_pid[Y_CTRL].Update(g_odom.pose.pose.position.y-g_pid_setpoints.y, dt);
    g_pid[Z_CTRL].Update(g_odom.pose.pose.position.z-g_pid_setpoints.z, dt);

    double LOOKAHEAD = g_pid_gains.x; //seconds
    double SENSITIVITY =g_pid_gains.y;
    ignition::math::Vector3d YOUR_TARGET_HERE(g_pid_setpoints.x, g_pid_setpoints.y, g_pid_setpoints.z);

    ignition::math::Vector3d  curVel(g_tw.linear.x, g_tw.linear.y, g_tw.linear.z);
    ignition::math::Vector3d  curPos(g_odom.pose.pose.position.x, g_odom.pose.pose.position.y, g_odom.pose.pose.position.z);
    ignition::math::Vector3d  futurePos = curPos + LOOKAHEAD * curVel;
    ignition::math::Vector3d  desiredPos = YOUR_TARGET_HERE;
    ignition::math::Vector3d  applyForce = (futurePos - desiredPos) * SENSITIVITY;

    //g_pid.Update(g_tw.linear.z-g_pid_setpoints.z, dt);

    t.thrusts.data[X_CTRL] = applyForce.X();// g_pid[X_CTRL].GetCmd();
    t.thrusts.data[Y_CTRL] = applyForce.Y();//g_pid[Y_CTRL].GetCmd();
    t.thrusts.data[Z_CTRL] = applyForce.Z();//g_pid[Z_CTRL].GetCmd();
    p.publish(t);
    ros::spinOnce();

    //double pe, ie, de;
    //g_pid.GetErrors(pe, ie, de);
    ROS_INFO_THROTTLE(0.1, "PID active");//%f, %f, (%f,  %f,  %f)", g_pid.GetCmd(), g_odom.pose.pose.position.z-g_pid_setpoints.z, pe, ie, de);
  }
  ROS_INFO("published");
}
