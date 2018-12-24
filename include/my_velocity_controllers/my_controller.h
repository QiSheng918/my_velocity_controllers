#ifndef MY_CONTROLLER_H
#define MY_CONTROLLER_H


#include <ros/node_handle.h>
#include <urdf/model.h>
#include <control_toolbox/pid.h>
#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>
#include <realtime_tools/realtime_publisher.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <control_msgs/JointControllerState.h>
#include <std_msgs/Float64.h>
#include <control_msgs/JointControllerState.h>
#include <realtime_tools/realtime_buffer.h>

namespace my_velocity_controllers
{

class MyController: public controller_interface::Controller<hardware_interface::VelocityJointInterface>
{
public:

  struct Commands
  {
    double position_; // Last commanded position
    double velocity_; // Last commanded velocity
    bool has_velocity_; // false if no velocity command has been specified
    bool has_position_;
  };

  MyController();
  ~MyController();


  bool init(hardware_interface::VelocityJointInterface *robot, ros::NodeHandle &n);
  void starting(const ros::Time& time);
  void update(const ros::Time& time, const ros::Duration& period);


  void getGains(double &p, double &i, double &d, double &i_max, double &i_min);
  void getGains(double &p, double &i, double &d, double &i_max, double &i_min, bool &antiwindup);
  void setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min, const bool &antiwindup = false);


  hardware_interface::JointHandle joint_;
  urdf::JointConstSharedPtr joint_urdf_;
  realtime_tools::RealtimeBuffer<Commands> command_;
  Commands command_struct_; 

private:
  int loop_count_;
  control_toolbox::Pid pid_controller_;     

  // boost::scoped_ptr<
  //   realtime_tools::RealtimePublisher<
  //     control_msgs::JointControllerState> > controller_state_publisher_ ;

  ros::Subscriber sub_position_command_;
  ros::Subscriber sub_velocity_command_;
  
  void setPositionCommandCB(const std_msgs::Float64ConstPtr& msg);

  void setVelocityCommandCB(const std_msgs::Float64ConstPtr& msg);


  void enforceJointLimits(double &command);

};
} 
#endif