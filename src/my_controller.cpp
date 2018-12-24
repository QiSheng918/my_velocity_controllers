#include "../include/my_velocity_controllers/my_controller.h"
#include <angles/angles.h>
#include <pluginlib/class_list_macros.hpp>

namespace my_velocity_controllers {

MyController::MyController()
  : loop_count_(0)
{}

MyController::~MyController()
{
  sub_position_command_.shutdown();
  sub_velocity_command_.shutdown();
}

bool MyController::init(hardware_interface::VelocityJointInterface *robot, ros::NodeHandle &n)
{
  // Get joint name from parameter server
  std::string joint_name;
  if (!n.getParam("joint", joint_name))
  {
    ROS_ERROR("No joint given (namespace: %s)", n.getNamespace().c_str());
    return false;
  }

  // Load PID Controller using gains set on parameter server
  if (!pid_controller_.init(ros::NodeHandle(n, "pid")))
    return false;

  // Start realtime state publisher
  // controller_state_publisher_.reset(
  //   new realtime_tools::RealtimePublisher<control_msgs::JointControllerState>(n, "state", 1));

  // Start command subscriber
  sub_position_command_ = n.subscribe<std_msgs::Float64>("position_command", 1, &MyController::setPositionCommandCB, this);

  sub_velocity_command_ = n.subscribe<std_msgs::Float64>("velocity_command", 1, &MyController::setVelocityCommandCB, this);
  // Get joint handle from hardware interface
  joint_ = robot->getHandle(joint_name);

  // Get URDF info about joint
  urdf::Model urdf;
  if (!urdf.initParamWithNodeHandle("robot_description", n))
  {
    ROS_ERROR("Failed to parse urdf file");
    return false;
  }
  joint_urdf_ = urdf.getJoint(joint_name);
  if (!joint_urdf_)
  {
    ROS_ERROR("Could not find joint '%s' in urdf", joint_name.c_str());
    return false;
  }

  return true;
}

void MyController::setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min, const bool &antiwindup)
{
  pid_controller_.setGains(p,i,d,i_max,i_min,antiwindup);
}

void MyController::getGains(double &p, double &i, double &d, double &i_max, double &i_min)
{
  bool dummy;
  pid_controller_.getGains(p,i,d,i_max,i_min,dummy);
}

void MyController::getGains(double &p, double &i, double &d, double &i_max, double &i_min, bool &antiwindup)
{
  pid_controller_.getGains(p,i,d,i_max,i_min,antiwindup);
}

void MyController::starting(const ros::Time& time)
{
  double pos_command = joint_.getPosition();

  // Make sure joint is within limits if applicable
  enforceJointLimits(pos_command);

  command_struct_.position_ = pos_command;
  command_struct_.has_velocity_ = false;
  command_struct_.has_position_=true;

  command_.initRT(command_struct_);

  pid_controller_.reset();
}

void MyController::update(const ros::Time& time, const ros::Duration& period)
{
  command_struct_ = *(command_.readFromRT());
  double command_position = command_struct_.position_;
  double command_velocity1 = command_struct_.velocity_;
  bool has_velocity_ =  command_struct_.has_velocity_;
  bool has_position_ =  command_struct_.has_position_;


  if(has_velocity_){
    joint_.setCommand(command_velocity1);
  }
  else{
      double error, vel_error;
      double commanded_velocity;
      double current_position = joint_.getPosition();

  // Make sure joint is within limits if applicable
      enforceJointLimits(command_position);

  // Compute position error
      if (joint_urdf_->type == urdf::Joint::REVOLUTE)
      {
   angles::shortest_angular_distance_with_limits(
      current_position,
      command_position,
      joint_urdf_->limits->lower,
      joint_urdf_->limits->upper,
      error);
      }
  else if (joint_urdf_->type == urdf::Joint::CONTINUOUS)
  {
    error = angles::shortest_angular_distance(current_position, command_position);
  }
  else //prismatic
  {
    error = command_position - current_position;
  }
  commanded_velocity = pid_controller_.computeCommand(error, period);
  joint_.setCommand(commanded_velocity);
  }
  // publish state
  // if (loop_count_ % 10 == 0)
  // {
  //   if(controller_state_publisher_ && controller_state_publisher_->trylock())
  //   {
  //     controller_state_publisher_->msg_.header.stamp = time;
  //     controller_state_publisher_->msg_.set_point = command_position;
  //     controller_state_publisher_->msg_.process_value = current_position;
  //     controller_state_publisher_->msg_.process_value_dot = joint_.getVelocity();
  //     controller_state_publisher_->msg_.error = error;
  //     controller_state_publisher_->msg_.time_step = period.toSec();
  //     controller_state_publisher_->msg_.command = commanded_velocity;

  //     double dummy;
  //     bool antiwindup;
  //     getGains(controller_state_publisher_->msg_.p,
  //       controller_state_publisher_->msg_.i,
  //       controller_state_publisher_->msg_.d,
  //       controller_state_publisher_->msg_.i_clamp,
  //       dummy,
  //       antiwindup);
  //     controller_state_publisher_->msg_.antiwindup = static_cast<char>(antiwindup);
  //     controller_state_publisher_->unlockAndPublish();
  //   }
  // }
  // loop_count_++;
}

void MyController::setPositionCommandCB(const std_msgs::Float64ConstPtr& msg)
{
  command_struct_.position_ = msg->data;
  command_struct_.has_velocity_ = false;
  command_struct_.has_position_=true;
  command_.writeFromNonRT(command_struct_);
}

void MyController::setVelocityCommandCB(const std_msgs::Float64ConstPtr &msg)
{
  command_struct_.velocity_ = msg->data;
  command_struct_.has_velocity_ = true;
  command_struct_.has_position_ = false;
  command_.writeFromNonRT(command_struct_);
}

// Note: we may want to remove this function once issue https://github.com/ros/angles/issues/2 is resolved
void MyController::enforceJointLimits(double &command)
{
  // Check that this joint has applicable limits
  if (joint_urdf_->type == urdf::Joint::REVOLUTE || joint_urdf_->type == urdf::Joint::PRISMATIC)
  {
    if( command > joint_urdf_->limits->upper ) // above upper limnit
    {
      command = joint_urdf_->limits->upper;
    }
    else if( command < joint_urdf_->limits->lower ) // below lower limit
    {
      command = joint_urdf_->limits->lower;
    }
  }
}

} // namespace

PLUGINLIB_EXPORT_CLASS( my_velocity_controllers::MyController, controller_interface::ControllerBase)
