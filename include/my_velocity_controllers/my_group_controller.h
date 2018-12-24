#ifndef MY_VELOCITY_CONTROLLERS_MY_GROUP_CONTROLLER_H
#define MY_VELOCITY_CONTROLLERS_MY_GROUP_CONTROLLER_H

#include <ros/node_handle.h>
#include <urdf/model.h>
#include <control_toolbox/pid.h>
#include <boost/scoped_ptr.hpp>
#include <realtime_tools/realtime_publisher.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <std_msgs/Float64MultiArray.h>
#include <control_msgs/JointControllerState.h>
#include <realtime_tools/realtime_buffer.h>

namespace my_velocity_controllers
{

class MyGroupController : public controller_interface::Controller<hardware_interface::VelocityJointInterface>
{
  public:
    MyGroupController();
    ~MyGroupController();

    struct Commands
    {
        double position_;   
        double velocity_;   
        bool has_velocity_; 
        bool has_position_;
    };

    bool init(hardware_interface::VelocityJointInterface *hw, ros::NodeHandle &n);
    void starting(const ros::Time &time);
    void update(const ros::Time & /*time*/, const ros::Duration & /*period*/);

    std::vector<std::string> joint_names_;
    std::vector<hardware_interface::JointHandle> joints_;
    realtime_tools::RealtimeBuffer<std::vector<Commands> > commands_buffer_;
    unsigned int n_joints_;
    std::vector<Commands> command_structs_;

  private:
    ros::Subscriber sub_position_command_;
    ros::Subscriber sub_velocity_command_;
    std::vector<control_toolbox::Pid> pid_controllers_; /**< Internal PID controllers. */

    std::vector<urdf::JointConstSharedPtr> joint_urdfs_;

    void positionCommandCB(const std_msgs::Float64MultiArrayConstPtr &msg);
    void velocityCommandCB(const std_msgs::Float64MultiArrayConstPtr &msg);

    void enforceJointLimits(double &command, unsigned int index);
}; // class

} // namespace my_velocity_controllers

#endif
