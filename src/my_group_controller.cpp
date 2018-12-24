#include "../include/my_velocity_controllers/my_group_controller.h"
#include <pluginlib/class_list_macros.hpp>
#include <angles/angles.h>

namespace my_velocity_controllers
{

/**
 * \brief Forward command controller for a set of effort controlled joints (torque or force).
 *
 * This class forwards the commanded efforts down to a set of joints.
 *
 * \section ROS interface
 *
 * \param type Must be "JointGroupEffortController".
 * \param joints List of names of the joints to control.
 *
 * Subscribes to:
 * - \b command (std_msgs::Float64MultiArray) : The joint efforts to apply
 */
MyGroupController::MyGroupController() {}
MyGroupController::~MyGroupController() 
{ 
    sub_position_command_.shutdown();
    sub_velocity_command_.shutdown(); 
}

bool MyGroupController::init(hardware_interface::VelocityJointInterface *hw, ros::NodeHandle &n)
{
    // List of controlled joints
    std::string param_name = "joints";
    if (!n.getParam(param_name, joint_names_))
    {
        ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << n.getNamespace() << ").");
        return false;
    }
    n_joints_ = joint_names_.size();

    if (n_joints_ == 0)
    {
        ROS_ERROR_STREAM("List of joint names is empty.");
        return false;
    }

    // Get URDF
    urdf::Model urdf;
    if (!urdf.initParam("robot_description"))
    {
        ROS_ERROR("Failed to parse urdf file");
        return false;
    }

    pid_controllers_.resize(n_joints_);
    command_structs_.resize(n_joints_);

    for (unsigned int i = 0; i < n_joints_; i++)
    {
        try
        {
            joints_.push_back(hw->getHandle(joint_names_[i]));
        }
        catch (const hardware_interface::HardwareInterfaceException &e)
        {
            ROS_ERROR_STREAM("Exception thrown: " << e.what());
            return false;
        }

        urdf::JointConstSharedPtr joint_urdf = urdf.getJoint(joint_names_[i]);
        if (!joint_urdf)
        {
            ROS_ERROR("Could not find joint '%s' in urdf", joint_names_[i].c_str());
            return false;
        }
        joint_urdfs_.push_back(joint_urdf);

        // Load PID Controller using gains set on parameter server
        if (!pid_controllers_[i].init(ros::NodeHandle(n, joint_names_[i] + "/pid")))
        {
            ROS_ERROR_STREAM("Failed to load PID parameters from " << joint_names_[i] + "/pid");
            return false;
        }
    }

    // commands_buffer_.writeFromNonRT(std::vector<double>(n_joints_, 0.0));

    sub_position_command_ = n.subscribe<std_msgs::Float64MultiArray>("position_command", 1, &MyGroupController::positionCommandCB, this);
    sub_velocity_command_ = n.subscribe<std_msgs::Float64MultiArray>("velocity_command", 1, &MyGroupController::velocityCommandCB, this);
    return true;
}

void MyGroupController::starting(const ros::Time &time)
{
    for (unsigned int i = 0; i < n_joints_; i++){
        double pos_command = joints_[i].getPosition();
        enforceJointLimits(pos_command,i);

        command_structs_[i].position_ = pos_command;
        command_structs_[i].has_velocity_ = false;
        command_structs_[i].has_position_ = true;
        pid_controllers_[i].reset();
    }
    commands_buffer_.initRT(command_structs_);
}

void MyGroupController::update(const ros::Time &time, const ros::Duration &period)
{
    std::vector<Commands> commands_ = *commands_buffer_.readFromRT();
    for (unsigned int i = 0; i < n_joints_; i++)
    {
        double command_position = commands_[i].position_;
        double command_velocity1 = commands_[i].velocity_;
        bool has_velocity_ = commands_[i].has_velocity_;
        bool has_position_ = commands_[i].has_position_;
        if (has_velocity_)
        {
            joints_[i].setCommand(command_velocity1);
        }
        else{
            double error;
            double commanded_velocity;
            double current_position = joints_[i].getPosition();

            enforceJointLimits(command_position,i);
            if (joint_urdfs_[i]->type == urdf::Joint::REVOLUTE)
            {
                angles::shortest_angular_distance_with_limits(
                    current_position,
                    command_position,
                    joint_urdfs_[i]->limits->lower,
                    joint_urdfs_[i]->limits->upper,
                    error);
            }
            else if (joint_urdfs_[i]->type == urdf::Joint::CONTINUOUS)
            {
                error = angles::shortest_angular_distance(current_position, command_position);
            }
            else //prismatic
            {
                error = command_position - current_position;
            }
            commanded_velocity = pid_controllers_[i].computeCommand(error, period);
            joints_[i].setCommand(commanded_velocity);
        }
    }
}


void MyGroupController::positionCommandCB(const std_msgs::Float64MultiArrayConstPtr &msg)
{
    if (msg->data.size() != n_joints_)
    {
        ROS_ERROR_STREAM("Dimension of command (" << msg->data.size() << ") does not match number of joints (" << n_joints_ << ")! Not executing!");
        return;
    }
    for (unsigned int i = 0; i < n_joints_; i++){
        command_structs_[i].position_ = msg->data[i];
        command_structs_[i].has_velocity_ = false;
        command_structs_[i].has_position_ = true;
    }
    commands_buffer_.writeFromNonRT(command_structs_);
}

void MyGroupController::velocityCommandCB(const std_msgs::Float64MultiArrayConstPtr &msg)
{
    if (msg->data.size() != n_joints_)
    {
        ROS_ERROR_STREAM("Dimension of command (" << msg->data.size() << ") does not match number of joints (" << n_joints_ << ")! Not executing!");
        return;
    }
    for (unsigned int i = 0; i < n_joints_; i++)
    {
        command_structs_[i].velocity_ = msg->data[i];
        command_structs_[i].has_velocity_ = true;
        command_structs_[i].has_position_ = false;
    }
    commands_buffer_.writeFromNonRT(command_structs_);
}


void MyGroupController::enforceJointLimits(double &command, unsigned int index)
{
    // Check that this joint has applicable limits
    if (joint_urdfs_[index]->type == urdf::Joint::REVOLUTE || joint_urdfs_[index]->type == urdf::Joint::PRISMATIC)
    {
        if (command > joint_urdfs_[index]->limits->upper) // above upper limnit
        {
            command = joint_urdfs_[index]->limits->upper;
        }
        else if (command < joint_urdfs_[index]->limits->lower) // below lower limit
        {
            command = joint_urdfs_[index]->limits->lower;
        }
    }
}
} // namespace my_velocity_controllers

PLUGINLIB_EXPORT_CLASS(my_velocity_controllers::MyGroupController, controller_interface::ControllerBase)
