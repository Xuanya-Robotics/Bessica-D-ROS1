#include "bessica_d_driver/bessica_d_hw_interface.h"
#include <vector>

namespace bessica_d_driver
{

BessicaDHardwareInterface::BessicaDHardwareInterface(ros::NodeHandle& nh) : nh_(nh) {}

bool BessicaDHardwareInterface::init()
{
    // Get joint names from the parameter server
    if (!nh_.getParam("joints", joint_names_))
    {
        ROS_ERROR("Could not find 'joints' parameter on the parameter server.");
        return false;
    }
    num_joints_ = joint_names_.size();
    ROS_INFO("Initializing hardware interface for %d joints.", (int)num_joints_);

    // Resize storage vectors
    joint_velocities_.resize(num_joints_, 0.0); // Initialize dummy velocities to zero
    joint_efforts_.resize(num_joints_, 0.0); // Not used, but required by the interface
    joint_positions_.resize(num_joints_, 0.0);
    joint_position_commands_.resize(num_joints_, 0.0);
    
    // --- NEW --- Resize the new raw data vector as well
    raw_joint_positions_.resize(num_joints_, 0.0);

    // Create a map for efficient name-to-index lookup
    for (size_t i = 0; i < num_joints_; ++i)
    {
        joint_name_to_index_map_[joint_names_[i]] = i;
    }

    // Register handles with the ros_control interfaces
    for (size_t i = 0; i < num_joints_; ++i)
    {
        // Joint State Interface
        jnt_state_interface_.registerHandle(hardware_interface::JointStateHandle(
                joint_names_[i], &joint_positions_[i], &joint_velocities_[i], &joint_efforts_[i]));
        // Position Joint Interface
        pos_jnt_interface_.registerHandle(hardware_interface::JointHandle(
            jnt_state_interface_.getHandle(joint_names_[i]), &joint_position_commands_[i]));
    }

    // Register the interfaces with this class
    registerInterface(&jnt_state_interface_);
    registerInterface(&pos_jnt_interface_);

    // Initialize ROS publisher and subscriber
    joint_command_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_commands", 1);
    joint_state_sub_ = nh_.subscribe("/joint_states", 10, &BessicaDHardwareInterface::jointStateCallback, this);

    ROS_INFO("Bessica-D hardware interface initialized successfully.");
    return true;
}

void BessicaDHardwareInterface::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(command_mutex_); // Lock to ensure thread safety

    // Map incoming joint states to our internal RAW storage vectors
    for (size_t i = 0; i < msg->name.size(); ++i)
    {
        auto it = joint_name_to_index_map_.find(msg->name[i]);
        if (it != joint_name_to_index_map_.end())
        {
            size_t index = it->second;
            // --- MODIFIED --- Write to the raw data vector, not the main one
            raw_joint_positions_[index] = msg->position[i];
        }
    }
}

void BessicaDHardwareInterface::read(const ros::Time& time, const ros::Duration& period)
{
    // --- MODIFIED --- This function is now responsible for synchronizing the data
    std::lock_guard<std::mutex> lock(command_mutex_);
    // Copy the latest data from the raw vector to the one used by ros_control
    joint_positions_ = raw_joint_positions_;
    // Note: We are not updating velocities or efforts, so they will remain zero.
    // This is acceptable if you are only using position control.
}

void BessicaDHardwareInterface::write(const ros::Time& time, const ros::Duration& period)
{
    sensor_msgs::JointState command_msg;
    command_msg.header.stamp = ros::Time::now();

    // No lock needed here if we assume joint_position_commands_ is only written
    // to by the controller manager in this thread context.
    command_msg.name = joint_names_;
    command_msg.position = joint_position_commands_;

    joint_command_pub_.publish(command_msg);
}

} // namespace bessica_d_driver