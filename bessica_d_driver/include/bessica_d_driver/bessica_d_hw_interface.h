#ifndef BESSICA_D_HW_INTERFACE_H
#define BESSICA_D_HW_INTERFACE_H

#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <sensor_msgs/JointState.h>
#include <string>
#include <vector>
#include <mutex> // Required for std::mutex and std::lock_guard

namespace bessica_d_driver
{

class BessicaDHardwareInterface : public hardware_interface::RobotHW
{
public:
    BessicaDHardwareInterface(ros::NodeHandle& nh);
    bool init();
    void read(const ros::Time& time, const ros::Duration& period);
    void write(const ros::Time& time, const ros::Duration& period);

private:
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);

    ros::NodeHandle nh_;
    ros::Publisher joint_command_pub_;
    ros::Subscriber joint_state_sub_;

    // ROS-Control interfaces
    hardware_interface::JointStateInterface jnt_state_interface_;
    hardware_interface::PositionJointInterface pos_jnt_interface_;

    // Data storage
    std::vector<std::string> joint_names_;
    std::map<std::string, size_t> joint_name_to_index_map_;
    size_t num_joints_;
    
    // These vectors are used by the controller manager
    std::vector<double> joint_positions_;
    std::vector<double> joint_velocities_;
    std::vector<double> joint_efforts_;
    std::vector<double> joint_position_commands_;
    
    // --- NEW ---
    // This vector stores the data from the callback for thread-safe access
    std::vector<double> raw_joint_positions_;
    std::mutex command_mutex_;
};

} // namespace bessica_d_driver

#endif // BESSICA_D_HW_INTERFACE_H