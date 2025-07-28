#ifndef BESSICA_D_DRIVER_NODE_H
#define BESSICA_D_DRIVER_NODE_H

#include "ros/ros.h"
#include "serial_communicator.hpp" // Assuming this is a non-ROS helper class
#include "std_msgs/Bool.h"
#include "sensor_msgs/JointState.h"
#include <memory>
#include <vector>
#include <string>
#include <mutex>

class BessicaDDriverNode
{
public:
    BessicaDDriverNode();
    ~BessicaDDriverNode();

private:
    // ROS 1 NodeHandle
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_; // Private NodeHandle for parameters

    // Initialization
    void load_parameters();
    void setup_ros_communications();
    
    // Callbacks for incoming commands
    void joint_command_callback(const sensor_msgs::JointState::ConstPtr& msg);
    void zero_calibrate_callback(const std_msgs::Bool::ConstPtr& msg);
    void demonstration_mode_callback(const std_msgs::Bool::ConstPtr& msg);
    
    // Timer callbacks
    void process_serial_data_callback(const ros::TimerEvent& event);
    void reconnect_callback(const ros::TimerEvent& event);

    // Main processing loop
    void process_serial_data();
    void parse_dual_arm_states_frame(const std::vector<uint8_t>& payload);
    void parse_error_frame(const std::vector<uint8_t>& payload);

    // Data Conversion & Framing
    uint16_t rad_to_hardware_value(double angle_rad);
    uint16_t rad_to_hardware_value_grip(double angle_rad);
    double hardware_value_to_rad(uint16_t hw_value);
    double hardware_value_to_rad_grip(uint16_t hw_value);
    std::vector<uint8_t> generate_simple_frame(uint8_t command, const std::vector<uint8_t>& data);
    uint8_t calculate_checksum(const std::vector<uint8_t>& frame_data);

    // Member Variables
    std::unique_ptr<SerialCommunicator> communicator_;
    ros::Timer processing_timer_;
    ros::Timer reconnect_timer_;

    // Publishers & Subscribers
    ros::Publisher joint_state_pub_std_;
    ros::Subscriber joint_command_sub_;
    ros::Subscriber zero_calib_sub_;
    ros::Subscriber demo_mode_sub_;

    // Configuration and State
    bool debug_mode_;
    double rate_limit_sec_;
    ros::Time last_process_time_;

    // Dual-Arm Configuration
    std::vector<int> servo_to_joint_map_index_;
    std::vector<double> left_arm_joint_directions_;
    std::vector<double> right_arm_joint_directions_;
    std::vector<int> joint_to_servo_map_index_;
    std::vector<double> joint_to_servo_map_direction_;
    std::vector<double> servo_to_joint_map_direction_;
    std::vector<std::string> left_joint_names_;
    std::vector<std::string> right_joint_names_;
    std::string left_gripper_name_;
    std::string right_gripper_name_;
    std::mutex serial_write_mutex_;
};

#endif // BESSICA_D_DRIVER_NODE_H