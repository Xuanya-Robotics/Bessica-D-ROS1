#include "bessica_d_driver/bessica_d_driver_node.hpp"
#include <cmath>
#include <numeric> // For std::accumulate
#include <map>
#include <set>   // For std::set
#include <thread> // for std::this_thread
#include <chrono> // for std::chrono


// --- Command IDs, Data Identifiers, etc. remain the same ---
constexpr uint8_t CMD_GRIPPER = 0x02;
constexpr uint8_t CMD_ZERO_CAL = 0x03;
constexpr uint8_t CMD_DEMO_CONTROL = 0x13;
constexpr uint8_t PRESENT_POSITION = 0x38;
constexpr uint8_t FEEDBACK_ERROR = 0xEE;
constexpr int JOINTS_PER_ARM = 7;
constexpr int SERVOS_PER_ARM = 10;
constexpr size_t SINGLE_ARM_CMD_PAYLOAD_SIZE = 1 + SERVOS_PER_ARM * 2;
constexpr size_t SINGLE_ARM_FRAME_SIZE = 5 + SINGLE_ARM_CMD_PAYLOAD_SIZE; // 26 bytes
constexpr size_t GRIPPER_CMD_PAYLOAD_SIZE = 1 + 2;
constexpr size_t GRIPPER_FRAME_SIZE = 5 + GRIPPER_CMD_PAYLOAD_SIZE; // 8 bytes


BessicaDDriverNode::BessicaDDriverNode() : pnh_("~"), last_process_time_(0.0)
{
    load_parameters();
    setup_ros_communications();

    // Attempt initial connection
    if (communicator_->connect()) {
        ROS_INFO("Initial connection successful. Enabling full torque mode.");
    } else {
        ROS_ERROR("Initial connection failed. Starting reconnect timer.");
        reconnect_timer_ = nh_.createTimer(ros::Duration(5.0), &BessicaDDriverNode::reconnect_callback, this);
    }
}


BessicaDDriverNode::~BessicaDDriverNode()
{
    if (communicator_) {
        communicator_->disconnect();
    }
}
void BessicaDDriverNode::load_parameters()
{
    std::string port;
    int baud_rate;
    
    pnh_.param<std::string>("port", port, "/dev/ttyUSB0");
    pnh_.param<int>("baud_rate", baud_rate, 921600);
    pnh_.param<bool>("debug_mode", debug_mode_, false);
    pnh_.param<double>("rate_limit_sec", rate_limit_sec_, 0.01);

    communicator_ = std::make_unique<SerialCommunicator>(port, baud_rate, debug_mode_);

    // These could also be loaded from the parameter server if desired
    joint_to_servo_map_index_ = {0, 0, 1, 1, 2, 3, 3, 4, 5, 6};
    joint_to_servo_map_direction_ = {-1.0, -1.0, 1.0, -1.0, 1.0, 1.0, -1.0, 1.0, 1.0, 1.0};
    servo_to_joint_map_index_ = {0, -1, 1, -1, 2, 3,-1, 4, 5, 6}; 
    servo_to_joint_map_direction_ = {1.0, 0, 1.0, 0, 1.0, 1.0, 0.0, 1.0, 1.0, 1.0};
    left_arm_joint_directions_ = {1.0, 1.0, 1.0, 1.0, 1.0, -1.0, 1.0};
    right_arm_joint_directions_ = {-1.0, 1.0, -1.0, 1.0, -1.0, -1.0, 1.0};
    left_joint_names_ = {"left_arm_joint1", "left_arm_joint2", "left_arm_joint3", "left_arm_joint4", "left_arm_joint5", "left_arm_joint6", "left_arm_joint7"};
    right_joint_names_ = {"right_arm_joint1", "right_arm_joint2", "right_arm_joint3", "right_arm_joint4", "right_arm_joint5", "right_arm_joint6", "right_arm_joint7"};
    left_gripper_name_ = "left_arm_gripper_joint1";
    right_gripper_name_ = "right_arm_gripper_joint1";
}


void BessicaDDriverNode::setup_ros_communications()
{
    joint_state_pub_std_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 10);
    joint_command_sub_ = nh_.subscribe("/joint_commands", 10, &BessicaDDriverNode::joint_command_callback, this);
    zero_calib_sub_ = nh_.subscribe("/zero_calibrate", 10, &BessicaDDriverNode::zero_calibrate_callback, this);
    demo_mode_sub_ = nh_.subscribe("/demonstration", 10, &BessicaDDriverNode::demonstration_mode_callback, this);
    processing_timer_ = nh_.createTimer(ros::Duration(0.01), &BessicaDDriverNode::process_serial_data_callback, this);
}


void BessicaDDriverNode::reconnect_callback(const ros::TimerEvent& event)
{
    if (!communicator_->is_connected()) {
        ROS_INFO("Attempting to reconnect...");
        if (communicator_->connect()) {
            ROS_INFO("Reconnect successful! Enabling full torque mode.");
            reconnect_timer_.stop(); // Stop the timer upon success
        }
    } else {
        reconnect_timer_.stop(); // Stop if already connected
    }

}

void BessicaDDriverNode::joint_command_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
    if (!communicator_->is_connected()) {
        ROS_WARN_THROTTLE(5.0, "Communicator not connected, skipping joint command.");
        return;
    }
    std::lock_guard<std::mutex> write_lock(serial_write_mutex_);

    const uint8_t LEFT_ARM_ID = 0x01;
    const uint8_t RIGHT_ARM_ID = 0x02;

    std::map<std::string, double> joint_map;
    std::set<std::string> received_joints;
    for (size_t i = 0; i < msg->name.size(); ++i) {
        joint_map[msg->name[i]] = msg->position[i];
        received_joints.insert(msg->name[i]);
    }
    // --- LEFT ARM ---
    // Check if any left arm joint is in the message
    if (std::any_of(left_joint_names_.begin(), left_joint_names_.end(),
                    [&](const std::string& name){ return received_joints.count(name); })) {
        std::vector<uint8_t> frame(SINGLE_ARM_FRAME_SIZE);
        frame[0] = FRAME_START_BYTE;
        frame[1] = CMD_DUAL_ARM;
        frame[2] = SINGLE_ARM_CMD_PAYLOAD_SIZE; // 21
        frame[3] = LEFT_ARM_ID;

        std::vector<double> angles;
        for (const auto& name : left_joint_names_) {
            angles.push_back(joint_map.count(name) ? joint_map.at(name) : 0.0);
        }

        for (int i = 0; i < SERVOS_PER_ARM; ++i) {
            int joint_idx = joint_to_servo_map_index_[i];
            double mapped_angle = angles[joint_idx] * left_arm_joint_directions_[joint_idx];
            double final_rad = mapped_angle * joint_to_servo_map_direction_[i];
            uint16_t hw_val = rad_to_hardware_value(final_rad);

            size_t byte_idx = 4 + i * 2;
            frame[byte_idx]     = hw_val & 0xFF;
            frame[byte_idx + 1] = (hw_val >> 8) & 0xFF;
        }

        frame[SINGLE_ARM_FRAME_SIZE - 2] = calculate_checksum(frame);
        frame[SINGLE_ARM_FRAME_SIZE - 1] = FRAME_END_BYTE;

        communicator_->write_raw_frame(frame);
        // wait for 2 ms to ensure the frame is sent completely
    }
    // Check for left gripper
    if (received_joints.count(left_gripper_name_)) {
        std::vector<uint8_t> frame(GRIPPER_FRAME_SIZE);
        frame[0] = FRAME_START_BYTE;
        frame[1] = CMD_GRIPPER;
        frame[2] = GRIPPER_CMD_PAYLOAD_SIZE; 
        frame[3] = LEFT_ARM_ID;

        uint16_t hw_val = rad_to_hardware_value_grip(joint_map.at(left_gripper_name_));
        frame[4] = hw_val & 0xFF;
        frame[5] = (hw_val >> 8) & 0xFF;

        frame[GRIPPER_FRAME_SIZE - 2] = calculate_checksum(frame);
        frame[GRIPPER_FRAME_SIZE - 1] = FRAME_END_BYTE;
        communicator_->write_raw_frame(frame);
    }

    // --- RIGHT ARM ---
    if (std::any_of(right_joint_names_.begin(), right_joint_names_.end(),
                    [&](const std::string& name){ return received_joints.count(name); })) {
        std::vector<uint8_t> frame(SINGLE_ARM_FRAME_SIZE);
        frame[0] = FRAME_START_BYTE;
        frame[1] = CMD_DUAL_ARM;
        frame[2] = SINGLE_ARM_CMD_PAYLOAD_SIZE; // 21
        frame[3] = RIGHT_ARM_ID;

        std::vector<double> angles;
        for (const auto& name : right_joint_names_) {
            angles.push_back(joint_map.count(name) ? joint_map.at(name) : 0.0);
        }

        for (int i = 0; i < SERVOS_PER_ARM; ++i) {
            int joint_idx = joint_to_servo_map_index_[i];
            double mapped_angle = angles[joint_idx] * right_arm_joint_directions_[joint_idx];
            double final_rad = mapped_angle * joint_to_servo_map_direction_[i];
            uint16_t hw_val = rad_to_hardware_value(final_rad);

            size_t byte_idx = 4 + i * 2;
            frame[byte_idx]     = hw_val & 0xFF;
            frame[byte_idx + 1] = (hw_val >> 8) & 0xFF;
        }

        frame[SINGLE_ARM_FRAME_SIZE - 2] = calculate_checksum(frame);
        frame[SINGLE_ARM_FRAME_SIZE - 1] = FRAME_END_BYTE;
        communicator_->write_raw_frame(frame);
    }
    if (received_joints.count(right_gripper_name_)) {
        std::vector<uint8_t> frame(GRIPPER_FRAME_SIZE);
        frame[0] = FRAME_START_BYTE;
        frame[1] = CMD_GRIPPER;
        frame[2] = GRIPPER_CMD_PAYLOAD_SIZE; // 3
        frame[3] = RIGHT_ARM_ID;

        uint16_t hw_val = rad_to_hardware_value_grip(joint_map.at(right_gripper_name_));
        frame[4] = hw_val & 0xFF;
        frame[5] = (hw_val >> 8) & 0xFF;

        frame[GRIPPER_FRAME_SIZE - 2] = calculate_checksum(frame);
        frame[GRIPPER_FRAME_SIZE - 1] = FRAME_END_BYTE;
        // communicator_->print_hex_frame("right Gripper Frame: ", frame);

        communicator_->write_raw_frame(frame);

    }
}


void BessicaDDriverNode::process_serial_data_callback(const ros::TimerEvent& event)
{
    process_serial_data();
}


uint8_t BessicaDDriverNode::calculate_checksum(const std::vector<uint8_t>& frame)
{
    // Correct checksum based on Python: sum of payload bytes (from index 3 up to, but not including, the checksum byte)
    if (frame.size() < 5) return 0;
    int sum = std::accumulate(frame.begin() + 3, frame.end() - 2, 0);
    return static_cast<uint8_t>(sum % 2);
}



void BessicaDDriverNode::process_serial_data()
{
    if (!communicator_->is_connected()) return;
    std::vector<uint8_t> packet;
    while (communicator_->get_packet(packet)) {
        if (packet.empty()) continue;
        uint8_t command_id = packet[0];
        std::vector<uint8_t> data_payload(packet.begin() + 1, packet.end());

        switch (command_id) {
            case CMD_DUAL_ARM:
                parse_dual_arm_states_frame(data_payload);
                break;
            case FEEDBACK_ERROR:
                parse_error_frame(data_payload);
                break;
            default:
                ROS_WARN_STREAM("Received frame with unhandled command ID: 0x" << std::hex << static_cast<int>(command_id));
                break;
        }
    }
}

void BessicaDDriverNode::parse_dual_arm_states_frame(const std::vector<uint8_t>& payload)
{
    ros::Time now = ros::Time::now();
    // if ((now - last_process_time_).toSec() < rate_limit_sec_) return;
    last_process_time_ = now;

    if (payload.size() < DUAL_ARM_FEEDBACK_PAYLOAD_SIZE) {
        ROS_WARN("Dual arm feedback payload too short. Expected %zu, got %zu.", DUAL_ARM_FEEDBACK_PAYLOAD_SIZE, payload.size());
        return;
    }
    if (payload[0] != PRESENT_POSITION) {
        ROS_WARN("Incorrect data type in feedback. Expected 0x%02X, got 0x%02X.", PRESENT_POSITION, payload[0]);
        return;
    }

    sensor_msgs::JointState js_msg;
    js_msg.header.stamp = now;


    const size_t left_arm_payload_start_idx = 1;
    const size_t right_arm_payload_start_idx = 1 + 22;
    struct ArmFeedbackData {
        const std::vector<std::string>& joint_names;
        const std::string& gripper_name;
        const std::vector<double>& directions;
        size_t data_start_index; // Starting index within the payload
    };

    std::vector<ArmFeedbackData> arms_to_parse = {
        {left_joint_names_, left_gripper_name_, left_arm_joint_directions_, left_arm_payload_start_idx},  // Left arm data starts at frame index 4
        {right_joint_names_, right_gripper_name_, right_arm_joint_directions_, right_arm_payload_start_idx} // Right arm data starts at frame index 26
    };

    for (const auto& arm : arms_to_parse) {
        std::vector<double> joint_values(JOINTS_PER_ARM, 0.0);
        for (int i = 0; i < SERVOS_PER_ARM; ++i) {
            size_t byte_idx = arm.data_start_index + i * 2;
            if (byte_idx + 1 >= payload.size()) { // Safety check
                ROS_ERROR("Payload parsing error: servo index out of bounds.");
                return;
            }

            uint16_t hw_val = payload[byte_idx] | (payload[byte_idx + 1] << 8);

            int joint_idx = servo_to_joint_map_index_[i];
            if (joint_idx != -1) {
                // Apply arm mirroring direction AFTER converting from hardware value
                joint_values[joint_idx] = hardware_value_to_rad(hw_val) * arm.directions[joint_idx];
            }
        }
        // Parse Gripper
        size_t gripper_byte_idx = arm.data_start_index + SERVOS_PER_ARM * 2;
        if (gripper_byte_idx + 1 >= payload.size()) { // Safety check
            ROS_ERROR("Payload parsing error: gripper index out of bounds.");
            return;
        }
        uint16_t gripper_hw_val = payload[gripper_byte_idx] | (payload[gripper_byte_idx + 1] << 8);
        double gripper_rad = hardware_value_to_rad_grip(gripper_hw_val);
        
        js_msg.name.insert(js_msg.name.end(), arm.joint_names.begin(), arm.joint_names.end());
        js_msg.position.insert(js_msg.position.end(), joint_values.begin(), joint_values.end());
        js_msg.name.push_back(arm.gripper_name);
        js_msg.position.push_back(gripper_rad);
    }
    joint_state_pub_std_.publish(js_msg);
}



void BessicaDDriverNode::parse_error_frame(const std::vector<uint8_t>& payload)
{
    if (payload.size() < 2) {
        ROS_WARN("Error frame payload too short. Expected at least 2 bytes, got %zu.", payload.size());
        return;
    }
    uint8_t error_type = payload[0];
    uint8_t error_param = payload[1];
    ROS_ERROR("Received Error Frame from Hardware: Type=0x%02X, Param=0x%02X", error_type, error_param);
}



std::vector<uint8_t> BessicaDDriverNode::generate_simple_frame(uint8_t command, const std::vector<uint8_t>& data)
{
    // std::lock_guard<std::mutex> write_lock(serial_write_mutex_);
    std::vector<uint8_t> frame(5 + data.size());
    frame[0] = FRAME_START_BYTE;
    frame[1] = command;
    frame[2] = data.size();
    std::copy(data.begin(), data.end(), frame.begin() + 3);
    frame[frame.size() - 2] = calculate_checksum(frame);
    frame[frame.size() - 1] = FRAME_END_BYTE;
    return frame;
}



void BessicaDDriverNode::zero_calibrate_callback(const std_msgs::Bool::ConstPtr& msg)
{
    if (msg->data) {
        ROS_INFO("Received Zero Calibration command for both arms.");
        std::lock_guard<std::mutex> lock(serial_write_mutex_);
        auto frame = generate_simple_frame(CMD_ZERO_CAL, {0x03});
        communicator_->write_raw_frame(frame);
        ros::Duration(1.0).sleep(); // Wait for command to be processed
    }
}

void BessicaDDriverNode::demonstration_mode_callback(const std_msgs::Bool::ConstPtr& msg)
{
    uint8_t torque_state = msg->data ? 0x00 : 0x01; 
    ROS_INFO("%s Torque (Demonstration Mode).", msg->data ? "Disabling" : "Enabling");
    std::lock_guard<std::mutex> lock(serial_write_mutex_);
    auto frame = generate_simple_frame(CMD_DEMO_CONTROL, {0x03, torque_state});
    communicator_->write_raw_frame(frame);
}

uint16_t BessicaDDriverNode::rad_to_hardware_value(double angle_rad) {
    double angle_deg = angle_rad * 180.0 / M_PI;
    angle_deg = std::max(-180.0, std::min(180.0, angle_deg));
    int value = static_cast<int>((angle_deg + 180.0) / 360.0 * 4096.0);
    return std::max(0, std::min(4095, value));
}

double BessicaDDriverNode::hardware_value_to_rad(uint16_t hw_value) {
    hw_value = std::max(0, std::min(4095, (int)hw_value));
    double angle_deg = -180.0 + (static_cast<double>(hw_value) / 4095.0) * 360.0;
    return angle_deg * M_PI / 180.0;
}

uint16_t BessicaDDriverNode::rad_to_hardware_value_grip(double angle_rad)
{
    double angle_deg = angle_rad * 180.0 / M_PI;
    angle_deg = std::max(0.0, std::min(100.0, angle_deg)); // Clamp to expected [0, 100] deg range
    int hardware_value = static_cast<int>(angle_deg * 8.52 + 2048.0);
    return std::max(2048, std::min(3590, hardware_value));
}

double BessicaDDriverNode::hardware_value_to_rad_grip(uint16_t hw_value) {
    hw_value = std::max(2048, std::min(3590, (int)hw_value));
    double angle_deg = (static_cast<double>(hw_value) - 2048.0) / 8.52;
    return angle_deg * M_PI / 180.0;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bessica_d_driver_node");
    BessicaDDriverNode driver_node;
    ros::spin();
    return 0;
}