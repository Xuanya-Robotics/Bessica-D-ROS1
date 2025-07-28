#include "serial_communicator.hpp"
#include <ros/console.h>
#include <chrono>
#include <numeric>
#include <iomanip>
#include <sstream>
#include <ros/ros.h>

SerialCommunicator::SerialCommunicator(std::string port_name, uint32_t baud_rate, bool debug_mode)
    : port_name_(std::move(port_name)),
      baud_rate_(baud_rate),
      debug_mode_(debug_mode),
      is_running_(false)
{
    ROS_INFO("SerialCommunicator (using ros-serial) created for port '%s' at %u bps.", port_name_.c_str(), baud_rate_);
    if (debug_mode_) {
        ROS_INFO("Debug mode is enabled.");
    }
}

SerialCommunicator::~SerialCommunicator()
{
    disconnect();
}

bool SerialCommunicator::connect()
{
    if (serial_port_.isOpen()) {
        return true;
    }
    ROS_INFO("Attempting to open serial port: %s @ %u bps", port_name_.c_str(), baud_rate_);

    try {
        serial_port_.setPort(port_name_);
        serial_port_.setBaudrate(baud_rate_);
        // Set a timeout. This is important for robust reading.
        serial::Timeout timeout = serial::Timeout::simpleTimeout(1000); // 1-second timeout
        serial_port_.setTimeout(timeout);
        serial_port_.open();
    }
    catch (const serial::IOException& e) {
        ROS_ERROR("Failed to open serial port %s: %s", port_name_.c_str(), e.what());
        return false;
    }
    
    if (serial_port_.isOpen()) {
        is_running_ = true;
        read_thread_ = std::thread(&SerialCommunicator::read_thread_loop, this);
        ROS_INFO("Serial port %s opened successfully. Read thread started.", port_name_.c_str());
        return true;
    }
    return false;
}

void SerialCommunicator::disconnect()
{
    is_running_ = false;
    if (read_thread_.joinable()) {
        read_thread_.join();
    }
    if (serial_port_.isOpen()) {
        serial_port_.close();
        ROS_INFO("Serial port disconnected.");
    }
}

bool SerialCommunicator::is_connected() const
{
    return serial_port_.isOpen();
}

bool SerialCommunicator::get_packet(std::vector<uint8_t>& buffer)
{
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (received_packets_queue_.empty()) {
        return false;
    }
    buffer = received_packets_queue_.front();
    received_packets_queue_.pop_front();
    return true;
}

bool SerialCommunicator::write_raw_frame(const std::vector<uint8_t>& frame)
{
    if (!serial_port_.isOpen()) {
        ROS_WARN("Write raw frame failed: port is not open.");
        return false;
    }
    
    std::lock_guard<std::mutex> lock(serial_write_mutex_);
    try {
        if (debug_mode_) {
           print_hex_frame("Sending Raw Frame: ", frame);
        }
        size_t bytes_written = serial_port_.write(frame);
        // Add time sleep to ensure the write operation completes
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        if (bytes_written != frame.size()) {
             ROS_WARN("Serial write timeout. Wrote %zu of %zu bytes.", bytes_written, frame.size());
             return false; // Indicate failure
        }
    } catch (const std::exception& e) {
        ROS_ERROR("Exception while writing raw frame to serial port %s: %s", port_name_.c_str(), e.what());
        disconnect(); // Disconnect on write error
        return false;
    }
    return true;
}

void SerialCommunicator::read_thread_loop()
{
    ROS_INFO("Starting robust (state machine) read thread for port %s.", port_name_.c_str());
    std::vector<uint8_t> frame_buffer;
    bool is_in_frame = false; // State machine variable

    while (is_running_ && ros::ok())
    {
        try
        {
            if (!serial_port_.isOpen()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                continue;
            }

            if (serial_port_.available() > 0)
            {
                uint8_t byte;
                size_t bytes_read = serial_port_.read(&byte, 1);

                if (bytes_read == 1)
                {
                    if (!is_in_frame) {
                        if (byte == FRAME_START_BYTE) {
                            frame_buffer.clear();
                            frame_buffer.push_back(byte);
                            is_in_frame = true;
                        }
                    } else { // is_in_frame == true
                        frame_buffer.push_back(byte);
                        if (frame_buffer.size() >= 3) { // Check if we have length byte
                           uint8_t payload_len = frame_buffer[2];
                           size_t expected_total_len = static_cast<size_t>(payload_len) + 5;
                           
                           if (frame_buffer.size() == expected_total_len) {
                               // We have a potential full frame.
                               if (frame_buffer.back() == FRAME_END_BYTE) {
                                   if (validate_checksum(frame_buffer, payload_len)) {
                                       std::vector<uint8_t> packet_to_queue;
                                       packet_to_queue.push_back(frame_buffer[1]);
                                       packet_to_queue.insert(packet_to_queue.end(),
                                                              frame_buffer.begin() + 3,
                                                              frame_buffer.begin() + 3 + payload_len);
                                       {
                                           std::lock_guard<std::mutex> lock(queue_mutex_);
                                           received_packets_queue_.push_back(packet_to_queue);
                                       }
                                   } else {
                                       if (debug_mode_) print_hex_frame("Recv BAD CHECKSUM: ", frame_buffer);
                                   }
                               } else {
                                   if (debug_mode_) print_hex_frame("Recv BAD END BYTE: ", frame_buffer);
                               }
                               // Frame processed (good or bad), reset for next one.
                               is_in_frame = false;
                               frame_buffer.clear();
                           } else if (frame_buffer.size() > expected_total_len || expected_total_len > 256) {
                               // Frame is longer than expected or length byte is absurd. It's corrupt.
                               if (debug_mode_) ROS_WARN("Frame length mismatch or corrupt. Discarding.");
                               is_in_frame = false;
                               frame_buffer.clear();
                           }
                        }
                    }
                }
            }
            else {
                // No data available, sleep briefly.
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
        catch (const std::exception& e) {
            ROS_ERROR("Unhandled exception in read thread loop: %s", e.what());
            is_in_frame = false;
            frame_buffer.clear();
            if(serial_port_.isOpen()) serial_port_.close();
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
    ROS_INFO("Read thread for port %s exiting.", port_name_.c_str());
}

bool SerialCommunicator::validate_checksum(const std::vector<uint8_t>& frame, uint8_t payload_len) const
{
    // The frame must have the exact expected length
    if (frame.size() != static_cast<size_t>(payload_len) + 5) return false;
    
    uint8_t calculated_checksum = sumElements(frame, 3, 3 + payload_len);
    uint8_t received_checksum = frame[frame.size() - 2];

    return received_checksum == calculated_checksum;
}

uint8_t SerialCommunicator::sumElements(const std::vector<uint8_t>& data, size_t from, size_t to) const
{
    uint32_t sum = 0;
    // Ensure 'to' does not go out of bounds
    size_t end = std::min(to, data.size());
    for (size_t i = from; i < end; ++i) {
        sum += data[i];
    }
    return sum % 2; // Assuming checksum is sum modulo 2
}

uint8_t SerialCommunicator::calculate_checksum(const std::vector<uint8_t>& frame_data) const
{
    if (frame_data.size() < 5) {
        return 0;
    }
    int sum = std::accumulate(frame_data.begin() + 3, frame_data.end() - 2, 0);
    return static_cast<uint8_t>(sum % 2);
}

void SerialCommunicator::print_hex_frame(const std::string& prefix, const std::vector<uint8_t>& data) const
{
    // if (!debug_mode_) {
    //     return;
    // }
    std::stringstream ss;
    ss << prefix << "[" << data.size() << " bytes]: ";
    ss << std::hex << std::uppercase << std::setfill('0');
    for (const auto& byte : data) {
        ss << std::setw(2) << static_cast<int>(byte) << " ";
    }
    ROS_INFO_STREAM(ss.str());
}