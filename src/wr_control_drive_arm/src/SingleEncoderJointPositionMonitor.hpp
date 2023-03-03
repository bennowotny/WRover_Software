#ifndef SINGLE_ENCODER_JOINT_POSITION_MONITOR_H
#define SINGLE_ENCODER_JOINT_POSITION_MONITOR_H

#include "RoboclawChannel.hpp"
#include <cstdint>
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <std_msgs/UInt32.h>
#include <string>

/**
 * @brief A struct to contain the encoder configuration parameters
 *
 */
struct EncoderConfiguration {
    int32_t countsPerRotation;
    int32_t offset;
};

/**
 * @brief How a simple, direct-read joint reports its positioned
 *
 */
class SingleEncoderJointPositionMonitor {
public:
    /**
     * @brief Construct a new Single Encoder Joint Position Monitor object
     *
     * @param controllerName The name of the motor controller with the encoder
     * @param channel The Roboclaw Channel of the encoder
     * @param config Encoder configuration parameters
     * @param node ROS NodeHandle to create Subscribers
     */
    SingleEncoderJointPositionMonitor(const std::string &controllerName, RoboclawChannel channel, EncoderConfiguration config, ros::NodeHandle node);
    /**
     * @brief Reports joint position
     *
     * @return double The joint position, in radians
     */
    auto operator()() -> double;

    // Rule of 5
    // TODO: Question necessity
    SingleEncoderJointPositionMonitor(const SingleEncoderJointPositionMonitor &);
    auto operator=(const SingleEncoderJointPositionMonitor &) -> SingleEncoderJointPositionMonitor & = delete;
    SingleEncoderJointPositionMonitor(SingleEncoderJointPositionMonitor &&) noexcept;
    auto operator=(SingleEncoderJointPositionMonitor &&) -> SingleEncoderJointPositionMonitor & = delete;
    ~SingleEncoderJointPositionMonitor() = default;

private:
    /// ROS subscriber callback - stores encoder readings
    void onEncoderReceived(const std_msgs::UInt32::ConstPtr &msg);

    /// The current joint position
    std::atomic<double> position;
    /// ROS subscriber to read the encoder
    std::shared_ptr<ros::Subscriber> encoderSubscriber;
    /// The number of encoder counts per joint rotation
    const int32_t countsPerRotation;
    /// The offset of the encoder with respect to the joint
    const int32_t offset;
};

#endif