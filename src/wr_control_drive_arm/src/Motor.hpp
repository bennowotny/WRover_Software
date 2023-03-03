#ifndef MOTOR_H
#define MOTOR_H

#include "RoboclawChannel.hpp"
#include "ros/publisher.h"
#include <string>

class Motor {
public:
    /**
     * @brief Construct a new Motor object, assuming WRoboclaw control
     *
     * @param controllerName The friendly name of the controller
     * @param channel The channel id of the motor on the WRoboclaw controller
     * @param node ROS NodeHandle to construct the speed Publisher
     */
    Motor(const std::string &controllerName, RoboclawChannel channel, ros::NodeHandle node);
    /**
     * @brief Set the speed of the motor
     *
     * @param speed Speed of the motor
     */
    void setSpeed(double speed);
    /**
     * @brief Is this motor experiencing an overcurrent fault
     * TODO: Implement
     *
     * @return true The motor is in an overcurrent event
     * @return false The motor is not in an overcurrent event
     */
    auto isOverCurrent() -> bool;

private:
    /// ROS Publisher for motor speeds
    ros::Publisher motorSpeedPublisher;
};

#endif