#include "DifferentialJointToMotorSpeedConverter.hpp"
#include "Motor.hpp"
#include "ros/init.h"
#include <cstdint>
#include <ros/ros.h>

/**
 * @brief Main entry point, used to test the differential joint to speed converter
 *
 * @param argc Command line arguments count, unused
 * @param argv Command line arguments, unused
 * @return int32_t Program return code
 */
auto main(int32_t argc, char **argv) -> int32_t {
    ros::init(argc, argv, "testNode");

    // ROS NodeHandle for network connectivity
    ros::NodeHandle n{};
    using std::literals::string_literals::operator""s;

    // Create left/right differential motors
    const auto wristLeftMotor{std::make_shared<Motor>("aux2"s, RoboclawChannel::A, n)};
    const auto wristRightMotor{std::make_shared<Motor>("aux2"s, RoboclawChannel::B, n)};

    // Create speed dispatcher for differential joint
    const auto differentialSpeedConverter{std::make_shared<DifferentialJointToMotorSpeedConverter>(wristLeftMotor, wristRightMotor)};

    // Run at 50 Hz
    ros::Rate loopRate{50};
    while (ros::ok()) {
        // Edit the test here: test different speeds in different configurations and observe on hardware to get best results
        differentialSpeedConverter->setPitchSpeed(0.3);
        differentialSpeedConverter->setRollSpeed(0);
        loopRate.sleep();
    }

    return 0;
}