/**
 * @file ArmControlActionServer.cpp
 * @author Ben Nowotny
 * @brief The exeutable file to run the Arm Control Action Server
 * @date 2022-12-05
 */
#include "DifferentialJointToMotorSpeedConverter.hpp"
#include "DirectJointToMotorSpeedConverter.hpp"
#include "Joint.hpp"
#include "Motor.hpp"
#include "RoboclawChannel.hpp"
#include "SingleEncoderJointPositionMonitor.hpp"
#include "XmlRpcValue.h"
#include "ros/init.h"

#include <actionlib/server/simple_action_server.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <csignal>
#include <memory>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Trigger.h>
#include <string>
#include <thread>
#include <unordered_map>

using XmlRpc::XmlRpcValue;

/**
 * @brief Refresh rate of ros::Rate
 */
constexpr float CLOCK_RATE{50};

/**
 * @brief Max rate to produce warning messages
 *
 */
constexpr double IK_WARN_RATE{1.0 / 2};

/**
 * @brief Max joint speed during motion
 *
 */
constexpr double JOINT_SAFETY_MAX_SPEED{0.5};

/**
 * @brief Max joint speed to hold position
 *
 */
constexpr double JOINT_SAFETY_HOLD_SPEED{0.3};

/**
 * @brief Nessage cache size of publisher
 */
constexpr std::uint32_t MESSAGE_CACHE_SIZE{10};

/**
 * @brief Period between timer callback
 */
constexpr float TIMER_CALLBACK_DURATION{1.0 / 50.0};

/**
 * @brief Defines space for all Joint references
 */
std::unordered_map<std::string, std::unique_ptr<Joint>> namedJointMap;

/**
 * @brief Simplify the SimpleActionServer reference name
 */
using Server = actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>;
/**
 * @brief The service server for enabling IK
 */
ros::ServiceServer enableServiceServer;
/**
 * @brief The status of IK program
 */
std::atomic_bool IKEnabled{true};
/**
 * @brief The service client for disabling IK
 */
ros::ServiceClient enableServiceClient;

/**
 * @brief Perform the given action as interpreted as moving the arm joints to
 * specified positions
 *
 * @param goal The goal state given
 * @param as The Action Server this is occuring on
 */
void execute(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal,
             Server *server) {
    // Kill action if IK is currently disabled
    if (!IKEnabled) {
        server->setAborted();
        ROS_WARN_THROTTLE( // NOLINT(hicpp-no-array-decay,hicpp,hicpp-vararg,cppcoreguidelines-pro-bounds-array-to-pointer-decay, cppcoreguidelines-pro-type-vararg)
            IK_WARN_RATE,
            "IK is disabled");
        return;
    }

    // DEBUG: Print the target waypoint table for the current motion
    for (const auto &jointName : goal->trajectory.joint_names) {
        std::cout << jointName << "\t";
    }
    std::cout << std::endl;
    for (const auto &waypoint : goal->trajectory.points) {
        for (const auto &jointVal : waypoint.positions) {
            std::cout << jointVal << "\t";
        }
        std::cout << std::endl;
    }

    // For each waypoint in the current motion
    for (const auto &currTargetPosition : goal->trajectory.points) {

        // Scale the joint power based on MoveIt estimation
        const double VELOCITY_MAX = abs(*std::max_element(
            currTargetPosition.velocities.begin(),
            currTargetPosition.velocities.end(),
            [](double lhs, double rhs) -> bool { return abs(lhs) < abs(rhs); }));

        // Give each waypoint it's target/max speed for the current waypoint
        for (uint32_t i = 0; i < goal->trajectory.joint_names.size(); ++i) {
            auto jointVelocity{JOINT_SAFETY_HOLD_SPEED};
            if (VELOCITY_MAX != 0)
                jointVelocity = currTargetPosition.velocities.at(i) / VELOCITY_MAX * JOINT_SAFETY_MAX_SPEED;

            namedJointMap.at(goal->trajectory.joint_names.at(i))->setTarget(currTargetPosition.positions.at(i), jointVelocity);
        }
    }

    auto waypointComplete{false};
    ros::Rate updateRate{CLOCK_RATE};

    /**
     * While...
     * 1. The waypoint is not yet complete (as reported by the joints)
     * 2. ROS has not died
     * 3. There is not a more recent target available
     */
    while (!waypointComplete && ros::ok() && !server->isNewGoalAvailable()) {
        // Determine if the waypoint is complete
        waypointComplete = true;
        for (const auto &[_, joint] : namedJointMap) {
            waypointComplete &= joint->hasReachedTarget();
            if (!joint->hasReachedTarget())
                std::cout << "Waiting on joint " << _ << std::endl;
        }
        // Throttle CPU usage
        updateRate.sleep();
    }

    // Report preemption if it occurred
    if (server->isNewGoalAvailable())
        server->setPreempted();
    // When all positions have been reached, set the current task as succeeded
    else
        server->setSucceeded();
    std::cout << "Action Complete!" << std::endl;
}

/**
 * @brief Get the encoder configuration from ROS config files
 *
 * @param params ROS parameter dictionary
 * @param jointName The joint to query
 * @return EncoderConfiguration The configuration of the encoder for this joint
 */
auto getEncoderConfigFromParams(const XmlRpcValue &params, const std::string &jointName) -> EncoderConfiguration {
    return {.countsPerRotation = static_cast<int32_t>(params[jointName]["counts_per_rotation"]),
            .offset = static_cast<int32_t>(params[jointName]["offset"])};
}

/**
 * @brief The main executable method of the node.  Starts the ROS node and the
 * Action Server for processing Arm Control commands
 *
 * @param argc The number of program arguments
 * @param argv The given program arguments
 * @return int The status code on exiting the program
 */
auto main(int argc, char **argv) -> int {
    std::cout << "start main" << std::endl;
    // Initialize the current node as ArmControlSystem
    ros::init(argc, argv, "ArmControlActionServer");
    // Create the NodeHandle to the current ROS node
    ros::NodeHandle n;
    // Create a private NodeHandle for the ROS parameters in config files
    ros::NodeHandle pn{"~"};

    // Get the encoder parameter dictionary to query later
    XmlRpcValue encParams;
    pn.getParam("encoder_parameters", encParams);

    // Initialize all motors with their MoveIt name, WRoboclaw initialization,
    // and reference to the current node
    std::cout << "init motors" << std::endl;

    using std::literals::string_literals::operator""s;

    const auto turntableMotor{std::make_shared<Motor>("aux0"s, RoboclawChannel::A, n)};
    const auto shoulderMotor{std::make_shared<Motor>("aux0"s, RoboclawChannel::B, n)};
    const auto elbowMotor{std::make_shared<Motor>("aux1"s, RoboclawChannel::A, n)};
    const auto forearmRollMotor{std::make_shared<Motor>("aux1"s, RoboclawChannel::B, n)};
    const auto wristLeftMotor{std::make_shared<Motor>("aux2"s, RoboclawChannel::A, n)};
    const auto wristRightMotor{std::make_shared<Motor>("aux2"s, RoboclawChannel::B, n)};

    // Create position monitors for the joints
    // Made as an object for isolation to reflect potential changes to mechanical design
    const auto turntablePositionMonitor{std::make_shared<SingleEncoderJointPositionMonitor>(
        "aux0"s,
        RoboclawChannel::A,
        getEncoderConfigFromParams(encParams, "turntable"),
        n)};
    const auto shoulderPositionMonitor{std::make_shared<SingleEncoderJointPositionMonitor>(
        "aux0"s,
        RoboclawChannel::B,
        getEncoderConfigFromParams(encParams, "shoulder"),
        n)};
    const auto elbowPositionMonitor{std::make_shared<SingleEncoderJointPositionMonitor>(
        "aux1"s,
        RoboclawChannel::A,
        getEncoderConfigFromParams(encParams, "elbow"),
        n)};
    const auto forearmRollPositionMonitor{std::make_shared<SingleEncoderJointPositionMonitor>(
        "aux1"s,
        RoboclawChannel::B,
        getEncoderConfigFromParams(encParams, "forearmRoll"),
        n)};
    const auto wristPitchPositionMonitor{std::make_shared<SingleEncoderJointPositionMonitor>(
        "aux2"s,
        RoboclawChannel::A,
        getEncoderConfigFromParams(encParams, "wristPitch"),
        n)};
    const auto wristRollPositionMonitor{std::make_shared<SingleEncoderJointPositionMonitor>(
        "aux2"s,
        RoboclawChannel::B,
        getEncoderConfigFromParams(encParams, "wristRoll"),
        n)};

    // Create the joint-to-motor speed converters for each joint (set)
    const auto turntableSpeedConverter{std::make_shared<DirectJointToMotorSpeedConverter>(turntableMotor, MotorSpeedDirection::REVERSE)};
    const auto shoulderSpeedConverter{std::make_shared<DirectJointToMotorSpeedConverter>(shoulderMotor, MotorSpeedDirection::REVERSE)};
    const auto elbowSpeedConverter{std::make_shared<DirectJointToMotorSpeedConverter>(elbowMotor, MotorSpeedDirection::REVERSE)};
    const auto forearmRollSpeedConverter{std::make_shared<DirectJointToMotorSpeedConverter>(forearmRollMotor, MotorSpeedDirection::REVERSE)};
    const auto differentialSpeedConverter{std::make_shared<DifferentialJointToMotorSpeedConverter>(wristLeftMotor, wristRightMotor)};

    // Initialize all Joints

    std::cout << "init joints" << std::endl;

    // Create the virtual joint
    // Due to physical configuration, each joint is given (via parameter) its:
    // * Method to measure position
    // * Method to dispatch motor speeds
    namedJointMap.insert({"turntable_joint", std::make_unique<Joint>(
                                                 "turntable"s,
                                                 [turntablePositionMonitor]() -> double { return (*turntablePositionMonitor)(); },
                                                 [turntableSpeedConverter](double speed) { (*turntableSpeedConverter)(speed); },
                                                 n)});
    namedJointMap.insert({"shoulder_joint", std::make_unique<Joint>(
                                                "shoulder",
                                                [shoulderPositionMonitor]() -> double { return (*shoulderPositionMonitor)(); },
                                                [shoulderSpeedConverter](double speed) { (*shoulderSpeedConverter)(speed); },
                                                n)});
    namedJointMap.insert({"elbowPitch_joint", std::make_unique<Joint>(
                                                  "elbow",
                                                  [elbowPositionMonitor]() -> double { return (*elbowPositionMonitor)(); },
                                                  [elbowSpeedConverter](double speed) { (*elbowSpeedConverter)(speed); },
                                                  n)});
    namedJointMap.insert({"elbowRoll_joint", std::make_unique<Joint>(
                                                 "forearmRoll",
                                                 [forearmRollPositionMonitor]() -> double { return (*forearmRollPositionMonitor)(); },
                                                 [forearmRollSpeedConverter](double speed) { (*forearmRollSpeedConverter)(speed); },
                                                 n)});
    namedJointMap.insert({"wristPitch_joint", std::make_unique<Joint>(
                                                  "wristPitch",
                                                  [wristPitchPositionMonitor]() -> double { return (*wristPitchPositionMonitor)(); },
                                                  [converter = differentialSpeedConverter](double speed) { converter->setPitchSpeed(speed); },
                                                  n)});
    namedJointMap.insert({"wristRoll_link", std::make_unique<Joint>(
                                                "wristRoll",
                                                [wristRollPositionMonitor]() -> double { return (*wristRollPositionMonitor)(); },
                                                [converter = differentialSpeedConverter](double speed) { converter->setRollSpeed(speed); },
                                                n)});

    // Initialize the Action Server
    Server server(
        n, "/arm_controller/follow_joint_trajectory",
        [&server](auto goal) { execute(goal, &server); }, false);
    // Start the Action Server
    server.start();
    std::cout << "server started" << std::endl;

    // TODO: Create arbitration service
    enableServiceServer = n.advertiseService(
        "start_IK",
        static_cast<boost::function<bool(std_srvs::Trigger::Request &,
                                         std_srvs::Trigger::Response &)>>(
            [](std_srvs::Trigger::Request &req,
               std_srvs::Trigger::Response &res) -> bool {
                IKEnabled = true;
                res.message = "Arm IK Enabled";
                res.success = static_cast<unsigned char>(true);
                return true;
            }));

    // TODO: Create arbitration service
    enableServiceClient =
        n.serviceClient<std_srvs::Trigger>("PLACEHOLDER_NAME");

    std::cout << "entering ROS spin..." << std::endl;
    // ROS spin for communication with other nodes
    ros::spin();
    // Return 0 on exit (successful exit)
    return 0;
}
