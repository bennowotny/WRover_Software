#ifndef JOINT_H
#define JOINT_H

#include "ros/publisher.h"
#include "ros/subscriber.h"
#include "ros/timer.h"
#include "std_msgs/Float64.h"
#include <functional>
#include <string>

/**
 * @brief Represents a virtual joint in the MoveIt configuration
 *
 */
class Joint {
public:
    /**
     * @brief Construct a new Joint object
     *
     * @param name The name of the Joint, as configured in MoveIt
     * @param positionMonitor How this Joint reports its position (in radians)
     * @param motorSpeedDispatcher How this motor can set its speed (abstract floating speed [-1, 1])
     * @param node The ROS Node used to construct Publishers/Subscribers/Timers
     */
    explicit Joint(std::string name, std::function<double()> positionMonitor, std::function<void(double)> motorSpeedDispatcher, ros::NodeHandle node);

    /**
     * @brief Set the current position target of the joint and how much it should try to hold it
     *
     * @param target The target position of the joint, in radians
     * @param maxSpeed The maximum speed used to get to/hold the joint position
     */
    void setTarget(double target, double maxSpeed);

    /**
     * @brief Has this joint reached its target?
     *
     * @return true This joint is close to its target
     * @return false This joint is not close to its target
     */
    [[nodiscard]] auto hasReachedTarget() const -> bool;

    /**
     * @brief Get the name of the joint
     *
     * @return std::string The name of the joint
     */
    [[nodiscard]] auto getName() const -> std::string;

    /**
     * @brief Force a motor stop.  Disables position holding
     *
     */
    void stop();

private:
    /// How fast to report the current joint position
    static constexpr double FEEDBACK_UPDATE_FREQUENCY_HZ{50};
    /// How close do we have to be for 'done'-ness?
    static constexpr double JOINT_TOLERANCE_RADIANS{3 * M_PI / 180};

    /// Joint name
    const std::string name;
    /// How this joint gets its position
    const std::function<double()> positionMonitor;
    /// How this joint sets its speed
    const std::function<void(double)> motorSpeedDispatcher;

    /// ROS subscriber callback - on output from the PID loop
    void onControlLoopOutput(const std_msgs::Float64::ConstPtr &msg);
    /// ROS timer callback - updates the PID loop feedback and setpoint
    void onFeedbackUpdateEvent(const ros::TimerEvent &event);

    /// Current joint target
    std::atomic<double> target;
    /// Maximum joint speed
    std::atomic<double> maxSpeed;
    /// Can we execute motions now?
    std::atomic<bool> executeMotion;
    /// ROS Timer to manage control loop inputs
    ros::Timer controlLoopUpdateTimer;
    /// ROS Subscriber to get control loop output
    ros::Subscriber controlLoopOutputSubscriber;
    /// ROS Publisher to set control loop setpoint
    ros::Publisher controlLoopSetpointPublisher;
    /// ROS Publisher to set control loop feedback
    ros::Publisher controlLoopFeedbackPublisher;
};

#endif