#ifndef DIRECT_JOINT_TO_MOTOR_SPEED_CONVERTER_H
#define DIRECT_JOINT_TO_MOTOR_SPEED_CONVERTER_H

#include "Motor.hpp"
#include <memory>

/**
 * @brief Declare set values for the direction of the joint under positive motor speed
 *
 */
enum class MotorSpeedDirection {
    FORWARD,
    REVERSE
};

/**
 * @brief Handle speed conversions for simple joints (direct-drive)
 *
 */
class DirectJointToMotorSpeedConverter {
public:
    /**
     * @brief Construct a new Direct Joint To Motor Speed Converter object
     *
     * @param outputMotor The motor associated with this joint
     * @param direction The direction of the virtual joint under positive motor power
     */
    DirectJointToMotorSpeedConverter(std::shared_ptr<Motor> outputMotor, MotorSpeedDirection direction);

    /**
     * @brief Dispatches the motor speed based on reversing status
     * TODO: Refactor to not operator overload
     *
     * @param speed The speed of the motor, a double on the interval [-1, 1] with 0 as the stop value
     */
    void operator()(double speed);

    // Rule of 5
    // TODO: Question necessity
    DirectJointToMotorSpeedConverter(const DirectJointToMotorSpeedConverter &) = default;
    auto operator=(const DirectJointToMotorSpeedConverter &) -> DirectJointToMotorSpeedConverter & = delete;
    DirectJointToMotorSpeedConverter(DirectJointToMotorSpeedConverter &&) = default;
    auto operator=(DirectJointToMotorSpeedConverter &&) -> DirectJointToMotorSpeedConverter & = delete;
    ~DirectJointToMotorSpeedConverter() = default;

private:
    /// Direction of the motor
    const MotorSpeedDirection direction;
    /// Motor to dispatch speeds to
    const std::shared_ptr<Motor> outputMotor;
};

#endif