#ifndef DIFFERENTIAL_JOINT_TO_MOTOR_SPEED_CONVERTER_H
#define DIFFERENTIAL_JOINT_TO_MOTOR_SPEED_CONVERTER_H

#include "Motor.hpp"

/**
 * @brief Converts pitch/roll motor speeds to differential motor speeds
 *
 */
class DifferentialJointToMotorSpeedConverter {
public:
    /**
     * @brief Construct a new Differential Joint To Motor Speed Converter object
     * The 'left'-ness or 'right'-ness of any motor is slightly arbitrary, but it must be reflected in the linear transformation on speed dispatch
     *
     * @param leftMotor The left motor of the differential drive
     * @param rightMotor The right motor of the differential drive
     */
    DifferentialJointToMotorSpeedConverter(std::shared_ptr<Motor> leftMotor, std::shared_ptr<Motor> rightMotor);

    /**
     * @brief Set the pitch speed of the differential joint
     *
     * @param speed The pitch speed of the joint
     */
    void setPitchSpeed(double speed);
    /**
     * @brief Set the roll speed of the differential joint
     *
     * @param speed The rolls speed of the joint
     */
    void setRollSpeed(double speed);

    // Rule of 5 Definitions
    // TODO: Evaluate purpose of these
    DifferentialJointToMotorSpeedConverter(const DifferentialJointToMotorSpeedConverter &);
    auto operator=(const DifferentialJointToMotorSpeedConverter &) -> DifferentialJointToMotorSpeedConverter & = delete;
    DifferentialJointToMotorSpeedConverter(DifferentialJointToMotorSpeedConverter &&) noexcept;
    auto operator=(DifferentialJointToMotorSpeedConverter &&) -> DifferentialJointToMotorSpeedConverter & = delete;
    ~DifferentialJointToMotorSpeedConverter() = default;

private:
    /**
     * @brief Stored pitch speed
     *
     */
    std::atomic<double> cachedPitchSpeed;

    /**
     * @brief Stored roll speed
     *
     */
    std::atomic<double> cachedRollSpeed;

    /**
     * @brief THREADING: required for potential ROS asynchronisity
     *
     */
    std::recursive_mutex mutex;

    /// Left Motor of the differential drive
    const std::shared_ptr<Motor> leftMotor;
    /// Right Motor of the differential drive
    const std::shared_ptr<Motor> rightMotor;

    /// Constant used in tuning linear transformation
    // TODO: Replace with gear ratio speed adjustments
    static constexpr double AVERAGE_SCALING_FACTOR{1};

    /**
     * @brief Use combined pitch and roll data to dispatch motor speeds.
     * This is aggregated to a single methods to let the motors 'agree' on speeds to accomplish both joint goals.
     *
     */
    void dispatchDifferentialSpeed();
};

#endif