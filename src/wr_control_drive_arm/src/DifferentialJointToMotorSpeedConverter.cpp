#include "DifferentialJointToMotorSpeedConverter.hpp"
#include <mutex>

DifferentialJointToMotorSpeedConverter::DifferentialJointToMotorSpeedConverter(
    std::shared_ptr<Motor> leftMotor,
    std::shared_ptr<Motor> rightMotor)
    : cachedPitchSpeed{0},
      cachedRollSpeed{0},
      leftMotor{std::move(leftMotor)},
      rightMotor{std::move(rightMotor)} {}

// TODO: Question necessity
DifferentialJointToMotorSpeedConverter::DifferentialJointToMotorSpeedConverter(
    const DifferentialJointToMotorSpeedConverter &other)
    : cachedPitchSpeed{other.cachedPitchSpeed.load()},
      cachedRollSpeed{other.cachedRollSpeed.load()},
      leftMotor{other.leftMotor},
      rightMotor{other.rightMotor} {}

// TODO: Question necessity
DifferentialJointToMotorSpeedConverter::DifferentialJointToMotorSpeedConverter(
    DifferentialJointToMotorSpeedConverter &&other) noexcept
    : cachedPitchSpeed{other.cachedPitchSpeed.load()},
      cachedRollSpeed{other.cachedRollSpeed.load()},
      leftMotor{other.leftMotor},
      rightMotor{other.rightMotor} {}

void DifferentialJointToMotorSpeedConverter::setPitchSpeed(double speed) {
    cachedPitchSpeed = speed;
    dispatchDifferentialSpeed();
}

void DifferentialJointToMotorSpeedConverter::setRollSpeed(double speed) {
    cachedRollSpeed = speed;
    dispatchDifferentialSpeed();
}

void DifferentialJointToMotorSpeedConverter::dispatchDifferentialSpeed() {
    // TODO: Re-test guard effectiveness in MT environment
    // const std::lock_guard<std::recursive_mutex> guard{mutex};
    /*
     * This linear transformation is tuned based on the physical configuration of the arm.
     * Test this in testDifferential.cpp
     */
    auto m1Speed{-cachedPitchSpeed + (cachedRollSpeed * AVERAGE_SCALING_FACTOR)};
    auto m2Speed{-cachedPitchSpeed - (cachedRollSpeed * AVERAGE_SCALING_FACTOR)};
    leftMotor->setSpeed(m1Speed);
    rightMotor->setSpeed(m2Speed);
    // std::cout << "ptch: " << cachedPitchSpeed.load() << "\troll: " << cachedRollSpeed.load() << "\tlspd: " << m1Speed << "\trspd: " << m2Speed << std::endl;
}