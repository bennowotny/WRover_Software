#include "SingleEncoderJointPositionMonitor.hpp"
#include "MathUtil.hpp"
#include "RoboclawChannel.hpp"
#include "ros/node_handle.h"

using std::literals::string_literals::operator""s;
using MathUtil::RADIANS_PER_ROTATION;

SingleEncoderJointPositionMonitor::SingleEncoderJointPositionMonitor(
    const std::string &controllerName,
    RoboclawChannel channel,
    EncoderConfiguration config,
    ros::NodeHandle node)
    : countsPerRotation{config.countsPerRotation},
      offset{config.offset},
      position{0},
      encoderSubscriber{
          std::make_shared<ros::Subscriber>(node.subscribe(
              "/hsi/roboclaw/"s + controllerName + "/enc/" + (channel == RoboclawChannel::A ? "left" : "right"),
              1,
              &SingleEncoderJointPositionMonitor::onEncoderReceived,
              this))} {}

// TODO: Question necessity
SingleEncoderJointPositionMonitor::SingleEncoderJointPositionMonitor(
    const SingleEncoderJointPositionMonitor &other)
    : countsPerRotation(other.countsPerRotation),
      offset(other.offset),
      encoderSubscriber{other.encoderSubscriber},
      position{other.position.load()} {}

// TODO: Question necessity
SingleEncoderJointPositionMonitor::SingleEncoderJointPositionMonitor(
    SingleEncoderJointPositionMonitor &&other) noexcept
    : countsPerRotation(other.countsPerRotation),
      offset(other.offset),
      encoderSubscriber{std::move(other.encoderSubscriber)},
      position{other.position.load()} {}

auto SingleEncoderJointPositionMonitor::operator()() -> double {
    return position;
}

void SingleEncoderJointPositionMonitor::onEncoderReceived(const std_msgs::UInt32::ConstPtr &msg) {
    // Encoder is on the interval [0, 2047] (integer)
    // Needs to be (-pi, pi] (floating point)
    auto enc = static_cast<double>(msg->data);
    auto rotations = MathUtil::corrMod(enc - offset, countsPerRotation) / countsPerRotation;
    position = MathUtil::corrMod(rotations * RADIANS_PER_ROTATION + M_PI, RADIANS_PER_ROTATION) - M_PI;
}
