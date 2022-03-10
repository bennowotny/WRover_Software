#ifndef HEARTBEAT_HEADER_GUARD
#define HEARTBEAT_HEADER_GUARD

#include <ros/ros.h>
#include "FrequencyCharacteristics.hpp"
#include "XmlRpcValue.h"
#include "ros/node_handle.h"
#include <optional>

class HeartBeat{
private:
    std::string name;
    ros::ServiceClient heart;
    FrequencyCharacteristics freq;
    std::optional<ros::Timer> autoBeat;
    std::atomic_bool initialPulseSent;
    HeartBeat(const std::string &heartbeatName, const FrequencyCharacteristics &freq, ros::NodeHandle &n, bool runAsync = true);
    void autoPulse(const ros::TimerEvent &event);
public:
    void pulse();
    static std::unique_ptr<HeartBeat> makeHeartBeat(XmlRpc::XmlRpcValue &rosParam, ros::NodeHandle &n);
};
#endif