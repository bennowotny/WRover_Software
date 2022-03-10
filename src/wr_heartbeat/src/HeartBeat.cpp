#include "HeartBeat.hpp"
#include "ros/node_handle.h"
#include <memory>
#include <std_srvs/Empty.h>

HeartBeat::HeartBeat(const std::string &heartbeatName, const FrequencyCharacteristics &freq, ros::NodeHandle &n, bool runAsync) : freq{freq}, name{heartbeatName}, initialPulseSent{false}{
    this->heart = n.serviceClient<std_srvs::Empty>(heartbeatName, true);
    if(runAsync)
        this->autoBeat = n.createTimer(ros::Rate{freq.nominalFrequency}, &HeartBeat::autoPulse, this);
}

std::unique_ptr<HeartBeat> HeartBeat::makeHeartBeat(XmlRpc::XmlRpcValue &rosParam, ros::NodeHandle &n){

    if(rosParam.getType() != XmlRpc::XmlRpcValue::TypeStruct){
        ROS_FATAL("Heartbeat Monitor definition must be a ROSParam dictionary.");
        throw std::logic_error("Heartbeat has invalid format");
    }
    if(!rosParam.hasMember("heartbeatName") || rosParam["heartbeatName"].getType() != XmlRpc::XmlRpcValue::TypeString){
        ROS_FATAL("Parameter 'heartbeatName' must exist as a ROSParam of type string in a Heartbeat definition.");
        throw std::logic_error("Parameter 'heartbeatName' is invalid.");
    }
    std::string heartbeatName = rosParam["heartbeatName"];
    auto freq = FrequencyCharacteristics::makeFrequencyCharacteristics(rosParam);

    return std::move(std::unique_ptr<HeartBeat>(new HeartBeat{heartbeatName, freq, n}));
}

void HeartBeat::pulse(){
    std_srvs::Empty srv;
    this->heart.call(srv);
}

void HeartBeat::autoPulse(const ros::TimerEvent &event){
    this->pulse();

    if(!this->initialPulseSent){
        ROS_INFO_ONCE("Heartbeat '%s' pulsed, entering monitoring...", this->name.data());
        this->initialPulseSent = true;
        return;
    }

    // Self-check pulse rate
    auto const last_time = event.last_real;
    auto const timeSinceLastPub = (event.current_real - last_time).toSec();
    auto const adjDeviationFromExpected_Pulse = ((timeSinceLastPub - freq.getPeriod()) / freq.getPeriod());
    if (adjDeviationFromExpected_Pulse > freq.nominalTolerance)
        ROS_WARN("TIME WARNING:  %0.3f time since last pulse check on '%s', %0.3f expected (%0.3f%% deviation)", timeSinceLastPub, this->name.data(), freq.getPeriod(), 100*adjDeviationFromExpected_Pulse);
}