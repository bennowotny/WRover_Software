#include "HeartMonitor.hpp"
#include "std_srvs/Empty.h"
#include <string>

HeartMonitor::HeartMonitor(const std::string &monitorName, FrequencyCharacteristics &freq, ros::NodeHandle &n) : freq{freq}, name{monitorName}, initialHeartbeat{false}, monitorInitialized{false}{
    this->monitor = n.advertiseService(monitorName, &HeartMonitor::registerHeartBeat, this);
    this->lastHeartbeat = ros::Time::now();
    ros::Rate test{freq.nominalFrequency};
    this->heartbeatChecker = n.createTimer(test, &HeartMonitor::checkHeartbeat, this);
}

std::unique_ptr<HeartMonitor> HeartMonitor::makeHeartMonitor(XmlRpc::XmlRpcValue &monitorRosParam, ros::NodeHandle &n) {

    if(monitorRosParam.getType() != XmlRpc::XmlRpcValue::TypeStruct){
        ROS_FATAL("Heartbeat Monitor definition must be a ROSParam dictionary.");
        throw std::logic_error("Heartbeat Monitor has invalid format");
    }
    if(!monitorRosParam.hasMember("monitorName") || monitorRosParam["monitorName"].getType() != XmlRpc::XmlRpcValue::TypeString){
        ROS_FATAL("Parameter 'monitorName' must exist as a ROSParam of type string in a Monitor definition.");
        throw std::logic_error("Parameter 'monitorName' is invalid.");
    }
    std::string monitorName = monitorRosParam["monitorName"];
    auto freq = FrequencyCharacteristics::makeFrequencyCharacteristics(monitorRosParam);

    return std::move(std::unique_ptr<HeartMonitor>(new HeartMonitor{monitorName, freq, n}));
}

bool HeartMonitor::registerHeartBeat(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    this->lastHeartbeat = ros::Time::now();
    this->initialHeartbeat = true;
    ROS_INFO_ONCE("Heartbeat '%s' heard, entering monitoring...", this->name.data());
    return true;
}

void HeartMonitor::checkHeartbeat(const ros::TimerEvent& event){

    // Check for Heartbeat
    if(this->initialHeartbeat){
        auto const elapsedTime = (event.current_real - this->lastHeartbeat).toSec();
        auto const adjDeviationFromExpected_Heartbeat = ((elapsedTime - freq.getPeriod()) / freq.getPeriod());
        if (adjDeviationFromExpected_Heartbeat > freq.nominalTolerance)
            ROS_ERROR("TIME FAULT:  %0.3f time since last heartbeat on '%s', %0.3f expected (%0.3f%% deviation)", elapsedTime, this->name.data(), freq.getPeriod(), 100*adjDeviationFromExpected_Heartbeat);
    } else return;

    if(!this->monitorInitialized){
        this->monitorInitialized = true;
        return;
    }
    // Self-check monitor rate
    auto const last_time = event.last_real;
    auto const timeSinceLastCheck = (event.current_real - last_time).toSec();
    auto const adjDeviationFromExpected_Monitor = ((timeSinceLastCheck - freq.getPeriod()) / freq.getPeriod());
    if (adjDeviationFromExpected_Monitor > freq.nominalTolerance)
        ROS_WARN("TIME WARNING:  %0.3f time since last monitor check on '%s', %0.3f expected (%0.3f%% deviation)", timeSinceLastCheck, this->name.data(), freq.getPeriod(), 100*adjDeviationFromExpected_Monitor);
}